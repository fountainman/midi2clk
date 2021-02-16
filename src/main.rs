#![no_std]
#![no_main]

mod ppq;
mod display;
use ppq::Ppq;
use display::update_display;

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
//use panic_semihosting as _; // panic handler

use rtic::cyccnt::{Instant, U32Ext};

use stm32f1xx_hal::{
    prelude::*,
    serial,
    gpio::{
        gpioc::{PC13, PC14},
        gpiob::{PB8, PB9, PB6, PB7},
        //gpioa::{PA9, PA10},
        {Output, PushPull},
        {Input, PullUp},
        {Alternate, OpenDrain},
    },
    timer::{Event, Timer},
    pac::{I2C1, USART1},
    usb::{UsbBus, UsbBusType, Peripheral},
    i2c::{BlockingI2c, DutyCycle, Mode},
};
use cortex_m::asm::delay;
use usb_device::{
    bus::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_midi::{
    data::{
        usb::constants::USB_AUDIO_CLASS,
        usb::constants::USB_MIDISTREAMING_SUBCLASS,
    },
    midi_device::MidiClass,
};

//use core::fmt::Write;
use ssd1306::{
    prelude::*,
    Builder,
    I2CDIBuilder,
};

// Import peripheral control methods from general HAL definition
use embedded_hal::digital::v2::{OutputPin, InputPin};
use core::ptr::write_volatile;

use core::fmt::Write;

const TIMEOUT: u32 = 10_000_000;

// Setup the app. We're using the hal Peripheral Access Crate for peripherals
#[rtic::app(device = stm32f1xx_hal::pac, monotonic = rtic::cyccnt::CYCCNT,
peripherals = true)]
// RTIC application
const APP: () = {
    // Resources shared by all handlers
    struct Resources {
        // This is not initialized here, so must be initialized in init and passed through LateResources
        led: PC13<Output<PushPull>>,
        midi: MidiClass<'static, UsbBusType>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        display: GraphicsMode<I2CInterface<BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>,PB9<Alternate<OpenDrain>>) >>, DisplaySize128x64>,
        ppq: Ppq,
        beat_clock: PC14<Output<PushPull>>,
        buttons: (PB7<Input<PullUp>>, PB6<Input<PullUp>>),
        #[init(false)]
        button_pressed: bool,
        EXTI: stm32f1xx_hal::pac::EXTI,
        clocks: stm32f1xx_hal::rcc::Clocks,
        serial: (serial::Tx<USART1>, serial::Rx<USART1>),
        #[init(0)]
        counter: u8,
        last_tick: Instant,
    }

    // Init function (duh)
    #[init (schedule = [check_timeout])]
    // CX object contains our PAC. LateResources
    fn init(cx: init::Context) -> init::LateResources{
        // Configure external interrupts (Used for buttons on BP6 and PB7)
        // Enable AFIO clock
        cx.device.RCC.apb2enr.write(|w| w.afioen().enabled());
        // Enable EXTI6 and EXTI7 interrupts
        cx.device.EXTI.imr.modify(|_,w| w.mr6().set_bit());
        cx.device.EXTI.imr.modify(|_,w| w.mr7().set_bit());
        // Set falling trigger
        cx.device.EXTI.ftsr.modify(|_,w| w.tr6().set_bit());
        cx.device.EXTI.ftsr.modify(|_,w| w.tr7().set_bit());
        // Set rising trigger
        cx.device.EXTI.rtsr.modify(|_,w| w.tr6().set_bit());
        cx.device.EXTI.rtsr.modify(|_,w| w.tr7().set_bit());

        // Enable cycle counter
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();
        // Take ownership of clock register
        let mut rcc = cx.device.RCC.constrain();
        // Take ownership of flash peripheral
        let mut flash = cx.device.FLASH.constrain();
        // Take ownership of AFIO register
        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        // Fuck my life
        // Attach EXTI7 and EXTI6 to PortB (IE, enable interrupts on PB7 and PB6)
        unsafe {
            // Get address of AFIO register block
            let afio_ptr: *mut u32 = stm32f1xx_hal::pac::AFIO::ptr() as *mut u32;
            // Offset by three registers (see datasheet) to get exticr2
            let exticr2_ptr: *mut u32 = afio_ptr.offset(3);
            // 16b..(N/A, 16 bit reg), 4b0001 (set EXTI7 to PB), 4b0001 (set EXTI6 to PB), 8b0..(don't care)
            write_volatile(exticr2_ptr, 0b00000000000000000001000100000000);
        }

        // Configure clocks and make clock object from clock register
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);

        // Make sure clocks work with usb
        assert!(clocks.usbclk_valid());

        // -----
        // Timer
        // -----
        let mut timer = Timer::tim2(cx.device.TIM2, &clocks, &mut rcc.apb1);
        timer.start_count_down(100.hz()).listen(Event::Update);

        // Split GPIO ports into smaller pin objects
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        // Configure LED and clock output
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        let mut beat_clock = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
        led.set_high().unwrap();
        beat_clock.set_low().unwrap();

        // Configure button inputs
        let button_next = gpiob.pb7.into_pull_up_input(&mut gpiob.crl);
        let button_prev = gpiob.pb6.into_pull_up_input(&mut gpiob.crl);

        // -----------
        // Init serial
        // -----------
        let tx1_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx1_pin = gpioa.pa10.into_floating_input(&mut gpioa.crh);
        let cfg = serial::Config::default().baudrate(115_200.bps());
        let usart1 = serial::Serial::usart1(
            cx.device.USART1,
            (tx1_pin, rx1_pin),
            &mut afio.mapr,
            cfg,
            clocks,
            &mut rcc.apb2,
        );
        let (tx, rx) = usart1.split();

        // ----------------
        // Init I2C display
        // ----------------
        // Init IO pins
        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        // Init i2c peripheral
        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        // Init i2c interface
        let interface = I2CDIBuilder::new().init(i2c);
        // Create display in graphics mode
        let mut display: GraphicsMode<_, _> = Builder::new().connect(interface).into();
        // Init display
        display.init().unwrap();
        display.clear();

        let ppq = Ppq::Ppq24;

        // Schedule the display to be updated with initial value
        update_display(&mut display, ppq.to_str());

        // ----------
        // USB CONFIG
        // ----------

        // Configure USB pins and reset bus
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().unwrap();
        delay(clocks.sysclk().0 / 100);

        // Create USB peripheral object
        let usb: Peripheral = Peripheral {
            usb: cx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        };

        // Make empty static UsbBusAllocator. This must be static because it goes out of scope
        // at the end of init but still has references taken out inside usb_dev and midi
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;
        // Define USB_BUS declared earlier (requires unsafe)
        unsafe{ USB_BUS = Some(UsbBus::new(usb))};
        // Create midi endpoints (unwrap necessary to get UsbBusAllocator out of Option)
        let midi = MidiClass::new(unsafe{USB_BUS.as_ref().unwrap()});
        // Set up USB device. No new endpoints can be made after this.
        let usb_dev = UsbDeviceBuilder::new(unsafe{USB_BUS.as_ref().unwrap()}, UsbVidPid(0x16c0, 0x5e4))
            .manufacturer("liakhova")
            .product("MIDI to CC Converter")
            .serial_number("1")
            .device_class(USB_AUDIO_CLASS)
            .device_sub_class(USB_MIDISTREAMING_SUBCLASS)
            .build();

        // Schedule timeout checker
        let last_tick: Instant = cx.start;
        cx.schedule.check_timeout(cx.start + TIMEOUT.cycles()).unwrap();

        // Return late resources to resources object
        init::LateResources {
            led,
            usb_dev,
            midi,
            display,
            ppq,
            buttons: (button_next, button_prev),
            beat_clock,
            EXTI: cx.device.EXTI,
            clocks,
            serial: (tx, rx),
            last_tick,
        }
    }

    // Idle function run when no tasks are running
    #[idle(resources = [])]
    fn idle(mut _cx: idle::Context) -> ! {
        loop {}
    }

    #[task (resources = [last_tick], schedule = [check_timeout], spawn = [stop], priority = 1)]
    fn check_timeout(mut cx: check_timeout::Context){
        //let mut ticktime : Instant = rtic::CYCCNT::zero();
        //cx.resources.last_tick.lock(|last_tick| {ticktime = *last_tick});
        //if ticktime.elapsed() > TIMEOUT.cycles() {
        //    cx.spawn.stop().unwrap();
       // }
        //cx.schedule.check_timeout(Instant::now() + TIMEOUT.cycles()).unwrap();
    }

    #[task(binds = EXTI9_5, resources = [&clocks, buttons, EXTI, ppq, display, button_pressed], spawn = [stop], priority = 1)]
    fn handle_buttons(mut cx: handle_buttons::Context){
        // Clear interrupt bits
        cx.resources.EXTI.pr.modify(|_, w| w.pr6().set_bit());
        cx.resources.EXTI.pr.modify(|_, w| w.pr7().set_bit());

        let mut disp_str: &'static str = "";
        // Check button state
        match(cx.resources.buttons.0.is_low().unwrap(), cx.resources.buttons.1.is_low().unwrap()) {
            // "Next" button pressed
            (true, false) => {
                if *cx.resources.button_pressed == false {
                    *cx.resources.button_pressed = true;
                    cx.resources.ppq.lock(|ppq|{
                        *ppq = ppq.next();
                        disp_str = ppq.to_str();
                    });
                    let _ = cx.spawn.stop();
                    update_display(&mut cx.resources.display, disp_str);
                }
            },
            // Both buttons pressed
            (true, true) => *cx.resources.button_pressed = true,
            // "Prev" button pressed
            (false, true) => {
                if *cx.resources.button_pressed == false {
                    *cx.resources.button_pressed = true;
                    cx.resources.ppq.lock(|ppq|{
                        *ppq = ppq.prev();
                        disp_str = ppq.to_str();
                    });
                    let _ = cx.spawn.stop();
                    update_display(&mut cx.resources.display, disp_str);
                }
            },
            // No buttons pressed (Ready to handle another button press)
            (false, false) => {
                *cx.resources.button_pressed = false;
                // Debounce delay for 1/40 of a second
                delay(cx.resources.clocks.sysclk().0 / 40 );
            },
        }
    }

    #[task(resources = [beat_clock, counter, ppq, led, last_tick], priority=2)]
    fn tick(cx: tick::Context){
        *cx.resources.last_tick = Instant::now();
        let ppqnum: u8 = cx.resources.ppq.to_u8();
        if ppqnum < 24 {
            let count_max: u8 = cx.resources.ppq.to_max();
            if *cx.resources.counter == 0 {
                cx.resources.beat_clock.set_high().unwrap();
                cx.resources.led.set_low().unwrap();
            } else if *cx.resources.counter == (count_max >> 1){
                cx.resources.beat_clock.set_low().unwrap();
                cx.resources.led.set_high().unwrap();
            }
            *cx.resources.counter += 1;
            if *cx.resources.counter >= count_max {*cx.resources.counter = 0;}
        }else{
            // Unimplemented: clock multiplication
            return
        }
    }

    #[task(resources = [beat_clock, counter, ppq, led], priority=2)]
    fn stop(cx: stop::Context){
        *cx.resources.counter = 0;
        cx.resources.beat_clock.set_low().unwrap();
        cx.resources.led.set_high().unwrap();
    }

    #[task(binds = USB_HP_CAN_TX, spawn = [handle_usb], priority=3)]
    fn usb_hp_can_tx(cx: usb_hp_can_tx::Context){
        cx.spawn.handle_usb().unwrap();
    }

    #[task(binds = USB_LP_CAN_RX0, spawn = [handle_usb], priority=3)]
    fn usb_lp_can_rx0(cx: usb_lp_can_rx0::Context){
        cx.spawn.handle_usb().unwrap();
    }

    #[task(resources = [midi, usb_dev], spawn = [tick, stop], priority=3)]
    fn handle_usb(cx: handle_usb::Context){
        // Make sure we have data, if not we can leave
        if !cx.resources.usb_dev.poll(&mut [cx.resources.midi]) {
            return;
        }
        let mut buffer = [0; 64];
        if let Ok(_) = cx.resources.midi.read(&mut buffer) {
            for packet in buffer.chunks(4){
                if packet[0] == 0 {break;}
                match packet[1] {
                    0xf8 => {let _ = cx.spawn.tick();}
                    0xfa => {let _ = cx.spawn.stop();}
                    0xfc => {let _ = cx.spawn.stop();}
                    _ => {}
                }
            }
        }
        return;
    }
    // Required for software tasks
    extern "C" {
        // Uses the DMA1_CHANNELX interrupts for software
        // task scheduling.
        fn DMA1_CHANNEL1();
        fn DMA1_CHANNEL2();
        fn DMA1_CHANNEL3();
    }
};

