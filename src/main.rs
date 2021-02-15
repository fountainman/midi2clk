#![no_std]
#![no_main]

mod ppq;
use ppq::Ppq;

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
//use panic_semihosting as _; // panic handler

use stm32f1xx_hal::{
    prelude::*,
    gpio::{
        gpioc::{PC13, PC14},
        gpiob::{PB8, PB9, PB6, PB7},
        {Output, PushPull},
        {Input, PullUp},
        {Alternate, OpenDrain},
    },
    pac::{I2C1},
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
        //usb_midi::usb_midi_event_packet::UsbMidiEventPacket,
        usb::constants::USB_AUDIO_CLASS,
        usb::constants::USB_MIDISTREAMING_SUBCLASS,
        /*midi::{
            notes::Note,
            midi::message::Message,
            midi::channel::Channel::*
        },*/
        usb_midi::midi_packet_reader::MidiPacketBufferReader,
    },
    midi_device::MidiClass,
};

//use core::fmt::Write;
use ssd1306::{
    prelude::*,
    Builder,
    I2CDIBuilder,
};
use embedded_graphics::{
    fonts::Text,
    pixelcolor::BinaryColor,
    prelude::*,
    style::TextStyle,
};
use profont::ProFont24Point;

// Import peripheral control methods from general HAL definition
use embedded_hal::digital::v2::{OutputPin, InputPin};
use core::ptr::write_volatile;

use cortex_m_semihosting::hio;
use core::fmt::Write;
//use cortex_m_semihosting::hprintln;

// Print function for debugging
#[allow(dead_code)]
pub fn print(value: [u8;4]) -> Result<(), core::fmt::Error> {
    let mut stdout = match hio::hstdout() {
        Ok(fd) => fd,
        Err(()) => return Err(core::fmt::Error),
    };

    write!(stdout, "Number: {:x}, {:x}, {:x}, {:x}\n", value[0], value[1], value[2], value[3])?;

    Ok(())
}

// Setup the app. We're using the hal Peripheral Access Crate for peripherals
#[rtic::app(device = stm32f1xx_hal::pac,
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
        #[init(Ppq::Ppq24)]
        ppq: Ppq,
        beat_clock: PC14<Output<PushPull>>,
        buttons: (PB7<Input<PullUp>>, PB6<Input<PullUp>>),
        #[init(false)]
        button_pressed: bool,
        EXTI: stm32f1xx_hal::pac::EXTI,
        clocks: stm32f1xx_hal::rcc::Clocks,
    }

    // Init function (duh)
    #[init(spawn = [update_display])]
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
            .freeze(&mut flash.acr);

        // Make sure clocks work with usb
        assert!(clocks.usbclk_valid());

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

        // Schedule the display to be updated with initial value
        cx.spawn.update_display().unwrap();

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

        // Return late resources to resources object
        init::LateResources {
            led,
            usb_dev,
            midi,
            display,
            buttons: (button_next, button_prev),
            beat_clock,
            EXTI: cx.device.EXTI,
            clocks
        }
    }

    // Idle function run when no tasks are running
    #[idle(resources = [])]
    fn idle(mut _cx: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = EXTI9_5, spawn = [update_display], resources = [clocks, buttons, EXTI, ppq, button_pressed], priority = 1)]
    fn handle_buttons(cx: handle_buttons::Context){
        // Clear interrupt bits
        cx.resources.EXTI.pr.modify(|_, w| w.pr6().set_bit());
        cx.resources.EXTI.pr.modify(|_, w| w.pr7().set_bit());

        // Check button state
        match(cx.resources.buttons.0.is_low().unwrap(), cx.resources.buttons.1.is_low().unwrap()) {
            // "Next" button pressed
            (true, false) => {
                if *cx.resources.button_pressed == false {
                    *cx.resources.button_pressed = true;
                    *cx.resources.ppq = cx.resources.ppq.next();
                    cx.spawn.update_display().unwrap();
                }
            },
            // Both buttons pressed
            (true, true) => *cx.resources.button_pressed = true,
            // "Prev" button pressed
            (false, true) => {
                if *cx.resources.button_pressed == false {
                    *cx.resources.button_pressed = true;
                    *cx.resources.ppq = cx.resources.ppq.prev();
                    cx.spawn.update_display().unwrap();
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

    #[task(resources = [display, ppq], priority=1)]
    fn update_display(cx: update_display::Context){

        cx.resources.display.clear();
        // Prepare to print PPQ value
        Text::new(&(cx.resources.ppq.to_str()), Point::new(20,16))
            .into_styled(TextStyle::new(ProFont24Point, BinaryColor::On))
            .draw(cx.resources.display)
            .unwrap();

        cx.resources.display.flush().unwrap();
    }

    #[task(binds = USB_HP_CAN_TX, spawn = [handle_usb], priority=3)]
    fn usb_hp_can_tx(cx: usb_hp_can_tx::Context){
        cx.spawn.handle_usb().unwrap();
    }

    #[task(binds = USB_LP_CAN_RX0, spawn = [handle_usb], priority=3)]
    fn usb_lp_can_rx0(cx: usb_lp_can_rx0::Context){
        cx.spawn.handle_usb().unwrap();
    }
    #[task(resources = [led, clocks], priority=1)]
    fn blink_led(mut cx: blink_led::Context) {
        cx.resources.led.lock(|led| {
            led.set_low().unwrap();
        });
        delay(cx.resources.clocks.sysclk().0 / 80);
        cx.resources.led.lock(|led| {
            led.set_high().unwrap();
        });
    }

    #[task(resources = [led, midi, usb_dev], spawn = [blink_led], priority=3)]
    fn handle_usb(cx: handle_usb::Context){
        // Make sure we have data, if not we can leave
        if !cx.resources.usb_dev.poll(&mut [cx.resources.midi]) {
            return;
        }

        let mut buffer = [0; 64];
        if let Ok(size) = cx.resources.midi.read(&mut buffer) {
            let buffer_reader = MidiPacketBufferReader::new(&buffer, size);
            for packet in buffer_reader.into_iter() {
                if let Ok(packet) = packet {
                    /*match packet.message {
                        Message::NoteOn(Channel1, Note::C2, ..) => {
                            cx.resources.led.set_low().unwrap();
                        },
                        Message::NoteOff(Channel1, Note::C2, ..) => {
                            cx.resources.led.set_high().unwrap();
                        },
                        _ => {}
                    }*/
                    let bytes: [u8;4] = packet.into();
                    match bytes[1] {
                        //0xf8 => {let _ = cx.spawn.blink_led();},
                        //0xF8 => {cx.spawn.blink_led().unwrap();},
                        _ => {print(bytes).unwrap();},
                    }
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
    }
};

