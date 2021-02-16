use crate::ppq::Ppq;

use stm32f1xx_hal::{
    gpio::{
        gpiob::{PB8, PB9},
        {Alternate, OpenDrain},
    },
    pac::{I2C1},
    i2c::{BlockingI2c},
};

use ssd1306::{
    prelude::*,
};

use embedded_graphics::{
    fonts::Text,
    pixelcolor::BinaryColor,
    prelude::*,
    style::TextStyle,
};
use profont::ProFont24Point;


pub fn update_display(
    display: &mut GraphicsMode<I2CInterface<BlockingI2c<I2C1, (PB8<Alternate<OpenDrain>>,PB9<Alternate<OpenDrain>>) >>, DisplaySize128x64>,
    ppq: &mut Ppq,
) {
    display.clear();
    // Prepare to print PPQ value
    Text::new(&(ppq.to_str()), Point::new(20,16))
        .into_styled(TextStyle::new(ProFont24Point, BinaryColor::On))
        .draw(display)
        .unwrap();

    display.flush().unwrap();
}