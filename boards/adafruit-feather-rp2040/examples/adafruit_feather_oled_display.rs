//! Rainbow effect color wheel using the onboard NeoPixel on an Adafruit Feather RP2040 board
//!
//! This flows smoothly through various colors on the onboard NeoPixel.
//! Uses the `ws2812_pio` driver to control the NeoPixel, which in turns uses the
//! RP2040's PIO block.
#![no_std]
#![no_main]

use adafruit_feather_rp2040::entry;
use adafruit_feather_rp2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        pio::PIOExt,
        timer::Timer,
        watchdog::Watchdog,
        Sio, I2C,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use core::iter::once;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, PrimitiveStyle},
    text::Text,
};
use embedded_hal::timer::CountDown;
use embedded_plots::{
    axis::Scale,
    curve::{Curve, PlotPoint},
    single_plot::SinglePlot,
};
use fugit::ExtU32;
use fugit::RateExtU32;
use panic_halt as _;
use rp2040_hal::gpio::FunctionI2C;
use sh1107::{prelude::*, Builder};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    //Configure the OLED display SH1107 -----------------------------------------

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.sda.into_mode::<FunctionI2C>();
    let scl_pin = pins.scl.into_mode::<FunctionI2C>();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // initialize ADC
    use rp2040_hal::adc::Adc;
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temperature_sensor = adc.enable_temp_sensor();

    let mut disp: GraphicsMode<_> = Builder::new()
        .with_size(DisplaySize::Display64x128)
        .with_rotation(DisplayRotation::Rotate90)
        .connect_i2c(i2c)
        .into();

    // Create a text style for drawing the font:
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    disp.init().unwrap();
    disp.clear();
    Text::new("123456", Point::new(0, 15), text_style)
        .draw(&mut disp)
        .unwrap();
    disp.flush().unwrap();

    // --------------------------------------------------------------------------
    delay.start(750.millis());
    let _ = nb::block!(delay.wait());

    use embedded_hal::adc::OneShot;
    use heapless::HistoryBuffer;
    use heapless::Vec;
    let mut reading = HistoryBuffer::<u16, 10>::new();
    let mut points = Vec::<PlotPoint, 10>::new();

    // Infinite colour wheel loop
    let mut n: u8 = 128;

    loop {
        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);

        if n % 10 == 0 {
            disp.clear();
            Text::new(color_sector_text(n), Point::new(20, 10), text_style)
                .draw(&mut disp)
                .ok();

            Circle::new(Point::new(80, 10), 16)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(&mut disp)
                .ok();

            let t = adc.read(&mut temperature_sensor).unwrap_or(0);
            reading.write(t);

            use core::fmt::Write;
            use heapless::String;
            let mut data = String::<32>::new();
            write!(data, "{t}").ok();

            Text::new(data.as_str(), Point::new(100, 20), text_style)
                .draw(&mut disp)
                .ok();

            points.clear();
            for (i, &v) in reading.oldest_ordered().enumerate() {
                points
                    .push(PlotPoint {
                        x: i as i32,
                        y: v as i32,
                    })
                    .ok();
            }

            if points.len() > 5 {
                let curve = Curve::from_data(points.as_slice());
                let curve_list = [(curve, BinaryColor::On)];

                //TODO: RangeFraction is not working properly. 
                //RangeFraction(5) should have 5 ticks, but get 9. If # of points < 5, seems to panic.
                if let Ok(plot) = SinglePlot::new(&curve_list, Scale::RangeFraction(5), Scale::Fixed(1000)) {
                    plot.into_drawable(Point { x: 30, y: 20 }, Point { x: 120, y: 45 })
                        .set_color(BinaryColor::On)
                        .set_text_color(BinaryColor::On)
                        .draw(&mut disp)
                        .ok();
                }
            }

            disp.flush().ok();
        }
        delay.start(25.millis());
        let _ = nb::block!(delay.wait());
    }
}

fn color_sector_text(wheel_pos: u8) -> &'static str {
    match 255 - wheel_pos {
        0..=84 => "Blue -> Red",
        85..=169 => "Green -> Blue",
        _ => "Red -> Green",
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
