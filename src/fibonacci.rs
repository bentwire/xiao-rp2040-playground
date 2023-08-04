#![no_std]
#![no_main]

mod pins;
mod colorwheel;
use colorwheel::colorwheel;
use cortex_m::delay::Delay;
use seeeduino_xiao_rp2040::hal::timer::CountDown;
use fugit::RateExtU32;

use embedded_hal::digital::v2::OutputPin;
use hal::pio::PIOExt;
use hal::Timer;
// Force link against panic_halt;
use panic_halt as _;
use seeeduino_xiao_rp2040::entry;
use seeeduino_xiao_rp2040::hal;
use seeeduino_xiao_rp2040::hal::gpio::{FunctionConfig, PinId, Function, ValidPinMode};
use seeeduino_xiao_rp2040::hal::pac;
use seeeduino_xiao_rp2040::hal::pio::StateMachineIndex;
use seeeduino_xiao_rp2040::hal::prelude::*;
use smart_leds::SmartLedsWrite;
use ws2812_pio::Ws2812;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let core = pac::CorePeripherals::take().unwrap();
    let mut pac = pac::Peripherals::take().unwrap();
 
    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
 
    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        seeeduino_xiao_rp2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // PClk
    let pclk_freq = clocks.peripheral_clock.freq();
 
    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);
    let _core_id = hal::Sio::core();

    // Set the pins up according to their function on this particular board
    let pins = pins::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
 
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // UART Config
    let _debug_uart = hal::uart::UartPeripheral::new(pac.UART0,
                (pins.tx.into_mode(), pins.rx.into_mode()),
                &mut pac.RESETS).enable(
                    hal::uart::UartConfig::new(
                        115200u32.Hz(), 
                        hal::uart::DataBits::Eight, 
                        None, 
                        hal::uart::StopBits::One),
                    pclk_freq).unwrap();


    // Setup PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Timer for Neopixel driver
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // Setup Neopixel RGB LED driver
    let mut ws: Ws2812<pac::PIO0, hal::pio::SM0, hal::timer::CountDown<'_>, hal::gpio::bank0::Gpio3> = Ws2812::new(
        pins.mosi.into_mode(), //pins.neopixel_data.into_mode(),
        &mut pio,
        sm0,
        pclk_freq,
        timer.count_down(),
    );

    // Configure the on board LED pins to operate as a push-pull output
    let mut led_blue_pin = pins.led_blue.into_push_pull_output();
    let mut led_green_pin = pins.led_green.into_push_pull_output();
    let mut led_red_pin = pins.led_red.into_push_pull_output();

    // All on board LEDs off.
    led_blue_pin.set_high().unwrap();
    led_green_pin.set_high().unwrap();
    led_red_pin.set_high().unwrap();

    // The 64 pixels on the fibonacci board
    // All blue for now.
    let mut pixels: [rgb::RGB<u8>; 64] = [rgb::RGB8::new(0, 0, 30); 64];
    
    led_green_pin.set_low().unwrap();
    ws.write(pixels.iter().copied()).unwrap();
    led_blue_pin.set_low().unwrap();

    delay.delay_ms(1000);
    
    led_green_pin.set_high().unwrap();

    for pix in pixels.iter_mut() {
        pix.b = 0;
    }
    //led_red_pin.set_low().unwrap();

    ws.write(pixels.iter().copied()).unwrap();

    delay.delay_ms(1000);

    led_blue_pin.set_high().unwrap();

    loop {
        led_red_pin.set_high().unwrap();
        rainbow_stripes(&mut ws, &mut delay);
        //delay.delay_ms(1000);

        led_red_pin.set_low().unwrap();
        rainbow_concentric(&mut ws, &mut delay);
        //delay.delay_ms(1000);
        rainbow_spirals(&mut ws, &mut delay);
    }
}        

fn rainbow_stripes<'l, P, SM, I>(ws: &mut Ws2812<P, SM, CountDown<'l>, I>, delay: &mut Delay)
    where
        P: PIOExt + FunctionConfig,
        Function<P>: ValidPinMode<I>,
        SM: StateMachineIndex,
        I: PinId,
{
    let mut pixels: [rgb::RGB<u8>; 64] = [rgb::RGB8::new(0, 0, 0); 64];

    let num_pixels = pixels.len();

    for _i in 0..5u8 {
        for cidx in 0..255u8 {
            let mut pix = 0;
            for pixel in pixels.iter_mut() {
                let pidx = (pix * 256 / num_pixels) + (255 - cidx) as usize;
                (pixel.r, pixel.g, pixel.b) = colorwheel((pidx & 0xff) as u8);
                pix += 1;
            }
            ws.write(pixels.iter().copied()).unwrap();
            delay.delay_ms(10);
        }
    }
}

fn rainbow_concentric<'l, P, SM, I>(ws: &mut Ws2812<P, SM, CountDown<'l>, I>, delay: &mut Delay)
    where
        P: PIOExt + FunctionConfig,
        Function<P>: ValidPinMode<I>,
        SM: StateMachineIndex,
        I: PinId,
{
    // Pixel translation matrix from rainbow+demo.py
    let spiral_translate = [33,26,42,25,27,41,19,43,34,18,49,20,32,40,11,48,24,17,53,10,44,39,12,54,21,28,52,5,47,35,13,59,9,31,50,4,55,23,16,60,6,45,38,3,58,8,29,61,1,56,36,14,63,7,30,51,2,57,22,15,62,0,46,37];

    let mut pixels: [rgb::RGB<u8>; 64] = [rgb::RGB8::new(0, 0, 0); 64];

    let num_pixels = pixels.len();

    for _i in 0..5u8 {
        for cidx in 0..255u8 {
            for pix in 0..num_pixels {
                let pidx = (pix * 256 / num_pixels) + cidx as usize;
                let pixel = &mut pixels[spiral_translate[pix]];
                (pixel.r, pixel.g, pixel.b) = colorwheel((pidx & 0xff) as u8);
            }
            ws.write(pixels.iter().copied()).unwrap();
            delay.delay_ms(10);
        }
    }
}

fn rainbow_spirals<'l, P, SM, I>(ws: &mut Ws2812<P, SM, CountDown<'l>, I>, delay: &mut Delay)
    where
        P: PIOExt + FunctionConfig,
        Function<P>: ValidPinMode<I>,
        SM: StateMachineIndex,
        I: PinId,
{
    // Pixel translation matrix from rainbow+demo.py
    let spiral_arms_91translate = [11,11,5,5,6,6,7,33,26,19,10,9,8,22,20,21,21,23,36,36,37,24,24,35,35,38,51,51,25,34,39,39,50,61,62,40,40,52,60,60,63,63,41,53,53,59,58,58,57,42,49,54,54,55,56,46,48,48,47,47,45,45,30,43,44,44,31,29,29,15,32,32,28,28,16,14,14,27,17,17,13,3,3,2,18,12,12,4,1,1,0];
    
    let mut pixels: [rgb::RGB<u8>; 64] = [rgb::RGB8::new(0, 0, 0); 64];

    for _i in 0..5u8 {
        for cidx in 0..255u8 {
            for pix in 0..91 {
                let pidx = (pix * 256 / 91) + cidx as usize;
                let pixel = &mut pixels[spiral_arms_91translate[pix]];
                (pixel.r, pixel.g, pixel.b) = colorwheel((pidx & 0xff) as u8);
            }
            ws.write(pixels.iter().copied()).unwrap();
            delay.delay_ms(10);
        }
    }
}

