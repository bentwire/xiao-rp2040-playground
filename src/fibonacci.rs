#![no_std]
#![no_main]

mod pins;

use fugit::RateExtU32;

use embedded_hal::digital::v2::{OutputPin,ToggleableOutputPin};
use hal::pio::PIOExt;
use hal::Timer;
// Force link against panic_halt;
use panic_halt as _;
use rgb::{ComponentMap, ComponentSlice};
use seeeduino_xiao_rp2040::entry;
use seeeduino_xiao_rp2040::hal;
use seeeduino_xiao_rp2040::hal::pac;
use seeeduino_xiao_rp2040::hal::prelude::*;
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
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
 
    // UART Config
    let debug_uart = hal::uart::UartPeripheral::new(pac.UART0,
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
    let mut ws = Ws2812::new(
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

    led_green_pin.set_high().unwrap();

    // The 64 pixels on the fibonacci board
    let mut pixels: [rgb::RGB<u8>; 64] = [rgb::RGB8::new(0, 0, 255); 64];
    
    loop {
        ws.write(pixels.iter().copied()).unwrap();
    }
}        
