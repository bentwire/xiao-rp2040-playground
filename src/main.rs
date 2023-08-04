#![no_std]
#![no_main]

mod pins;

use fugit::RateExtU32;

use embedded_hal::digital::v2::{OutputPin,ToggleableOutputPin};
use hal::pio::PIOExt;
use hal::Timer;
use hal::multicore::{Multicore, Stack};
// Force link against panic_halt;
use panic_halt as _;
use rgb::{ComponentMap, ComponentSlice};
use seeeduino_xiao_rp2040::entry;
use seeeduino_xiao_rp2040::hal;
use seeeduino_xiao_rp2040::hal::pac;
use seeeduino_xiao_rp2040::hal::prelude::*;
use seeeduino_xiao_rp2040::hal::usb::UsbBus;
use smart_leds::{SmartLedsWrite, RGB8};
use bytemuck::cast;
use ws2812_pio::Ws2812;

use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut CORE1_STACK: Stack<4096> = Stack::new();

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
 
    // Initialize USB.
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut usb_serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .product("Serial port")
        .device_class(USB_CLASS_CDC)
        .build();

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);
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

    // Configure the LED pins to operate as a push-pull output
    let mut led_blue_pin = pins.led_blue.into_push_pull_output();
    let mut led_green_pin = pins.led_green.into_push_pull_output();
    let mut led_red_pin = pins.led_red.into_push_pull_output();

    // Setup multicore
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    led_blue_pin.set_high().unwrap();
    led_green_pin.set_high().unwrap();
    led_red_pin.set_low().unwrap();

    debug_uart.write_full_blocking(b"Start Core 1\r\n");
    let _core1_task = core1.spawn(unsafe {&mut CORE1_STACK.mem}, move || {
        let core = unsafe { pac::CorePeripherals::steal() };
        let mut pac = unsafe { pac::Peripherals::steal() };

        let mut sio = hal::Sio::new(pac.SIO);
        let _core_id = hal::Sio::core();

        // Setup PIO
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

        let mut delay1 = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        // Timer for Neopixel driver
        let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

        // Setup Neopixel RGB LED
        let mut ws = Ws2812::new(
            pins.mosi.into_mode(), //pins.neopixel_data.into_mode(),
            &mut pio,
            sm0,
            pclk_freq,
            timer.count_down(),
        );

        // Turn on Neopixel RGB LED
        //let mut neopixel_power = pins.neopixel_power.into_push_pull_output();
        //neopixel_power.set_high().unwrap();


        // The 64 pixels on the fibonacci board
        let mut pixels: [rgb::RGB<u8>; 64] = [rgb::RGB8::default(); 64];
        
        let mut current_color: rgb::RGB<i16> = rgb::RGB::<i16>::default();
        let mut next_color: rgb::RGB<i16>;// = rgb::RGB::<i16>::default();
        let mut req_color: rgb::RGB<u8> = RGB8::new(0, 0, 255);
        ws.write([req_color].iter().copied()).unwrap();
        
        led_blue_pin.set_high().unwrap();
        led_red_pin.set_high().unwrap();
        loop {
            // FIFO is 8 deep.
            let input = sio.fifo.read();
            match input {
                // Data to read.
                Some(word) => {
                    // Turn RED off Blue on.
                    led_red_pin.set_high().unwrap();
                    led_blue_pin.set_low().unwrap();
                    
                    let rgba_color: rgb::RGBA<u8> = cast(word);
                    let req_color = rgba_color.rgb();
                    next_color = req_color.into();
        
                    // Get the difference so we can 'slew' to the new color
                    let diff_color: rgb::RGB<i16> = next_color - current_color;
                    // Set the incrment signs.
                    let diff_color = diff_color.map(|c| { if c < 0 { -1i16 } else if c > 0 { 1i16 } else { 0i16 }});
        
                    loop {
                        // Toggle blue LED every loop we are running.
                        //led_blue_pin.toggle().unwrap();
                        let color: rgb::RGB<u8> = current_color.iter().map(|c| { if c < 0 { 0u8 } else if c > 255 { 255u8 } else { c as u8}}).collect();
                        for pixel in pixels.iter_mut() {
                            *pixel = color;
                        }
                        ws.write(pixels.iter().copied()).unwrap();
                        delay1.delay_ms(10);
    
                        // Are we done yet?
                        if current_color.r != next_color.r {
                            // Set blue led on
                            led_blue_pin.set_low().unwrap();
                            current_color.r = current_color.r + diff_color.r;
                        }
                        
                        if current_color.g != next_color.g {
                            // Set blue led on
                            led_blue_pin.set_low().unwrap();
                            current_color.g = current_color.g + diff_color.g;
                        }
                        
                        if current_color.b != next_color.b {
                            // Set blue led on
                            led_blue_pin.set_low().unwrap();
                            current_color.b = current_color.b + diff_color.b;
                        }

                        if current_color.r == next_color.r && current_color.g == next_color.g && current_color.b == next_color.b {
                            // Set blue led off
                            led_blue_pin.set_high().unwrap();
                            break;
                        }
                        
                    }
                    //
                    //sio.fifo.write_blocking(cast(new_color));
                }
                None => {
                    delay1.delay_ms(10);
                    led_red_pin.toggle().unwrap();
                }
            }
        }
    }).unwrap();

    //let mut incr = RGB8::new(1, 2, 3);

    let mut color = RGB8::default();

    led_green_pin.set_high().unwrap();
    loop {
        
        // See if USB is ready.  If its not, don't do anything that takes time here!
        // It will cause the USB transaction to time out!
        if !usb_dev.poll(&mut [ &mut usb_serial ]) {
            led_green_pin.set_high().unwrap();
            continue;
        }

        // color += incr;
        // incr = incr.iter().map(|ch| { ch + 1 }).collect();

        // // Set RGB LED via core1 task.
        // let data: u32 = bytemuck::cast(color.alpha(0u8));

        // sio.fifo.write_blocking(data);

        // //delay0.delay_ms(delay_time);
        // led_green_pin.toggle().unwrap();


        // Serial buffer
        let mut buf = [0u8; 64];

        // is there data?
        // Make this a serialized struct eventually.
        match usb_serial.read(&mut buf) {
            // We received some data.
            Ok(rx_cnt) => {
                led_green_pin.set_low().unwrap();
                if rx_cnt >= 3 {
                    color.as_mut_slice().copy_from_slice(&buf[0..3]);

                    // Set RGB LED via core1 task.
                    let data: u32 = bytemuck::cast(color.alpha(0u8));

                    sio.fifo.write_blocking(data);                    
                }
            },
            // Something broke.
            Err(_) => {

            },
        }
        
        // Constantly send current color value.
        // This should be a serialized struct with
        // all the data to send eventually...
        match usb_serial.write(color.as_slice()) {
            // count bytes were written
            Ok(_count) => {
                // Don't care yet.
            },
            // No data could be written (buffers full)
            Err(UsbError::WouldBlock) => {
                // Don't care yet.
            },
            // An error occurred
            Err(_err) => {
                // Don't care yet.
            },
        };        
    }
}