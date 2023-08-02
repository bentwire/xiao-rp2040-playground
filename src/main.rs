#![no_std]
#![no_main]

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
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    //let core = pac::CorePeripherals::take().unwrap();
 
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
 
    // Initialize USB.
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .product("Serial port")
        .device_class(USB_CLASS_CDC)
        .build();

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);
 
    // Set the pins up according to their function on this particular board
    let pins = seeeduino_xiao_rp2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
 
    // Configure the pins to operate as a push-pull output
    let mut led_blue_pin = pins.led_blue.into_push_pull_output();
    let mut led_green_pin = pins.led_green.into_push_pull_output();
    let mut led_red_pin = pins.led_red.into_push_pull_output();

    // Clk
    let pclk_freq = clocks.peripheral_clock.freq();

    // Setup multicore
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    led_blue_pin.set_high().unwrap();
    led_green_pin.set_high().unwrap();
    led_red_pin.set_low().unwrap();

    let _core1_task = core1.spawn(unsafe {&mut CORE1_STACK.mem}, move || {
        //core1_task(clocks.system_clock.freq(), pclk_freq, led_blue_pin, led_red_pin)
        let core = unsafe { pac::CorePeripherals::steal() };
        let mut pac = unsafe { pac::Peripherals::steal() };

        let mut sio = hal::Sio::new(pac.SIO);

        // Setup PIO
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

        // Timer for Neopixel driver
        let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

        // Setup Neopixel RGB LED
        let mut ws = Ws2812::new(
            pins.neopixel_data.into_mode(),
            &mut pio,
            sm0,
            pclk_freq,//clocks.peripheral_clock.freq(),
            timer.count_down(),
        );

        // Turn on Neopixel RGB LED
        let mut neopixel_power = pins.neopixel_power.into_push_pull_output();
        neopixel_power.set_high().unwrap();


        let mut delay1 = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
        let mut current_color: rgb::RGB<i16> = rgb::RGB::<i16>::default();
        let mut next_color: rgb::RGB<i16>;// = rgb::RGB::<i16>::default();
        //let mut req_color: rgb::RGB<u8> = RGB8::new(0, 0, 0);
        //ws.write([req_color].iter().copied()).unwrap();
        
        led_blue_pin.set_high().unwrap();
        led_red_pin.set_high().unwrap();
        loop {
            let input = sio.fifo.read();
            match input {
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
                        ws.write([color].iter().copied()).unwrap();
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

    //let delay_time = 100;
    led_green_pin.set_high().unwrap();
    loop {
        //led_blue_pin.set_high().unwrap();
        //led_red_pin.set_high().unwrap();
        
        //color = color.iter().map(|ch| { ch + 1 }).collect();
        // See if USB is ready.  If its not, don't do anything that takes time here!
        // It will cause the USB transaction to time out!
        if !usb_dev.poll(&mut [ &mut serial ]) {
            // color += incr;
            // incr = incr.iter().map(|ch| { ch + 1 }).collect();
    
            // // Set RGB LED via core1 task.
            // let data: u32 = bytemuck::cast(color.alpha(0u8));
    
            // sio.fifo.write_blocking(data);
    
            // //delay0.delay_ms(delay_time);
            // led_green_pin.toggle().unwrap();
    
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
        match serial.read(&mut buf) {
            Ok(rx_cnt) => {
                led_green_pin.toggle().unwrap();
                if rx_cnt >= 3 {
                    color.as_mut_slice().copy_from_slice(&buf[0..3]);

                    // Set RGB LED via core1 task.
                    let data: u32 = bytemuck::cast(color.alpha(0u8));

                    sio.fifo.write_blocking(data);                    
                }
            },
            Err(_) => {

            },
        }

        match serial.write(color.as_slice()) {
            Ok(_count) => {
                // count bytes were written
            },
            // No data could be written (buffers full)
            Err(UsbError::WouldBlock) => {

            },
            // An error occurred
            Err(_err) => {

            },
        };        
    }
}

// fn core1_task(sys_freq: Rate<u32, 1, 1>, pclk_freq: Rate<u32, 1, 1>, 
//         mut blue_led: Pin<hal::gpio::bank0::Gpio25, hal::gpio::Output<hal::gpio::PushPull>>,
//         mut red_led: Pin<hal::gpio::bank0::Gpio17, hal::gpio::Output<hal::gpio::PushPull>>) -> ! {
//     let mut pac = unsafe { pac::Peripherals::steal() };
//     let core = unsafe { pac::CorePeripherals::steal() };

//     let mut sio = hal::Sio::new(pac.SIO);
//     let pins = seeeduino_xiao_rp2040::Pins::new(
//         pac.IO_BANK0,
//         pac.PADS_BANK0,
//         sio.gpio_bank0,
//         &mut pac.RESETS,
//     );

//     // Setup PIO
//     let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
 
//     let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

//     // Setup Neopixel RGB LED
//     let mut ws = Ws2812::new(
//         pins.neopixel_data.into_mode(),
//         &mut pio,
//         sm0,
//         pclk_freq,//clocks.peripheral_clock.freq(),
//         timer.count_down(),
//     );
 
//     // Turn on Neopixel RGB LED
//     let mut neopixel_power = pins.neopixel_power.into_push_pull_output();
//     neopixel_power.set_high().unwrap();


//     let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq.to_Hz());
//     let mut current_color: rgb::RGB<i16> = rgb::RGB::<i16>::default();
//     let mut next_color: rgb::RGB<i16> = rgb::RGB::<i16>::default();
//     let mut req_color: rgb::RGB<u8> = RGB8::new(0, 0, 0);
//     //ws.write([req_color].iter().copied()).unwrap();
    
//     blue_led.set_high().unwrap();
//     red_led.set_high().unwrap();

//     loop {
//         let input = sio.fifo.read();
//         match input {
//             Some(word) => {
//                 // Turn RED off Blue on.
//                 red_led.set_high().unwrap();
//                 blue_led.set_low().unwrap();

//                 next_color = req_color.into();
//                 //req_color = cast(word);
    
//                 // Get the difference so we can 'slew' to the new color
//                 let diff_color: rgb::RGB<i16> = current_color - next_color;
//                 // Set the incrment signs.
//                 diff_color.map(|c| { if c < 0 { -1 } else if c > 0 { 1 } else { 0 }});
    
//                 loop {
//                     // Toggle blue LED every loop we are running.
//                     blue_led.toggle().unwrap();
//                     let color: rgb::RGB<u8> = current_color.iter().map(|c| { if c < 0 { 0u8 } else if c > 255 { 255u8 } else { c as u8}}).collect();
//                     ws.write([color].iter().copied()).unwrap();
//                     delay.delay_ms(10);
//                     current_color += diff_color;

//                     // Are we done yet?
//                     if current_color == next_color {
//                         // Set blue led off
//                         blue_led.set_high().unwrap();
//                         break;
//                     }
//                 }
//                 //
//                 //sio.fifo.write_blocking(cast(new_color));
//             }
//             None => {
//                 delay.delay_ms(500);
//                 red_led.toggle().unwrap();
//             }
//         }
//     }
// }