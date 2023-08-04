use seeeduino_xiao_rp2040::hal;

hal::bsp_pins!(
    Gpio0 {
        name: tx,
        aliases: { FunctionUart: UartTx }
    },
    Gpio1 {
        name: rx
        aliases: { FunctionUart: UartRx, FunctionSpi: Csn }
    },
    Gpio2 {
        name: sck,
        aliases: { FunctionSpi: Sck }
    },
    Gpio3 {
         name: mosi,
         aliases: { FunctionSpi: Mosi }
    },
    Gpio4 {
         name: miso,
         aliases: { FunctionSpi: Miso }
    },
    Gpio6 {
        name: sda,
        aliases: { FunctionI2C: Sda }
    },
    Gpio7 {
        name: scl,
        aliases: { FunctionI2C: Scl }
    },
    Gpio10 { name: gpio10 },
    Gpio11 { name: neopixel_power },
    Gpio12 { name: neopixel_data },
    Gpio16 {
        name: led_green,
        aliases: { FunctionPwm: LedGreenPwm }
    },
    Gpio17 {
        name: led_red,
        aliases: { FunctionPwm: LedRedPwm }
    },
    Gpio25 {
        name: led_blue,
        aliases: { FunctionPwm: LedBluePwm }
    },
    Gpio26 { name: a0 },
    Gpio27 { name: a1 },
    Gpio28 { name: a2 },
    Gpio29 { name: a3 },
);