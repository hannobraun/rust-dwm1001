#![no_main]
#![no_std]

#[allow(unused_imports)]
use panic_halt;

use rtfm::app;
use nrf52832_pac::{
    self,
    UARTE0,
};

use nrf52832_hal::{
    gpio::{
        Level,
    },
    uarte::{
        self,
        Uarte,
        Parity as UartParity,
        Baudrate as UartBaudrate,
    },
};
use nrf52832_hal::{
    gpio::GpioExt,
    uarte::UarteExt,
};

static MESSAGE: &str = "Hexxo!\r\n";

#[app(device = nrf52832_pac)]
const APP: () = {
    static mut UART: Uarte<UARTE0> = ();

    #[init]
    fn init() {
        let pins = device.P0.split();
        let uarte0 = device.UARTE0.constrain(uarte::Pins {
                txd: pins.p0_05.into_push_pull_output(Level::High).degrade(),
                rxd: pins.p0_11.into_push_pull_output(Level::High).degrade(),
                cts: None,
                rts: None,
            },
            UartParity::EXCLUDED,
            UartBaudrate::BAUD115200,
        );
        UART = uarte0;
    }

    #[idle(resources = [UART])]
    fn idle() -> ! {
        let out: [u8; 8] = [
            0x48, // H
            0x65, // e
            0x6C, // l
            0x6C, // l
            0x6F, // o
            0x21, // !
            0x0d, // CR
            0x0a  // LF
        ];

        // Prints
        resources.UART.write(
            &out
        ).unwrap();

        // Doesn't print
        resources.UART.write(
            &MESSAGE.as_bytes(),
        ).unwrap();

        let mut foo: [u8; 8] = [0u8; 8];
        foo.copy_from_slice(MESSAGE.as_bytes());

        // Does print
        resources.UART.write(
            &foo
        ).unwrap();

        loop {}
    }
};
