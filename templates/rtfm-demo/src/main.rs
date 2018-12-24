#![no_main]
#![no_std]

#[allow(unused_imports)]
use panic_semihosting;

use cortex_m_semihosting::{debug, hprintln};
use rtfm::app;

use nrf52832_pac;

#[app(device = nrf52832_pac)]
const APP: () = {
    #[init]
    fn init() {
        hprintln!("init").unwrap();
    }

    #[idle]
    fn idle() -> ! {
        hprintln!("idle").unwrap();

        debug::exit(debug::EXIT_SUCCESS);

        loop {}
    }
};
