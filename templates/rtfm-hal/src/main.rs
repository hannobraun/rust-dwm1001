#![no_main]
#![no_std]

#[allow(unused_imports)]
use panic_halt;

use rtfm::app;
use nrf52832_pac;

#[app(device = nrf52832_pac)]
const APP: () = {
    #[init]
    fn init() {
    }

    #[idle]
    fn idle() -> ! {
        loop {}
    }
};
