#![no_main]
#![no_std]

use cortex_m_rt::entry;
use nb::block;
use panic_semihosting;
use cortex_m;
use bbqueue::{cortex_m_bbq, BBQueue};

use dwm1001::{
    nrf52832_hal::{
        prelude::*,
        timer::Timer,
    },
    DWM1001,
    Buffers,
};

const MEME: &[u8] = b"\r\n\r\nDid you ever hear the tragedy of Darth Plagueis The Wise? I thought not. It's not a story the Jedi would tell you. It's a Sith legend. Darth Plagueis was a Dark Lord of the Sith, so powerful and so wise he could use the Force to influence the midichlorians to create life... He had such a knowledge of the dark side that he could even keep the ones he cared about from dying. The dark side of the Force is a pathway to many abilities some consider to be unnatural. He became so powerful... the only thing he was afraid of was losing his power, which eventually, of course, he did. Unfortunately, he taught his apprentice everything he knew, then his apprentice killed him in his sleep. Ironic. He could save others from death, but not himself.\r\n\r\n";

#[entry]
fn main() -> ! {
    let (prod, cons) = cortex_m_bbq!(2048).unwrap().split();

    let bfrs = Buffers {
        uart_tx_producer: prod,
        uart_tx_consumer: cons,
    };

    let mut dwm1001 = DWM1001::take(bfrs).unwrap();
    let mut timer = dwm1001.TIMER0.constrain();

    unsafe {
        cortex_m::interrupt::enable();
    }

    loop {
        dwm1001.leds.D12.enable();
        delay(&mut timer, 3_000); // 20ms
        dwm1001.leds.D12.disable();
        delay(&mut timer, 5_000); // 230ms

        dwm1001.uart.write_async(&MEME).unwrap();
    }
}


fn delay<T>(timer: &mut Timer<T>, cycles: u32) where T: TimerExt {
    timer.start(cycles);
    block!(timer.wait()).unwrap();
}
