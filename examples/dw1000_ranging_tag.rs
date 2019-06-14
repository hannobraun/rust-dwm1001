//! Range measurement tag node
//!
//! This is a tag node used for range measurement. Tags use anchor nodes to
//! measure their distance from those anchors.
//!
//! Currently, distance measurements have a highly inaccurate result. One reason
//! that could account for this is the lack of antenna delay calibration, but
//! it's possible that there are various hidden bugs that contribute to this.

#![no_main]
#![no_std]


extern crate panic_semihosting;


use cortex_m_rt::entry;
use nb::block;

use dwm1001::{
    prelude::*,
    debug,
    dw1000::{
        mac,
        ranging::{
            self,
            Message as _RangingMessage,
        },
    },
    nrf52832_hal::{
        nrf52832_pac::SPIM2,
        Delay,
        Spim,
    },
    DWM1001,
    block_timeout,
    repeat_timeout,
    print,
};


#[entry]
fn main() -> ! {
    debug::init();

    let mut dwm1001 = DWM1001::take().unwrap();

    let mut delay  = Delay::new(dwm1001.SYST);
    let mut rng    = dwm1001.RNG.constrain();

    dwm1001.DW_RST.reset_dw1000(&mut delay);
    let mut dw1000 = dwm1001.DW1000.init()
        .expect("Failed to initialize DW1000");

    let mut dw_irq = dwm1001.DW_IRQ;
    let mut nvic   = dwm1001.NVIC;
    let mut gpiote = dwm1001.GPIOTE;

    // These are the hardcoded calibration values from the dwm1001-examples
    // repository[1]. Ideally, the calibration values would be determined using
    // the proper calibration procedure, but hopefully those are good enough for
    // now.
    //
    // [1] https://github.com/Decawave/dwm1001-examples
    dw1000.set_antenna_delay(16456, 16300)
        .expect("Failed to set antenna delay");

    // Set network address
    dw1000
        .set_address(
            mac::PanId(0x0d57),                  // hardcoded network id
            mac::ShortAddress(rng.random_u16()), // random device address
        )
        .expect("Failed to set address");

    let mut task_timer    = dwm1001.TIMER0.constrain();
    let mut timeout_timer = dwm1001.TIMER1.constrain();

    loop {
        let mut buf = [0; 128];

        // Listen for messages
        task_timer.start(100_000u32);
        repeat_timeout!(
            &mut task_timer,
            {
                let mut future = dw1000
                    .receive()
                    .expect("Failed to receive message");
                future.enable_interrupts()
                    .expect("Failed to enable interrupts");

                timeout_timer.start(100_000u32);
                block_timeout!(&mut timeout_timer, {
                    dw_irq.wait_for_interrupts(
                        &mut nvic,
                        &mut gpiote,
                        &mut timeout_timer,
                    );
                    future.wait(&mut buf)
                })
            },
            (message) {
                let ping = ranging::Ping::decode::<Spim<SPIM2>>(&message)
                    .expect("Failed to decode ping");
                if let Some(ping) = ping {
                    // Received ping from an anchor. Reply with a ranging
                    // request.

                    let mut future = ranging::Request::new(&mut dw1000, ping)
                        .expect("Failed to initiate request")
                        .send(&mut dw1000)
                        .expect("Failed to initiate request transmission");
                    future.enable_interrupts()
                        .expect("Failed to enable interrupts");

                    timeout_timer.start(100_000u32);
                    block!({
                        dw_irq.wait_for_interrupts(
                            &mut nvic,
                            &mut gpiote,
                            &mut timeout_timer,
                        );
                        future.wait()
                    })
                    .expect("Failed to send ranging request");

                    continue;
                }

                let response =
                    ranging::Response::decode::<Spim<SPIM2>>(&message)
                        .expect("Failed to decode response");
                if let Some(response) = response {
                    // If this is not a PAN ID and short address, it doesn't
                    // come from a compatible node. Ignore it.
                    let (pan_id, addr) = match response.source {
                        mac::Address::Short(pan_id, addr) =>
                            (pan_id, addr),
                        _ =>
                            continue,
                    };

                    // Ranging response received. Compute distance.
                    match ranging::compute_distance_mm(&response) {
                        Some(distance_mm) => {
                            print!("{:04x}:{:04x} - {} mm\n",
                                pan_id.0,
                                addr.0,
                                distance_mm,
                            );
                        }
                        None => {
                            print!("Distance too large; can't compute");
                            continue;
                        }
                    }

                    continue;
                }

                print!("Ignored message that was neither ping nor response\n");
            };
            (_error) {
                // ignore error
            };
        );
    }
}
