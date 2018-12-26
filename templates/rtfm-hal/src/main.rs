#![no_main]
#![no_std]

#[allow(unused_imports)]
use panic_halt;

use nrf52832_pac::{self, GPIOTE as REGISTER_GPIOTE, P0, UARTE0};
use rtfm::{app, Instant, U32Ext};

use core::fmt::Write;
use cortex_m::asm::wfi;
use heapless::{consts::*, String};
use nrf52832_hal::{gpio::GpioExt, uarte::UarteExt};
use nrf52832_hal::{
    gpio::{p0::P0_Pin, Floating, Input, Level, Output, PushPull},
    prelude::*,
    uarte::{self, Baudrate as UartBaudrate, Parity as UartParity, Uarte},
};

/// State machine for debouncing the Button
#[derive(Debug, Clone, Copy)]
pub enum ButtonState {
    Idle,
    SetActive((Instant, u64)),
}

#[derive(Debug)]
pub struct Status {
    blinks: u64,
    toggles: u64,
    btn_state: ButtonState,
}

const TICKS_PER_SEC: u32 = 64_000_000;

#[app(device = nrf52832_pac)]
const APP: () = {
    // Late Resources
    static mut UART: Uarte<UARTE0> = ();
    static mut LED_GREEN: P0_Pin<Output<PushPull>>  = ();
    static mut LED_RED: P0_Pin<Output<PushPull>>    = ();
    static mut SWITCH_PIN: P0_Pin<Input<Floating>>  = ();
    static mut REG_GPIOTE: REGISTER_GPIOTE          = ();
    static mut STATUS: Status                       = ();
    static mut FORMAT_BUF: String<U512>             = ();
    static     MESSAGE: String<U32>                 = ();

    #[init(schedule = [blink])]
    fn init() {
        let pins = device.P0.split();
        let uarte0 = device.UARTE0.constrain(
            uarte::Pins {
                txd: pins.p0_05.into_push_pull_output(Level::High).degrade(),
                rxd: pins.p0_11.into_push_pull_output(Level::High).degrade(),
                cts: None,
                rts: None,
            },
            UartParity::EXCLUDED,
            UartBaudrate::BAUD230400,
        );

        let in_pin = pins.p0_02.degrade().into_floating_input();

        // AJM: nrf52-hal has no API to set the internal pullups. For now, we
        // manually set this here, retaining the rest of the normal floating configuration
        unsafe {
            (*P0::ptr())
                .pin_cnf[2]
                .modify(|_r, w| w.pull().pullup());
        }

        // AJM: nrf52-hal has no API for `gpiote`. We manually configure the input
        // pin as an event-triggering input, and to trigger on any-edge transition.
        // We will use P0.02, which is SW2 on the DWM1001-DEV board
        device.GPIOTE.config[0].write(|w| {
            w.mode().event().polarity().toggle();
            unsafe {
                w.psel().bits(2) // must match SWITCH_PIN
            }
        });

        // AJM: Manually enable the interrupt for the 0th gpiote instance
        device.GPIOTE.intenset.write(|w| {
            // must match config[n] above!
            w.in0().set()
        });

        // Schedule the first iteration of the blinky task
        schedule
            .blink(Instant::now() + 32_000_000.cycles())
            .unwrap();

        // Set the "Late Resources"

        UART = uarte0;

        // LEDs:
        // 30: D09 / Green
        // 14: D12 / Red
        // 22: D11 / Red
        // 31: D10 / Green
        LED_GREEN = pins.p0_31.degrade().into_push_pull_output(Level::High);
        LED_RED = pins.p0_14.degrade().into_push_pull_output(Level::High);

        SWITCH_PIN = in_pin;
        REG_GPIOTE = device.GPIOTE;
        MESSAGE = String::from("Hello, World!\r\n");
        FORMAT_BUF = String::new();

        STATUS = Status {
            blinks: 0,
            toggles: 0,
            btn_state: ButtonState::Idle,
        };
    }

    /// This task blinks an LED, and sends data over the UART
    /// at a fixed periodic rate, using the `timer-queue` feature
    #[task(resources = [LED_RED, UART, MESSAGE, STATUS, FORMAT_BUF], schedule = [blink])]
    fn blink() {
        static mut IS_HIGH: bool = true;

        let next_interval = if *IS_HIGH {
            // Turn LED on (active low)
            (*resources.LED_RED).set_low();

            // // Greet the world
            // resources.UART.write(
            //     resources.MESSAGE.as_bytes()
            // ).unwrap();

            // Update the state variable
            resources.STATUS.blinks += 1;

            resources.FORMAT_BUF.clear();

            write!(&mut resources.FORMAT_BUF, "{:?}\r\n", resources.STATUS).unwrap();

            // Greet the world
            resources
                .UART
                .write(resources.FORMAT_BUF.as_bytes())
                .unwrap();

            // Short interval
            TICKS_PER_SEC / 16
        } else {
            // Turn LED off (active low)
            (*resources.LED_RED).set_high();

            // Long interval
            15 * (TICKS_PER_SEC / 16)
        };

        *IS_HIGH = !*IS_HIGH;

        // Schedule next update
        schedule.blink(scheduled + next_interval.cycles()).unwrap();
    }

    #[idle(resources = [UART])]
    fn idle() -> ! {
        loop {
            wfi();
        }
    }

    /// This interrupt is triggered by any changing edge of a given
    /// GPIO pin
    #[interrupt(resources = [LED_GREEN, SWITCH_PIN, REG_GPIOTE, STATUS])]
    fn GPIOTE() {
        // Tracks the state machine of our button
        static mut STATE: ButtonState = ButtonState::Idle;

        // Tracks the state of our LED
        static mut IS_HIGH: bool = true;

        // Pick a somewhat high debounce time, so it is easier to see
        // that the LED doesn't change on short taps
        const CYCLES_PER_MS: u32 = 64_000_000 / 1000;
        const DEBOUNCE_TIME: u32 = CYCLES_PER_MS * 100;

        // Button is active-low
        let pressed = (*resources.SWITCH_PIN).is_low();

        // TODO(AJM): Allow for bounces when pressed (e.g. release
        // before debounce time, but quickly re-presses)
        *STATE = match (&STATE, pressed) {
            // Button pressed
            (ButtonState::Idle, true) => {
                ButtonState::SetActive(
                    (Instant::now(), resources.STATUS.blinks)
                )
            }

            // button-released event but we're idle? What? Stay Idle
            (ButtonState::Idle, false) => ButtonState::Idle,

            // button-pressed event, but we're waiting for button-release
            (ButtonState::SetActive(inst), true) => ButtonState::SetActive(*inst),
            // We have released the button after pressing it
            (ButtonState::SetActive(inst), false) => {
                // Check for minimal debounce hold time
                // AJM: Workaround for https://github.com/japaric/cortex-m-rtfm/issues/121,
                //   Instants are not happy around clock rollovers. If the button has
                //   been pressed for >= 2 "blinks" (should be about a second, but as low
                //   as 500ms or so in the worst case), we're good
                if (resources.STATUS.blinks >= (inst.1 + 2))
                    || (inst.0.elapsed() >= DEBOUNCE_TIME.cycles()) {
                    // Held for long enough
                    if *IS_HIGH {
                        (*resources.LED_GREEN).set_low();
                    } else {
                        (*resources.LED_GREEN).set_high();
                    }

                    resources.STATUS.toggles += 1;
                    *IS_HIGH = !*IS_HIGH;
                }

                // Return to idle state
                ButtonState::Idle
            }
        };

        // Mark event as addressed
        resources.REG_GPIOTE.events_in[0].write(|w| w);

        // Update status variable
        resources.STATUS.btn_state = *STATE;
    }

    // Since we are using periodic times, rtfm needs a sacrificial interrupt.
    // We're not using the Radio for now, so that works
    extern "C" {
        fn RADIO();
    }
};
