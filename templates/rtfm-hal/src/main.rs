#![no_main]
#![no_std]

#[allow(unused_imports)]
use panic_halt;

use nrf52832_pac::{self, GPIOTE as REGISTER_GPIOTE, P0, UARTE0};
use rtfm::{app, Instant, U32Ext};

use nrf52832_hal::{gpio::GpioExt, uarte::UarteExt};
use nrf52832_hal::{
    gpio::{p0::P0_Pin, Floating, Input, Level, Output, PushPull},
    prelude::*,
    uarte::{self, Baudrate as UartBaudrate, Error as UartError, Parity as UartParity, Uarte},
};
use cortex_m::asm::wfi;

/// AJM: The EasyDMA engine does not copy data from flash. This extension
/// trait (inefficiently) copies this buffer to RAM, then sends from there.
///
/// See https://github.com/nrf-rs/nrf52-hal/issues/37
trait CopyWrite {
    fn flash_write(&mut self, buf: &[u8]) -> Result<(), UartError>;
}

impl CopyWrite for Uarte<UARTE0> {
    fn flash_write(&mut self, buf: &[u8]) -> Result<(), UartError> {
        const BUF_SIZE: usize = 255;

        if buf.len() > BUF_SIZE {
            return Err(UartError::TxBufferTooLong);
        }

        let mut ram_buf: [u8; BUF_SIZE] = [0; BUF_SIZE];
        ram_buf[0..buf.len()].copy_from_slice(buf);
        self.write(&ram_buf[0..buf.len()])
    }
}

/// State machine for debouncing the Button
enum ButtonState {
    Idle,
    SetActive(Instant),
}

const TICKS_PER_SEC: u32 = 64_000_000;

#[app(device = nrf52832_pac)]
const APP: () = {
    // Late Resources
    static mut UART:        Uarte<UARTE0>               = ();
    static mut LED_GREEN:   P0_Pin<Output<PushPull>>    = ();
    static mut LED_RED:     P0_Pin<Output<PushPull>>    = ();
    static mut SWITCH_PIN:  P0_Pin<Input<Floating>>     = ();
    static mut REG_GPIOTE:  REGISTER_GPIOTE             = ();

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
            UartBaudrate::BAUD115200,
        );

        let in_pin = pins.p0_02.degrade().into_floating_input();

        // AJM: nrf52-hal has no API to set the internal pullups. For now, we
        // manually set this here, retaining the rest of the normal floating configuration
        unsafe {
            (*P0::ptr()).pin_cnf[2].modify(|_r, w| w.pull().pullup());
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
    }

    /// This task blinks an LED, and sends data over the UART
    /// at a fixed periodic rate, using the `timer-queue` feature
    #[task(resources = [LED_RED, UART], schedule = [blink])]
    fn blink() {
        static mut IS_HIGH: bool = true;

        let next_interval = if *IS_HIGH {
            // Turn LED on (active low)
            (*resources.LED_RED).set_low();

            // Greet the world
            resources.UART.flash_write(b"Hello, world!\r\n").unwrap();

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
    #[interrupt(resources = [LED_GREEN, SWITCH_PIN, REG_GPIOTE])]
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

        *STATE = match (&STATE, pressed) {
            // Button pressed
            (ButtonState::Idle, true) => ButtonState::SetActive(Instant::now()),

            // button-released event but we're idle? What? Stay Idle
            (ButtonState::Idle, false) => ButtonState::Idle,

            // button-pressed event, but we're waiting for button-release
            (ButtonState::SetActive(inst), true) => {
                // AJM: For some reason, this triggers a LOT when the button is pressed,
                // not sure if there is a software error, or if the internal pullups just aren't sufficient
                // for this purpose. In theory, we should probably set the time to Instant::now(), but with
                // the retriggers, this is problematic. For now, we will just keep the time of first press,
                // to avoid losing presses. This is likely to cause real performance issues outside of this
                // demo, as I was seeing tens of thousands of "retriggers" per second when the button is
                // held down (low)
                ButtonState::SetActive(*inst)
            }
            // We have released the button after pressing it
            (ButtonState::SetActive(inst), false) => {
                // Check for minimal debounce hold time
                if inst.elapsed() >= DEBOUNCE_TIME.cycles() {
                    // Held for long enough
                    if *IS_HIGH {
                        (*resources.LED_GREEN).set_low();
                    } else {
                        (*resources.LED_GREEN).set_high();
                    }

                    *IS_HIGH = !*IS_HIGH;
                }

                // Return to idle state
                ButtonState::Idle
            }
        };

        // Mark event as addressed
        resources.REG_GPIOTE.events_in[0].modify(|_r, w| w);
    }

    // Since we are using periodic times, rtfm needs a sacrificial interrupt.
    // We're not using the Radio for now, so that works
    extern "C" {
        fn RADIO();
    }
};
