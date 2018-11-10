//! Accesses the LIS2DH12 3-axis accelerometer


#![no_main]
#![no_std]


#[macro_use] extern crate cortex_m_rt;
#[macro_use] extern crate dwm1001;

// extern crate panic_semihosting;
extern crate panic_halt;

use dwm1001::{
    // debug,
    DWM1001,
    prelude::*,
    nrf52832_hal::Delay,
};


#[entry]
fn main() -> ! {
    // debug::init();

    let mut dwm1001 = DWM1001::take().unwrap();

    let     clocks = dwm1001.CLOCK.constrain().freeze();
    let mut delay  = Delay::new(dwm1001.SYST, clocks);

    // SDO/SA0 is connected to the supply voltage, so the least significant bit
    // is 1. See datasheet, section 6.1.1.
    let address = 0b0011001;

    let mut write_buf = [0u8; 1];
    let mut read_buf  = [0u8; 1];

    // Read WHOAMI register (address 0x0F)
    write_buf[0] = 0x0F;
    dwm1001.LIS2DH12.write(address, &write_buf)
        .expect("Failed to write to I2C");
    dwm1001.LIS2DH12.read(address, &mut read_buf)
        .expect("Failed to read from I2C");
    assert_eq!(read_buf[0], 0b00110011);

    // print!("WHOAMI: {:08b}\n", read_buf[0]);

    let mut chunk = WriteBuf::new();

    loop {
        dwm1001.LIS2DH12.write(address, &write_buf)
            .expect("Failed to write to I2C");
        dwm1001.LIS2DH12.read(address, &mut read_buf)
            .expect("Failed to read from I2C");
        assert_eq!(read_buf[0], 0b00110011);

        write!(&mut chunk, "WHOAMI: {:08b}\r\n", read_buf[0]);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        delay.delay_ms(1000u32);
    }
}

use core::{fmt, fmt::Write};

struct WriteBuf {
    data: [u8; 256],
    pos: usize,
}

impl WriteBuf {
    pub fn new() -> Self {
        WriteBuf {
            data: [0; 256],
            pos: 0,
        }
    }

    pub fn clear(&mut self) {
        self.pos = 0;
    }

    pub fn slice_and_reset(&mut self) -> &[u8] {
        let pos = self.pos;
        self.clear();
        &self.data[0..pos]
    }
}

impl Write for WriteBuf {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let dat = s.as_bytes();
        let len = dat.len();
        assert!(len <= (self.data.len() - self.pos));
        self.data[self.pos..self.pos+len].copy_from_slice(&dat);
        self.pos += len;
        Ok(())
    }
}
