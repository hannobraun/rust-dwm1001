//! Accesses the LIS2DH12 3-axis accelerometer


#![no_main]
#![no_std]


#[macro_use] extern crate cortex_m_rt;
#[macro_use] extern crate dwm1001;

// extern crate panic_semihosting;
extern crate panic_halt;

use core::sync::atomic::{AtomicUsize, Ordering};

use dwm1001::{
    // debug,
    DWM1001,
    prelude::*,
    nrf52832_hal::{
        Delay,
        Twim,
        nrf52832_pac::TWIM1,
    },
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

    let mut acc = Lis2Dh12::new(address, dwm1001.LIS2DH12);

    let mut chunk = WriteBuf::new();


    let mut x = AtomicUsize::new(5);
    let y = x.swap(10, Ordering::SeqCst);





    // init
    {
        let whoami = acc.whoami().unwrap();

        write!(&mut chunk, "WHOAMI: {:08b}\r\n", whoami);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        assert_eq!(whoami, 0b00110011);

        acc.low_power_enable(false).unwrap();
        acc.high_resolution_enable(true).unwrap();
        acc.endian_swap_enable(false).unwrap();

        let temp_en = acc.temp_is_enabled().unwrap();
        write!(&mut chunk, "TEMP_EN: {}\r\n", temp_en);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        let bdu_en = acc.block_data_update_is_enabled().unwrap();
        write!(&mut chunk, "BDU_EN: {}\r\n", bdu_en);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        let odr = acc.get_odr().unwrap();
        write!(&mut chunk, "ODR: {:?}\r\n", odr);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        let ctrl_1 = acc.get_control_1().unwrap();
        write!(&mut chunk, "CTRL_1: {:08b}\r\n", ctrl_1);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        let ctrl_4 = acc.get_control_4().unwrap();
        write!(&mut chunk, "CTRL_4: {:08b}\r\n", ctrl_4);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        let stat_aux = acc.get_status_aux().unwrap();
        write!(&mut chunk, "STAT AUX: 0x{:02X}\r\n", stat_aux);
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        if !temp_en {
            acc.temp_enable(true).unwrap();
        }

        if !bdu_en {
            acc.block_data_update_enable(true).unwrap();
        }

        if odr != Odr::HR_NORMAL_LP_1HZ {
            acc.set_odr(Odr::HR_NORMAL_LP_1HZ).unwrap();
        }
    }

    loop {
        write!(&mut chunk, "Temp: {}\r\n", acc.get_temp().unwrap());
        dwm1001.uart.write(chunk.slice_and_reset()).unwrap();

        delay.delay_ms(500u32);
    }
}

// ---------------------------------------------------------------------------

struct Lis2Dh12 {
    address: u8,
    periph: Twim<TWIM1>
}

pub mod registers {
    pub const STATUS_AUX: u8 = 0x07;
    pub const TEMP_OUT_L: u8 = 0x0C;
    pub const TEMP_OUT_H: u8 = 0x0D;
    pub const WHOAMI: u8 = 0x0F;
    pub const TEMP_CONFIG: u8 = 0x1F;
    pub const CTRL1: u8 = 0x20;
    pub const CTRL4: u8 = 0x23;
}


#[derive(Debug, Ord, PartialOrd, Eq, PartialEq)]
enum Odr {
    POWER_DOWN,
    HR_NORMAL_LP_1HZ,
    HR_NORMAL_LP_10HZ,
    HR_NORMAL_LP_25HZ,
    HR_NORMAL_LP_50HZ,
    HR_NORMAL_LP_100HZ,
    HR_NORMAL_LP_200HZ,
    HR_NORMAL_LP_400HZ,
    LP_1620HZ,
    HR_NORMAL_1344HZ_LP_5376_HZ,
}

impl Odr {
    fn from_u8(data: u8) -> Self {
        use crate::Odr::*;
        match data {
            0b0000 => POWER_DOWN,
            0b0001 => HR_NORMAL_LP_1HZ,
            0b0010 => HR_NORMAL_LP_10HZ,
            0b0011 => HR_NORMAL_LP_25HZ,
            0b0100 => HR_NORMAL_LP_50HZ,
            0b0101 => HR_NORMAL_LP_100HZ,
            0b0110 => HR_NORMAL_LP_200HZ,
            0b0111 => HR_NORMAL_LP_400HZ,
            0b1000 => LP_1620HZ,
            0b1001 => HR_NORMAL_1344HZ_LP_5376_HZ,
            _ => panic!(),
        }
    }

    fn into_u8(self) -> u8 {
        use crate::Odr::*;
        match self {
             POWER_DOWN => 0b0000,
             HR_NORMAL_LP_1HZ => 0b0001,
             HR_NORMAL_LP_10HZ => 0b0010,
             HR_NORMAL_LP_25HZ => 0b0011,
             HR_NORMAL_LP_50HZ => 0b0100,
             HR_NORMAL_LP_100HZ => 0b0101,
             HR_NORMAL_LP_200HZ => 0b0110,
             HR_NORMAL_LP_400HZ => 0b0111,
             LP_1620HZ => 0b1000,
             HR_NORMAL_1344HZ_LP_5376_HZ => 0b1001,
            _ => panic!(),
        }
    }
}

impl Lis2Dh12 {
    fn new(address: u8, periph: Twim<TWIM1>) -> Self {
        Lis2Dh12 {
            address,
            periph,
        }
    }

    #[inline]
    fn read_one(&mut self, sub_addr: u8) -> Result<u8, ()> {
        let mut buf = [0];
        self.periph.write_then_read(self.address, &[sub_addr], &mut buf).map_err(|_| ())?;
        Ok(buf[0])
    }

    #[inline]
    fn read_two(&mut self, sub_addr: u8) -> Result<u16, ()> {
        let mut buf = [0; 2];
        self.periph.write_then_read(self.address, &[0x80 | sub_addr], &mut buf).map_err(|_| ())?;
        Ok(
             (buf[0] as u16) |
            ((buf[1] as u16) << 8)
        )
    }

    #[inline]
    fn write_one(&mut self, sub_addr: u8, data: u8) -> Result<(), ()> {
        self.periph.write(self.address, &[sub_addr, data]).map_err(|_| ())
    }

    fn whoami(&mut self) -> Result<u8, ()> {
        self.read_one(registers::WHOAMI)
    }

    fn temp_is_enabled(&mut self) -> Result<bool, ()> {
        match self.read_one(registers::TEMP_CONFIG)? {
            0b1100_0000 => Ok(true),
            0b0000_0000 => Ok(false),
            _ => Err(()),
        }
    }

    fn temp_enable(&mut self, enable: bool) -> Result<(), ()> {
        let en = if enable {
            0b1100_0000
        } else {
            0b0000_0000
        };

        self.write_one(registers::TEMP_CONFIG, en)
    }

    fn block_data_update_is_enabled(&mut self) -> Result<bool, ()> {
        Ok((self.read_one(registers::CTRL4)? & 0b1000_0000) != 0)
    }

    fn block_data_update_enable(&mut self, enable: bool) -> Result<(), ()> {
        let current_val = self.read_one(registers::CTRL4)?;

        // TODO, don't rewrite if necessary
        let data = if enable {
            current_val | 0b1000_0000
        } else {
            current_val & !0b1000_0000
        };

        self.write_one(registers::CTRL4, data)
    }

    fn endian_swap_is_enabled(&mut self) -> Result<bool, ()> {
        Ok((self.read_one(registers::CTRL4)? & 0b0100_0000) != 0)
    }

    fn endian_swap_enable(&mut self, enable: bool) -> Result<(), ()> {
        let current_val = self.read_one(registers::CTRL4)?;

        // TODO, don't rewrite if necessary
        let data = if enable {
            current_val | 0b0100_0000
        } else {
            current_val & !0b0100_0000
        };

        self.write_one(registers::CTRL4, data)
    }

    fn high_resolution_is_enabled(&mut self) -> Result<bool, ()> {
        Ok((self.read_one(registers::CTRL4)? & 0b0100_0000) != 0)
    }

    fn high_resolution_enable(&mut self, enable: bool) -> Result<(), ()> {
        let current_val = self.read_one(registers::CTRL4)?;

        // TODO, don't rewrite if necessary
        let data = if enable {
            current_val | 0b0000_1000
        } else {
            current_val & !0b0000_1000
        };

        self.write_one(registers::CTRL4, data)
    }

    fn get_odr(&mut self) -> Result<Odr, ()> {
        let data = self.read_one(registers::CTRL1)?;
        Ok(Odr::from_u8(data >> 4))
    }

    fn set_odr(&mut self, odr: Odr) -> Result<(), ()> {
        let data = self.read_one(registers::CTRL1)?;
        self.write_one(
            registers::CTRL1,
            (data & 0x0F) | (odr.into_u8() << 4)
        )
    }

    fn get_temp_raw(&mut self) -> Result<u16, ()> {
        // TODO, preconditions
        // self.read_two(registers::TEMP_OUT_L)

        let hi = self.read_one(registers::TEMP_OUT_H)?;
        let lo = self.read_one(registers::TEMP_OUT_L)?;

        Ok(((hi as u16) << 8) | (lo as u16))
    }

    fn get_temp(&mut self) -> Result<i32, ()> {
        let temp: i16 = self.get_temp_raw()? as i16;
        Ok(i32::from(temp >> 6))
    }

    fn get_status_aux(&mut self) -> Result<u8, ()> {
        self.read_one(registers::STATUS_AUX)
    }

    fn get_control_1(&mut self) -> Result<u8, ()> {
        self.read_one(registers::CTRL1)
    }

    fn get_control_4(&mut self) -> Result<u8, ()> {
        self.read_one(registers::CTRL4)
    }

    fn low_power_enable(&mut self, enable: bool) -> Result<(), ()> {
        let data = self.read_one(registers::CTRL1)?;
        let set = if enable {
            data | 0b0000_1000
        } else {
            data & 0b1111_0111
        };

        self.write_one(registers::CTRL1, set)
    }
}

// ---------------------------------------------------------------------------

use core::{fmt, fmt::Write};

struct WriteBuf {
    data: [u8; 255],
    pos: usize,
}

impl WriteBuf {
    pub fn new() -> Self {
        WriteBuf {
            data: [0; 255],
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
