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

macro_rules! acc_register {
    (
        $(
            $reg_upper:ident,
            $reg_addr:expr,
            $rw:expr,
            $default:expr; {
                #[$doc:meta]
                $(
                    $first_bit:expr,
                    $last_bit:expr,
                    $field_upper:expr;
                    #[$reg_doc:meta]
                )*
            }
        )*
    ) => {
        $(
            pub struct $reg_upper;
        )*
    }
}

acc_register! {
    STATUS_REG_AUX, 0x07, RO, None; {
        /// TODO - DOCS
        2, 2, TDA; /// TODO - DOCS
        6, 6, TOR; /// TODO - DOCS
    }
    OUT_TEMP_L, 0x0C, RO, None; {
        /// TODO - DOCS
        0, 7, T_LOW; /// TODO - DOCS
    }
    TEMP_OUT_H, 0x0D, RO, None; {
        /// TODO - DOCS
        0, 7, T_HIGH; /// TODO - DOCS
    }
    WHO_AM_I, 0x0F, RO, None; {
        /// TODO - DOCS
        0, 7, DEV_ID; /// TODO - DOCS
    }
    CTRL_REG0, 0x1E, RW, Some(0b001_0000); {
        /// TODO - DOCS
        7, 7, SDO_PU_DISC; /// TODO - DOCS
    }
    TEMP_CFG_REG, 0x1F, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        6, 7, TEMP_EN; /// TODO - DOCS
    }
    CTRL_REG1, 0x20, RW, Some(0b0000_0111); {
        /// TODO - DOCS
        0, 0, X_EN; /// TODO - DOCS
        1, 1, Y_EN; /// TODO - DOCS
        2, 2, Z_EN; /// TODO - DOCS
        3, 3, LP_EN; /// TODO - DOCS
        4, 7, ODR; /// TODO - DOCS
    }
    CTRL_REG2, 0x21, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 0, HP_IA1; /// TODO - DOCS
        1, 1, HP_IA2; /// TODO - DOCS
        2, 2, HP_CLICK; /// TODO - DOCS
        3, 3, FDS; /// TODO - DOCS
        4, 5, HP_CF; /// TODO - DOCS
        6, 7, HP_M; /// TODO - DOCS
    }
    CTRL_REG3, 0x22, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        1, 1, I1_OVERRUN; /// TODO - DOCS
        2, 2, I1_WTM; /// TODO - DOCS
        4, 4, I1_ZYXDA; /// TODO - DOCS
        5, 5, I1_IA2; /// TODO - DOCS
        6, 6, I1_IA1; /// TODO - DOCS
        7, 7, I1_CLICK; /// TODO - DOCS
    }
    CTRL_REG4, 0x23, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 0, SIM; /// TODO - DOCS
        1, 2, ST; /// TODO - DOCS
        3, 3, HR; /// TODO - DOCS
        4, 5, FS; /// TODO - DOCS
        6, 6, BLE; /// TODO - DOCS
        7, 7, BDU; /// TODO - DOCS
    }
    CTRL_REG5, 0x24, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 0, D4D_INT2; /// TODO - DOCS
        1, 1, LIR_INT2; /// TODO - DOCS
        2, 2, D4D_INT1; /// TODO - DOCS
        3, 3, LIR_INT1; /// TODO - DOCS
        6, 6, FIFO_EN; /// TODO - DOCS
        7, 7, BOOT; /// TODO - DOCS
    }
    CTRL_REG6, 0x25, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        1, 1, INT_POLARITY; /// TODO - DOCS
        3, 3, I2_ACT; /// TODO - DOCS
        4, 4, I2_BOOT; /// TODO - DOCS
        5, 5, I2_IA2; /// TODO - DOCS
        6, 6, I2_IA1; /// TODO - DOCS
        7, 7, I2_CLICK; /// TODO - DOCS
    }
    REFERENCE, 0x26, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 7, REF; /// TODO - DOCS
    }
    STATUS_REG, 0x27, RO, None; {
        /// TODO - DOCS
        0, 0, XDA; /// TODO - DOCS
        1, 1, YDA; /// TODO - DOCS
        2, 2, ZDA; /// TODO - DOCS
        3, 3, ZYXDA; /// TODO - DOCS
        4, 4, XOR; /// TODO - DOCS
        5, 5, YOR; /// TODO - DOCS
        6, 6, ZOR; /// TODO - DOCS
        7, 7, ZYXOR; /// TODO - DOCS
    }
    OUT_X_L, 0x28, RO, None; {
        /// TODO - DOCS
        0, 7, X_ACC_LOW; /// TODO - DOCS
    }
    OUT_X_H, 0x29, RO, None; {
        /// TODO - DOCS
        0, 7, X_ACC_HIGH; /// TODO - DOCS
    }
    OUT_Y_L, 0x2A, RO, None; {
        /// TODO - DOCS
        0, 7, Y_ACC_LOW; /// TODO - DOCS
    }
    OUT_Y_H, 0x2B, RO, None; {
        /// TODO - DOCS
        0, 7, Y_ACC_HIGH; /// TODO - DOCS
    }
    OUT_Z_L, 0x2C, RO, None; {
        /// TODO - DOCS
        0, 7, Z_ACC_LOW; /// TODO - DOCS
    }
    OUT_Z_H, 0x2D, RO, None; {
        /// TODO - DOCS
        0, 7, Z_ACC_HIGH; /// TODO - DOCS
    }
    FIFO_CTRL_REG, 0x2E, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 4, FTH; /// TODO - DOCS
        5, 5, TR; /// TODO - DOCS
        6, 7, FM; /// TODO - DOCS
    }
    FIFO_SRC_REG, 0x2F, RO, None; {
        /// TODO - DOCS
        0, 4, FSS; /// TODO - DOCS
        5, 5, EMPTY; /// TODO - DOCS
        6, 6, OVRN_FIFO; /// TODO - DOCS
        7, 7, WTM; /// TODO - DOCS
    }
    INT1_CFG, 0x30, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 0, XLIE; /// TODO - DOCS
        1, 1, XHIE; /// TODO - DOCS
        2, 2, YLIE; /// TODO - DOCS
        3, 3, YHIE; /// TODO - DOCS
        4, 4, ZLIE; /// TODO - DOCS
        5, 5, ZHIE; /// TODO - DOCS
        6, 6, SIX_D; /// TODO - DOCS
        7, 7, AOI; /// TODO - DOCS
    }
    INT1_SRC, 0x31, RO, None; {
        /// TODO - DOCS
        0, 0, XL; /// TODO - DOCS
        1, 1, XH; /// TODO - DOCS
        2, 2, YL; /// TODO - DOCS
        3, 3, YH; /// TODO - DOCS
        4, 4, ZL; /// TODO - DOCS
        5, 5, ZH; /// TODO - DOCS
        6, 6, IA; /// TODO - DOCS
    }
    INT1_THS, 0x32, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 6, THS; /// TODO - DOCS
    }
    INT1_DURATION, 0x33, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 6, DUR; /// TODO - DOCS
    }
    INT2_CFG, 0x34, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 0, XLIE; /// TODO - DOCS
        1, 1, XHIE; /// TODO - DOCS
        2, 2, YLIE; /// TODO - DOCS
        3, 3, YHIE; /// TODO - DOCS
        4, 4, ZLIE; /// TODO - DOCS
        5, 5, ZHIE; /// TODO - DOCS
        6, 6, SIX_D; /// TODO - DOCS
        7, 7, AOI; /// TODO - DOCS
    }
    INT2_SRC, 0x35, RO, None; {
        /// TODO - DOCS
        0, 0, XL; /// TODO - DOCS
        1, 1, XH; /// TODO - DOCS
        2, 2, YL; /// TODO - DOCS
        3, 3, YH; /// TODO - DOCS
        4, 4, ZL; /// TODO - DOCS
        5, 5, ZH; /// TODO - DOCS
        6, 6, IA; /// TODO - DOCS
    }
    INT2_THS, 0x36, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 6, THS; /// TODO - DOCS
    }
    INT2_DURATION, 0x37, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 6, DUR; /// TODO - DOCS
    }
    CLICK_CFG, 0x38, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 0, XS; /// TODO - DOCS
        1, 1, XD; /// TODO - DOCS
        2, 2, YS; /// TODO - DOCS
        3, 3, YD; /// TODO - DOCS
        4, 4, ZS; /// TODO - DOCS
        5, 5, ZD; /// TODO - DOCS
    }
    CLICK_SRC, 0x39, RO, None; {
        /// TODO - DOCS
        0, 0, X_CD; /// TODO - DOCS
        1, 1, Y_CD; /// TODO - DOCS
        2, 2, Z_CD; /// TODO - DOCS
        3, 3, SIGN; /// TODO - DOCS
        4, 4, S_CLICK; /// TODO - DOCS
        5, 5, D_CLICK; /// TODO - DOCS
        6, 6, IA; /// TODO - DOCS
    }
    CLICK_THS, 0x3A, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 6, THS; /// TODO - DOCS
        7, 7, LIR_CLICK; /// TODO - DOCS
    }
    TIME_LIMIT, 0x3B, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 6, TLI; /// TODO - DOCS
    }
    TIME_LATENCY, 0x3C, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 7, TLA; /// TODO - DOCS
    }
    TIME_WINDOW, 0x3D, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 7, TW; /// TODO - DOCS
    }
    ACT_THS, 0x3E, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 6, ACTH; /// TODO - DOCS
    }
    ACT_DUR, 0x3F, RW, Some(0b0000_0000); {
        /// TODO - DOCS
        0, 7, ACTD; /// TODO - DOCS
    }
}
