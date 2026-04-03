// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

//! Register definitions and register access

use core::{cell::RefCell, borrow::BorrowMut};
use crate::types::*;

// use embedded_hal::i2c;
use embedded_hal::i2c::I2c;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};


pub struct RegisterAccess<'a, I2C> {
    i2c: &'a Mutex<CriticalSectionRawMutex, RefCell<I2C>>,
    address: SlaveAddress,
    /// Stores if the FUNC_CFG_ACCESS - which is used to switch between normal registers and
    /// embedded functions registers - is currently configured to access the former or the latter
    embedded_functions_registers_selected: bool,
}

/// Manages register access to [`Register`]s, i.e. to [`PrimaryRegister`]s and
/// [`EmbeddedFunctionsRegister`]s
///
/// Since [`PrimaryRegister`]s and [`EmbeddedFunctionsRegister`]s are accessed on different
/// register pages which are switched via the `FUNC_CFG_ACCESS` register, this type keeps track of
/// which register page is currently selected. For each register access, the currently selected
/// register page is switched by updating `FUNC_CFG_ACCESS` if required.
///
/// Methods suffixed with `reg_address` or `reg_addresses` (e.g.
/// [`read_reg_addresses`](Self::read_reg_addresses)) directly access a register address without
/// taking care of selecting the appropriate register page. This may be useful if low-level
/// access is required, for example to give register access to an externally generated machine
/// learning core configuration. Note, however, that even in these cases the currently selected
/// register page is properly tracked because all register writes/updates are checked for
/// changing the `FUNC_CFG_ACCESS` register.
impl<'a, I2C> RegisterAccess<'a, I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: &'a Mutex<CriticalSectionRawMutex, RefCell<I2C>>, address: SlaveAddress) -> Self {
        Self {
            i2c,
            address,
            embedded_functions_registers_selected: false,
        }
    }

    pub fn destroy(self) -> &'a Mutex<CriticalSectionRawMutex, RefCell<I2C>> {
        self.i2c
    }

    /// Ensures that the right register page is selected in the FUNC_CFG_ACCESS register (which
    /// switches between normal and embedded functions registers) to access the given register
    /// afterwards. The register address for the subsequent register address is returned on
    /// success.
    fn select_register_page(&mut self, reg: impl Into<Register>) -> Result<u8, Error> {
        match reg.into() {
            Register::Primary(reg) => {
                if self.embedded_functions_registers_selected {
                    self.update_reg_address(PrimaryRegister::FUNC_CFG_ACCESS as u8, 0x00, 0x80)?;
                    self.embedded_functions_registers_selected = false;
                }
                Ok(reg as u8)
            }
            Register::EmbeddedFunctions(reg) => {
                if !self.embedded_functions_registers_selected {
                    self.update_reg_address(PrimaryRegister::FUNC_CFG_ACCESS as u8, 0x80, 0x80)?;
                    self.embedded_functions_registers_selected = true;
                }
                Ok(reg as u8)
            }
        }
    }

    fn check_register_page_change(&mut self, reg_address: u8, value: u8) {
        if reg_address == PrimaryRegister::FUNC_CFG_ACCESS as u8 {
            self.embedded_functions_registers_selected = value & 0x80 != 0;
        }
    }

    pub fn read_reg(&mut self, reg: impl Into<Register>) -> Result<u8, Error> {
        let reg_address = self.select_register_page(reg)?;
        let mut tbuf = [1; 1];
        self.read_reg_addresses(reg_address, &mut tbuf)?;
        Ok(tbuf[0])
    }

    pub fn read_regs(&mut self, reg: impl Into<Register>, buf: &mut [u8]) -> Result<(), Error> {
        let reg_address = self.select_register_page(reg)?;
        self.read_reg_addresses(reg_address, buf)
    }

    /// Directly read from register address
    ///
    /// This method does not take care of handling the `FUNC_CFG_ACCESS` register. Use
    /// [`read_reg`](Self::read_reg) or [`read_regs`](Self::read_regs) instead if reasonable.
    ///
    /// For example, using this method directly may be useful when interfacing with C example code.
    pub fn read_reg_addresses(&mut self, reg_address: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.i2c.lock(|inner| {
            inner.borrow_mut().write_read(self.address as u8, &[reg_address], buf)
            .map_err(|_| Error::I2cReadError)
        })?;
        Ok(())
    }

    pub fn write_reg(&mut self, reg: impl Into<Register>, data: u8) -> Result<(), Error> {
        let reg_address = self.select_register_page(reg)?;
        let update = [reg_address, data];
        self.check_register_page_change(reg_address, data);
        self.i2c.lock(|inner| {
            inner.borrow_mut().write(self.address as u8, &update)
            .map_err(|_| Error::I2cWriteError)
        })
    }

    pub fn write_regs(&mut self, reg: impl Into<Register>, data: &[u8]) -> Result<(), Error> {
        let reg_address = self.select_register_page(reg)?;
        self.write_reg_addresses(reg_address, data)
    }

    /// Directly write to register address
    ///
    /// See documentation for [`read_reg_addressess`](Self::read_reg_addresses).
    pub fn write_reg_addresses(&mut self, reg_address: u8, data: &[u8]) -> Result<(), Error> {
        // We can address at most 128 registers (7 bit)
        if data.len() > 128 {
            return Err(Error::InvalidData);
        }
        let mut update = [0; 129];
        update[0] = reg_address;
        for (i, byte) in data.iter().enumerate() {
            update[1 + i] = *byte;
            // Checking each byte may be inefficient, but it is definitely simple and probably
            // correct
            self.check_register_page_change(reg_address + i as u8, *byte);
        }
        self.i2c.lock(|inner| {
            inner.borrow_mut().write(self.address as u8, &update[..data.len() + 1])
            .map_err(|_| Error::I2cWriteError)
        })
    }

    /// Updates the bits in register `reg` specified by `bitmask` with payload `data`.
    pub fn update_reg(
        &mut self,
        reg: impl Into<Register>,
        data: u8,
        bitmask: u8,
    ) -> Result<(), Error> {
        let reg_address = self.select_register_page(reg)?;
        defmt::info!("selected register page");
        self.update_reg_address(reg_address, data, bitmask)
    }

    /// Directly updates the bits at the given register address
    ///
    /// See documentation for [`read_reg_addresses`](Self::read_reg_addresses).
    pub fn update_reg_address(
        &mut self,
        reg_address: u8,
        data: u8,
        bitmask: u8,
    ) -> Result<(), Error> {
        // We have to do a read of the register first to keep the bits we don't want to touch.
        let mut buf = [0; 1];
        self.read_reg_addresses(reg_address, &mut buf)?;
        let mut val = buf[0];

        // First set `bitmask` bits to zero,
        val &= !bitmask;
        // then write our data to these bits.
        val |= data & bitmask;

        // A write takes the register address as first byte, the data as second.
        let update = [reg_address, val];
        self.check_register_page_change(reg_address, val);
        self.i2c.lock(|inner| {
            inner.borrow_mut().write(self.address as u8, &update)
            .map_err(|_| Error::I2cWriteError)
        })
    }
}

/// All registers which are accessible from the primary SPI/I2C/MIPI I3C interfaces. These consist
/// of the union of the normal configuration registers (see [`PrimaryRegister`]) and the
/// [`EmbeddedFunctionsRegister`]s.
#[derive(Clone, Copy)]
pub enum Register {
    Primary(PrimaryRegister),
    EmbeddedFunctions(EmbeddedFunctionsRegister),
}

impl From<PrimaryRegister> for Register {
    fn from(reg: PrimaryRegister) -> Self {
        Register::Primary(reg)
    }
}

impl From<EmbeddedFunctionsRegister> for Register {
    fn from(reg: EmbeddedFunctionsRegister) -> Self {
        Register::EmbeddedFunctions(reg)
    }
}

/// Registers which are accessible from the primary SPI/I2C/MIPI I3C interfaces.
///
/// **Note:** These Register names correspond to the normal function registers.
/// When embedded function access is enabled in `FUNC_CFG_ACCESS` these addresses correspond to different registers.
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
pub enum PrimaryRegister {
    FUNC_CFG_ACCESS = 0x01,
    PIN_CTRL = 0x02,
    S4S_TPH_L = 0x04,
    S4S_TPH_H = 0x05,
    S4S_RR = 0x06,
    FIFO_CTRL1 = 0x07,
    FIFO_CTRL2 = 0x08,
    FIFO_CTRL3 = 0x09,
    FIFO_CTRL4 = 0x0A,
    COUNTER_BDR_REG1 = 0x0B,
    COUNTER_BDR_REG2 = 0x0C,
    INT1_CTRL = 0x0D,
    INT2_CTRL = 0x0E,
    WHO_AM_I = 0x0F,
    CTRL1_XL = 0x10,
    CTRL2_G = 0x11,
    CTRL3_C = 0x12,
    CTRL4_C = 0x13,
    CTRL5_C = 0x14,
    CTRL6_C = 0x15,
    CTRL7_G = 0x16,
    CTRL8_XL = 0x17,
    CTRL9_XL = 0x18,
    CTRL10_C = 0x19,
    ALL_INT_SRC = 0x1A,
    WAKE_UP_SRC = 0x1B,
    TAP_SRC = 0x1C,
    D6D_SRC = 0x1D,
    STATUS_REG = 0x1E,
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,
    OUTX_L_A = 0x28,
    OUTX_H_A = 0x29,
    OUTY_L_A = 0x2A,
    OUTY_H_A = 0x2B,
    OUTZ_L_A = 0x2C,
    OUTZ_H_A = 0x2D,
    EMB_FUNC_STATUS_MAINPAGE = 0x35,
    FSM_STATUS_A_MAINPAGE = 0x36,
    FSM_STATUS_B_MAINPAGE = 0x37,
    MLC_STATUS_MAINPAGE = 0x38,
    STATUS_MASTER_MAINPAGE = 0x39,
    FIFO_STATUS1 = 0x3A,
    FIFO_STATUS2 = 0x3B,
    TIMESTAMP0 = 0x40,
    TIMESTAMP1 = 0x41,
    TIMESTAMP2 = 0x42,
    TIMESTAMP3 = 0x43,
    UI_STATUS_REG_OIS = 0x49,
    UI_OUTX_L_G_OIS = 0x4A,
    UI_OUTX_H_G_OIS = 0x4B,
    UI_OUTY_L_G_OIS = 0x4C,
    UI_OUTY_H_G_OIS = 0x4D,
    UI_OUTZ_L_G_OIS = 0x4E,
    UI_OUTZ_H_G_OIS = 0x4F,
    UI_OUTX_L_A_OIS = 0x50,
    UI_OUTX_H_A_OIS = 0x51,
    UI_OUTY_L_A_OIS = 0x52,
    UI_OUTY_H_A_OIS = 0x53,
    UI_OUTZ_L_A_OIS = 0x54,
    UI_OUTZ_H_A_OIS = 0x55,
    TAP_CFG0 = 0x56,
    TAP_CFG1 = 0x57,
    TAP_CFG2 = 0x58,
    TAP_THS_6D = 0x59,
    INT_DUR2 = 0x5A,
    WAKE_UP_THS = 0x5B,
    WAKE_UP_DUR = 0x5C,
    FREE_FALL = 0x5D,
    MD1_CFG = 0x5E,
    MD2_CFG = 0x5F,
    S4S_ST_CMD_CODE = 0x60,
    S4S_DT_REG = 0x61,
    I3C_BUS_AVB = 0x62,
    INTERNAL_FREQ_FINE = 0x63,
    UI_INT_OIS = 0x6F,
    UI_CTRL1_OIS = 0x70,
    UI_CTRL2_OIS = 0x71,
    UI_CTRL3_OIS = 0x72,
    X_OFS_USR = 0x73,
    Y_OFS_USR = 0x74,
    Z_OFS_USR = 0x75,
    FIFO_DATA_OUT_TAG = 0x78,
    FIFO_DATA_OUT_X_L = 0x79,
    FIFO_DATA_OUT_X_H = 0x7A,
    FIFO_DATA_OUT_Y_L = 0x7B,
    FIFO_DATA_OUT_Y_H = 0x7C,
    FIFO_DATA_OUT_Z_L = 0x7D,
    FIFO_DATA_OUT_Z_H = 0x7E,
}

/// Embedded Functions Registers
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
pub enum EmbeddedFunctionsRegister {
    PAGE_SEL = 0x02,
    EMB_FUNC_EN_A = 0x04,
    EMB_FUNC_EN_B = 0x05,
    PAGE_ADDRESS = 0x08,
    PAGE_VALUE = 0x09,
    EMB_FUNC_INT1 = 0x0A,
    FSM_INT1_A = 0x0B,
    FSM_INT1_B = 0x0C,
    MLC_INT1 = 0x0D,
    EMB_FUNC_INT2 = 0x0E,
    FSM_INT2_A = 0x0F,
    FSM_INT2_B = 0x10,
    MLC_INT2 = 0x11,
    EMB_FUNC_STATUS = 0x12,
    FSM_STATUS_A = 0x13,
    FSM_STATUS_B = 0x14,
    MLC_STATUS = 0x15,
    PAGE_RW = 0x17,
    EMB_FUNC_FIFO_CFG = 0x44,
    FSM_ENABLE_A = 0x46,
    FSM_ENABLE_B = 0x47,
    FSM_LONG_COUNTER_L = 0x48,
    FSM_LONG_COUNTER_H = 0x49,
    FSM_LONG_COUNTER_CLEAR = 0x4A,
    FSM_OUTS1 = 0x4C,
    FSM_OUTS2 = 0x4D,
    FSM_OUTS3 = 0x4E,
    FSM_OUTS4 = 0x4F,
    FSM_OUTS5 = 0x50,
    FSM_OUTS6 = 0x51,
    FSM_OUTS7 = 0x52,
    FSM_OUTS8 = 0x53,
    FSM_OUTS9 = 0x54,
    FSM_OUTS10 = 0x55,
    FSM_OUTS11 = 0x56,
    FSM_OUTS12 = 0x57,
    FSM_OUTS13 = 0x58,
    FSM_OUTS14 = 0x59,
    FSM_OUTS15 = 0x5A,
    FSM_OUTS16 = 0x5B,
    EMB_FUNC_ODR_CFG_B = 0x5F,
    EMB_FUNC_ODR_CFG_C = 0x60,
    STEP_COUNTER_L = 0x62,
    STEP_COUNTER_H = 0x63,
    EMB_FUNC_SRC = 0x64,
    EMB_FUNC_INIT_A = 0x66,
    EMB_FUNC_INIT_B = 0x67,
    MLC0_SRC = 0x70,
    MLC1_SRC = 0x71,
    MLC2_SRC = 0x72,
    MLC3_SRC = 0x73,
    MLC4_SRC = 0x74,
    MLC5_SRC = 0x75,
    MLC6_SRC = 0x76,
    MLC7_SRC = 0x77,
}

/// Commands Enum to specify Command structures
/// which set various register bits of the lsm.
///
/// A command can either be a command without parameters or with one or multiple parameters.
///
/// One command will write bits to its assigned register by specifying the register,
/// the mask and the bits with the impl over the enum.
/// Since some settings span over multiple registers,
/// the register function returns register addresses dependent on the command parameters.
#[derive(Clone, Copy)]
pub(crate) enum Command {
    SwReset,
    SetDataRateXl(DataRate),
    SetAccelScale(AccelerometerScale),
    SetDataRateG(DataRate),
    SetGyroScale(GyroscopeScale),
    TapEnable(TapCfg),
    TapThreshold(Axis, u8),
    TapDuration(u8),
    TapQuiet(u8),
    TapShock(u8),
    TapMode(TapMode),
    InterruptEnable(bool),
    MapInterrupt(InterruptLine, InterruptSource, bool),
}

impl Command {
    /// Returns the register address to write to for the specific command.
    pub(crate) fn register(&self) -> PrimaryRegister {
        match *self {
            Command::SwReset => PrimaryRegister::CTRL3_C,
            Command::SetDataRateXl(_) => PrimaryRegister::CTRL1_XL,
            Command::SetAccelScale(_) => PrimaryRegister::CTRL1_XL,
            Command::TapEnable(_) => PrimaryRegister::TAP_CFG0,
            Command::TapThreshold(Axis::X, _) => PrimaryRegister::TAP_CFG1,
            Command::TapThreshold(Axis::Y, _) => PrimaryRegister::TAP_CFG2,
            Command::TapThreshold(Axis::Z, _) => PrimaryRegister::TAP_THS_6D,
            Command::TapDuration(_) => PrimaryRegister::INT_DUR2,
            Command::TapQuiet(_) => PrimaryRegister::INT_DUR2,
            Command::TapShock(_) => PrimaryRegister::INT_DUR2,
            Command::TapMode(_) => PrimaryRegister::WAKE_UP_THS,
            Command::InterruptEnable(_) => PrimaryRegister::TAP_CFG2,
            Command::MapInterrupt(InterruptLine::INT1, _, _) => PrimaryRegister::MD1_CFG,
            Command::MapInterrupt(InterruptLine::INT2, _, _) => PrimaryRegister::MD2_CFG,
            Command::SetDataRateG(_) => PrimaryRegister::CTRL2_G,
            Command::SetGyroScale(_) => PrimaryRegister::CTRL2_G,
        }
    }
    /// Returns a byte containing data to be written by the command.
    ///
    /// For booleans this is mostly done by converting and shifting to the corresponding position.
    pub(crate) fn bits(&self) -> u8 {
        match *self {
            Command::SwReset => 0x01,
            Command::SetDataRateXl(dr) => dr as u8,
            Command::SetAccelScale(fs) => fs as u8,
            Command::TapEnable(cfg) => {
                (cfg.en_x_y_z.0 as u8) << 3
                    | (cfg.en_x_y_z.1 as u8) << 2
                    | (cfg.en_x_y_z.2 as u8) << 1
                    | (cfg.lir as u8)
            }
            Command::TapThreshold(_, value) => value,
            Command::TapDuration(value) => value << 4,
            Command::TapQuiet(value) => value << 2,
            Command::TapShock(value) => value,
            Command::TapMode(TapMode::Single) => 0 << 7,
            Command::TapMode(TapMode::SingleAndDouble) => 1 << 7,
            Command::InterruptEnable(en) => (en as u8) << 7,
            Command::MapInterrupt(_, InterruptSource::SleepChange, en) => (en as u8) << 7,
            Command::MapInterrupt(_, InterruptSource::SingleTap, en) => (en as u8) << 6,
            Command::MapInterrupt(_, InterruptSource::WakeUp, en) => (en as u8) << 5,
            Command::MapInterrupt(_, InterruptSource::FreeFall, en) => (en as u8) << 4,
            Command::MapInterrupt(_, InterruptSource::DoubleTap, en) => (en as u8) << 3,
            Command::MapInterrupt(_, InterruptSource::D6d, en) => (en as u8) << 2,
            Command::MapInterrupt(_, InterruptSource::EmbeddedFunctions, en) => (en as u8) << 1,
            Command::MapInterrupt(InterruptLine::INT1, InterruptSource::SHUB, en) => en as u8,
            Command::MapInterrupt(InterruptLine::INT2, InterruptSource::Timestamp, en) => en as u8,
            Command::MapInterrupt(InterruptLine::INT2, InterruptSource::SHUB, _) => 0,
            Command::MapInterrupt(InterruptLine::INT1, InterruptSource::Timestamp, _) => 0,
            Command::SetDataRateG(dr) => dr as u8,
            Command::SetGyroScale(fs) => fs as u8,
        }
    }
    /// Returns the bit mask for the specified Command.
    pub(crate) fn mask(&self) -> u8 {
        match *self {
            Command::SwReset => 0x01,
            Command::SetDataRateXl(_) => 0xF0,
            Command::SetAccelScale(_) => 0b0000_1100,
            Command::TapEnable(_) => 0b0000_1111,
            Command::TapThreshold(_, _) => 0b0001_1111,
            Command::TapDuration(_) => 0xF0,
            Command::TapQuiet(_) => 0x0C,
            Command::TapShock(_) => 0x03,
            Command::TapMode(_) => 0x80,
            Command::InterruptEnable(_) => 0x80,
            Command::MapInterrupt(_, InterruptSource::SleepChange, _) => 0b1000_0000,
            Command::MapInterrupt(_, InterruptSource::SingleTap, _) => 0b0100_0000,
            Command::MapInterrupt(_, InterruptSource::WakeUp, _) => 0b0010_0000,
            Command::MapInterrupt(_, InterruptSource::FreeFall, _) => 0b0001_0000,
            Command::MapInterrupt(_, InterruptSource::DoubleTap, _) => 0b0000_1000,
            Command::MapInterrupt(_, InterruptSource::D6d, _) => 0b0000_0100,
            Command::MapInterrupt(_, InterruptSource::EmbeddedFunctions, _) => 0b0000_0010,
            Command::MapInterrupt(InterruptLine::INT1, InterruptSource::SHUB, _) => 0b0000_0001,
            Command::MapInterrupt(InterruptLine::INT2, InterruptSource::Timestamp, _) => {
                0b0000_0001
            }
            Command::MapInterrupt(InterruptLine::INT2, InterruptSource::SHUB, _) => 0b0000_0000,
            Command::MapInterrupt(InterruptLine::INT1, InterruptSource::Timestamp, _) => {
                0b0000_0000
            }
            Command::SetDataRateG(_) => 0xF0,
            Command::SetGyroScale(_) => 0b0000_1110,
        }
    }
}
