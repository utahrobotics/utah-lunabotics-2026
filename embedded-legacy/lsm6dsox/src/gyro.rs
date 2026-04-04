// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

use super::*;
use measurements::AngularVelocity;

impl<I2C, Delay> Lsm6dsox<'_, I2C, Delay>
where
    I2C: I2c,
    Delay: DelayNs,
{
    /// Sets the measurement output rate.
    ///
    /// Note: [DataRate::Freq1Hz6] is not supported by the gyroscope and will yield an [Error::InvalidData].
    pub fn set_gyro_sample_rate(&mut self, data_rate: DataRate) -> Result<(), Error> {
        match data_rate {
            DataRate::Freq1Hz6 => Err(Error::NotSupported),
            _ => {
                self.update_reg_command(Command::SetDataRateG(data_rate))?;
                self.config.g_odr = data_rate;
                Ok(())
            }
        }
    }

    /// Sets the gyroscope measurement range.
    ///
    /// Values up to this scale will be reported correctly.
    pub fn set_gyro_scale(&mut self, scale: GyroscopeScale) -> Result<(), Error> {
        self.update_reg_command(Command::SetGyroScale(scale))?;
        self.config.g_scale = scale;
        Ok(())
    }

    /// Get a angular rate reading.
    ///
    /// If no data is ready returns the appropriate [Error].
    pub fn angular_rate(&mut self) -> Result<AngularRate, Error> {
        let data_rdy = self.registers.read_reg(PrimaryRegister::STATUS_REG)?;
        if (data_rdy & 0b0000_0010) == 0 {
            // bit 2 of STATUS_REG indicates if gyro data is available
            Err(Error::NoDataReady)
        } else {
            let mut data_raw: [u8; 6] = [0; 6]; // All 3 axes x, y, z i16 values, decoded little endian, 2nd Complement
            self.registers
                .read_regs(PrimaryRegister::OUTX_L_G, &mut data_raw)?;
            let mut data: [f32; 3] = [0.0; 3];
            let factor = self.config.g_scale.to_factor();
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * factor;
            }

            Ok(AngularRate {
                x: AngularVelocity::from_hertz((data[0] / 360.0).into()),
                y: AngularVelocity::from_hertz((data[1] / 360.0).into()),
                z: AngularVelocity::from_hertz((data[2] / 360.0).into()),
            })
        }
    }

    /// Get a *raw* angular rate reading.
    ///
    /// If no data is ready returns the appropriate [Error].
    pub fn angular_rate_raw(&mut self) -> Result<RawAngularRate, Error> {
        let data_rdy = self.registers.read_reg(PrimaryRegister::STATUS_REG)?;
        if (data_rdy & 0b0000_0010) == 0 {
            // bit 2 of STATUS_REG indicates if gyro data is available
            Err(Error::NoDataReady)
        } else {
            let mut data_raw: [u8; 6] = [0; 6]; // All 3 axes x, y, z i16 values, decoded little endian, 2nd Complement
            self.registers
                .read_regs(PrimaryRegister::OUTX_L_G, &mut data_raw)?;
            let mut data: [i16; 3] = [0; 3];
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]);
            }

            Ok(RawAngularRate {
                x: data[0],
                y: data[1],
                z: data[2],
            })
        }
    }
}
