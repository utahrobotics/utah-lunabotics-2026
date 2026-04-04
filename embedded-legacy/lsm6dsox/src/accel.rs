// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

use core::convert::TryFrom;

use super::*;

impl<I2C, Delay> Accelerometer for Lsm6dsox<'_, I2C, Delay>
where
    I2C: I2c,
    Delay: DelayNs,
{
    type Error = Error;

    fn accel_norm(
        &mut self,
    ) -> Result<accelerometer::vector::F32x3, accelerometer::Error<Self::Error>> {
        // First read the status register to determine if data is available,
        // if so, read the six bytes of data and convert it.
        let data_rdy = self.registers.read_reg(PrimaryRegister::STATUS_REG)?;
        if (data_rdy & 0b0000_0001) == 0 {
            // check if XLDA bit is set in STATUS_REG
            Err(accelerometer::Error::new_with_cause(
                accelerometer::ErrorKind::Mode,
                Error::NoDataReady,
            ))
        } else {
            let mut data_raw: [u8; 6] = [0; 6]; // All 3 axes x, y, z i16 values, decoded little endian, 2nd Complement
            self.registers
                .read_regs(PrimaryRegister::OUTX_L_A, &mut data_raw)?;
            let mut data: [f32; 3] = [0.0; 3];
            let factor = self.config.xl_scale.to_factor();
            // Now convert the raw i16 values to f32 engineering units
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2]) as f32 * factor
            }

            Ok(F32x3::new(data[0], data[1], data[2]))
        }
    }

    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<Self::Error>> {
        // Read the sample rate first, update the config if necessary and then report the current rate.
        // Since the user shouldn't have to update the config manually,
        // this is done here in case the config differs from the device state (which it shouldn't, but who knows).
        // It may be better to only query the config here if this function is called often.
        let sample_rate_raw = self.registers.read_reg(PrimaryRegister::CTRL1_XL)?;
        let sample_rate = DataRate::try_from(sample_rate_raw & 0xF0)
            .map_err(|_| accelerometer::Error::new(accelerometer::ErrorKind::Device))?;
        self.config.xl_odr = sample_rate;
        Ok(sample_rate.into())
    }
}

impl<I2C, Delay> RawAccelerometer<I16x3> for Lsm6dsox<'_, I2C, Delay>
where
    I2C: I2c,
    Delay: DelayNs,
{
    type Error = Error;

    fn accel_raw(
        &mut self,
    ) -> Result<accelerometer::vector::I16x3, accelerometer::Error<Self::Error>> {
        // First read the status register to determine if data is available,
        // if so, read the six bytes of data and convert it.
        let data_rdy = self.registers.read_reg(PrimaryRegister::STATUS_REG)?;
        if (data_rdy & 0b0000_0001) == 0 {
            // check if XLDA bit is set in STATUS_REG
            Err(accelerometer::Error::new_with_cause(
                accelerometer::ErrorKind::Mode,
                Error::NoDataReady,
            ))
        } else {
            let mut data_raw: [u8; 6] = [0; 6]; // All 3 axes x, y, z i16 values, decoded little endian, 2nd Complement
            self.registers
                .read_regs(PrimaryRegister::OUTX_L_A, &mut data_raw)?;
            let mut data: [i16; 3] = [0; 3];
            // Now convert the raw i16 values to f32 engineering units
            for i in 0..3 {
                data[i] = LittleEndian::read_i16(&data_raw[i * 2..i * 2 + 2])
            }

            Ok(I16x3::new(data[0], data[1], data[2]))
        }
    }
}

impl<I2C, Delay> Lsm6dsox<'_, I2C, Delay>
where
    I2C: I2c,
    Delay: DelayNs,
{
    /// Sets the measurement output rate.
    pub fn set_accel_sample_rate(&mut self, data_rate: DataRate) -> Result<(), Error> {
        self.update_reg_command(Command::SetDataRateXl(data_rate))?;
        self.config.xl_odr = data_rate;
        Ok(())
    }

    /// Sets the acceleration measurement range.
    ///
    /// Values up to this scale will be reported correctly.
    pub fn set_accel_scale(&mut self, scale: AccelerometerScale) -> Result<(), Error> {
        self.update_reg_command(Command::SetAccelScale(scale))?;
        self.config.xl_scale = scale;
        Ok(())
    }

    /// Sets up double-tap recognition and enables Interrupts on INT2 pin.
    ///
    /// Configures everything necessary to reasonable defaults.
    /// This includes setting the accelerometer scale to 2G, configuring power modes, setting values for thresholds
    /// and optionally mapping a interrupt pin, maps only single-tap or double-tap to the pin.
    pub fn setup_tap_detection(
        &mut self,
        tap_cfg: TapCfg,
        tap_mode: TapMode,
        int_line: Option<InterruptLine>,
    ) -> Result<(), Error> {
        /* Set Output Data Rate */
        self.update_reg_command(Command::SetDataRateXl(DataRate::Freq104Hz))?;
        // Output data rate and full scale could be set with one register update, maybe change this
        /* Set full scale */
        self.update_reg_command(Command::SetAccelScale(AccelerometerScale::Accel2g))?;
        self.config.xl_scale = AccelerometerScale::Accel2g;

        /*
        Set XL_ULP_EN = 1 and XL_HM_MODE = 0 = high-performance mode
        Refer to application note table 8 and 9 for further information on operating modes.
        */
        self.registers
            .update_reg(PrimaryRegister::CTRL5_C, 0b1000_0000, 0b1000_0000)?;
        self.registers
            .update_reg(PrimaryRegister::CTRL6_C, 0b0000_0000, 0b0000_0000)?;

        /* Enable tap detection */
        self.update_reg_command(Command::TapEnable(tap_cfg))?;

        /* Set tap threshold 0x08 = 500mg for configured FS_XL*/
        self.update_reg_command(Command::TapThreshold(Axis::X, 0x08))?;
        self.update_reg_command(Command::TapThreshold(Axis::Y, 0x08))?;
        self.update_reg_command(Command::TapThreshold(Axis::Z, 0x08))?;

        self.update_reg_command(Command::TapDuration(0x07))?;
        self.update_reg_command(Command::TapQuiet(0x03))?;
        self.update_reg_command(Command::TapShock(0x03))?;

        self.update_reg_command(Command::TapMode(tap_mode))?;

        /* Enable Interrupts */
        if let Some(int_line) = int_line {
            self.update_reg_command(Command::InterruptEnable(true))?; // This must always be enabled
            match (tap_mode, int_line) {
                (TapMode::Single, line) => self.update_reg_command(Command::MapInterrupt(
                    line,
                    InterruptSource::SingleTap,
                    true,
                ))?,
                (TapMode::SingleAndDouble, line) => self.update_reg_command(
                    Command::MapInterrupt(line, InterruptSource::DoubleTap, true),
                )?,
            }
        }
        Ok(())
    }

    /// Sets up Significant Motion Detection, routs interrupts to INT1 pin
    pub fn setup_smd(&mut self) -> Result<(), Error> {
        // first enable significant motion detection
        self.registers.update_reg(
            EmbeddedFunctionsRegister::EMB_FUNC_EN_A,
            0b00100000,
            0b00100000,
        )?;

        // significant motion detection routed to INT1 pin
        self.registers
            .update_reg(EmbeddedFunctionsRegister::EMB_FUNC_INT1, 1, 0b00100000)?;

        // enable latched interrupt mode
        self.registers
            .update_reg(EmbeddedFunctionsRegister::PAGE_RW, 0b10000000, 0b10000000)?;

        // enable embedded functions interrupt router
        self.registers
            .update_reg(PrimaryRegister::MD1_CFG, 1, 0b00000010)?;

        self.set_accel_scale(AccelerometerScale::Accel2g)?;
        self.set_accel_sample_rate(DataRate::Freq26Hz)?;
        Ok(())
    }

    pub fn check_smd(&mut self) -> Result<bool, Error> {
        let status = self
            .registers
            .read_reg(EmbeddedFunctionsRegister::EMB_FUNC_STATUS)?;

        // check if there is an interrupt status bit set
        return Ok((status & 0b00100000) > 0);
    }

    /// Checks the tap source register.
    ///
    /// - The Register will be cleared according to the LIR setting.
    /// - The interrupt flag will be cleared after this check, or according to the LIR mode.
    /// - If LIR is set to `False` the interrupt will be set for the quiet-time window and clears automatically after that.
    pub fn check_tap(&mut self) -> Result<BitFlags<TapSource>, Error> {
        let buf = self.registers.read_reg(PrimaryRegister::TAP_SRC)?;

        BitFlags::from_bits(buf).map_err(|_| Error::InvalidData)
    }

    /// Sets the tap Threshold for each individual axis.
    ///
    /// [...] [These registers] are used to select the unsigned threshold value used to detect
    /// the tap event on the respective axis. The value of 1 LSB of these 5 bits depends on the selected accelerometer
    /// full scale: 1 LSB = (FS_XL)/(2⁵). The unsigned threshold is applied to both positive and negative slope data.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    pub fn set_tap_threshold(&mut self, x: u8, y: u8, z: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapThreshold(Axis::X, x))?;
        self.update_reg_command(Command::TapThreshold(Axis::Y, y))?;
        self.update_reg_command(Command::TapThreshold(Axis::Z, z))
    }

    /// Sets the duration of maximum time gap for double tap recognition. Default value: `0b0000`
    ///
    /// In the double-tap case, the Duration time window defines the maximum time between two consecutive detected
    /// taps. The Duration time period starts just after the completion of the Quiet time of the first tap. The `DUR[3:0]` bits
    /// of the `INT_DUR2` register are used to set the Duration time window value: the default value of these bits is `0000b`
    /// and corresponds to `16/ODR_XL` time, where `ODR_XL` is the accelerometer output data rate. If the `DUR[3:0]` bits
    /// are set to a different value, 1 LSB corresponds to `32/ODR_XL` time.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    pub fn set_tap_duration(&mut self, dur: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapDuration(dur))
    }

    /// Sets the expected quiet time after a tap detection. Default value: `0b00`
    ///
    /// In the double-tap case, the Quiet time window defines the time after the first tap recognition in which there must
    /// not be any overcoming threshold event. When latched mode is disabled (`LIR` bit of `TAP_CFG` is set to 0), the
    /// Quiet time also defines the length of the interrupt pulse (in both single and double-tap case).[^note]
    ///
    /// The `QUIET[1:0]` bits of the `INT_DUR2` register are used to set the Quiet time window value:
    /// the default value of these bits is `00b` and corresponds to `2/ODR_XL` time, where `ODR_XL` is the accelerometer output data rate.
    /// If the `QUIET[1:0]` bits are set to a different value, 1 LSB corresponds to `4/ODR_XL` time.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    pub fn set_tap_quiet(&mut self, quiet: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapQuiet(quiet))
    }

    /// Sets the maximum duration of the over-threshold event. Default value: `0b00`
    ///
    /// The Shock time window defines the maximum duration of the overcoming threshold event: the acceleration must
    /// return below the threshold before the Shock window has expired, otherwise the tap event is not detected. The
    /// `SHOCK[1:0]` bits of the `INT_DUR2` register are used to set the Shock time window value: the default value of
    /// these bits is 00b and corresponds to `4/ODR_XL` time, where `ODR_XL` is the accelerometer output data rate. If the
    /// `SHOCK[1:0]` bits are set to a different value, 1 LSB corresponds to `8/ODR_XL` time.[^note]
    ///
    /// [^note]: Definition from the LSM6DSOX Application Note
    pub fn set_tap_shock(&mut self, shock: u8) -> Result<(), Error> {
        self.update_reg_command(Command::TapShock(shock))
    }
}
