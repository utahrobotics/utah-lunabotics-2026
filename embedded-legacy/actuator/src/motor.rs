use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PWM_SLICE0, PWM_SLICE1, PWM_SLICE2, PWM_SLICE3, PWM_SLICE4, PWM_SLICE5, PWM_SLICE6, PWM_SLICE7};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, PwmError, SetDutyCycle};
use defmt::{error, info, warn};
use embedded_common::ActuatorCommand;
use embedded_common::Actuator;
use embedded_common::Direction;

pub struct Motor<'d> {
    // m1_slp (active=high)
    sleep: Output<'d>,

    // 0 forward, 1 backward
    dir: Output<'d>,

    // speed
    pwm: Pwm<'d>,
}

impl<'d> Motor<'d> {
    pub fn new_m2(
        sleep_pin: embassy_rp::peripherals::PIN_17,
        dir_pin: embassy_rp::peripherals::PIN_14,
        pwm_pin: embassy_rp::peripherals::PIN_16,
        pwm_slice: PWM_SLICE0,
    ) -> Self {
        let sleep = Output::new(sleep_pin, Level::High);
        let dir = Output::new(dir_pin, Level::Low);

        let pwm = Pwm::new_output_a(pwm_slice, pwm_pin, PwmConfig::default());

        Motor { sleep, dir, pwm }
    }

    pub fn new_m1(
        sleep_pin: embassy_rp::peripherals::PIN_10,
        dir_pin: embassy_rp::peripherals::PIN_15,
        pwm_pin: embassy_rp::peripherals::PIN_9,
        pwm_slice: PWM_SLICE4,
    ) -> Self {
        let sleep = Output::new(sleep_pin, Level::High);
        let dir = Output::new(dir_pin, Level::Low);

        let pwm = Pwm::new_output_b(pwm_slice, pwm_pin, PwmConfig::default());

        Motor { sleep, dir, pwm }
    }

    /// set motor speed
    pub fn set_speed(&mut self, speed: u16) -> Result<(), PwmError> {
        let max_duty = self.pwm.max_duty_cycle();
        if speed <= max_duty {
            self.pwm.set_duty_cycle(speed)?;
            // info!("Motor speed set to: speed: {}, max duty: {}", speed, max_duty);
        } else {
            warn!(
                "Error: Speed {} must be between 0 and {}",
                speed, max_duty
            );
        }
        Ok(())
    }

    /// set direction
    pub fn set_direction(&mut self, direction: Direction) {
        match direction {
            Direction::Forward => {
                self.dir.set_low();
                // info!("Motor direction set to forward");
            }
            Direction::Backward => {
                self.dir.set_high();
                // info!("Motor direction set to backward");
            }
        }
    }

    pub fn enable(&mut self) {
        self.sleep.set_high();
        info!("Motor enabled");
    }

    pub fn disable(&mut self) {
        self.sleep.set_low();
        info!("Motor disabled");
    }

    pub fn get_max_duty(&self) -> u16 {
        self.pwm.max_duty_cycle()
    }
}