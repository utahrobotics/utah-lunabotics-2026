#[cfg(feature = "production")]
mod prod_impl {
    use std::sync::Arc;
    use std::time::Duration;

    use crate::utils::CobsCodec;
    use crate::utils::udev_poll;
    use crossbeam::queue::ArrayQueue;
    use crossbeam_channel::Receiver;
    use cu29::prelude::*;
    use embedded_common::ActuatorCommand;
    use embedded_common::FromPicoV3;
    use embedded_common::IMU_READING_DELAY_MS;
    use futures_util::StreamExt;
    use tasker::get_tokio_handle;
    use tasker::tokio;
    use tasker::tokio::io::AsyncWriteExt;
    use tasker::tokio::io::BufStream;
    use tasker::tokio::io::WriteHalf;
    use tasker::tokio::sync::watch;
    use tasker::tokio::time::timeout;
    use tokio_serial::SerialPortBuilderExt;
    use tokio_serial::SerialStream;
    use tokio_util::codec::FramedRead;
    use udev::EventType;
    use udev::MonitorBuilder;
    use udev::Udev;

    pub struct V3PicoTask {
        /// when a v3 pico is connected, it's serial port will be available on this rx
        path_rx: Receiver<String>,
        is_broken: Option<tokio::sync::watch::Receiver<bool>>,
        serial_port_writer: Option<WriteHalf<BufStream<SerialStream>>>,

        /// Message queue read from the pico
        from_pico: Arc<&'static ArrayQueue<FromPicoV3>>,
    }

    impl Freezable for V3PicoTask {}

    impl CuTask for V3PicoTask {
        // input is the actuator command as bytes
        // ActuatorCommand doesn't implement Serialize or Decode due to being no_std
        // the reason this is a tuple is a hack to get around the fact that copper doesnt support one task having multiple outputs in the same way it does multiple inputs
        type Input<'m> = input_msg!((
            Option<common::Steering>, // ignore steering in this task
            Option<embedded_common::ActuatorCommand>
        ));
        // output is the FromPicoV3 struct serialized as bytes
        type Output<'m> = output_msg!(FromPicoV3);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let (path_tx, path_rx) = crossbeam_channel::bounded(1);
            std::thread::spawn(move || {
                let mut monitor = match MonitorBuilder::new() {
                    Ok(x) => x,
                    Err(e) => {
                        error!("Failed to create udev monitor: {}", e.to_string());
                        return;
                    }
                };
                monitor = match monitor.match_subsystem("tty") {
                    Ok(x) => x,
                    Err(e) => {
                        error!("Failed to set match-subsystem filter: {}", e.to_string());
                        return;
                    }
                };
                let listener = match monitor.listen() {
                    Ok(x) => x,
                    Err(e) => {
                        error!("Failed to listen for udev events: {}", e.to_string());
                        return;
                    }
                };

                let mut enumerator = {
                    let udev = match Udev::new() {
                        Ok(x) => x,
                        Err(e) => {
                            error!("Failed to create udev context: {}", e.to_string());
                            return;
                        }
                    };
                    match udev::Enumerator::with_udev(udev) {
                        Ok(x) => x,
                        Err(e) => {
                            error!("Failed to create udev enumerator: {}", e.to_string());
                            return;
                        }
                    }
                };
                if let Err(e) = enumerator.match_subsystem("tty") {
                    error!("Failed to set match-subsystem filter: {}", e.to_string());
                }
                let devices = match enumerator.scan_devices() {
                    Ok(x) => x,
                    Err(e) => {
                        error!("Failed to scan devices: {}", e.to_string());
                        return;
                    }
                };

                // infinite iterator
                devices
                    .into_iter()
                    .chain(
                        udev_poll(listener)
                            .filter(|event| event.event_type() == EventType::Add)
                            .map(|event| event.device()),
                    )
                    .for_each(|device| {
                        let Some(path) = device.devnode() else {
                            return;
                        };
                        let Some(path_str) = path.to_str() else {
                            return;
                        };
                        let Some(serial_cstr) = device.property_value("ID_SERIAL") else {
                            return;
                        };
                        let Some(serial) = serial_cstr.to_str() else {
                            warning!("Failed to parse serial of device {}", path_str);
                            return;
                        };
                        match serial.strip_prefix("USR_V3PICO_") {
                            Some(_) => {
                                if path_tx.send(path.to_string_lossy().to_string()).is_err() {
                                    warning!("Couldn't send controller path");
                                }
                            }
                            None if serial == "USR_V3PICO" => {
                                warning!(
                                    "Actuator controller at path {} has no serial number",
                                    path
                                );
                            }
                            None => {} // Device doesn't match, silently ignore
                        }
                    })
            });
            let from_pico = Arc::new(&*Box::leak(Box::new(ArrayQueue::new(50))));
            Ok(Self {
                path_rx,
                is_broken: None,
                from_pico,
                serial_port_writer: None,
            })
        }

        /// pre process checks if self.path_rx has a value queued up, and if so opens the serial port, splits it
        /// into a read half and a write half, then sets self.serial_port_writer and spawns a reader async task that will
        /// produce messages from the pico
        fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
            let Ok(path_str) = self.path_rx.try_recv() else {
                return Ok(());
            };

            let port = get_tokio_handle().block_on(async {
                let port = match tokio_serial::new(&path_str, 150000)
                    .flow_control(tokio_serial::FlowControl::Hardware)
                    .open_native_async()
                {
                    Ok(mut x) => {
                        if let Err(e) = x.set_exclusive(true) {
                            warning!(
                                "Failed to set V3Pico controller port {} exclusive: {}",
                                path_str,
                                e.to_string()
                            );
                        }
                        Some(x)
                    }
                    Err(e) => {
                        eprintln!(
                            "Failed to open V3Pico controller port {}: {}",
                            path_str,
                            e.to_string()
                        );
                        None
                    }
                };

                port
            });
            if port.is_none() {
                return Err(CuError::new_with_cause(
                    "fialed to open port",
                    std::io::Error::other("failed to open port"),
                ));
            }

            let port = port.unwrap();

            let port = BufStream::new(port);
            let (reader, writer): (
                tokio::io::ReadHalf<BufStream<tokio_serial::SerialStream>>,
                tokio::io::WriteHalf<BufStream<tokio_serial::SerialStream>>,
            ) = tokio::io::split(port);
            let reader: FramedRead<
                tokio::io::ReadHalf<BufStream<tokio_serial::SerialStream>>,
                CobsCodec,
            > = FramedRead::new(reader, CobsCodec {});
            let (is_broken_tx, is_broken_rx) = watch::channel(false);
            spawn_reader_thread(reader, is_broken_tx, self.from_pico.clone());
            self.is_broken = Some(is_broken_rx);
            self.serial_port_writer = Some(writer);
            Ok(())
        }

        /// If there is an actuator command available, and self.serial_port_writer is set, then
        /// the actuator command is written to the serial port.
        /// Messages from the pico are popped off the queue and sent to the downstream task.
        fn process<'i, 'o>(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'i>,
            output: &mut Self::Output<'o>,
        ) -> CuResult<()> {
            if let Some((_, Some(actuator_cmd))) = input.payload()
                && let Some(ref mut writer) = self.serial_port_writer
            {
                let _ = get_tokio_handle().block_on(async {
                    if let Err(e) = writer
                        .write(&ActuatorCommand::serialize(&actuator_cmd))
                        .await
                    {
                        return Err(CuError::new_with_cause("failed to write to serial port", e));
                    }
                    if let Err(e) = writer.flush().await {
                        return Err(CuError::new_with_cause("failed to flush to serial port", e));
                    }
                    return Ok(());
                })?;
            }
            if let Some(reading) = self.from_pico.pop() {
                output.set_payload(reading);
            } else {
                output.clear_payload();
            }

            Ok(())
        }

        /// If the reader async task reports an error, return it.

        fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
            if let Some(ref is_broken) = self.is_broken
                && *is_broken.borrow()
            {
                error!("Pico broken");
                return Err(CuError::new_with_cause(
                    "Error reading form pico",
                    std::io::Error::other("is broken signal received"),
                ));
            }

            Ok(())
        }
    }

    /// continually reads messages from the pico, pushing to from_pico queue
    fn spawn_reader_thread(
        mut reader: FramedRead<
            tokio::io::ReadHalf<BufStream<tokio_serial::SerialStream>>,
            CobsCodec,
        >,
        is_broken_tx: tokio::sync::watch::Sender<bool>,
        from_pico: Arc<&'static ArrayQueue<FromPicoV3>>,
    ) -> tokio::task::JoinHandle<()> {
        get_tokio_handle().spawn(async move {
            let mut no_reading_count = 0;
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(IMU_READING_DELAY_MS - 1))
                    .await;
                let Ok(reading) = timeout(Duration::from_millis(200), reader.next()).await else {
                    error!("Pico has become unresponsive.");
                    let _ = is_broken_tx.send(true);
                    break;
                };
                if let Some(Err(e)) = reading {
                    let _ = is_broken_tx.send(true);
                    error!("failed to read from pico: {}", e.to_string());
                    break;
                }
                if let None = reading {
                    no_reading_count += 1;
                    if no_reading_count <= 5 {
                        let _ = is_broken_tx.send(true);
                        error!("Pico has become unresponsive. (no reading count)");
                        break;
                    }
                    continue;
                }
                let reading = reading.unwrap().unwrap();
                let Ok(reading) = reading.try_into() else {
                    warning!("not 105 bytes");
                    continue;
                };
                let Ok(reading) = FromPicoV3::deserialize(reading) else {
                    error!("Failed to deserialize message from picov3 serial port");
                    let _ = is_broken_tx.send(true);
                    match powercycle_ioctl() {
                        Ok(_) => {}
                        Err(e) => {
                            error!("ioctl failed: {}", e.to_string());
                        }
                    }
                    break;
                };
                if let Err(_) = from_pico.push(reading) {
                    error!("From Pico queue full, dropping reading");
                    is_broken_tx.send(true).unwrap();
                    break;
                }
            }
        })
    }

    /// install this binary from misc/usb-reset
    fn powercycle_ioctl() -> Result<(), std::io::Error> {
        let _ = std::process::Command::new("usb-reset").spawn()?;
        std::thread::sleep(Duration::from_secs_f32(0.02));
        Ok(())
    }
}

#[cfg(feature = "production")]
pub use prod_impl::*;

#[cfg(not(feature = "production"))]
pub use resim_impl::*;

#[cfg(not(feature = "production"))]
mod resim_impl {
    use cu29::{cutask::CuTask, prelude::*};
    use embedded_common::FromPicoV3;

    pub struct V3PicoTask {}
    impl Freezable for V3PicoTask {}
    impl CuTask for V3PicoTask {
        // input is the actuator command as bytes
        // ActuatorCommand doesn't implement Serialize or Decode due to being no_std
        // the reason this is a tuple is a hack to get around the fact that copper doesnt support one task having multiple outputs in the same way it does multiple inputs
        type Input<'m> = input_msg!((
            Option<common::Steering>, // ignore steering in this task
            Option<embedded_common::ActuatorCommand>
        ));
        // output is the FromPicoV3 struct serialized as bytes
        type Output<'m> = output_msg!(FromPicoV3);
        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process<'i, 'o>(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'i>,
            output: &mut Self::Output<'o>,
        ) -> CuResult<()> {
            Ok(())
        }
    }
}
