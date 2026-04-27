#[cfg(feature = "production")]
mod prod_impl {
    use std::ops::Deref as _;
    use std::sync::Arc;
    use std::sync::Mutex;
    use std::sync::OnceLock;
    use std::time::Duration;

    use crate::utils::CobsCodec;
    use crate::utils::udev_poll;
    use crossbeam::atomic::AtomicCell;
    use crossbeam::queue::ArrayQueue;
    use crossbeam_channel::Receiver;
    use cu29::prelude::*;
    use embedded_common::ActuatorCommand;
    use embedded_common::FromPico;
    use embedded_common::IMU_READING_DELAY_MS;
    use futures_util::StreamExt;
    use std::time::Instant;
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
    pub static LAST_RESET: OnceLock<AtomicCell<Instant>> = OnceLock::new();

    pub struct V3PicoTask {
        /// when a v3 pico is connected, it's serial port will be available on this rx
        path_rx: Receiver<String>,
        is_broken: Option<tokio::sync::watch::Receiver<bool>>,
        /// Channel to send actuator commands to the async writer task
        cmd_tx: Option<tokio::sync::mpsc::Sender<[u8; ActuatorCommand::SIZE]>>,

        /// Message queue read from the pico
        from_pico: Arc<&'static ArrayQueue<FromPico>>,
        last_reading: Instant,
        /// prevents us from powercycling a bajillion times in a row
        last_powercycle: Instant,
        teri_mode: bool,
        speed_ratio: f64,
        wrong_pico_msg: Arc<Mutex<Option<String>>>,
    }

    impl Freezable for V3PicoTask {}

    impl CuTask for V3PicoTask {
        // input is the actuator command as bytes
        // the reason this is a tuple is a hack to get around the fact that copper doesnt support one task
        // having multiple outputs in the same way it does multiple inputs
        type Input<'m> = input_msg!(embedded_common::ActuatorCommand);
        type Output<'m> = output_msg!(FromPico);
        type Resources<'r> = ();

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let teri_mode = if let Some(config) = config {
                config.get::<bool>("teri_mode")?.unwrap_or(false)
            } else {
                false
            };

            let speed_ratio = if let Some(config) = config {
                config.get::<f64>("speed_ratio")?.unwrap_or(1.0)
            } else {
                1.0
            }.clamp(0.0, 1.0);

            let (path_tx, path_rx) = crossbeam_channel::bounded(1);
            let wrong_pico_msg = Arc::new(Mutex::new(None));
            let msg_clone = Arc::clone(&wrong_pico_msg);
            // this thread monitors for device add events, and then checks the serial number and if it matches what we expect for the pico, it sends the 
            // path to path_rx, where that path tries to be opened in pre_process. 
            // this is so we can hot plug the pico and it auto reconnects.
            std::thread::spawn(move || {
                let mut monitor = match MonitorBuilder::new() {
                    Ok(x) => x,
                    Err(e) => {
                        eprintln!("Failed to create udev monitor: {}", e.to_string());
                        return;
                    }
                };
                monitor = match monitor.match_subsystem("tty") {
                    Ok(x) => x,
                    Err(e) => {
                        eprintln!("Failed to set match-subsystem filter: {}", e.to_string());
                        return;
                    }
                };
                let listener = match monitor.listen() {
                    Ok(x) => x,
                    Err(e) => {
                        eprintln!("Failed to listen for udev events: {}", e.to_string());
                        return;
                    }
                };

                let mut enumerator = {
                    let udev = match Udev::new() {
                        Ok(x) => x,
                        Err(e) => {
                            eprintln!("Failed to create udev context: {}", e.to_string());
                            return;
                        }
                    };
                    match udev::Enumerator::with_udev(udev) {
                        Ok(x) => x,
                        Err(e) => {
                            eprintln!("Failed to create udev enumerator: {}", e.to_string());
                            return;
                        }
                    }
                };
                if let Err(e) = enumerator.match_subsystem("tty") {
                    eprintln!("Failed to set match-subsystem filter: {}", e.to_string());
                }
                let devices = match enumerator.scan_devices() {
                    Ok(x) => x,
                    Err(e) => {
                        eprintln!("Failed to scan devices: {}", e.to_string());
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
                        let Some(serial_cstr) = device.property_value("ID_SERIAL_SHORT") else {
                            return;
                        };
                        let Some(serial) = serial_cstr.to_str() else {
                            eprintln!("Failed to parse serial of device {}", path_str);
                            return;
                        };
                        match serial {
                            embedded_common::PRIME_PICO_SERIAL => {
                                if teri_mode {
                                    msg_clone.lock().unwrap().replace(String::from("Teri mode expected, but non teri pico was detected"));
                                    eprintln!("[PICO] Teri mode expected, but non teri pico was detected");
                                    return;
                                }
                                if path_tx.send(path.to_string_lossy().to_string()).is_err() {
                                    eprintln!("[PICO] Couldn't send controller path");
                                }
                                println!("[Info] Opened pico");
                            }
                            embedded_common::SECONDARY_PICO_SERIAL => {
                                eprintln!("[PICO] Unexpeced serial (is the secondary pico connected but not the prime pico?");
                            }
                            embedded_common::TERI_PICO_SERIAL => {
                                if teri_mode {
                                    if path_tx.send(path.to_string_lossy().to_string()).is_err() {
                                        eprintln!("[PICO] Couldn't send controller path");
                                    }
                                    println!("[Info] Opened pico");
                                } else {
                                    msg_clone.lock().unwrap().replace(String::from("TERI mode pico detected when teri mode isnt activated"));
                                    eprintln!("[PICO] TERI mode pico detected when teri mode isnt activated");
                                }
                            }
                            _ => {

                            }
                        }
                    })
            });
            let from_pico = Arc::new(&*Box::leak(Box::new(ArrayQueue::new(50))));
            Ok(Self {
                wrong_pico_msg,
                path_rx,
                is_broken: None,
                from_pico,
                cmd_tx: None,
                last_reading: Instant::now(),
                last_powercycle: Instant::now(),
                teri_mode,
                speed_ratio
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
                let port = match tokio_serial::new(&path_str, 115200)
                    .flow_control(tokio_serial::FlowControl::None)
                    .open_native_async()
                {
                    Ok(mut x) => {
                        if let Err(e) = x.set_exclusive(true) {
                            eprintln!(
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
                    "failed to open port",
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
            spawn_reader_thread(reader, is_broken_tx.clone(), self.from_pico.clone());

            let (cmd_tx, cmd_rx) = tokio::sync::mpsc::channel::<[u8; ActuatorCommand::SIZE]>(4);
            spawn_writer_task(writer, cmd_rx, is_broken_tx, self.teri_mode);

            self.is_broken = Some(is_broken_rx);
            self.cmd_tx = Some(cmd_tx);
            Ok(())
        }

        /// Sends actuator commands to the async writer task via channel.
        /// Messages from the pico are popped off the queue and sent to the downstream task.
        fn process<'i, 'o>(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'i>,
            output: &mut Self::Output<'o>,
        ) -> CuResult<()> {
            if let Some(mut actuator_cmd) = input.payload().cloned()
                && let Some(ref cmd_tx) = self.cmd_tx
            {
                actuator_cmd.apply_speed_factor(self.speed_ratio as f32);
                let serialized = ActuatorCommand::serialize(&actuator_cmd);
                if let Err(_) = cmd_tx.try_send(serialized) {
                    eprintln!("[PICO] Command channel full or closed, dropping command");
                }
            }
            if let Some(reading) = self.from_pico.pop() {
                output.set_payload(reading);
                self.last_reading = Instant::now();
                if let FromPico::Error(e) = reading {
                    return Err(CuError::from(format!("Pico Error: {e:?}")))
                }
            } else {
                output.clear_payload();
            }
            if let Some(msg) = self.wrong_pico_msg.lock().unwrap().deref() {
                return Err(CuError::from(msg.to_owned()));
            }

            if self.last_reading.elapsed().as_millis() > 1000 {
                return Err(CuError::new_with_cause(
                    "Pico Unresponsive",
                    std::io::Error::other("Pico Unresponsive"),
                ));
            }
            Ok(())
        }

        /// If the reader async task reports an error, return it.

        fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
            if let Some(ref is_broken) = self.is_broken
                && *is_broken.borrow()
            {
                self.powercycle_ioctl();
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
        from_pico: Arc<&'static ArrayQueue<FromPico>>,
    ) -> tokio::task::JoinHandle<()> {
        get_tokio_handle().spawn(async move {
            let mut no_reading_count = 0;
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(IMU_READING_DELAY_MS - 1))
                    .await;
                let Ok(reading) = timeout(Duration::from_millis(200), reader.next()).await else {
                    eprintln!("Pico has become unresponsive.");
                    let _ = is_broken_tx.send(true);
                    break;
                };
                if let Some(Err(e)) = reading {
                    let _ = is_broken_tx.send(true);
                    eprintln!("failed to read from pico: {}", e.to_string());
                    break;
                }
                if let None = reading {
                    no_reading_count += 1;
                    if no_reading_count <= 5 {
                        let _ = is_broken_tx.send(true);
                        eprintln!("Pico has become unresponsive. (no reading count)");
                        break;
                    }
                    continue;
                }
                let reading = reading.unwrap().unwrap();
                let Ok(reading) = reading.try_into() else {
                    eprintln!("not 105 bytes");
                    continue;
                };
                let Ok(reading) = FromPico::deserialize(reading) else {
                    eprintln!("Failed to deserialize message from picov3 serial port");
                    let _ = is_broken_tx.send(true);
                    break;
                };
                if let Err(_) = from_pico.push(reading) {
                    eprintln!("From Pico queue full, dropping reading");
                    // is_broken_tx.send(true).unwrap();
                    break;
                }
            }
        })
    }
    /// Receives actuator commands from the channel and writes them to the serial port
    fn spawn_writer_task(
        mut writer: WriteHalf<BufStream<SerialStream>>,
        mut cmd_rx: tokio::sync::mpsc::Receiver<[u8; ActuatorCommand::SIZE]>,
        is_broken_tx: tokio::sync::watch::Sender<bool>,
        teri_mode: bool,
    ) -> tokio::task::JoinHandle<()> {
        get_tokio_handle().spawn(async move {
            while let Some(data) = cmd_rx.recv().await {
                if teri_mode{
                    if let Err(e) = writer.write_all(&data).await {
                        eprintln!("[PICO] Failed to write to serial port: {e}");
                        let _ = is_broken_tx.send(true);
                        break;
                    }
                    if let Err(e) = writer.flush().await {
                        eprintln!("[PICO] Failed to flush serial port: {e}");
                        let _ = is_broken_tx.send(true);
                        break;
                    }
                } else {
                    let mut encoded = cobs::encode_vec(&data);
                    encoded.push(0u8);
                    if let Err(e) = writer.write_all(&encoded).await {
                        eprintln!("[PICO] Failed to write to serial port: {e}");
                        let _ = is_broken_tx.send(true);
                        break;
                    }
                    if let Err(e) = writer.flush().await {
                        eprintln!("[PICO] Failed to flush serial port: {e}");
                        let _ = is_broken_tx.send(true);
                        break;
                    }
                }
            }
        })
    }

    impl V3PicoTask {
        /// install this binary from misc/usb-reset
        fn powercycle_ioctl(&mut self) {
            if self.last_powercycle.elapsed().as_secs() > 1 {
                eprintln!("Pico broken, powercycling");
                get_tokio_handle().spawn(async {
                    match std::process::Command::new("usb-reset")
                        .arg("v3pico")
                        .spawn()
                    {
                        Ok(_) => {}
                        Err(e) => {
                            println!("Ioctl failed: {e}");
                        }
                    }
                });
                self.last_powercycle = Instant::now();
            }
        }
    }
}

#[cfg(feature = "production")]
pub use prod_impl::*;

#[cfg(not(feature = "production"))]
pub use resim_impl::*;

#[cfg(not(feature = "production"))]
mod resim_impl {
    use cu29::{cutask::CuTask, prelude::*};
    use embedded_common::FromPico;

    pub struct V3PicoTask {}
    impl Freezable for V3PicoTask {}
    impl CuTask for V3PicoTask {
        // input is the actuator command as bytes
        // ActuatorCommand doesn't implement Serialize or Decode due to being no_std
        // the reason this is a tuple is a hack to get around the fact that copper doesnt support one task having multiple outputs in the same way it does multiple inputs
        type Input<'m> = input_msg!(embedded_common::ActuatorCommand);
        // output is the FromPicoV3 struct serialized as bytes
        type Output<'m> = output_msg!(FromPico);
        type Resources<'r> = ();
        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process<'i, 'o>(
            &mut self,
            _clock: &RobotClock,
            _input: &Self::Input<'i>,
            _output: &mut Self::Output<'o>,
        ) -> CuResult<()> {
            Ok(())
        }
    }
}
