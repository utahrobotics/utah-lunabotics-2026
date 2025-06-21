use cu29_runtime::{cutask::{CuMsg, CuSinkTask, CuTask, Freezable}, input_msg};
use cu29::{clock::RobotClock, CuError, CuResult};
use crate::{Level, LogDestination, RerunInput, RerunIpc, RerunOptions};
use rerun::Points3D;

impl Freezable for RerunIpc {}

impl<'cl> CuSinkTask<'cl> for RerunIpc {
    type Input = input_msg!('cl, RerunInput);

    fn new(config: Option<&cu29_runtime::config::ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized {
        if config.is_none() {
            return CuResult::Err(CuError::new_with_cause("no config provided", 
                std::io::Error::other("no config")
            ))
        }
        let config = config.unwrap();

        let log_destination = match config.get::<String>("log_destination")
            .as_deref()
            .map(str::to_ascii_lowercase)
            .as_deref()
        {
            Some("grpc") => {
                let addr = config.get::<String>("grpc_addr")
                                 .unwrap_or_else(|| "127.0.0.1:9870".into());
                LogDestination::Grpc { addr }
            }
            Some("file") => {
                let path = config.get::<String>("file_path")
                .unwrap_or_else(|| "rerun.rrd".into());
                LogDestination::File { path }
            }
            _ => LogDestination::Spawn,      // default
        };

        let level = match config.get::<String>("level")
            .as_deref()
            .map(str::to_ascii_lowercase)
            .as_deref()
        {
            Some("all") => Level::All,
            Some("minimal") => Level::Minimal,
            Some("none") => Level::None,
            _ => Level::Minimal,
        };
        let options = RerunOptions {
            level,
            log_destination,
        };
        Ok(RerunIpc::new(options))
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {

        Ok(())
    }
}
