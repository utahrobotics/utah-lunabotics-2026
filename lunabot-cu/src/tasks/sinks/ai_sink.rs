use bincode::{config::standard, encode_to_vec};
use common::{FromHost, FromLunabase};
use cu29::{
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuMsg, CuSinkTask, Freezable},
    input_msg,
    prelude::*,
    CuError, CuResult,
};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::prelude::*;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use iceoryx_types::{FromHostBytes, FROM_HOST_MAX_BYTES};

const FROM_HOST_SERVICE: &str = "lunabot/host_to_ai";

pub struct AiSink {
    publisher: Publisher<ipc::Service, FromHostBytes, ()>,
}

impl Freezable for AiSink {}

impl CuSinkTask for AiSink {
    type Input<'m> = input_msg!(Option<FromLunabase>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        // Create iceoryx publisher
        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .map_err(|e| CuError::new_with_cause("AiSink: node create", e))?;

        let service = node
            .service_builder(
                &ServiceName::new(FROM_HOST_SERVICE)
                    .map_err(|e| CuError::new_with_cause("AiSink: invalid service name", e))?,
            )
            .publish_subscribe::<FromHostBytes>()
            .open_or_create()
            .map_err(|e| CuError::new_with_cause("AiSink: service", e))?;

        let publisher = service
            .publisher_builder()
            .create()
            .map_err(|e| CuError::new_with_cause("AiSink: publisher", e))?;

        Ok(Self { publisher })
    }

    fn process(&mut self, clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(Some(msg)) = input.payload() {
            // Convert to FromHost variant
            let host_msg = FromHost::FromLunabase { msg: *msg };

            // Encode with bincode
            let config = standard();
            let bytes = encode_to_vec(&host_msg, config)
                .map_err(|e| CuError::new_with_cause("AiSink: encode", e))?;

            if bytes.len() > FROM_HOST_MAX_BYTES {
                return Err(CuError::from("AiSink: message too large for buffer"));
            }

            let mut payload = FromHostBytes::default();
            payload.len = bytes.len() as u32;
            payload.data[..bytes.len()].copy_from_slice(&bytes);

            match self.publisher.loan_uninit() {
                Ok(sample) => {
                    let initialized = sample.write_payload(payload);
                    let _ = initialized.send();
                }
                Err(e) => {
                    return Err(CuError::new_with_cause("AiSink: loan_uninit", e));
                }
            }
        }
        Ok(())
    }
}
