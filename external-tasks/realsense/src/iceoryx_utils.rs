use iceoryx2::{
    node::NodeBuilder,
    port::{publisher::Publisher, subscriber::Subscriber},
    prelude::{ServiceName, UnableToDeliverStrategy},
    service::ipc,
};
use iceoryx_types::{IceoryxDepthFrame, IceoryxOccupancyGrid, IceoryxPointCloud};

/// Creates a new iceoryx2 node for IPC services
pub fn create_node() -> iceoryx2::node::Node<ipc::Service> {
    NodeBuilder::new()
        .create::<ipc::Service>()
        .expect("Failed to create iceoryx2 node")
}

/// Creates a cloud publisher for the given serial
pub fn create_depth_frame_publisher<const SIZE: usize>(
    node: &iceoryx2::node::Node<ipc::Service>,
    serial: &str,
) -> Publisher<ipc::Service, IceoryxDepthFrame<SIZE>, ()> {
    let depth_service_name = format!("realsense/{}/depth", serial);
    let depth_service = node
        .service_builder(&ServiceName::new(&depth_service_name).expect("Invalid service name"))
        .publish_subscribe::<IceoryxDepthFrame<SIZE>>()
        .enable_safe_overflow(true)
        .subscriber_max_buffer_size(20)
        .open_or_create()
        .expect("Failed to create cloud service");

    depth_service
        .publisher_builder()
        .create()
        .expect("Failed to create cloud publisher")
}
