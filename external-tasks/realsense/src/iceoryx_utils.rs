use iceoryx2::{
    node::NodeBuilder,
    port::{publisher::Publisher, subscriber::Subscriber},
    prelude::{ServiceName, UnableToDeliverStrategy},
    service::ipc,
};
use iceoryx_types::{IceoryxOccupancyGrid, IceoryxPointCloud};

/// Creates a new iceoryx2 node for IPC services
pub fn create_node() -> iceoryx2::node::Node<ipc::Service> {
    NodeBuilder::new()
        .create::<ipc::Service>()
        .expect("Failed to create iceoryx2 node")
}

/// Creates a cloud publisher for the given serial
pub fn create_cloud_publisher(
    node: &iceoryx2::node::Node<ipc::Service>,
    serial: &str,
) -> Publisher<ipc::Service, IceoryxPointCloud, ()> {
    let cloud_service_name = format!("realsense/{}/cloud", serial);
    let cloud_service = node
        .service_builder(&ServiceName::new(&cloud_service_name).expect("Invalid service name"))
        .publish_subscribe::<IceoryxPointCloud>()
        .enable_safe_overflow(false)
        .subscriber_max_buffer_size(20)
        .open_or_create()
        .expect("Failed to create cloud service");

    cloud_service
        .publisher_builder()
        .unable_to_deliver_strategy(UnableToDeliverStrategy::Block)
        .create()
        .expect("Failed to create cloud publisher")
}

/// Creates an occupancy publisher for the given serial
pub fn create_occupancy_publisher(
    node: &iceoryx2::node::Node<ipc::Service>,
    serial: &str,
) -> Publisher<ipc::Service, IceoryxOccupancyGrid, ()> {
    let occupancy_service_name = format!("realsense/{}/occupancy", serial);
    let occupancy_service = node
        .service_builder(&ServiceName::new(&occupancy_service_name).expect("Invalid service name"))
        .publish_subscribe::<IceoryxOccupancyGrid>()
        .open_or_create()
        .expect("Failed to create occupancy service");

    occupancy_service
        .publisher_builder()
        .create()
        .expect("Failed to create occupancy publisher")
}

/// Creates a subscriber for the RealSense isometry transform
pub fn create_isometry_subscriber(
    node: &iceoryx2::node::Node<ipc::Service>,
) -> Subscriber<ipc::Service, [f64; 16], ()> {
    let from_service = node
        .service_builder(
            &ServiceName::new("localizer/realsense_isometry").expect("invalid service name"),
        )
        .publish_subscribe::<[f64; 16]>()
        .enable_safe_overflow(true)
        .open_or_create()
        .expect("Realsense: failed to open localizer→realsense service");

    from_service
        .subscriber_builder()
        .create()
        .expect("Realsense: failed to create subscriber")
}
