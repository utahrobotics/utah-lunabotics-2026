#[cfg(target_os = "linux")]
pub mod v4lstream;
pub mod udev_monitor;
pub mod april_detection_logger;
pub mod auto_camera;
pub mod auto_gstreamer;
pub use udev_monitor::UdevMonitor;
pub use april_detection_logger::DetectionLogger;
pub use auto_camera::V4lAutoCam;
pub use auto_camera::V4lAutoCam as V4lAutoCamera;
pub use udev_monitor::NewDevice;
pub use auto_gstreamer::*;