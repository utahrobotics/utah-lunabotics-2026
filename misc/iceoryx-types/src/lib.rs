//! Common point types shared between C++ and Rust for iceoryx2 publish-subscribe.
//!
//! The struct layout **must** match the C++ definition used by the Unilidar
//! publisher (see `unilidar_iceoryx_publisher/include/imu_point_types.hpp`).
//! Keeping the same memory layout allows zero-copy transfer between the
//! languages.

pub mod sim;

#[cfg(any(feature = "sim", feature = "resim"))]
pub use sim::*;

pub mod production;
#[cfg(feature = "production")]
pub use production::*;
