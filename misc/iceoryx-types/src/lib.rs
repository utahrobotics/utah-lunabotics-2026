//! Common point types shared between C++ and Rust for iceoryx2 publish-subscribe.
//!
//! The struct layout **must** match the C++ definition used by the Unilidar
//! publisher (see `unilidar_iceoryx_publisher/include/imu_point_types.hpp`).
//! Keeping the same memory layout allows zero-copy transfer between the
//! languages.

#[cfg(all(any(feature = "sim", feature = "resim"), not(feature = "production")))]
pub mod sim;

#[cfg(all(any(feature = "sim", feature = "resim"), not(feature = "production")))]
pub use sim::*;

#[cfg(all(feature = "production", not(all(feature="sim", feature="resim"))))]
pub mod production;
#[cfg(all(feature = "production", not(all(feature="sim", feature="resim"))))]
pub use production::*;


extern crate cu_bincode as bincode;