//! Kernel driver subsystem entry point
//!
//! This module exposes the core driver primitives (traits, manager) and the
//! higher-level driver system initialization routines.

pub mod manager;
pub mod serial;
pub mod system;
pub mod traits;

pub use manager::DriverManager;
pub use serial::register_serial_driver;
pub use traits::{Device, DeviceClass, DeviceId, Driver};
