//! Monitor command implementations
//!
//! This module contains all the interactive commands available in the kernel monitor.
//! Each command is implemented as a method on the `Monitor` struct, organized by category.

pub mod control;
pub mod cpu;
pub mod devices;
pub mod io;
pub mod memory;
pub mod pci;
pub mod system;
pub mod tables;
pub mod usb;
