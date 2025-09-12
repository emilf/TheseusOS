//! Driver system for output communication
//! 
//! This module provides a unified interface for output drivers that can work
//! both during and after UEFI boot services.

pub mod uefi_serial;
pub mod raw_serial;
pub mod qemu_debug;
pub mod manager;

// pub use manager::OutputDriver; // Removed unused import
