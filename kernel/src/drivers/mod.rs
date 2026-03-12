//! Module: drivers
//!
//! SOURCE OF TRUTH:
//! - docs/plans/drivers-and-io.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - This module is the public entry point for the kernel driver subsystem.
//! - Driver registration, device descriptors, PCI/USB support, and system-level driver bring-up are organized beneath this namespace.
//! - Public re-exports here should match the current driver-framework vocabulary used elsewhere in the kernel.
//!
//! SAFETY:
//! - Re-exporting a subsystem here does not reduce the safety obligations documented in the concrete driver modules.
//! - High-level entrypoint docs must not imply a richer driver lifecycle or isolation model than the current manager/traits implementation actually provides.
//!
//! PROGRESS:
//! - docs/plans/drivers-and-io.md
//!
//! Kernel driver subsystem entry point.
//!
//! This module exposes the core driver primitives and the higher-level driver
//! system initialization routines.

pub mod manager;
pub mod pci;
pub mod serial;
pub mod system;
pub mod traits;
pub mod usb;

pub use super::interrupts::{APIC_TIMER_VECTOR, SERIAL_RX_VECTOR};
pub use manager::DriverManager;
pub use serial::{init_serial, register_serial_driver};
pub use traits::{Device, DeviceClass, DeviceId, Driver};
