//! Module: bootloader::drivers
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//!
//! INVARIANTS:
//! - This module tree contains the bootloader-side output-driver stack.
//! - Bootloader drivers exist to support firmware-phase observability; they are distinct from later kernel drivers.
//!
//! SAFETY:
//! - Similar names between bootloader and kernel drivers should not blur the fact that they live in different execution environments with different guarantees.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! Bootloader output-driver subsystem.
//!
//! It is deliberately tiny and boot-phase-specific.

pub mod manager;
pub mod qemu_debug;
pub mod raw_serial;
pub mod uefi_serial;

// pub use manager::OutputDriver; // Removed unused import
