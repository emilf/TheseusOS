//! Module: bootloader::drivers::uefi_serial
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//!
//! INVARIANTS:
//! - This module provides the bootloader-side UEFI Serial I/O output path.
//! - Availability is tied to the firmware boot-services phase and the presence of a matching protocol handle.
//!
//! SAFETY:
//! - Boot-services availability checks are phase-sensitive and should not be mistaken for later kernel/runtime guarantees.
//! - This driver is firmware-environment plumbing, not the same serial model used after the kernel takes over.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! UEFI serial driver.
//!
//! Uses the UEFI Serial I/O protocol for output. Only works during boot services.

use crate::serial::serial_write_line;
use uefi::boot::SearchType;
use uefi::proto::console::serial::Serial;
use uefi::{Handle, Identify};

/// Bootloader output driver backed by UEFI Serial I/O.
pub struct UefiSerialDriver {
    handle: Option<Handle>,
}

impl UefiSerialDriver {
    /// Create the UEFI serial driver.
    pub fn new() -> Self {
        let handle = uefi::boot::locate_handle_buffer(SearchType::ByProtocol(&Serial::GUID))
            .ok()
            .and_then(|buf| buf.first().copied());

        Self { handle }
    }

    /// Check if UEFI boot services still appear active.
    pub fn is_boot_services_active() -> bool {
        // Check if we can still access UEFI services by trying to get the system table
        uefi::table::system_table_raw().is_some()
    }

    /// Check if this driver currently appears usable during the firmware boot phase.
    pub fn is_available(&self) -> bool {
        Self::is_boot_services_active() && self.handle.is_some()
    }
}

impl crate::drivers::manager::Driver for UefiSerialDriver {
    fn write_line(&self, message: &str) -> bool {
        if !self.is_available() {
            return false;
        }

        serial_write_line(self.handle, message);
        true
    }

    fn is_available(&self) -> bool {
        self.is_available()
    }

    fn name(&self) -> &'static str {
        "UEFI Serial"
    }
}
