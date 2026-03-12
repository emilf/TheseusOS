//! Module: bootloader::serial
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//!
//! INVARIANTS:
//! - This module provides UEFI-phase serial helpers only; it is not the kernel’s long-term serial implementation.
//! - These helpers are best-effort boot-time observability paths available before `ExitBootServices`.
//!
//! SAFETY:
//! - UEFI Serial protocol access is only valid while the firmware-side environment is still live.
//! - Silent best-effort failure is a boot-time logging tradeoff, not proof that serial output succeeded.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! Bootloader serial utilities.
//!
//! This module provides simple serial output helpers for the UEFI bootloader phase.

use uefi::{boot, prelude::*, proto::console::serial::Serial};

/// Write raw bytes through the UEFI Serial I/O protocol.
///
/// A successful call only means the firmware serial protocol accepted the write.
/// It does not prove that a human-visible serial path is wired end to end.
pub fn serial_write(serial_handle: Option<Handle>, data: &[u8]) {
    if let Some(handle) = serial_handle {
        // Try to open the Serial protocol exclusively
        // exclusive = no other code can use it while we have it open
        if let Ok(mut serial) = boot::open_protocol_exclusive::<Serial>(handle) {
            // Write data (ignore errors - best effort during early boot)
            let _ = serial.write(data);
        }
    }
}

/// Write one CRLF-terminated line through the firmware serial path.
pub fn serial_write_line(serial_handle: Option<Handle>, line: &str) {
    serial_write(serial_handle, line.as_bytes());
    serial_write(serial_handle, b"\r\n");
}
