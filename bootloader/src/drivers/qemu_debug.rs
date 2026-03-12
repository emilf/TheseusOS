//! Module: bootloader::drivers::qemu_debug
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//!
//! INVARIANTS:
//! - This module provides the QEMU debug-port output path used by the bootloader.
//! - It is QEMU-specific observability support, not portable firmware I/O.
//!
//! SAFETY:
//! - Assuming availability here is a workflow/testing assumption about the QEMU environment, not a general hardware truth.
//! - Output success here says something about observability, not about broader boot correctness.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! QEMU debug driver.
//!
//! Uses QEMU's debug output port (0xe9) for simple output.

/// Bootloader output driver backed by QEMU's debug port.
pub struct QemuDebugDriver;

impl QemuDebugDriver {
    /// Create the QEMU debug-port driver.
    pub fn new() -> Self {
        Self
    }

    /// Check if this driver is considered available.
    ///
    /// Current bootloader policy treats this as available because normal development
    /// runs are expected to happen under QEMU with a debug-port sink attached.
    pub fn is_available(&self) -> bool {
        true
    }

    /// Write one byte to the QEMU debug path.
    fn write_char(&self, ch: u8) {
        theseus_shared::qemu_print_bytes!(&[ch]);
    }

    /// Write a string to the QEMU debug path.
    fn write_string(&self, s: &str) {
        theseus_shared::qemu_print_bytes!(s.as_bytes());
    }
}

impl crate::drivers::manager::Driver for QemuDebugDriver {
    fn write_line(&self, message: &str) -> bool {
        self.write_string(message);
        self.write_char(b'\r');
        self.write_char(b'\n');
        true
    }

    fn is_available(&self) -> bool {
        self.is_available()
    }

    fn name(&self) -> &'static str {
        "QEMU Debug"
    }
}
