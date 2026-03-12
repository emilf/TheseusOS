//! Module: panic
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//!
//! INVARIANTS:
//! - Kernel panic reporting goes through the logging path and then terminates QEMU with an error status.
//! - Panic handling is a last-resort reporting path and does not attempt graceful recovery.
//!
//! SAFETY:
//! - Panic code must avoid allocator dependence and any behavior likely to recurse into a second panic.
//! - Diagnostic richness is secondary to reliability; a short reliable panic report beats a clever fragile one.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//!
//! Kernel panic handling.
//!
//! This module provides the last-resort panic path that reports failure and then
//! terminates the QEMU run.

/// Kernel panic handler.
///
/// Emit a short panic report and then exit QEMU.
#[cfg(not(test))]
#[panic_handler]
pub fn panic_handler(panic_info: &core::panic::PanicInfo) -> ! {
    // Output panic information using direct QEMU debug port
    // Note: We avoid using the allocator during panic to prevent recursive panics

    // Use logging system in panic handler - it's allocation-free with stack buffers
    use crate::log_error;
    log_error!("KERNEL PANIC: Panic occurred");
    if let Some(location) = panic_info.location() {
        log_error!("  location: {}", location.file());
        log_error!("  line: {}", location.line());
    }
    log_error!("  panic payload (type info only)");

    theseus_shared::qemu_exit_error!();

    loop {}
}
