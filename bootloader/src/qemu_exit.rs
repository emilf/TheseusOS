//! Module: bootloader::qemu_exit
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status
//!
//! INVARIANTS:
//! - This module provides bootloader-side QEMU exit helpers for success/failure reporting.
//! - QEMU-exit handling here is part of test/debug workflow support, not portable platform logic.
//!
//! SAFETY:
//! - These helpers assume a QEMU environment with the expected exit device semantics.
//! - Immediate guest termination is the point, so callers must not expect recovery after invoking them.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! QEMU exit utilities.

use theseus_shared::constants::exit_codes;

/// Exit QEMU after emitting a final debug message.
pub unsafe fn exit_qemu_with_message(message: &str, exit_code: u8) {
    theseus_shared::qemu_println!(message);
    theseus_shared::qemu_exit!(exit_code);
}

/// Exit QEMU with an error code and message.
///
/// This keeps bootloader-side failure reporting aligned with the repo's QEMU-first
/// test workflow.
#[allow(dead_code)] // Intended for future use
pub unsafe fn exit_qemu_error(message: &str) {
    exit_qemu_with_message(message, exit_codes::QEMU_ERROR);
}

/// Exit QEMU due to a panic with panic information.
///
/// This is a workflow/testing helper for panic-time diagnostics under QEMU,
/// not a general recovery path or portable panic-reporting mechanism.
#[allow(dead_code)] // Intended for future use
pub unsafe fn exit_qemu_on_panic(panic_info: &core::panic::PanicInfo) {
    // Create a simple panic message
    let message = alloc::format!("PANIC: {:?}", panic_info.message());

    exit_qemu_error(&message);
}
