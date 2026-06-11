//! Module: testing
//!
//! SOURCE OF TRUTH:
//! - docs/plans/testing.md
//! - docs/plans/agent-test-automation.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//! - docs/axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status
//!
//! INVARIANTS:
//! - This module only compiles when the `kernel-tests` feature is enabled.
//! - Tests are registered into a static array of `Test` structs.
//! - Each test returns `Result<(), &'static str>`. Returning `Ok(())` = PASS,
//!   returning `Err(reason)` = FAIL.
//! - A test that panics will be caught by the kernel panic handler, which writes
//!   `QEMU_ERROR` (1) to port 0xf4 → QEMU exits 3 → tool maps to PANIC (exit 2).
//! - Test runner uses `qemu_exit_test_pass!()` (raw 3 → QEMU exits 7) for PASS
//!   and `qemu_exit_test_fail!()` (raw 4 → QEMU exits 9) for FAIL.
//! - The `theseus-qemu test` tool maps QEMU exits 7/9/3/124 to exit codes 0/1/2/3.
//! - No string parsing required — the exit code is the sole verdict.
//! - Test output goes via standard kernel logging (debugcon) for human debugging.
//! - `.test-output.log` is saved on non-PASS exit for inspection.
//!
//! SAFETY:
//! - Tests run with full kernel privileges. A test that writes to wrong memory
//!   can corrupt kernel state and cause cascading failures. Keep tests focused
//!   and conservative about what subsystems they assume are initialized.
//!
//! PROGRESS:
//! - docs/plans/testing.md
//! - docs/plans/agent-test-automation.md

#![cfg(feature = "kernel-tests")]

use crate::{log_error, log_info};

/// A single test entry: a name and a function that returns Ok(()) or Err(reason).
pub(crate) struct Test {
    pub(crate) name: &'static str,
    pub(crate) func: fn() -> Result<(), &'static str>,
}

/// Run all registered kernel tests and exit QEMU with the result.
///
/// Called from the boot path (`after_high_half_entry`) when the
/// `kernel-tests` feature is enabled. This function never returns.
///
/// Each test returns `Result<(), &'static str>`. If it returns `Ok(())`,
/// the test PASSED. If it returns `Err(reason)`, the test FAILED.
/// A test that panics will be caught by the kernel's panic handler
/// (writes QEMU_ERROR=1 to port 0xf4, QEMU exits 3, tool maps to PANIC).
pub(crate) fn run_kernel_tests(tests: &[Test]) -> ! {
    log_info!("=== KERNEL TESTS ===");
    log_info!("Running {} test(s)", tests.len());

    let mut passed = 0u32;
    let mut failed = 0u32;

    for test in tests {
        match (test.func)() {
            Ok(()) => {
                log_info!("  {}: PASS", test.name);
                passed += 1;
            }
            Err(reason) => {
                log_error!("  {}: FAIL — {}", test.name, reason);
                failed += 1;
            }
        }
    }

    log_info!("TEST_SUMMARY: {} passed, {} failed", passed, failed);

    if failed > 0 {
        log_error!("TEST_RESULT: FAIL");
        theseus_shared::qemu_exit_test_fail!();
    } else {
        log_info!("TEST_RESULT: PASS");
        theseus_shared::qemu_exit_test_pass!();
    }

    loop {}
}
