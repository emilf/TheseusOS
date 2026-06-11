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
//!   returning `Err(reason)` = FAIL. A test that panics will cause the kernel's
//!   panic handler to exit QEMU with exit code 1 (FAIL).
//! - After all tests complete successfully, the kernel exits QEMU with PASS (exit 0)
//!   via the existing `isa-debug-exit` device at port 0xf4.
//! - Test output goes via standard kernel logging (debugcon) so it's always visible.
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
/// and exit QEMU with exit code 1 (interpreted as FAIL by the tooling).
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
        theseus_shared::qemu_exit_error!();
    } else {
        log_info!("TEST_RESULT: PASS");
        theseus_shared::qemu_exit_ok!();
    }

    loop {}
}
