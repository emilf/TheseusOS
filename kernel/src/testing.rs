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
// Test scenarios selected via build-time feature flags.
// This lets us manufacture PASS/FAIL/PANIC/TIMEOUT conditions from the same codebase.
#[cfg(feature = "kernel-tests")]
mod test_scenarios {
    /// Which test scenario to run. Set via kernel config or feature flag.
    /// Warn-free: individual variants are #[allow]ed since only one is selected at build time.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[allow(dead_code)]
    pub(crate) enum Scenario {
        /// All tests pass (default)
        AllPass,
        /// One test fails — verify FAIL exit code
        OneFail,
        /// A test panics — verify PANIC exit code
        OnePanic,
        /// Hang forever — verify TIMEOUT
        Hang,
    }

    /// Determine the scenario to run.
    ///
    /// Default: `AllPass`. Override via cargo feature flags:
    ///   `kernel-test-scenario-fail`   → OneFail   → expect exit 1 (FAIL)
    ///   `kernel-test-scenario-panic`  → OnePanic  → expect exit 2 (PANIC)
    ///   `kernel-test-scenario-hang`   → Hang      → expect exit 3 (TIMEOUT)
    pub(crate) fn get_scenario() -> Scenario {
        #[cfg(feature = "kernel-test-scenario-fail")]
        { return Scenario::OneFail; }

        #[cfg(feature = "kernel-test-scenario-panic")]
        { return Scenario::OnePanic; }

        #[cfg(feature = "kernel-test-scenario-hang")]
        { return Scenario::Hang; }

        #[cfg(not(any(
            feature = "kernel-test-scenario-fail",
            feature = "kernel-test-scenario-panic",
            feature = "kernel-test-scenario-hang",
        )))]
        { Scenario::AllPass }
    }
}

// ---------------------------------------------------------------------------
// Actual tests — each exercises infrastructure that must work for real tests.
// ---------------------------------------------------------------------------

/// Test that the ISA debug exit PASS path works.
/// Uses `qemu_exit_test_pass!()` macro directly to verify the raw exit code path.
fn test_isa_debug_exit_pass() -> Result<(), &'static str> {
    // We can't actually call qemu_exit_test_pass!() here because that exits QEMU.
    // Instead, test that the constant is correct and the macro compiles.
    // The actual code path verification happens at the QEMU exit code level.
    if theseus_shared::constants::exit_codes::QEMU_EXIT_TEST_PASS != 3 {
        return Err("QEMU_EXIT_TEST_PASS constant mismatch");
    }
    if theseus_shared::constants::exit_codes::QEMU_EXIT_TEST_FAIL != 4 {
        return Err("QEMU_EXIT_TEST_FAIL constant mismatch");
    }
    Ok(())
}

/// Test that the panic handler writes the expected exit code.
/// Verifies QEMU_PANIC constant is correct.
fn test_panic_exit_code_constant() -> Result<(), &'static str> {
    if theseus_shared::constants::exit_codes::QEMU_PANIC != 2 {
        return Err("QEMU_PANIC constant mismatch");
    }
    Ok(())
}

/// Test basic integer arithmetic (trivial sanity check that kernel tests run at all).
fn test_simple_math() -> Result<(), &'static str> {
    if 1 + 1 != 2 {
        return Err("basic arithmetic failed");
    }
    Ok(())
}

/// Test that the QEMU_EXIT constant is accessible and correct.
fn test_qemu_exit_port_constant() -> Result<(), &'static str> {
    if theseus_shared::constants::io_ports::QEMU_EXIT != 0xf4 {
        return Err("QEMU_EXIT port constant mismatch");
    }
    if theseus_shared::constants::io_ports::QEMU_DEBUG != 0xe9 {
        return Err("QEMU_DEBUG port constant mismatch");
    }
    Ok(())
}

/// ISA debug exit transform test: verify we know the QEMU exit code formula.
/// QEMU transforms: qemu_exit = (raw_val << 1) | 1.
/// So raw 3 → QEMU exit 7, raw 4 → QEMU exit 9, raw 1 → QEMU exit 3.
fn test_isa_exit_transform() -> Result<(), &'static str> {
    fn compute_qemu_exit(raw: u8) -> i32 {
        ((raw as i32) << 1) | 1
    }
    if compute_qemu_exit(3) != 7 {
        return Err("expected raw 3 → QEMU exit 7");
    }
    if compute_qemu_exit(4) != 9 {
        return Err("expected raw 4 → QEMU exit 9");
    }
    if compute_qemu_exit(1) != 3 {
        return Err("expected raw 1 → QEMU exit 3");
    }
    if compute_qemu_exit(0) != 1 {
        return Err("expected raw 0 → QEMU exit 1");
    }
    Ok(())
}

/// Build the test list based on the selected scenario.
pub(crate) fn build_test_list() -> &'static [Test] {
    use test_scenarios::Scenario;

    match test_scenarios::get_scenario() {
        Scenario::AllPass => {
            // All tests pass → should produce PASS verdict
            &[
                Test { name: "test_isa_exit_transform", func: test_isa_exit_transform },
                Test { name: "test_qemu_exit_port_constant", func: test_qemu_exit_port_constant },
                Test { name: "test_isa_debug_exit_pass", func: test_isa_debug_exit_pass },
                Test { name: "test_panic_exit_code_constant", func: test_panic_exit_code_constant },
                Test { name: "test_simple_math", func: test_simple_math },
            ]
        }
        Scenario::OneFail => {
            // One test fails → should produce FAIL verdict
            &[
                Test { name: "test_intentional_fail", func: || Err("intentional failure for FAIL test") },
                Test { name: "test_simple_math", func: test_simple_math },
            ]
        }
        Scenario::OnePanic => {
            // Test that panics → should produce PANIC verdict via panic handler
            &[
                Test { name: "test_intentional_panic", func: || -> Result<(), &'static str> {
                    panic!("intentional panic for PANIC test");
                }},
            ]
        }
        Scenario::Hang => {
            // Empty test list, but this won't produce a verdict —
            // the `run_kernel_tests()` function will call either qemu_exit_test_pass!()
            // or qemu_exit_test_fail!().
            // To trigger a TIMEOUT, we need the kernel to never call either.
            // This scenario is handled specially in `run_kernel_tests`.
            &[]
        }
    }
}

pub(crate) fn run_kernel_tests() -> ! {
    use test_scenarios::Scenario;

    let scenario = test_scenarios::get_scenario();

    // Special case: Hang scenario — never exit, let QEMU timeout
    if scenario == Scenario::Hang {
        log_info!("=== KERNEL TESTS (HANG SCENARIO) ===");
        log_info!("Scenario: Hang — entering infinite loop for TIMEOUT test");
        loop {
            core::hint::spin_loop();
        }
    }

    let tests = build_test_list();

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
