# Testing Plan — TheseusOS

How to test a bare-metal OS kernel without making a mess of the codebase.
Last updated: 2026-03-26 (clean slate — all previous test infrastructure removed).

---

## Starting Point

All previous test files have been removed:
- `tests/bare_metal_tests.rs` — deleted (never tested anything real; bypassed the framework it claimed to use)
- `tests/kernel_tests.rs` — deleted (full of `assert!(true)` placeholders; called APIs that no longer existed)
- `tests/should_panic.rs` — deleted (worked mechanically but pointless in isolation)
- `shared/src/test_environments.rs` — deleted (taxonomy without enforcement)
- `run_tests.sh` — deleted (referenced test binaries that didn't exist)
- `kernel/Cargo.toml` `[[test]]` entries — removed
- `Makefile` test targets and macros — removed

The kernel builds cleanly. We start from here.

---

## Hard Constraints

Three things shape how testing works in a bare-metal kernel:

1. **No `cargo test`.** Tests are kernel binaries. Each test is a full QEMU boot — UEFI firmware loads the bootloader, bootloader hands off to the test binary, the test runs, QEMU exits. This takes several seconds per test suite.

2. **Boot is the context.** You cannot isolate a subsystem the way you'd mock a dependency in a library. To test the physical frame allocator, you have to boot far enough to initialize it. The test environment *is* the boot sequence.

3. **Failures are destructive.** A wrong memory write doesn't return an error — it corrupts state and causes a fault minutes later. Tests need to be conservative about what they assume is initialized.

---

## The Right Approach: Two Tiers

### Tier 1: In-kernel unit tests (no separate binary)

The simplest and most maintainable form: a `#[cfg(test)]` module inside the relevant kernel module, with tests that run as part of the normal boot sequence when a `KERNEL_TESTS` feature flag is enabled.

**How it works:**
- Add a `tests` module gated on `#[cfg(feature = "kernel-tests")]` inside the module being tested
- The kernel `main` checks a compile-time constant (or the feature flag) and calls `run_kernel_tests()` before going idle
- `run_kernel_tests()` calls each registered test, reports pass/fail via debugcon, then calls `qemu_exit_ok!()` or `qemu_exit_error!()`
- The existing `isa-debug-exit` device at port `0xf4` handles the exit

**Why this is better than separate `[[test]]` binaries:**
- Tests run in the same environment as production code — same boot sequence, same init order, same memory layout
- No separate binary that needs its own `kernel_main`, bootloader glue, and disk image
- Impossible for a test binary to call APIs that don't exist in the real kernel
- Tests stay colocated with the code they test

**Feature flag:** `kernel-tests` (add to `kernel/Cargo.toml` as an optional feature)

**Sketch:**

```rust
// kernel/src/memory/frame_allocator.rs

#[cfg(feature = "kernel-tests")]
pub(crate) mod tests {
    use super::*;

    pub fn run() -> Result<(), &'static str> {
        test_alloc_returns_nonzero_pa()?;
        test_alloc_free_roundtrip()?;
        Ok(())
    }

    fn test_alloc_returns_nonzero_pa() -> Result<(), &'static str> {
        let pa = crate::physical_memory::alloc_frame()
            .map_err(|_| "alloc_frame failed")?;
        if pa == 0 {
            return Err("alloc_frame returned PA 0");
        }
        crate::physical_memory::free_frame(pa)
            .map_err(|_| "free_frame failed")?;
        Ok(())
    }

    fn test_alloc_free_roundtrip() -> Result<(), &'static str> {
        let pa1 = crate::physical_memory::alloc_frame().map_err(|_| "alloc 1 failed")?;
        let pa2 = crate::physical_memory::alloc_frame().map_err(|_| "alloc 2 failed")?;
        if pa1 == pa2 {
            return Err("two allocs returned same PA");
        }
        crate::physical_memory::free_frame(pa1).map_err(|_| "free 1 failed")?;
        crate::physical_memory::free_frame(pa2).map_err(|_| "free 2 failed")?;
        Ok(())
    }
}
```

```rust
// kernel/src/lib.rs or main.rs

#[cfg(feature = "kernel-tests")]
pub fn run_kernel_tests() {
    use crate::{log_info, log_error};
    log_info!("=== KERNEL TESTS ===");

    let results: &[(&str, fn() -> Result<(), &'static str>)] = &[
        ("frame_allocator", crate::memory::frame_allocator::tests::run),
        // add more here as subsystems grow
    ];

    let mut passed = 0;
    let mut failed = 0;
    for (name, f) in results {
        match f() {
            Ok(()) => { log_info!("  PASS: {}", name); passed += 1; }
            Err(e) => { log_error!("  FAIL: {} — {}", name, e); failed += 1; }
        }
    }

    log_info!("=== {} passed, {} failed ===", passed, failed);
    if failed > 0 {
        theseus_shared::qemu_exit_error!();
    } else {
        theseus_shared::qemu_exit_ok!();
    }
}
```

```toml
# kernel/Cargo.toml
[features]
kernel-tests = []
```

**Running:**
```bash
cargo run -p theseus-qemu -- --headless --features kernel-tests
# or:
cargo build --package theseus-kernel --target x86_64-unknown-none \
    --release --features kernel-tests
```

Add a `make test` target to the Makefile once this is set up.

---

### Tier 2: Monitor-driven verification (interactive / semi-automated)

For things that are too stateful or environment-dependent to unit test cleanly — "does the APIC timer calibrate to a sane value?", "is the xHCI controller showing as bound?" — the debug monitor is the right tool.

This isn't automated (yet), but the QMP socket and serial relay make scripting possible:

```bash
# Boot with relays, then query the monitor
cargo run -p theseus-qemu -- --relays --headless &
sleep 3  # wait for boot
echo "memory" | nc -U /tmp/qemu-monitor-host
echo "cpu apic" | nc -U /tmp/qemu-monitor-host
```

A future `make verify` target could script a sequence of monitor commands and grep the output for expected patterns. This is the right approach for integration-level checks once Phase 1 subsystems exist.

---

## What Tests to Write and When

Write tests alongside the implementation — not after. The Phase 1 completion criteria (in `docs/plans/phase1-cpu-platform.md`) are the test specifications.

### When 1.6 (CPUID) is implemented:
```
PASS: CpuFeatures::detect() completes without panic
PASS: x2apic field matches CPUID leaf 0x1 ECX bit 21
FAIL (panic expected): CpuFeatures::get() before detect() — test via should-panic variant
```

### When 1.1.2 (VA Allocator) is implemented:
```
PASS: alloc_va(4096, 4096) twice returns non-overlapping ranges
PASS: alloc_va(4096, 8192) returns 8192-aligned address
PASS: alloc_va(0, ...) returns None or panics cleanly
```

### When 1.1.3 (Runtime Mapper) is implemented:
```
PASS: map_page(va, pa, flags) then translate(va) == pa
PASS: unmap_page(va) then translate(va) == None
FAIL: map_page on already-mapped VA returns Err(AlreadyMapped)
```

### When 1.1.4 (Stack Allocator) is implemented:
```
PASS: alloc_kernel_stack(65536).top is 16-byte aligned
PASS: alloc + free leaves frame allocator and VA allocator in consistent state
```

### When 1.4.2 (Scheduler tick) is implemented:
```
PASS: TICK_COUNT > 0 after busy-waiting ~50ms
PASS: current_tick() returns monotonically increasing values
PASS: ticks_to_ms(APIC_TICKS_PER_MS) == 1
```

### When 1.2.1 (driver_data fix) is implemented:
```
PASS: set_driver_state(42u32), driver_state::<u32>() == Some(&42)
PASS: set_driver_state(42u32), driver_state::<u64>() == None
```

---

## Agent Guidance

When an agent implements a Phase 1 task:

1. Write the implementation
2. Add a `#[cfg(feature = "kernel-tests")]` test module in the same file
3. Register it in `run_kernel_tests()` in `lib.rs`
4. Build with `--features kernel-tests` and confirm it compiles
5. Run with `cargo run -p theseus-qemu -- --headless` (without tests) — confirm no regressions
6. Run with `--features kernel-tests` — confirm tests pass
7. Include the test output in the PR

The test module shipping with the implementation is non-negotiable. A task without a test is not done.

---

## What Not to Do

- **Don't write separate `[[test]]` binaries** unless there's a very specific reason (e.g., a should-panic test that needs a custom panic handler). The separate binary approach requires its own boot glue and inevitably drifts from the real kernel.
- **Don't write tests that `assert!(true)`** or test things that can't fail. Every test must have a plausible failure mode.
- **Don't add test infrastructure that touches production boot paths** — feature flags keep test code out of release builds entirely.
- **Don't leave logspam on** — test output should go to debugcon, not serial, and should be minimal.
