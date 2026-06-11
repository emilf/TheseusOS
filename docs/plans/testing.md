# Testing Plan — TheseusOS

How to test a bare-metal OS kernel without making a mess of the codebase.
Last updated: 2026-06-11 (test-automation-v1: kernel test runner + theseus-qemu test + TDD workflow).

---

## Current State

The test infrastructure is now implemented and operational:

**Kernel side:**
- `kernel/src/testing.rs` — `Test` struct and `run_kernel_tests()` function
- Feature-gated on `#[cfg(feature = "kernel-tests")]` — no impact on release builds
- Tests are `fn() -> Result<(), &'static str>` — `Ok(())` = PASS, `Err(reason)` = FAIL
- Test output goes through standard kernel logging (debugcon port 0xE9)
- Registered in `after_high_half_entry()` in `environment.rs`, right before the idle loop

**Tooling side:**
- `theseus-qemu test` subcommand — builds with `--features kernel-tests`, runs QEMU headless, maps exit code to verdict
- Exit codes: 0=PASS, 1=FAIL, 2=PANIC, 3=TIMEOUT
- `.test-output.log` saved on any non-PASS exit
- `--timeout <secs>` (default 60), `--print`, `--no-build` flags
- `make test` convenience target

**Feature propagation:**
- `kernel/Cargo.toml`: `kernel-tests = []`
- `bootloader/Cargo.toml`: `kernel-tests = ["theseus-kernel/kernel-tests"]`
- Build via `make all FEATURES=kernel-tests` or `cargo run -p theseus-qemu -- test`

---

## Hard Constraints

Three things shape how testing works in a bare-metal kernel:

1. **No `cargo test`.** Tests are kernel binaries. Each test is a full QEMU boot — UEFI firmware loads the bootloader, bootloader hands off to the test binary, the test runs, QEMU exits. This takes several seconds per test suite.

2. **Boot is the context.** You cannot isolate a subsystem the way you'd mock a dependency in a library. To test the physical frame allocator, you have to boot far enough to initialize it. The test environment *is* the boot sequence.

3. **Failures are destructive.** A wrong memory write doesn't return an error — it corrupts state and causes a fault minutes later. Tests need to be conservative about what they assume is initialized.

---

## Writing Tests

### Test structure

Each test is a `Test` struct with a `name` and `func`:

```rust
let test_list: &[crate::testing::Test] = &[
    Test {
        name: "frame_allocator_basic",
        func: || -> Result<(), &'static str> {
            // test body
            Ok(())
        },
    },
    Test {
        name: "frame_allocator_free_roundtrip",
        func: test_alloc_free_roundtrip,
    },
];
```

Tests return `Result<(), &'static str>`:
- `Ok(())` = **PASS** — the test passed
- `Err(reason)` = **FAIL** — the test failed with a specific reason
- Panic = **FAIL** — the kernel's panic handler calls `qemu_exit_error!()` (exit code 1)

### Where to put tests

Tests belong in a `#[cfg(feature = "kernel-tests")]` module inside the subsystem
they test. The test module should be registered in the `Test` array passed to
`run_kernel_tests()` in `kernel/src/environment.rs`.

**Example (in `kernel/src/environment.rs`):**

```rust
#[cfg(feature = "kernel-tests")]
{
    let test_list: &[crate::testing::Test] = &[
        Test { name: "ping", func: || Ok(()) },
    ];
    crate::testing::run_kernel_tests(test_list);
}
```

### Adding tests to subsystems

See `docs/plans/testing.md` for test suggestions per subsystem:

### When 1.6 (CPUID):
```
PASS: CpuFeatures::detect() completes without panic
PASS: x2apic field matches CPUID leaf 0x1 ECX bit 21
FAIL (panic expected): CpuFeatures::get() before detect()
```

### When 1.1.2 (VA Allocator):
```
PASS: alloc_va(4096, 4096) twice returns non-overlapping ranges
PASS: alloc_va(4096, 8192) returns 8192-aligned address
PASS: alloc_va(0, ...) returns Err(InvalidSize)
```

### When 1.1.3 (Runtime Mapper):
```
PASS: map_page(va, pa, flags) then translate(va) == pa
PASS: unmap_page(va) then translate(va) == None
FAIL: map_page on already-mapped VA returns Err(AlreadyMapped)
```

### When 1.1.4 (Stack Allocator):
```
PASS: alloc_kernel_stack(65536).top is 16-byte aligned
PASS: alloc + free leaves frame allocator and VA allocator in consistent state
```

### When 1.4.2 (Scheduler tick):
```
PASS: TICK_COUNT > 0 after busy-waiting ~50ms
PASS: current_tick() returns monotonically increasing values
PASS: ticks_to_ms(APIC_TICKS_PER_MS) == 1
```

### When 1.2.1 (driver_data fix):
```
PASS: set_driver_state(42u32), driver_state::<u32>() == Some(&42)
PASS: set_driver_state(42u32), driver_state::<u64>() == None
```

---

## Running Tests

The primary command:

```bash
# Build kernel-tests feature, boot QEMU headless, report verdict via exit code
cargo run -p theseus-qemu -- test

# With custom timeout
cargo run -p theseus-qemu -- test --timeout 120

# Print QEMU output to stdout
cargo run -p theseus-qemu -- test --print

# Convenience target
make test
```

### Exit code scheme

| Exit Code | Verdict | Meaning |
|-----------|---------|---------|
| 0 | PASS | All tests passed |
| 1 | FAIL | At least one test returned Err(...), or kernel panicked |
| 2 | PANIC | Kernel explicitly reported panic through exit code 2 |
| 3 | TIMEOUT | QEMU killed by timeout (default 60s) |

On any non-PASS exit, `.test-output.log` is saved in the current directory.

---

## TDD Workflow

The mandatory TDD workflow for agent-driven development:

1. **Baseline:** `cargo run -p theseus-qemu -- test` → exit 0 (PASS)
2. **Define:** Write the test asserting the feature's success criteria
3. **RED:** `cargo run -p theseus-qemu -- test` → exit 1 (FAIL)
4. **Implement:** Write the feature code. Do NOT modify the test.
5. **GREEN:** `cargo run -p theseus-qemu -- test` → exit 0 (PASS)
6. **Report:** `Verdict: PASS (exit code 0)`

**Cardinal rule: The test is the truth.** Once step 3 confirms the test fails
on clean code, it is frozen. Do not redefine success criteria, stub the test,
or claim partial success.

---

## Agent Guidance

When an agent implements a feature:

1. Write the implementation
2. Add a `#[cfg(feature = "kernel-tests")]` test module
3. Register tests in the `Test` array in `after_high_half_entry()`
4. Build with `make all FEATURES=kernel-tests` and confirm it compiles
5. Run with `cargo run -p theseus-qemu -- test` — confirm tests pass
6. Include the test output in the PR

The test module shipping with the implementation is non-negotiable. A task
without a test is not done.

See `docs/plans/agent-test-automation.md` for the full design document and
`docs/agent-prompt-template.md` for the copy-pasteable agent instruction template.

---

## What Not to Do

- **Don't write separate `[[test]]` binaries** — the single-binary approach is simpler and avoids boot drift.
- **Don't write tests that `assert!(true)`** or test things that can't fail. Every test must have a plausible failure mode.
- **Don't add test infrastructure that touches production boot paths** — feature flags keep test code out of release builds entirely.
- **Don't leave logspam on** — test output goes through the kernel's logging system, which respects log levels.
