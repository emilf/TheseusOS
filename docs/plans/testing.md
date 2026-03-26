# Testing Plan — TheseusOS

How to think about, write, and run tests in a bare-metal OS kernel.
Last updated: 2026-03-26.

---

## What We Have Today

### Test harnesses (in `tests/`)

| File | Environment | Status |
|------|-------------|--------|
| `bare_metal_tests.rs` | Pre-heap, pre-interrupts | Runs. Tests are trivially simple (1+1=2, stack copy, loop) |
| `kernel_tests.rs` | Post full kernel init | Placeholder. `test_heap_allocation` works; rest is `assert!(true)` |
| `should_panic.rs` | Bare-metal | Works. Verifies panic handler fires and exits QEMU correctly |

### Infrastructure

- `make test-bare-metal` / `test-kernel` / `test-panic` / `test-all` — exist in Makefile
- `run_tests.sh` — old, likely broken (references a `simple` test binary that doesn't exist, uses outdated build flow)
- `shared/src/test_environments.rs` — `TestEnvironment` enum and capability specs. Solid taxonomy, not yet enforced anywhere.
- QEMU `isa-debug-exit` device — already wired; exit code 0 = pass, 1 = fail
- `theseus-qemu` runner — preferred way to run the kernel; not yet integrated with tests

### What's missing

- **Meaningful tests.** Current tests don't exercise any real kernel subsystems.
- **Test-to-feature mapping.** No tests correspond to the Phase 1 completion criteria.
- **Running state.** It's unclear if `make test-bare-metal` currently passes without errors. Needs verification.
- **Agent guidance.** No pattern for how agents should write tests when implementing a task.
- **CI hook.** Tests aren't run automatically on PRs.

---

## Testing Philosophy for a Bare-Metal Kernel

Three hard constraints shape everything:

1. **No test host.** Tests run inside QEMU as a kernel binary. There is no `cargo test` magic. Each test is a full boot.
2. **Boot is slow.** Each QEMU run is multiple seconds. Keep test suites short and targeted.
3. **Failure modes are brutal.** A bad test doesn't return a nice error — it triple-faults, hangs, or GPFs. Tests must be conservative in what they assume is initialized.

### Consequence: test at the right layer

| Layer | What you can test | What you can't |
|-------|------------------|----------------|
| Bare-metal | CPU arithmetic, stack, pure-Rust logic (no alloc), inline asm, debugcon output | Heap, interrupts, MMIO, anything needing kernel init |
| Post-kernel-init | Heap allocation, physical allocator, memory mapping, driver registration, APIC timer tick | User mode, VFS, anything not yet implemented |
| Monitor commands | Integration-style: "does `memory` command print sane values?", "does `cpu apic` show the right mode?" | Anything automated (monitor is interactive) |

### The assert + exit pattern

We use QEMU's `isa-debug-exit` device. Tests communicate pass/fail by writing to port `0xf4`:
- `0` → QEMU exits with code `(0 << 1) | 1 = 1` (success in our convention)
- `1` → QEMU exits with code `(1 << 1) | 1 = 3` (failure)

The macros in `theseus_shared` (`qemu_exit_ok!`, `qemu_exit_error!`) handle this.

A test panicking = `panic_handler` fires = `qemu_exit_error!` = QEMU exits with failure code.

---

## What Tests to Write and When

The Phase 1 completion criteria (in `docs/plans/phase1-cpu-platform.md`) double as a test specification. For each implemented task, write at least one test that exercises its completion criterion.

### Bare-metal tests (no kernel init required)

These can be written now or alongside implementation:

**1.6 CPUID abstraction** (`[ ] TODO`)
```
- CpuFeatures::detect() returns a populated struct without panicking
- CpuFeatures::get() panics before detect() is called (should_panic variant)
- x2apic field matches CPUID leaf 0x1 ECX bit 21
```

**1.1.2 VA Allocator** (`[ ] TODO`)
```
- Two alloc_va() calls return non-overlapping ranges
- alloc_va() respects requested alignment
- free_va() followed by alloc_va() of same size succeeds (bump won't reclaim, but freelist will)
- alloc_va(0) returns None or panics cleanly
```

### Post-kernel-init tests (after full boot sequence)

Write these alongside the implementation, after the relevant subsystem is working:

**1.1.3 Runtime Mapper** (`[ ] TODO`)
```
- map_page(va, pa, flags) succeeds for an unmapped VA
- translate(va) returns the PA after mapping
- unmap_page(va) returns Ok and translate() returns None afterwards
- map_page() on an already-mapped VA returns Err(AlreadyMapped)
```

**1.1.4 Stack Allocator** (`[ ] TODO`)
```
- alloc_kernel_stack(65536) returns a StackRegion with top 16-byte aligned
- Writing to the last byte of the stack succeeds
- Writing to the guard page triggers #PF (should_panic variant at bare-metal level is impractical here; use debug assertion + monitor verification instead)
- free_kernel_stack(region) leaves allocators in consistent state
```

**1.4.2 Scheduler tick** (`[ ] TODO`)
```
- After init_scheduler_tick(), TICK_COUNT > 0 after a busy-wait loop
- current_tick() returns a monotonically increasing value across two reads
- ticks_to_ms(APIC_TICKS_PER_MS) == 1 (sanity check on calibration)
```

**1.2.1 Driver state** (`[ ] TODO`)
```
- set_driver_state(42u32), driver_state::<u32>() == Some(&42)
- set_driver_state(42u32), driver_state::<u64>() == None (wrong type)
```

**1.2.2 PCI → DriverManager** (`[ ] TODO`)
```
- After boot, driver_manager has at least one device registered
- The xHCI controller appears in the device list with DeviceClass::UsbController
- Its BAR0 is decoded as a DeviceResource::Memory entry
```

---

## How to Add a Test

### Pattern for a bare-metal test

Add a function to `tests/bare_metal_tests.rs` and call it from `kernel_main`:

```rust
fn test_va_allocator_non_overlapping() {
    use theseus_kernel::memory::va_alloc::KernelVaAllocator;
    // ...
    let a = KernelVaAllocator::global().alloc_va(4096, 4096).unwrap();
    let b = KernelVaAllocator::global().alloc_va(4096, 4096).unwrap();
    assert!(a != b);
    assert!(a + 4096 <= b || b + 4096 <= a); // non-overlapping
}
```

Then in `kernel_main`:
```rust
test_va_allocator_non_overlapping();
```

### Pattern for a post-kernel-init test

Add a function to `tests/kernel_tests.rs` and call it from the init block:

```rust
fn test_runtime_mapper_roundtrip() {
    use theseus_kernel::memory::{physical_memory, runtime_mapper::KernelMapper};
    let pa = physical_memory::alloc_frame().expect("no frames");
    let va = 0xFFFF_9000_0000_0000u64; // or use VA allocator
    let mut mapper = KernelMapper::get();
    mapper.map_page(va, pa, /* flags */ ).expect("map failed");
    assert_eq!(mapper.translate(va), Some(pa));
    mapper.unmap_page(va).expect("unmap failed");
    assert_eq!(mapper.translate(va), None);
    physical_memory::free_frame(pa).unwrap();
}
```

### Pattern for a should_panic test

For cases where a specific panic is the correct outcome, add a new file in `tests/` following the `should_panic.rs` pattern: write a custom `panic_handler` that exits with success, and `kernel_main` that triggers the expected panic.

---

## Running Tests

```bash
# Bare-metal suite (fastest, no kernel init)
make test-bare-metal

# Full kernel-initialized suite (slower, needs full boot)
make test-kernel

# Panic verification suite
make test-panic

# All suites
make test-all

# Timeout override (default is 20s per suite)
make TIMEOUT=60 test-bare-metal
```

**Verify the suites run before trusting them.** As of the audit:
- `make test-bare-metal` — likely passes, but not recently verified
- `make test-kernel` — kernel_tests.rs calls `theseus_kernel::initialize_heap_from_handoff` which may not be a current public API symbol. Needs a build check.
- `run_tests.sh` — outdated, references a `simple` test that doesn't exist. Ignore it.

### First thing to do: verify baseline

```bash
cd /home/emil/.openclaw/workspace/TheseusOS
make test-bare-metal  # should pass (1+1=2 level)
make test-kernel      # may fail if public API doesn't match
make test-panic       # should pass
```

Fix any build errors before adding new tests.

---

## Agent Workflow: Tests as Part of a Task

When an agent implements a Phase 1 task, the flow should be:

1. **Before writing code:** Read the task's Completion Criteria in `phase1-cpu-platform.md`
2. **Write the implementation**
3. **Write at least one test** exercising the completion criterion
4. **Run `make test-bare-metal` or `make test-kernel`** — must pass before PR
5. **Include test output** (or a log excerpt) in the PR description

Prompt addition for agent tasks:
```
After implementing, write at least one test in tests/bare_metal_tests.rs or 
tests/kernel_tests.rs that exercises the completion criterion. 
Run `make test-bare-metal` (or test-kernel) and include the output showing it passes.
```

---

## Longer-Term Testing Goals

These are not Phase 1 work, but worth knowing about:

- **Monitor-driven integration tests:** The debug monitor can be scripted via QMP/serial. A test harness could: boot kernel, send monitor commands via serial, parse output, assert on results. This is the right approach for testing things that are too complex to assert in a unit test (e.g., "does the APIC timer calibrate to a sane value?").
- **Fuzzing memory management:** Once we have a runtime mapper and stack allocator, fuzz the allocation/free paths for double-frees and aliasing bugs.
- **CI via GitHub Actions:** Run `make test-bare-metal` on every PR push. QEMU is available in standard GitHub Actions runners. Needs a workflow file.
- **Per-subsystem AGENTS.md:** As subsystems grow, add scoped `AGENTS.md` files in e.g. `kernel/src/memory/` with local testing expectations.
