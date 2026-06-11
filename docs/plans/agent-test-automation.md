# Agent Test Automation

How to run feature tests automatically and let an agent (or human) get back
a clear PASS/FAIL/PANIC/TIMEOUT verdict without reading QEMU output.

**Status:** Implemented  
**Supersedes:** `testing.md` sections on test execution workflow  
**Dependencies:** `testing.md` (kernel-side test infrastructure)

---

## 1. Problem

The existing `testing.md` defines how to write Tier 1 in-kernel tests, but
leaves the **execution automation** to manual QEMU invocation. An agent
implementing a feature must:

1. Build with `--features kernel-tests`
2. Parse QEMU serial/debugcon output looking for PASS/FAIL lines
3. Determine overall success from log output that may be many pages long

This costs tokens and is error-prone. The goal: **the agent asks for a test
result and gets back one of PASS / FAIL / PANIC / TIMEOUT as a structured exit
code, without reading a single line of output.**

---

## 2. Exit Code Scheme

The `theseus-qemu test` subcommand always returns one of four exit codes.
This is the only signal the agent reads — no output parsing required.

| Exit Code | Verdict  | Agent Interpret | Action |
|-----------|----------|-----------------|--------|
| 0         | PASS     | ✅ All tests green | Done, report |
| 1         | FAIL     | ❌ Test(s) failed  | Read `.test-output.log` to see which one |
| 2         | PANIC    | 💥 Kernel crash   | Kernel panicked or triple-faulted |
| 3         | TIMEOUT  | ⏰ Hung or deadlock | Kernel didn't finish in time |

The exit code is the sole verdict. No output parsing needed.

### How It Works (Internals)

The kernel test runner writes a distinct raw value via `isa-debug-exit`
at port `0xf4`. The ISA device transforms: `qemu_exit = (raw_val << 1) | 1`.
`theseus-qemu test` reverses the transform to identify the verdict:

| Kernel Writes | Raw Val | QEMU Exits | Tool Maps To |
|---------------|---------|------------|--------------|
| `TEST_PASS`   | 3       | 7          | exit 0 (PASS) |
| `TEST_FAIL`   | 4       | 9          | exit 1 (FAIL) |
| `QEMU_ERROR` (panic handler) | 1 | 3   | exit 2 (PANIC) |
| timeout(1)    | —       | 124        | exit 3 (TIMEOUT) |

These values are distinct from the normal boot flow exit codes, so there
is never ambiguity between "the kernel booted successfully" and "the tests
passed".

---

## 3. TDD Workflow (Mandatory Order)

This is **test-driven development** for kernel features. The order is
critical and non-negotiable.

### The Full Cycle

**Step 1 — Baseline: confirm existing tests PASS**

```bash
cargo run -p theseus-qemu -- test
# → "TEST PASS", exit 0
```

Start from a known-good state. If existing tests fail, something is already
broken — do not proceed.

**Step 2 — Define: write the test for the feature's success criteria**

Add tests in a `#[cfg(feature = "kernel-tests")]` module. Each test is
`fn() -> Result<(), &'static str>`. Register it in the `run_kernel_tests()`
call in `after_high_half_entry()`.

**Step 3 — Red: confirm the new test FAILS on a clean build**

```bash
cargo run -p theseus-qemu -- test
# → "TEST FAIL", exit 1
```

This proves the feature does **not** already exist and the test is **actually
testing something**. If step 3 produces PASS, the test is too loose or the
feature already exists — do not proceed until the test correctly FAILS.

**Step 4 — Implement: write the feature code**

All implementation happens here, after the definition of done is locked in.
**The test cannot be changed now** — it is the truth.

**Step 5 — Green: confirm the new test PASSES and existing tests still PASS**

```bash
cargo run -p theseus-qemu -- test
# → "TEST PASS", exit 0
```

Both conditions must hold. If the new test passes but an old one broke,
that is a REGRESSION and counts as FAIL.

**Step 6 — Done: agent reports PASS with evidence**

```
## Verification
Tests ran: `cargo run -p theseus-qemu -- test`
Verdict: PASS (exit code 0)
```

### Why This Order Matters

If a model writes the test *after* implementation, it can slant the test to
match whatever it built (or failed to build). By writing and **confirming the
test fails** before writing any implementation code, the definition of done
is locked in before the model can rationalize its way out.

This workflow prevents:
- Redefining success criteria to match partial results
- Writing stubs that report success without implementing the real logic
- Promising to "do it properly later"
- Claiming the feature was already partially working

The test cannot be changed once step 3 confirms it fails on clean code.

---

## 4. Kernel Test Infrastructure

### Test registration

Tests are registered in a static array passed to `crate::testing::run_kernel_tests()`.
The call site lives in `kernel/src/environment.rs` in `after_high_half_entry()`,
right before the idle loop decision point:

```rust
#[cfg(feature = "kernel-tests")]
{
    let test_list: &[crate::testing::Test] = &[
        // add tests here
    ];
    crate::testing::run_kernel_tests(test_list);
}
```

### Test structure

Each test is a `Test` with a name and a `fn() -> Result<(), &'static str>`:

```rust
Test {
    name: "my_feature_basic",
    func: || -> Result<(), &'static str> {
        // Assert the feature works
        Ok(())
    },
}
```

- Return `Ok(())` → PASS
- Return `Err("reason")` → FAIL
- Panic → The existing kernel panic handler exits QEMU with exit code 1
  (interpreted as FAIL by the tooling)

### Feature flag

The `kernel-tests` feature is defined in `kernel/Cargo.toml` and propagated
through `bootloader/Cargo.toml`:

```toml
# kernel/Cargo.toml
[features]
kernel-tests = []

# bootloader/Cargo.toml
[features]
kernel-tests = ["theseus-kernel/kernel-tests"]
```

---

## 5. The `theseus-qemu test` Subcommand

```bash
# Run all kernel tests
cargo run -p theseus-qemu -- test

# Run with custom timeout (default: 60s)
cargo run -p theseus-qemu -- test --timeout 120

# Print QEMU output to stdout after run
cargo run -p theseus-qemu -- test --print

# Skip the build step
cargo run -p theseus-qemu -- test --no-build
```

**Behaviour:**

1. Builds the project with `FEATURES=kernel-tests` via `make all` (unless `--no-build`)
2. Runs QEMU with `--headless`, `Min` profile, and the configured `--timeout`
3. Waits for QEMU to exit
4. Prints one-line verdict to stdout:
   - `TEST PASS` (exit 0)
   - `TEST FAIL` (exit 1)
   - `KERNEL PANIC` (exit 2)
   - `TIMEOUT` (exit 3)
5. Saves `.test-output.log` on any non-PASS exit for agent inspection
6. If `--print` is provided, dumps full QEMU output to stdout/stderr

**Implementation:** `tools/theseus-qemu/src/main.rs` — `Cmd::Test`, `TestArgs`, `TestVerdict` enum, `run_kernel_tests()` function.

### `make test`

```bash
make test
```

Calls `cargo run -p theseus-qemu -- test`. Supports `--print` and `--timeout` via
direct cargo args.

---

## 6. Exit Code Details

### Exit code 0 — TEST PASS

All tests passed. No `.test-output.log` is saved (no need). The agent can
report success immediately.

### Exit code 1 — TEST FAIL

At least one test returned `Err(...)` (the kernel wrote `TEST_FAIL` raw value 4
to port 0xf4, causing QEMU to exit with code 9 — the tool maps this to exit 1).
The `.test-output.log` contains the kernel debugcon output and should be
inspected to find the failing test. Search for `FAIL —` in the log.

### Exit code 2 — KERNEL PANIC

The kernel panic handler calls `qemu_exit_error!()` (writes `QEMU_ERROR` = 1
to port 0xf4, causing QEMU to exit with code `(1<<1)|1 = 3` — the tool maps
this to exit 2). Also covers triple faults.

### Exit code 3 — TIMEOUT

QEMU was killed by `timeout(1)` after the configured timeout. This usually
indicates:
- The kernel entered the idle loop instead of running tests (feature flag
  not enabled in build)
- A deadlock or infinite loop in kernel bring-up
- A test that hangs instead of panicking

Check `.test-output.log` to see what the kernel printed before the timeout.

---

## 7. Implementation History

### v1: Initial infrastructure (this branch)

- `shared/src/constants.rs`: `QEMU_EXIT_TEST_PASS` (3) and `QEMU_EXIT_TEST_FAIL` (4)
  — distinct raw values for ISA debug exit, transformed to QEMU exits 7 and 9
- `shared/src/macros.rs`: `qemu_exit_test_pass!()` and `qemu_exit_test_fail!()` macros
- `kernel/src/testing.rs`: Kernel-side test runner module, uses the test-specific exit macros
- `tools/theseus-qemu/src/main.rs`: `test` subcommand with `TestVerdict` exit code mapping
  (raw QEMU exit → 0/1/2/3), no debugcon string parsing
- `kernel/Cargo.toml` + `bootloader/Cargo.toml`: `kernel-tests` feature
- `Makefile`: `make test` target
- `docs/agent-prompt-template.md`: TDD workflow instruction block added
- `docs/map.md`: Updated with new plans and modules

### v2 (future): Integration tests

For tests that need external verification — "does a keyboard event arrive?"
or "is the framebuffer drawing correctly?" — the QMP/serial/debugcon sockets
provide the plumbing. A future `make verify` target could:

```bash
# Boot kernel, wait for ready marker, send QMP commands, check output
cargo run -p theseus-qemu -- --headless --qmp --serial unix &
# Script queries via QMP, parses serial output, returns PASS/FAIL
```

Phase C (integration tests) is deferred until we need tests that can't be
expressed as in-kernel assertions.

---

## 8. Relationship with `testing.md`

| Aspect | `testing.md` (existing) | This doc |
|--------|------------------------|----------|
| Test *writing* | Yes — test modules, runner | Defers to `testing.md` |
| Test *registration* | Yes — list in `run_kernel_tests()` | Uses static `&[Test]` array |
| Test *execution* | Manual — run with flags | Automated — `theseus-qemu test` |
| Test *result* | Must read log output | Exit code only |
| Agent workflow | Not addressed | Primary design target |

This doc **adds to** `testing.md` rather than rewriting it.

---

## Checklist

- [x] `theseus-qemu test` subcommand implemented
- [x] `make test` target in Makefile
- [ ] Confirmed exit code 0 on clean test suite (no tests yet — no-op PASS)
- [ ] Confirmed exit code 1 on failing test
- [ ] Confirmed exit code 2 on kernel panic
- [ ] Confirmed exit code 3 on timeout
- [x] `.test-output.log` saved on failure
- [x] Agent prompt template updated
- [x] `docs/map.md` updated
- [ ] Phase C: deferred until needed
