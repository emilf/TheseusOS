Here’s a clean document you can save as e.g. `openclaw-heartbeat-and-tests.md`.

---

# OpenClaw Heartbeat + Test Strategy Notes

## Goals

* Let OpenClaw explore and contribute autonomously without destabilizing main.
* Make progress “diffable” and “reviewable,” not vibes-based.
* Build a test harness that cannot accidentally entangle with normal boot.
* Reduce manual QA by turning QEMU runs into structured artifacts (logs + exit codes).

---

## Repo Structure and Branch Discipline

### Branches

* `main`: **protected**. No direct commits by OpenClaw. Human-reviewed merges only.
* `feature/<topic>`: all autonomous work goes here.
* Optional: `exp/<topic>` for “wild” exploration that’s allowed to be messy.

### Commit rules

* Small, atomic commits. One subsystem change per commit.
* Each commit message includes:

  * What changed
  * Why (invariant, bug, plan item)
  * How verified (test name / QEMU run log)

### “No refactors for vibes”

Refactors require a stated measurable reason:

* fixes a bug
* reduces unsafe surface
* enables a test
* enforces an invariant
* improves determinism / logging
  Otherwise: do not do it.

---

## Heartbeat Constitution (put this into the heartbeat prompt)

### Allowed activities per heartbeat

OpenClaw may do **exactly one** of these per cycle:

1. **Implement** the next smallest planned step (in a feature branch), following guidelines below.
2. **Research** one OSdev topic and write structured notes (no code changes unless explicitly allowed).
3. **Triage**: read logs / failures from last run and produce a plan + suspected root cause + next steps.

### Hard constraints

* Never modify `main`.
* Never change boot behavior in normal mode as part of “tests.”
* No speculative hardware poking without a cited spec section or a subsystem contract note.

### Output artifacts required each heartbeat

* A short “heartbeat report” (markdown) containing:

  * What it attempted
  * Why this was the next step
  * What changed (files / branch / commits)
  * How it was verified (logs, exit code, test results)
  * Known uncertainties / TODOs

### Model delegation policy (multi-model society rules)

* **Tool runner model** must handle:

  * running QEMU, collecting logs, parsing outputs, simple repo operations
* **Local coder model** may handle:

  * mechanical code changes with a tight spec (small diff, clear API)
* **Codex (orchestrator)** must handle:

  * spec interpretation, architecture decisions, concurrency, MSI-X/xHCI/APIC/PCIe rules
  * reviewing diffs produced by smaller models
* If uncertain: write a design note instead of guessing.

### “Stop conditions”

If any of the following happens, OpenClaw stops and writes a note instead of continuing:

* ambiguous spec requirement
* more than one plausible implementation choice
* repeated failure without new evidence
* a change would touch init ordering, interrupts, paging, or memory map semantics

---

## Documentation Standards (enforced by heartbeat)

### Always keep these in sync

For every subsystem touched:

* Inline comments for “why,” not “what.”
* Rustdoc for public APIs: invariants, safety, call context.
* A subsystem markdown doc that states:

  * purpose
  * state machine sketch
  * invariants
  * error model
  * what may run in IRQ context
  * what requires single-threading / locks
  * dependencies on other subsystems

### Add a `contract.md` per tricky subsystem

For things like APIC, PCI/MSI-X, xHCI, scheduler, memory manager:

* invariants
* forbidden actions
* initialization sequence
* “observable signals” for tests (counters, trace events, state queries)

---

## OpenClaw Environment and Telemetry (QEMU + tmux)

### Make runs deterministic and artifact-rich

Collect at least:

* serial output (`-serial ...`)
* `isa-debugcon` output (`-debugcon ...` or similar)
* QEMU monitor log (optional)
* exit status signaling pass/fail (e.g. `isa-debug-exit`)

### Log formatting

* Prefix streams when merging logs:

  * `[SER]`, `[DBG]`, `[MON]`
* Keep one merged “timeline” log per run.

### Golden transcript

* Maintain a known-good boot transcript.
* CI / harness diffs current output vs golden with tolerance for timestamps.

### Crash/panic telemetry

Have a consistent panic frame:

* CPU id, RIP, RSP, RFLAGS
* CR2, CR3
* error code
* last N trace events (see trace buffer below)

---

## Tests: Design Rules That Prevent “Harness Ate the Kernel”

### Tests are a boot mode, not a feature

Add a boot param / mode switch:

* `mode=normal`
* `mode=test`
* optional `test=<name>` / `testgroup=<name>`

Early init:

* if test mode: run test runner → print results → exit QEMU pass/fail
* else: normal boot

### Minimal “kernel test API” (freeze this)

Expose only:

* `test_log!()`
* `assert!`, `assert_eq!` (panic goes to test-aware handler)
* `test_pass()` / `test_fail(code)`
* optional `with_timeout(ticks, f)`

No sprawling frameworks. No ad-hoc globals.

### Public-API-only rule

Tests may only call:

* stable public interfaces
* explicit “test hooks” that are part of the public contract

No reaching into internal modules “just for convenience.”

### Test levels (safety tiers)

Tag tests so they don’t accidentally trash hardware state:

* **L0**: pure logic, no hardware (data structures, parsers)
* **L1**: core kernel services (allocator, paging, logging), no device IO
* **L2**: hardware but idempotent (APIC tick counters, HPET reads)
* **L3**: destructive/exclusive (xHCI reset, MSI-X enable, PCI reconfig)

Default run: L0 + L1 only. L2/L3 require explicit opt-in.

### Avoid Heisenbugs: trace buffer

Add a lightweight trace ring buffer:

* event id + timestamp + a couple payload u64s
* dump only on failure (or end of run)
  This avoids serial logging altering timing during IRQ-heavy tests.

### Result signaling

* Exit QEMU with deterministic status:

  * pass → code 0 / known value
  * fail → nonzero / specific code per failing test

---

## Planning Workflow (keeps AI “on the rails”)

### Feature plan format

For each feature/subsystem, maintain a living plan that includes:

* prerequisites
* next smallest step
* acceptance criteria
* observability needed (what will prove it works?)
* test idea(s) per step

Update the plan in real time as steps are implemented.

### “Acceptance criteria first”

Before coding, the orchestrator writes:

* what observable will change
* what log line / counter / state query proves success
* what would falsify it

This prevents “tests that test nothing” and “features that never converge.”

---

## Metrics for the Experiment (optional but fun/useful)

Track per heartbeat:

* which model did what (Codex vs local coder vs tool runner)
* diff size and files touched
* pass/fail and which tests ran
* number of reversions / rejected PRs
* time-to-merge per task type

This tells you whether the system is learning to delegate and whether autonomy is net-positive.

---

## Immediate TODO Checklist (practical next steps)

1. Implement `mode=test` and a tiny test runner skeleton.
2. Add QEMU exit signaling for pass/fail.
3. Add merged-log capture (serial + debugcon at minimum).
4. Add a trace ring buffer and dump-on-fail.
5. Establish golden boot transcript + diffing.
6. Write `contract.md` for the next “hard” subsystem you’ll tackle (MSI-X or xHCI).
7. Embed the heartbeat constitution + delegation policy into the heartbeat prompt.
8. Enforce branch discipline: autonomous work only in `feature/*`.

---

If you want, you can paste your current heartbeat prompt and I’ll rewrite it into a “constitutional” version that includes these rules (same intent, just engineered to be hard for the model to misinterpret).

