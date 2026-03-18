# AGENTS.md

Repository-wide operating rules for AI models working on TheseusOS.

This file is a **working guide for making good changes safely**. It tells you how to get context, where architectural truth lives, how to update docs when code changes, and how to avoid making the system noisier or more confusing.

## 1. Core Philosophy

- TheseusOS currently boots as a **single UEFI executable**.
- The codebase is `no_std` unless a file explicitly documents otherwise.
- **Reality beats intention.** Do not describe designs that do not exist in the code.
- **Invariants are architecture.** They are not optional commentary.
- **Unsafe code requires explicit contracts.** If an unsafe block depends on a mapping, ownership, interrupt-state, or ABI assumption, say so.
- **Small, reviewable diffs win.** Avoid mixing logic changes, doc rewrites, drive-by renames, and formatting churn in one patch.

## 2. Start Here: How to Get the Right Context

Before making architectural or runtime-sensitive changes, read the repo docs in this order:

1. `docs/index.md` — front door / navigation
2. `docs/bootstrap-status.md` — what the documentation bootstrap completed vs. what is just maintenance
3. `docs/_inventory.md` — evidence-only implementation facts
4. Relevant files in `docs/axioms/` — binding truths the current code relies on
5. Relevant files in `docs/plans/` — status, risks, mismatch notes, and intended sequencing
6. `docs/map.md` — plan ↔ axiom ↔ module cross-reference
7. `docs/development-and-debugging.md` and `docs/logging.md` — workflow and logging policy

### Fast path by task type

- **Boot / handoff / UEFI**
  - `docs/axioms/boot.md`
  - `docs/plans/boot-flow.md`
  - `docs/plans/memory.md`
- **Paging / heap / frame allocation / mappings**
  - `docs/axioms/memory.md`
  - `docs/plans/memory.md`
- **GDT / IDT / APIC / ACPI / timer / interrupt bring-up**
  - `docs/axioms/arch-x86_64.md`
  - `docs/plans/interrupts-and-platform.md`
- **Logging / panic / monitor / serial / QEMU debug**
  - `docs/axioms/debug.md`
  - `docs/plans/observability.md`
  - `docs/logging.md`
- **Drivers / PCI / USB / input / monitor device views**
  - `docs/plans/drivers-and-io.md`
  - `docs/map.md`

## 3. Sources of Truth: What Each Doc Means

### `docs/_inventory.md`

Use this as an **evidence dump**, not as the architecture spec.
It is for answering questions like:

- where does boot start?
- when is `ExitBootServices` called?
- what symbol currently initializes paging?
- where does the memory map get remapped?

It should cite real files and symbols.
It is not the place to define policy or invariants.

### `docs/axioms/`

Axioms are **binding architectural truths**.

Use axioms for statements like:

- the kernel boots as a single UEFI executable
- boot services are exited before `kernel_entry`
- higher-half / `PHYS_OFFSET` assumptions
- logging / panic / monitor invariants

If code changes invalidate an axiom, update the axiom in the same change.

### `docs/plans/`

Plans are for:

- checklists
- mismatch notes
- risks / footguns
- sequencing / transition work
- intent that has not yet become global truth

Checklist markers:

- `[ ] TODO:` not yet true
- `[~] IN PROGRESS:` actively being changed
- `[x] IMPLEMENTED:` now true in the code
- `[!] RISK:` current hazard or mismatch

If an item becomes implemented reality, rewrite it in present tense.

### `docs/map.md`

This is the synchronization surface that links:

- plans → modules
- modules → governing plans
- axioms → affected modules

If you move code or change ownership, update the map.

## 4. Non-Negotiable Rules

- Do **not** invent architecture, APIs, or guarantees.
- Every architectural claim should be backed by repo evidence: file path + symbol/section.
- When reality is unclear, write `TODO`, `UNKNOWN`, or a short mismatch note instead of bluffing.
- Do **not** mix logic changes into purely mechanical doc/header cleanup unless the task explicitly calls for both.
- Keep narrative docs subordinate to axioms/plans if there is a conflict.
- Treat `docs/archive/` as historical material, not current authority.
- Do autonomous work on a **feature branch**, not directly on `main`.
- Keep `main` stable and reviewable; when a feature or fix is ready, open a PR instead of treating local branch work as implicitly mergeable.

## 5. Before Writing Code

Before changing logic in any meaningful way:

1. Identify the governing axiom sections.
2. Identify the governing plan sections.
3. Restate the invariants your change must preserve.
4. Check the relevant module rustdoc headers for local safety/invariant notes.
5. If the area is bring-up-sensitive, inspect the actual code paths too — not just the docs.

Good examples:

- changing boot handoff fields → read `shared/src/handoff.rs`, `bootloader/src/boot_sequence.rs`, `kernel/src/handoff.rs`, plus `docs/axioms/boot.md` and `docs/plans/boot-flow.md`
- changing paging setup → read `kernel/src/memory.rs`, submodules under `kernel/src/memory/`, `docs/axioms/memory.md`, `docs/plans/memory.md`
- changing logging output → read `kernel/src/logging/*`, `kernel/src/panic.rs`, `docs/axioms/debug.md`, `docs/plans/observability.md`, `docs/logging.md`

## 6. After Writing Code

Any architectural or runtime-sensitive code change must also update docs.

Minimum synchronization checklist:

1. Update relevant checklist items in `docs/plans/`
2. Update `docs/axioms/` if a binding truth changed
3. Update `docs/map.md` if module ownership / relationships changed
4. Keep affected Rust module doc headers accurate
5. If you changed a debugging workflow, runner behavior, or output policy, update `docs/development-and-debugging.md` and/or `docs/logging.md`

If you changed no architectural truth, say so in the PR/commit summary rather than silently leaving stale docs behind.

## 7. Rust Module Documentation Rules

Architecture-bearing Rust modules should begin with the standard header shape:

```rust
//! Module: <name>
//!
//! SOURCE OF TRUTH:
//! - docs/plans/<plan>.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/<file>.md#A1:...
//!
//! INVARIANTS:
//! - ...
//!
//! SAFETY:
//! - ...
//!
//! PROGRESS:
//! - docs/plans/<plan>.md
```

Use this header to document:

- which plans govern the module
- which axioms the module relies on
- what invariants must remain true
- what safety assumptions matter for unsafe code / interrupts / raw pointers / mappings

Do **not** let these headers drift into vague essays.
Keep them crisp and mechanically useful.

## 8. Code Writing Rules

### Unsafe code

Every unsafe change should make the contract clearer, not murkier.
Document assumptions around:

- pointer validity
- alignment
- memory mapping state
- interrupt state / reentrancy
- exclusivity / aliasing
- ABI / calling convention
- MMIO and port-I/O ordering assumptions

### Mechanical refactors

Mechanical refactors should not change behavior.
If you are only moving comments/docs/headers around, keep logic untouched.

### New abstractions

Do not create abstractions just because they feel elegant.
Prefer abstractions that:

- match current repo reality
- reduce bring-up risk
- improve inspectability/debuggability
- make invariants easier to state and verify

### TODO discipline

A good TODO explains what is unknown or deferred.
A bad TODO handwaves.

Prefer:

- what is currently true
- what is missing
- what evidence would resolve it

## 9. Logging and Logspam Rules

TheseusOS depends heavily on logs during bring-up, so logging changes need discipline.
Read `docs/logging.md` before changing output behavior.

### The default goal

Default boot output should stay **high-signal**.
That means:

- **ERROR/WARN** should stand out and usually reach serial and QEMU debug output
- **INFO** should be for major milestones only
- **DEBUG/TRACE** should be opt-in and normally biased toward the QEMU debug path, not default serial spam

### Do not introduce extra logspam

Avoid:

- per-iteration logs in hot loops
- repeated warnings in interrupt paths
- verbose dumps on every boot by default
- large device-table or descriptor dumps at INFO level
- “temporary” debug prints left enabled globally

Prefer:

- one-shot logs
- summarized counts / state snapshots
- rate-limited warnings
- module-scoped debug overrides
- DEBUG/TRACE instead of INFO for detailed state

### Logging level guidance

- `ERROR`: system is broken or correctness is compromised
- `WARN`: unexpected but recoverable
- `INFO`: major milestones only
- `DEBUG`: focused debugging detail
- `TRACE`: extremely chatty instrumentation; never default-on

### Output routing guidance

Current policy is roughly:

- ERROR/WARN → visible on serial and QEMU debug by default
- INFO/DEBUG/TRACE → usually QEMU debug first, unless explicitly routed otherwise

If you change that policy, update:

- `docs/logging.md`
- `docs/plans/observability.md`
- any affected code comments / module headers

### Special care areas

Be conservative with logging inside:

- interrupt handlers
- timer paths
- serial RX/TX fast paths
- USB/xHCI event-ring processing
- page-fault / panic-adjacent paths

In those areas, extra logs can:

- hide the real bug
- perturb timing
- flood QEMU logs
- consume AI context unnecessarily

## 10. Debugging / Workflow Notes for Models

Preferred current run path is the Rust QEMU runner:

- `cargo run -p theseus-qemu -- --headless`
- `cargo run -p theseus-qemu -- print --headless`
- `cargo run -p theseus-qemu -- --relays --headless`

The old `startQemu.sh` path still exists, but it is not the preferred front door.

When debugging:

- prefer reproducible runner/config changes over ad-hoc one-off shell hacks
- keep QEMU debug flags minimal by default
- use monitor commands and targeted logging before adding new spammy tracing
- if a change affects logging or monitor behavior, document how to exercise it

## 11. When to Update Which Docs

### Update `docs/axioms/*` when...

- a global truth changed
- a boot contract changed
- a paging / heap / PHYS_OFFSET invariant changed
- a panic/logging/runtime-inspection invariant changed
- a platform assumption became newly binding

### Update `docs/plans/*` when...

- implementation status changed
- a risk was discovered/resolved
- an old plan statement no longer matches the repo
- work moved from intent to implemented reality

### Update `docs/_inventory.md` when...

- a new fact needs evidence capture
- an implementation path moved enough that symbol/file references changed
- the inventory would otherwise become misleading

### Update `docs/development-and-debugging.md` / `docs/logging.md` when...

- workflow changed
- QEMU runner usage changed
- logging defaults or routing changed
- monitor/debugging ergonomics changed

## 12. If You Are Unsure

Stop and ask if any of these are ambiguous:

- whether a statement is current truth or future intent
- whether a change should be treated as architectural vs local
- whether a logging change is acceptable at default verbosity
- whether a doc belongs in axioms, plans, inventory, or archive
- whether a risky refactor should be split into smaller patches

When in doubt: **be evidence-first, keep the diff small, and leave a precise note rather than guessing.**

## 13. Final Principle

This repository is designed so smaller models and future humans can work safely.
Your job is not just to change code.
Your job is to preserve and improve the clarity of:

- what is true
- what is planned
- what is risky
- and how the next person can verify all of it

Scoped `AGENTS.md` files may add stricter local rules, but they must not contradict this file.
