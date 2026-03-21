# x2APIC Preparation Plan

## Scope

Track the staged work needed before TheseusOS can safely consider switching any runtime interrupt path from xAPIC MMIO access to x2APIC MSR-based access.

This plan is intentionally narrower than "implement x2APIC." It focuses first on:

- truth capture
- access-path cleanup
- observability
- guardrails against accidentally mixing unsupported APIC modes with xAPIC-only assumptions

## Plan vs Repo

- The repo already detects x2APIC capability via CPUID and now reports current APIC mode/base state via `IA32_APIC_BASE`.
- The real runtime interrupt path is still xAPIC/MMIO-oriented:
  - LAPIC register access uses MMIO through `PHYS_OFFSET`
  - timer setup, EOI writes, and APIC ID reads all assume xAPIC-style register access
- That means the codebase is now better at **reporting** x2APIC state than **operating** in x2APIC mode, which is exactly where we want to be before any mode switch.

## Current Implemented Invariants

- [x] IMPLEMENTED: CPUID/monitor tooling can report x2APIC capability separately from general APIC presence.
- [x] IMPLEMENTED: `IA32_APIC_BASE` is decoded into a structured runtime view that reports LAPIC base, global enable, BSP bit, and whether x2APIC is currently enabled.
- [x] IMPLEMENTED: The current mainline interrupt path still uses xAPIC-style MMIO register access for LAPIC operations.
- [x] IMPLEMENTED: Monitor `status` and `cpuid` now expose the distinction between x2APIC capability and x2APIC enablement.

## Phase 1 — Recon + Truth Capture

Goal: enumerate exactly where the current runtime assumes xAPIC/MMIO semantics.

### Checklist

- [x] IMPLEMENTED: `IA32_APIC_BASE` handling has been surfaced explicitly so current APIC mode/base state is observable.
- [~] IN PROGRESS: Audit every APIC-facing call site for xAPIC-only assumptions, especially:
  - LAPIC timer configuration/start/mask helpers
  - EOI writes in interrupt handlers
  - APIC ID reads used by serial/USB IRQ routing
  - any direct assumptions about LAPIC base or ID register layout
- [x] IMPLEMENTED: The first audited xAPIC-only touchpoints have been recorded and partially centralized:
  - EOI writes in `kernel/src/interrupts/handlers.rs` now flow through `kernel/src/interrupts/apic.rs::local_apic_eoi`
  - APIC ID reads in `kernel/src/drivers/serial.rs` and `kernel/src/drivers/usb/xhci/mod.rs` now flow through `kernel/src/interrupts/apic.rs::local_apic_id`
  - LAPIC timer register access in `kernel/src/interrupts/timer.rs` now flows through `kernel/src/interrupts/apic.rs::{local_apic_read, local_apic_write}` instead of open-coding the xAPIC access path at each call site
- [ ] TODO: Confirm whether any current helper already assumes x2APIC-style semantics implicitly or would become wrong immediately after an x2APIC mode switch.

## Phase 2 — Structural Cleanup

Goal: isolate APIC access policy from APIC users so later behavior changes are localized.

### Checklist

- [ ] TODO: Refactor APIC access so call sites depend on a smaller helper surface instead of open-coding xAPIC assumptions.
- [ ] TODO: Separate "what APIC mode are we in?" from "how do we read/write a LAPIC register in that mode?"
- [ ] TODO: Reduce duplication around APIC ID reads and EOI writes so any future mode-aware access path has fewer call sites to update.
- [ ] TODO: Keep this phase behavior-preserving unless a tiny guardrail change is clearly safer than preserving a silent bad assumption.

## Phase 3 — Observability + Guardrails

Goal: make unsupported APIC-mode combinations obvious instead of silently dangerous.

### Checklist

- [ ] TODO: Add explicit runtime reporting for APIC access support, not just APIC capability/mode.
- [x] IMPLEMENTED: The shared LAPIC access helpers (`local_apic_read/write`, `local_apic_eoi`, `local_apic_id`) now hard-fail if `IA32_APIC_BASE` reports x2APIC enabled, so we never silently perform xAPIC/MMIO accesses in an incompatible mode.
- [ ] TODO: Extend monitor/debugging surfaces if needed so APIC mode transitions can be inspected without adding default boot logspam.

## Phase 4 — First Narrow Functional x2APIC Step

Goal: if earlier phases leave the code in a clean state, land one very small functional increment.

### Candidate first increments

- [ ] TODO: x2APIC-safe APIC ID retrieval path
- [ ] TODO: x2APIC-safe EOI path
- [ ] TODO: one or two mode-aware LAPIC register helpers needed for timer bring-up

**Non-goal for the first functional step:**
- full x2APIC conversion of every APIC/IOAPIC/MSI-related path in one patch

## Risks

- [!] RISK: Enabling x2APIC before isolating xAPIC-only MMIO assumptions would likely break LAPIC timer, EOI, or APIC ID paths in ways that are hard to debug.
- [!] RISK: APIC base reporting is now more truthful, but that does not mean the runtime is ready for x2APIC mode changes.
- [!] RISK: Mixing "capability" and "enablement" language would make the docs lie exactly where we need precision most.
- [!] RISK: APIC changes are ordering-sensitive and can fail in ways that look like generic interrupt/timer instability rather than an obvious APIC-mode bug.

## Related Axioms

- `../axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked`
- `../axioms/arch-x86_64.md#A4:-SMP-discovery-exists-but-AP-startup-is-not-yet-a-documented-implemented-invariant`
- `../axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface`

## Implementing / Audit Targets

- `kernel/src/interrupts/apic.rs`
- `kernel/src/interrupts/timer.rs`
- `kernel/src/interrupts/handlers.rs`
- `kernel/src/interrupts/mod.rs`
- `kernel/src/environment.rs`
- `kernel/src/drivers/serial.rs`
- `kernel/src/drivers/usb/xhci/mod.rs`
- `kernel/src/monitor/commands/cpu.rs`
- `kernel/src/monitor/commands/system.rs`
- `docs/_inventory.md`
- `docs/external/specs_and_datasheets/intel cpu manuals 325462-088-sdm-vol-1-2abcd-3abcd-4.pdf`
- `docs/external/specs_and_datasheets/intel multiprocessor specification - APIC info too.pdf`
- `docs/external/specs_and_datasheets/ioapic.pdf`
