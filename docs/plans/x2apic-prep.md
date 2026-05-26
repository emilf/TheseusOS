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
- [x] IMPLEMENTED: Audit every APIC-facing call site for xAPIC-only assumptions, especially:
  - LAPIC timer configuration/start/mask helpers
  - EOI writes in interrupt handlers
  - APIC ID reads used by serial/USB IRQ routing
  - any direct assumptions about LAPIC base or ID register layout
- [x] IMPLEMENTED: The first audited xAPIC-only touchpoints have been recorded and partially centralized:
  - EOI writes in `kernel/src/interrupts/handlers.rs` now flow through `kernel/src/interrupts/apic.rs::local_apic_eoi`
  - APIC ID reads in `kernel/src/drivers/serial.rs` and `kernel/src/drivers/usb/xhci/mod.rs` now flow through `kernel/src/interrupts/apic.rs::local_apic_id`
  - LAPIC timer register access in `kernel/src/interrupts/timer.rs` now flows through `kernel/src/interrupts/apic.rs::{local_apic_read, local_apic_write}` instead of open-coding the xAPIC access path at each call site
- [x] IMPLEMENTED: All LAPIC helpers (`local_apic_read/write/eoi/id`) dispatch through `cached_apic_mode()` — confirmed safe for x2APIC mode switch.

## Phase 2 — Structural Cleanup ✅

Goal: isolate APIC access policy from APIC users so later behavior changes are localized.

### Checklist

- [x] IMPLEMENTED: APIC access centralized in `local_apic_read/write/eoi/id` — no open-coded xAPIC assumptions at call sites.
- [x] IMPLEMENTED: `ApicAccessMode` separates "what mode" from "how to access"; `cached_apic_mode()` is the single policy point.
- [x] IMPLEMENTED: APIC ID reads and EOI writes unified through `local_apic_id()` and `local_apic_eoi()`.
- [x] IMPLEMENTED: Phase was behavior-preserving; no silent bad assumptions remain.

## Phase 3 — Observability + Guardrails ✅

Goal: make unsupported APIC-mode combinations obvious instead of silently dangerous.

### Checklist

- [x] IMPLEMENTED: Boot log reports `APIC mode: x2apic` (or xapic/disabled) via `init_apic_mode()`.
- [x] IMPLEMENTED: LAPIC helpers panic on `ApicAccessMode::Disabled` — never silently misfire.
- [x] IMPLEMENTED: Monitor `cpu` and `status` commands expose APIC mode, base, x2apic flag, and BSP bit.
- [x] IMPLEMENTED: x2APIC enable attempt is logged: success → `"x2APIC enabled"`, failure → `"x2APIC enable failed"`, unsupported → `"x2APIC not supported"`, already-on → `"x2APIC already enabled"`.

## Phase 4 — First Narrow Functional x2APIC Step ✅

Goal: if earlier phases leave the code in a clean state, land one very small functional increment.

### Checklist

- [x] IMPLEMENTED: `try_enable_x2apic()` — enables x2APIC mode if CPUID reports support, verifies via readback, logs result. Called before `init_apic_mode()` in `environment.rs`.
- [x] IMPLEMENTED: x2APIC-safe APIC ID retrieval via `local_apic_id()` (MSR 0x820, no 24-bit shift).
- [x] IMPLEMENTED: x2APIC-safe EOI via `local_apic_eoi()` (MSR write to 0x80B offset).
- [x] IMPLEMENTED: x2APIC-safe timer bring-up — `lapic_timer_configure()` and all timer helpers use `local_apic_read/write()` which dispatches through mode cache.
- [x] VERIFIED: Full boot in QEMU with `-cpu max` (x2APIC advertised): `x2APIC enabled` → `APIC mode: x2apic` → `LAPIC timer interrupt received successfully` — complete boot including ACPI, USB/xHCI enumeration.

**Confirmed non-goals kept out of scope:**
- IOAPIC / MSI-X mode-switching (separate concern)
- IPI support via 64-bit ICR MSR (noted in `local_apic_write` comment, deferred to SMP bringup)
- Full IOAPIC/MSI path conversion

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
