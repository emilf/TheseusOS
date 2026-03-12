# Interrupts and Platform Plan

## Scope

Track descriptor-table setup, APIC interrupt routing, timer bring-up, ACPI platform discovery, and SMP-adjacent platform status.

## Plan vs Repo

- The repo clearly implements GDT/TSS/IDT/APIC bring-up, but current docs do not make it obvious which interrupt truths are binding and which SMP pieces are still just discovery.

## Implemented Invariants

- [x] IMPLEMENTED: The kernel installs a GDT/TSS with dedicated IST stacks for double fault, NMI, machine check, and page fault handling.
- [x] IMPLEMENTED: The kernel installs an IDT before the high-half jump and reinstalls it after the stack switch in higher-half execution.
- [x] IMPLEMENTED: The kernel masks the legacy PIC during APIC-based interrupt bring-up.
- [x] IMPLEMENTED: The kernel configures and tests LAPIC timer delivery during bring-up before starting a periodic timer.
- [x] IMPLEMENTED: ACPI parsing records CPU/APIC topology, IO APIC data, and local APIC address into runtime platform structures.
- [x] IMPLEMENTED: The serial driver programs IOAPIC routing for interrupt-driven receive handling.
- [x] IMPLEMENTED: The current CPU setup path detects CPUID features once, applies a limited CR0/CR4 setup during bring-up, and enables SSE/AVX state only when the detected feature set and current setup path allow it.

## TODO

- [ ] TODO: Document an implemented AP startup path if and when one exists in the repo.
- [ ] TODO: Promote any future x2APIC runtime bring-up to an axiom only after the main path depends on it.

## Risks

- [!] RISK: SMP discovery exists, but a documented AP-startup/runtime multi-core bring-up path is not yet an implemented invariant.
- [!] RISK: Descriptor-table and timer correctness currently rely on several staged transitions during high-half bring-up, which makes ordering mistakes expensive.

## Related Axioms

- `../axioms/arch-x86_64.md#A1:-The-kernel-is-x86_64-no_std-code-using-the-x86-interrupt-ABI`
- `../axioms/arch-x86_64.md#A2:-GDT/TSS-setup-provides-dedicated-IST-stacks-for-critical-faults`
- `../axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked`
- `../axioms/arch-x86_64.md#A4:-SMP-discovery-exists-but-AP-startup-is-not-yet-a-documented-implemented-invariant`

## Implementing Modules

- `kernel/src/gdt.rs`
- `kernel/src/interrupts/mod.rs`
- `kernel/src/interrupts/handlers.rs`
- `kernel/src/interrupts/apic.rs`
- `kernel/src/interrupts/timer.rs`
- `kernel/src/acpi/mod.rs`
- `kernel/src/acpi/madt.rs`
- `kernel/src/drivers/serial.rs`
- `kernel/src/environment.rs`
