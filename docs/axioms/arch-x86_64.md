# x86_64 Architecture Axioms

Binding truths for CPU mode, descriptor tables, and interrupt-controller assumptions.

## A1: The kernel is x86_64 `no_std` code using the x86-interrupt ABI

**REQUIRED**

The kernel is built as `no_std` x86_64 code and uses the nightly `abi_x86_interrupt` feature for IDT handlers.

Implements / evidence:
- `kernel/src/lib.rs`
- `kernel/src/interrupts/handlers.rs`

Related plans:
- `../plans/interrupts-and-platform.md#implemented-invariants`

Affected modules:
- `kernel/src/lib.rs`
- `kernel/src/interrupts/*`

## A2: GDT/TSS setup provides dedicated IST stacks for critical faults

**REQUIRED**

The runtime GDT/TSS setup includes dedicated IST entries for at least double fault, NMI, machine check, and page fault handling, and the bring-up path reloads the runtime GDT/TSS state before those fault handlers are relied upon.

Implements / evidence:
- `kernel/src/gdt.rs#build_gdt_state`
- `kernel/src/gdt.rs#refresh_tss_ist`

Related plans:
- `../plans/interrupts-and-platform.md#implemented-invariants`

Affected modules:
- `kernel/src/gdt.rs`
- `kernel/src/interrupts/*`
- `kernel/src/environment.rs`

## A3: Interrupt delivery is APIC-based during kernel bring-up, with legacy PIC masked

**REQUIRED**

The interrupt bring-up path masks the legacy 8259 PIC and uses LAPIC/IOAPIC infrastructure for runtime interrupt delivery.

Implements / evidence:
- `kernel/src/interrupts/apic.rs#disable_all_interrupts`
- `kernel/src/drivers/serial.rs#program_io_apic_entry`
- `kernel/src/environment.rs#continue_after_stack_switch`

Related plans:
- `../plans/interrupts-and-platform.md#implemented-invariants`

Affected modules:
- `kernel/src/interrupts/apic.rs`
- `kernel/src/drivers/serial.rs`
- `kernel/src/environment.rs`

## A4: SMP discovery exists, but AP startup is not yet a documented implemented invariant

**REQUIRED**

The current repo parses ACPI processor/APIC topology and records CPU counts, but it does not yet document an implemented AP startup path.

Implements / evidence:
- `kernel/src/acpi/mod.rs#initialize_acpi`
- `kernel/src/acpi/madt.rs#parse_madt`

Related plans:
- `../plans/interrupts-and-platform.md#todo`

Affected modules:
- `kernel/src/acpi/*`
- `kernel/src/drivers/system.rs`
