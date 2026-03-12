# Overview

TheseusOS is a teaching-focused x86_64 operating system written in Rust. The codebase demonstrates how to stitch together a modern UEFI boot flow, higher-half kernel, and device/interrupt infrastructure using safe abstractions where possible and carefully audited `unsafe` when required. The bootloader and kernel are built as a *single binary* so you can step through the entire bring-up path without context switches or ELF parsing.

## Project Goals
- **Rust fluency** — idiomatic systems Rust with controlled `unsafe` blocks and strong module boundaries.
- **OS theory in practice** — paging, interrupts, drivers, and kernel services that align with classic textbook material.
- **x86_64 familiarity** — concrete exposure to control registers, APIC programming, model specific registers, and UEFI firmware.

## Current Capabilities
- UEFI bootloader that gathers graphics, ACPI, CPU, firmware, and memory map data before calling `kernel_entry`.
- Higher-half kernel that installs its own GDT/IDT, brings up LAPIC timer interrupts, and manages logging, drivers, and memory.
- Deterministic virtual memory layout with a `PHYS_OFFSET` window, kernel heap bootstrap, and MMIO mappings.
- Serial-first tooling: unified logging, COM1 interrupt handler, and an interactive Wozmon-inspired monitor.
- QEMU-friendly workflows with `theseus-qemu` as the preferred current path, while some older helper scripts and debug plumbing still remain in-repo.

> For binding architectural truth, prefer `docs/axioms/` and `docs/plans/` over this narrative overview.

## Suggested Learning Path
1. Read the [Boot Sequence & Handoff](boot-sequence.md) guide to understand how the UEFI phase gathers information.
2. Continue with [Kernel Architecture](kernel-architecture.md) to see how control transitions to Rust code after `ExitBootServices`.
3. Deep dive into [Memory Management](memory-management.md) to understand paging and frame allocation.
4. Explore [Hardware & Drivers](hardware-and-drivers.md) and [Development & Debugging](development-and-debugging.md) when you are ready to experiment.

The docs are cross-linked like a wiki, but prefer the axioms/plans split whenever a narrative page sounds smoother than the code really is.
