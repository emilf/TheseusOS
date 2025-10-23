# TheseusOS Documentation Hub

Welcome! This living wiki covers how the unified TheseusOS bootloader and kernel are structured today. Start with the [Overview](overview.md) for the big picture, then dive into the areas you want to explore.

- [Overview](overview.md) — project goals, architecture snapshot, and suggested learning paths.
- [Boot Sequence & Handoff](boot-sequence.md) — how the UEFI loader prepares the machine and hands control to the kernel.
- [Kernel Architecture](kernel-architecture.md) — tour of the major subsystems that come online during `kernel_entry`.
- [Memory Management](memory-management.md) — virtual layout, frame allocation, and paging helpers.
- [Hardware & Drivers](hardware-and-drivers.md) — device inventory, driver manager, serial stack, and interrupts.
- [Driver Systems Deep Dive](driver-systems-deep-dive.md) — comprehensive technical guide to DMA, PCI discovery, and driver framework.
- [Development & Debugging](development-and-debugging.md) — building, running under QEMU, logging, and the serial monitor.

Looking for historical notes? Everything that pre-dates this rewrite now lives in [docs/archive](archive/).
