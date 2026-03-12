# Documentation Map

Bidirectional map between plans, axioms, and implementing modules.

Treat this as a synchronization checklist as much as a navigation aid.

This file is meant to be a maintenance surface, not polished prose: if a module or doc moves, update the mapping here in the same change.

## Plans → Implementations

### `plans/boot-flow.md`

- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `bootloader/src/acpi.rs`
- `bootloader/src/hardware.rs`
- `bootloader/src/display.rs`
- `bootloader/src/system_info.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `bootloader/src/drivers/mod.rs`
- `bootloader/src/drivers/manager.rs`
- `bootloader/src/drivers/qemu_debug.rs`
- `bootloader/src/drivers/raw_serial.rs`
- `bootloader/src/drivers/uefi_serial.rs`
- `shared/src/handoff.rs`
- `kernel/src/lib.rs`
- `kernel/src/handoff.rs`

### `plans/memory.md`

- `bootloader/src/main.rs`
- `bootloader/src/memory.rs`
- `bootloader/src/boot_sequence.rs`
- `shared/src/handoff.rs`
- `kernel/src/memory.rs`
- `kernel/src/memory/mapping.rs`
- `kernel/src/memory/frame_allocator.rs`
- `kernel/src/memory/page_tables.rs`
- `kernel/src/memory/page_table_builder.rs`
- `kernel/src/memory/temporary_window.rs`
- `kernel/src/physical_memory.rs`
- `kernel/src/environment.rs`

### `plans/interrupts-and-platform.md`

- `bootloader/src/acpi.rs`
- `kernel/src/gdt.rs`
- `kernel/src/interrupts/mod.rs`
- `kernel/src/interrupts/handlers.rs`
- `kernel/src/interrupts/apic.rs`
- `kernel/src/interrupts/timer.rs`
- `kernel/src/acpi/mod.rs`
- `kernel/src/acpi/madt.rs`
- `kernel/src/drivers/serial.rs`
- `kernel/src/environment.rs`

### `plans/observability.md`

- `bootloader/src/display.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `bootloader/src/drivers/mod.rs`
- `bootloader/src/drivers/manager.rs`
- `bootloader/src/drivers/qemu_debug.rs`
- `bootloader/src/drivers/raw_serial.rs`
- `bootloader/src/drivers/uefi_serial.rs`
- `kernel/src/lib.rs`
- `kernel/src/logging/mod.rs`
- `kernel/src/logging/output.rs`
- `kernel/src/logging/filter.rs`
- `kernel/src/logging/macros.rs`
- `kernel/src/panic.rs`
- `kernel/src/display.rs`
- `kernel/src/framebuffer.rs`
- `kernel/src/bootlogo.rs`
- `kernel/src/monitor/mod.rs`
- `kernel/src/monitor/parsing.rs`
- `kernel/src/monitor/commands/*`
- `docs/development-and-debugging.md`
- `docs/logging.md`
- `docs/qemu-runner.md`
- `tools/theseus-qemu/src/main.rs`

### `plans/drivers-and-io.md`

- `kernel/src/drivers/mod.rs`
- `kernel/src/drivers/manager.rs`
- `kernel/src/drivers/pci.rs`
- `kernel/src/drivers/system.rs`
- `kernel/src/drivers/traits.rs`
- `kernel/src/drivers/usb/mod.rs`
- `kernel/src/drivers/usb/handoff.rs`
- `kernel/src/drivers/usb/xhci/*`
- `kernel/src/input/*`
- `kernel/src/monitor/mod.rs`
- `kernel/src/monitor/commands/devices.rs`
- `kernel/src/monitor/commands/pci.rs`
- `kernel/src/monitor/commands/usb.rs`
- `docs/hardware-and-drivers.md`
- `docs/archive/driver-systems-deep-dive.md`
- `tools/theseus-qemu/src/main.rs`

## Modules → Governing Plans

### Boot / handoff

- `bootloader/src/main.rs` → `plans/boot-flow.md`
- `bootloader/src/boot_sequence.rs` → `plans/boot-flow.md`
- `bootloader/src/memory.rs` → `plans/boot-flow.md`, `plans/memory.md`
- `bootloader/src/acpi.rs` → `plans/boot-flow.md`, `plans/interrupts-and-platform.md`
- `bootloader/src/system_info.rs` → `plans/boot-flow.md`, `plans/interrupts-and-platform.md`
- `bootloader/src/hardware.rs` → `plans/boot-flow.md`, `plans/drivers-and-io.md`
- `bootloader/src/display.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `bootloader/src/serial.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `bootloader/src/qemu_exit.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `bootloader/src/drivers/mod.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `bootloader/src/drivers/manager.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `bootloader/src/drivers/qemu_debug.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `bootloader/src/drivers/raw_serial.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `bootloader/src/drivers/uefi_serial.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `shared/src/handoff.rs` → `plans/boot-flow.md`, `plans/memory.md`
- `kernel/src/lib.rs` → `plans/boot-flow.md`, `plans/observability.md`
- `kernel/src/handoff.rs` → `plans/boot-flow.md`, `plans/memory.md`

### Memory

- `bootloader/src/main.rs` → `plans/boot-flow.md`, `plans/memory.md`
- `bootloader/src/boot_sequence.rs` → `plans/boot-flow.md`, `plans/memory.md`
- `bootloader/src/memory.rs` → `plans/boot-flow.md`, `plans/memory.md`
- `shared/src/handoff.rs` → `plans/boot-flow.md`, `plans/memory.md`
- `kernel/src/memory.rs` → `plans/memory.md`
- `kernel/src/memory/mapping.rs` → `plans/memory.md`
- `kernel/src/memory/frame_allocator.rs` → `plans/memory.md`
- `kernel/src/memory/page_tables.rs` → `plans/memory.md`
- `kernel/src/memory/page_table_builder.rs` → `plans/memory.md`
- `kernel/src/memory/temporary_window.rs` → `plans/memory.md`
- `kernel/src/physical_memory.rs` → `plans/memory.md`
- `kernel/src/handoff.rs` → `plans/memory.md`
- `kernel/src/environment.rs` → `plans/memory.md`, `plans/interrupts-and-platform.md`

### Platform / interrupts

- `kernel/src/gdt.rs` → `plans/interrupts-and-platform.md`
- `kernel/src/interrupts/mod.rs` → `plans/interrupts-and-platform.md`
- `kernel/src/interrupts/handlers.rs` → `plans/interrupts-and-platform.md`
- `kernel/src/interrupts/apic.rs` → `plans/interrupts-and-platform.md`
- `kernel/src/interrupts/timer.rs` → `plans/interrupts-and-platform.md`
- `kernel/src/acpi/mod.rs` → `plans/interrupts-and-platform.md`
- `kernel/src/acpi/madt.rs` → `plans/interrupts-and-platform.md`
- `kernel/src/drivers/serial.rs` → `plans/interrupts-and-platform.md`, `plans/observability.md`, `plans/drivers-and-io.md`
- `kernel/src/drivers/system.rs` → `plans/interrupts-and-platform.md`, `plans/drivers-and-io.md`

### Drivers / I/O

- `kernel/src/drivers/mod.rs` → `plans/drivers-and-io.md`
- `kernel/src/drivers/manager.rs` → `plans/drivers-and-io.md`
- `kernel/src/drivers/pci.rs` → `plans/drivers-and-io.md`
- `kernel/src/drivers/traits.rs` → `plans/drivers-and-io.md`
- `kernel/src/drivers/usb/mod.rs` → `plans/drivers-and-io.md`
- `kernel/src/drivers/usb/handoff.rs` → `plans/drivers-and-io.md`
- `kernel/src/drivers/usb/xhci/*` → `plans/drivers-and-io.md`
- `kernel/src/input/*` → `plans/drivers-and-io.md`
- `kernel/src/monitor/commands/devices.rs` → `plans/drivers-and-io.md`
- `kernel/src/monitor/commands/pci.rs` → `plans/drivers-and-io.md`
- `kernel/src/monitor/commands/usb.rs` → `plans/drivers-and-io.md`
- `kernel/src/monitor/mod.rs` → `plans/drivers-and-io.md`, `plans/observability.md`
- `tools/theseus-qemu/src/main.rs` → `plans/drivers-and-io.md`, `plans/observability.md`

### Observability

- `bootloader/src/display.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `bootloader/src/serial.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `bootloader/src/qemu_exit.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `bootloader/src/drivers/mod.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `bootloader/src/drivers/manager.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `bootloader/src/drivers/qemu_debug.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `bootloader/src/drivers/raw_serial.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `bootloader/src/drivers/uefi_serial.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `kernel/src/logging/mod.rs` → `plans/observability.md`
- `kernel/src/logging/output.rs` → `plans/observability.md`
- `kernel/src/logging/filter.rs` → `plans/observability.md`
- `kernel/src/logging/macros.rs` → `plans/observability.md`
- `kernel/src/panic.rs` → `plans/observability.md`
- `kernel/src/display.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `kernel/src/framebuffer.rs` → `plans/observability.md`, `plans/boot-flow.md`
- `kernel/src/bootlogo.rs` → `plans/observability.md`
- `kernel/src/monitor/mod.rs` → `plans/observability.md`, `plans/drivers-and-io.md`
- `kernel/src/monitor/parsing.rs` → `plans/observability.md`
- `kernel/src/monitor/commands/*` → `plans/observability.md`, `plans/drivers-and-io.md`
- `docs/logging.md` → `plans/observability.md`
- `docs/qemu-runner.md` → `plans/observability.md`, `plans/drivers-and-io.md`

## Axioms → Affected Modules

### `axioms/boot.md`

- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `bootloader/src/acpi.rs`
- `bootloader/src/hardware.rs`
- `bootloader/src/display.rs`
- `bootloader/src/system_info.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `bootloader/src/drivers/*`
- `shared/src/handoff.rs`
- `kernel/src/lib.rs`
- `kernel/src/handoff.rs`

### `axioms/memory.md`

- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `shared/src/handoff.rs`
- `kernel/src/handoff.rs`
- `kernel/src/memory.rs`
- `kernel/src/memory/mapping.rs`
- `kernel/src/environment.rs`
- `kernel/src/physical_memory.rs`
- `kernel/src/acpi/mod.rs`
- `kernel/src/interrupts/apic.rs`

### `axioms/arch-x86_64.md`

- `kernel/src/lib.rs`
- `kernel/src/gdt.rs`
- `kernel/src/interrupts/*`
- `kernel/src/acpi/*`
- `kernel/src/drivers/serial.rs`
- `kernel/src/environment.rs`
- `kernel/src/drivers/pci.rs`
- `kernel/src/drivers/usb/*`

### `axioms/debug.md`

- `bootloader/src/display.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `bootloader/src/drivers/*`
- `kernel/src/lib.rs`
- `kernel/src/logging/*`
- `kernel/src/panic.rs`
- `kernel/src/monitor/*`
- `kernel/src/interrupts/debug.rs`
- `kernel/src/display.rs`
- `kernel/src/framebuffer.rs`
- `kernel/src/bootlogo.rs`
- `docs/development-and-debugging.md`
- `docs/logging.md`
- `docs/qemu-runner.md`
