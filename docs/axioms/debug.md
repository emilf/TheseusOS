# Debug and Observability Axioms

Binding truths for logging, panic reporting, and runtime inspection.

These are the invariants the observability stack relies on today.

## A1: Kernel logging is initialized at kernel entry and is designed to work without heap allocation

**REQUIRED**

The kernel logging subsystem is initialized at the start of `kernel_entry`, and formatting/output paths are written to remain usable in panic and interrupt contexts without heap allocation. Before the kernel takes over, the bootloader exposes a separate best-effort observability stack whose current policy prefers QEMU debug output and falls back to firmware/raw serial helpers.

Implements / evidence:
- `bootloader/src/drivers/manager.rs`
- `bootloader/src/serial.rs`
- `kernel/src/lib.rs#kernel_entry`
- `kernel/src/logging/mod.rs`
- `kernel/src/logging/output.rs`

Related plans:
- `../plans/observability.md#implemented-invariants`

Affected modules:
- `bootloader/src/main.rs`
- `bootloader/src/display.rs`
- `bootloader/src/drivers/manager.rs`
- `bootloader/src/drivers/mod.rs`
- `bootloader/src/drivers/qemu_debug.rs`
- `bootloader/src/drivers/raw_serial.rs`
- `bootloader/src/drivers/uefi_serial.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `kernel/src/lib.rs`
- `kernel/src/logging/*`
- `kernel/src/panic.rs`

## A2: Panic handling reports failure through kernel logging and exits QEMU with error status

**REQUIRED**

The kernel panic handler logs panic details and terminates QEMU through the debug-exit mechanism.

Implements / evidence:
- `bootloader/src/qemu_exit.rs`
- `kernel/src/panic.rs#panic_handler`

Related plans:
- `../plans/observability.md#implemented-invariants`

Affected modules:
- `bootloader/src/qemu_exit.rs`
- `kernel/src/panic.rs`
- `kernel/src/logging/*`

## A3: The runtime monitor is a first-class inspection surface

**REQUIRED**

The kernel monitor exposes runtime inspection for descriptor tables, memory maps, page tables, CPU state, ACPI state, logging targets, and device state. Its serial input path is intentionally split so IRQ handling only queues bytes while heavier parsing and command execution happen later in monitor context.

Implements / evidence:
- `kernel/src/monitor/mod.rs`
- `kernel/src/monitor/parsing.rs`
- `kernel/src/monitor/commands/tables.rs`
- `kernel/src/monitor/commands/system.rs`
- `kernel/src/monitor/commands/memory.rs`

Related plans:
- `../plans/observability.md#implemented-invariants`

Affected modules:
- `bootloader/src/display.rs`
- `bootloader/src/drivers/*`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `kernel/src/monitor/*`
- `kernel/src/logging/*`
- `kernel/src/interrupts/debug.rs`
- `kernel/src/display.rs`
- `kernel/src/framebuffer.rs`
- `kernel/src/bootlogo.rs`
- `docs/development-and-debugging.md`
- `docs/logging.md`
- `docs/qemu-runner.md`
