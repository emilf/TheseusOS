# Hardware & Drivers

TheseusOS keeps hardware discovery simple by doing most enumeration in the bootloader and then layering a lightweight driver framework on top of the handoff data. This page captures how devices, interrupts, and I/O fit together.

## Hardware Inventory from Boot
- The bootloader classifies UEFI handles by protocol (`LoadedImage`, `PciIo`, `UsbIo`, etc.) and stores the results in the handoff (`handoff.hardware_*` fields).
- At runtime you can review the captured inventory via the serial monitor command `devices` (see [Development & Debugging](development-and-debugging.md)).
- Configuration toggle `config::PRINT_HARDWARE_INVENTORY` prints the inventory during bring-up without needing the monitor.

## Interrupt Subsystem
Modules: `kernel/src/interrupts`.

- `setup_idt()` installs handlers for CPU exceptions (divide error, general protection, page fault, double fault) and hardware IRQs.
- IST (Interrupt Stack Table) entries are assigned to critical faults so they run on hardened stacks; see `gdt::ist_stack_ranges`.
- Local APIC support lives in `interrupts/apic.rs`; helpers like `lapic_timer_configure`, `lapic_timer_start_*`, and `lapic_timer_mask` expose timer control.
- Timer ticks increment a global atomic (`interrupts::TIMER_TICKS`) used for delays and the framebuffer heart animation.
- The serial IRQ (vector 0x41) routes bytes from COM1 into the monitor through `monitor::notify_serial_byte`.

Useful monitor commands:
- `idt` — dumps vector summaries and handler addresses.
- `lapic` — shows MMIO registers and timer state.
- `timer` — prints tick counters and allows manual firing of the LAPIC.

## Driver Manager
Location: `kernel/src/drivers/manager.rs`.

- `DriverManager` keeps two registries: known drivers and discovered devices.
- Drivers implement `drivers::traits::Driver` and register themselves via `driver_manager().lock().register_driver(...)`.
- Devices are lightweight structs (`drivers::traits::Device`) that carry class IDs, IRQ lines, and optional MMIO regions.
- When a device is added (`add_device`), the manager probes registered drivers. `probe`+`init` must both succeed before the device is marked as bound.
- IRQ dispatch (`handle_irq`) looks up the device by IRQ then lets the bound driver handle the event. If no driver claims it, the IRQ is reported as unhandled.

### Serial Driver Stack
- `drivers::serial` provides a simple UART16550 driver for COM1, wired to IRQ4 and the 0x3F8 I/O port.
- The driver feeds bytes into the logging subsystem and the monitor.
- The bootloader also exposes serial output options; the chosen configuration is printed during boot (`OutputDriver::current_driver_name()`).

### System Driver Init
- `drivers::system::init()` parses ACPI tables (when available) to collect CPU and IO-APIC counts before driver registration.
- The function returns a summary used for runtime logging. Failures are benign: the kernel logs a warning and continues.

## Monitoring & Control
- The Wozmon-inspired serial monitor lives in `kernel/src/monitor`. Commands are grouped by topic under `commands/`.
- Highlights:
  - `mem`, `dump`, `fill` — read/write memory in hex and ASCII.
  - `regs`, `cpuid`, `msr` — inspect CPU state.
  - `io`, `in`, `out` — access x86 I/O ports.
  - `loglevel`, `logoutput` — adjust logging filters without rebuilding.
  - `devices`, `drivers` — inspect registered devices and drivers.
- Because the monitor is interrupt driven, you can keep the CPU halted (`hlt`) and wake it only when a serial byte arrives.

## Where to Look Next
- Cross-reference [Memory Management](memory-management.md) to see how MMIO regions and physical frames are mapped before drivers start.
- Visit [Development & Debugging](development-and-debugging.md) for instructions on using the monitor, logging, and QEMU debug channels effectively.
