# Hardware & Drivers

TheseusOS front-loads hardware discovery in the bootloader and then hands a curated inventory to the kernel. The kernel layers interrupts, device enumeration, and the driver framework on top of that handoff. This page explains how the pieces fit together and how to write your own driver.

- New to paging and address space setup? See [Memory Management](memory-management.md).
- Looking for run/debug tips? Jump to [Development & Debugging](development-and-debugging.md).

## Architecture Snapshot

1. **Bootloader inventory** — UEFI services enumerate graphics, firmware, ACPI, PCI/handle devices, and serial ports. Everything is serialized into `handoff.hardware_*`.
2. **Driver system init** — `drivers::system::init()` validates ACPI, registers built-in drivers (serial), and turns inventory entries into `Device` descriptors.
3. **Driver manager** — Maintains registries of drivers and devices, handles probing, and dispatches interrupts/I/O to the proper driver.
4. **Monitor & tooling** — The serial monitor and logging stack sit on top of the driver framework for inspection and control.

The design favors clarity over features: no hot removal yet, synchronous I/O paths, and first-driver-wins binding.

## Bootloader Hardware Inventory

- Stored in `theseus_shared::handoff::HardwareInventory` entries accessible via `handoff.hardware_inventory_ptr`.
- Each entry carries a type tag, optional MMIO base, IRQ line, and textual description.
- During kernel bring-up:
  - `config::PRINT_HARDWARE_INVENTORY` toggles verbose logging of the inventory.
  - `drivers::system::init()` converts entries into `Device` structs and calls `driver_manager().lock().add_device(device)`.
- At runtime:
  - Serial monitor command `devices` prints the current registry (and bound driver state).
  - `drivers::system::init()` returns `PlatformInfo` containing ACPI parsing results (CPUs, IO-APICs).

## Interrupt Subsystem Essentials

Location: `kernel/src/interrupts`.

- `setup_idt()` installs exception/IRQ handlers; IST stacks (from `gdt::ist_stack_ranges`) protect critical faults.
- Local APIC helpers (`interrupts/apic.rs`) expose `lapic_timer_configure`, `lapic_timer_start_*`, `lapic_timer_mask`, and direct MMIO access.
- `interrupts::install_timer_vector_runtime()` and `lapic_timer_start_periodic()` keep the APIC timer running for the framebuffer heartbeat and for driver timeouts.
- `interrupts::TIMER_TICKS` counts ticks for coarse timing.
- The serial IRQ (vector `SERIAL_RX_VECTOR`, 0x41) funnels bytes into `monitor::notify_serial_byte`, which drives the CLI.

Monitor commands for interrupts:
- `idt` — dump vector table and gate attributes.
- `lapic` — inspect APIC registers and redirection state.
- `timer` — show tick count, program one-off timers.

## Driver Framework

### Key Types (`kernel/src/drivers/traits.rs`)

| Item | Purpose |
| --- | --- |
| `DeviceClass` | Broad capability category (e.g., `Serial`). Used to pre-filter probes. |
| `DeviceId` | Identity (ACPI HID, PCI BDF, class, or raw string). |
| `Device` | Runtime descriptor storing ID, class, MMIO base, IRQ, and opaque driver state (`driver_data`). |
| `Driver` trait | Lifecycle hooks: `supported_classes`, `probe`, `init`, `remove`, `irq_handler`, and optional `read`/`write`. |

Drivers stash internal state by writing pointers/handles into `Device::driver_data` via `set_driver_state`. Helper methods `driver_state::<T>()` and `driver_state_mut::<T>()` retrieve typed references.

### Driver Manager (`kernel/src/drivers/manager.rs`)

- Singleton behind a `spin::Mutex`: `driver_manager() -> &'static Mutex<DriverManager>`.
- Core responsibilities:
  - **Registry maintenance** — `register_driver` (drivers) and `add_device` (devices).
  - **Probe loop** — For each unbound device, iterate drivers, call `supports_class`, `probe`, and finally `init`. First successful driver binds the device.
  - **IRQ dispatch** — `handle_irq(irq)` walks devices with matching IRQs and forwards to `Driver::irq_handler`.
  - **Class/device I/O** — `write_class`, `read_class`, `write_to_device`, `read_from_device`.
- Thread-safety: All operations lock the manager; keep critical sections short. IRQ handlers may acquire the lock, so drivers should avoid long-running operations inside `irq_handler`.

### Built-in Drivers (`kernel/src/drivers`)

- `serial.rs` — 16550 UART driver (COM1). Implements `Driver` with interrupt-driven receive, polled transmit, and a circular RX buffer.
- `system.rs` — Glue that initializes ACPI, installs IO-APIC info for serial, converts inventory entries into `Device`s, and kicks off the monitor.
- Additional drivers register themselves during init by calling `driver_manager().lock().register_driver(&MY_DRIVER)`.

## Writing a Driver

Follow this pattern when adding a new hardware driver:

1. **Model the device**
   - Choose or add a `DeviceClass` in `drivers::traits`.
   - Define a `struct DriverState` with MMIO base pointers, buffers, etc.
2. **Implement the `Driver` trait**
   - `supported_classes` should list the class(es) you handle.
   - `probe(&mut Device)` validates IDs, resources, or ACPI data. Return `Err` to decline the device.
   - `init(&mut Device)` programs hardware, allocates state (typically via `Box::leak(Box::new(...))`), and stores a pointer with `device.set_driver_state`.
   - Implement `irq_handler`, `read`, `write`, or other optional hooks as appropriate.
3. **Register the driver**
   - Expose a `pub static` implementing the trait.
   - Call `driver_manager().lock().register_driver(&MY_DRIVER);` during system init (e.g., in `drivers::system::init` after ACPI discovery or from a module-level `init()` function).
4. **Provide device descriptors**
   - For firmware-provided devices, they arrive via the handoff inventory.
   - For dynamically discovered hardware, construct `Device::new(DeviceId::...)`, fill `phys_addr`/`irq` fields, and call `driver_manager().lock().add_device(device)`.

### Driver Skeleton

```rust
use crate::drivers::manager::driver_manager;
use crate::drivers::traits::{Device, DeviceClass, DeviceId, Driver};

struct FooState {
    regs: *mut FooRegisters,
}

struct FooDriver;
static FOO_DRIVER: FooDriver = FooDriver;

impl Driver for FooDriver {
    fn supported_classes(&self) -> &'static [DeviceClass] {
        &[DeviceClass::Unknown] // replace with a dedicated class if needed
    }

    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        match dev.id {
            DeviceId::Pci { vendor, device, .. } if is_my_controller(vendor, device) => Ok(()),
            _ => Err("not my device"),
        }
    }

    fn init(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        let phys = dev.phys_addr.ok_or("missing MMIO base")?;
        let regs = unsafe { crate::memory::phys_to_virt_pa(phys) as *mut FooRegisters };
        let state = Box::leak(Box::new(FooState { regs }));
        dev.set_driver_state(state);
        program_hardware(state)?;
        Ok(())
    }

    fn irq_handler(&'static self, dev: &mut Device, _irq: u32) -> bool {
        let state = dev.driver_state::<FooState>().expect("state missing");
        unsafe { handle_interrupt(state) }
    }
}

pub fn register() {
    driver_manager().lock().register_driver(&FOO_DRIVER);
}
```

Tips:
- Allocate runtime state on the heap and leak it deliberately; the driver manager stores raw pointers in `driver_data`.
- For memory-mapped I/O, convert physical addresses using `phys_to_virt_pa`.
- Use `physical_memory::PersistentFrameAllocator` if the driver needs to map new regions (`map_range_with_policy`).

### Device Registration Flow

When a device arrives (via firmware inventory or runtime discovery):
1. Construct `Device::new(DeviceId::...)`.
2. Set optional fields: `device.class`, `device.phys_addr`, `device.irq`.
3. Call `driver_manager().lock().add_device(device)`.
4. The manager immediately runs the probe loop and, if successful, calls your driver’s `init`.

## Serial Driver Deep Dive

- Handles legacy COM1 (I/O port 0x3F8, IRQ 4, or remapped GSI).
- Reception: Interrupt-driven via circular buffer (`rx_buffer`, `head`, `tail`).
- Transmission: Polled writes to THR once LSR bit 5 indicates empty.
- Integration points:
  - Registers with driver manager in `serial::register_serial_driver()`.
  - `serial::init_serial()` configures hardware, programs IO-APIC redirection (if available), and seeds the monitor.
  - Logging macros route output through `DriverManager::write_class(DeviceClass::Serial, ...)`, so the serial driver doubles as the kernel console backend.

## Monitoring & Control

- Serial monitor (`kernel/src/monitor`) is the primary admin interface.
- Commands relevant to hardware/drivers:
  - `devices` — list registered devices, classes, IRQs, driver state.
  - `drivers` — enumerate driver registry.
  - `gdt`, `idt`, `lapic`, `timer` — inspect descriptor tables and interrupt routing.
  - `io`, `in`, `out`, `mmio` — perform ad-hoc port or MMIO reads/writes for debugging.
- Because the monitor runs in an IRQ-driven loop, the CPU can `hlt` in idle while still servicing serial input.

## Best Practices & Safety Notes

- Keep driver `probe`/`init` deterministic and quick; they run while the driver manager lock is held.
- Always verify handoff-provided resources (MMIO base, IRQ) before using them; firmware bugs happen.
- When mapping MMIO, use the helpers from [Memory Management](memory-management.md) and select the correct cacheability flags.
- Return `false` from `irq_handler` if you did not service the interrupt; this helps diagnose IRQ routing issues.
- If your driver allocates frames or heap memory during `init`, provide a `remove` implementation to clean up when hot-unplug support arrives.

## Where to Look Next

- Dive into the paging story and MMIO mapping helpers in [Memory Management](memory-management.md).
- For step-by-step runtime workflows (building, QEMU, GDB, monitor), read [Development & Debugging](development-and-debugging.md).
- Historical design notes for drivers live in `docs/archive/` (e.g., `serial-driver.md`, `DRIVERS_MVP.md`).
