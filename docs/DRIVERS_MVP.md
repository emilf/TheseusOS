## Device/Driver Pivot — Minimal Workplan & MVP Driver-Kernel Interface

Goal: make the codebase ready to shift focus from boot/debug noise to device management, drivers, ACPI-based platform discovery, and bringing up APs (SMP). This document lists the smallest, highest-value changes to make that pivot feasible and defines a minimal, pragmatic driver-kernel interface (MVP) for initial drivers.

### Quick context (current relevant artifacts)
- Bootloader currently discovers ACPI RSDP and exposes it in the `Handoff` (`shared/src/handoff.rs` -> `Handoff.acpi_rsdp`).
- Output driver manager lives in `bootloader/src/drivers/` and uses a simple `Driver` trait (output-only).
- Kernel environment currently initializes IDT/LAPIC and has code paths for LAPIC timer and APIC interactions in `kernel/src/environment.rs`.

### Minimum work to make the pivot viable (priority ordered)
1. **Handoff & ACPI**
   - Ensure `Handoff.acpi_rsdp` is set by the bootloader (already present) and reliably copied to the kernel.
   - Add an ACPI parsing entry point in the kernel that consumes `handoff.acpi_rsdp` and produces a small platform summary (MADT/APIC IDs, HPET/LAPIC MMIO, _ optionally PCI root bridges_).
   - File targets: `kernel/src/acpi/mod.rs`, `bootloader/src/acpi.rs` (confirm RSDP copy).

2. **Kernel-side physical memory mapping for ACPI parsing**
   - Implement a tiny `AcpiMapping` handler using the kernel's `PHYS_OFFSET` mapping (don't require UEFI runtime).
   - File target: `kernel/src/acpi/mod.rs`.

3. **Small, predictable driver core**
   - Add a minimal driver framework in kernel:
     - `Driver` trait with lifecycle methods.
     - `Device` / `DeviceId` structs.
     - `DriverManager` with registration and a simple probe loop.
   - File targets: `kernel/src/drivers/traits.rs`, `kernel/src/drivers/manager.rs`, `kernel/src/drivers/mod.rs`.

4. **Platform enumeration & simple buses**
   - Implement ACPI MADT parser to enumerate CPUs and IO APIC entries.
   - Basic PCI bus enumerator (optional for first pass — can be deferred if platform is minimal).
   - File targets: `kernel/src/acpi/madt.rs`, `kernel/src/bus/pci.rs` (defer advanced PCI features).

5. **AP (SMP) bootstrap minimal path**
   - Implement code to start APs via local APIC and SIPI using discovered MADT info.
   - Provide `smp::start_aps()` that accepts an AP entry pointer and per-CPU stack setup.
   - File targets: `kernel/src/smp.rs`, `kernel/src/cpu/percpu.rs`.

6. **Interrupt and IRQ plumbing for drivers**
   - Provide API for drivers to claim IRQs and register a handler (wrap LAPIC/IOAPIC).
   - File target: `kernel/src/irq.rs` or `kernel/src/interrupts/irq_manager.rs`.

7. **Example drivers to validate the stack**
   - Serial console driver (reuse/port `bootloader/src/drivers/raw_serial.rs` into kernel).
   - Simple timer driver using LAPIC (for heart/tick test).
   - Optional: basic PCI NIC driver stub (probe-only).
   - File targets: `kernel/src/drivers/serial.rs`, `kernel/src/drivers/timer.rs`.

8. **Clean up noisy debug & centralize constants**
   - Pare down inline debug in `continue_after_stack_switch()` and other hot paths.
   - Centralize constants into `kernel/src/constants.rs` to avoid scattering magic values.

### Minimal success criteria (MVP)
- Kernel receives `Handoff` with `acpi_rsdp` and can parse MADT, producing list of CPU/APIC IDs.
- `smp::start_aps()` can start at least one AP (or can simulate start in QEMU) and set up per-CPU stacks + basic per-CPU init.
- A kernel `DriverManager` can register a serial driver and produce console output after boot (so other debug prints work without bootloader helper).
- Drivers can request IRQ registration and receive timer IRQs from LAPIC.

### Suggested timeline / phases (small increments)
- Phase 0 (1-2 days): Add kernel ACPI module + mapping handler; verify MADT parsing.
- Phase 1 (1-3 days): Implement `drivers::traits` + `drivers::manager` and a kernel serial driver; confirm kernel console works post-boot.
- Phase 2 (2-4 days): MADT-based AP enumeration + `smp::start_aps()` and minimal AP entry that registers per-CPU state.
- Phase 3 (2-4 days): IRQ manager + sample timer driver + driver IRQ handling tests.
- Phase 4 (optional): PCI enumerator + PCI driver probe flow.

### MVP driver-kernel interface (Rust sketch)
- Keep it small, single-thread-first (kernel can evolve to threaded later).
- Focus on lifecycle, probe, IRQ, and simple I/O. Use `Result` for recoverable errors.

```rust
// kernel/src/drivers/traits.rs
/// Minimal device identifier (PCI, ACPI, simple strings)
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum DeviceId {
    Acpi(&'static str),        // e.g., ACPI HID like "PNP0A03"
    Pci { bus: u8, dev: u8, func: u8 },
    Raw(&'static str),         // driver-managed tagging
}

/// Minimal Device representation
pub struct Device {
    pub id: DeviceId,
    pub phys_addr: Option<u64>, // MMIO base if applicable
    pub irq: Option<u32>,
    pub driver_data: Option<*mut core::ffi::c_void>, // opaque per-device data
}

/// Driver lifecycle and basic ops
pub trait Driver: Sync + Send {
    /// Called once at registration; return Ok(true) if driver wants to be probed now.
    fn on_register(&'static self) -> core::result::Result<bool, &'static str> { Ok(true) }

    /// Called to probe a Device; return Ok(()) on success (driver bound)
    fn probe(&'static self, dev: &mut Device) -> core::result::Result<(), &'static str>;

    /// Called when device is removed/unbound
    fn remove(&'static self, dev: &mut Device);

    /// Optional IRQ handler for this driver; returns whether it handled the IRQ.
    fn irq_handler(&'static self, dev: &mut Device, irq: u32) -> bool { false }

    /// Simple read/write API (for character devices); use Result for errors
    fn write(&'static self, _dev: &mut Device, _buf: &[u8]) -> core::result::Result<usize, &'static str> { Err("unimplemented") }
    fn read(&'static self, _dev: &mut Device, _buf: &mut [u8]) -> core::result::Result<usize, &'static str> { Err("unimplemented") }
}
```

```rust
// kernel/src/drivers/manager.rs (high-level sketch)
use crate::drivers::traits::{Driver, Device, DeviceId};

pub struct DriverManager {
    // simple Vecs for MVP; later replace with lock-protected registries
    pub drivers: alloc::vec::Vec<&'static dyn Driver>,
    pub devices: alloc::vec::Vec<Device>,
}

impl DriverManager {
    pub const fn new() -> Self { /* ... */ }

    /// Register a driver into the system
    pub fn register_driver(&mut self, drv: &'static dyn Driver) {
        if let Ok(true) = drv.on_register() {
            self.drivers.push(drv);
        }
    }

    /// Add detected device (from ACPI/PCI/hand-off) — then run probe loop.
    pub fn add_device(&mut self, dev: Device) {
        self.devices.push(dev);
        self.probe_new_devices();
    }

    /// Run a simple probe loop: for each unbound device, call each driver.probe()
    pub fn probe_new_devices(&mut self) {
        for dev in self.devices.iter_mut() {
            if dev.driver_data.is_some() { continue; } // already bound
            for drv in self.drivers.iter() {
                if drv.probe(dev).is_ok() {
                    // mark bound (driver can set driver_data)
                    break;
                }
            }
        }
    }

    /// IRQ dispatching helper
    pub fn handle_irq(&self, irq: u32) -> bool {
        for dev in self.devices.iter() {
            if let Some(dirq) = dev.irq {
                if dirq == irq {
                    if let Some(drv_ptr) = dev.driver_data {
                        // cast back to &dyn Driver (MVP: store index or other safe handle later)
                    }
                }
            }
        }
        false
    }
}