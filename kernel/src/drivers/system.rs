//! High-level driver system initialization
//!
//! This module glues together ACPI discovery and the driver manager. It is
//! responsible for initializing ACPI, enumerating platform devices, and feeding
//! them into the manager so that concrete drivers can bind.

use crate::acpi::{initialize_acpi, PlatformInfo};
use crate::display::kernel_write_line;
use crate::handoff::handoff_phys_ptr;

use super::manager::driver_manager;
use super::traits::{Device, DeviceId};

/// Result type for driver system operations
pub type DriverResult<T> = Result<T, &'static str>;

/// Initialize the driver system using information from the handoff
pub fn init() -> DriverResult<PlatformInfo> {
    kernel_write_line("[driver] initializing driver system");

    let handoff = unsafe { &*(handoff_phys_ptr() as *const theseus_shared::handoff::Handoff) };

    if handoff.acpi_rsdp == 0 {
        kernel_write_line("[driver] no ACPI RSDP present; driver discovery skipped");
        return Err("acpi rsdp missing");
    }

    let platform_info = initialize_acpi(handoff.acpi_rsdp)?;

    // Register CPU and IO APIC devices based on MADT
    if let Some(madt) = &platform_info.madt_info {
        // CPU devices
        for (idx, apic_id) in madt.cpu_apic_ids.iter().enumerate() {
            let mut device = Device::new(DeviceId::Raw("cpu"));
            device.driver_data = None;
            device.irq = None;
            kernel_write_line("[driver] registering CPU device");
            driver_manager().lock().add_device(device);

            // Reuse the shared hex printing utilities for consistency
            kernel_write_line("[driver] CPU index:");
            theseus_shared::print_hex_u64_0xe9!(idx as u64);
            kernel_write_line(" APIC ID:");
            theseus_shared::print_hex_u64_0xe9!(*apic_id as u64);
            kernel_write_line("");
        }

        // IO APIC devices
        for io_apic in &madt.io_apics {
            let mut device = Device::new(DeviceId::Raw("ioapic"));
            device.phys_addr = Some(io_apic.address);
            device.irq = Some(io_apic.gsi_base);
            kernel_write_line("[driver] registering IO APIC device");
            driver_manager().lock().add_device(device);
        }
    }

    Ok(platform_info)
}
