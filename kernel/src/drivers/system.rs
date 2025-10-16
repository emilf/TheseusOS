//! High-level driver system initialization
//!
//! This module glues together ACPI discovery and the driver manager. It is
//! responsible for initializing ACPI, enumerating platform devices, and feeding
//! them into the manager so that concrete drivers can bind.

use crate::acpi::{self, PlatformInfo};
use crate::config;
use crate::{log_debug, log_info, log_warn};
use crate::handoff::handoff_phys_ptr;

use super::manager::driver_manager;
use super::serial;
use super::traits::{Device, DeviceClass, DeviceId};
use crate::monitor;

/// Result type for driver system operations
pub type DriverResult<T> = Result<T, &'static str>;

/// Initialize the driver system using information from the handoff
pub fn init() -> DriverResult<PlatformInfo> {
    log_debug!("Initializing driver system");

    let handoff = unsafe { &*(handoff_phys_ptr() as *const theseus_shared::handoff::Handoff) };

    let platform_info = acpi::initialize_acpi(handoff.acpi_rsdp)?;
    log_info!("ACPI initialization complete");

    if let Some(madt) = &platform_info.madt_info {
        if let Some(io_apic) = madt.io_apics.first() {
            serial::install_io_apic_info(io_apic.address, io_apic.gsi_base);
            log_debug!("Serial IO APIC info installed: address={:#x} gsi_base={}",
                io_apic.address, io_apic.gsi_base);
        }
    }

    serial::init_serial();
    log_debug!("Serial driver registered");

    if handoff.hardware_device_count == 0 || handoff.hardware_inventory_ptr == 0 {
        log_warn!("Hardware inventory missing");
        return Ok(platform_info);
    }

    log_debug!("Registering UEFI hardware inventory");

    let count = handoff.hardware_device_count as usize;
    let bytes = handoff.hardware_inventory_size as usize;
    let slice =
        unsafe { core::slice::from_raw_parts(handoff.hardware_inventory_ptr as *const u8, bytes) };

    for idx in 0..count {
        let entry = theseus_shared::handoff::HardwareDevice::from_bytes(slice, idx)
            .ok_or("invalid hardware inventory entry")?;

        if crate::config::PRINT_HARDWARE_INVENTORY {
            log_debug!("Inventory entry {}: type={} address={:#x} irq={}",
                idx, entry.device_type_str(),
                entry.address.unwrap_or(0),
                entry.irq.unwrap_or(0));
        }

        let device_id = match entry.device_type {
            theseus_shared::handoff::DEVICE_TYPE_SERIAL => DeviceId::Class(DeviceClass::Serial),
            _ => DeviceId::Raw(entry.device_type_str()),
        };
        let mut device = Device::new(device_id);
        if entry.device_type == theseus_shared::handoff::DEVICE_TYPE_SERIAL {
            device.class = DeviceClass::Serial;
            if let Some(addr) = entry.address {
                device.phys_addr = Some(addr);
            }
            if let Some(irq) = entry.irq {
                device.irq = Some(irq);
            }
        }
        driver_manager().lock().add_device(device);
    }

    monitor::init();
    if config::ENABLE_KERNEL_MONITOR {
        log_info!("Monitor activated");
    } else {
        log_debug!("Monitor disabled via config");
    }

    Ok(platform_info)
}
