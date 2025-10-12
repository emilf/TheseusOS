//! High-level driver system initialization
//!
//! This module glues together ACPI discovery and the driver manager. It is
//! responsible for initializing ACPI, enumerating platform devices, and feeding
//! them into the manager so that concrete drivers can bind.

use crate::acpi::{self, PlatformInfo};
use crate::display::kernel_write_line;
use crate::handoff::handoff_phys_ptr;

use super::manager::driver_manager;
use super::serial;
use super::traits::{Device, DeviceClass, DeviceId};
use crate::monitor;

/// Result type for driver system operations
pub type DriverResult<T> = Result<T, &'static str>;

/// Initialize the driver system using information from the handoff
pub fn init() -> DriverResult<PlatformInfo> {
    kernel_write_line("[driver] initializing driver system");

    serial::init_serial();
    kernel_write_line("[driver] serial driver registered");

    let handoff = unsafe { &*(handoff_phys_ptr() as *const theseus_shared::handoff::Handoff) };

    let platform_info = acpi::initialize_acpi(handoff.acpi_rsdp)?;
    kernel_write_line("[driver] ACPI initialization complete");

    if handoff.hardware_device_count == 0 || handoff.hardware_inventory_ptr == 0 {
        kernel_write_line("[driver] hardware inventory missing");
        return Ok(platform_info);
    }

    kernel_write_line("[driver] registering UEFI hardware inventory");

    let count = handoff.hardware_device_count as usize;
    let bytes = handoff.hardware_inventory_size as usize;
    let slice =
        unsafe { core::slice::from_raw_parts(handoff.hardware_inventory_ptr as *const u8, bytes) };

    for idx in 0..count {
        let entry = theseus_shared::handoff::HardwareDevice::from_bytes(slice, idx)
            .ok_or("invalid hardware inventory entry")?;

        if crate::config::PRINT_HARDWARE_INVENTORY {
            kernel_write_line("[driver] inventory entry index:");
            theseus_shared::print_hex_u64_0xe9!(idx as u64);
            kernel_write_line(" type:");
            kernel_write_line(entry.device_type_str());
            if let Some(addr) = entry.address {
                kernel_write_line(" address:");
                theseus_shared::print_hex_u64_0xe9!(addr);
            }
            if let Some(irq) = entry.irq {
                kernel_write_line(" irq:");
                theseus_shared::print_hex_u64_0xe9!(irq as u64);
            }
            kernel_write_line("");
        }

        let device_id = match entry.device_type {
            theseus_shared::handoff::DEVICE_TYPE_SERIAL => DeviceId::Class(DeviceClass::Serial),
            _ => DeviceId::Raw(entry.device_type_str()),
        };
        let mut device = Device::new(device_id);
        if entry.device_type == theseus_shared::handoff::DEVICE_TYPE_SERIAL {
            device.class = DeviceClass::Serial;
        }
        driver_manager().lock().add_device(device);
    }

    monitor::init();
    kernel_write_line("[monitor] activated");

    Ok(platform_info)
}
