//! High-level driver system initialization
//!
//! This module glues together ACPI discovery and the driver manager. It is
//! responsible for initializing ACPI, enumerating platform devices, and feeding
//! them into the manager so that concrete drivers can bind.

use alloc::format;
use alloc::vec::Vec;

use crate::acpi::{self, PlatformInfo};
use crate::config;
use crate::handoff::handoff_phys_ptr;
use crate::{log_debug, log_info, log_trace, log_warn};

use super::manager::driver_manager;
use super::pci;
use super::serial;
use super::traits::{Device, DeviceClass, DeviceId};
use crate::monitor;

/// Result type for driver system operations
pub type DriverResult<T> = Result<T, &'static str>;

/// Initialize the driver system using information from the handoff
pub fn init() -> DriverResult<PlatformInfo> {
    log_debug!("Initializing driver system");

    let handoff = unsafe { &*(handoff_phys_ptr() as *const theseus_shared::handoff::Handoff) };

    let mut platform_info = acpi::initialize_acpi(handoff.acpi_rsdp)?;
    log_info!("ACPI initialization complete");

    if let Some(madt) = &platform_info.madt_info {
        if let Some(io_apic) = madt.io_apics.first() {
            serial::install_io_apic_info(io_apic.address, io_apic.gsi_base);
            log_debug!(
                "Serial IO APIC info installed: address={:#x} gsi_base={}",
                io_apic.address,
                io_apic.gsi_base
            );
        }
    }

    serial::init_serial();
    log_debug!("Serial driver registered");

    if handoff.hardware_device_count == 0 || handoff.hardware_inventory_ptr == 0 {
        log_warn!("Hardware inventory missing");
    } else {
        log_debug!("Registering UEFI hardware inventory");

        let count = handoff.hardware_device_count as usize;
        let bytes = handoff.hardware_inventory_size as usize;
        let slice = unsafe {
            core::slice::from_raw_parts(handoff.hardware_inventory_ptr as *const u8, bytes)
        };

        for idx in 0..count {
            let entry = theseus_shared::handoff::HardwareDevice::from_bytes(slice, idx)
                .ok_or("invalid hardware inventory entry")?;

            if crate::config::PRINT_HARDWARE_INVENTORY {
                log_debug!(
                    "Inventory entry {}: type={} address={:#x} irq={}",
                    idx,
                    entry.device_type_str(),
                    entry.address.unwrap_or(0),
                    entry.irq.unwrap_or(0)
                );
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
    }

    let pci_topology = pci::enumerate(&platform_info.pci_config_regions);
    if pci_topology.functions.is_empty() {
        log_warn!("PCI scan: no devices discovered");
    } else {
        log_info!(
            "PCI scan: discovered {} function(s)",
            pci_topology.functions.len()
        );
    }

    platform_info.pci_bridges = pci_topology.bridges.clone();
    for bridge in platform_info.pci_bridges.iter() {
        log_debug!(
            "PCI bridge {:04x}:{:02x}:{:02x}.{} secondary={:02x} subordinate={:02x} max_child={:02x}",
            bridge.segment,
            bridge.bus,
            bridge.device,
            bridge.function,
            bridge.secondary_bus,
            bridge.subordinate_bus,
            bridge.max_child_bus
        );
    }

    let mut pci_devices: Vec<Device> = Vec::new();
    for info in pci_topology.functions.iter() {
        let class = classify_pci_device(info);
        if class == DeviceClass::Bridge {
            log_trace!(
                "Skipping PCI bridge {:04x}:{:02x}:{:02x}.{}",
                info.segment,
                info.bus,
                info.device,
                info.function
            );
            continue;
        }

        let mut device = Device::new(DeviceId::Pci {
            segment: info.segment,
            bus: info.bus,
            device: info.device,
            function: info.function,
        });
        device.class = class;

        if let Some(bar) = info.first_memory_bar() {
            if let Some(base) = bar.memory_base() {
                device.phys_addr = Some(base);
            }
        }

        if let Some(irq) = info.interrupt_line() {
            device.irq = Some(irq as u32);
        }

        let log_msg = format!(
            "PCI {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x} -> {:?} irq={:?} phys={:?} msi={} msix={}",
            info.segment,
            info.bus,
            info.device,
            info.function,
            info.vendor_id,
            info.device_id,
            info.class_code,
            info.subclass,
            info.prog_if,
            device.class,
            device.irq,
            device.phys_addr,
            info.capabilities.msi,
            info.capabilities.msix
        );
        if matches!(device.class, DeviceClass::UsbController) {
            log_info!("Registering USB controller: {}", log_msg);
        } else {
            log_debug!("Registering PCI function: {}", log_msg);
        }

        pci_devices.push(device);
    }

    if !pci_devices.is_empty() {
        let mut manager = driver_manager().lock();
        for device in pci_devices {
            manager.add_device(device);
        }
    }

    monitor::init();
    if config::ENABLE_KERNEL_MONITOR {
        log_info!("Monitor activated");
    } else {
        log_debug!("Monitor disabled via config");
    }

    Ok(platform_info)
}

fn classify_pci_device(info: &pci::PciDeviceInfo) -> DeviceClass {
    match info.class_code {
        0x0C => match info.subclass {
            0x03 => DeviceClass::UsbController,
            _ => DeviceClass::Unknown,
        },
        0x01 => DeviceClass::Storage,
        0x02 => DeviceClass::Network,
        0x06 => match info.subclass {
            0x00 | 0x04 => DeviceClass::Bridge,
            _ => DeviceClass::Unknown,
        },
        _ => DeviceClass::Unknown,
    }
}
