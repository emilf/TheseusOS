//! Module: drivers::system
//!
//! SOURCE OF TRUTH:
//! - docs/plans/drivers-and-io.md
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - This module is the bridge from discovered platform state into the runtime driver manager.
//! - ACPI/platform discovery, PCI enumeration, USB handoff, and driver registration are sequenced here during bring-up.
//! - The module may seed the monitor and serial/USB-facing drivers, but it is not itself the long-term owner of device-specific runtime state.
//!
//! SAFETY:
//! - Handoff-derived inventory and ACPI-derived resource descriptions are inputs to validation and classification, not blind guarantees that a device is safe to program.
//! - Driver registration and device insertion happen under the manager’s synchronization rules; avoid expanding this path with long-running work while locks are held.
//! - Platform-to-driver translation must preserve IRQ/MMIO/resource identity accurately or downstream drivers will program the wrong hardware.
//!
//! PROGRESS:
//! - docs/plans/drivers-and-io.md
//! - docs/plans/interrupts-and-platform.md
//!
//! High-level driver system initialization.

use alloc::format;
use alloc::vec::Vec;

use crate::acpi::{self, PlatformInfo};
use crate::config;
use crate::handoff::handoff_phys_ptr;
use crate::{log_debug, log_info, log_trace, log_warn};

use super::manager::driver_manager;
use super::pci;
use super::serial;
use super::traits::{Device, DeviceClass, DeviceId, DeviceResource};
use super::usb;
use crate::monitor;

/// Result type for driver system operations
pub type DriverResult<T> = Result<T, &'static str>;

/// Initialize the driver system using information from the handoff.
///
/// This sequences ACPI discovery, firmware inventory import, PCI enumeration,
/// USB handoff/registration, and monitor bring-up.
pub fn init() -> DriverResult<PlatformInfo> {
    log_debug!("Initializing driver system");

    // Get handoff information from bootloader
    let handoff = unsafe { &*(handoff_phys_ptr() as *const theseus_shared::handoff::Handoff) };

    // Initialize ACPI and parse platform information
    let mut platform_info = acpi::initialize_acpi(handoff.acpi_rsdp)?;
    log_info!("ACPI initialization complete");

    // Set up IO APIC information for serial driver
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

    // Initialize and register serial driver
    serial::init_serial();
    log_debug!("Serial driver registered");

    // Process UEFI hardware inventory if available
    if handoff.hardware_device_count == 0 || handoff.hardware_inventory_ptr == 0 {
        log_warn!("Hardware inventory missing");
    } else {
        log_debug!("Registering UEFI hardware inventory");

        let count = handoff.hardware_device_count as usize;
        let bytes = handoff.hardware_inventory_size as usize;
        let slice = unsafe {
            core::slice::from_raw_parts(handoff.hardware_inventory_ptr as *const u8, bytes)
        };

        // Process each hardware device entry
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

            // Create device descriptor based on device type
            let device_id = match entry.device_type {
                theseus_shared::handoff::DEVICE_TYPE_SERIAL => DeviceId::Class(DeviceClass::Serial),
                _ => DeviceId::Raw(entry.device_type_str()),
            };
            let mut device = Device::new(device_id);

            // Set up device-specific information
            if entry.device_type == theseus_shared::handoff::DEVICE_TYPE_SERIAL {
                device.class = DeviceClass::Serial;
                if let Some(addr) = entry.address {
                    device.phys_addr = Some(addr);
                }
                if let Some(irq) = entry.irq {
                    device.irq = Some(irq);
                }
            }

            // Register device with driver manager
            driver_manager().lock().add_device(device);
        }
    }

    // Enumerate PCI devices
    let pci_topology = pci::enumerate(&platform_info.pci_config_regions);
    if pci_topology.functions.is_empty() {
        log_warn!("PCI scan: no devices discovered");
    } else {
        log_info!(
            "PCI scan: discovered {} function(s)",
            pci_topology.functions.len()
        );
    }

    // Store PCI bridge information
    platform_info.pci_bridges = pci_topology.bridges.clone();
    for bridge in platform_info.pci_bridges.iter() {
        log_debug!(
            "PCI bridge {:04x}:{:02x}:{:02x}.{} secondary={:02x} subordinate={:02x}",
            bridge.segment,
            bridge.bus,
            bridge.device,
            bridge.function,
            bridge.secondary_bus,
            bridge.subordinate_bus
        );
    }

    // Ensure firmware releases ownership of any legacy USB controllers
    usb::ensure_legacy_usb_handoff(&pci_topology.functions);
    // Register USB-class drivers (e.g., xHCI) after handoff
    usb::init();

    // Register PCI devices with driver manager
    let mut pci_devices: Vec<Device> = Vec::new();
    for info in pci_topology.functions.iter() {
        let class = classify_pci_device(info);

        // Skip bridge devices (they're handled separately)
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

        // Create device descriptor for PCI function
        let mut device = Device::new(DeviceId::Pci {
            segment: info.segment,
            bus: info.bus,
            device: info.device,
            function: info.function,
        });
        device.class = class;

        // Convert PCI BARs to device resources
        for bar in info.bars.iter() {
            match bar {
                pci::PciBar::Memory32 {
                    base,
                    size,
                    prefetchable,
                }
                | pci::PciBar::Memory64 {
                    base,
                    size,
                    prefetchable,
                } => {
                    if *size > 0 {
                        device.resources.push(DeviceResource::Memory {
                            base: *base,
                            size: *size,
                            prefetchable: *prefetchable,
                        });
                    }
                }
                pci::PciBar::Io { base, size } => {
                    if *size > 0 {
                        device.resources.push(DeviceResource::Io {
                            base: *base,
                            size: *size,
                        });
                    }
                }
                pci::PciBar::None => {}
            }
        }

        // Set primary memory address from first memory BAR
        if let Some(bar) = info.first_memory_bar() {
            if let Some(base) = bar.memory_base() {
                device.phys_addr = Some(base);
            }
        }

        // Set interrupt information
        if let Some(irq) = info.interrupt_line() {
            device.irq = Some(irq as u32);
        }

        // Log device information
        let log_msg = format!(
            "PCI {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x} -> {:?} irq={:?} msi={} msix={}",
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

    // Register all PCI devices with driver manager
    if !pci_devices.is_empty() {
        let mut manager = driver_manager().lock();
        for device in pci_devices {
            manager.add_device(device);
        }
    }

    // Initialize kernel monitor
    monitor::init();
    if config::ENABLE_KERNEL_MONITOR {
        log_info!("Monitor activated");
    } else {
        log_debug!("Monitor disabled via config");
    }

    Ok(platform_info)
}

/// Classify a PCI device based on its class code and subclass.
///
/// This function maps PCI device class codes to the kernel's device classes.
/// It uses the standard PCI class code definitions to determine the appropriate
/// device class for driver binding.
///
/// # Arguments
/// * `info` - PCI device information containing class codes
///
/// # Returns
/// * `DeviceClass` - The appropriate device class for this PCI device
///
/// # PCI Class Code Mapping
/// - **0x0C03**: USB Controller (USB)
/// - **0x0106**: Storage Controller (SATA AHCI)
/// - **0x0108**: Storage Controller (NVM Express)
/// - **0x01xx**: Storage Controller (other storage devices)
/// - **0x02xx**: Network Controller
/// - **0x0600**: Bridge Device (Host bridge)
/// - **0x0604**: Bridge Device (PCI-to-PCI bridge)
/// - **Other**: Unknown device class
fn classify_pci_device(info: &pci::PciDeviceInfo) -> DeviceClass {
    match info.class_code {
        0x0C => match info.subclass {
            0x03 => DeviceClass::UsbController, // USB
            _ => DeviceClass::Unknown,
        },
        0x01 => match info.subclass {
            0x06 => DeviceClass::Storage, // SATA AHCI
            0x08 => DeviceClass::Storage, // NVM Express
            _ => DeviceClass::Storage,
        },
        0x02 => DeviceClass::Network,
        0x03 => match info.subclass {
            0x00 => DeviceClass::Display, // VGA
            0x01 => DeviceClass::Display, // XGA
            0x02 => DeviceClass::Display, // 3D
            _ => DeviceClass::Display,
        },
        0x04 => match info.subclass {
            0x00 => DeviceClass::Audio, // Audio
            0x01 => DeviceClass::Audio, // Modem
            0x03 => DeviceClass::Audio, // HDA
            _ => DeviceClass::Audio,
        },
        0x06 => match info.subclass {
            0x00 | 0x04 => DeviceClass::Bridge,
            _ => DeviceClass::Unknown,
        },
        0x0D => match info.subclass {
            0x00 => DeviceClass::Input, // Keyboard
            0x01 => DeviceClass::Input, // Pen
            0x02 => DeviceClass::Input, // Mouse
            0x03 => DeviceClass::Input, // Scanner
            0x10 => DeviceClass::Input, // Gameport
            _ => DeviceClass::Input,
        },
        0x0E => DeviceClass::Wireless,
        _ => DeviceClass::Unknown,
    }
}
