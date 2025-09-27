extern crate alloc;

use crate::drivers::manager::write_line;
use alloc::format;
use alloc::vec::Vec;
use theseus_shared::constants::hardware;
use uefi::proto::device_path::{
    text::{AllowShortcuts, DevicePathToText, DisplayOnly},
    DevicePath,
};
use uefi::proto::loaded_image::LoadedImage;
use uefi::{boot::SearchType, Identify};

/// Hardware device information
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HardwareDevice {
    /// Device type (PCI, USB, SATA, etc.)
    pub device_type: u32,
    /// Device handle pointer
    pub handle_ptr: u64,
    /// Device path size in bytes
    pub device_path_size: u32,
    /// Device path data pointer
    pub device_path_ptr: u64,
}

/// Hardware inventory structure
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HardwareInventory {
    /// Number of devices
    pub device_count: u32,
    /// Pointer to device array
    pub devices_ptr: u64,
    /// Total size of inventory data
    pub total_size: u64,
}

/// Collect hardware inventory using device paths
pub fn collect_hardware_inventory(verbose: bool) -> Option<HardwareInventory> {
    write_line("Collecting hardware inventory using device paths...");

    let mut devices = Vec::new();

    // Find all handles that support DevicePath protocol (more comprehensive)
    let handles = match uefi::boot::locate_handle_buffer(SearchType::ByProtocol(&DevicePath::GUID))
    {
        Ok(handles) => handles,
        Err(_) => {
            write_line("✗ Failed to locate device path handles");
            return None;
        }
    };

    write_line(&format!("Found {} device path handles", handles.len()));

    if verbose {
        write_line("Verbose mode: Displaying detailed device information...");
    }

    // Get DevicePathToText protocol for converting paths to human-readable text
    // First, find a handle that supports the DevicePathToText protocol
    let device_path_to_text = match uefi::boot::get_handle_for_protocol::<DevicePathToText>() {
        Ok(handle) => {
            // Now open the protocol on the found handle
            match uefi::boot::open_protocol_exclusive::<DevicePathToText>(handle) {
                Ok(protocol) => Some(protocol),
                Err(_) => {
                    if verbose {
                        write_line("  Warning: Failed to open DevicePathToText protocol");
                    }
                    None
                }
            }
        }
        Err(_) => {
            if verbose {
                write_line(
                    "  Warning: DevicePathToText protocol not available, showing raw pointers only",
                );
            }
            None
        }
    };

    // Enumerate all handles and get device path information
    for (index, handle) in handles.iter().enumerate() {
        let mut device = HardwareDevice {
            device_type: 1, // Generic Device Path
            handle_ptr: core::ptr::addr_of!(*handle) as usize as u64,
            device_path_size: 0,
            device_path_ptr: 0,
        };

        // Try to get the device path for this handle
        if let Ok(device_path_protocol) = uefi::boot::open_protocol_exclusive::<DevicePath>(*handle)
        {
            // Store the device path pointer
            device.device_path_ptr = core::ptr::addr_of!(device_path_protocol) as usize as u64;

            // Try to convert to human-readable text if protocol is available
            if let Some(ref to_text) = device_path_to_text {
                match to_text.convert_device_path_to_text(
                    &device_path_protocol,
                    DisplayOnly(false),
                    AllowShortcuts(true),
                ) {
                    Ok(text) => {
                        if verbose {
                            // Convert PoolString to readable format using as_str_in_buf
                            let mut path_buf = alloc::string::String::new();
                            if let Ok(_) = text.as_str_in_buf(&mut path_buf) {
                                write_line(&format!("  Device {}: {}", index + 1, path_buf));
                            } else {
                                write_line(&format!(
                                    "  Device {}: [Failed to convert to string]",
                                    index + 1
                                ));
                            }
                        }
                        // PoolString doesn't have len(), so we'll use a placeholder size
                        device.device_path_size = 64; // Placeholder size for device path text
                    }
                    Err(_) => {
                        if verbose {
                            write_line(&format!(
                                "  Device {}: Handle 0x{:016x} (failed to convert to text)",
                                index + 1,
                                core::ptr::addr_of!(*handle) as usize as u64
                            ));
                        }
                    }
                }
            } else {
                if verbose {
                    write_line(&format!(
                        "  Device {}: Handle 0x{:016x} (no text conversion)",
                        index + 1,
                        core::ptr::addr_of!(*handle) as usize as u64
                    ));
                }
            }
        }

        devices.push(device);

        // Limit to first 10 devices to avoid hanging
        // Bounds check to prevent excessive memory usage and potential hangs
        if devices.len() >= hardware::MAX_HARDWARE_DEVICES {
            if verbose {
                write_line(&format!(
                    "  ... (limiting to first {} devices)",
                    hardware::MAX_HARDWARE_DEVICES
                ));
            }
            break;
        }
    }

    if devices.is_empty() {
        write_line("✗ No hardware devices found");
        return None;
    }

    let inventory = HardwareInventory {
        device_count: devices.len() as u32,
        devices_ptr: devices.as_ptr() as u64,
        total_size: (devices.len() * core::mem::size_of::<HardwareDevice>()) as u64,
    };

    if verbose {
        write_line(&format!(
            "✓ Hardware inventory collected: {} devices",
            devices.len()
        ));
        write_line(&format!(
            "  Device struct size: {} bytes",
            core::mem::size_of::<HardwareDevice>()
        ));
        write_line(&format!(
            "  Total inventory size: {} bytes ({:.2} KB)",
            inventory.total_size,
            inventory.total_size as f64 / 1024.0
        ));
        write_line(&format!(
            "  Devices array pointer: 0x{:016x}",
            inventory.devices_ptr
        ));
    } else {
        write_line(&format!(
            "✓ Hardware inventory collected: {} devices",
            devices.len()
        ));
    }

    Some(inventory)
}

/// Display hardware inventory in a beautiful format
pub fn display_hardware_inventory(inventory: &HardwareInventory) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                Hardware Device Inventory               │");
    write_line("├─────────────────────────────────────────────────────────┤");

    // Bounds check for device count
    if inventory.device_count == 0 {
        write_line("│ No hardware devices found                            │");
        write_line("└─────────────────────────────────────────────────────────┘");
        write_line("");
        return;
    }

    // Sanity check: ensure device count is reasonable
    if inventory.device_count > hardware::MAX_HARDWARE_DEVICES as u32 {
        write_line(&format!(
            "│ Warning: Device count ({}) exceeds maximum ({}) │",
            inventory.device_count,
            hardware::MAX_HARDWARE_DEVICES
        ));
    }

    write_line(&alloc::format!(
        "│ Total Devices Found: {}                              │",
        inventory.device_count
    ));
    write_line(&alloc::format!(
        "│ Inventory Size: {} bytes ({:.2} KB)              │",
        inventory.total_size,
        inventory.total_size as f64 / 1024.0
    ));
    write_line(&alloc::format!(
        "│ Devices Pointer: 0x{:016x}                    │",
        inventory.devices_ptr
    ));
    write_line("├─────────────────────────────────────────────────────────┤");

    // Display device types summary
    write_line("│ Device Type Summary:                                  │");
    write_line("│   Device Path Handles: Found                         │");
    write_line("│   Human-Readable Paths: Available                     │");
    write_line("│   Protocol Support: DevicePathToText                  │");

    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Get the current loaded image device path
pub fn get_loaded_image_device_path() -> Option<(u64, u32)> {
    write_line("Getting loaded image device path...");

    let image_handle = uefi::boot::image_handle();

    // Try to open LoadedImage protocol
    let loaded_image = match uefi::boot::open_protocol_exclusive::<LoadedImage>(image_handle) {
        Ok(img) => img,
        Err(_) => {
            write_line("✗ Failed to open LoadedImage protocol");
            return None;
        }
    };

    // Get the device path
    let device_path = loaded_image.file_path();

    if let Some(path) = device_path {
        // For now, we'll just store the pointer without trying to get size
        // The DevicePath doesn't expose len() and as_ptr() methods directly
        let path_ptr = core::ptr::addr_of!(*path) as *const u8 as usize as u64;
        let path_size = 0; // We can't easily get the size without more complex parsing

        write_line(&format!("✓ Loaded image device path found"));
        write_line(&format!("  Path pointer: 0x{:016x}", path_ptr));

        Some((path_ptr, path_size))
    } else {
        write_line("✗ No device path available for loaded image");
        None
    }
}
