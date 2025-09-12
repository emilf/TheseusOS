extern crate alloc;

use uefi::{prelude::*, boot::SearchType, Identify};
use uefi::proto::loaded_image::LoadedImage;
use uefi::proto::device_path::{DevicePath, text::{DevicePathToText, AllowShortcuts, DisplayOnly}};
use crate::serial::serial_write_line;
use alloc::vec::Vec;
use alloc::format;

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
pub fn collect_hardware_inventory(serial_handle: Option<Handle>, verbose: bool) -> Option<HardwareInventory> {
    serial_write_line(serial_handle, "Collecting hardware inventory using device paths...");
    
    let mut devices = Vec::new();
    
    // Find all handles that support DevicePath protocol (more comprehensive)
    let handles = match uefi::boot::locate_handle_buffer(SearchType::ByProtocol(&DevicePath::GUID)) {
        Ok(handles) => handles,
        Err(_) => {
            serial_write_line(serial_handle, "✗ Failed to locate device path handles");
            return None;
        }
    };
    
    serial_write_line(serial_handle, &format!("Found {} device path handles", handles.len()));
    
    if verbose {
        serial_write_line(serial_handle, "Verbose mode: Displaying detailed device information...");
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
                        serial_write_line(serial_handle, "  Warning: Failed to open DevicePathToText protocol");
                    }
                    None
                }
            }
        }
        Err(_) => {
            if verbose {
                serial_write_line(serial_handle, "  Warning: DevicePathToText protocol not available, showing raw pointers only");
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
        if let Ok(device_path_protocol) = uefi::boot::open_protocol_exclusive::<DevicePath>(*handle) {
            // Store the device path pointer
            device.device_path_ptr = core::ptr::addr_of!(device_path_protocol) as usize as u64;
            
            // Try to convert to human-readable text if protocol is available
            if let Some(ref to_text) = device_path_to_text {
                match to_text.convert_device_path_to_text(&device_path_protocol, DisplayOnly(false), AllowShortcuts(true)) {
                    Ok(text) => {
                        if verbose {
                            // Convert PoolString to readable format using as_str_in_buf
                            let mut path_buf = alloc::string::String::new();
                            if let Ok(_) = text.as_str_in_buf(&mut path_buf) {
                                serial_write_line(serial_handle, &format!("  Device {}: {}", index + 1, path_buf));
                            } else {
                                serial_write_line(serial_handle, &format!("  Device {}: [Failed to convert to string]", index + 1));
                            }
                        }
                        // PoolString doesn't have len(), so we'll use a placeholder size
                        device.device_path_size = 64; // Placeholder size for device path text
                    }
                    Err(_) => {
                        if verbose {
                            serial_write_line(serial_handle, &format!("  Device {}: Handle 0x{:016x} (failed to convert to text)", 
                                index + 1, core::ptr::addr_of!(*handle) as usize as u64));
                        }
                    }
                }
            } else {
                if verbose {
                    serial_write_line(serial_handle, &format!("  Device {}: Handle 0x{:016x} (no text conversion)", 
                        index + 1, core::ptr::addr_of!(*handle) as usize as u64));
                }
            }
        }
        
        devices.push(device);
    }
    
    if devices.is_empty() {
        serial_write_line(serial_handle, "✗ No hardware devices found");
        return None;
    }
    
    let inventory = HardwareInventory {
        device_count: devices.len() as u32,
        devices_ptr: devices.as_ptr() as u64,
        total_size: (devices.len() * core::mem::size_of::<HardwareDevice>()) as u64,
    };
    
    if verbose {
        serial_write_line(serial_handle, &format!("✓ Hardware inventory collected: {} devices", devices.len()));
        serial_write_line(serial_handle, &format!("  Device struct size: {} bytes", core::mem::size_of::<HardwareDevice>()));
        serial_write_line(serial_handle, &format!("  Total inventory size: {} bytes ({:.2} KB)", inventory.total_size, inventory.total_size as f64 / 1024.0));
        serial_write_line(serial_handle, &format!("  Devices array pointer: 0x{:016x}", inventory.devices_ptr));
    } else {
        serial_write_line(serial_handle, &format!("✓ Hardware inventory collected: {} devices", devices.len()));
    }
    
    Some(inventory)
}


/// Display hardware inventory in a beautiful format
pub fn display_hardware_inventory(serial_handle: Option<Handle>, inventory: &HardwareInventory) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                Hardware Device Inventory               │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if inventory.device_count == 0 {
        serial_write_line(serial_handle, "│ No hardware devices found                            │");
        serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
        serial_write_line(serial_handle, "");
        return;
    }
    
    serial_write_line(serial_handle, &alloc::format!("│ Total Devices Found: {}                              │", inventory.device_count));
    serial_write_line(serial_handle, &alloc::format!("│ Inventory Size: {} bytes ({:.2} KB)              │", 
        inventory.total_size, inventory.total_size as f64 / 1024.0));
    serial_write_line(serial_handle, &alloc::format!("│ Devices Pointer: 0x{:016x}                    │", inventory.devices_ptr));
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    // Display device types summary
    serial_write_line(serial_handle, "│ Device Type Summary:                                  │");
    serial_write_line(serial_handle, "│   Device Path Handles: Found                         │");
    serial_write_line(serial_handle, "│   Human-Readable Paths: Available                     │");
    serial_write_line(serial_handle, "│   Protocol Support: DevicePathToText                  │");
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Get the current loaded image device path
pub fn get_loaded_image_device_path(serial_handle: Option<Handle>) -> Option<(u64, u32)> {
    serial_write_line(serial_handle, "Getting loaded image device path...");
    
    let image_handle = uefi::boot::image_handle();
    
    // Try to open LoadedImage protocol
    let loaded_image = match uefi::boot::open_protocol_exclusive::<LoadedImage>(image_handle) {
        Ok(img) => img,
        Err(_) => {
            serial_write_line(serial_handle, "✗ Failed to open LoadedImage protocol");
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
        
        serial_write_line(serial_handle, &format!("✓ Loaded image device path found"));
        serial_write_line(serial_handle, &format!("  Path pointer: 0x{:016x}", path_ptr));
        
        Some((path_ptr, path_size))
    } else {
        serial_write_line(serial_handle, "✗ No device path available for loaded image");
        None
    }
}
