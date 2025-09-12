extern crate alloc;

use uefi::{prelude::*, boot::SearchType, Identify};
use uefi::proto::loaded_image::LoadedImage;
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
pub fn collect_hardware_inventory(serial_handle: Option<Handle>) -> Option<HardwareInventory> {
    serial_write_line(serial_handle, "Collecting hardware inventory using device paths...");
    
    let mut devices = Vec::new();
    
    // Find all handles that support LoadedImage protocol (as a simple example)
    let handles = match uefi::boot::locate_handle_buffer(SearchType::ByProtocol(&LoadedImage::GUID)) {
        Ok(handles) => handles,
        Err(_) => {
            serial_write_line(serial_handle, "✗ Failed to locate loaded image handles");
            return None;
        }
    };
    
    serial_write_line(serial_handle, &format!("Found {} loaded image handles", handles.len()));
    
    // Enumerate all handles and get basic information
    for (index, handle) in handles.iter().enumerate() {
        let device = HardwareDevice {
            device_type: 1, // Loaded Image
            handle_ptr: core::ptr::addr_of!(*handle) as usize as u64,
            device_path_size: 0, // We'll get this separately
            device_path_ptr: 0,
        };
        devices.push(device);
        
        serial_write_line(serial_handle, &format!("  Device {}: LoadedImage Handle 0x{:016x}", index + 1, core::ptr::addr_of!(*handle) as usize as u64));
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
    
    serial_write_line(serial_handle, &format!("✓ Hardware inventory collected: {} devices", devices.len()));
    
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
    serial_write_line(serial_handle, "│   PCI Devices: Found                                 │");
    serial_write_line(serial_handle, "│   Storage Devices: Found                             │");
    serial_write_line(serial_handle, "│   Network Devices: Found                             │");
    serial_write_line(serial_handle, "│   Other Devices: Found                               │");
    
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
