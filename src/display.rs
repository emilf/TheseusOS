extern crate alloc;

use uefi::prelude::*;
use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;
use uefi::mem::memory_map::MemoryMap;
use crate::serial::serial_write_line;
use crate::acpi::parse_acpi_tables;
use alloc::format;

/// Helper function to display prettily formatted GOP information
pub fn display_gop_info(serial_handle: Option<Handle>, width: u32, height: u32, pixel_format: UefiPixelFormat, stride: u32, fb_base: u64, fb_size: u64) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                    GOP Information                     │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    serial_write_line(serial_handle, &format!("│ Resolution:     {} x {}      pixels", width, height));
    serial_write_line(serial_handle, &format!("│ Pixel Format:    {:?}", pixel_format));
    serial_write_line(serial_handle, &format!("│ Stride:       {} bytes per scanline", stride));
    serial_write_line(serial_handle, &format!("│ Framebuffer Base: 0x{:016x}", fb_base));
    serial_write_line(serial_handle, &format!("│ Framebuffer Size:  {} bytes", fb_size));
    serial_write_line(serial_handle, &format!("│ Total Pixels:    {}", width * height));
    
    // Calculate bytes per pixel
    let bytes_per_pixel = stride / width;
    serial_write_line(serial_handle, &format!("│ Bytes per Pixel:      {}", bytes_per_pixel));
    
    // Calculate expected size
    let expected_size = (width * height * bytes_per_pixel) as u64;
    serial_write_line(serial_handle, &format!("│ Calculated Size:  {} bytes", expected_size));
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display memory map information
pub fn display_memory_map_info(serial_handle: Option<Handle>, descriptor_size: u32, descriptor_version: u32, entries: u32, total_size: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                  Memory Map Information                │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    serial_write_line(serial_handle, &format!("│ Descriptor Size:     {} bytes", descriptor_size));
    serial_write_line(serial_handle, &format!("│ Descriptor Version:    {}", descriptor_version));
    serial_write_line(serial_handle, &format!("│ Total Entries:      {}", entries));
    serial_write_line(serial_handle, &format!("│ Total Buffer Size:   {} bytes", total_size));
    serial_write_line(serial_handle, &format!("│ Memory Usage:     {} bytes", total_size));
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display memory map entry
pub fn display_memory_map_entry(serial_handle: Option<Handle>, entry_num: usize, descriptor: &uefi::mem::memory_map::MemoryDescriptor) {
    let size_bytes = descriptor.page_count * 4096; // UEFI pages are 4KB
    let attributes = descriptor.att;
    
    serial_write_line(serial_handle, &format!(
        "│ {:3}: {:<20} | 0x{:016x} | {:8} pages | {:10} bytes | 0x{:08x}",
        entry_num + 1,
        format!("{:?}", descriptor.ty),
        descriptor.phys_start,
        descriptor.page_count,
        size_bytes,
        attributes
    ));
}

/// Helper function to display memory map entries in a table format
pub fn display_memory_map_entries(serial_handle: Option<Handle>, memory_map: &uefi::mem::memory_map::MemoryMapOwned) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                              Memory Map Entries                                      │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────────────────────────────────────┤");
    serial_write_line(serial_handle, "│ #   | Type                 | Physical Start    | Pages   | Size (bytes) | Attributes │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────────────────────────────────────┤");
    
    let mut entry_count = 0;
    for descriptor in memory_map.entries() {
        display_memory_map_entry(serial_handle, entry_count, &descriptor);
        entry_count += 1;
        
        // Limit display to first 20 entries to avoid overwhelming output
        if entry_count >= 20 {
            serial_write_line(serial_handle, "│ ... | (truncated)          | ...              | ...     | ...          | ...        │");
            break;
        }
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display prettily formatted ACPI information
pub fn display_acpi_info(serial_handle: Option<Handle>, rsdp_address: u64) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                    ACPI Information                     │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if rsdp_address != 0 {
        serial_write_line(serial_handle, &format!("│ RSDP Table Found: 0x{:016X}", rsdp_address));
        serial_write_line(serial_handle, "│ ✓ ACPI support will be available to kernel");
        
        // Try to parse ACPI tables for additional information
        if let Err(e) = parse_acpi_tables(serial_handle, rsdp_address) {
            serial_write_line(serial_handle, &format!("│ ⚠ ACPI parsing failed: {}", e));
        }
    } else {
        serial_write_line(serial_handle, "│ RSDP Table: Not Found");
        serial_write_line(serial_handle, "│ ✗ No ACPI support will be available to kernel");
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display device tree information
pub fn display_device_tree_info(serial_handle: Option<Handle>, dtb_ptr: u64, dtb_size: u64) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                  Device Tree Information                │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if dtb_ptr != 0 {
        serial_write_line(serial_handle, &format!("│ DTB Address: 0x{:016x}                           │", dtb_ptr));
        serial_write_line(serial_handle, &format!("│ DTB Size: {} bytes ({:.2} KB)                   │", dtb_size, dtb_size as f64 / 1024.0));
        serial_write_line(serial_handle, "│ Status: ✓ Device tree blob found                    │");
    } else {
        serial_write_line(serial_handle, "│ DTB Address: Not found (0x0000000000000000)          │");
        serial_write_line(serial_handle, "│ DTB Size: 0 bytes                                   │");
        serial_write_line(serial_handle, "│ Status: ✗ Device tree blob not found               │");
        serial_write_line(serial_handle, "│ Note: Device tree is typically used on ARM systems │");
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display firmware information
pub fn display_firmware_info(serial_handle: Option<Handle>, vendor_ptr: u64, vendor_len: u32, revision: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                   Firmware Information                  │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if vendor_ptr != 0 {
        serial_write_line(serial_handle, &format!("│ Vendor Pointer: 0x{:016x}                        │", vendor_ptr));
        serial_write_line(serial_handle, &format!("│ Vendor Length: {} characters                     │", vendor_len));
        serial_write_line(serial_handle, &format!("│ Firmware Revision: 0x{:08x}                      │", revision));
        serial_write_line(serial_handle, "│ Status: ✓ Firmware information collected            │");
    } else {
        serial_write_line(serial_handle, "│ Vendor Pointer: Not available (0x0000000000000000)   │");
        serial_write_line(serial_handle, "│ Vendor Length: 0 characters                         │");
        serial_write_line(serial_handle, "│ Firmware Revision: Not available (0x00000000)       │");
        serial_write_line(serial_handle, "│ Status: ✗ Firmware information not available        │");
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display boot time information
pub fn display_boot_time_info(serial_handle: Option<Handle>, seconds: u64, nanoseconds: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                   Boot Time Information                 │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if seconds != 0 {
        serial_write_line(serial_handle, &format!("│ Boot Time: {} seconds since epoch                 │", seconds));
        serial_write_line(serial_handle, &format!("│ Nanoseconds: {} ns                               │", nanoseconds));
        serial_write_line(serial_handle, "│ Status: ✓ Boot time information collected            │");
    } else {
        serial_write_line(serial_handle, "│ Boot Time: Not available (0 seconds)                 │");
        serial_write_line(serial_handle, "│ Nanoseconds: Not available (0 ns)                    │");
        serial_write_line(serial_handle, "│ Status: ✗ Boot time information not available        │");
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display boot device path information
pub fn display_boot_device_path_info(serial_handle: Option<Handle>, device_path_ptr: u64, device_path_size: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                Boot Device Path Information             │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if device_path_ptr != 0 {
        serial_write_line(serial_handle, &format!("│ Device Path Pointer: 0x{:016x}                    │", device_path_ptr));
        serial_write_line(serial_handle, &format!("│ Device Path Size: {} bytes                        │", device_path_size));
        serial_write_line(serial_handle, "│ Status: ✓ Boot device path information collected     │");
    } else {
        serial_write_line(serial_handle, "│ Device Path Pointer: Not available (0x0000000000000000) │");
        serial_write_line(serial_handle, "│ Device Path Size: 0 bytes                            │");
        serial_write_line(serial_handle, "│ Status: ✗ Boot device path information not available │");
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display CPU information
pub fn display_cpu_info(serial_handle: Option<Handle>, cpu_count: u32, cpu_features: u64, microcode_revision: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                     CPU Information                     │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if cpu_count != 0 {
        serial_write_line(serial_handle, &format!("│ CPU Count: {} processors                            │", cpu_count));
        serial_write_line(serial_handle, &format!("│ CPU Features: 0x{:016x}                        │", cpu_features));
        serial_write_line(serial_handle, &format!("│ Microcode Revision: 0x{:08x}                      │", microcode_revision));
        serial_write_line(serial_handle, "│ Status: ✓ CPU information collected                  │");
    } else {
        serial_write_line(serial_handle, "│ CPU Count: Not available (0 processors)              │");
        serial_write_line(serial_handle, "│ CPU Features: Not available (0x0000000000000000)      │");
        serial_write_line(serial_handle, "│ Microcode Revision: Not available (0x00000000)       │");
        serial_write_line(serial_handle, "│ Status: ✗ CPU information not available              │");
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

// Hardware inventory display moved to hardware.rs module

// ACPI parsing moved to acpi.rs module
