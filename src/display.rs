extern crate alloc;

use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;
use uefi::mem::memory_map::MemoryMap;
use crate::acpi::parse_acpi_tables;
use crate::drivers::OutputDriver;
use alloc::format;

/// Helper function to display prettily formatted GOP information
pub fn display_gop_info(output_driver: &mut OutputDriver, width: u32, height: u32, pixel_format: UefiPixelFormat, stride: u32, fb_base: u64, fb_size: u64) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                    GOP Information                     │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    output_driver.write_line(&format!("│ Resolution:     {} x {}      pixels", width, height));
    output_driver.write_line(&format!("│ Pixel Format:    {:?}", pixel_format));
    output_driver.write_line(&format!("│ Stride:       {} bytes per scanline", stride));
    output_driver.write_line(&format!("│ Framebuffer Base: 0x{:016x}", fb_base));
    output_driver.write_line(&format!("│ Framebuffer Size:  {} bytes", fb_size));
    output_driver.write_line(&format!("│ Total Pixels:    {}", width * height));
    
    // Calculate bytes per pixel
    let bytes_per_pixel = stride / width;
    output_driver.write_line(&format!("│ Bytes per Pixel:      {}", bytes_per_pixel));
    
    // Calculate expected size
    let expected_size = (width * height * bytes_per_pixel) as u64;
    output_driver.write_line(&format!("│ Calculated Size:  {} bytes", expected_size));
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display memory map information
pub fn display_memory_map_info(output_driver: &mut OutputDriver, descriptor_size: u32, descriptor_version: u32, entries: u32, total_size: u32) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                  Memory Map Information                │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    output_driver.write_line(&format!("│ Descriptor Size:     {} bytes", descriptor_size));
    output_driver.write_line(&format!("│ Descriptor Version:    {}", descriptor_version));
    output_driver.write_line(&format!("│ Total Entries:      {}", entries));
    output_driver.write_line(&format!("│ Total Buffer Size:   {} bytes", total_size));
    output_driver.write_line(&format!("│ Memory Usage:     {} bytes", total_size));
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display memory map entry
pub fn display_memory_map_entry(output_driver: &mut OutputDriver, entry_num: usize, descriptor: &uefi::mem::memory_map::MemoryDescriptor) {
    let size_bytes = descriptor.page_count * 4096; // UEFI pages are 4KB
    let attributes = descriptor.att;
    
    output_driver.write_line(&format!(
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
pub fn display_memory_map_entries(output_driver: &mut OutputDriver, memory_map: &uefi::mem::memory_map::MemoryMapOwned) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                              Memory Map Entries                                      │");
    output_driver.write_line("├─────────────────────────────────────────────────────────────────────────────────────────┤");
    output_driver.write_line("│ #   | Type                 | Physical Start    | Pages   | Size (bytes) | Attributes │");
    output_driver.write_line("├─────────────────────────────────────────────────────────────────────────────────────────┤");
    
    let mut entry_count = 0;
    for descriptor in memory_map.entries() {
        display_memory_map_entry(output_driver, entry_count, &descriptor);
        entry_count += 1;
        
        // Limit display to first 20 entries to avoid overwhelming output
        if entry_count >= 20 {
            output_driver.write_line("│ ... | (truncated)          | ...              | ...     | ...          | ...        │");
            break;
        }
    }
    
    output_driver.write_line("└─────────────────────────────────────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display prettily formatted ACPI information
pub fn display_acpi_info(output_driver: &mut OutputDriver, rsdp_address: u64) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                    ACPI Information                     │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    
    if rsdp_address != 0 {
        output_driver.write_line(&format!("│ RSDP Table Found: 0x{:016X}", rsdp_address));
        output_driver.write_line("│ ✓ ACPI support will be available to kernel");
        
        // Try to parse ACPI tables for additional information
        if let Err(e) = parse_acpi_tables(output_driver, rsdp_address) {
            output_driver.write_line(&format!("│ ⚠ ACPI parsing failed: {}", e));
        }
    } else {
        output_driver.write_line("│ RSDP Table: Not Found");
        output_driver.write_line("│ ✗ No ACPI support will be available to kernel");
    }
    
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display device tree information
pub fn display_device_tree_info(output_driver: &mut OutputDriver, dtb_ptr: u64, dtb_size: u64) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                  Device Tree Information                │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    
    if dtb_ptr != 0 {
        output_driver.write_line(&format!("│ DTB Address: 0x{:016x}                           │", dtb_ptr));
        output_driver.write_line(&format!("│ DTB Size: {} bytes ({:.2} KB)                   │", dtb_size, dtb_size as f64 / 1024.0));
        output_driver.write_line("│ Status: ✓ Device tree blob found                    │");
    } else {
        output_driver.write_line("│ DTB Address: Not found (0x0000000000000000)          │");
        output_driver.write_line("│ DTB Size: 0 bytes                                   │");
        output_driver.write_line("│ Status: ✗ Device tree blob not found               │");
        output_driver.write_line("│ Note: Device tree is typically used on ARM systems │");
    }
    
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display firmware information
pub fn display_firmware_info(output_driver: &mut OutputDriver, vendor_ptr: u64, vendor_len: u32, revision: u32) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                   Firmware Information                  │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    
    if vendor_ptr != 0 {
        output_driver.write_line(&format!("│ Vendor Pointer: 0x{:016x}                        │", vendor_ptr));
        output_driver.write_line(&format!("│ Vendor Length: {} characters                     │", vendor_len));
        output_driver.write_line(&format!("│ Firmware Revision: 0x{:08x}                      │", revision));
        output_driver.write_line("│ Status: ✓ Firmware information collected            │");
    } else {
        output_driver.write_line("│ Vendor Pointer: Not available (0x0000000000000000)   │");
        output_driver.write_line("│ Vendor Length: 0 characters                         │");
        output_driver.write_line("│ Firmware Revision: Not available (0x00000000)       │");
        output_driver.write_line("│ Status: ✗ Firmware information not available        │");
    }
    
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display boot time information
pub fn display_boot_time_info(output_driver: &mut OutputDriver, seconds: u64, nanoseconds: u32) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                   Boot Time Information                 │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    
    if seconds != 0 {
        output_driver.write_line(&format!("│ Boot Time: {} seconds since epoch                 │", seconds));
        output_driver.write_line(&format!("│ Nanoseconds: {} ns                               │", nanoseconds));
        output_driver.write_line("│ Status: ✓ Boot time information collected            │");
    } else {
        output_driver.write_line("│ Boot Time: Not available (0 seconds)                 │");
        output_driver.write_line("│ Nanoseconds: Not available (0 ns)                    │");
        output_driver.write_line("│ Status: ✗ Boot time information not available        │");
    }
    
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display boot device path information
pub fn display_boot_device_path_info(output_driver: &mut OutputDriver, device_path_ptr: u64, device_path_size: u32) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                Boot Device Path Information             │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    
    if device_path_ptr != 0 {
        output_driver.write_line(&format!("│ Device Path Pointer: 0x{:016x}                    │", device_path_ptr));
        output_driver.write_line(&format!("│ Device Path Size: {} bytes                        │", device_path_size));
        output_driver.write_line("│ Status: ✓ Boot device path information collected     │");
    } else {
        output_driver.write_line("│ Device Path Pointer: Not available (0x0000000000000000) │");
        output_driver.write_line("│ Device Path Size: 0 bytes                            │");
        output_driver.write_line("│ Status: ✗ Boot device path information not available │");
    }
    
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

/// Helper function to display CPU information
pub fn display_cpu_info(output_driver: &mut OutputDriver, cpu_count: u32, cpu_features: u64, microcode_revision: u32) {
    output_driver.write_line("");
    output_driver.write_line("┌─────────────────────────────────────────────────────────┐");
    output_driver.write_line("│                     CPU Information                     │");
    output_driver.write_line("├─────────────────────────────────────────────────────────┤");
    
    if cpu_count != 0 {
        output_driver.write_line(&format!("│ CPU Count: {} processors                            │", cpu_count));
        output_driver.write_line(&format!("│ CPU Features: 0x{:016x}                        │", cpu_features));
        output_driver.write_line(&format!("│ Microcode Revision: 0x{:08x}                      │", microcode_revision));
        output_driver.write_line("│ Status: ✓ CPU information collected                  │");
    } else {
        output_driver.write_line("│ CPU Count: Not available (0 processors)              │");
        output_driver.write_line("│ CPU Features: Not available (0x0000000000000000)      │");
        output_driver.write_line("│ Microcode Revision: Not available (0x00000000)       │");
        output_driver.write_line("│ Status: ✗ CPU information not available              │");
    }
    
    output_driver.write_line("└─────────────────────────────────────────────────────────┘");
    output_driver.write_line("");
}

// Hardware inventory display moved to hardware.rs module

// ACPI parsing moved to acpi.rs module
