extern crate alloc;

use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;
use uefi::mem::memory_map::MemoryMap;
use crate::acpi::parse_acpi_tables;
use crate::drivers::manager::write_line;
use alloc::format;

/// Helper function to display prettily formatted GOP information
pub fn display_gop_info(width: u32, height: u32, pixel_format: UefiPixelFormat, stride: u32, fb_base: u64, fb_size: u64) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                    GOP Information                     │");
    write_line("├─────────────────────────────────────────────────────────┤");
    write_line(&format!("│ Resolution:     {} x {}      pixels", width, height));
    write_line(&format!("│ Pixel Format:    {:?}", pixel_format));
    write_line(&format!("│ Stride:       {} bytes per scanline", stride));
    write_line(&format!("│ Framebuffer Base: 0x{:016x}", fb_base));
    write_line(&format!("│ Framebuffer Size:  {} bytes", fb_size));
    write_line(&format!("│ Total Pixels:    {}", width * height));
    
    // Calculate bytes per pixel
    let bytes_per_pixel = stride / width;
    write_line(&format!("│ Bytes per Pixel:      {}", bytes_per_pixel));
    
    // Calculate expected size
    let expected_size = (width * height * bytes_per_pixel) as u64;
    write_line(&format!("│ Calculated Size:  {} bytes", expected_size));
    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Helper function to display memory map information
pub fn display_memory_map_info(descriptor_size: u32, descriptor_version: u32, entries: u32, total_size: u32) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                  Memory Map Information                │");
    write_line("├─────────────────────────────────────────────────────────┤");
    write_line(&format!("│ Descriptor Size:     {} bytes", descriptor_size));
    write_line(&format!("│ Descriptor Version:    {}", descriptor_version));
    write_line(&format!("│ Total Entries:      {}", entries));
    write_line(&format!("│ Total Buffer Size:   {} bytes", total_size));
    write_line(&format!("│ Memory Usage:     {} bytes", total_size));
    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Helper function to display memory map entry
pub fn display_memory_map_entry(entry_num: usize, descriptor: &uefi::mem::memory_map::MemoryDescriptor) {
    let size_bytes = descriptor.page_count * 4096; // UEFI pages are 4KB
    let attributes = descriptor.att;
    
    write_line(&format!(
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
pub fn display_memory_map_entries(memory_map: &uefi::mem::memory_map::MemoryMapOwned) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────────────────────────────────────┐");
    write_line("│                              Memory Map Entries                                      │");
    write_line("├─────────────────────────────────────────────────────────────────────────────────────────┤");
    write_line("│ #   | Type                 | Physical Start    | Pages   | Size (bytes) | Attributes │");
    write_line("├─────────────────────────────────────────────────────────────────────────────────────────┤");
    
    let mut entry_count = 0;
    for descriptor in memory_map.entries() {
        display_memory_map_entry(entry_count, &descriptor);
        entry_count += 1;
        
        // Limit display to first 20 entries to avoid overwhelming output
        if entry_count >= 20 {
            write_line("│ ... | (truncated)          | ...              | ...     | ...          | ...        │");
            break;
        }
    }
    
    write_line("└─────────────────────────────────────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Helper function to display prettily formatted ACPI information
pub fn display_acpi_info(rsdp_address: u64) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                    ACPI Information                     │");
    write_line("├─────────────────────────────────────────────────────────┤");
    
    if rsdp_address != 0 {
        write_line(&format!("│ RSDP Table Found: 0x{:016X}", rsdp_address));
        write_line("│ ✓ ACPI support will be available to kernel");
        
        // Try to parse ACPI tables for additional information
        if let Err(e) = parse_acpi_tables(rsdp_address) {
            write_line(&format!("│ ⚠ ACPI parsing failed: {}", e));
        }
    } else {
        write_line("│ RSDP Table: Not Found");
        write_line("│ ✗ No ACPI support will be available to kernel");
    }
    
    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

// Device tree display removed (x86-only)

/// Helper function to display firmware information
pub fn display_firmware_info(vendor_ptr: u64, vendor_len: u32, revision: u32) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                   Firmware Information                  │");
    write_line("├─────────────────────────────────────────────────────────┤");
    
    if vendor_ptr != 0 {
        write_line(&format!("│ Vendor Pointer: 0x{:016x}                        │", vendor_ptr));
        write_line(&format!("│ Vendor Length: {} characters                     │", vendor_len));
        write_line(&format!("│ Firmware Revision: 0x{:08x}                      │", revision));
        write_line("│ Status: ✓ Firmware information collected            │");
    } else {
        write_line("│ Vendor Pointer: Not available (0x0000000000000000)   │");
        write_line("│ Vendor Length: 0 characters                         │");
        write_line("│ Firmware Revision: Not available (0x00000000)       │");
        write_line("│ Status: ✗ Firmware information not available        │");
    }
    
    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Helper function to display boot time information
pub fn display_boot_time_info(seconds: u64, nanoseconds: u32) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                   Boot Time Information                 │");
    write_line("├─────────────────────────────────────────────────────────┤");
    
    if seconds != 0 {
        write_line(&format!("│ Boot Time: {} seconds since epoch                 │", seconds));
        write_line(&format!("│ Nanoseconds: {} ns                               │", nanoseconds));
        write_line("│ Status: ✓ Boot time information collected            │");
    } else {
        write_line("│ Boot Time: Not available (0 seconds)                 │");
        write_line("│ Nanoseconds: Not available (0 ns)                    │");
        write_line("│ Status: ✗ Boot time information not available        │");
    }
    
    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Helper function to display boot device path information
pub fn display_boot_device_path_info(device_path_ptr: u64, device_path_size: u32) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                Boot Device Path Information             │");
    write_line("├─────────────────────────────────────────────────────────┤");
    
    if device_path_ptr != 0 {
        write_line(&format!("│ Device Path Pointer: 0x{:016x}                    │", device_path_ptr));
        write_line(&format!("│ Device Path Size: {} bytes                        │", device_path_size));
        write_line("│ Status: ✓ Boot device path information collected     │");
    } else {
        write_line("│ Device Path Pointer: Not available (0x0000000000000000) │");
        write_line("│ Device Path Size: 0 bytes                            │");
        write_line("│ Status: ✗ Boot device path information not available │");
    }
    
    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Helper function to display CPU information
pub fn display_cpu_info(cpu_count: u32, cpu_features: u64, microcode_revision: u32) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                     CPU Information                     │");
    write_line("├─────────────────────────────────────────────────────────┤");
    
    if cpu_count != 0 {
        write_line(&format!("│ CPU Count: {} processors                            │", cpu_count));
        write_line(&format!("│ CPU Features: 0x{:016x}                        │", cpu_features));
        write_line(&format!("│ Microcode Revision: 0x{:08x}                      │", microcode_revision));
        write_line("│ Status: ✓ CPU information collected                  │");
    } else {
        write_line("│ CPU Count: Not available (0 processors)              │");
        write_line("│ CPU Features: Not available (0x0000000000000000)      │");
        write_line("│ Microcode Revision: Not available (0x00000000)       │");
        write_line("│ Status: ✗ CPU information not available              │");
    }
    
    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

// Hardware inventory display moved to hardware.rs module

// ACPI parsing moved to acpi.rs module
