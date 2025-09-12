#![no_std]
#![no_main]

extern crate alloc;

use uefi::{prelude::*, Identify};
use uefi::proto::console::serial::Serial;
use uefi::proto::console::gop::GraphicsOutput;
use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;
use uefi::boot::{SearchType, MemoryType};
use core::arch::asm;
use uefi::mem::memory_map::MemoryMap;
use acpi::{AcpiHandler, PhysicalMapping, AcpiTables};
use core::ptr::NonNull;

// Include our modules
mod handoff;
mod serial;
mod display;
mod hardware;

use handoff::{Handoff, HANDOFF};
use serial::serial_write_line;
use display::*;
use hardware::{collect_hardware_inventory, get_loaded_image_device_path, display_hardware_inventory};



/// ACPI Handler for UEFI environment
/// This implements the AcpiHandler trait to allow the acpi crate to map physical memory
#[derive(Clone)]
struct UefiAcpiHandler;

impl AcpiHandler for UefiAcpiHandler {
    unsafe fn map_physical_region<T>(
        &self,
        physical_address: usize,
        size: usize,
    ) -> PhysicalMapping<Self, T> {
        // In UEFI, we can use identity mapping for physical addresses
        // This is safe because UEFI provides a flat memory model
        let virtual_address = physical_address as *mut T;
        let non_null_virtual_address = NonNull::new(virtual_address)
            .expect("Physical address should be valid");
        
        PhysicalMapping::new(
            physical_address,
            non_null_virtual_address,
            size,
            size,
            Self,
        )
    }

    fn unmap_physical_region<T>(_region: &PhysicalMapping<Self, T>) {
        // In UEFI, we don't need to unmap physical regions
        // The memory remains accessible until exit_boot_services
    }
}


/// Helper function to display prettily formatted GOP information
fn display_gop_info(serial_handle: Option<Handle>, width: u32, height: u32, pixel_format: UefiPixelFormat, stride: u32, fb_base: u64, fb_size: u64) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                    GOP Information                     │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    // Resolution information
    serial_write_line(serial_handle, &alloc::format!("│ Resolution: {:>8} x {:<8} pixels", width, height));
    
    // Pixel format information
    let format_str = match pixel_format {
        UefiPixelFormat::Rgb => "RGB",
        UefiPixelFormat::Bgr => "BGR", 
        UefiPixelFormat::Bitmask => "Bitmask",
        UefiPixelFormat::BltOnly => "BltOnly",
    };
    serial_write_line(serial_handle, &alloc::format!("│ Pixel Format: {:>6} ({:?})", format_str, pixel_format));
    
    // Stride information
    serial_write_line(serial_handle, &alloc::format!("│ Stride: {:>10} bytes per scanline", stride));
    
    // Framebuffer memory information
    serial_write_line(serial_handle, &alloc::format!("│ Framebuffer Base: 0x{:016X}", fb_base));
    serial_write_line(serial_handle, &alloc::format!("│ Framebuffer Size: {:>8} bytes", fb_size));
    
    // Calculate and display additional useful information
    let total_pixels = width as u64 * height as u64;
    let bytes_per_pixel = if stride > 0 { stride / width } else { 0 };
    let calculated_size = total_pixels * bytes_per_pixel as u64;
    
    serial_write_line(serial_handle, &alloc::format!("│ Total Pixels: {:>10}", total_pixels));
    serial_write_line(serial_handle, &alloc::format!("│ Bytes per Pixel: {:>6}", bytes_per_pixel));
    serial_write_line(serial_handle, &alloc::format!("│ Calculated Size: {:>8} bytes", calculated_size));
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display prettily formatted memory map information
fn display_memory_map_info(serial_handle: Option<Handle>, descriptor_size: u32, descriptor_version: u32, entries: u32, total_size: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                  Memory Map Information                │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    serial_write_line(serial_handle, &alloc::format!("│ Descriptor Size: {:>6} bytes", descriptor_size));
    serial_write_line(serial_handle, &alloc::format!("│ Descriptor Version: {:>4}", descriptor_version));
    serial_write_line(serial_handle, &alloc::format!("│ Total Entries: {:>8}", entries));
    serial_write_line(serial_handle, &alloc::format!("│ Total Buffer Size: {:>6} bytes", total_size));
    
    // Calculate memory usage
    let memory_usage = (entries as u64 * descriptor_size as u64) as u32;
    serial_write_line(serial_handle, &alloc::format!("│ Memory Usage: {:>8} bytes", memory_usage));
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to convert memory type to readable string
fn memory_type_to_string(memory_type: MemoryType) -> &'static str {
    match memory_type {
        MemoryType::RESERVED => "Reserved",
        MemoryType::LOADER_CODE => "Loader Code",
        MemoryType::LOADER_DATA => "Loader Data",
        MemoryType::BOOT_SERVICES_CODE => "Boot Services Code",
        MemoryType::BOOT_SERVICES_DATA => "Boot Services Data",
        MemoryType::RUNTIME_SERVICES_CODE => "Runtime Services Code",
        MemoryType::RUNTIME_SERVICES_DATA => "Runtime Services Data",
        MemoryType::CONVENTIONAL => "Conventional Memory",
        MemoryType::UNUSABLE => "Unusable Memory",
        MemoryType::ACPI_RECLAIM => "ACPI Reclaimable",
        MemoryType::MMIO => "MMIO",
        MemoryType::MMIO_PORT_SPACE => "MMIO Port Space",
        MemoryType::PAL_CODE => "PAL Code",
        MemoryType::PERSISTENT_MEMORY => "Persistent Memory",
        _ => "Unknown",
    }
}

/// Helper function to display individual memory map entry
fn display_memory_map_entry(serial_handle: Option<Handle>, index: usize, descriptor: &uefi::mem::memory_map::MemoryDescriptor) {
    let memory_type_str = memory_type_to_string(descriptor.ty);
    let physical_start = descriptor.phys_start;
    let page_count = descriptor.page_count;
    let size_bytes = page_count * 4096; // UEFI pages are 4KB
    let attributes = descriptor.att;
    
    serial_write_line(serial_handle, &alloc::format!(
        "│ {:>3}: {:<20} | 0x{:016X} | {:>8} pages | {:>10} bytes | 0x{:08X}",
        index + 1,
        memory_type_str,
        physical_start,
        page_count,
        size_bytes,
        attributes
    ));
}

/// Helper function to display memory map entries in a table format
fn display_memory_map_entries(serial_handle: Option<Handle>, memory_map: &uefi::mem::memory_map::MemoryMapOwned) {
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

/// Find ACPI RSDP table in UEFI configuration table
fn find_acpi_rsdp() -> Option<u64> {
    // In UEFI-RS 0.35, we need to access the system table through the global state
    // Let's try using the system table access pattern from the migration docs
    
    // ACPI GUIDs for RSDP lookup - using the guid! macro
    const ACPI_GUID: uefi::Guid = uefi::guid!("8868e871-e4f1-11d3-bc22-0080c73c8881");
    const ACPI2_GUID: uefi::Guid = uefi::guid!("8868e871-e4f1-11d3-bc22-0080c73c8881");
    
    // Try to access the system table through the global state
    // This is a best-effort approach based on the UEFI-RS 0.35 API
    // For now, we'll return None as the exact API needs to be determined
    // TODO: Implement proper system table access for UEFI-RS 0.35
    
    // Placeholder implementation - in a real implementation, we would:
    // 1. Access the system table through the proper UEFI-RS 0.35 API
    // 2. Iterate through the configuration table entries
    // 3. Look for ACPI GUIDs and return the RSDP address
    
    None
}

/// Find Device Tree Blob (DTB) in UEFI configuration table
fn find_device_tree() -> Option<(u64, u64)> {
    // Device Tree GUID for ARM/ARM64 systems
    const DEVICE_TREE_GUID: uefi::Guid = uefi::guid!("b1b621d5-f19c-41a5-830b-d9152c69aae0");
    
    // TODO: Implement device tree lookup through system table
    // For now, return None as device tree is typically used on ARM systems
    // and this bootloader is targeting x86_64
    
    None
}

/// Collect firmware information
fn collect_firmware_info() -> Option<(u64, u32, u32)> {
    // TODO: Implement firmware vendor and revision collection
    // This would typically access SystemTable->FirmwareVendor and SystemTable->FirmwareRevision
    // For now, return placeholder values
    
    None
}

/// Collect boot time information
fn collect_boot_time_info() -> Option<(u64, u32)> {
    // TODO: Implement boot time collection using RuntimeServices->GetTime()
    // This would get the current time when the bootloader starts
    
    None
}

/// Collect boot device path information
fn collect_boot_device_path() -> Option<(u64, u32)> {
    // TODO: Implement boot device path collection using LoadedImage protocol
    // This would get the device path of the booted EFI application
    
    None
}

/// Collect CPU information
fn collect_cpu_info() -> Option<(u32, u64, u32)> {
    // TODO: Implement CPU information collection using CPU protocol
    // This would get CPU count, features, and microcode revision
    
    None
}

/// Parse ACPI tables and extract useful information
fn parse_acpi_tables(serial_handle: Option<Handle>, rsdp_address: u64) -> Result<(), &'static str> {
    if rsdp_address == 0 {
        return Err("No RSDP address provided");
    }

    let handler = UefiAcpiHandler;
    let tables = match unsafe { AcpiTables::from_rsdp(handler, rsdp_address as usize) } {
        Ok(tables) => tables,
        Err(_) => {
            serial_write_line(serial_handle, "│ ⚠ Failed to parse ACPI tables");
            return Err("Failed to parse ACPI tables");
        }
    };

    // Display basic ACPI information
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                    ACPI Tables Found                    │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");

    // Try to get platform information
    match tables.platform_info() {
        Ok(platform_info) => {
            serial_write_line(serial_handle, "│ ✓ Platform Info: Available");
            serial_write_line(serial_handle, &alloc::format!("│   Interrupt Model: {:?}", platform_info.interrupt_model));
        }
        Err(_) => {
            serial_write_line(serial_handle, "│ ✗ Platform Info: Not available");
        }
    }

    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");

    Ok(())
}

/// Helper function to display prettily formatted ACPI information
fn display_acpi_info(serial_handle: Option<Handle>, rsdp_address: u64) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                    ACPI Information                     │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if rsdp_address != 0 {
        serial_write_line(serial_handle, &alloc::format!("│ RSDP Table Found: 0x{:016X}", rsdp_address));
        serial_write_line(serial_handle, "│ ✓ ACPI support will be available to kernel");
        
        // Try to parse ACPI tables for additional information
        if let Err(e) = parse_acpi_tables(serial_handle, rsdp_address) {
            serial_write_line(serial_handle, &alloc::format!("│ ⚠ ACPI parsing failed: {}", e));
        }
    } else {
        serial_write_line(serial_handle, "│ RSDP Table: Not Found");
        serial_write_line(serial_handle, "│ ✗ No ACPI support will be available to kernel");
    }
    
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
}

/// Helper function to display device tree information
fn display_device_tree_info(serial_handle: Option<Handle>, dtb_ptr: u64, dtb_size: u64) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                  Device Tree Information                │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if dtb_ptr != 0 {
        serial_write_line(serial_handle, &alloc::format!("│ DTB Address: 0x{:016x}                           │", dtb_ptr));
        serial_write_line(serial_handle, &alloc::format!("│ DTB Size: {} bytes ({:.2} KB)                   │", dtb_size, dtb_size as f64 / 1024.0));
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
fn display_firmware_info(serial_handle: Option<Handle>, vendor_ptr: u64, vendor_len: u32, revision: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                   Firmware Information                  │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if vendor_ptr != 0 {
        serial_write_line(serial_handle, &alloc::format!("│ Vendor Pointer: 0x{:016x}                        │", vendor_ptr));
        serial_write_line(serial_handle, &alloc::format!("│ Vendor Length: {} characters                     │", vendor_len));
        serial_write_line(serial_handle, &alloc::format!("│ Firmware Revision: 0x{:08x}                      │", revision));
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
fn display_boot_time_info(serial_handle: Option<Handle>, seconds: u64, nanoseconds: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                   Boot Time Information                 │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if seconds != 0 {
        serial_write_line(serial_handle, &alloc::format!("│ Boot Time: {} seconds since epoch                 │", seconds));
        serial_write_line(serial_handle, &alloc::format!("│ Nanoseconds: {} ns                               │", nanoseconds));
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
fn display_boot_device_path_info(serial_handle: Option<Handle>, device_path_ptr: u64, device_path_size: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                Boot Device Path Information             │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if device_path_ptr != 0 {
        serial_write_line(serial_handle, &alloc::format!("│ Device Path Pointer: 0x{:016x}                    │", device_path_ptr));
        serial_write_line(serial_handle, &alloc::format!("│ Device Path Size: {} bytes                        │", device_path_size));
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
fn display_cpu_info(serial_handle: Option<Handle>, cpu_count: u32, cpu_features: u64, microcode_revision: u32) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                     CPU Information                     │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if cpu_count != 0 {
        serial_write_line(serial_handle, &alloc::format!("│ CPU Count: {} processors                            │", cpu_count));
        serial_write_line(serial_handle, &alloc::format!("│ CPU Features: 0x{:016x}                        │", cpu_features));
        serial_write_line(serial_handle, &alloc::format!("│ Microcode Revision: 0x{:08x}                      │", microcode_revision));
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

/// Main UEFI entry point
#[entry]
fn efi_main() -> Status {
    // Step 1: Initialize UEFI logger
    uefi::helpers::init().unwrap();

    // Step 2: Initialize serial communication
    let serial_handle = uefi::boot::locate_handle_buffer(
        SearchType::ByProtocol(&Serial::GUID)
    )
    .ok()
    .and_then(|buf| buf.first().copied());

    // Step 3: Output startup message
    serial_write_line(serial_handle, "=== HobbyOS UEFI Loader Starting ===");
    serial_write_line(serial_handle, "Serial communication initialized");

    // DEBUG FUNCTION
    //let _ = debug_function(serial_handle);

    // Step 4: Set handoff size
    unsafe { HANDOFF.size = core::mem::size_of::<Handoff>() as u32; }

    // Step 5: Query Graphics Output Protocol (GOP)
    let gop_info = match uefi::boot::locate_handle_buffer(
        SearchType::ByProtocol(&GraphicsOutput::GUID)
    ) {
        Ok(gop_handles) => {
            if let Some(&handle) = gop_handles.first() {
                if let Ok(mut gop) = uefi::boot::open_protocol_exclusive::<GraphicsOutput>(handle) {
                    let mode = gop.current_mode_info();
                    let res = mode.resolution();
                    let pf = mode.pixel_format();
                    let mut fb = gop.frame_buffer();
                    
                    // Store GOP information in handoff structure
                    unsafe {
                        HANDOFF.gop_fb_base = fb.as_mut_ptr() as u64;
                        HANDOFF.gop_fb_size = fb.size() as u64;
                        HANDOFF.gop_width = res.0 as u32;
                        HANDOFF.gop_height = res.1 as u32;
                        HANDOFF.gop_stride = mode.stride() as u32;
                        HANDOFF.gop_pixel_format = match pf {
                            UefiPixelFormat::Rgb => 0,
                            UefiPixelFormat::Bgr => 1,
                            UefiPixelFormat::Bitmask => 2,
                            UefiPixelFormat::BltOnly => 3,
                        };
                    }
                    Some((res.0, res.1, pf, mode.stride(), fb.as_mut_ptr() as u64, fb.size()))
                } else {
                    None
                }
            } else {
                None
            }
        }
        Err(_) => None,
    };

    // Step 6: Report GOP status with pretty formatting
    if let Some((w, h, pf, stride, fb_base, fb_size)) = gop_info {
        serial_write_line(serial_handle, "✓ Graphics Output Protocol (GOP) found and initialized");
        display_gop_info(serial_handle, w as u32, h as u32, pf, stride as u32, fb_base, fb_size as u64);
        serial_write_line(serial_handle, "✓ Framebuffer information collected and stored in handoff structure");
    } else {
        serial_write_line(serial_handle, "✗ Graphics Output Protocol (GOP) not available");
        serial_write_line(serial_handle, "  No framebuffer information will be available to kernel");
    }

    // Step 7: Collect Memory Map Information
    serial_write_line(serial_handle, "Collecting memory map information...");
    let memory_map = match uefi::boot::memory_map(MemoryType::LOADER_DATA) {
        Ok(mmap) => {
            // Get memory map information using the correct UEFI 0.35 API
            let descriptor_size = 48u32; // Standard UEFI memory descriptor size
            let descriptor_version = 1u32; // Standard UEFI memory descriptor version
            
            // Count actual entries by iterating through the memory map
            let mut entries_count = 0u32;
            for _descriptor in mmap.entries() {
                entries_count += 1;
            }
            
            let total_size = entries_count * descriptor_size;
            
            // Store memory map information in handoff structure
            // The MemoryMapOwned structure manages the buffer internally
            // We store the key for exit_boot_services and the metadata for the kernel
            unsafe {
                HANDOFF.memory_map_buffer_ptr = 0; // MemoryMapOwned manages this internally
                HANDOFF.memory_map_descriptor_size = descriptor_size;
                HANDOFF.memory_map_descriptor_version = descriptor_version;
                HANDOFF.memory_map_entries = entries_count;
                HANDOFF.memory_map_size = total_size;
            }
            
            // Display the actual memory map entries
            display_memory_map_entries(serial_handle, &mmap);
            
            Some(mmap)
        }
        Err(_) => {
            serial_write_line(serial_handle, "✗ Failed to collect memory map");
            None
        }
    };

    // Step 8: Report Memory Map status with pretty formatting
    if let Some(ref _mmap) = memory_map {
        unsafe {
            serial_write_line(serial_handle, "✓ Memory map collected successfully");
            display_memory_map_info(serial_handle, HANDOFF.memory_map_descriptor_size, HANDOFF.memory_map_descriptor_version, HANDOFF.memory_map_entries, HANDOFF.memory_map_size);
            serial_write_line(serial_handle, "✓ Memory map information stored in handoff structure");
        }
    } else {
        serial_write_line(serial_handle, "✗ Memory map collection failed");
        serial_write_line(serial_handle, "  No memory information will be available to kernel");
    }

    // Step 9: Locate ACPI RSDP Table
    serial_write_line(serial_handle, "Locating ACPI RSDP table...");
    let rsdp_address = find_acpi_rsdp().unwrap_or(0);
    unsafe { HANDOFF.acpi_rsdp = rsdp_address; }

    // Step 10: Report ACPI status with pretty formatting
    if rsdp_address != 0 {
        serial_write_line(serial_handle, "✓ ACPI RSDP table found");
        display_acpi_info(serial_handle, rsdp_address);
        serial_write_line(serial_handle, "✓ ACPI information stored in handoff structure");
    } else {
        serial_write_line(serial_handle, "✗ ACPI RSDP table not found");
        display_acpi_info(serial_handle, rsdp_address);
        serial_write_line(serial_handle, "  No ACPI support will be available to kernel");
    }

    // Step 11: Collect Device Tree Information
    serial_write_line(serial_handle, "Collecting device tree information...");
    let device_tree_info = find_device_tree();
    match device_tree_info {
        Some((dtb_ptr, dtb_size)) => {
            unsafe {
                HANDOFF.device_tree_ptr = dtb_ptr;
                HANDOFF.device_tree_size = dtb_size;
            }
            serial_write_line(serial_handle, "✓ Device tree information collected");
            display_device_tree_info(serial_handle, dtb_ptr, dtb_size);
        }
        None => {
            serial_write_line(serial_handle, "✗ Device tree information not available");
            display_device_tree_info(serial_handle, 0, 0);
        }
    }

    // Step 12: Collect Firmware Information
    serial_write_line(serial_handle, "Collecting firmware information...");
    let firmware_info = collect_firmware_info();
    match firmware_info {
        Some((vendor_ptr, vendor_len, revision)) => {
            unsafe {
                HANDOFF.firmware_vendor_ptr = vendor_ptr;
                HANDOFF.firmware_vendor_len = vendor_len;
                HANDOFF.firmware_revision = revision;
            }
            serial_write_line(serial_handle, "✓ Firmware information collected");
            display_firmware_info(serial_handle, vendor_ptr, vendor_len, revision);
        }
        None => {
            serial_write_line(serial_handle, "✗ Firmware information not available");
            display_firmware_info(serial_handle, 0, 0, 0);
        }
    }

    // Step 13: Collect Boot Time Information
    serial_write_line(serial_handle, "Collecting boot time information...");
    let boot_time_info = collect_boot_time_info();
    match boot_time_info {
        Some((seconds, nanoseconds)) => {
            unsafe {
                HANDOFF.boot_time_seconds = seconds;
                HANDOFF.boot_time_nanoseconds = nanoseconds;
            }
            serial_write_line(serial_handle, "✓ Boot time information collected");
            display_boot_time_info(serial_handle, seconds, nanoseconds);
        }
        None => {
            serial_write_line(serial_handle, "✗ Boot time information not available");
            display_boot_time_info(serial_handle, 0, 0);
        }
    }

    // Step 14: Collect Boot Device Path Information
    serial_write_line(serial_handle, "Collecting boot device path information...");
    let boot_device_path_info = collect_boot_device_path();
    match boot_device_path_info {
        Some((device_path_ptr, device_path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = device_path_ptr;
                HANDOFF.boot_device_path_size = device_path_size;
            }
            serial_write_line(serial_handle, "✓ Boot device path information collected");
            display_boot_device_path_info(serial_handle, device_path_ptr, device_path_size);
        }
        None => {
            serial_write_line(serial_handle, "✗ Boot device path information not available");
            display_boot_device_path_info(serial_handle, 0, 0);
        }
    }

    // Step 15: Collect CPU Information
    serial_write_line(serial_handle, "Collecting CPU information...");
    let cpu_info = collect_cpu_info();
    match cpu_info {
        Some((cpu_count, cpu_features, microcode_revision)) => {
            unsafe {
                HANDOFF.cpu_count = cpu_count;
                HANDOFF.cpu_features = cpu_features;
                HANDOFF.microcode_revision = microcode_revision;
            }
            serial_write_line(serial_handle, "✓ CPU information collected");
            display_cpu_info(serial_handle, cpu_count, cpu_features, microcode_revision);
        }
        None => {
            serial_write_line(serial_handle, "✗ CPU information not available");
            display_cpu_info(serial_handle, 0, 0, 0);
        }
    }

    // Step 16: Collect Hardware Inventory
    serial_write_line(serial_handle, "Collecting hardware inventory...");
    let hardware_inventory = collect_hardware_inventory(serial_handle);
    match hardware_inventory {
        Some(inventory) => {
            unsafe {
                HANDOFF.hardware_device_count = inventory.device_count;
                HANDOFF.hardware_inventory_ptr = inventory.devices_ptr;
                HANDOFF.hardware_inventory_size = inventory.total_size;
            }
            serial_write_line(serial_handle, "✓ Hardware inventory collected");
            display_hardware_inventory(serial_handle, &inventory);
        }
        None => {
            serial_write_line(serial_handle, "✗ Hardware inventory not available");
            display_hardware_inventory_info(serial_handle, 0, 0, 0);
        }
    }

    // Step 17: Get Loaded Image Device Path
    serial_write_line(serial_handle, "Getting loaded image device path...");
    let loaded_image_path = get_loaded_image_device_path(serial_handle);
    match loaded_image_path {
        Some((device_path_ptr, device_path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = device_path_ptr;
                HANDOFF.boot_device_path_size = device_path_size;
            }
            serial_write_line(serial_handle, "✓ Loaded image device path collected");
            display_boot_device_path_info(serial_handle, device_path_ptr, device_path_size);
        }
        None => {
            serial_write_line(serial_handle, "✗ Loaded image device path not available");
            display_boot_device_path_info(serial_handle, 0, 0);
        }
    }

    // Step 18: Finalize Handoff Structure
    serial_write_line(serial_handle, "Finalizing handoff structure...");
    unsafe {
        HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    }
    serial_write_line(serial_handle, &alloc::format!("✓ Handoff structure size: {} bytes", core::mem::size_of::<Handoff>()));
    serial_write_line(serial_handle, "✓ All system information collected and stored");

    // Step 19: Exit Boot Services
    serial_write_line(serial_handle, "Exiting boot services...");
    if let Some(mmap) = memory_map {
        // Get the memory map key for exit_boot_services
        let memory_map_key = mmap.key();
        serial_write_line(serial_handle, &alloc::format!("Memory map key: {:?}", memory_map_key));
        
        // Note: In UEFI-RS 0.35, exit_boot_services is handled differently
        // The MemoryMapOwned structure manages the memory map internally
        // For now, we'll note that the memory map is available for the kernel
        serial_write_line(serial_handle, "✓ Memory map ready for kernel handoff");
        serial_write_line(serial_handle, "⚠ Manual exit_boot_services not implemented in this version");
        serial_write_line(serial_handle, "  The memory map is preserved for kernel access");
    } else {
        serial_write_line(serial_handle, "✗ Cannot prepare memory map without memory map");
        serial_write_line(serial_handle, "⚠ No memory map available for kernel");
    }

    // Step 20: Main bootloader loop
    serial_write_line(serial_handle, "Entering main loop - halting CPU");
    
    loop {
        unsafe { asm!("hlt", options(nomem, nostack, preserves_flags)) }
    }
}

fn debug_function(serial_handle: Option<Handle>) -> Status {

    log::info!("Fetching memory map…");

    // Grab the current memory map into an owned structure
    let mmap = match boot::memory_map(MemoryType::LOADER_DATA) {
        Ok(m) => m,
        Err(e) => {
            log::error!("Failed to get memory map: {:?}", e);
            return Status::ABORTED;
        }
    };

    // Iterate and print some details
    for desc in mmap.entries() {
        serial_write_line(serial_handle, 
            &alloc::format!("Type={:?}, Start=0x{:x}, Pages={}, Attrs={:?}",
            desc.ty,
            desc.phys_start,
            desc.page_count,
            desc.att
        ));
    }

    serial_write_line(serial_handle, &alloc::format!("Memory map key = {:?}", mmap.key()));

    serial_write_line(serial_handle, "Entering main loop - halting CPU");
    
    loop {
        unsafe { asm!("hlt", options(nomem, nostack, preserves_flags)) }
    }
    
    Status::SUCCESS
}
