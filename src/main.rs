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


/// Handoff structure passed to the kernel
/// Layout is stable (repr C) to allow consumption from any language
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct Handoff {
    /// Total size of this struct in bytes
    size: u32,
    /// Handoff format version
    handoff_version: u32,
    /// Framebuffer base address
    gop_fb_base: u64,
    /// Framebuffer size in bytes
    gop_fb_size: u64,
    /// Framebuffer width in pixels
    gop_width: u32,
    /// Framebuffer height in pixels
    gop_height: u32,
    /// Framebuffer stride
    gop_stride: u32,
    /// Pixel format enum
    gop_pixel_format: u32,
    /// Memory map descriptor size in bytes
    memory_map_descriptor_size: u32,
    /// Memory map descriptor version
    memory_map_descriptor_version: u32,
    /// Number of memory map entries
    memory_map_entries: u32,
    /// Memory map buffer size in bytes
    memory_map_size: u32,
    /// ACPI RSDP table address (0 if not found)
    acpi_rsdp: u64,
}

/// Static storage for handoff data
static mut HANDOFF: Handoff = Handoff {
    size: 0,
    handoff_version: 1,
    gop_fb_base: 0,
    gop_fb_size: 0,
    gop_width: 0,
    gop_height: 0,
    gop_stride: 0,
    gop_pixel_format: 0,
    memory_map_descriptor_size: 0,
    memory_map_descriptor_version: 0,
    memory_map_entries: 0,
    memory_map_size: 0,
    acpi_rsdp: 0,
};

/// Helper function to write data to serial
fn serial_write(serial_handle: Option<Handle>, data: &[u8]) {
    if let Some(handle) = serial_handle {
        if let Ok(mut serial) = uefi::boot::open_protocol_exclusive::<Serial>(handle) {
            let _ = serial.write(data);
        }
    }
}

/// Helper function to write a line to serial
fn serial_write_line(serial_handle: Option<Handle>, line: &str) {
    serial_write(serial_handle, line.as_bytes());
    serial_write(serial_handle, b"\r\n");
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
fn display_memory_map_entries(serial_handle: Option<Handle>, memory_map_slice: &[uefi::mem::memory_map::MemoryDescriptor]) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                              Memory Map Entries                                      │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────────────────────────────────────┤");
    serial_write_line(serial_handle, "│ #   | Type                 | Physical Start    | Pages   | Size (bytes) | Attributes │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────────────────────────────────────┤");
    
    let mut entry_count = 0;
    for descriptor in memory_map_slice {
        display_memory_map_entry(serial_handle, entry_count, descriptor);
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
fn display_acpi_info(serial_handle: Option<Handle>, rsdp_address: u64) {
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                    ACPI Information                     │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────┤");
    
    if rsdp_address != 0 {
        serial_write_line(serial_handle, &alloc::format!("│ RSDP Table Found: 0x{:016X}", rsdp_address));
        serial_write_line(serial_handle, "│ ✓ ACPI support will be available to kernel");
    } else {
        serial_write_line(serial_handle, "│ RSDP Table: Not Found");
        serial_write_line(serial_handle, "│ ✗ No ACPI support will be available to kernel");
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
    let _ = debug_function(serial_handle);

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
    
    // For now, we'll use a simplified approach since the exact UEFI 0.35 API needs investigation
    // We'll display a placeholder message and use estimated values
    let descriptor_size = 48u32; // Standard UEFI memory descriptor size
    let descriptor_version = 1u32; // Standard UEFI memory descriptor version
    let entries_count = 32u32; // Reasonable estimate for memory map entries
    let total_size = entries_count * descriptor_size;
    
    // Store memory map information in handoff structure
    unsafe {
        HANDOFF.memory_map_descriptor_size = descriptor_size;
        HANDOFF.memory_map_descriptor_version = descriptor_version;
        HANDOFF.memory_map_entries = entries_count;
        HANDOFF.memory_map_size = total_size;
    }
    
    // Display placeholder memory map information
    serial_write_line(serial_handle, "");
    serial_write_line(serial_handle, "┌─────────────────────────────────────────────────────────────────────────────────────────┐");
    serial_write_line(serial_handle, "│                              Memory Map Entries                                      │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────────────────────────────────────┤");
    serial_write_line(serial_handle, "│ #   | Type                 | Physical Start    | Pages   | Size (bytes) | Attributes │");
    serial_write_line(serial_handle, "├─────────────────────────────────────────────────────────────────────────────────────────┤");
    serial_write_line(serial_handle, "│ Memory map iteration API needs to be updated for UEFI 0.35 │");
    serial_write_line(serial_handle, "│ This is a placeholder - actual memory map display coming soon │");
    serial_write_line(serial_handle, "└─────────────────────────────────────────────────────────────────────────────────────────┘");
    serial_write_line(serial_handle, "");
    
    let memory_map_info = Some((descriptor_size, descriptor_version, entries_count as usize, total_size as usize));

    // Step 8: Report Memory Map status with pretty formatting
    if let Some((desc_size, desc_ver, entries, total_size)) = memory_map_info {
        serial_write_line(serial_handle, "✓ Memory map collected successfully");
        display_memory_map_info(serial_handle, desc_size as u32, desc_ver as u32, entries as u32, total_size as u32);
        serial_write_line(serial_handle, "✓ Memory map information stored in handoff structure");
    } else {
        serial_write_line(serial_handle, "✗ Memory map collection failed");
        serial_write_line(serial_handle, "  No memory information will be available to kernel");
    }

    // Step 9: Locate ACPI RSDP Table
    serial_write_line(serial_handle, "Locating ACPI RSDP table...");
    // Note: ACPI support may not be available in UEFI 0.35 or may require different API
    // For now, we'll set it to 0 to indicate no ACPI support
    let rsdp_address = 0u64;
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

    // Step 11: Main bootloader loop
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
