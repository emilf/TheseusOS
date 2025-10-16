//! Display and output module
//!
//! This module provides functions for displaying kernel information and debug output.
//! It handles formatted output to the QEMU debug port and provides utilities for
//! displaying handoff structure contents in a readable format.

use crate::log_trace;
use theseus_shared::handoff::Handoff;

/// Print key/value pair with hex value
///
/// # Parameters
///
/// * `name` - The field name to display
/// * `value` - The 64-bit value to display in hex
fn print_kv_u64(name: &str, value: u64) {
    log_trace!("  {}: {:#x}", name, value);
}

/// Print key/value pair with hex value (32-bit version)
///
/// # Parameters
///
/// * `name` - The field name to display
/// * `value` - The 32-bit value to display in hex
fn print_kv_u32(name: &str, value: u32) {
    print_kv_u64(name, value as u64);
}

/// Pretty print the Handoff structure with optional raw byte dump
///
/// This function displays the contents of the handoff structure in a formatted
/// table, showing all the system information passed from the bootloader to the kernel.
///
/// # Parameters
///
/// * `h` - Reference to the handoff structure to display
/// * `dump_bytes` - If true, also dump the raw bytes of the structure
pub fn dump_handoff(h: &Handoff, dump_bytes: bool) {
    log_trace!("┌─────────────────────────────────────────────────────────┐");
    log_trace!("│                     Handoff Contents                    │");
    log_trace!("├─────────────────────────────────────────────────────────┤");

    print_kv_u32("size", h.size);
    print_kv_u32("handoff_version", h.handoff_version);

    log_trace!("│ Graphics (GOP)");
    print_kv_u64("gop_fb_base", h.gop_fb_base);
    print_kv_u64("gop_fb_size", h.gop_fb_size);
    print_kv_u32("gop_width", h.gop_width);
    print_kv_u32("gop_height", h.gop_height);
    print_kv_u32("gop_stride", h.gop_stride);
    print_kv_u32("gop_pixel_format", h.gop_pixel_format);

    log_trace!("│ Memory Map");
    print_kv_u64("memory_map_buffer_ptr", h.memory_map_buffer_ptr);
    print_kv_u32("memory_map_descriptor_size", h.memory_map_descriptor_size);
    print_kv_u32(
        "memory_map_descriptor_version",
        h.memory_map_descriptor_version,
    );
    print_kv_u32("memory_map_entries", h.memory_map_entries);
    print_kv_u32("memory_map_size", h.memory_map_size);

    log_trace!("│ ACPI");
    print_kv_u64("acpi_rsdp", h.acpi_rsdp);

    log_trace!("│ UEFI");
    print_kv_u64("uefi_system_table", h.uefi_system_table);
    print_kv_u64("uefi_image_handle", h.uefi_image_handle);

    log_trace!("│ Firmware");
    print_kv_u64("firmware_vendor_ptr", h.firmware_vendor_ptr);
    print_kv_u32("firmware_vendor_len", h.firmware_vendor_len);
    print_kv_u32("firmware_revision", h.firmware_revision);

    log_trace!("│ Boot Time / Device Path");
    print_kv_u64("boot_time_seconds", h.boot_time_seconds);
    print_kv_u32("boot_time_nanoseconds", h.boot_time_nanoseconds);
    print_kv_u64("boot_device_path_ptr", h.boot_device_path_ptr);
    print_kv_u32("boot_device_path_size", h.boot_device_path_size);

    log_trace!("│ CPU / Hardware");
    print_kv_u32("cpu_count", h.cpu_count);
    print_kv_u64("cpu_features", h.cpu_features);
    print_kv_u32("microcode_revision", h.microcode_revision);
    print_kv_u32("hardware_device_count", h.hardware_device_count);
    print_kv_u64("hardware_inventory_ptr", h.hardware_inventory_ptr);
    print_kv_u64("hardware_inventory_size", h.hardware_inventory_size);

    log_trace!("│ Kernel Image");
    print_kv_u64("kernel_virtual_base", h.kernel_virtual_base);
    print_kv_u64("kernel_physical_base", h.kernel_physical_base);
    print_kv_u64("kernel_virtual_entry", h.kernel_virtual_entry);
    print_kv_u64("kernel_image_size", h.kernel_image_size);

    log_trace!("│ Temp Heap");
    print_kv_u64("temp_heap_base", h.temp_heap_base);
    print_kv_u64("temp_heap_size", h.temp_heap_size);

    log_trace!("│ Status");
    print_kv_u32("boot_services_exited", h.boot_services_exited);

    log_trace!("└─────────────────────────────────────────────────────────┘");

    if dump_bytes {
        dump_handoff_bytes(
            h as *const Handoff as *const u8,
            core::mem::size_of::<Handoff>(),
        );
    }
}

/// Dump raw bytes of the handoff in a hex table (16 bytes per row)
///
/// This function displays the raw memory contents of the handoff structure
/// in a hexadecimal format, useful for debugging memory layout issues.
///
/// # Parameters
///
/// * `ptr` - Pointer to the start of the memory to dump
/// * `size` - Number of bytes to dump
///
/// # Safety
///
/// The caller must ensure that the memory pointed to by `ptr` is valid
/// and readable for `size` bytes.
pub fn dump_handoff_bytes(ptr: *const u8, size: usize) {
    log_trace!("Handoff raw bytes (hex):");
    let mut offset: usize = 0;
    while offset < size {
        // Collect 16 bytes for this line
        let mut line_bytes = [0u8; 16];
        let mut count = 0;
        while count < 16 && (offset + count) < size {
            line_bytes[count] = unsafe { core::ptr::read_volatile(ptr.add(offset + count)) };
            count += 1;
        }
        
        // Log the line
        if count > 0 {
            log_trace!("  +{:#06x}: {:02x?}", offset, &line_bytes[..count]);
        }
        offset += 16;
    }
}

pub fn kernel_write_serial(msg: &str) {
    use crate::drivers::manager::driver_manager;
    use crate::drivers::traits::DeviceClass;

    let mut manager = driver_manager().lock();
    let _ = manager.write_class(DeviceClass::Serial, msg.as_bytes());
}
