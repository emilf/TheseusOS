//! Display and output module
//!
//! This module provides functions for displaying kernel information and debug output.
//! It handles formatted output to the QEMU debug port and provides utilities for
//! displaying handoff structure contents in a readable format.

use theseus_shared::handoff::Handoff;

/// Simple kernel output function that writes directly to QEMU debug port
///
/// This function provides a simple way to output text from the kernel to the
/// QEMU debug port (0xE9). It's used for kernel logging and debug output.
///
/// # Parameters
///
/// * `message` - The message string to output
pub fn kernel_write_line(message: &str) {
    theseus_shared::qemu_println!(message);
}

/// Print key/value pair with hex value
///
/// # Parameters
///
/// * `name` - The field name to display
/// * `value` - The 64-bit value to display in hex
fn print_kv_u64(name: &str, value: u64) {
    theseus_shared::qemu_print!(name);
    theseus_shared::qemu_print!(": ");
    theseus_shared::qemu_print!("0x");
    theseus_shared::print_hex_u64_0xe9!(value);
    theseus_shared::qemu_println!("");
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
    kernel_write_line("");
    kernel_write_line("┌─────────────────────────────────────────────────────────┐");
    kernel_write_line("│                     Handoff Contents                    │");
    kernel_write_line("├─────────────────────────────────────────────────────────┤");

    print_kv_u32("size", h.size);
    print_kv_u32("handoff_version", h.handoff_version);

    kernel_write_line("│ Graphics (GOP)");
    print_kv_u64("gop_fb_base", h.gop_fb_base);
    print_kv_u64("gop_fb_size", h.gop_fb_size);
    print_kv_u32("gop_width", h.gop_width);
    print_kv_u32("gop_height", h.gop_height);
    print_kv_u32("gop_stride", h.gop_stride);
    print_kv_u32("gop_pixel_format", h.gop_pixel_format);

    kernel_write_line("│ Memory Map");
    print_kv_u64("memory_map_buffer_ptr", h.memory_map_buffer_ptr);
    print_kv_u32("memory_map_descriptor_size", h.memory_map_descriptor_size);
    print_kv_u32(
        "memory_map_descriptor_version",
        h.memory_map_descriptor_version,
    );
    print_kv_u32("memory_map_entries", h.memory_map_entries);
    print_kv_u32("memory_map_size", h.memory_map_size);

    kernel_write_line("│ ACPI");
    print_kv_u64("acpi_rsdp", h.acpi_rsdp);

    kernel_write_line("│ UEFI");
    print_kv_u64("uefi_system_table", h.uefi_system_table);
    print_kv_u64("uefi_image_handle", h.uefi_image_handle);

    kernel_write_line("│ Firmware");
    print_kv_u64("firmware_vendor_ptr", h.firmware_vendor_ptr);
    print_kv_u32("firmware_vendor_len", h.firmware_vendor_len);
    print_kv_u32("firmware_revision", h.firmware_revision);

    kernel_write_line("│ Boot Time / Device Path");
    print_kv_u64("boot_time_seconds", h.boot_time_seconds);
    print_kv_u32("boot_time_nanoseconds", h.boot_time_nanoseconds);
    print_kv_u64("boot_device_path_ptr", h.boot_device_path_ptr);
    print_kv_u32("boot_device_path_size", h.boot_device_path_size);

    kernel_write_line("│ CPU / Hardware");
    print_kv_u32("cpu_count", h.cpu_count);
    print_kv_u64("cpu_features", h.cpu_features);
    print_kv_u32("microcode_revision", h.microcode_revision);
    print_kv_u32("hardware_device_count", h.hardware_device_count);
    print_kv_u64("hardware_inventory_ptr", h.hardware_inventory_ptr);
    print_kv_u64("hardware_inventory_size", h.hardware_inventory_size);

    kernel_write_line("│ Kernel Image");
    print_kv_u64("kernel_virtual_base", h.kernel_virtual_base);
    print_kv_u64("kernel_physical_base", h.kernel_physical_base);
    print_kv_u64("kernel_virtual_entry", h.kernel_virtual_entry);
    print_kv_u64("kernel_image_size", h.kernel_image_size);

    kernel_write_line("│ Temp Heap");
    print_kv_u64("temp_heap_base", h.temp_heap_base);
    print_kv_u64("temp_heap_size", h.temp_heap_size);

    kernel_write_line("│ Status");
    print_kv_u32("boot_services_exited", h.boot_services_exited);

    kernel_write_line("└─────────────────────────────────────────────────────────┘");

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
    kernel_write_line("");
    kernel_write_line("Handoff raw bytes (hex):");
    let mut offset: usize = 0;
    while offset < size {
        // print offset
        theseus_shared::qemu_print!("+");
        theseus_shared::print_hex_u64_0xe9!(offset as u64);
        theseus_shared::qemu_print!(": ");
        let mut i = 0;
        while i < 16 && (offset + i) < size {
            let b = unsafe { core::ptr::read_volatile(ptr.add(offset + i)) } as u64;
            if i > 0 {
                theseus_shared::qemu_print!(" ");
            }
            // two hex digits
            let hi = ((b >> 4) & 0xF) as u8;
            let lo = (b & 0xF) as u8;
            let to_ch = |n: u8| -> u8 {
                if n < 10 {
                    b'0' + n
                } else {
                    b'A' + (n - 10)
                }
            };
            theseus_shared::out_char_0xe9!(to_ch(hi));
            theseus_shared::out_char_0xe9!(to_ch(lo));
            i += 1;
        }
        theseus_shared::qemu_println!("");
        offset += 16;
    }
}

/// Old summary function kept for reference; not used by default
#[allow(dead_code)]
pub fn display_handoff_info(_handoff: &Handoff) {
    kernel_write_line("");
    kernel_write_line("┌─────────────────────────────────────────────────────────┐");
    kernel_write_line("│                Kernel Handoff Information               │");
    kernel_write_line("├─────────────────────────────────────────────────────────┤");
    kernel_write_line("│ Handoff Size: Available");
    kernel_write_line("│ Memory Map Entries: Available");
    kernel_write_line("│ Memory Map Size: Available");
    kernel_write_line("│ Memory Map Buffer: Available");
    kernel_write_line("│ ACPI RSDP: Available");
    kernel_write_line("│ Framebuffer: Available");
    kernel_write_line("│ Hardware Devices: Available");
    kernel_write_line("│");
    kernel_write_line("│ Virtual Memory Information:");
    kernel_write_line("│   Virtual Base: Available");
    kernel_write_line("│   Physical Base: Available");
    kernel_write_line("│   Virtual Entry: Available");
    kernel_write_line("│   Page Table Root: Available");
    kernel_write_line("│   Virtual Memory: Available");
    kernel_write_line("└─────────────────────────────────────────────────────────┘");
    kernel_write_line("");
}

pub fn kernel_write_serial(msg: &str) {
    use crate::drivers::manager::driver_manager;
    use crate::drivers::traits::DeviceClass;

    let mut manager = driver_manager().lock();
    let _ = manager.write_class(DeviceClass::Serial, msg.as_bytes());
}
