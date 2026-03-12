//! Module: display
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//!
//! INVARIANTS:
//! - This module contains human-oriented formatting helpers for boot/kernel state dumps, especially the handoff structure.
//! - These helpers are diagnostic surfaces layered on top of the current logging/output path; they are not separate sources of architectural truth.
//!
//! SAFETY:
//! - Raw-byte dumps and pointer-based formatting helpers assume the referenced structure is valid and readable in the current mapping state.
//! - Pretty-printing debug state is best-effort observability and must not become a hidden dependency for correct kernel behavior.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! Display-side debug formatting helpers.
//!
//! This module contains human-oriented dump helpers for handoff/kernel state.

use crate::log_trace;
use theseus_shared::handoff::Handoff;

/// Print one `name = hex value` pair for a 64-bit field.
fn print_kv_u64(name: &str, value: u64) {
    log_trace!("  {}: {:#x}", name, value);
}

/// Print one `name = hex value` pair for a 32-bit field.
fn print_kv_u32(name: &str, value: u32) {
    print_kv_u64(name, value as u64);
}

/// Pretty-print the handoff structure, optionally with a raw byte dump.
///
/// This is a diagnostic helper for inspecting what the bootloader handed to the
/// kernel; it is not part of the handoff contract itself.
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

/// Dump raw handoff bytes in a 16-byte-per-row hex table.
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

/// Write a string through the kernel serial device class.
pub fn kernel_write_serial(msg: &str) {
    use crate::drivers::manager::driver_manager;
    use crate::drivers::traits::DeviceClass;

    let mut manager = driver_manager().lock();
    let _ = manager.write_class(DeviceClass::Serial, msg.as_bytes());
}
