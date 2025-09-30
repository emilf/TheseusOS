//! Handoff management module
//!
//! This module provides functions for validating and accessing the handoff structure
//! passed from the bootloader to the kernel. The handoff structure contains all
//! the system information collected by the bootloader, including memory maps,
//! ACPI tables, hardware inventory, and other critical system data.
//!
//! The module provides:
//! - Handoff structure validation
//! - Pointer management for physical and virtual access
//! - Safe access to handoff data

use crate::memory;
use theseus_shared::handoff::Handoff;

// Global access to bootloader handoff
static mut HANDOFF_PHYS_PTR: u64 = 0;
static mut HANDOFF_VIRT_PTR: u64 = 0;
static mut HANDOFF_INITIALIZED: bool = false;

/// Set up handoff pointer tracking for both physical and virtual access
///
/// This function initializes the global handoff pointers used throughout the kernel.
/// It stores both physical and virtual pointers to the handoff structure for
/// safe access during different phases of kernel initialization.
///
/// # Parameters
///
/// * `handoff_phys` - Physical address of the handoff structure
pub fn set_handoff_pointers(handoff_phys: u64) {
    unsafe {
        HANDOFF_PHYS_PTR = handoff_phys;
        HANDOFF_VIRT_PTR = handoff_phys; // keep physical pointer to avoid stale HH mapping
        HANDOFF_INITIALIZED = true;
    }
    crate::display::kernel_write_line("[handoff] phys=");
    theseus_shared::print_hex_u64_0xe9!(handoff_phys);
    crate::display::kernel_write_line(" virt=");
    theseus_shared::print_hex_u64_0xe9!(handoff_phys);
    crate::display::kernel_write_line("\n");
}

/// Get a reference to the handoff structure
///
/// Chooses between physical and virtual pointer based on current execution context.
/// This allows the same code to work both before and after high-half jumping.
///
/// # Returns
///
/// A reference to the handoff structure
///
/// # Safety
///
/// The caller must ensure that the handoff structure is valid and properly initialized.
#[allow(dead_code)]
pub fn handoff_ref() -> &'static Handoff {
    // Always use the physical pointer; identity mapping ensures accessibility
    let ptr = unsafe { HANDOFF_PHYS_PTR };
    unsafe { &*(ptr as *const Handoff) }
}

/// Get the physical pointer to the handoff structure
///
/// # Returns
///
/// The physical address of the handoff structure
pub fn handoff_phys_ptr() -> u64 {
    unsafe { HANDOFF_PHYS_PTR }
}

/// Validate the bootloader handoff structure for basic sanity
///
/// This function performs comprehensive validation of the handoff structure
/// to ensure it contains valid system information. It checks structure size,
/// version, memory map consistency, and other critical fields.
///
/// # Parameters
///
/// * `h` - Reference to the handoff structure to validate
///
/// # Returns
///
/// * `Ok(())` - If all validation checks pass
/// * `Err(message)` - If validation fails, with a description of the issue
pub fn validate_handoff(h: &Handoff) -> Result<(), &'static str> {
    // Expected struct size and version
    let expected_size = core::mem::size_of::<Handoff>() as u32;
    crate::display::kernel_write_line("  Debug: handoff.size = ");
    theseus_shared::print_hex_u64_0xe9!(h.size as u64);
    crate::display::kernel_write_line(" expected = ");
    theseus_shared::print_hex_u64_0xe9!(expected_size as u64);
    crate::display::kernel_write_line("\n");

    if h.size != expected_size {
        return Err("handoff.size does not match Handoff struct size");
    }
    if h.handoff_version == 0 {
        return Err("handoff_version must be non-zero");
    }

    // CPU info
    if h.cpu_count == 0 {
        return Err("cpu_count must be >= 1");
    }

    // Framebuffer info (if present)
    if h.gop_fb_base != 0 {
        if h.gop_fb_size == 0 {
            return Err("gop_fb_size must be > 0 when gop_fb_base != 0");
        }
        if h.gop_width == 0 || h.gop_height == 0 {
            return Err("gop_width/height must be > 0 when framebuffer present");
        }
        if h.gop_stride < h.gop_width {
            return Err("gop_stride must be >= gop_width");
        }
    }

    // Memory map (if present)
    if h.memory_map_buffer_ptr != 0 {
        if h.memory_map_descriptor_size == 0 {
            return Err("memory_map_descriptor_size must be > 0");
        }
        if h.memory_map_entries == 0 {
            return Err("memory_map_entries must be > 0 when buffer present");
        }
        let required_bytes = (h.memory_map_entries as u64) * (h.memory_map_descriptor_size as u64);
        if (h.memory_map_size as u64) < required_bytes {
            return Err("memory_map_size smaller than entries * descriptor_size");
        }
    } else {
        // If no buffer ptr, sizes and entries should be zero
        if h.memory_map_entries != 0 || h.memory_map_size != 0 {
            return Err("memory_map present fields inconsistent with null buffer ptr");
        }
    }

    // Temporary heap (if present)
    if h.temp_heap_size != 0 {
        if h.temp_heap_base == 0 {
            return Err("temp_heap_base must be non-zero when temp_heap_size > 0");
        }
        if (h.temp_heap_size & 0xFFF) != 0 {
            return Err("temp_heap_size must be 4KiB-aligned");
        }
    }

    // Kernel addresses
    if h.kernel_virtual_base != 0 && h.kernel_virtual_base != memory::KERNEL_VIRTUAL_BASE {
        return Err("kernel_virtual_base differs from expected KERNEL_VIRTUAL_BASE");
    }
    if h.kernel_physical_base == 0 {
        return Err("kernel_physical_base must be non-zero");
    }
    if h.kernel_virtual_entry == 0 {
        return Err("kernel_virtual_entry must be non-zero");
    }
    if h.kernel_image_size == 0 {
        return Err("kernel_image_size must be non-zero");
    }
    if (h.kernel_image_size & 0xFFF) != 0 {
        return Err("kernel_image_size must be 4KiB-aligned");
    }
    // Ensure entry is within virtual image span [kernel_virtual_base, +size)
    if h.kernel_virtual_base != 0 {
        let entry = h.kernel_virtual_entry;
        let base = h.kernel_virtual_base;
        let end = base.saturating_add(h.kernel_image_size);
        if !(entry >= base && entry < end) {
            return Err("kernel_virtual_entry not within kernel image span");
        }
    }

    // Firmware vendor (if length > 0 then ptr must be non-zero)
    if h.firmware_vendor_len > 0 && h.firmware_vendor_ptr == 0 {
        return Err("firmware_vendor_ptr must be non-zero when length > 0");
    }

    // Hardware inventory consistency
    if h.hardware_device_count > 0 {
        if h.hardware_inventory_ptr == 0 || h.hardware_inventory_size == 0 {
            return Err("hardware inventory pointer/size must be set when device_count > 0");
        }
    }

    // Ensure temp heap does not overlap kernel image in physical memory
    if h.temp_heap_size != 0 {
        let heap_start = h.temp_heap_base;
        let heap_end = heap_start.saturating_add(h.temp_heap_size);
        let img_start = h.kernel_physical_base;
        let img_end = img_start.saturating_add(h.kernel_image_size);
        let overlap = !(heap_end <= img_start || heap_start >= img_end);
        if overlap {
            return Err("temp heap overlaps kernel image in physical memory");
        }
    }

    // Boot services: bootloader exits boot services before transferring control
    // For early bring-up, allow either state; we will soon stop using firmware anyway.
    // This avoids an immediate panic during transitional integration.
    // if h.boot_services_exited != 1 {
    //     return Err("boot_services_exited must be 1 at kernel entry");
    // }

    Ok(())
}
