//! Boot sequence orchestration
//!
//! This module contains the main boot sequence logic that orchestrates
//! the collection of system information and preparation of the handoff structure.

use uefi::boot::{MemoryType, SearchType};
use uefi::mem::memory_map::MemoryMap;
use uefi::proto::console::gop::GraphicsOutput;
use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;
use uefi::Identify;
use uefi::Status;

use crate::acpi::find_acpi_rsdp;
use crate::display::*;
use crate::drivers::manager::write_line;
use crate::hardware::{
    collect_hardware_inventory, display_hardware_inventory, get_loaded_image_device_path,
};
use crate::system_info::*;
use alloc::format;
use theseus_shared::handoff::{Handoff, HANDOFF};

/// Resolve kernel image base/size using UEFI LoadedImage information.
/// For a single-binary boot, we treat the bootloader image as the kernel image.
pub fn set_kernel_image_from_loaded_image() {
    // Do not touch UEFI protocols here; compute fields from our own entry symbol.
    theseus_shared::qemu_println!("Setting kernel image fields (single binary, no UEFI)");
    let entry_low = theseus_kernel::kernel_entry as usize as u64;
    let align_2mb: u64 = 2 * 1024 * 1024;
    let img_base = entry_low & !(align_2mb - 1);
    let aligned_size = 16 * 1024 * 1024; // 16 MiB span to cover code+data
    let offset_within_image = entry_low.saturating_sub(img_base);

    unsafe {
        HANDOFF.kernel_physical_base = img_base;
        HANDOFF.kernel_image_size = aligned_size;
        HANDOFF.kernel_virtual_base = 0xFFFFFFFF80000000;
        HANDOFF.kernel_virtual_entry = if offset_within_image < aligned_size {
            HANDOFF.kernel_virtual_base.wrapping_add(offset_within_image)
        } else {
            HANDOFF.kernel_virtual_base
        };
    }
    theseus_shared::qemu_println!("✓ Kernel image fields set (single binary)");
}

/// Initialize the UEFI environment and output driver
pub fn initialize_uefi_environment() -> Result<(), Status> {
    // Initialize UEFI logger
    uefi::helpers::init().unwrap();

    // Set handoff size
    unsafe {
        HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    }

    Ok(())
}

/// Collect graphics output protocol information
pub fn collect_graphics_info(verbose: bool) -> bool {
    write_line("Collecting graphics information...");

    let gop_info =
        match uefi::boot::locate_handle_buffer(SearchType::ByProtocol(&GraphicsOutput::GUID)) {
            Ok(gop_handles) => {
                if let Some(&handle) = gop_handles.first() {
                    if let Ok(mut gop) =
                        uefi::boot::open_protocol_exclusive::<GraphicsOutput>(handle)
                    {
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
                        Some((
                            res.0,
                            res.1,
                            pf,
                            mode.stride(),
                            fb.as_mut_ptr() as u64,
                            fb.size(),
                        ))
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            Err(_) => None,
        };

    // Report GOP status
    if let Some((w, h, pf, stride, fb_base, fb_size)) = gop_info {
        write_line("✓ Graphics Output Protocol (GOP) found and initialized");
        if verbose {
            display_gop_info(
                w as u32,
                h as u32,
                pf,
                stride as u32,
                fb_base,
                fb_size as u64,
            );
        }
        write_line("✓ Framebuffer information collected and stored in handoff structure");
        true
    } else {
        write_line("✗ Graphics Output Protocol (GOP) not available");
        write_line("  No framebuffer information will be available to kernel");
        false
    }
}

/// Collect memory map information
pub fn collect_memory_map(verbose: bool) -> Option<uefi::mem::memory_map::MemoryMapOwned> {
    write_line("Collecting memory map information...");

    match uefi::boot::memory_map(MemoryType::LOADER_DATA) {
        Ok(mmap) => {
            // Get memory map information using the correct UEFI 0.35 API
            let meta = mmap.meta();
            let descriptor_size = meta.desc_size as u32;
            let descriptor_version = meta.desc_version as u32;
            let entries_count = mmap.len() as u32;
            let total_size = meta.map_size as u32;

            // Store memory map information in handoff structure
            unsafe {
                HANDOFF.memory_map_buffer_ptr = mmap.buffer().as_ptr() as u64;
                HANDOFF.memory_map_descriptor_size = descriptor_size;
                HANDOFF.memory_map_descriptor_version = descriptor_version;
                HANDOFF.memory_map_entries = entries_count;
                HANDOFF.memory_map_size = total_size;
            }

            // Display the actual memory map entries (if verbose)
            if verbose {
                display_memory_map_entries(&mmap);
            }

            write_line("✓ Memory map collected successfully");
            if verbose {
                unsafe {
                    display_memory_map_info(
                        HANDOFF.memory_map_descriptor_size,
                        HANDOFF.memory_map_descriptor_version,
                        HANDOFF.memory_map_entries,
                        HANDOFF.memory_map_size,
                    );
                }
            }
            write_line("✓ Memory map information stored in handoff structure");

            Some(mmap)
        }
        Err(_) => {
            write_line("✗ Failed to collect memory map");
            None
        }
    }
}

/// Collect ACPI information
pub fn collect_acpi_info(verbose: bool) -> bool {
    write_line("Locating ACPI RSDP table...");
    if verbose {
        write_line("  Debug: About to call find_acpi_rsdp");
    }
    let rsdp_address = find_acpi_rsdp(verbose).unwrap_or(0);
    if verbose {
        write_line("  Debug: find_acpi_rsdp returned");
    }
    unsafe {
        HANDOFF.acpi_rsdp = rsdp_address;
    }

    if rsdp_address != 0 {
        write_line("✓ ACPI RSDP table found");
        if verbose {
            display_acpi_info(rsdp_address);
        }
        write_line("✓ ACPI information stored in handoff structure");
        true
    } else {
        write_line("✗ ACPI RSDP table not found");
        if verbose {
            display_acpi_info(rsdp_address);
        }
        write_line("  No ACPI support will be available to kernel");
        false
    }
}

/// Collect system information (firmware, boot time, etc.)
pub fn collect_system_info(verbose: bool) {
    // UEFI System Table and Image Handle
    write_line("Collecting UEFI system table and image handle...");
    match uefi::table::system_table_raw() {
        Some(system_table) => {
            let image_handle = uefi::boot::image_handle();
            unsafe {
                HANDOFF.uefi_system_table = system_table.as_ptr() as u64;
                HANDOFF.uefi_image_handle = image_handle.as_ptr() as u64;
            }
            write_line("✓ UEFI system table and image handle collected");
        }
        None => {
            write_line("✗ UEFI system table not available");
            unsafe {
                HANDOFF.uefi_system_table = 0;
                HANDOFF.uefi_image_handle = 0;
            }
        }
    }

    // Firmware Information
    write_line("Collecting firmware information...");
    match collect_firmware_info() {
        Some((vendor_ptr, vendor_len, revision)) => {
            unsafe {
                HANDOFF.firmware_vendor_ptr = vendor_ptr;
                HANDOFF.firmware_vendor_len = vendor_len;
                HANDOFF.firmware_revision = revision;
            }
            write_line("✓ Firmware information collected");
            if verbose {
                display_firmware_info(vendor_ptr, vendor_len, revision);
            }
        }
        None => {
            write_line("✗ Firmware information not available");
            if verbose {
                display_firmware_info(0, 0, 0);
            }
        }
    }

    // Boot Time Information
    write_line("Collecting boot time information...");
    match collect_boot_time_info() {
        Some((seconds, nanoseconds)) => {
            unsafe {
                HANDOFF.boot_time_seconds = seconds;
                HANDOFF.boot_time_nanoseconds = nanoseconds;
            }
            write_line("✓ Boot time information collected");
            if verbose {
                display_boot_time_info(seconds, nanoseconds);
            }
        }
        None => {
            write_line("✗ Boot time information not available");
            if verbose {
                display_boot_time_info(0, 0);
            }
        }
    }

    // Boot Device Path Information
    write_line("Collecting boot device path information...");
    match collect_boot_device_path() {
        Some((device_path_ptr, device_path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = device_path_ptr;
                HANDOFF.boot_device_path_size = device_path_size;
            }
            write_line("✓ Boot device path information collected");
            if verbose {
                display_boot_device_path_info(device_path_ptr, device_path_size);
            }
        }
        None => {
            write_line("✗ Boot device path information not available");
            if verbose {
                display_boot_device_path_info(0, 0);
            }
        }
    }

    // CPU Information
    write_line("Collecting CPU information...");
    match collect_cpu_info() {
        Some((cpu_count, cpu_features, microcode_revision)) => {
            unsafe {
                HANDOFF.cpu_count = cpu_count;
                HANDOFF.cpu_features = cpu_features;
                HANDOFF.microcode_revision = microcode_revision;
            }
            write_line("✓ CPU information collected");
            if verbose {
                display_cpu_info(cpu_count, cpu_features, microcode_revision);
            }
        }
        None => {
            write_line("✗ CPU information not available");
            if verbose {
                display_cpu_info(0, 0, 0);
            }
        }
    }
}

/// Collect hardware inventory
pub fn collect_hardware_inventory_info(verbose: bool) -> bool {
    write_line("Collecting hardware inventory...");
    match collect_hardware_inventory(verbose) {
        Some(inventory) => {
            unsafe {
                HANDOFF.hardware_device_count = inventory.device_count;
                HANDOFF.hardware_inventory_ptr = inventory.devices_ptr;
                HANDOFF.hardware_inventory_size = inventory.total_size;
            }
            write_line("✓ Hardware inventory collected");
            if verbose {
                display_hardware_inventory(&inventory);
            }
            true
        }
        None => {
            write_line("✗ Hardware inventory collection failed");
            false
        }
    }
}

/// Get loaded image device path
pub fn collect_loaded_image_path(verbose: bool) -> bool {
    write_line("Getting loaded image device path...");
    match get_loaded_image_device_path() {
        Some((path_ptr, path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = path_ptr;
                HANDOFF.boot_device_path_size = path_size;
            }
            write_line("✓ Loaded image device path collected");
            if verbose {
                display_boot_device_path_info(path_ptr, path_size);
            }
            true
        }
        None => {
            write_line("✗ Loaded image device path not available");
            false
        }
    }
}

/// Finalize the handoff structure
pub fn finalize_handoff_structure() {
    write_line("Finalizing handoff structure...");
    unsafe {
        // No fallback defaults here; kernel relies on accurate values
        HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    }
    write_line(&format!(
        "✓ Handoff structure size: {} bytes",
        core::mem::size_of::<Handoff>()
    ));
    write_line("✓ All system information collected and stored");
}

/// Load the kernel binary (simplified version)
///
/// This function allocates memory for the kernel and creates a placeholder binary.
///
/// # Arguments
///
/// * `memory_map` - The UEFI memory map
/// * `output_driver` - Output driver for logging
///
/// # Returns
///
/// * `Ok((physical_address, entry_point))` - Physical address and entry point where kernel was loaded
/// * `Err(status)` - Error loading kernel

/// Jump to the kernel entry point
///
/// This function performs the final handoff from bootloader to kernel.
/// It sets up the kernel environment and jumps to the kernel entry point.
///
/// # Safety
///
/// This function is unsafe because it:
/// - Jumps to arbitrary code (kernel entry point)
/// - Assumes the kernel is properly loaded at the expected address
/// - Performs operations that cannot be undone

/// Jump to the kernel entry point with persistent handoff structure
///
/// This function performs the final handoff from bootloader to kernel after
/// exiting boot services. It passes the persistent handoff structure address
/// to the kernel.
///
/// # Safety
///
/// This function is unsafe because it:
/// - Jumps to arbitrary code (kernel entry point)
/// - Assumes the kernel is properly loaded at the expected address
/// - Performs operations that cannot be undone
/// - Accesses memory after exit_boot_services
pub unsafe fn jump_to_kernel_with_handoff(_physical_entry_point: u64, handoff_ptr: *const Handoff) {
    write_line("Entering jump_to_kernel_with_handoff");
    // Copy handoff into a persistent LOADER_DATA buffer and pass its physical address
    let handoff_size = core::mem::size_of::<Handoff>() as u64;
    write_line(&format!(
        "Allocating persistent handoff buffer, size {} (0x{:x})",
        handoff_size, handoff_size
    ));
    let handoff_region = match crate::memory::allocate_memory(handoff_size, MemoryType::LOADER_DATA)
    {
        Ok(r) => r,
        Err(e) => {
            write_line(&format!("✗ Failed to allocate handoff buffer: {:?}", e));
            return;
        }
    };
    let handoff_phys = handoff_region.physical_address;
    write_line(&format!(
        "Allocated handoff buffer at phys 0x{:016x}",
        handoff_phys
    ));

    write_line(&format!(
        "Copying handoff from src {:p} to phys 0x{:016x}",
        handoff_ptr, handoff_phys
    ));
    core::ptr::copy_nonoverlapping(
        handoff_ptr as *const u8,
        handoff_phys as *mut u8,
        handoff_size as usize,
    );

    // Debug: read back size field from the freshly copied structure
    let copied_size = unsafe { *(handoff_phys as *const u32) };
    write_line(&format!(
        "Copied HANDOFF.size readback: {} (0x{:08x})",
        copied_size, copied_size
    ));

    // Exit boot services before transferring control to the kernel
    write_line("Exiting boot services before kernel handoff...");
    let _memory_map = unsafe { uefi::boot::exit_boot_services(None) };

    // Mark boot services exited in the copied handoff
    unsafe {
        (*(handoff_phys as *mut Handoff)).boot_services_exited = 1;
    }

    // Call kernel directly from the same binary
    let entry: extern "C" fn(u64) -> ! = theseus_kernel::kernel_entry;
    let arg = handoff_phys as u64;
    let _ = arg; // keep name for clarity
    entry(handoff_phys as u64)
}
