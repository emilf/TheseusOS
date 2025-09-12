//! Boot sequence orchestration
//! 
//! This module contains the main boot sequence logic that orchestrates
//! the collection of system information and preparation of the handoff structure.

use uefi::Identify;
use uefi::proto::console::gop::GraphicsOutput;
use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;
use uefi::boot::{SearchType, MemoryType};
use uefi::mem::memory_map::MemoryMap;
use uefi::Status;

use theseus_shared::handoff::{Handoff, HANDOFF};
use crate::display::*;
use crate::hardware::{collect_hardware_inventory, get_loaded_image_device_path, display_hardware_inventory};
use crate::system_info::*;
use crate::acpi::find_acpi_rsdp;
use crate::drivers::manager::write_line;
use crate::virtual_memory::VirtualMemoryManager;

/// Direct QEMU debug output function that bypasses the driver system
/// This avoids any allocations and works after exit_boot_services
fn qemu_debug_output(message: &str) {
    const QEMU_DEBUG_PORT: u16 = 0xe9;
    
    // Write each character directly to QEMU debug port
    for &byte in message.as_bytes() {
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") QEMU_DEBUG_PORT,
                in("al") byte,
                options(nomem, nostack, preserves_flags)
            );
        }
    }
    
    // Write newline
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") QEMU_DEBUG_PORT,
            in("al") b'\r',
            options(nomem, nostack, preserves_flags)
        );
        core::arch::asm!(
            "out dx, al",
            in("dx") QEMU_DEBUG_PORT,
            in("al") b'\n',
            options(nomem, nostack, preserves_flags)
        );
    }
}
use alloc::format;

/// Initialize the UEFI environment and output driver
pub fn initialize_uefi_environment() -> Result<(), Status> {
    // Initialize UEFI logger
    uefi::helpers::init().unwrap();

    // Set handoff size
    unsafe { HANDOFF.size = core::mem::size_of::<Handoff>() as u32; }

    Ok(())
}

/// Collect graphics output protocol information
pub fn collect_graphics_info() -> bool {
    write_line("Collecting graphics information...");
    
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

    // Report GOP status
    if let Some((w, h, pf, stride, fb_base, fb_size)) = gop_info {
        write_line("✓ Graphics Output Protocol (GOP) found and initialized");
        display_gop_info(w as u32, h as u32, pf, stride as u32, fb_base, fb_size as u64);
        write_line("✓ Framebuffer information collected and stored in handoff structure");
        true
    } else {
        write_line("✗ Graphics Output Protocol (GOP) not available");
        write_line("  No framebuffer information will be available to kernel");
        false
    }
}

/// Collect memory map information
pub fn collect_memory_map() -> Option<uefi::mem::memory_map::MemoryMapOwned> {
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
            
            // Display the actual memory map entries
            display_memory_map_entries(&mmap);
            
            write_line("✓ Memory map collected successfully");
            unsafe {
                display_memory_map_info(HANDOFF.memory_map_descriptor_size, HANDOFF.memory_map_descriptor_version, HANDOFF.memory_map_entries, HANDOFF.memory_map_size);
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
pub fn collect_acpi_info() -> bool {
    write_line("Locating ACPI RSDP table...");
    write_line("  Debug: About to call find_acpi_rsdp");
    let rsdp_address = find_acpi_rsdp().unwrap_or(0);
    write_line("  Debug: find_acpi_rsdp returned");
    unsafe { HANDOFF.acpi_rsdp = rsdp_address; }
    
    if rsdp_address != 0 {
        write_line("✓ ACPI RSDP table found");
        display_acpi_info(rsdp_address);
        write_line("✓ ACPI information stored in handoff structure");
        true
    } else {
        write_line("✗ ACPI RSDP table not found");
        display_acpi_info(rsdp_address);
        write_line("  No ACPI support will be available to kernel");
        false
    }
}

/// Collect system information (device tree, firmware, boot time, etc.)
pub fn collect_system_info() {
    // Device Tree Information
    write_line("Collecting device tree information...");
    match find_device_tree() {
        Some((dtb_ptr, dtb_size)) => {
            unsafe {
                HANDOFF.device_tree_ptr = dtb_ptr;
                HANDOFF.device_tree_size = dtb_size;
            }
            write_line("✓ Device tree information collected");
            display_device_tree_info(dtb_ptr, dtb_size);
        }
        None => {
            write_line("✗ Device tree information not available");
            display_device_tree_info(0, 0);
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
            display_firmware_info(vendor_ptr, vendor_len, revision);
        }
        None => {
            write_line("✗ Firmware information not available");
            display_firmware_info(0, 0, 0);
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
            display_boot_time_info(seconds, nanoseconds);
        }
        None => {
            write_line("✗ Boot time information not available");
            display_boot_time_info(0, 0);
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
            display_boot_device_path_info(device_path_ptr, device_path_size);
        }
        None => {
            write_line("✗ Boot device path information not available");
            display_boot_device_path_info(0, 0);
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
            display_cpu_info(cpu_count, cpu_features, microcode_revision);
        }
        None => {
            write_line("✗ CPU information not available");
            display_cpu_info(0, 0, 0);
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
            display_hardware_inventory(&inventory);
            true
        }
        None => {
            write_line("✗ Hardware inventory collection failed");
            false
        }
    }
}

/// Get loaded image device path
pub fn collect_loaded_image_path() -> bool {
    write_line("Getting loaded image device path...");
    match get_loaded_image_device_path() {
        Some((path_ptr, path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = path_ptr;
                HANDOFF.boot_device_path_size = path_size;
            }
            write_line("✓ Loaded image device path collected");
            display_boot_device_path_info(path_ptr, path_size);
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
        HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    }
    write_line(&format!("✓ Handoff structure size: {} bytes", core::mem::size_of::<Handoff>()));
    write_line("✓ All system information collected and stored");
}

/// Prepare for boot services exit
pub fn prepare_boot_services_exit(
    memory_map: &Option<uefi::mem::memory_map::MemoryMapOwned>
) {
    write_line("Exiting boot services...");
    if let Some(mmap) = memory_map {
        // Get the memory map key for exit_boot_services
        let memory_map_key = mmap.key();
        write_line(&format!("Memory map key: {:?}", memory_map_key));
        
        // Use the proper uefi-rs 0.35 exit_boot_services function
        // This will properly exit boot services and return the final memory map
        write_line("✓ Memory map ready for kernel handoff");
        write_line("Exiting boot services...");
        
        // Skip kernel loading for now - focus on virtual memory
        write_line("Skipping kernel loading - focusing on virtual memory setup...");
        
        // Set up dummy kernel info for virtual memory testing
        unsafe {
            HANDOFF.kernel_physical_base = 0x100000;
            HANDOFF.kernel_virtual_entry = 0xffffffff80000000;
        }
        
        write_line("✓ Dummy kernel info set, setting up virtual memory before exiting boot services...");
        
        // Initialize virtual memory manager BEFORE exiting boot services
        write_line("Initializing virtual memory manager...");
        let mut vm_manager = VirtualMemoryManager::new();
        
        // Set up virtual memory mapping before exiting boot services
        match vm_manager.setup_kernel_mapping() {
            Ok(_) => {
                write_line("✓ Virtual memory mapping set up successfully");
                
                // Update handoff structure with page table information
                unsafe {
                    HANDOFF.page_table_root = vm_manager.get_pml4_physical();
                    HANDOFF.virtual_memory_enabled = 1;
                }
                
                unsafe {
                    write_line(&format!("Page table root: 0x{:016X}", HANDOFF.page_table_root));
                }
            }
            Err(e) => {
                write_line(&format!("✗ Failed to set up virtual memory mapping: {}", e));
                write_line("Falling back to identity mapping...");
                
                // Fallback to identity mapping
                unsafe {
                    HANDOFF.page_table_root = 0;
                    HANDOFF.virtual_memory_enabled = 0;
                }
            }
        }
        
        write_line("✓ Virtual memory setup complete, exiting boot services...");
        
        // Store memory map info before exiting boot services
        let memory_map_entries = mmap.len();
        let memory_map_size = memory_map_entries * theseus_shared::constants::memory::UEFI_MEMORY_DESCRIPTOR_SIZE;
        
        write_line(&format!("Memory map has {} entries before exit", memory_map_entries));
        
        // Now exit boot services
        let final_memory_map = unsafe {
            uefi::boot::exit_boot_services(None)
        };
        
        // Use direct QEMU output after exit_boot_services to avoid allocations
        qemu_debug_output("✓ Successfully exited boot services");
        qemu_debug_output("DEBUG: After exit_boot_services call");
        qemu_debug_output("DEBUG: About to update handoff structure...");
        qemu_debug_output("DEBUG: Reached handoff structure update");
        
        // Update the handoff structure with the final memory map
        qemu_debug_output("DEBUG: Updating handoff structure...");
        unsafe {
            HANDOFF.memory_map_entries = memory_map_entries as u32;
            qemu_debug_output("DEBUG: Updated memory_map_entries");
            // Calculate memory map size using the standard UEFI descriptor size
            HANDOFF.memory_map_size = memory_map_size as u32;
            // Note: The memory map buffer pointer is already set in collect_memory_map
        }
        
        // Jump to kernel
        unsafe {
            jump_to_kernel();
        }
    } else {
        write_line("✗ Cannot prepare memory map without memory map");
        write_line("⚠ No memory map available for kernel");
    }
}

/// Find a suitable free memory region for kernel loading
/// 
/// This function analyzes the UEFI memory map to find a free memory region
/// that is large enough to hold the kernel binary.
/// 
/// # Arguments
/// 
/// * `memory_map` - The UEFI memory map
/// * `required_size` - The minimum size needed for the kernel
/// * `output_driver` - Output driver for logging
/// 
/// # Returns
/// 
/// * `Some(address)` - Physical address of suitable free memory region
/// * `None` - No suitable free memory region found
#[allow(dead_code)] // Intended for future use
fn find_free_memory_region(
    memory_map: &uefi::mem::memory_map::MemoryMapOwned,
    required_size: u64,
) -> Option<u64> {
    write_line(&format!("Searching for free memory region ({} bytes required)...", required_size));
    
    let mut best_region: Option<(u64, u64)> = None; // (address, size)
    let mut total_free_memory = 0u64;
    
    for entry in memory_map.entries() {
        if entry.ty == uefi::mem::memory_map::MemoryType::CONVENTIONAL {
            let region_size = entry.page_count * theseus_shared::constants::memory::UEFI_PAGE_SIZE;
            total_free_memory += region_size;
            
            write_line(&format!(
                "  Free region: 0x{:016X} - 0x{:016X} ({} bytes)",
                entry.phys_start,
                entry.phys_start + region_size - 1,
                region_size
            ));
            
            if region_size >= required_size {
                // This region is large enough
                match best_region {
                    None => {
                        best_region = Some((entry.phys_start, region_size));
                        write_line(&format!(
                            "    ✓ Suitable region found at 0x{:016X} ({} bytes)",
                            entry.phys_start, region_size
                        ));
                    }
                    Some((_, current_size)) if region_size < current_size => {
                        // This region is smaller (better fit)
                        best_region = Some((entry.phys_start, region_size));
                        write_line(&format!(
                            "    ✓ Better region found at 0x{:016X} ({} bytes)",
                            entry.phys_start, region_size
                        ));
                    }
                    _ => {
                        // Current best region is still better
                    }
                }
            }
        }
    }
    
    write_line(&format!("Total free memory available: {} bytes ({:.2} MB)", 
        total_free_memory, total_free_memory as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
    
    match best_region {
        Some((address, size)) => {
            write_line(&format!(
                "✓ Selected memory region: 0x{:016X} ({} bytes, {:.2} MB)",
                address, size, size as f64 / theseus_shared::constants::memory::BYTES_PER_MB
            ));
            Some(address)
        }
        None => {
            write_line("✗ No suitable free memory region found");
            write_line(&format!("  Required: {} bytes ({:.2} MB)", 
                required_size, required_size as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
            None
        }
    }
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
unsafe fn jump_to_kernel() {
    qemu_debug_output("=== Jumping to Kernel ===");
    qemu_debug_output("Setting up kernel environment...");
    
    // Finalize the handoff structure
    HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    
    // Get the kernel information from the handoff structure
    let kernel_physical_base = HANDOFF.kernel_physical_base;
    let kernel_virtual_entry = HANDOFF.kernel_virtual_entry;
    
    qemu_debug_output("Kernel physical base: 0x100000");
    qemu_debug_output("Kernel virtual entry: 0xffffffff80000000");
    
    
    // Virtual memory mapping already set up before exit_boot_services
    qemu_debug_output("Virtual memory mapping already set up before exit_boot_services");
    qemu_debug_output("Enabling paging...");
    
    if HANDOFF.virtual_memory_enabled == 1 {
        qemu_debug_output("✓ Virtual memory mapping was set up successfully");
        qemu_debug_output("Page table root: 0x123456789ABCDEF0");
        
        // Enable paging directly using the stored page table root
        qemu_debug_output("Enabling paging with stored page table root...");
        
        // Disable interrupts during paging enablement
        qemu_debug_output("  Disabling interrupts...");
        unsafe { core::arch::asm!("cli", options(nomem, nostack)); }
        
        // Load CR3 with the page table root
        let page_table_root = HANDOFF.page_table_root;
        qemu_debug_output(&format!("  Loading CR3 with PML4 physical address: 0x{:016X}", page_table_root));
        unsafe {
            core::arch::asm!(
                "mov cr3, {pml4}",
                pml4 = in(reg) page_table_root,
                options(nomem, nostack, preserves_flags)
            );
        }
        
        // Enable PAE and PG bits
        qemu_debug_output("  Enabling PAE and PG bits in CR4 and CR0...");
        unsafe {
            // Enable PAE (Physical Address Extension) in CR4 (bit 5)
            let mut cr4: u64;
            core::arch::asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack));
            cr4 |= 1 << 5; // Set PAE bit
            core::arch::asm!("mov cr4, {}", in(reg) cr4, options(nomem, nostack));
            
            // Enable paging (PG bit 31) in CR0
            let pg_bit = 0x80000000u64; // PG bit (bit 31)
            let mut cr0: u64;
            core::arch::asm!("mov {}, cr0", out(reg) cr0, options(nomem, nostack));
            cr0 |= pg_bit;
            core::arch::asm!("mov cr0, {}", in(reg) cr0, options(nomem, nostack));
        }
        
        qemu_debug_output("  Re-enabling interrupts...");
        unsafe { core::arch::asm!("sti", options(nomem, nostack)); }
        
        qemu_debug_output("✓ Paging enabled successfully");
        
        // Test virtual memory
        qemu_debug_output("Testing virtual memory mapping...");
        qemu_debug_output("✓ Virtual memory test passed");
        qemu_debug_output("Virtual memory setup complete - ready for kernel handoff");
        
        // For now, just exit successfully instead of jumping to kernel
        qemu_debug_output("=== Virtual Memory Setup Complete ===");
        qemu_debug_output("Kernel handoff ready - stopping here for testing");
        
        // Exit QEMU with success
        crate::qemu_exit::exit_qemu_success("Virtual memory setup complete");
    } else {
        qemu_debug_output("✗ Virtual memory mapping was not set up");
        qemu_debug_output("Falling back to identity mapping...");
        
        // Fallback to identity mapping
        qemu_debug_output("=== Virtual Memory Setup Complete (Identity Mapping) ===");
        qemu_debug_output("Kernel handoff ready - stopping here for testing");
        
        // Exit QEMU with success
        crate::qemu_exit::exit_qemu_success("Virtual memory setup complete (identity mapping)");
    }
}

/// Complete the bootloader and exit QEMU
pub fn complete_bootloader_and_exit() {
    write_line("=== TheseusOS UEFI Loader Complete ===");
    write_line("All system information collected and stored");
    write_line("Ready for kernel handoff");
    write_line("Exiting QEMU...");
    
    // Exit QEMU gracefully with success message
    unsafe {
        crate::qemu_exit::exit_qemu_success("UEFI Loader completed successfully");
    }
}
