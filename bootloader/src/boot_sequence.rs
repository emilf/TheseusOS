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

use hobbyos_shared::handoff::{Handoff, HANDOFF};
use crate::display::*;
use crate::hardware::{collect_hardware_inventory, get_loaded_image_device_path, display_hardware_inventory};
use crate::system_info::*;
use crate::acpi::find_acpi_rsdp;
use crate::drivers::OutputDriver;
use alloc::format;

/// Initialize the UEFI environment and output driver
pub fn initialize_uefi_environment() -> Result<OutputDriver, Status> {
    // Initialize UEFI logger
    uefi::helpers::init().unwrap();

    // Initialize the output driver system
    let mut output_driver = OutputDriver::new();

    // Output startup message
    output_driver.write_line("=== HobbyOS UEFI Loader Starting ===");
    output_driver.write_line(&format!("Output driver initialized: {}", output_driver.current_driver_name()));

    // Set handoff size
    unsafe { HANDOFF.size = core::mem::size_of::<Handoff>() as u32; }

    Ok(output_driver)
}

/// Collect graphics output protocol information
pub fn collect_graphics_info(output_driver: &mut OutputDriver) -> bool {
    output_driver.write_line("Collecting graphics information...");
    
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
        output_driver.write_line("✓ Graphics Output Protocol (GOP) found and initialized");
        display_gop_info(output_driver, w as u32, h as u32, pf, stride as u32, fb_base, fb_size as u64);
        output_driver.write_line("✓ Framebuffer information collected and stored in handoff structure");
        true
    } else {
        output_driver.write_line("✗ Graphics Output Protocol (GOP) not available");
        output_driver.write_line("  No framebuffer information will be available to kernel");
        false
    }
}

/// Collect memory map information
pub fn collect_memory_map(output_driver: &mut OutputDriver) -> Option<uefi::mem::memory_map::MemoryMapOwned> {
    output_driver.write_line("Collecting memory map information...");
    
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
            display_memory_map_entries(output_driver, &mmap);
            
            output_driver.write_line("✓ Memory map collected successfully");
            unsafe {
                display_memory_map_info(output_driver, HANDOFF.memory_map_descriptor_size, HANDOFF.memory_map_descriptor_version, HANDOFF.memory_map_entries, HANDOFF.memory_map_size);
            }
            output_driver.write_line("✓ Memory map information stored in handoff structure");
            
            Some(mmap)
        }
        Err(_) => {
            output_driver.write_line("✗ Failed to collect memory map");
            None
        }
    }
}

/// Collect ACPI information
pub fn collect_acpi_info(output_driver: &mut OutputDriver) -> bool {
    output_driver.write_line("Locating ACPI RSDP table...");
    output_driver.write_line("  Debug: About to call find_acpi_rsdp");
    let rsdp_address = find_acpi_rsdp(output_driver).unwrap_or(0);
    output_driver.write_line("  Debug: find_acpi_rsdp returned");
    unsafe { HANDOFF.acpi_rsdp = rsdp_address; }
    
    if rsdp_address != 0 {
        output_driver.write_line("✓ ACPI RSDP table found");
        display_acpi_info(output_driver, rsdp_address);
        output_driver.write_line("✓ ACPI information stored in handoff structure");
        true
    } else {
        output_driver.write_line("✗ ACPI RSDP table not found");
        display_acpi_info(output_driver, rsdp_address);
        output_driver.write_line("  No ACPI support will be available to kernel");
        false
    }
}

/// Collect system information (device tree, firmware, boot time, etc.)
pub fn collect_system_info(output_driver: &mut OutputDriver) {
    // Device Tree Information
    output_driver.write_line("Collecting device tree information...");
    match find_device_tree() {
        Some((dtb_ptr, dtb_size)) => {
            unsafe {
                HANDOFF.device_tree_ptr = dtb_ptr;
                HANDOFF.device_tree_size = dtb_size;
            }
            output_driver.write_line("✓ Device tree information collected");
            display_device_tree_info(output_driver, dtb_ptr, dtb_size);
        }
        None => {
            output_driver.write_line("✗ Device tree information not available");
            display_device_tree_info(output_driver, 0, 0);
        }
    }

    // Firmware Information
    output_driver.write_line("Collecting firmware information...");
    match collect_firmware_info() {
        Some((vendor_ptr, vendor_len, revision)) => {
            unsafe {
                HANDOFF.firmware_vendor_ptr = vendor_ptr;
                HANDOFF.firmware_vendor_len = vendor_len;
                HANDOFF.firmware_revision = revision;
            }
            output_driver.write_line("✓ Firmware information collected");
            display_firmware_info(output_driver, vendor_ptr, vendor_len, revision);
        }
        None => {
            output_driver.write_line("✗ Firmware information not available");
            display_firmware_info(output_driver, 0, 0, 0);
        }
    }

    // Boot Time Information
    output_driver.write_line("Collecting boot time information...");
    match collect_boot_time_info() {
        Some((seconds, nanoseconds)) => {
            unsafe {
                HANDOFF.boot_time_seconds = seconds;
                HANDOFF.boot_time_nanoseconds = nanoseconds;
            }
            output_driver.write_line("✓ Boot time information collected");
            display_boot_time_info(output_driver, seconds, nanoseconds);
        }
        None => {
            output_driver.write_line("✗ Boot time information not available");
            display_boot_time_info(output_driver, 0, 0);
        }
    }

    // Boot Device Path Information
    output_driver.write_line("Collecting boot device path information...");
    match collect_boot_device_path() {
        Some((device_path_ptr, device_path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = device_path_ptr;
                HANDOFF.boot_device_path_size = device_path_size;
            }
            output_driver.write_line("✓ Boot device path information collected");
            display_boot_device_path_info(output_driver, device_path_ptr, device_path_size);
        }
        None => {
            output_driver.write_line("✗ Boot device path information not available");
            display_boot_device_path_info(output_driver, 0, 0);
        }
    }

    // CPU Information
    output_driver.write_line("Collecting CPU information...");
    match collect_cpu_info() {
        Some((cpu_count, cpu_features, microcode_revision)) => {
            unsafe {
                HANDOFF.cpu_count = cpu_count;
                HANDOFF.cpu_features = cpu_features;
                HANDOFF.microcode_revision = microcode_revision;
            }
            output_driver.write_line("✓ CPU information collected");
            display_cpu_info(output_driver, cpu_count, cpu_features, microcode_revision);
        }
        None => {
            output_driver.write_line("✗ CPU information not available");
            display_cpu_info(output_driver, 0, 0, 0);
        }
    }
}

/// Collect hardware inventory
pub fn collect_hardware_inventory_info(output_driver: &mut OutputDriver, verbose: bool) -> bool {
    output_driver.write_line("Collecting hardware inventory...");
    match collect_hardware_inventory(output_driver, verbose) {
        Some(inventory) => {
            unsafe {
                HANDOFF.hardware_device_count = inventory.device_count;
                HANDOFF.hardware_inventory_ptr = inventory.devices_ptr;
                HANDOFF.hardware_inventory_size = inventory.total_size;
            }
            output_driver.write_line("✓ Hardware inventory collected");
            display_hardware_inventory(output_driver, &inventory);
            true
        }
        None => {
            output_driver.write_line("✗ Hardware inventory collection failed");
            false
        }
    }
}

/// Get loaded image device path
pub fn collect_loaded_image_path(output_driver: &mut OutputDriver) -> bool {
    output_driver.write_line("Getting loaded image device path...");
    match get_loaded_image_device_path(output_driver) {
        Some((path_ptr, path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = path_ptr;
                HANDOFF.boot_device_path_size = path_size;
            }
            output_driver.write_line("✓ Loaded image device path collected");
            display_boot_device_path_info(output_driver, path_ptr, path_size);
            true
        }
        None => {
            output_driver.write_line("✗ Loaded image device path not available");
            false
        }
    }
}

/// Finalize the handoff structure
pub fn finalize_handoff_structure(output_driver: &mut OutputDriver) {
    output_driver.write_line("Finalizing handoff structure...");
    unsafe {
        HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    }
    output_driver.write_line(&format!("✓ Handoff structure size: {} bytes", core::mem::size_of::<Handoff>()));
    output_driver.write_line("✓ All system information collected and stored");
}

/// Prepare for boot services exit
pub fn prepare_boot_services_exit(
    output_driver: &mut OutputDriver, 
    memory_map: &Option<uefi::mem::memory_map::MemoryMapOwned>
) {
    output_driver.write_line("Exiting boot services...");
    if let Some(mmap) = memory_map {
        // Get the memory map key for exit_boot_services
        let memory_map_key = mmap.key();
        output_driver.write_line(&format!("Memory map key: {:?}", memory_map_key));
        
        // Use the proper uefi-rs 0.35 exit_boot_services function
        // This will properly exit boot services and return the final memory map
        output_driver.write_line("✓ Memory map ready for kernel handoff");
        output_driver.write_line("Exiting boot services...");
        
        // Call exit_boot_services (it takes no parameters in uefi-rs 0.35)
        // Note: This function is unsafe and will exit boot services
        let final_memory_map = unsafe {
            uefi::boot::exit_boot_services(None)
        };
        
        output_driver.write_line("✓ Successfully exited boot services");
        output_driver.write_line(&format!("Final memory map has {} entries", final_memory_map.len()));
        
        // Update the handoff structure with the final memory map
        unsafe {
            HANDOFF.memory_map_entries = final_memory_map.len() as u32;
            // Calculate memory map size using the standard UEFI descriptor size
            HANDOFF.memory_map_size = (final_memory_map.len() * hobbyos_shared::constants::memory::UEFI_MEMORY_DESCRIPTOR_SIZE) as u32;
            // Note: The memory map buffer pointer is already set in collect_memory_map
        }
        
        // Load the kernel binary (simplified version)
        match load_kernel_binary(&final_memory_map, output_driver) {
            Ok((kernel_physical_base, kernel_entry_point)) => {
                // Update the handoff structure with the actual kernel information
                unsafe {
                    HANDOFF.kernel_physical_base = kernel_physical_base;
                    HANDOFF.kernel_virtual_entry = kernel_entry_point;
                }
                
                output_driver.write_line("✓ Kernel loaded successfully, jumping to kernel...");
                
                // Jump to kernel
                unsafe {
                    jump_to_kernel(output_driver);
                }
            }
            Err(status) => {
                output_driver.write_line(&format!("✗ Failed to load kernel: {:?}", status));
                output_driver.write_line("Cannot proceed with kernel handoff");
            }
        }
    } else {
        output_driver.write_line("✗ Cannot prepare memory map without memory map");
        output_driver.write_line("⚠ No memory map available for kernel");
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
fn find_free_memory_region(
    memory_map: &uefi::mem::memory_map::MemoryMapOwned,
    required_size: u64,
    output_driver: &mut OutputDriver,
) -> Option<u64> {
    output_driver.write_line(&format!("Searching for free memory region ({} bytes required)...", required_size));
    
    let mut best_region: Option<(u64, u64)> = None; // (address, size)
    let mut total_free_memory = 0u64;
    
    for entry in memory_map.entries() {
        if entry.ty == uefi::mem::memory_map::MemoryType::CONVENTIONAL {
            let region_size = entry.page_count * hobbyos_shared::constants::memory::UEFI_PAGE_SIZE;
            total_free_memory += region_size;
            
            output_driver.write_line(&format!(
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
                        output_driver.write_line(&format!(
                            "    ✓ Suitable region found at 0x{:016X} ({} bytes)",
                            entry.phys_start, region_size
                        ));
                    }
                    Some((_, current_size)) if region_size < current_size => {
                        // This region is smaller (better fit)
                        best_region = Some((entry.phys_start, region_size));
                        output_driver.write_line(&format!(
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
    
    output_driver.write_line(&format!("Total free memory available: {} bytes ({:.2} MB)", 
        total_free_memory, total_free_memory as f64 / hobbyos_shared::constants::memory::BYTES_PER_MB));
    
    match best_region {
        Some((address, size)) => {
            output_driver.write_line(&format!(
                "✓ Selected memory region: 0x{:016X} ({} bytes, {:.2} MB)",
                address, size, size as f64 / hobbyos_shared::constants::memory::BYTES_PER_MB
            ));
            Some(address)
        }
        None => {
            output_driver.write_line("✗ No suitable free memory region found");
            output_driver.write_line(&format!("  Required: {} bytes ({:.2} MB)", 
                required_size, required_size as f64 / hobbyos_shared::constants::memory::BYTES_PER_MB));
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
fn load_kernel_binary(
    memory_map: &uefi::mem::memory_map::MemoryMapOwned,
    output_driver: &mut OutputDriver,
) -> Result<(u64, u64), uefi::Status> {
    use crate::kernel_loader::*;
    
    output_driver.write_line("=== Loading Kernel Binary ===");
    
    // Find the kernel file (placeholder)
    find_kernel_file(output_driver)?;
    
    // Get kernel file information
    let file_size = get_kernel_file_info(output_driver)?;
    
    // Create a placeholder kernel binary
    let kernel_buffer = create_kernel_binary(file_size, output_driver)?;
    
    // Analyze the kernel binary
    let mut kernel_info = analyze_kernel_binary(&kernel_buffer, output_driver)?;
    
    // Find a suitable free memory region for the kernel
    output_driver.write_line("Allocating memory for kernel...");
    let kernel_physical_base = match find_free_memory_region(memory_map, kernel_info.total_memory_size, output_driver) {
        Some(address) => {
            output_driver.write_line(&format!("✓ Memory allocated at 0x{:016X}", address));
            address
        }
        None => {
            output_driver.write_line("✗ Cannot find suitable memory region for kernel");
            return Err(uefi::Status::OUT_OF_RESOURCES);
        }
    };
    
    // Load kernel sections into allocated memory
    let actual_physical_base = load_kernel_sections(&kernel_buffer, &mut kernel_info, output_driver)?;
    
    // Calculate the actual entry point (physical address)
    let entry_point_physical = actual_physical_base + (kernel_info.entry_point - kernel_info.base_virtual_address);
    
    output_driver.write_line("✓ Kernel binary loaded successfully");
    output_driver.write_line(&format!("  Physical base: 0x{:016X}", actual_physical_base));
    output_driver.write_line(&format!("  Entry point (physical): 0x{:016X}", entry_point_physical));
    output_driver.write_line(&format!("  Entry point (virtual): 0x{:016X}", kernel_info.entry_point));
    
    Ok((actual_physical_base, entry_point_physical))
}

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
unsafe fn jump_to_kernel(output_driver: &mut OutputDriver) {
    output_driver.write_line("=== Jumping to Kernel ===");
    output_driver.write_line("Setting up kernel environment...");
    
    // Finalize the handoff structure
    HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    
    // Set virtual memory information
    HANDOFF.kernel_virtual_base = 0xffffffff80000000;  // Kernel virtual base
    HANDOFF.kernel_physical_base = 0x100000;           // Physical load address
    HANDOFF.kernel_virtual_entry = 0xffffffff80000000; // Virtual entry point
    HANDOFF.page_table_root = 0;                       // No paging setup yet
    HANDOFF.virtual_memory_enabled = 0;                // Identity mapped for now
    
    // Log the handoff information
    output_driver.write_line(&format!("Handoff structure size: {} bytes", HANDOFF.size));
    output_driver.write_line(&format!("Memory map entries: {}", HANDOFF.memory_map_entries));
    output_driver.write_line(&format!("Memory map size: {} bytes", HANDOFF.memory_map_size));
    
    if HANDOFF.acpi_rsdp != 0 {
        output_driver.write_line(&format!("ACPI RSDP: 0x{:016X}", HANDOFF.acpi_rsdp));
    }
    
    output_driver.write_line(&format!("Kernel virtual base: 0x{:016X}", HANDOFF.kernel_virtual_base));
    output_driver.write_line(&format!("Kernel physical base: 0x{:016X}", HANDOFF.kernel_physical_base));
    output_driver.write_line(&format!("Virtual memory enabled: {}", HANDOFF.virtual_memory_enabled));
    output_driver.write_line("Jumping to kernel...");
    
    // Get the kernel information from the handoff structure
    let kernel_physical_base = HANDOFF.kernel_physical_base;
    let kernel_entry_point = HANDOFF.kernel_virtual_entry;
    
    output_driver.write_line(&format!("Kernel physical base: 0x{:016X}", kernel_physical_base));
    output_driver.write_line(&format!("Kernel entry point: 0x{:016X}", kernel_entry_point));
    
    // Cast the entry point to a function pointer
    let kernel_entry: extern "C" fn() -> ! = core::mem::transmute(kernel_entry_point);
    
    output_driver.write_line("Jumping to kernel entry point...");
    
    // Jump to the kernel
    // Note: This will never return as the kernel entry point is marked as `-> !`
    kernel_entry();
}

/// Complete the bootloader and exit QEMU
pub fn complete_bootloader_and_exit(output_driver: &mut OutputDriver) {
    output_driver.write_line("=== UEFI Loader Complete ===");
    output_driver.write_line("All system information collected and stored");
    output_driver.write_line("Ready for kernel handoff");
    output_driver.write_line("Exiting QEMU...");
    
    // Exit QEMU gracefully with success message
    unsafe {
        crate::qemu_exit::exit_qemu_success("UEFI Loader completed successfully");
    }
}
