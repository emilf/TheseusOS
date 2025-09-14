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
use alloc::format;

/// Initialize the UEFI environment and output driver
pub fn initialize_uefi_environment() -> Result<(), Status> {
    // Initialize UEFI logger
    uefi::helpers::init().unwrap();

    // Set handoff size
    unsafe { HANDOFF.size = core::mem::size_of::<Handoff>() as u32; }

    Ok(())
}

/// Test only identity mapping to verify basic virtual memory functionality
/// This function tests that identity mapping works correctly after set_virtual_address_map
unsafe fn test_identity_mapping_only(handoff_physical_addr: u64) {
    // Use QEMU debug port for output (no heap allocation needed)
    let debug_port: u16 = 0xe9;
    
    // Output test start message
    output_debug_string("=== Identity Mapping Test ===\n", debug_port);
    
    // Test data to write
    let test_data: [u8; 8] = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE];
    
    // Use a safe memory location for identity mapping test
    // We'll use a location in the handoff structure that we know is safe
    let identity_test_addr = handoff_physical_addr + 0x200; // 512 bytes into handoff
    
    // Write to physical address
    let identity_phys_ptr = identity_test_addr as *mut u8;
    for i in 0..8 {
        *identity_phys_ptr.add(i) = test_data[i];
    }
    
    // Read back from same physical address (identity mapping)
    let identity_read_ptr = identity_test_addr as *const u8;
    let mut success = true;
    for i in 0..8 {
        if *identity_read_ptr.add(i) != test_data[i] {
            success = false;
            break;
        }
    }
    
    if success {
        output_debug_string("  ✓ Identity mapping works after set_virtual_address_map\n", debug_port);
    } else {
        output_debug_string("  ✗ Identity mapping failed after set_virtual_address_map\n", debug_port);
    }
    
    // Output test completion message
    output_debug_string("=== Identity Mapping Test Complete ===\n", debug_port);
    output_debug_string("Exiting QEMU...\n", debug_port);
    
    // Exit QEMU using the QEMU exit device
    exit_qemu(0);
}

/// Test virtual memory mapping by writing to high virtual addresses and reading from low physical addresses
/// This function tests that our virtual memory mapping is working correctly
/// It writes test data to high virtual memory and verifies it appears in low physical memory
unsafe fn test_virtual_memory_mapping(kernel_physical_base: u64, handoff_physical_addr: u64) {
    // Use QEMU debug port for output (no heap allocation needed)
    let debug_port: u16 = 0xe9;
    
    // Output test start message
    output_debug_string("=== Virtual Memory Mapping Test ===\n", debug_port);
    
    // Test data to write
    let test_data: [u8; 8] = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE];
    
    // Test 1: Simple identity mapping test first
    output_debug_string("Test 1: Identity mapping test\n", debug_port);
    
    // Use a safe memory location for identity mapping test
    // We'll use a location in the handoff structure that we know is safe
    let identity_test_addr = handoff_physical_addr + 0x200; // 512 bytes into handoff
    
    // Write to physical address
    let identity_phys_ptr = identity_test_addr as *mut u8;
    for i in 0..8 {
        *identity_phys_ptr.add(i) = test_data[i];
    }
    
    // Read back from same physical address (identity mapping)
    let identity_read_ptr = identity_test_addr as *const u8;
    let mut success = true;
    for i in 0..8 {
        if *identity_read_ptr.add(i) != test_data[i] {
            success = false;
            break;
        }
    }
    
    if success {
        output_debug_string("  ✓ Identity mapping works\n", debug_port);
    } else {
        output_debug_string("  ✗ Identity mapping failed\n", debug_port);
    }
    
    // Test 2: Kernel virtual memory mapping (more conservative approach)
    output_debug_string("Test 2: Kernel virtual memory mapping\n", debug_port);
    
    // Kernel virtual address: 0xffffffff80000000
    let kernel_virtual_addr = 0xffffffff80000000u64;
    let kernel_test_offset = 0x1000; // 4KB offset into kernel memory
    let kernel_test_virt_addr = kernel_virtual_addr + kernel_test_offset;
    let kernel_test_phys_addr = kernel_physical_base + kernel_test_offset;
    
    // Try to write a single byte first to test if the mapping works
    let kernel_virt_ptr = kernel_test_virt_addr as *mut u8;
    *kernel_virt_ptr = 0x42; // Write a test byte
    
    // Read back from low physical address
    let kernel_phys_ptr = kernel_test_phys_addr as *const u8;
    if *kernel_phys_ptr == 0x42 {
        output_debug_string("  ✓ Kernel virtual->physical mapping works\n", debug_port);
    } else {
        output_debug_string("  ✗ Kernel virtual->physical mapping failed\n", debug_port);
    }
    
    // Test 3: Handoff virtual memory mapping (more conservative approach)
    output_debug_string("Test 3: Handoff virtual memory mapping\n", debug_port);
    
    // Handoff virtual address: 0xffffffff80010000
    let handoff_virtual_addr = 0xffffffff80010000u64;
    let handoff_test_offset = 0x100; // 256 byte offset into handoff memory
    let handoff_test_virt_addr = handoff_virtual_addr + handoff_test_offset;
    let handoff_test_phys_addr = handoff_physical_addr + handoff_test_offset;
    
    // Try to write a single byte first to test if the mapping works
    let handoff_virt_ptr = handoff_test_virt_addr as *mut u8;
    *handoff_virt_ptr = 0x84; // Write a test byte
    
    // Read back from low physical address
    let handoff_phys_ptr = handoff_test_phys_addr as *const u8;
    if *handoff_phys_ptr == 0x84 {
        output_debug_string("  ✓ Handoff virtual->physical mapping works\n", debug_port);
    } else {
        output_debug_string("  ✗ Handoff virtual->physical mapping failed\n", debug_port);
    }
    
    // Output test completion message
    output_debug_string("=== Virtual Memory Mapping Test Complete ===\n", debug_port);
    output_debug_string("Exiting QEMU...\n", debug_port);
    
    // Exit QEMU using the QEMU exit device
    exit_qemu(0);
}

/// Output a string to the QEMU debug port (0xe9)
/// This function doesn't use heap allocation and is safe to call after boot services exit
unsafe fn output_debug_string(s: &str, port: u16) {
    for byte in s.bytes() {
        core::arch::asm!(
            "out dx, al",
            in("dx") port,
            in("al") byte,
            options(nostack, preserves_flags)
        );
    }
}

/// Exit QEMU using the QEMU exit device
unsafe fn exit_qemu(exit_code: u32) {
    const QEMU_EXIT_PORT: u16 = 0xf4;
    core::arch::asm!(
        "out dx, eax",
        in("dx") QEMU_EXIT_PORT,
        in("eax") exit_code,
        options(nostack, preserves_flags)
    );
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

/// Copy handoff structure to persistent memory
/// 
/// This function allocates persistent memory and copies the current handoff structure
/// there so it remains accessible after exit_boot_services.
/// 
/// # Returns
/// 
/// * `Ok(u64)` - Physical address of the persistent handoff structure
/// * `Err(Status)` - Error if allocation or copying failed
fn copy_handoff_to_persistent_memory() -> Result<u64, Status> {
    use crate::memory::allocate_persistent_memory;
    
    write_line("Copying handoff structure to persistent memory...");
    
    // Allocate persistent memory for the handoff structure
    let handoff_size = core::mem::size_of::<Handoff>() as u64;
    let persistent_region = allocate_persistent_memory(handoff_size)?;
    
    // Copy the current handoff structure to the persistent memory
    unsafe {
        let src = &raw const HANDOFF as *const Handoff;
        let dst = persistent_region.physical_address as *mut Handoff;
        
        core::ptr::copy_nonoverlapping(src, dst, 1);
        
        write_line(&format!("✓ Handoff structure copied to persistent memory at 0x{:016X}", persistent_region.physical_address));
    }
    
    Ok(persistent_region.physical_address)
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
        
                // Load the kernel binary BEFORE exiting boot services
                // (we need UEFI file system protocols to read the kernel file)
                write_line("About to call load_kernel_binary...");
                match crate::kernel_loader::load_kernel_binary(mmap) {
                    Ok((kernel_physical_base, kernel_entry_point)) => {
                        // Update the handoff structure with the actual kernel information
                        unsafe {
                            HANDOFF.kernel_physical_base = kernel_physical_base;
                            HANDOFF.kernel_virtual_entry = kernel_entry_point;
                        }
                        
                        write_line("✓ Kernel loaded successfully, preparing for boot services exit...");
                        
                        // Update handoff structure with kernel information
                        unsafe {
                            HANDOFF.kernel_physical_base = kernel_physical_base;
                            HANDOFF.kernel_virtual_entry = kernel_entry_point;
                            HANDOFF.kernel_virtual_base = 0xffffffff80000000; // Virtual base
                            HANDOFF.page_table_root = 0; // No paging yet
                            HANDOFF.virtual_memory_enabled = 0; // Identity mapped
                            
                            // Ensure handoff structure is properly initialized
                            HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
                            write_line(&format!("Handoff structure size set to: {} bytes", core::mem::size_of::<Handoff>()));
                        }
                        
                        // Copy handoff structure to persistent memory BEFORE exiting boot services
                        let persistent_handoff_addr = match copy_handoff_to_persistent_memory() {
                            Ok(addr) => addr,
                            Err(status) => {
                                write_line(&format!("✗ Failed to copy handoff structure to persistent memory: {:?}", status));
                                return;
                            }
                        };
                        
                        // Prepare everything we need before exiting boot services
                        write_line("Preparing virtual memory mapping...");
                        let virtual_memory_result = prepare_virtual_memory_mapping(mmap, kernel_physical_base, kernel_entry_point, persistent_handoff_addr);
                        
                        write_line("✓ All preparations complete, exiting boot services...");
                        
                        // Now exit boot services
                        let final_memory_map = unsafe {
                            uefi::boot::exit_boot_services(None)
                        };
                        
                        // Note: We can't use write_line after exit_boot_services due to heap allocation issues
                        
                        // Update the persistent handoff structure with the final memory map
                        unsafe {
                            let persistent_handoff = persistent_handoff_addr as *mut Handoff;
                            (*persistent_handoff).memory_map_entries = final_memory_map.len() as u32;
                            // Calculate memory map size using the standard UEFI descriptor size
                            (*persistent_handoff).memory_map_size = (final_memory_map.len() * theseus_shared::constants::memory::UEFI_MEMORY_DESCRIPTOR_SIZE) as u32;
                            // Note: The memory map buffer pointer is already set in collect_memory_map
                        }
                        
                        // Update virtual memory status in persistent handoff
                        let virtual_memory_enabled = match virtual_memory_result {
                            Ok(_) => 1, // Virtual memory mapping was prepared
                            Err(_) => 0, // Virtual memory mapping failed, using identity mapping
                        };
                        
                        unsafe {
                            let persistent_handoff = persistent_handoff_addr as *mut Handoff;
                            (*persistent_handoff).virtual_memory_enabled = virtual_memory_enabled;
                        }
                        
                        // Set up virtual memory mapping using UEFI Runtime Services
                        // This must be done after exit_boot_services and without heap allocation
                        match virtual_memory_result {
                            Ok((descriptors, count)) => {
                                match setup_virtual_memory_mapping_post_exit(&final_memory_map, descriptors, count) {
                                    Ok(()) => {
                                        // Virtual memory mapping established successfully
                                        // Test virtual memory mapping by accessing high memory addresses
                                        unsafe {
                                            test_identity_mapping_only(persistent_handoff_addr);
                                        }
                                    }
                                    Err(_) => {
                                        // Virtual memory mapping failed, but we can continue with physical addressing
                                        // This is not fatal - the kernel can still run with identity mapping
                                    }
                                }
                            }
                            Err(_) => {
                                // Virtual memory preparation failed, continuing with physical addressing
                            }
                        }
                        
                        // Jump to kernel with persistent handoff structure address
                        // Use virtual addresses if virtual memory mapping was successful
                        let (kernel_entry_addr, handoff_virt_addr) = match virtual_memory_result {
                            Ok(_) => {
                                // Virtual memory mapping was prepared, use virtual addresses
                                (kernel_entry_point, 0xffffffff80010000u64) // Handoff structure virtual address
                            }
                            Err(_) => {
                                // Virtual memory mapping failed, use physical addresses
                                (kernel_physical_base, persistent_handoff_addr)
                            }
                        };
                        
                        unsafe {
                            jump_to_kernel_with_persistent_handoff(kernel_entry_addr, handoff_virt_addr);
                        }
            }
            Err(status) => {
                write_line(&format!("✗ Failed to load kernel: {:?}", status));
                write_line("Cannot proceed with kernel handoff");
            }
        }
    } else {
        write_line("✗ Cannot prepare memory map without memory map");
        write_line("⚠ No memory map available for kernel");
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
unsafe fn jump_to_kernel_with_persistent_handoff(physical_entry_point: u64, persistent_handoff_addr: u64) {
    // Note: We can't use write_line after exit_boot_services due to heap allocation issues
    // Direct UEFI output would be needed here, but for now we'll proceed silently
    
    // Cast the physical entry point to a function pointer that takes handoff address
    // The kernel entry point should accept the handoff structure address as a parameter
    let kernel_entry: extern "C" fn(handoff_addr: u64) -> ! = core::mem::transmute(physical_entry_point);
    
    // Jump to the kernel with the persistent handoff structure address
    // Note: This will never return as the kernel entry point is marked as `-> !`
    kernel_entry(persistent_handoff_addr);
}

/// Prepare virtual memory mapping before exiting boot services
/// This function prepares all the data structures needed for virtual memory mapping
/// without actually calling set_virtual_address_map (which must be done after exit_boot_services)
fn prepare_virtual_memory_mapping(
    memory_map: &uefi::mem::memory_map::MemoryMapOwned,
    kernel_physical_base: u64,
    kernel_virtual_base: u64,
    handoff_physical_addr: u64,
) -> Result<([uefi::mem::memory_map::MemoryDescriptor; 200], usize), uefi::Status> {
    use uefi::mem::memory_map::{MemoryDescriptor, MemoryType, MemoryAttribute};
    
    // Create a fixed-size array for memory descriptors
    let mut descriptors: [MemoryDescriptor; 200] = unsafe { core::mem::zeroed() };
    let mut descriptor_count = 0;
    
    // Convert memory map entries to descriptors
    for entry in memory_map.entries() {
        if descriptor_count >= 200 {
            break;
        }
        
        descriptors[descriptor_count] = MemoryDescriptor {
            ty: entry.ty,
            phys_start: entry.phys_start,
            virt_start: if entry.ty == MemoryType::CONVENTIONAL {
                // Map conventional memory 1:1 (identity mapping)
                entry.phys_start
            } else {
                // Keep other memory types at their physical addresses
                entry.phys_start
            },
            page_count: entry.page_count,
            att: entry.att,
        };
        descriptor_count += 1;
    }
    
    // For now, only test identity mapping
    // TODO: Add kernel and handoff high memory mappings once identity mapping is confirmed working
    // 
    // // Add kernel mapping to high virtual memory
    // if descriptor_count < 200 {
    //     descriptors[descriptor_count] = MemoryDescriptor {
    //         ty: MemoryType::LOADER_DATA,
    //         phys_start: kernel_physical_base,
    //         virt_start: kernel_virtual_base,
    //         page_count: 32, // 32 pages = 128KB for kernel
    //         att: MemoryAttribute::from_bits_truncate(0x0000000f),
    //     };
    //     descriptor_count += 1;
    // }
    // 
    // // Add handoff structure mapping to high virtual memory
    // if descriptor_count < 200 {
    //     descriptors[descriptor_count] = MemoryDescriptor {
    //         ty: MemoryType::LOADER_DATA,
    //         phys_start: handoff_physical_addr,
    //         virt_start: 0xffffffff80010000u64, // Handoff structure at high virtual address
    //         page_count: 1, // 1 page = 4KB for handoff structure
    //         att: MemoryAttribute::from_bits_truncate(0x0000000f),
    //     };
    //     descriptor_count += 1;
    // }
    
    Ok((descriptors, descriptor_count))
}

/// Set up virtual memory mapping after exiting boot services
/// This function calls set_virtual_address_map without using heap allocations
fn setup_virtual_memory_mapping_post_exit(
    _memory_map: &uefi::mem::memory_map::MemoryMapOwned,
    mut descriptors: [uefi::mem::memory_map::MemoryDescriptor; 200],
    descriptor_count: usize,
) -> Result<(), uefi::Status> {
    use uefi::runtime;
    
    // Get the system table to access runtime services
    let system_table = match uefi::table::system_table_raw() {
        Some(st) => st,
        None => return Err(uefi::Status::UNSUPPORTED),
    };
    
    // Call set_virtual_address_map
    match unsafe { 
        runtime::set_virtual_address_map(
            &mut descriptors[..descriptor_count],
            system_table.as_ptr()
        ) 
    } {
        Ok(()) => Ok(()),
        Err(err) => Err(err.status()),
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
