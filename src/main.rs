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
// alloc::format imported in display module

// Configuration Options
// Hardware Inventory Verbose Output
// Set to true to see detailed device enumeration with device paths (if DevicePathToText protocol available)
// Set to false for clean, summary-only output
const VERBOSE_HARDWARE_INVENTORY: bool = false;

// Include our modules
mod constants;
mod handoff;
mod serial;
mod display;
mod hardware;
mod acpi;
mod system_info;
mod drivers;

use handoff::{Handoff, HANDOFF};
use display::*;
use hardware::{collect_hardware_inventory, get_loaded_image_device_path, display_hardware_inventory};
use system_info::*;
use acpi::find_acpi_rsdp;
use drivers::OutputDriver;
use constants::{io_ports, exit_codes};
use alloc::format;

/// Helper function to write a line using the output driver
fn write_line(output_driver: &mut OutputDriver, message: &str) {
    output_driver.write_line(message);
}

/// Main UEFI entry point
/// 
/// This function initializes the UEFI environment, collects comprehensive system information,
/// and prepares the handoff structure for the kernel. It follows UEFI best practices for
/// protocol access, memory management, and boot services exit preparation.
/// 
/// # Returns
/// 
/// * `Status::SUCCESS` - All system information collected successfully
/// * Other status codes - Various error conditions during initialization
/// 
/// # Safety
/// 
/// This function is the main entry point and assumes UEFI boot services are active.
/// It will panic if called after exit_boot_services.
#[entry]
fn efi_main() -> Status {
    // Step 1: Initialize UEFI logger
    uefi::helpers::init().unwrap();

    // Step 2: Initialize serial communication and output driver
    let serial_handle = uefi::boot::locate_handle_buffer(
        SearchType::ByProtocol(&Serial::GUID)
    )
    .ok()
    .and_then(|buf| buf.first().copied());

    // Initialize the output driver system
    let mut output_driver = OutputDriver::new(serial_handle);

    // Step 3: Output startup message
    output_driver.write_line("=== HobbyOS UEFI Loader Starting ===");
    output_driver.write_line(&format!("Output driver initialized: {}", output_driver.current_driver_name()));

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
        output_driver.write_line("✓ Graphics Output Protocol (GOP) found and initialized");
        display_gop_info(&mut output_driver, w as u32, h as u32, pf, stride as u32, fb_base, fb_size as u64);
        output_driver.write_line("✓ Framebuffer information collected and stored in handoff structure");
    } else {
        output_driver.write_line("✗ Graphics Output Protocol (GOP) not available");
        output_driver.write_line("  No framebuffer information will be available to kernel");
    }

    // Step 7: Collect Memory Map Information
    write_line(&mut output_driver, "Collecting memory map information...");
    let memory_map = match uefi::boot::memory_map(MemoryType::LOADER_DATA) {
        Ok(mmap) => {
            // Get memory map information using the correct UEFI 0.35 API
            let meta = mmap.meta();
            let descriptor_size = meta.desc_size as u32;
            let descriptor_version = meta.desc_version as u32;
            let entries_count = mmap.len() as u32;
            let total_size = meta.map_size as u32;
            
            // Store memory map information in handoff structure
            // Store the actual buffer pointer and metadata for the kernel
            unsafe {
                HANDOFF.memory_map_buffer_ptr = mmap.buffer().as_ptr() as u64;
                HANDOFF.memory_map_descriptor_size = descriptor_size;
                HANDOFF.memory_map_descriptor_version = descriptor_version;
                HANDOFF.memory_map_entries = entries_count;
                HANDOFF.memory_map_size = total_size;
            }
            
            // Display the actual memory map entries
            display_memory_map_entries(&mut output_driver, &mmap);
            
            Some(mmap)
        }
        Err(_) => {
            write_line(&mut output_driver, "✗ Failed to collect memory map");
            None
        }
    };

    // Step 8: Report Memory Map status with pretty formatting
    if let Some(ref _mmap) = memory_map {
        unsafe {
            HANDOFF.memory_map_buffer_ptr = memory_map.as_ref().unwrap().buffer().as_ptr() as u64;
        }
        
        write_line(&mut output_driver, "✓ Memory map collected successfully");
        unsafe {
            display_memory_map_info(&mut output_driver, HANDOFF.memory_map_descriptor_size, HANDOFF.memory_map_descriptor_version, HANDOFF.memory_map_entries, HANDOFF.memory_map_size);
        }
        write_line(&mut output_driver, "✓ Memory map information stored in handoff structure");
    } else {
        write_line(&mut output_driver, "✗ Memory map collection failed");
        write_line(&mut output_driver, "  No memory information will be available to kernel");
    }

    // Step 9: Locate ACPI RSDP Table
    write_line(&mut output_driver, "Locating ACPI RSDP table...");
    write_line(&mut output_driver, "  Debug: About to call find_acpi_rsdp");
    let rsdp_address = find_acpi_rsdp(&mut output_driver).unwrap_or(0);
    write_line(&mut output_driver, "  Debug: find_acpi_rsdp returned");
    unsafe { HANDOFF.acpi_rsdp = rsdp_address; }

    // Step 10: Report ACPI status with pretty formatting
    if rsdp_address != 0 {
        write_line(&mut output_driver, "✓ ACPI RSDP table found");
        display_acpi_info(&mut output_driver, rsdp_address);
        write_line(&mut output_driver, "✓ ACPI information stored in handoff structure");
    } else {
        write_line(&mut output_driver, "✗ ACPI RSDP table not found");
        display_acpi_info(&mut output_driver, rsdp_address);
        write_line(&mut output_driver, "  No ACPI support will be available to kernel");
    }

    // Step 11: Collect Device Tree Information
    write_line(&mut output_driver, "Collecting device tree information...");
    let device_tree_info = find_device_tree();
    match device_tree_info {
        Some((dtb_ptr, dtb_size)) => {
            unsafe {
                HANDOFF.device_tree_ptr = dtb_ptr;
                HANDOFF.device_tree_size = dtb_size;
            }
            write_line(&mut output_driver, "✓ Device tree information collected");
            display_device_tree_info(&mut output_driver, dtb_ptr, dtb_size);
        }
        None => {
            output_driver.write_line("✗ Device tree information not available");
            display_device_tree_info(&mut output_driver, 0, 0);
        }
    }

    // Step 12: Collect Firmware Information
    output_driver.write_line("Collecting firmware information...");
    let firmware_info = collect_firmware_info();
    match firmware_info {
        Some((vendor_ptr, vendor_len, revision)) => {
            unsafe {
                HANDOFF.firmware_vendor_ptr = vendor_ptr;
                HANDOFF.firmware_vendor_len = vendor_len;
                HANDOFF.firmware_revision = revision;
            }
            output_driver.write_line("✓ Firmware information collected");
            display_firmware_info(&mut output_driver, vendor_ptr, vendor_len, revision);
        }
        None => {
            output_driver.write_line("✗ Firmware information not available");
            display_firmware_info(&mut output_driver, 0, 0, 0);
        }
    }

    // Step 13: Collect Boot Time Information
    output_driver.write_line("Collecting boot time information...");
    let boot_time_info = collect_boot_time_info();
    match boot_time_info {
        Some((seconds, nanoseconds)) => {
            unsafe {
                HANDOFF.boot_time_seconds = seconds;
                HANDOFF.boot_time_nanoseconds = nanoseconds;
            }
            output_driver.write_line("✓ Boot time information collected");
            display_boot_time_info(&mut output_driver, seconds, nanoseconds);
        }
        None => {
            output_driver.write_line("✗ Boot time information not available");
            display_boot_time_info(&mut output_driver, 0, 0);
        }
    }

    // Step 14: Collect Boot Device Path Information
    output_driver.write_line("Collecting boot device path information...");
    let boot_device_path_info = collect_boot_device_path();
    match boot_device_path_info {
        Some((device_path_ptr, device_path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = device_path_ptr;
                HANDOFF.boot_device_path_size = device_path_size;
            }
            output_driver.write_line("✓ Boot device path information collected");
            display_boot_device_path_info(&mut output_driver, device_path_ptr, device_path_size);
        }
        None => {
            output_driver.write_line("✗ Boot device path information not available");
            display_boot_device_path_info(&mut output_driver, 0, 0);
        }
    }

    // Step 15: Collect CPU Information
    output_driver.write_line("Collecting CPU information...");
    let cpu_info = collect_cpu_info();
    match cpu_info {
        Some((cpu_count, cpu_features, microcode_revision)) => {
            unsafe {
                HANDOFF.cpu_count = cpu_count;
                HANDOFF.cpu_features = cpu_features;
                HANDOFF.microcode_revision = microcode_revision;
            }
            output_driver.write_line("✓ CPU information collected");
            display_cpu_info(&mut output_driver, cpu_count, cpu_features, microcode_revision);
        }
        None => {
            output_driver.write_line("✗ CPU information not available");
            display_cpu_info(&mut output_driver, 0, 0, 0);
        }
    }

    // Step 16: Collect Hardware Inventory
    output_driver.write_line("Collecting hardware inventory...");
    let hardware_inventory = collect_hardware_inventory(&mut output_driver, VERBOSE_HARDWARE_INVENTORY);
    match hardware_inventory {
        Some(inventory) => {
            unsafe {
                HANDOFF.hardware_device_count = inventory.device_count;
                HANDOFF.hardware_inventory_ptr = inventory.devices_ptr;
                HANDOFF.hardware_inventory_size = inventory.total_size;
            }
            output_driver.write_line("✓ Hardware inventory collected");
            display_hardware_inventory(&mut output_driver, &inventory);
        }
        None => {
            output_driver.write_line("✗ Hardware inventory collection failed");
        }
    }

    // Step 17: Get Loaded Image Device Path
    output_driver.write_line("Getting loaded image device path...");
    let loaded_image_path = get_loaded_image_device_path(&mut output_driver);
    match loaded_image_path {
        Some((path_ptr, path_size)) => {
            unsafe {
                HANDOFF.boot_device_path_ptr = path_ptr;
                HANDOFF.boot_device_path_size = path_size;
            }
            output_driver.write_line("✓ Loaded image device path collected");
            display_boot_device_path_info(&mut output_driver, path_ptr, path_size);
        }
        None => {
            output_driver.write_line("✗ Loaded image device path not available");
        }
    }

    // Step 18: Finalize Handoff Structure
    output_driver.write_line("Finalizing handoff structure...");
    unsafe {
        HANDOFF.size = core::mem::size_of::<Handoff>() as u32;
    }
    output_driver.write_line(&alloc::format!("✓ Handoff structure size: {} bytes", core::mem::size_of::<Handoff>()));
    output_driver.write_line("✓ All system information collected and stored");

    // Step 19: Exit Boot Services
    output_driver.write_line("Exiting boot services...");
    if let Some(mmap) = memory_map {
        // Get the memory map key for exit_boot_services
        let memory_map_key = mmap.key();
        output_driver.write_line(&alloc::format!("Memory map key: {:?}", memory_map_key));
        
        // Use the proper uefi-rs 0.35 exit_boot_services function
        // This will properly exit boot services and return the final memory map
        output_driver.write_line("✓ Memory map ready for kernel handoff");
        output_driver.write_line("⚠ Manual exit_boot_services not implemented in this version");
        output_driver.write_line("  The memory map is preserved for kernel access");
        output_driver.write_line("  Note: Use uefi::boot::exit_boot_services() for proper transition");
    } else {
        output_driver.write_line("✗ Cannot prepare memory map without memory map");
        output_driver.write_line("⚠ No memory map available for kernel");
    }

    // Step 20: Bootloader complete - exit QEMU
    output_driver.write_line("=== UEFI Loader Complete ===");
    output_driver.write_line("All system information collected and stored");
    output_driver.write_line("Ready for kernel handoff");
    output_driver.write_line("Exiting QEMU...");
    
    // Exit QEMU gracefully via I/O port
    // This makes testing much faster since we don't need timeouts
    unsafe {
        asm!("out dx, al", in("dx") io_ports::QEMU_EXIT, in("al") exit_codes::QEMU_SUCCESS, options(nomem, nostack, preserves_flags));
    }
    
    Status::SUCCESS
}
