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
mod boot_sequence;

use boot_sequence::*;
use drivers::OutputDriver;
use uefi::Status;

/// Main UEFI entry point
/// 
/// This function orchestrates the boot sequence by calling specialized functions
/// to collect system information and prepare the handoff structure for the kernel.
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
    // Initialize UEFI environment and output driver
    let (mut output_driver, _serial_handle) = initialize_uefi_environment()
        .map_err(|_| Status::ABORTED)?;

    // Collect all system information
    collect_graphics_info(&mut output_driver);
    let memory_map = collect_memory_map(&mut output_driver);
    collect_acpi_info(&mut output_driver);
    collect_system_info(&mut output_driver);
    collect_hardware_inventory_info(&mut output_driver, VERBOSE_HARDWARE_INVENTORY);
    collect_loaded_image_path(&mut output_driver);

    // Finalize handoff structure
    finalize_handoff_structure(&mut output_driver);

    // Prepare for boot services exit
    prepare_boot_services_exit(&mut output_driver, &memory_map);

    // Complete bootloader and exit QEMU
    complete_bootloader_and_exit(&mut output_driver);
    
    Status::SUCCESS
}
