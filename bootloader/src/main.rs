#![no_std]
#![no_main]

extern crate alloc;

use uefi::prelude::*;
// alloc::format imported in display module

// Configuration Options
// Hardware Inventory Verbose Output
// Set to true to see detailed device enumeration with device paths (if DevicePathToText protocol available)
// Set to false for clean, summary-only output
const VERBOSE_HARDWARE_INVENTORY: bool = false;

// Include our modules
mod serial;
mod display;
mod hardware;
mod acpi;
mod system_info;
mod boot_sequence;
mod qemu_exit;

// Use shared library
use theseus_shared::constants;

// Include bootloader-specific modules
mod drivers;
mod kernel_loader;
mod memory;

use boot_sequence::*;
use uefi::Status;
use drivers::manager::{OutputDriver, write_line};
use alloc::format;

/// Panic handler for bootloader
#[panic_handler]
fn panic_handler(_panic_info: &core::panic::PanicInfo) -> ! {
    // Output panic information to QEMU debug port (avoiding allocator)
    const PANIC_PREFIX: &[u8] = b"BOOTLOADER PANIC: ";
    
    // Write panic prefix
    for &byte in PANIC_PREFIX {
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") constants::io_ports::QEMU_DEBUG,
                in("al") byte,
                options(nomem, nostack, preserves_flags)
            );
        }
    }
    
    // Output a generic panic message (we can't easily format the actual message without allocator)
    const PANIC_MSG: &[u8] = b"Panic occurred";
    for &byte in PANIC_MSG {
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") constants::io_ports::QEMU_DEBUG,
                in("al") byte,
                options(nomem, nostack, preserves_flags)
            );
        }
    }
    
    // Write newline
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") constants::io_ports::QEMU_DEBUG,
            in("al") b'\n',
            options(nomem, nostack, preserves_flags)
        );
    }
    
    // Exit QEMU with error
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") constants::io_ports::QEMU_EXIT,
            in("al") constants::exit_codes::QEMU_ERROR,
            options(nomem, nostack, preserves_flags)
        );
    }
    
    loop {}
}

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
    // Initialize UEFI environment and global output driver
    match initialize_uefi_environment() {
        Ok(_) => {},
        Err(_) => return Status::ABORTED,
    };
    
    // Initialize the global output driver
    OutputDriver::init_global();
    
    write_line("=== TheseusOS UEFI Loader Starting ===");
    write_line(&format!("Output driver initialized: {}", drivers::manager::current_driver_name()));

    // Test panic handler (uncomment to test)
    // panic!("Testing panic handler - this should exit QEMU with error message");
    
    // Test memory allocation functions
    write_line("=== Testing Memory Management ===");
    match memory::test_memory_allocation() {
        Ok(_) => {
            write_line("✓ Memory allocation tests passed");
        },
        Err(status) => {
            write_line(&format!("✗ Memory allocation tests failed: {:?}", status));
            return status;
        }
    }

    // Collect all system information
    collect_graphics_info();
    let memory_map = collect_memory_map();
    collect_acpi_info();
    collect_system_info();
    collect_hardware_inventory_info(VERBOSE_HARDWARE_INVENTORY);
    collect_loaded_image_path();

    // Finalize handoff structure
    finalize_handoff_structure();

    // Prepare for boot services exit and jump to kernel
    // This function will either jump to the kernel or exit QEMU if there's an error
    prepare_boot_services_exit(&memory_map);

    // If we reach here, it means there was an error in the handoff process
    // Complete bootloader and exit QEMU gracefully
    complete_bootloader_and_exit();
    
    Status::SUCCESS
}
