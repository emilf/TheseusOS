//! TheseusOS UEFI Bootloader
//!
//! This is the UEFI bootloader for TheseusOS. It runs in the UEFI environment
//! and is responsible for:
//!
//! 1. **System Information Collection**: Gathering memory maps, ACPI tables,
//!    graphics information, hardware inventory, and other system details
//! 2. **Kernel Loading**: Loading the kernel binary from the EFI system partition
//!    and parsing its ELF structure
//! 3. **Memory Management**: Allocating memory for the kernel and temporary heap
//! 4. **Boot Services Exit**: Calling UEFI's ExitBootServices to transition
//!    control from firmware to the kernel
//! 5. **Kernel Handoff**: Jumping to the kernel entry point with a handoff
//!    structure containing all collected system information
//!
//! ## Architecture
//!
//! The bootloader is organized into several modules:
//! - `serial`: Serial communication and output
//! - `display`: Console output and display management
//! - `hardware`: Hardware detection and inventory
//! - `acpi`: ACPI table discovery and parsing
//! - `memory`: Memory map collection and management
//! - `graphics`: Graphics Output Protocol (GOP) setup
//! - `kernel_loader`: ELF parsing and kernel loading
//! - `boot_sequence`: Main boot sequence orchestration
//! - `qemu_exit`: QEMU exit device integration for testing

#![no_std]
#![no_main]

extern crate alloc;

use uefi::prelude::*;
// alloc::format imported in display module
use theseus_shared::handoff::{Handoff, HANDOFF};

// Configuration Options for Bootloader Output
//
// These constants control the verbosity of bootloader output.
// Set to false for clean, minimal output suitable for AI agents and CI/CD.
// Set to true for detailed debugging output.

/// Global verbose output control
///
/// This constant is used throughout the bootloader to control debug output.
/// When set to false, only essential information is displayed.
/// When set to true, detailed debug information is shown including:
/// - Complete memory maps and ACPI information
/// - Detailed hardware inventory
/// - Verbose boot device path information
/// - Full system information dumps
///
/// # Usage
///
/// Pass this constant to information collection functions to control
/// their verbosity level. All display functions check this flag before
/// showing detailed information.
pub const VERBOSE_OUTPUT: bool = false;

// Memory Allocation Testing
// Set to true to run memory allocation tests during boot
// Set to false to skip tests for faster boot
const RUN_MEMORY_TESTS: bool = false;

// Include our modules
mod acpi;
mod boot_sequence;
mod display;
mod hardware;
mod qemu_exit;
mod serial;
mod system_info;

// Use shared library

// Include bootloader-specific modules
mod drivers;
mod memory;

use alloc::format;
use boot_sequence::*;
use drivers::manager::{write_line, OutputDriver};
use uefi::Status;
// (no additional shared imports)

// Use kernel's panic handler to avoid duplicate lang item

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
        Ok(_) => {}
        Err(_) => return Status::ABORTED,
    };

    // Initialize the global output driver
    OutputDriver::init_global();

    write_line("=== TheseusOS UEFI Loader Starting ===");
    write_line(&format!(
        "Output driver initialized: {}",
        drivers::manager::current_driver_name()
    ));

    // Test panic handler (uncomment to test)
    // panic!("Testing panic handler - this should exit QEMU with error message");

    // Test memory allocation functions (configurable)
    if RUN_MEMORY_TESTS {
        write_line("=== Testing Memory Management ===");
        match memory::test_memory_allocation() {
            Ok(_) => {
                write_line("✓ Memory allocation tests passed");
            }
            Err(status) => {
                write_line(&format!("✗ Memory allocation tests failed: {:?}", status));
                return status;
            }
        }
    } else {
        write_line("✓ Memory allocation tests skipped (disabled)");
    }

    // Collect all system information
    collect_graphics_info(VERBOSE_OUTPUT);
    let _memory_map = collect_memory_map(VERBOSE_OUTPUT);
    collect_acpi_info(VERBOSE_OUTPUT);
    collect_system_info(VERBOSE_OUTPUT);
    collect_hardware_inventory_info(VERBOSE_OUTPUT);
    collect_loaded_image_path(VERBOSE_OUTPUT);

    // Allocate temporary heap for kernel (non-overlapping with kernel image)
    write_line("=== Allocating Temporary Heap for Kernel ===");
    const TEMP_HEAP_SIZE: u64 = 1024 * 1024; // 1MB
    // Defer actual allocation until after kernel image fields are computed

    // Finalize handoff structure
    finalize_handoff_structure();

    // Single-binary path: set kernel fields from entry symbol, then enter kernel
    boot_sequence::set_kernel_image_from_loaded_image();
    // Allocate the temp heap now, avoiding overlap with kernel image span
    unsafe {
        let img_start = HANDOFF.kernel_physical_base;
        let img_end = img_start.saturating_add(HANDOFF.kernel_image_size);
        match memory::allocate_memory_non_overlapping(
            TEMP_HEAP_SIZE,
            uefi::boot::MemoryType::LOADER_DATA,
            img_start,
            img_end,
        ) {
            Ok(region) => {
                write_line("✓ Temporary heap allocated successfully");
                write_line(&format!(
                    "  Base address: 0x{:016x}",
                    region.physical_address
                ));
                write_line(&format!(
                    "  Size: {} bytes ({} KB)",
                    region.size,
                    region.size / 1024
                ));
                HANDOFF.temp_heap_base = region.physical_address;
                HANDOFF.temp_heap_size = region.size;
                HANDOFF.boot_services_exited = 0;
            }
            Err(_) => {
                write_line("✗ Failed to allocate non-overlapping temporary heap");
                HANDOFF.temp_heap_base = 0;
                HANDOFF.temp_heap_size = 0;
            }
        }
    }
    // (markers removed)

    write_line("✓ All system information collected, preparing to exit boot services...");

    unsafe {
        crate::boot_sequence::jump_to_kernel_with_handoff(
            0,
            &raw const HANDOFF as *const Handoff,
        );
    }

    // If we reach here, it means there was an error in the handoff process
    // This should never happen since we jump to the kernel on success
    write_line("✗ ERROR: Unexpected bootloader completion - kernel handoff failed");
    panic!("Bootloader should never reach this point");
}
