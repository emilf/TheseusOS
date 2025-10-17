//! TheseusOS UEFI Bootloader (Single-Binary)
//!
//! This is the unified UEFI application for TheseusOS. It combines the bootloader
//! and kernel into a single binary, eliminating the need for separate ELF loading.
//!
//! ## Responsibilities
//!
//! 1. **System Information Collection**: Gathering memory maps, ACPI tables,
//!    graphics information, hardware inventory, and other system details
//! 2. **Memory Management**: Allocating memory for temporary heap and handoff structure
//! 3. **Handoff Preparation**: Computing kernel image base/size from loaded binary
//! 4. **Boot Services Exit**: Calling UEFI's `ExitBootServices` to transition
//!    from firmware to kernel control
//! 5. **Kernel Entry**: Direct function call to `kernel_entry` in the same binary
//!
//! ## Architecture
//!
//! The bootloader is organized into several modules:
//! - `boot_sequence`: Main boot sequence orchestration and kernel entry
//! - `memory`: Memory map collection and allocation helpers
//! - `acpi`: ACPI table discovery and parsing
//! - `hardware`: Hardware detection and inventory
//! - `display`: Console output formatting
//! - `drivers`: Output driver management (UEFI serial, raw serial, QEMU debug)
//! - `system_info`: Firmware, boot time, and CPU information collection
//! - `serial`: Serial communication primitives
//! - `qemu_exit`: QEMU exit device integration for testing
//!
//! ## Single-Binary Boot Flow
//!
//! 1. UEFI firmware loads `BOOTX64.EFI` into memory
//! 2. Bootloader phase: collect system information using UEFI Boot Services
//! 3. Compute kernel image base/size from the loaded binary's entry symbol
//! 4. Allocate non-overlapping temporary heap for kernel use
//! 5. Call `ExitBootServices` to leave UEFI control
//! 6. Call `kernel_entry` directly (same binary, different function)
//! 7. Kernel establishes higher-half mapping and continues execution

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
extern crate alloc;

use uefi::prelude::*;
// alloc::format imported in display module
use theseus_shared::handoff::{Handoff, HANDOFF};

//
// Configuration Options
//

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

/// Memory allocation testing flag
///
/// When `true`, runs UEFI memory allocation tests during boot.
/// When `false`, skips tests for faster boot time.
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
use uefi::mem::memory_map::MemoryType;
use uefi::Status;
// (no additional shared imports)

#[no_mangle]
#[used]
#[link_section = ".rodata"]
pub static THESEUS_DEBUG_SIGNATURE: [u8; 16] = *b"THESEUSDBGBASE!\0";

#[cfg(all(target_arch = "x86_64", target_os = "uefi"))]
// Provide the Windows-style stack probing symbol expected by LLVM when we rebuild
// the bootloader with our custom DWARF-emitting target. The implementation lives
// in compiler_builtins as `___chkstk_ms`, so we just export the legacy name.
core::arch::global_asm!(
    ".globl __chkstk",
    "__chkstk:",
    "    jmp ___chkstk_ms",
);

// Note: Panic handler is provided by the kernel library to avoid duplicate lang items

/// Main UEFI entry point for the unified bootloader+kernel binary
///
/// This function orchestrates the single-binary boot sequence:
/// 1. Initializes UEFI environment and output drivers
/// 2. Collects comprehensive system information (memory map, ACPI, GOP, hardware)
/// 3. Computes kernel image base/size from the loaded binary
/// 4. Allocates non-overlapping temporary heap for kernel use
/// 5. Exits UEFI Boot Services
/// 6. Calls directly into `kernel_entry` within the same binary
///
/// # Returns
///
/// This function never returns normally. It either:
/// - Successfully transitions to kernel via `kernel_entry` (does not return)
/// - Panics on fatal errors during boot sequence
///
/// # Panics
///
/// Panics if the kernel handoff fails unexpectedly (should never happen).
///
/// # Safety
///
/// This function assumes UEFI boot services are active. It calls `ExitBootServices`
/// before transferring control to the kernel.
#[entry]
fn efi_main() -> Status {
    // Install pre-exit allocators that forward to UEFI Boot Services
    theseus_shared::allocator::install_pre_exit_allocators(pre_exit_alloc, pre_exit_dealloc);
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
    let msg = format!("efi_main @ {:#x}", efi_main as usize);
    theseus_shared::qemu_println!(msg);

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
        crate::boot_sequence::jump_to_kernel_with_handoff(0, &raw const HANDOFF as *const Handoff);
    }

    // If we reach here, it means there was an error in the handoff process
    // This should never happen since we jump to the kernel on success
    write_line("✗ ERROR: Unexpected bootloader completion - kernel handoff failed");
    panic!("Bootloader should never reach this point");
}

// ----------------------------
// Pre-Exit allocator callbacks
// ----------------------------
use core::alloc::Layout;
#[alloc_error_handler]
fn alloc_oom(_: core::alloc::Layout) -> ! {
    // In bootloader, treat OOM as fatal.
    loop {
        core::hint::spin_loop();
    }
}

unsafe fn pre_exit_alloc(layout: Layout) -> *mut u8 {
    // Use UEFI allocate_pool as a generic allocator before ExitBootServices
    match uefi::boot::allocate_pool(MemoryType::LOADER_DATA, layout.size()) {
        Ok(mut buf) => buf.as_mut(),
        Err(_) => core::ptr::null_mut(),
    }
}

unsafe fn pre_exit_dealloc(ptr: *mut u8, _layout: Layout) {
    if !ptr.is_null() {
        let _ = uefi::boot::free_pool(core::ptr::NonNull::new_unchecked(ptr));
    }
}
