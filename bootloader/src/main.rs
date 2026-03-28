//! Module: bootloader::main
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A1:-The-kernel-boots-as-a-single-UEFI-executable
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//! - docs/axioms/boot.md#A3:-Kernel-image-metadata-is-derived-from-the-live-binary-not-a-separately-parsed-on-disk-image
//!
//! INVARIANTS:
//! - This file is the UEFI entrypoint for the current single-binary boot flow.
//! - The bootloader gathers boot-time platform state, prepares the handoff, and eventually transfers control directly to `kernel_entry`.
//! - The current boot path derives kernel image metadata from the live binary instead of loading a separate kernel ELF.
//!
//! SAFETY:
//! - Firmware-side allocation and handoff preparation must stay aligned with the later kernel expectations documented in the boot/memory plans.
//! - Comments here must not drift into describing an older multi-binary/ELF-loader architecture as if it were still current truth.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//!
//! TheseusOS UEFI bootloader (single-binary).

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

/// Global verbose-output policy for the bootloader.
///
/// The default is intentionally `false` so normal development/test runs stay
/// high-signal. Turning it on enables extra firmware/boot-time dumps such as
/// memory-map, ACPI, hardware-inventory, and boot-device detail.
pub const VERBOSE_OUTPUT: bool = false;

/// Bootloader memory-allocation test toggle.
///
/// When enabled, the UEFI-phase allocator tests run during startup. The default
/// stays off because the normal repo workflow prefers faster, less noisy boots.
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
core::arch::global_asm!(".globl __chkstk", "__chkstk:", "    jmp ___chkstk_ms",);

// Note: Panic handler is provided by the kernel library to avoid duplicate lang items

/// Main UEFI entry point for the unified bootloader+kernel binary.
///
/// This performs the current firmware-side discovery, handoff preparation, and
/// direct `ExitBootServices` → `kernel_entry` transfer.
#[entry]
fn efi_main() -> Status {
    // -----------------------------------------------------------------------
    // GDB debug mailbox — write runtime efi_main address to a fixed physical
    // location so automated GDB tooling can discover it via a watchpoint.
    //
    // Must happen before any UEFI call so the address is visible as early as
    // possible. The page is allocated via UEFI AllocateType::Address so the
    // firmware records our ownership in the memory map.
    //
    // Layout at DEBUG_MAILBOX_PHYS:
    //   +0x00  u64  runtime efi_main address   (written first)
    //   +0x08  u64  magic sentinel              (written second → GDB trigger)
    //
    // See: shared/src/constants.rs :: debug_mailbox
    //      debug.gdb                :: theseus-auto command
    // -----------------------------------------------------------------------
    use theseus_shared::constants::debug_mailbox;
    use uefi::boot::{self as uefi_boot, AllocateType};
    use uefi::mem::memory_map::MemoryType as UefiMemType;

    // Allocate the mailbox page via UEFI so firmware records our ownership.
    // Ignore errors — if the page is already allocated (e.g. by firmware) we
    // fall back to a direct write; in QEMU/OVMF this range is always free.
    let _ = uefi_boot::allocate_pages(
        AllocateType::Address(debug_mailbox::PHYS),
        UefiMemType::LOADER_DATA,
        1,
    );

    unsafe {
        let base = debug_mailbox::PHYS as *mut u64;
        // Write address first, then magic — GDB watches the magic location.
        base.add(0).write_volatile(efi_main as *const () as u64);
        base.add(1).write_volatile(debug_mailbox::MAGIC);
    }

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
    let msg = format!("efi_main @ {:#x}", efi_main as *const () as usize);
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
    const TEMP_HEAP_SIZE: u64 = 1024 * 1024; // 1 MiB
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
    // Reaching this point means the expected kernel transfer did not happen.
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
