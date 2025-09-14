#![no_std]
#![no_main]

extern crate alloc;

use uefi::prelude::*;
// alloc::format imported in display module
use theseus_shared::handoff::{Handoff, HANDOFF};

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

/// Allocate temporary heap memory for kernel setup
/// 
/// This function allocates a chunk of safe memory that the kernel can use for its
/// temporary heap during initialization. The memory is allocated using UEFI Boot Services
/// and stored in the handoff structure for the kernel to use. This allows the kernel
/// to have a working heap immediately upon entry without needing to parse memory maps.
fn allocate_temp_heap_for_kernel() {
    write_line("=== Allocating Temporary Heap for Kernel ===");
    
    // Allocate 1MB of conventional memory for the kernel's temporary heap
    const TEMP_HEAP_SIZE: u64 = 1024 * 1024; // 1MB
    
    match memory::allocate_memory(TEMP_HEAP_SIZE, uefi::boot::MemoryType::LOADER_DATA) {
        Ok(region) => {
            write_line(&format!("✓ Temporary heap allocated successfully"));
            write_line(&format!("  Base address: 0x{:016x}", region.physical_address));
            write_line(&format!("  Size: {} bytes ({} KB)", region.size, region.size / 1024));
            
            // Store the heap information in the handoff structure
            unsafe {
                HANDOFF.temp_heap_base = region.physical_address;
                HANDOFF.temp_heap_size = region.size;
                HANDOFF.boot_services_exited = 0;
            }
            
            write_line("✓ Temporary heap information stored in handoff structure");
        },
        Err(status) => {
            write_line(&format!("✗ Failed to allocate temporary heap: {:?}", status));
            write_line("Kernel will need to use fallback heap allocation");
            
            // Set heap fields to 0 to indicate allocation failed
            unsafe {
                HANDOFF.temp_heap_base = 0;
                HANDOFF.temp_heap_size = 0;
                HANDOFF.boot_services_exited = 0;
            }
        }
    }
}

/// Complete the bootloader and exit QEMU
fn complete_bootloader_and_exit() {
    write_line("=== TheseusOS UEFI Loader Complete ===");
    write_line("All system information collected and stored");
    write_line("Ready for kernel handoff");
    write_line("Exiting QEMU...");
    
    // Exit QEMU gracefully with success message
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") constants::io_ports::QEMU_EXIT,
            in("al") constants::exit_codes::QEMU_SUCCESS,
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

    // Allocate temporary heap for kernel
    allocate_temp_heap_for_kernel();

    // Finalize handoff structure
    finalize_handoff_structure();

    // Load kernel and jump to it
    // The kernel will handle its own boot services exit and virtual memory setup
    if let Some(mmap) = &memory_map {
        write_line("About to call load_kernel_binary...");
        match crate::kernel_loader::load_kernel_binary(mmap) {
            Ok((kernel_physical_base, kernel_entry_point)) => {
                // Update the handoff structure with the actual kernel information
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
                
                write_line("✓ All system information collected, jumping to kernel...");
                
                // Jump to kernel with handoff structure address
                unsafe {
                    crate::boot_sequence::jump_to_kernel_with_handoff(kernel_physical_base, &raw const HANDOFF as *const Handoff);
                }
            }
            Err(status) => {
                write_line(&format!("✗ Failed to load kernel: {:?}", status));
                write_line("Cannot proceed with kernel handoff");
            }
        }
    } else {
        write_line("✗ No memory map available for kernel loading");
    }

    // If we reach here, it means there was an error in the handoff process
    // Complete bootloader and exit QEMU gracefully
    complete_bootloader_and_exit();
    
    Status::SUCCESS
}
