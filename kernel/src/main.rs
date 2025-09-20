//! TheseusOS Kernel
//! 
//! This is the main kernel module for TheseusOS, a bare-metal operating system
//! designed for x86-64 systems. The kernel is loaded by a UEFI bootloader and
//! takes control after boot services have been exited.
//! 
//! ## Architecture
//! 
//! The kernel is organized into several modules:
//! - `gdt`: Global Descriptor Table setup and management
//! - `interrupts`: Interrupt handling and control
//! - `cpu`: CPU feature detection and configuration
//! - `memory`: Memory management and page table structures (not yet active)
//! 
//! ## Boot Process
//! 
//! 1. UEFI bootloader loads the kernel binary
//! 2. Bootloader collects system information (memory map, ACPI, etc.)
//! 3. Bootloader exits UEFI boot services
//! 4. Bootloader jumps to kernel entry point
//! 5. Kernel initializes heap from pre-allocated memory
//! 6. Kernel sets up environment (interrupts, GDT, CPU features)
//! 7. Kernel begins normal operation

#![no_std]
#![no_main]
#![feature(abi_x86_interrupt)]

extern crate alloc;

// Include our modules
mod gdt;
mod interrupts;
mod cpu;
mod memory;
mod allocator;
mod handoff;
mod display;
mod environment;
mod panic;

use allocator::initialize_heap_from_handoff;
use handoff::{set_handoff_pointers, validate_handoff};
use display::{kernel_write_line};
use environment::setup_kernel_environment;
// use boot_services::exit_boot_services; // Not needed - bootloader handles this

/// Main kernel entry point

// replaced by shared macros: out_char_0xe9! and print_hex_u64_0xe9!

/// Kernel entry point
/// 
/// This is where the kernel starts after the bootloader has:
/// 1. Collected all system information
/// 2. Exited boot services
/// 3. Jumped to this location
/// 
/// The handoff structure address is passed as a parameter.
#[no_mangle]
pub extern "C" fn kernel_main(handoff_addr: u64) -> ! {
    // Initialize kernel logging using QEMU debug port
    // Note: We use direct QEMU debug port output for kernel logging
    
    kernel_write_line("=== TheseusOS Kernel Starting ===");
    kernel_write_line("Kernel entry point reached successfully");

    // Initialize heap from memory map
    kernel_write_line("Initializing heap from memory map...");
    initialize_heap_from_handoff(handoff_addr);
    set_handoff_pointers(handoff_addr);
    
    kernel_write_line("Handoff structure address received");
    
    // Access the handoff structure from the passed address
    unsafe {
        if handoff_addr != 0 {
            let handoff_ptr = handoff_addr as *const theseus_shared::handoff::Handoff;
            let handoff = &*handoff_ptr;
            
            if handoff.size > 0 {
                kernel_write_line("Handoff structure found");
                // Dump full handoff for debugging; set second arg to false to disable raw bytes
                crate::display::dump_handoff(handoff, false);
                // Sanity-check critical fields to fail-fast on malformed handoff
                match validate_handoff(handoff) {
                    Ok(()) => kernel_write_line("Handoff validation passed"),
                    Err(msg) => {
                        kernel_write_line("Handoff validation failed: ");
                        theseus_shared::qemu_println!(msg);
                        panic!("Invalid handoff structure");
                    }
                }
                
                // Display system information from handoff (kept optional)
                // display_handoff_info(handoff);
                
                // Set up complete kernel environment (boot services have been exited)
                setup_kernel_environment(handoff, handoff.kernel_physical_base);
                
            } else {
                kernel_write_line("ERROR: Handoff structure has invalid size");
            }
        } else {
            kernel_write_line("ERROR: Handoff structure address is null");
        }
    }
    
    // For now, just exit QEMU
    kernel_write_line("Kernel initialization complete");
    kernel_write_line("Exiting QEMU...");
    
    theseus_shared::qemu_exit_ok!();
    
    loop {}
}