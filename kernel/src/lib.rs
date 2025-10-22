//! TheseusOS Kernel Library (Single-Binary Architecture)
//!
//! This library provides the kernel functionality for the unified UEFI+kernel binary.
//! It exports `kernel_entry` which is called directly by the UEFI bootloader after
//! exiting boot services. Integration tests also import functions from this library.

#![no_std]
#![feature(custom_test_frameworks)]
#![feature(abi_x86_interrupt)]

extern crate alloc;

// Re-export all the kernel modules
pub mod acpi;
pub mod boot;
pub mod bootlogo;
pub mod cpu;
pub mod display;
pub mod drivers;
pub mod environment;
pub mod framebuffer;
pub mod gdt;
pub mod handoff;
pub mod interrupts;
pub mod logging;
pub mod memory;
pub mod monitor;
pub mod panic;
pub mod physical_memory;
pub mod serial_debug;
pub mod stack;

// Config must come after logging since it references logging types
pub mod config;

// Re-export commonly used types and functions
pub use environment::setup_kernel_environment;
pub use handoff::{set_handoff_pointers, validate_handoff};

/// Unified kernel entry point for single-binary boot
///
/// This is the main kernel entry function called directly by the UEFI bootloader
/// after it exits boot services. It receives a physical address pointing to the
/// handoff structure containing all system information collected by the bootloader.
///
/// ## Boot Flow
///
/// 1. Bootloader collects system information using UEFI Boot Services
/// 2. Bootloader calls `ExitBootServices`
/// 3. Bootloader calls this function with handoff structure address
/// 4. Kernel validates handoff and sets up environment
/// 5. Kernel establishes higher-half virtual memory mapping
/// 6. Kernel initializes heap, interrupts, and other subsystems
/// 7. Kernel enters idle loop or exits based on configuration
///
/// # Arguments
///
/// * `handoff_addr` - Physical address of the handoff structure
///
/// # Panics
///
/// Panics if:
/// - Handoff structure is invalid or malformed
/// - Critical initialization steps fail
///
/// # Safety
///
/// This function assumes:
/// - UEFI boot services have been exited
/// - Handoff structure is valid and accessible
/// - System is in a stable state for kernel initialization
#[no_mangle]
pub extern "C" fn kernel_entry(handoff_addr: u64) -> ! {
    // Initialize logging subsystem first (before any log calls)
    crate::logging::init();

    // Initialize kernel logging using unified logging system
    log_info!("=== TheseusOS Kernel Starting ===");
    log_info!("Kernel entry point reached successfully");

    // Allocator shim is installed globally; kernel will arm it later
    log_debug!("Allocator shim active (pre-exit backend)");
    crate::handoff::set_handoff_pointers(handoff_addr);

    log_debug!("Handoff structure address received");

    // Access the handoff structure from the passed address
    unsafe {
        if handoff_addr != 0 {
            let handoff_ptr = handoff_addr as *const theseus_shared::handoff::Handoff;
            let handoff = &*handoff_ptr;

            if handoff.size > 0 {
                log_debug!("Handoff structure found");

                // Dump handoff structure only if verbose output is enabled
                if crate::config::VERBOSE_KERNEL_OUTPUT {
                    crate::display::dump_handoff(handoff, false);
                }

                // Sanity-check critical fields to fail-fast on malformed handoff
                match crate::handoff::validate_handoff(handoff) {
                    Ok(()) => log_debug!("Handoff validation passed"),
                    Err(msg) => {
                        log_error!("Handoff validation failed: {}", msg);
                        panic!("Invalid handoff structure");
                    }
                }

                // Set up handoff for timer interrupt access
                // TODO: Does this in a more fancy way in the future.
                crate::interrupts::set_handoff_for_timer(handoff);

                // Set up complete kernel environment (boot services have been exited)
                crate::environment::setup_kernel_environment(handoff, handoff.kernel_physical_base);
            } else {
                log_error!("ERROR: Handoff structure has invalid size");
            }
        } else {
            log_error!("ERROR: Handoff structure address is null");
        }
    }

    // If we reach here, it means setup_kernel_environment returned (which shouldn't happen)
    log_error!("ERROR: setup_kernel_environment returned unexpectedly");
    loop {}
}

/// Custom test runner for integration tests
///
/// This function is called by Rust's custom test framework to run all test cases.
/// However, we discovered that this mechanism has issues in bare-metal environments
/// where calling functions through the `&dyn Fn()` trait objects causes hangs.
///
/// **Current Status**: This function is kept for compatibility but is not used
/// in our current test implementation. Instead, we call test functions directly
/// from the `kernel_main` entry point to avoid the hanging issue.
///
/// **Why it doesn't work**: The issue appears to be related to how Rust's
/// custom test framework collects and calls test functions through trait objects
/// in a bare-metal environment. The `test()` call hangs indefinitely.
///
/// **Solution**: Direct function calls from `kernel_main` work reliably and
/// provide the same functionality without the framework overhead.
///
/// **Usage**: This function would be called by `test_main()` if we used the
/// standard custom test framework approach.
///
/// # Parameters
///
/// * `tests` - Slice of function pointers to test functions
///
/// # Example
///
/// ```rust
/// // This is what the framework would do (but doesn't work):
/// test_runner(&[&test_function_1, &test_function_2]);
/// ```
pub fn test_runner(tests: &[&dyn Fn()]) {
    // Print the number of tests we're about to run
    log_info!("Running {} tests", tests.len());

    // Attempt to run each test function
    // NOTE: This is where the hanging occurs in bare-metal environments
    for test in tests {
        test(); // This call hangs in bare-metal
    }

    // If we reach here, all tests passed
    log_info!("All tests passed!");
    theseus_shared::qemu_exit_ok!();
}
