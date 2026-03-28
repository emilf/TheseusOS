//! Module: kernel crate root
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A1:-The-kernel-boots-as-a-single-UEFI-executable
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//!
//! INVARIANTS:
//! - `kernel_entry` is the unified kernel entry point reached directly from the bootloader after firmware boot services are gone.
//! - Logging is initialized before the rest of kernel bring-up emits diagnostics.
//! - The crate root exposes the major kernel subsystems that participate in the documented bring-up path.
//!
//! SAFETY:
//! - `kernel_entry` assumes the bootloader already established the handoff copy and exited boot services successfully.
//! - The crate root is not where low-level MMIO/DMA safety is enforced; subsystem modules still own those contracts.
//! - Test-framework commentary here must not be mistaken for a current architecture guarantee unless the implementation path actually depends on it.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//! - docs/plans/interrupts-and-platform.md
//!
//! TheseusOS kernel crate for the current single-binary boot path.
//!
//! This crate exports `kernel_entry`, which the bootloader calls directly after
//! boot services are gone.

#![no_std]
#![feature(custom_test_frameworks)]
#![feature(abi_x86_interrupt)]

extern crate alloc;

// Re-export all the kernel modules
pub mod acpi;
pub mod boot;
pub mod bootlogo;
pub mod cpu;
pub mod cpu_features;
pub mod display;
pub mod drivers;
pub mod environment;
pub mod framebuffer;
pub mod gdt;
pub mod handoff;
pub mod input;
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

/// Unified kernel entry point for the single-binary boot path.
///
/// The bootloader calls this directly after `ExitBootServices`, passing the
/// physical address of the copied handoff structure.
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

                // Transitional timer/debug plumbing still reaches through handoff data.
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

    // Reaching this point means the expected non-returning environment setup path returned.
    log_error!("ERROR: setup_kernel_environment returned unexpectedly");
    loop {}
}

/// Legacy custom test-runner hook kept for framework compatibility.
pub fn test_runner(tests: &[&dyn Fn()]) {
    // Print the number of tests we're about to run
    log_info!("Running {} tests", tests.len());

    // Attempt to run each test function; this path is retained mainly as a
    // compatibility hook rather than the preferred bare-metal workflow.
    for test in tests {
        test(); // This call hangs in bare-metal
    }

    // If we reach here, all tests passed
    log_info!("All tests passed!");
    theseus_shared::qemu_exit_ok!();
}
