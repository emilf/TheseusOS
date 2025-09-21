//! TheseusOS Kernel Library
//! 
//! This library exposes the kernel functionality for use in integration tests.
//! The kernel binary imports from this library, and integration tests can
//! also import functions from here.

#![no_std]
#![feature(custom_test_frameworks)]
#![feature(abi_x86_interrupt)]

extern crate alloc;

// Re-export all the kernel modules
pub mod gdt;
pub mod interrupts;
pub mod cpu;
pub mod memory;
pub mod allocator;
pub mod handoff;
pub mod display;
pub mod environment;
pub mod panic;

// Re-export commonly used types and functions
pub use allocator::initialize_heap_from_handoff;
pub use handoff::{set_handoff_pointers, validate_handoff};
pub use display::kernel_write_line;
pub use environment::setup_kernel_environment;

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
    theseus_shared::qemu_print!("Running ");
    theseus_shared::print_hex_u64_0xe9!(tests.len() as u64);
    theseus_shared::qemu_println!(" tests");
    
    // Attempt to run each test function
    // NOTE: This is where the hanging occurs in bare-metal environments
    for test in tests {
        test(); // This call hangs in bare-metal
    }
    
    // If we reach here, all tests passed
    theseus_shared::qemu_println!("All tests passed!");
    theseus_shared::qemu_exit_ok!();
}
