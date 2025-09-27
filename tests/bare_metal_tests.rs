//! Bare-Metal Tests for TheseusOS
//!
//! These tests run immediately after the bootloader hands off control to the kernel.
//! They execute in a bare-metal environment with:
//! - No kernel services (heap, memory mapping, interrupts)
//! - No allocator available
//! - Only basic CPU and stack operations
//!
//! This is the most basic test environment and validates core functionality
//! before any kernel initialization.

#![no_std]
#![no_main]
#![feature(custom_test_frameworks)]
#![test_runner(dummy_test_runner)]
#![reexport_test_harness_main = "test_main"]

use core::panic::PanicInfo;

/// Bare-metal test entry point
///
/// This is called by the bootloader after it has:
/// 1. Exited UEFI boot services
/// 2. Jumped to this location
/// 3. Passed the handoff structure address
///
/// In bare-metal environment, we have:
/// - No heap allocator
/// - No kernel services
/// - No memory mapping
/// - No interrupt handling
/// - Only basic CPU and stack operations
#[no_mangle]
pub extern "C" fn kernel_main(_handoff_addr: u64) -> ! {
    // Print kernel main marker for debugging
    // This helps us track execution flow in QEMU output
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9", // QEMU debug port
            "mov al, 'M'",  // Marker: Main entry point
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Call tests directly - bypass the broken custom test framework
    // We discovered that the framework's test() call mechanism hangs
    // in bare-metal environments, so we call functions directly
    test_basic_cpu_ops();
    test_memory_operations();
    test_control_flow();

    // Print completion marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9",
            "mov al, 'D'", // Marker: Done
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Exit QEMU with success code (0)
    // This uses QEMU's isa-debug-exit device
    unsafe {
        core::arch::asm!(
            "mov dx, 0xf4", // isa-debug-exit device port
            "mov al, 0",    // Exit code 0 = success
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Infinite loop - should never be reached due to QEMU exit
    loop {}
}

/// Dummy test runner - we don't use it since we call tests directly
///
/// This function exists only to satisfy the custom test framework
/// requirements. It will never be called since we bypass test_main()
/// and call test functions directly in kernel_main().
pub fn dummy_test_runner(_tests: &[&dyn Fn()]) {
    // This will never be called since we bypass test_main()
}

/// Panic handler for bare-metal tests
///
/// In case of a panic (e.g., assertion failure), this handler:
/// 1. Prints a panic marker ('P') to QEMU debug port
/// 2. Exits QEMU with error code 1
/// 3. Enters infinite loop (should never be reached)
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Print panic marker for debugging
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9", // QEMU debug port
            "mov al, 'P'",  // Marker: Panic occurred
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Exit QEMU with error code (1)
    unsafe {
        core::arch::asm!(
            "mov dx, 0xf4", // isa-debug-exit device port
            "mov al, 1",    // Exit code 1 = failure
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Infinite loop - should never be reached due to QEMU exit
    loop {}
}

/// Test basic CPU operations in bare-metal environment
///
/// This test verifies fundamental CPU operations that should work
/// in any environment, including:
/// - Arithmetic operations (add, multiply)
/// - Comparison operations
/// - Basic control flow
///
/// Expected output: '1' (start) -> 'A' (end)
fn test_basic_cpu_ops() {
    // Print test start marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9", // QEMU debug port
            "mov al, '1'",  // Marker: Test 1 start
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Test basic arithmetic operations
    assert_eq!(1 + 1, 2); // Addition
    assert_eq!(5 * 3, 15); // Multiplication

    // Print test completion marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9",
            "mov al, 'A'", // Marker: Test 1 complete
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }
}

/// Test memory operations in bare-metal environment
///
/// This test verifies basic memory operations that work on the stack:
/// - Variable assignment and copying
/// - Stack-based operations
/// - Basic memory access patterns
///
/// Note: This does NOT test heap allocation, which is not available
/// in bare-metal environment.
///
/// Expected output: '2' (start) -> 'B' (end)
fn test_memory_operations() {
    // Print test start marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9", // QEMU debug port
            "mov al, '2'",  // Marker: Test 2 start
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Test basic stack operations
    let x = 42; // Stack variable
    let y = x; // Copy to another stack variable
    assert_eq!(x, y); // Verify the copy worked

    // Print test completion marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9",
            "mov al, 'B'", // Marker: Test 2 complete
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }
}

/// Test control flow in bare-metal environment
///
/// This test verifies basic control flow constructs:
/// - Loops (for loops)
/// - Variable mutation
/// - Conditional logic
/// - Function calls and returns
///
/// Expected output: '3' (start) -> 'C' (end)
fn test_control_flow() {
    // Print test start marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9", // QEMU debug port
            "mov al, '3'",  // Marker: Test 3 start
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }

    // Test basic control flow with loops
    let mut counter = 0; // Mutable variable
    for i in 1..=3 {
        // Range-based for loop
        counter += i; // Accumulate values
    }
    assert_eq!(counter, 6); // 1 + 2 + 3 = 6

    // Print test completion marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9",
            "mov al, 'C'", // Marker: Test 3 complete
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }
}
