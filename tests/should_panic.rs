//! Panic Tests for TheseusOS
//!
//! These tests verify that the kernel properly handles error conditions
//! by panicking when expected. They test the panic handling mechanism
//! and ensure that assertions work correctly.

#![no_std]
#![no_main]

extern crate theseus_shared;

use core::panic::PanicInfo;

/// Panic handler for panic tests
/// 
/// This is a special panic handler that indicates SUCCESS when a panic occurs.
/// This is because panic tests are designed to panic (e.g., via assert_eq!(0, 1))
/// and we want to verify that:
/// 1. The panic actually occurs
/// 2. The panic handler is called correctly
/// 3. We can exit cleanly after a panic
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Print success marker - panic occurred as expected
    theseus_shared::qemu_println!("[ok]");
    
    // Exit with success code (0) - panic was expected
    theseus_shared::qemu_exit_ok!();
    
    // Infinite loop - should never be reached due to QEMU exit
    loop {}
}

/// Panic test entry point
/// 
/// This test is designed to fail by calling a function that contains
/// an assertion that will always fail (assert_eq!(0, 1)). This verifies:
/// 1. Assertions work correctly
/// 2. Panic handling works correctly
/// 3. The test framework can detect and report panics properly
#[no_mangle]
pub extern "C" fn kernel_main(_handoff_addr: u64) -> ! {
    // Call a function that will panic
    should_fail();
    
    // If we reach here, the panic didn't occur, which is a test failure
    theseus_shared::qemu_println!("[test did not panic]");
    theseus_shared::qemu_exit_error!();
    
    // Infinite loop - should never be reached due to QEMU exit
    loop {}
}

/// Function designed to panic
/// 
/// This function contains an assertion that will always fail.
/// It's used to test that:
/// 1. assert_eq! macro works correctly
/// 2. Panics are handled properly
/// 3. The panic handler is called
fn should_fail() {
    theseus_shared::qemu_print!("should_panic::should_fail... ");
    
    // This assertion will always fail: 0 != 1
    // This should trigger a panic, which will call our panic handler
    assert_eq!(0, 1);
}
