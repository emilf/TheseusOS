//! Panic handling module
//! 
//! This module provides the kernel's panic handler and related panic utilities.

/// Panic handler for kernel
#[cfg(not(test))]
#[panic_handler]
pub fn panic_handler(_panic_info: &core::panic::PanicInfo) -> ! {
    // Try to output panic information
    // TODO: Do it without using allocator

    let message = "KERNEL PANIC: Panic occurred";
    
    theseus_shared::qemu_println!(message);
    theseus_shared::qemu_exit_error!();
    
    loop {}
}
