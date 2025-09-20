//! Panic handling module
//! 
//! This module provides the kernel's panic handler and related panic utilities.
//! When a panic occurs in the kernel, this handler is responsible for safely
//! shutting down the system and providing debug information.

/// Panic handler for kernel
/// 
/// This function is called when a panic occurs in the kernel. It attempts to
/// output panic information to the QEMU debug port and then exits QEMU with
/// an error code.
/// 
/// # Parameters
/// 
/// * `_panic_info` - Information about the panic (currently unused)
/// 
/// # Safety
/// 
/// This function is marked as `#[panic_handler]` and will be called by the
/// Rust runtime when a panic occurs. It must not return.
#[cfg(not(test))]
#[panic_handler]
pub fn panic_handler(_panic_info: &core::panic::PanicInfo) -> ! {
    // Output panic information using direct QEMU debug port
    // Note: We avoid using the allocator during panic to prevent recursive panics

    let message = "KERNEL PANIC: Panic occurred";
    
    theseus_shared::qemu_println!(message);
    theseus_shared::qemu_exit_error!();
    
    loop {}
}
