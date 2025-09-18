//! QEMU Exit Utilities
//! 
//! Provides functions for exiting QEMU with proper debug output.
//! This is separate from the driver system to ensure it works even during panics.

use theseus_shared::constants::exit_codes;

/// Exit QEMU with a message and exit code
/// 
/// This function writes a message directly to the QEMU debug port (0xe9)
/// and then exits QEMU with the specified exit code. This is useful for
/// both normal completion and panic handling.
/// 
/// # Parameters
/// 
/// * `message` - Message to write to QEMU debug output before exiting
/// * `exit_code` - Exit code to send to QEMU (typically QEMU_SUCCESS or QEMU_ERROR)
/// 
/// # Safety
/// 
/// This function performs direct I/O port operations and should only be called
/// when we're running under QEMU. It will cause the guest to exit immediately.
pub unsafe fn exit_qemu_with_message(message: &str, exit_code: u8) {
    theseus_shared::qemu_println!(message);
    theseus_shared::qemu_exit!(exit_code);
}


/// Exit QEMU with error and a message
#[allow(dead_code)] // Intended for future use
pub unsafe fn exit_qemu_error(message: &str) {
    exit_qemu_with_message(message, exit_codes::QEMU_ERROR);
}

/// Exit QEMU due to a panic with panic information
/// 
/// This can be called from a custom panic handler to provide useful
/// debugging information when the application panics.
#[allow(dead_code)] // Intended for future use
pub unsafe fn exit_qemu_on_panic(panic_info: &core::panic::PanicInfo) {
    // Create a simple panic message
    let message = alloc::format!("PANIC: {:?}", panic_info.message());
    
    exit_qemu_error(&message);
}
