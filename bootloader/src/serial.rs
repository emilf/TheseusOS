//! # Bootloader Serial Utilities
//!
//! This module provides simple serial output helpers for the UEFI bootloader phase.
//! These functions use UEFI's built-in Serial I/O Protocol, which is available before
//! ExitBootServices() is called.
//!
//! ## UEFI Serial I/O Protocol
//!
//! UEFI firmware provides a standardized `Serial` protocol that abstracts the underlying
//! hardware. This works across different platforms (physical hardware, QEMU, etc.) without
//! needing to know specific port addresses or hardware details.
//!
//! Key features:
//! - Hardware abstraction (no need to program UART registers directly)
//! - Works on both physical hardware and virtual machines
//! - Available during early boot before kernel drivers are loaded
//! - Disappears after ExitBootServices() (kernel must use its own drivers)
//!
//! ## Usage
//!
//! These functions are typically used for early boot logging:
//!
//! ```rust,no_run
//! serial_write_line(serial_handle, "Bootloader starting...");
//! serial_write_line(serial_handle, "Loading kernel...");
//! ```
//!
//! ## Transition to Kernel
//!
//! After the bootloader calls ExitBootServices():
//! 1. UEFI protocols (including Serial) become unavailable
//! 2. The kernel must initialize its own hardware drivers
//! 3. The kernel serial driver (in `kernel/src/drivers/serial.rs`) takes over
//!
//! This is why we have two separate serial implementations: one for UEFI (this module)
//! and one for the kernel (the 16550 driver).

use uefi::{boot, prelude::*, proto::console::serial::Serial};

/// Write raw data to serial port using UEFI Serial I/O Protocol
///
/// This function attempts to write data to the serial port by:
/// 1. Checking if a serial handle was provided
/// 2. Opening the UEFI Serial protocol exclusively
/// 3. Writing the data via the protocol's write() method
///
/// Failures are silently ignored (this is early boot code where error handling
/// is limited). If the serial port isn't available or write fails, execution
/// continues normally.
///
/// # Arguments
///
/// * `serial_handle` - Optional UEFI handle to a Serial I/O Protocol instance
/// * `data` - Byte slice to write to serial port
///
/// # Example
///
/// ```rust,no_run
/// serial_write(serial_handle, b"Hello, UEFI!\r\n");
/// ```
pub fn serial_write(serial_handle: Option<Handle>, data: &[u8]) {
    if let Some(handle) = serial_handle {
        // Try to open the Serial protocol exclusively
        // exclusive = no other code can use it while we have it open
        if let Ok(mut serial) = boot::open_protocol_exclusive::<Serial>(handle) {
            // Write data (ignore errors - best effort during early boot)
            let _ = serial.write(data);
        }
    }
}

/// Write a line of text to serial port (with CRLF termination)
///
/// This is a convenience wrapper around `serial_write()` that:
/// 1. Converts the string to bytes
/// 2. Writes the string
/// 3. Appends CR+LF (\\r\\n) for proper terminal line breaks
///
/// The CRLF sequence ensures the output displays correctly on most terminal
/// emulators, which expect both carriage return and line feed for new lines.
///
/// # Arguments
///
/// * `serial_handle` - Optional UEFI handle to a Serial I/O Protocol instance
/// * `line` - String slice to write (without trailing newline)
///
/// # Example
///
/// ```rust,no_run
/// serial_write_line(serial_handle, "Bootloader initialized");
/// serial_write_line(serial_handle, "Loading kernel...");
/// ```
pub fn serial_write_line(serial_handle: Option<Handle>, line: &str) {
    serial_write(serial_handle, line.as_bytes());
    serial_write(serial_handle, b"\r\n");
}
