//! # Serial Debug Utilities
//!
//! This module provides simple serial debugging tools for testing and validating
//! the serial driver functionality. These are particularly useful during early
//! kernel development when more sophisticated debugging infrastructure isn't
//! yet available.
//!
//! ## Reverse Echo Session
//!
//! The main feature is a "reverse echo" test that reads lines from the serial
//! port and echoes them back in reverse order. This validates:
//!
//! - Serial reception (IRQ-based input)
//! - Serial transmission (polled output)
//! - Bidirectional communication
//! - Line buffering and termination
//!
//! ### Example Session
//!
//! ```text
//! User types: hello world
//! System echoes: dlrow olleh
//!
//! User types: testing 123
//! System echoes: 321 gnitset
//! ```
//!
//! ## Usage
//!
//! Enable the reverse echo session by setting `config::RUN_POST_BOOT_SERIAL_REVERSE_ECHO`
//! to `true`. This will block kernel execution indefinitely, so it should only be
//! used for testing.
//!
//! ## Implementation Notes
//!
//! The reverse echo session is intentionally simple:
//! - No command processing (unlike the full monitor)
//! - Blocks on serial I/O (uses `hlt` instruction when no data available)
//! - Fixed line buffer size (256 bytes)
//! - Reverses entire line before sending
//!
//! This simplicity makes it ideal for verifying basic serial functionality before
//! bringing up more complex subsystems.

use alloc::vec::Vec;

use crate::drivers::manager::driver_manager;
use crate::drivers::traits::DeviceClass;

const MAX_LINE: usize = 256;

/// Run the reverse echo session
///
/// This function enters an infinite loop that:
/// 1. Reads one byte at a time from the serial port
/// 2. Accumulates bytes into a line buffer
/// 3. When Enter is pressed, reverses the line and echoes it back
/// 4. Repeats forever
///
/// The function uses the driver manager's class-based I/O interface to read
/// from any registered Serial class device. When no data is available, it
/// halts the CPU to save power (the next interrupt will wake it).
///
/// # Control Characters
///
/// - **Enter (CR/LF)**: Complete the line, reverse it, and send it back
/// - **Other printable characters**: Added to buffer and echoed
/// - **Non-printable**: Ignored (no special handling)
///
/// # Panics
///
/// This function never returns, so calling code should be aware that nothing
/// after this call will execute. It's designed for testing only.
///
/// # Example
///
/// ```no_run
/// if config::RUN_POST_BOOT_SERIAL_REVERSE_ECHO {
///     run_reverse_echo_session();  // Never returns
/// }
/// ```
pub fn run_reverse_echo_session() -> ! {
    crate::display::kernel_write_line("[serial] reverse echo session active");

    let mut line = Vec::with_capacity(MAX_LINE);
    let mut byte_buf = [0u8; 1];

    loop {
        let read = driver_manager()
            .lock()
            .read_class(DeviceClass::Serial, &mut byte_buf)
            .unwrap_or(0);

        if read == 0 {
            // No data available yet, yield until the next interrupt.
            x86_64::instructions::hlt();
            continue;
        }

        let byte = byte_buf[0];

        match byte {
            b'\r' | b'\n' => {
                if !line.is_empty() {
                    line.reverse();
                    let _ = driver_manager()
                        .lock()
                        .write_class(DeviceClass::Serial, &line);
                    line.clear();
                }
                let _ = driver_manager()
                    .lock()
                    .write_class(DeviceClass::Serial, b"\r\n");
            }
            _ => {
                if line.len() < MAX_LINE {
                    line.push(byte);
                    let _ = driver_manager()
                        .lock()
                        .write_class(DeviceClass::Serial, &[byte]);
                }
            }
        }
    }
}
