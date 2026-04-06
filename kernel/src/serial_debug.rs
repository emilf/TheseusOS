//! Module: serial_debug
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - This module provides intentionally simple post-boot serial-debug workflows separate from the full monitor.
//! - The reverse-echo session is a testing aid for validating end-to-end serial receive/transmit behavior.
//! - When enabled, this path intentionally takes over execution flow and does not attempt to coexist with richer interactive tooling.
//!
//! SAFETY:
//! - This module is for controlled debugging sessions; enabling it changes runtime behavior and can block normal kernel progress indefinitely.
//! - Serial-driver and driver-manager assumptions used here must remain simple enough to be useful during bring-up, not just after the whole system is healthy.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//!
//! Simple post-boot serial-debug helpers.
//!
//! This module contains intentionally narrow serial-debug workflows used during
//! bring-up and driver validation.

use alloc::vec::Vec;

use crate::drivers::manager::driver_manager;
use crate::drivers::traits::DeviceClass;
use crate::log_info;

/// Writes a formatted string directly to the serial port without going through
/// kernel logging. This is intended for low-level debugging output where
/// performance and direct visibility are important.
///
/// # Format
/// ```ignore
/// serial_print!("User: {}", user);
/// serial_print!("Count: {}", num);
/// serial_print!("Binary: {:08b}", val);
/// ```
///
/// # Safety
/// This function bypasses the normal kernel logging infrastructure and writes
/// directly to the serial driver. Ensure the serial driver is active and
/// properly configured before using this macro.
///
/// # Examples
/// ```ignore
/// let name = "Alice";
/// serial_print!("Hello {}", name);
/// serial_println!("Hello {}", name);
/// ```
#[macro_export]
macro_rules! serial_print {
    ($($arg:tt)*) => {{
        let _fmt = alloc::format!($($arg)*);
        let bytes = _fmt.as_bytes();
        let _ = $crate::drivers::manager::driver_manager().lock().write_class($crate::drivers::traits::DeviceClass::Serial, bytes);
    }};
}

/// Like `serial_print!` but appends a newline character (`\r\n`)
/// after the formatted output, ensuring proper line termination for
/// terminal/console display.
///
/// # Safety
/// Same as `serial_print!` - this bypasses kernel logging.
///
/// # Examples
/// ```ignore
/// serial_println!("Starting boot sequence...");
/// serial_println!("Error: {}", error_msg);
/// ```
#[macro_export]
macro_rules! serial_println {
    ($($arg:tt)*) => {{
        let _fmt = alloc::format!($($arg)*);
        let bytes = _fmt.as_bytes();
        let _ = $crate::drivers::manager::driver_manager().lock().write_class($crate::drivers::traits::DeviceClass::Serial, bytes);
        let _ = $crate::drivers::manager::driver_manager().lock().write_class($crate::drivers::traits::DeviceClass::Serial, b"\r\n");
    }};
}

const MAX_LINE: usize = 256;



/// Run the reverse-echo serial-debug session.
///
/// This takes over execution with a tiny serial loop and never returns.
pub fn run_reverse_echo_session() -> ! {
    log_info!("Serial reverse echo session active");

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
