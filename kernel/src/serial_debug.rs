//! Temporary serial debugging utilities.
//!
//! Provides a simple "reverse echo" session that can be enabled via
//! `config::RUN_POST_BOOT_SERIAL_REVERSE_ECHO`. This is useful when bringing
//! up early serial paths before the full monitor is ready.

use alloc::vec::Vec;

use crate::drivers::manager::driver_manager;
use crate::drivers::traits::DeviceClass;

const MAX_LINE: usize = 256;

/// Block forever, reading characters from the serial port and writing each
/// completed line back in reverse order.
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
