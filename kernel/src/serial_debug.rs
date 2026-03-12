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
