//! Module: bootloader::drivers::raw_serial
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//!
//! INVARIANTS:
//! - This module provides a raw UART fallback output path for the bootloader environment.
//! - It exists as a pragmatic fallback, not as the preferred primary observability path.
//!
//! SAFETY:
//! - Direct serial-port programming here assumes compatible UART/port layout and can be wrong on real hardware or firmware setups.
//! - Re-initializing hardware just to test availability is a tradeoff, not a proof of correctness.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! Raw serial driver.
//!
//! Direct hardware access to serial ports. Fallback option when other drivers fail.

use theseus_shared::constants::{hardware, io_ports::com1};
use x86_64::instructions::port::Port;

/// Bootloader fallback driver that pokes a COM1-style UART directly.
pub struct RawSerialDriver;

impl RawSerialDriver {
    /// Create the raw serial fallback driver.
    pub fn new() -> Self {
        Self
    }

    /// Initialize the serial port.
    ///
    /// This is a pragmatic fallback initialization sequence for a COM1-style UART.
    pub fn init(&self) -> bool {
        unsafe {
            // Configure COM1 using Port IO abstraction
            let mut line_ctrl: Port<u8> = Port::new(com1::LINE_CTRL);
            let mut int_enable: Port<u8> = Port::new(com1::INT_ENABLE);
            let mut data: Port<u8> = Port::new(com1::DATA);
            let mut modem_ctrl: Port<u8> = Port::new(com1::MODEM_CTRL);

            line_ctrl.write(0x03u8); // 8-N-1
            int_enable.write(0x00u8); // disable interrupts
            data.write(hardware::COM1_BAUD_DIVISOR as u8); // baud divisor low
            int_enable.write(0x00u8); // keep disabled
            modem_ctrl.write(0x03u8); // DTR|RTS
        }
        true
    }

    /// Check if this driver appears available.
    ///
    /// This is intentionally a pragmatic probe: it reinitializes the UART and assumes
    /// success if that sequence does not obviously fail.
    pub fn is_available(&self) -> bool {
        // Try to initialize and see if it works
        self.init()
    }

    /// Write one byte once the transmitter-holding register is ready.
    fn write_char(&self, ch: u8) {
        unsafe {
            let mut line_status: Port<u8> = Port::new(com1::LINE_STATUS);
            let mut data: Port<u8> = Port::new(com1::DATA);
            // Wait for THR empty
            while (line_status.read() & 0x20) == 0 {}
            data.write(ch);
        }
    }

    /// Write a string through the raw UART fallback path.
    fn write_string(&self, s: &str) {
        for &byte in s.as_bytes() {
            self.write_char(byte);
        }
    }
}

impl crate::drivers::manager::Driver for RawSerialDriver {
    fn write_line(&self, message: &str) -> bool {
        if !self.is_available() {
            return false;
        }

        self.write_string(message);
        self.write_char(b'\r'); // Carriage return
        self.write_char(b'\n'); // Line feed
        true
    }

    fn is_available(&self) -> bool {
        self.is_available()
    }

    fn name(&self) -> &'static str {
        "Raw Serial"
    }
}
