//! QEMU Debug Driver
//! 
//! Uses QEMU's debug output port (0xe9) for simple output. QEMU-specific driver.

use core::arch::asm;
use theseus_shared::constants::io_ports;

/// QEMU Debug driver implementation
pub struct QemuDebugDriver;

impl QemuDebugDriver {
    /// Create a new QEMU debug driver
    pub fn new() -> Self {
        Self
    }
    
    /// Check if this driver is available
    pub fn is_available(&self) -> bool {
        // Always available in QEMU - just writes to I/O port 0xe9
        true
    }
    
    /// Write a single character to QEMU debug port
    fn write_char(&self, ch: u8) {
        unsafe {
            asm!("out dx, al", in("dx") io_ports::QEMU_DEBUG, in("al") ch, options(nomem, nostack, preserves_flags));
        }
    }
    
    /// Write a string to QEMU debug port
    fn write_string(&self, s: &str) {
        for &byte in s.as_bytes() {
            self.write_char(byte);
        }
    }
}

impl crate::drivers::manager::Driver for QemuDebugDriver {
    fn write_line(&self, message: &str) -> bool {
        self.write_string(message);
        self.write_char(b'\r'); // Carriage return
        self.write_char(b'\n'); // Line feed
        true
    }
    
    fn is_available(&self) -> bool {
        self.is_available()
    }
    
    fn name(&self) -> &'static str {
        "QEMU Debug"
    }
}
