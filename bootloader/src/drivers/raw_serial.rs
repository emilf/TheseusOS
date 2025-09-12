//! Raw Serial Driver
//! 
//! Direct hardware access to serial ports. Fallback option when other drivers fail.

use core::arch::asm;
use theseus_shared::constants::{io_ports::com1, hardware};

/// Raw Serial driver implementation
pub struct RawSerialDriver;

impl RawSerialDriver {
    /// Create a new raw serial driver
    pub fn new() -> Self {
        Self
    }
    
    /// Initialize the serial port
    pub fn init(&self) -> bool {
        unsafe {
            // Configure COM1 port
            // Line Control Register: 8 data bits, no parity, 1 stop bit
            asm!("out dx, al", in("dx") com1::LINE_CTRL, in("al") 0x03u8, options(nomem, nostack, preserves_flags));
            
            // Divisor Latch Low Byte: 115200 baud
            asm!("out dx, al", in("dx") com1::INT_ENABLE, in("al") 0x00u8, options(nomem, nostack, preserves_flags));
            asm!("out dx, al", in("dx") com1::DATA, in("al") hardware::COM1_BAUD_DIVISOR as u8, options(nomem, nostack, preserves_flags));
            
            // Interrupt Enable Register: disable interrupts
            asm!("out dx, al", in("dx") com1::INT_ENABLE, in("al") 0x00u8, options(nomem, nostack, preserves_flags));
            
            // Modem Control Register: set DTR and RTS
            asm!("out dx, al", in("dx") com1::MODEM_CTRL, in("al") 0x03u8, options(nomem, nostack, preserves_flags));
        }
        true
    }
    
    /// Check if this driver is available
    pub fn is_available(&self) -> bool {
        // Try to initialize and see if it works
        self.init()
    }
    
    /// Write a single character to serial port
    fn write_char(&self, ch: u8) {
        unsafe {
            // Wait for transmitter holding register to be empty
            loop {
                let status: u8;
                asm!("in al, dx", out("al") status, in("dx") com1::LINE_STATUS, options(nomem, nostack, preserves_flags));
                if (status & 0x20) != 0 {
                    break;
                }
            }
            
            // Write character to data register
            asm!("out dx, al", in("dx") com1::DATA, in("al") ch, options(nomem, nostack, preserves_flags));
        }
    }
    
    /// Write a string to serial port
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
