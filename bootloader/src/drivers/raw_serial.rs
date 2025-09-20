//! Raw Serial Driver
//! 
//! Direct hardware access to serial ports. Fallback option when other drivers fail.

use x86_64::instructions::port::Port;
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
    
    /// Check if this driver is available
    pub fn is_available(&self) -> bool {
        // Try to initialize and see if it works
        self.init()
    }
    
    /// Write a single character to serial port
    fn write_char(&self, ch: u8) {
        unsafe {
            let mut line_status: Port<u8> = Port::new(com1::LINE_STATUS);
            let mut data: Port<u8> = Port::new(com1::DATA);
            // Wait for THR empty
            while (line_status.read() & 0x20) == 0 {}
            data.write(ch);
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
