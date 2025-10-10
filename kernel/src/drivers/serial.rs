//! COM1 Serial Terminal Driver
//!
//! This module implements a character-device driver for the COM1 serial port.
//! It provides read and write operations for serial communication and integrates
//! with the kernel driver framework.
//!
//! # Hardware
//! - Base I/O port: 0x3F8 (COM1)
//! - Registers: Data, Interrupt Enable, Line Control, Modem Control, Line Status
//! - Configuration: 8-N-1 (8 data bits, no parity, 1 stop bit)
//! - Baud rate: 115200 (standard for QEMU and most modern systems)
//!
//! # Usage
//! The driver automatically registers itself with the driver manager and probes
//! for COM1 hardware. Once bound, it provides read/write operations through
//! the Driver trait interface.

use crate::display::kernel_write_line;
use crate::drivers::traits::{Device, DeviceId, Driver};
use spin::Mutex;
use theseus_shared::constants::io_ports::com1;
use x86_64::instructions::port::Port;

/// COM1 serial port driver state
pub struct SerialDriver {
    /// Protects concurrent access to the serial port
    state: Mutex<SerialState>,
}

/// Internal state for the serial port
struct SerialState {
    /// Whether the port is initialized
    initialized: bool,
    /// Data register port
    data: Port<u8>,
    /// Interrupt enable register port
    int_enable: Port<u8>,
    /// Line control register port
    line_ctrl: Port<u8>,
    /// Modem control register port
    modem_ctrl: Port<u8>,
    /// Line status register port
    line_status: Port<u8>,
}

impl SerialState {
    /// Create a new uninitialized serial state
    const fn new() -> Self {
        Self {
            initialized: false,
            data: Port::new(com1::DATA),
            int_enable: Port::new(com1::INT_ENABLE),
            line_ctrl: Port::new(com1::LINE_CTRL),
            modem_ctrl: Port::new(com1::MODEM_CTRL),
            line_status: Port::new(com1::LINE_STATUS),
        }
    }

    /// Initialize the serial port hardware
    ///
    /// # Configuration
    /// - 8 data bits, no parity, 1 stop bit (8-N-1)
    /// - 115200 baud rate
    /// - Interrupts disabled
    /// - DTR and RTS signals enabled
    ///
    /// # Returns
    /// `true` if initialization succeeded, `false` otherwise
    fn init(&mut self) -> bool {
        if self.initialized {
            return true;
        }

        unsafe {
            // Disable interrupts
            self.int_enable.write(0x00);

            // Enable DLAB (set baud rate divisor)
            self.line_ctrl.write(0x80);

            // Set divisor to 1 (115200 baud)
            self.data.write(0x01); // Divisor low byte
            self.int_enable.write(0x00); // Divisor high byte

            // 8 bits, no parity, one stop bit (8-N-1)
            self.line_ctrl.write(0x03);

            // Enable FIFO, clear them, with 14-byte threshold
            self.int_enable.write(0xC7);

            // IRQs enabled, RTS/DSR set
            self.modem_ctrl.write(0x0B);

            // Test serial chip (loopback mode)
            self.modem_ctrl.write(0x1E);
            self.data.write(0xAE);

            // Check if serial is faulty (i.e., not same byte looped back)
            if self.data.read() != 0xAE {
                return false;
            }

            // If serial is not faulty, set it in normal operation mode
            // (not-loopback with IRQs enabled and OUT#1 and OUT#2 bits enabled)
            self.modem_ctrl.write(0x0F);
        }

        self.initialized = true;
        true
    }

    /// Check if the transmit buffer is empty
    fn is_transmit_empty(&mut self) -> bool {
        unsafe { (self.line_status.read() & 0x20) != 0 }
    }

    /// Write a single byte to the serial port
    ///
    /// This function waits for the transmit buffer to be empty before writing
    fn write_byte(&mut self, byte: u8) {
        unsafe {
            // Wait for transmit buffer to be empty
            while !self.is_transmit_empty() {
                core::hint::spin_loop();
            }
            self.data.write(byte);
        }
    }

    /// Check if data is available to read
    fn data_available(&mut self) -> bool {
        unsafe { (self.line_status.read() & 0x01) != 0 }
    }

    /// Read a single byte from the serial port
    ///
    /// Returns `None` if no data is available
    fn read_byte(&mut self) -> Option<u8> {
        if self.data_available() {
            unsafe { Some(self.data.read()) }
        } else {
            None
        }
    }
}

/// Global serial driver instance
static SERIAL_DRIVER: SerialDriver = SerialDriver::new();

impl SerialDriver {
    /// Create a new serial driver instance
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(SerialState::new()),
        }
    }

    /// Write a string to the serial port
    pub fn write_str(&'static self, s: &str) {
        let mut state = self.state.lock();
        if !state.initialized {
            return;
        }

        for byte in s.as_bytes() {
            // Convert \n to \r\n for proper terminal display
            if *byte == b'\n' {
                state.write_byte(b'\r');
            }
            state.write_byte(*byte);
        }
    }

    /// Write a buffer to the serial port
    pub fn write_bytes(&'static self, buf: &[u8]) -> usize {
        let mut state = self.state.lock();
        if !state.initialized {
            return 0;
        }

        for &byte in buf.iter() {
            state.write_byte(byte);
        }
        buf.len()
    }

    /// Read bytes from the serial port
    ///
    /// Returns the number of bytes actually read (may be 0 if no data available)
    pub fn read_bytes(&'static self, buf: &mut [u8]) -> usize {
        let mut state = self.state.lock();
        if !state.initialized {
            return 0;
        }

        let mut count = 0;
        for slot in buf.iter_mut() {
            if let Some(byte) = state.read_byte() {
                *slot = byte;
                count += 1;
            } else {
                break;
            }
        }
        count
    }
}

impl Driver for SerialDriver {
    fn on_register(&'static self) -> Result<bool, &'static str> {
        kernel_write_line("[serial] COM1 driver registering");
        Ok(true) // Probe immediately
    }

    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        // Only bind to COM1 serial devices
        match &dev.id {
            DeviceId::Raw(name) if *name == "com1" || *name == "uart" => {
                kernel_write_line("[serial] probing COM1 device");

                // Initialize the serial port
                let mut state = self.state.lock();
                if !state.init() {
                    return Err("Failed to initialize COM1");
                }
                drop(state);

                // Mark device as bound by storing a magic value
                dev.driver_data = Some(0xC041); // "COM1" in hex-ish

                kernel_write_line("[serial] COM1 initialized successfully");
                kernel_write_line("[serial] COM1 ready for I/O");

                // Test the serial port
                self.write_str("COM1 Serial Driver Initialized\r\n");

                Ok(())
            }
            _ => Err("Not a COM1 device"),
        }
    }

    fn remove(&'static self, dev: &mut Device) {
        if dev.driver_data == Some(0xC041) {
            kernel_write_line("[serial] removing COM1 driver");
            dev.driver_data = None;
        }
    }

    fn write(&'static self, dev: &mut Device, buf: &[u8]) -> Result<usize, &'static str> {
        if dev.driver_data != Some(0xC041) {
            return Err("Device not bound");
        }

        Ok(self.write_bytes(buf))
    }

    fn read(&'static self, dev: &mut Device, buf: &mut [u8]) -> Result<usize, &'static str> {
        if dev.driver_data != Some(0xC041) {
            return Err("Device not bound");
        }

        Ok(self.read_bytes(buf))
    }

    fn irq_handler(&'static self, dev: &mut Device, irq: u32) -> bool {
        // COM1 typically uses IRQ 4
        if dev.driver_data == Some(0xC041) && irq == 4 {
            // Handle serial interrupt (data received, transmit buffer empty, etc.)
            // For now, we just acknowledge we could handle it
            true
        } else {
            false
        }
    }
}

/// Register the COM1 serial driver with the driver manager
pub fn register_serial_driver() {
    use crate::drivers::manager::driver_manager;

    kernel_write_line("[serial] registering COM1 driver");
    driver_manager().lock().register_driver(&SERIAL_DRIVER);
}

/// Create and register a COM1 device for the driver to bind to
pub fn register_com1_device() {
    use crate::drivers::manager::driver_manager;

    kernel_write_line("[serial] registering COM1 device");

    let mut device = Device::new(DeviceId::Raw("com1"));
    device.phys_addr = Some(com1::DATA as u64); // Base I/O port address
    device.irq = Some(4); // COM1 IRQ

    driver_manager().lock().add_device(device);
}

