//! Serial driver implementation using the kernel driver framework.
//!
//! This module provides a simple driver that programs the COM1 UART and
//! exposes it through the generic driver interfaces for other subsystems to use.

use spin::Mutex;
use x86_64::instructions::port::Port;

use crate::drivers::manager::driver_manager;
use crate::drivers::traits::{Device, DeviceClass, Driver};
use crate::kernel_write_line;

static SERIAL_DRIVER: SerialDriver = SerialDriver;

static SERIAL_STATE: Mutex<Option<SerialPort>> = Mutex::new(None);

pub fn register_serial_driver() {
    driver_manager().lock().register_driver(&SERIAL_DRIVER);
}

pub fn init_serial() {
    if !crate::config::ENABLE_SERIAL_OUTPUT {
        return;
    }
    register_serial_driver();
}

struct SerialDriver;

struct SerialPort {
    base: u16,
}

impl SerialPort {
    const DEFAULT_BASE: u16 = theseus_shared::constants::io_ports::com1::DATA;

    fn new(base: u16) -> Self {
        Self { base }
    }

    fn write_byte(&self, byte: u8) {
        unsafe {
            let mut port = Port::new(self.base);
            port.write(byte);
        }
    }

    fn line_status(&self) -> u8 {
        unsafe {
            let mut port = Port::new(self.base + 5);
            port.read()
        }
    }

    fn write_buffer(&self, buf: &[u8]) {
        for &b in buf {
            while self.line_status() & 0x20 == 0 {}
            self.write_byte(if b == b'\n' { b'\r' } else { b });
            if b == b'\n' {
                while self.line_status() & 0x20 == 0 {}
                self.write_byte(b'\n');
            }
        }
    }
}

impl SerialDriver {
    fn init_port(&self, base: u16) -> Result<SerialPort, &'static str> {
        unsafe {
            let mut data = Port::<u8>::new(base);
            let mut int_enable = Port::<u8>::new(base + 1);
            let mut line_ctrl = Port::<u8>::new(base + 3);
            let mut modem_ctrl = Port::<u8>::new(base + 4);
            let mut fifo_ctrl = Port::<u8>::new(base + 2);

            line_ctrl.write(0x80);
            let divisor = theseus_shared::constants::hardware::COM1_BAUD_DIVISOR;
            data.write((divisor & 0xFF) as u8);
            int_enable.write((divisor >> 8) as u8);

            line_ctrl.write(0x03);

            fifo_ctrl.write(0xC7);

            modem_ctrl.write(0x0B);
        }

        Ok(SerialPort::new(base))
    }
}

impl Driver for SerialDriver {
    fn supported_classes(&self) -> &'static [DeviceClass] {
        &[DeviceClass::Serial]
    }

    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        match dev.class {
            DeviceClass::Serial => Ok(()),
            _ => Err("not serial"),
        }
    }

    fn init(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        if !crate::config::ENABLE_SERIAL_OUTPUT {
            return Err("serial output disabled");
        }
        let base = dev
            .phys_addr
            .map(|addr| addr as u16)
            .unwrap_or(SerialPort::DEFAULT_BASE);
        let port = self.init_port(base)?;
        {
            let mut guard = SERIAL_STATE.lock();
            *guard = Some(port);
        }
        kernel_write_line("[serial] initialized");
        Ok(())
    }

    fn write(&'static self, _dev: &mut Device, buf: &[u8]) -> Result<usize, &'static str> {
        let guard = SERIAL_STATE.lock();
        if let Some(port) = guard.as_ref() {
            port.write_buffer(buf);
            Ok(buf.len())
        } else {
            Err("serial port not initialized")
        }
    }
}
