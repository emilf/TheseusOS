//! Serial driver implementation using the kernel driver framework.
//!
//! This module provides a simple driver that programs the COM1 UART and
//! exposes it through the generic driver interfaces for other subsystems to use.

use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};
use spin::Mutex;
use x86_64::instructions::port::Port;

use crate::drivers::manager::driver_manager;
use crate::drivers::traits::{Device, DeviceClass, Driver};
use crate::interrupts;
use crate::kernel_write_line;
use crate::memory;

pub const LEGACY_COM1_IRQ: u32 = 4;
pub const LEGACY_COM1_BASE: u16 = theseus_shared::constants::io_ports::com1::DATA;
const IOAPIC_REGSEL_OFFSET: u64 = 0x00;
const IOAPIC_WINDOW_OFFSET: u64 = 0x10;
const IOAPIC_REDIRECTION_TABLE_BASE: u32 = 0x10;

struct SerialDriverState {
    port: SerialPort,
    irq_enabled: AtomicBool,
}

#[derive(Clone, Copy)]
struct IoApicInfo {
    address: u64,
    gsi_base: u32,
}

static SERIAL_DRIVER: SerialDriver = SerialDriver;
static SERIAL_STATE: Mutex<Option<SerialDriverState>> = Mutex::new(None);
static IO_APIC_INFO: Mutex<Option<IoApicInfo>> = Mutex::new(None);

pub fn register_serial_driver() {
    driver_manager().lock().register_driver(&SERIAL_DRIVER);
}

pub fn init_serial() {
    if !crate::config::ENABLE_SERIAL_OUTPUT {
        return;
    }
    register_serial_driver();
}

/// Write bytes directly to the serial port, bypassing the driver manager.
/// This is useful from interrupt context to avoid re-entrant locking.
pub fn write_bytes_direct(buf: &[u8]) -> Result<usize, &'static str> {
    let guard = SERIAL_STATE.lock();
    if let Some(state) = guard.as_ref() {
        state.port.write_buffer(buf);
        Ok(buf.len())
    } else {
        Err("serial port not initialized")
    }
}

struct SerialDriver;

#[derive(Clone, Copy)]
struct SerialPort {
    base: u16,
}

impl SerialPort {
    const DEFAULT_BASE: u16 = LEGACY_COM1_BASE;

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

    fn enable_interrupts(&self) {
        unsafe {
            // Enable Received Data Available interrupt.
            let mut int_enable = Port::<u8>::new(self.base + 1);
            int_enable.write(0x01);
            // LSR Trigger Level (FIFO control register)
            let mut fifo_ctrl = Port::<u8>::new(self.base + 2);
            fifo_ctrl.write(0xC7);
            // Modem Control: assert DTR, RTS, OUT1, OUT2 (OUT2 required for interrupts).
            let mut modem_ctrl = Port::<u8>::new(self.base + 4);
            modem_ctrl.write(0x0B);
        }
    }

    fn read_byte(&self) -> Option<u8> {
        unsafe {
            let mut lsr = Port::<u8>::new(self.base + 5);
            if lsr.read() & 0x01 == 0 {
                return None;
            }
            let mut data = Port::<u8>::new(self.base);
            Some(data.read())
        }
    }
}

impl SerialDriver {
    fn init_port(&self, base: u16) -> Result<SerialPort, &'static str> {
        unsafe {
            let mut data = Port::<u8>::new(base);
            let mut int_enable = Port::<u8>::new(base + 1);
            let mut line_ctrl = Port::<u8>::new(base + 3);

            // Enable DLAB to program baud divisor.
            line_ctrl.write(0x80);
            let divisor = theseus_shared::constants::hardware::COM1_BAUD_DIVISOR;
            data.write((divisor & 0xFF) as u8);
            int_enable.write((divisor >> 8) as u8);

            // 8 data bits, no parity, one stop bit; DLAB cleared.
            line_ctrl.write(0x03);
        }

        Ok(SerialPort::new(base))
    }

    fn configure_irq_route(&self, dev: &Device) -> Result<(), &'static str> {
        let info_opt = {
            let guard = IO_APIC_INFO.lock();
            *guard
        };
        let Some(info) = info_opt else {
            return Err("no IO APIC information available");
        };

        let gsi = dev.irq.unwrap_or(LEGACY_COM1_IRQ) as u32;
        if gsi < info.gsi_base {
            return Err("serial GSI below IO APIC base");
        }
        let pin = gsi - info.gsi_base;

        unsafe {
            program_io_apic_entry(
                info.address,
                pin,
                interrupts::SERIAL_RX_VECTOR,
                current_apic_id(),
            );
        }

        Ok(())
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

        if dev.phys_addr.is_none() {
            dev.phys_addr = Some(base as u64);
        }
        if dev.irq.is_none() {
            dev.irq = Some(LEGACY_COM1_IRQ);
        }

        if let Err(err) = self.configure_irq_route(dev) {
            kernel_write_line("[serial] failed to route IO APIC entry");
            kernel_write_line(err);
        } else {
            kernel_write_line("[serial] IO APIC route configured");
        }

        port.enable_interrupts();
        let state = SerialDriverState {
            port,
            irq_enabled: AtomicBool::new(true),
        };
        {
            let mut guard = SERIAL_STATE.lock();
            *guard = Some(state);
        }
        kernel_write_line("[serial] initialized");
        Ok(())
    }

    fn irq_handler(&'static self, _dev: &mut Device, _irq: u32) -> bool {
        if !crate::config::ENABLE_SERIAL_OUTPUT {
            return false;
        }
        let mut handled = false;
        let mut guard = SERIAL_STATE.lock();
        let Some(state) = guard.as_mut() else {
            return false;
        };
        if !state.irq_enabled.load(Ordering::Relaxed) {
            return false;
        }

        while let Some(byte) = state.port.read_byte() {
            handled = true;
            crate::monitor::notify_serial_byte(byte);
        }
        handled
    }

    fn write(&'static self, _dev: &mut Device, buf: &[u8]) -> Result<usize, &'static str> {
        write_bytes_direct(buf)
    }
}

pub fn install_io_apic_info(address: u64, gsi_base: u32) {
    let mut guard = IO_APIC_INFO.lock();
    *guard = Some(IoApicInfo { address, gsi_base });
}

unsafe fn program_io_apic_entry(address: u64, pin: u32, vector: u8, destination_apic_id: u8) {
    let base = memory::phys_to_virt_pa(address);
    let regsel = (base + IOAPIC_REGSEL_OFFSET) as *mut u32;
    let window = (base + IOAPIC_WINDOW_OFFSET) as *mut u32;

    let lower_index = IOAPIC_REDIRECTION_TABLE_BASE + pin * 2;
    let upper_index = lower_index + 1;

    // Configure low dword: vector, delivery mode (fixed), dest mode (physical),
    // polarity (high), trigger (edge), unmask, and clear any stale bits.
    ptr::write_volatile(regsel, lower_index);
    let mut lower = ptr::read_volatile(window);
    // Clear vector bits then set new vector.
    lower &= !0xFF;
    lower |= vector as u32;
    // Delivery mode (bits 8-10) fixed (000).
    lower &= !(0x7 << 8);
    // Destination mode (bit 11) physical (0).
    lower &= !(1 << 11);
    // Polarity (bit 13) high (0).
    lower &= !(1 << 13);
    // Trigger mode (bit 15) edge (0).
    lower &= !(1 << 15);
    // Unmask interrupt (bit 16 = 0).
    lower &= !(1 << 16);
    ptr::write_volatile(window, lower);

    // Configure upper dword: destination APIC ID.
    ptr::write_volatile(regsel, upper_index);
    let mut upper = ptr::read_volatile(window);
    upper &= !(0xFF << 24);
    upper |= (destination_apic_id as u32) << 24;
    ptr::write_volatile(window, upper);
}

fn current_apic_id() -> u8 {
    unsafe {
        let base = interrupts::get_apic_base();
        (interrupts::read_apic_register(base, 0x20) >> 24) as u8
    }
}
