//! # Driver Manager Implementation
//!
//! The driver manager is the central coordinator for all device drivers in the kernel.
//! It maintains registries of available drivers and discovered devices, and handles
//! the binding process between them.
//!
//! ## Responsibilities
//!
//! ### Driver Registry
//! - Maintains a list of all registered drivers
//! - Allows drivers to register at any time (typically during kernel init)
//! - Supports deferred probing (drivers can register before devices exist)
//!
//! ### Device Registry
//! - Maintains a list of all discovered devices
//! - Each device has metadata (class, IRQ, MMIO address, etc.)
//! - Devices can be added dynamically (hot-plug, ACPI enumeration, etc.)
//!
//! ### Binding/Probing
//! - When a device is registered, runs probe logic
//! - Asks each driver "do you support this device?"
//! - First driver that accepts binds to the device
//! - Device retains reference to its bound driver
//!
//! ### IRQ Dispatch
//! - Routes hardware interrupts to appropriate device drivers
//! - Searches devices by IRQ number
//! - Calls driver's `irq_handler()` method
//! - Returns whether the IRQ was handled
//!
//! ### Class-Based I/O
//! - Provides high-level read/write operations by device class
//! - Example: "write to any Serial device" without knowing which one
//! - Useful for system logging, debug output, etc.
//!
//! ## Probe Algorithm
//!
//! ```text
//! When device is registered:
//!   For each driver:
//!     If driver.supports_class(device.class):
//!       If driver.probe(device) succeeds:
//!         Call driver.init(device)
//!         Mark device as bound
//!         STOP (don't try other drivers)
//! ```
//!
//! ## Thread Safety
//!
//! The driver manager is protected by a global spin mutex (`DRIVER_MANAGER`).
//! All operations acquire this lock, so:
//! - Only one thread can modify registries at a time
//! - IRQ handlers can safely call into the manager
//! - No race conditions between driver registration and device enumeration
//!
//! ## Example Usage
//!
//! ```rust,no_run
//! // Register a driver
//! driver_manager().lock().register_driver(&MY_DRIVER);
//!
//! // Register a device
//! let device = Device::new(DeviceId::Class(DeviceClass::Serial));
//! driver_manager().lock().add_device(device);
//!
//! // Write to any serial device
//! driver_manager().lock().write_class(DeviceClass::Serial, b"Hello\n");
//!
//! // Handle an IRQ
//! let handled = driver_manager().lock().handle_irq(4);  // IRQ 4 = COM1
//! ```
//!
//! ## Design Rationale
//!
//! This design is deliberately simple for the MVP:
//! - No hot removal support (could be added later)
//! - No driver dependencies (drivers must self-order)
//! - No priority system (first-match wins)
//! - No async operations (all I/O is synchronous)
//!
//! These simplifications make the code easier to understand and debug, while
//! still providing enough functionality for a working kernel.

use alloc::vec::Vec;
use spin::Mutex;

use crate::display::kernel_write_line;

use super::traits::{Device, DeviceClass, DeviceId, Driver};

/// Global driver manager instance used across the kernel
static DRIVER_MANAGER: Mutex<DriverManager> = Mutex::new(DriverManager::new());

/// Access the global driver manager
pub fn driver_manager() -> &'static Mutex<DriverManager> {
    &DRIVER_MANAGER
}

/// Core driver manager state
pub struct DriverManager {
    drivers: Vec<&'static dyn Driver>,
    devices: Vec<Device>,
}

impl DriverManager {
    /// Create a new manager with empty registries
    pub const fn new() -> Self {
        Self {
            drivers: Vec::new(),
            devices: Vec::new(),
        }
    }

    /// Register a driver with the system. If `on_register` returns true, probe
    /// the driver immediately against existing devices.
    pub fn register_driver(&mut self, drv: &'static dyn Driver) {
        match drv.on_register() {
            Ok(true) => {
                kernel_write_line("[driver] registering driver");
                self.drivers.push(drv);
                self.probe_pending_devices();
            }
            Ok(false) => {
                kernel_write_line("[driver] driver registered (deferred)");
                self.drivers.push(drv);
            }
            Err(e) => {
                kernel_write_line("[driver] driver registration failed: ");
                kernel_write_line(e);
            }
        }
    }

    /// Add a discovered device and run probe logic
    pub fn add_device(&mut self, device: Device) {
        kernel_write_line("[driver] discovered device");
        self.devices.push(device);
        self.probe_pending_devices();
    }

    fn probe_pending_devices(&mut self) {
        for dev in self.devices.iter_mut() {
            if dev.driver_data.is_some() {
                continue;
            }
            for drv in self.drivers.iter() {
                if !drv.supports_class(dev.class) {
                    continue;
                }
                match drv.probe(dev) {
                    Ok(()) => {
                        if let Err(e) = drv.init(dev) {
                            kernel_write_line("[driver] device init failed:");
                            kernel_write_line(e);
                            continue;
                        }
                        kernel_write_line("[driver] device bound successfully");
                        break;
                    }
                    Err(_) => {
                        // Try next driver
                    }
                }
            }
        }
    }

    /// Simple IRQ dispatch helper. Returns true if handled.
    pub fn handle_irq(&mut self, irq: u32) -> bool {
        for dev in self.devices.iter_mut() {
            if let Some(dev_irq) = dev.irq {
                if dev_irq == irq {
                    for drv in self.drivers.iter() {
                        if drv.irq_handler(dev, irq) {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Iterate devices (debug/testing)
    pub fn devices(&self) -> &[Device] {
        &self.devices
    }

    pub fn write_class(&mut self, class: DeviceClass, buf: &[u8]) -> Result<usize, &'static str> {
        for dev in self.devices.iter_mut() {
            if dev.class != class {
                continue;
            }
            for drv in self.drivers.iter() {
                if !drv.supports_class(class) {
                    continue;
                }
                if let Ok(written) = drv.write(dev, buf) {
                    return Ok(written);
                }
            }
        }
        Err("no driver able to write to class")
    }

    pub fn read_class(
        &mut self,
        class: DeviceClass,
        buf: &mut [u8],
    ) -> Result<usize, &'static str> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut last_err: Option<&'static str> = None;
        for dev in self.devices.iter_mut() {
            if dev.class != class {
                continue;
            }
            if dev.driver_data.is_none() {
                continue;
            }
            for drv in self.drivers.iter() {
                if !drv.supports_class(class) {
                    continue;
                }
                match drv.read(dev, buf) {
                    Ok(read) => return Ok(read),
                    Err(e) => last_err = Some(e),
                }
            }
        }
        if let Some(err) = last_err {
            Err(err)
        } else {
            Ok(0)
        }
    }

    /// Write to a specific device by identifier.
    pub fn write_to_device(&mut self, id: &DeviceId, buf: &[u8]) -> Result<usize, &'static str> {
        let mut last_err: Option<&'static str> = None;
        for dev in self.devices.iter_mut() {
            if &dev.id != id {
                continue;
            }
            if dev.driver_data.is_none() {
                return Err("device not bound to driver");
            }
            for drv in self.drivers.iter() {
                if !drv.supports_class(dev.class) {
                    continue;
                }
                match drv.write(dev, buf) {
                    Ok(written) => return Ok(written),
                    Err(e) => last_err = Some(e),
                }
            }
            return Err(last_err.unwrap_or("no driver able to write to device"));
        }
        Err("device not found")
    }

    /// Read from a specific device by identifier.
    pub fn read_from_device(
        &mut self,
        id: &DeviceId,
        buf: &mut [u8],
    ) -> Result<usize, &'static str> {
        if buf.is_empty() {
            return Ok(0);
        }

        let mut last_err: Option<&'static str> = None;
        for dev in self.devices.iter_mut() {
            if &dev.id != id {
                continue;
            }
            if dev.driver_data.is_none() {
                return Err("device not bound to driver");
            }
            for drv in self.drivers.iter() {
                if !drv.supports_class(dev.class) {
                    continue;
                }
                match drv.read(dev, buf) {
                    Ok(read) => return Ok(read),
                    Err(e) => last_err = Some(e),
                }
            }
            return Err(last_err.unwrap_or("no driver able to read from device"));
        }
        Err("device not found")
    }
}
