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

use crate::{log_debug, log_error, log_trace};

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

    /// Register a driver with the system.
    ///
    /// This method adds a driver to the driver registry and optionally
    /// triggers immediate probing against existing devices. The driver's
    /// `on_register()` method is called to determine the registration behavior.
    ///
    /// # Arguments
    /// * `drv` - Driver to register (must have static lifetime)
    ///
    /// # Behavior
    /// - If `on_register()` returns `Ok(true)`: Driver is added and probed immediately
    /// - If `on_register()` returns `Ok(false)`: Driver is added but not probed yet
    /// - If `on_register()` returns `Err()`: Driver registration fails
    ///
    /// # Thread Safety
    /// This method must be called while holding the driver manager lock.
    pub fn register_driver(&mut self, drv: &'static dyn Driver) {
        match drv.on_register() {
            Ok(true) => {
                log_debug!("Registering driver");
                self.drivers.push(drv);
                // Probe immediately against existing devices
                self.probe_pending_devices();
            }
            Ok(false) => {
                log_debug!("Driver registered (deferred)");
                self.drivers.push(drv);
                // Driver will be probed when new devices are added
            }
            Err(e) => {
                log_error!("Driver registration failed: {}", e);
            }
        }
    }

    /// Add a discovered device and run probe logic.
    ///
    /// This method adds a new device to the device registry and attempts
    /// to bind it to a compatible driver. If a device with the same ID
    /// already exists, the new device's information is merged into the
    /// existing device descriptor.
    ///
    /// # Arguments
    /// * `device` - Device descriptor to add
    ///
    /// # Behavior
    /// 1. Check if device already exists (by ID)
    /// 2. If exists: merge new information into existing device
    /// 3. If new: add to device list
    /// 4. Run probe logic to find compatible driver
    ///
    /// # Thread Safety
    /// This method must be called while holding the driver manager lock.
    pub fn add_device(&mut self, device: Device) {
        log_trace!("Discovered device");
        
        // Check if device already exists
        if let Some(existing) = self.devices.iter_mut().find(|d| d.id == device.id) {
            // Merge new device information into existing device
            existing.merge_from(&device);
            return;
        }
        
        // Add new device to the registry
        self.devices.push(device);
        
        // Attempt to bind the device to a driver
        self.probe_pending_devices();
    }

    /// Probe all unbound devices against available drivers.
    ///
    /// This method implements the core driver binding algorithm. It iterates
    /// through all devices that don't have a driver bound and attempts to
    /// find a compatible driver for each one.
    ///
    /// # Algorithm
    /// 1. For each unbound device:
    /// 2.   For each registered driver:
    /// 3.     If driver supports device class:
    /// 4.       Call driver.probe(device)
    /// 5.       If probe succeeds:
    /// 6.         Call driver.init(device)
    /// 7.         Mark device as bound
    /// 8.         Stop trying other drivers (first-match wins)
    ///
    /// # Thread Safety
    /// This method must be called while holding the driver manager lock.
    fn probe_pending_devices(&mut self) {
        // Iterate through all devices
        for dev in self.devices.iter_mut() {
            // Skip devices that already have a driver bound
            if dev.driver_data.is_some() {
                continue;
            }
            
            // Try each registered driver
            for drv in self.drivers.iter() {
                // Check if driver supports this device class
                if !drv.supports_class(dev.class) {
                    continue;
                }
                
                // Attempt to probe the device
                match drv.probe(dev) {
                    Ok(()) => {
                        // Probe succeeded, initialize the device
                        if let Err(e) = drv.init(dev) {
                            log_error!("Device init failed: {}", e);
                            continue;
                        }
                        
                        log_debug!("Device bound successfully");
                        break; // Stop trying other drivers (first-match wins)
                    }
                    Err(_) => {
                        // Try next driver
                    }
                }
            }
        }
    }

    /// Handle an interrupt request by dispatching it to the appropriate driver.
    ///
    /// This method implements interrupt dispatch by finding the device(s) that
    /// are assigned the given IRQ and calling their driver's interrupt handler.
    /// The first driver that claims to handle the interrupt stops the search.
    ///
    /// # Arguments
    /// * `irq` - Interrupt request number to handle
    ///
    /// # Returns
    /// * `true` - Interrupt was handled by a driver
    /// * `false` - No driver claimed the interrupt
    ///
    /// # Algorithm
    /// 1. Find all devices assigned to this IRQ
    /// 2. For each device, try all registered drivers
    /// 3. Call driver.irq_handler(device, irq)
    /// 4. If handler returns true, stop searching
    ///
    /// # Thread Safety
    /// This method must be called while holding the driver manager lock.
    pub fn handle_irq(&mut self, irq: u32) -> bool {
        // Find devices assigned to this IRQ
        for dev in self.devices.iter_mut() {
            if let Some(dev_irq) = dev.irq {
                if dev_irq == irq {
                    // Try all registered drivers for this device
                    for drv in self.drivers.iter() {
                        if drv.irq_handler(dev, irq) {
                            return true; // Interrupt handled
                        }
                    }
                }
            }
        }
        false // No driver handled the interrupt
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
