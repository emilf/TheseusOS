//! Driver manager implementation
//!
//! The driver manager maintains two lightweight registries: one for drivers and
//! one for devices. When new devices are registered the manager runs a simple
//! probe loop, asking each driver to bind. Drivers indicating success can store
//! opaque binding data via the device descriptor.

use alloc::vec::Vec;
use spin::Mutex;

use crate::display::kernel_write_line;

use super::traits::{Device, Driver};

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
                match drv.probe(dev) {
                    Ok(()) => {
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
    pub fn handle_irq(&self, irq: u32) -> bool {
        for dev in self.devices.iter() {
            if let Some(dev_irq) = dev.irq {
                if dev_irq == irq {
                    for drv in self.drivers.iter() {
                        // Ask every driver; in the MVP we expect a single binding
                        if drv.irq_handler(&mut dev.clone(), irq) {
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

    /// Find a device by ID
    pub fn find_device(&self, id: &super::traits::DeviceId) -> Option<&Device> {
        self.devices.iter().find(|dev| &dev.id == id)
    }

    /// Find a mutable device by ID
    pub fn find_device_mut(&mut self, id: &super::traits::DeviceId) -> Option<&mut Device> {
        self.devices.iter_mut().find(|dev| &dev.id == id)
    }

    /// Get the driver bound to a specific device
    pub fn get_driver_for_device(&self, id: &super::traits::DeviceId) -> Option<&'static dyn super::traits::Driver> {
        // First find the device
        let device = self.find_device(id)?;
        
        // Check if it has driver data (meaning it's bound)
        if device.driver_data.is_none() {
            return None;
        }

        // Find the driver that can handle this device
        // In our simple system, we iterate through drivers and ask them to identify themselves
        // A more sophisticated system would store driver references in the device
        for drv in self.drivers.iter() {
            // Create a clone of the device for testing
            let mut test_dev = device.clone();
            if drv.probe(&mut test_dev).is_ok() {
                return Some(*drv);
            }
        }

        None
    }

    /// Write data to a device by ID
    pub fn write_to_device(&mut self, id: &super::traits::DeviceId, buf: &[u8]) -> Result<usize, &'static str> {
        // Find the device index first to avoid borrow conflicts
        let device_idx = self.devices.iter()
            .position(|dev| &dev.id == id)
            .ok_or("Device not found")?;
        
        // Try each driver with the device
        for drv in self.drivers.iter() {
            let device = &mut self.devices[device_idx];
            match drv.write(device, buf) {
                Ok(n) => return Ok(n),
                Err(_) => continue,
            }
        }

        Err("No driver can write to this device")
    }

    /// Read data from a device by ID
    pub fn read_from_device(&mut self, id: &super::traits::DeviceId, buf: &mut [u8]) -> Result<usize, &'static str> {
        // Find the device index first to avoid borrow conflicts
        let device_idx = self.devices.iter()
            .position(|dev| &dev.id == id)
            .ok_or("Device not found")?;
        
        // Try each driver with the device
        for drv in self.drivers.iter() {
            let device = &mut self.devices[device_idx];
            match drv.read(device, buf) {
                Ok(n) => return Ok(n),
                Err(_) => continue,
            }
        }

        Err("No driver can read from this device")
    }
}
