//! Driver manager implementation
//!
//! The driver manager maintains two lightweight registries: one for drivers and
//! one for devices. When new devices are registered the manager runs a simple
//! probe loop, asking each driver to bind. Drivers indicating success can store
//! opaque binding data via the device descriptor.

use alloc::vec::Vec;
use spin::Mutex;

use crate::display::kernel_write_line;

use super::traits::{Device, DeviceClass, Driver};

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
}
