//! Driver Manager
//! 
//! Manages different output drivers and automatically selects the best available one.

use uefi::Handle;

/// Trait that all output drivers must implement
pub trait Driver {
    /// Write a line of text
    fn write_line(&self, message: &str) -> bool;
    
    /// Check if this driver is currently available
    #[allow(dead_code)]
    fn is_available(&self) -> bool;
    
    /// Get the name of this driver
    fn name(&self) -> &'static str;
}

/// Main output driver manager
pub struct OutputDriver {
    uefi_serial: crate::drivers::uefi_serial::UefiSerialDriver,
    raw_serial: crate::drivers::raw_serial::RawSerialDriver,
    qemu_debug: crate::drivers::qemu_debug::QemuDebugDriver,
    current_driver: DriverType,
}

/// Types of available drivers
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(dead_code)]
pub enum DriverType {
    UefiSerial,
    RawSerial,
    QemuDebug,
    None,
}

impl OutputDriver {
    /// Create a new output driver manager
    pub fn new(serial_handle: Option<Handle>) -> Self {
        Self {
            uefi_serial: crate::drivers::uefi_serial::UefiSerialDriver::new(serial_handle),
            raw_serial: crate::drivers::raw_serial::RawSerialDriver::new(),
            qemu_debug: crate::drivers::qemu_debug::QemuDebugDriver::new(),
            current_driver: Self::select_best_driver(),
        }
    }
    
    /// Select the best available driver
    fn select_best_driver() -> DriverType {
        // For QEMU targets, prefer QEMU debug port as it's simpler and always available
        // This avoids conflicts with UEFI serial and provides consistent output
        DriverType::QemuDebug
    }
    
    /// Update the current driver based on boot services status
    pub fn update_driver(&mut self) {
        self.current_driver = Self::select_best_driver();
    }
    
    /// Write a line using the current driver
    pub fn write_line(&mut self, message: &str) -> bool {
        // Update driver selection in case boot services status changed
        self.update_driver();
        
        match self.current_driver {
            DriverType::UefiSerial => {
                if self.uefi_serial.is_available() {
                    return self.uefi_serial.write_line(message);
                }
                // Fallback to QEMU debug if UEFI serial fails
                self.current_driver = DriverType::QemuDebug;
                self.qemu_debug.write_line(message)
            }
            DriverType::QemuDebug => {
                if self.qemu_debug.is_available() {
                    return self.qemu_debug.write_line(message);
                }
                // Fallback to raw serial if QEMU debug fails
                self.current_driver = DriverType::RawSerial;
                self.raw_serial.write_line(message)
            }
            DriverType::RawSerial => {
                self.raw_serial.write_line(message)
            }
            DriverType::None => {
                false
            }
        }
    }
    
    /// Get the name of the current driver
    pub fn current_driver_name(&self) -> &'static str {
        match self.current_driver {
            DriverType::UefiSerial => self.uefi_serial.name(),
            DriverType::QemuDebug => self.qemu_debug.name(),
            DriverType::RawSerial => self.raw_serial.name(),
            DriverType::None => "None",
        }
    }
    
    /// Force switch to a specific driver
    #[allow(dead_code)]
    pub fn force_driver(&mut self, driver_type: DriverType) {
        self.current_driver = driver_type;
    }
    
    /// Check if any driver is available
    #[allow(dead_code)]
    pub fn is_available(&self) -> bool {
        self.uefi_serial.is_available() || 
        self.qemu_debug.is_available() || 
        self.raw_serial.is_available()
    }
}
