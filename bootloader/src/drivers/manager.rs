//! Driver Manager
//! 
//! Manages different output drivers and automatically selects the best available one.
//! Uses a global singleton instance since we're in a single-threaded environment.

use core::sync::atomic::{AtomicPtr, Ordering};

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

// Global singleton instance
static GLOBAL_OUTPUT_DRIVER: AtomicPtr<OutputDriver> = AtomicPtr::new(core::ptr::null_mut());

impl OutputDriver {
    /// Create a new output driver manager
    pub fn new() -> Self {
        Self {
            uefi_serial: crate::drivers::uefi_serial::UefiSerialDriver::new(),
            raw_serial: crate::drivers::raw_serial::RawSerialDriver::new(),
            qemu_debug: crate::drivers::qemu_debug::QemuDebugDriver::new(),
            current_driver: Self::select_best_driver(),
        }
    }
    
    /// Initialize the global output driver instance
    /// This should be called once at startup
    pub fn init_global() {
        let mut driver = Self::new();
        let driver_ptr = &mut driver as *mut OutputDriver;
        GLOBAL_OUTPUT_DRIVER.store(driver_ptr, Ordering::SeqCst);
        
        // Leak the driver to keep it alive for the lifetime of the program
        core::mem::forget(driver);
        
        // Write initial message
        write_line("Output driver initialized");
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
        // Don't update driver after boot services are exited to avoid allocations
        // self.update_driver();
        
        match self.current_driver {
            DriverType::UefiSerial => {
                // Don't check availability after boot services are exited to avoid allocations
                // if self.uefi_serial.is_available() {
                //     return self.uefi_serial.write_line(message);
                // }
                // Fallback to QEMU debug if UEFI serial fails
                self.current_driver = DriverType::QemuDebug;
                self.qemu_debug.write_line(message)
            }
            DriverType::QemuDebug => {
                // Don't check availability after boot services are exited to avoid allocations
                // if self.qemu_debug.is_available() {
                //     return self.qemu_debug.write_line(message);
                // }
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

// Global access functions

/// Write a line to the global output driver
/// This is the main function to use for output throughout the codebase
pub fn write_line(message: &str) -> bool {
    let driver_ptr = GLOBAL_OUTPUT_DRIVER.load(Ordering::SeqCst);
    if driver_ptr.is_null() {
        return false;
    }
    
    // SAFETY: We know the driver is valid and we're in a single-threaded environment
    unsafe {
        (*driver_ptr).write_line(message)
    }
}

/// Get the name of the current global driver
pub fn current_driver_name() -> &'static str {
    let driver_ptr = GLOBAL_OUTPUT_DRIVER.load(Ordering::SeqCst);
    if driver_ptr.is_null() {
        return "None";
    }
    
    // SAFETY: We know the driver is valid and we're in a single-threaded environment
    unsafe {
        (*driver_ptr).current_driver_name()
    }
}

/// Update the global driver (e.g., after boot services exit)
#[allow(dead_code)] // Intended for future use
pub fn update_driver() {
    let driver_ptr = GLOBAL_OUTPUT_DRIVER.load(Ordering::SeqCst);
    if driver_ptr.is_null() {
        return;
    }
    
    // SAFETY: We know the driver is valid and we're in a single-threaded environment
    unsafe {
        (*driver_ptr).update_driver();
    }
}

/// Force switch to a specific driver type
#[allow(dead_code)]
pub fn force_driver(driver_type: DriverType) {
    let driver_ptr = GLOBAL_OUTPUT_DRIVER.load(Ordering::SeqCst);
    if driver_ptr.is_null() {
        return;
    }
    
    // SAFETY: We know the driver is valid and we're in a single-threaded environment
    unsafe {
        (*driver_ptr).force_driver(driver_type);
    }
}
