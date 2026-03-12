//! Module: bootloader::drivers::manager
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//!
//! INVARIANTS:
//! - This module owns the bootloader-side output-driver selection and dispatch path.
//! - Bootloader output policy is intentionally simple and high-signal-biased during normal runs.
//! - The current driver-selection logic is part of boot-time observability, not a general runtime driver framework.
//!
//! SAFETY:
//! - Global singleton state here relies on the bootloader’s intended single-threaded execution model.
//! - Quieting noisy prefixes is an observability policy choice; it must not hide genuinely critical failure output.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/observability.md
//!
//! Bootloader output-driver manager.

use core::sync::atomic::{AtomicPtr, Ordering};

/// Trait implemented by bootloader-side output drivers.
pub trait Driver {
    /// Attempt to write one line of text through this driver.
    fn write_line(&self, message: &str) -> bool;

    /// Report whether this driver currently appears usable.
    #[allow(dead_code)]
    fn is_available(&self) -> bool;

    /// Return a human-readable driver name for diagnostics.
    fn name(&self) -> &'static str;
}

/// Bootloader output-driver coordinator.
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
    /// Create the bootloader output-driver manager.
    pub fn new() -> Self {
        Self {
            uefi_serial: crate::drivers::uefi_serial::UefiSerialDriver::new(),
            raw_serial: crate::drivers::raw_serial::RawSerialDriver::new(),
            qemu_debug: crate::drivers::qemu_debug::QemuDebugDriver::new(),
            current_driver: Self::select_best_driver(),
        }
    }

    /// Initialize the global output driver instance.
    ///
    /// This should run once during bootloader startup. The manager is deliberately
    /// leaked after installation so the global pointer remains valid for the rest of
    /// the firmware-side boot path.
    pub fn init_global() {
        let mut driver = Self::new();
        let driver_ptr = &mut driver as *mut OutputDriver;
        GLOBAL_OUTPUT_DRIVER.store(driver_ptr, Ordering::SeqCst);

        // Leak the driver to keep it alive for the lifetime of the program
        core::mem::forget(driver);

        // Write initial message
        write_line("Output driver initialized");
    }

    /// Select the best available driver.
    fn select_best_driver() -> DriverType {
        // Current repo policy is intentionally simple: prefer QEMU debug output for
        // the normal development/test workflow. This is a workflow choice, not a
        // deep adaptive hardware-detection strategy.
        DriverType::QemuDebug
    }

    /// Update the current driver based on the current bootloader policy.
    pub fn update_driver(&mut self) {
        self.current_driver = Self::select_best_driver();
    }

    /// Write a line using the current driver.
    ///
    /// Fallback between output drivers improves the odds of seeing a message, but it
    /// is still a best-effort observability path rather than a guarantee of delivery.
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
            DriverType::RawSerial => self.raw_serial.write_line(message),
            DriverType::None => false,
        }
    }

    /// Get the name of the current driver.
    pub fn current_driver_name(&self) -> &'static str {
        match self.current_driver {
            DriverType::UefiSerial => self.uefi_serial.name(),
            DriverType::QemuDebug => self.qemu_debug.name(),
            DriverType::RawSerial => self.raw_serial.name(),
            DriverType::None => "None",
        }
    }

    /// Force switch to a specific driver.
    #[allow(dead_code)]
    pub fn force_driver(&mut self, driver_type: DriverType) {
        self.current_driver = driver_type;
    }

    /// Check if any output path appears available.
    #[allow(dead_code)]
    pub fn is_available(&self) -> bool {
        self.uefi_serial.is_available()
            || self.qemu_debug.is_available()
            || self.raw_serial.is_available()
    }
}

// Global access functions

/// Write a line to the global output driver.
///
/// In normal runs this suppresses a set of noisy informational prefixes to keep
/// firmware-phase output high-signal.
pub fn write_line(message: &str) -> bool {
    // Keep default boot output high-signal to conserve context.
    // Verbose output can be re-enabled by toggling `VERBOSE_OUTPUT`.
    if !crate::VERBOSE_OUTPUT {
        let trimmed = message.trim_start();
        let noisy_prefixes = [
            "✓ ",
            "Collecting ",
            "Getting ",
            "Setting ",
            "Allocating ",
            "Copying ",
            "Exiting ",
            "Entering ",
            "Finalizing ",
            "Found ",
        ];
        if noisy_prefixes.iter().any(|p| trimmed.starts_with(p)) {
            return true;
        }
    }

    let driver_ptr = GLOBAL_OUTPUT_DRIVER.load(Ordering::SeqCst);
    if driver_ptr.is_null() {
        return false;
    }

    // SAFETY: We know the driver is valid and we're in a single-threaded environment
    unsafe { (*driver_ptr).write_line(message) }
}

/// Get the name of the current global driver.
pub fn current_driver_name() -> &'static str {
    let driver_ptr = GLOBAL_OUTPUT_DRIVER.load(Ordering::SeqCst);
    if driver_ptr.is_null() {
        return "None";
    }

    // SAFETY: We know the driver is valid and we're in a single-threaded environment
    unsafe { (*driver_ptr).current_driver_name() }
}

/// Update the global driver (e.g., after a boot-phase policy change).
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

/// Force switch to a specific global driver type.
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
