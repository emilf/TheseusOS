//! # Core Driver Traits and Device Descriptions
//!
//! This module defines the fundamental abstractions for the kernel's driver framework.
//! The design philosophy emphasizes simplicity and flexibility:
//!
//! - **Lightweight**: Minimal overhead for device representation
//! - **Generic**: Works for diverse hardware types (serial, storage, network, etc.)
//! - **Composable**: Drivers can layer on top of each other
//! - **Type-Safe**: Uses Rust's type system to ensure correctness
//!
//! ## Architecture Overview
//!
//! ```text
//! ┌─────────────────────────────────────────────┐
//! │         Driver Manager (registry)            │
//! │  - Maintains list of drivers and devices     │
//! │  - Runs probe logic on device registration   │
//! │  - Dispatches IRQs to appropriate drivers    │
//! └─────────────────────────────────────────────┘
//!                        │
//!         ┌──────────────┼──────────────┐
//!         ▼              ▼              ▼
//!    ┌────────┐    ┌────────┐    ┌────────┐
//!    │Driver 1│    │Driver 2│    │Driver N│
//!    │(Serial)│    │(Block) │    │ (...) │
//!    └────────┘    └────────┘    └────────┘
//!         │              │              │
//!         ▼              ▼              ▼
//!    ┌────────┐    ┌────────┐    ┌────────┐
//!    │Device 1│    │Device 2│    │Device N│
//!    │COM1    │    │ SATA  │    │ (...) │
//!    └────────┘    └────────┘    └────────┘
//! ```
//!
//! ## Device Lifecycle
//!
//! 1. **Discovery**: Hardware is discovered (via ACPI, PCI scan, etc.)
//! 2. **Registration**: A `Device` descriptor is created and registered
//! 3. **Probing**: Driver manager asks each driver if it supports the device
//! 4. **Initialization**: First matching driver's `init()` is called
//! 5. **Operation**: Driver handles I/O and IRQs for the device
//! 6. **Removal**: Driver's `remove()` is called when device is removed
//!
//! ## Device Classes
//!
//! Devices are categorized into broad classes (Serial, Block, Network, etc.).
//! This allows:
//! - Generic code to work with "any serial port" without knowing specifics
//! - Multiple drivers to implement the same class (e.g., 16550 UART, USB CDC-ACM)
//! - Fallback behavior when specific device isn't available
//!
//! ## Driver-Specific State
//!
//! Drivers can store arbitrary state in the device descriptor via `driver_data`.
//! This is an opaque `usize` that drivers typically use as a pointer to their
//! own state structure. The framework doesn't interpret this data; it just
//! preserves it.
//!
//! ## Example Usage
//!
//! ```rust,no_run
//! // Define a driver
//! struct MyDriver;
//!
//! impl Driver for MyDriver {
//!     fn supported_classes(&self) -> &'static [DeviceClass] {
//!         &[DeviceClass::Serial]
//!     }
//!
//!     fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str> {
//!         // Check if this is a device we support
//!         match dev.id {
//!             DeviceId::Acpi("PNP0501") => Ok(()),  // 16550-compatible UART
//!             _ => Err("not supported"),
//!         }
//!     }
//!
//!     fn init(&'static self, dev: &mut Device) -> Result<(), &'static str> {
//!         // Initialize hardware
//!         // ...
//!         Ok(())
//!     }
//!
//!     fn irq_handler(&'static self, dev: &mut Device, irq: u32) -> bool {
//!         // Handle interrupt
//!         true  // Indicate we handled it
//!     }
//! }
//!
//! // Register with driver manager
//! driver_manager().lock().register_driver(&MyDriver);
//! ```

use alloc::vec::Vec;
use core::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DeviceClass {
    Serial,
    UsbController,
    Storage,
    Network,
    Bridge,
    Unknown,
}

/// Minimal device identifier
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum DeviceId {
    /// ACPI hardware ID (HID) such as "PNP0A08"
    Acpi(&'static str),
    /// PCI Bus/Device/Function tuple
    Pci {
        segment: u16,
        bus: u8,
        device: u8,
        function: u8,
    },
    /// Class-based identifier for generic devices (e.g., Serial, Storage)
    Class(DeviceClass),
    /// Generic/raw identifier for platform-specific devices
    Raw(&'static str),
}

impl fmt::Display for DeviceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DeviceId::Acpi(hid) => write!(f, "ACPI:{}", hid),
            DeviceId::Pci {
                segment,
                bus,
                device,
                function,
            } => write!(
                f,
                "PCI:{:04x}:{:02x}:{:02x}.{:x}",
                segment, bus, device, function
            ),
            DeviceId::Raw(name) => write!(f, "RAW:{}", name),
            DeviceId::Class(class) => write!(f, "CLASS:{:?}", class),
        }
    }
}

/// Device descriptor tracked by the driver manager
#[derive(Debug, Clone)]
pub struct Device {
    /// Device identity (ACPI, PCI or raw)
    pub id: DeviceId,
    /// Device class for broad matching
    pub class: DeviceClass,
    /// MMIO physical base (if applicable)
    pub phys_addr: Option<u64>,
    /// IRQ vector/global system interrupt (if assigned)
    pub irq: Option<u32>,
    /// Hardware resources associated with the device (BARs, I/O ports, etc.)
    pub resources: Vec<DeviceResource>,
    /// Opaque driver-defined binding state stored as usize
    pub driver_data: Option<usize>,
}

impl Device {
    /// Construct a basic device descriptor
    pub const fn new(id: DeviceId) -> Self {
        let class = match id {
            DeviceId::Class(class) => class,
            _ => DeviceClass::Unknown,
        };
        Self {
            id,
            class,
            phys_addr: None,
            irq: None,
            resources: Vec::new(),
            driver_data: None,
        }
    }

    pub fn merge_from(&mut self, other: &Device) {
        if self.class == DeviceClass::Unknown && other.class != DeviceClass::Unknown {
            self.class = other.class;
        }
        if self.phys_addr.is_none() {
            self.phys_addr = other.phys_addr;
        }
        if self.irq.is_none() {
            self.irq = other.irq;
        }
        if self.driver_data.is_none() {
            self.driver_data = other.driver_data;
        }
        for res in other.resources.iter() {
            if !self.resources.iter().any(|existing| existing == res) {
                self.resources.push(*res);
            }
        }
    }

    /// Immutable access to the device resource list.
    pub fn resources(&self) -> &[DeviceResource] {
        &self.resources
    }

    /// Mutable access to the device resource list.
    pub fn resources_mut(&mut self) -> &mut Vec<DeviceResource> {
        &mut self.resources
    }

    /// Iterator over all memory resources, optionally filtering by the prefetchable flag.
    pub fn memory_resources(
        &self,
        prefetchable: Option<bool>,
    ) -> impl Iterator<Item = &DeviceResource> {
        self.resources
            .iter()
            .filter(move |res| match (res, prefetchable) {
                (
                    DeviceResource::Memory {
                        prefetchable: flag, ..
                    },
                    Some(expected),
                ) => *flag == expected,
                (DeviceResource::Memory { .. }, None) => true,
                _ => false,
            })
    }

    /// Iterator over all port-based I/O resources.
    pub fn io_resources(&self) -> impl Iterator<Item = &DeviceResource> {
        self.resources.iter().filter(|res| res.is_io())
    }

    /// Convenience helper for the first memory resource matching the `prefetchable` flag (or any if `None`).
    pub fn first_memory_resource(&self, prefetchable: Option<bool>) -> Option<&DeviceResource> {
        self.memory_resources(prefetchable).next()
    }

    /// Convenience helper for the first I/O resource, if present.
    pub fn first_io_resource(&self) -> Option<&DeviceResource> {
        self.io_resources().next()
    }

    pub fn set_driver_state<T>(&mut self, state: &T) {
        self.driver_data = Some(state as *const T as usize);
    }

    pub fn driver_state<T>(&self) -> Option<&T> {
        self.driver_data.map(|ptr| unsafe { &*(ptr as *const T) })
    }

    pub fn driver_state_mut<T>(&mut self) -> Option<&mut T> {
        self.driver_data.map(|ptr| unsafe { &mut *(ptr as *mut T) })
    }
}

/// Hardware resource descriptor provided to drivers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceResource {
    Memory {
        base: u64,
        size: u64,
        prefetchable: bool,
    },
    Io {
        base: u64,
        size: u64,
    },
}

impl DeviceResource {
    pub const fn base(&self) -> u64 {
        match self {
            DeviceResource::Memory { base, .. } | DeviceResource::Io { base, .. } => *base,
        }
    }

    pub const fn size(&self) -> u64 {
        match self {
            DeviceResource::Memory { size, .. } | DeviceResource::Io { size, .. } => *size,
        }
    }

    pub const fn prefetchable(&self) -> Option<bool> {
        match self {
            DeviceResource::Memory { prefetchable, .. } => Some(*prefetchable),
            DeviceResource::Io { .. } => None,
        }
    }

    pub const fn is_memory(&self) -> bool {
        matches!(self, DeviceResource::Memory { .. })
    }

    pub const fn is_io(&self) -> bool {
        matches!(self, DeviceResource::Io { .. })
    }
}

/// Core driver trait used by the kernel driver manager.
///
/// This trait defines the interface that all device drivers must implement
/// to integrate with the kernel's driver framework. It provides a unified
/// way to handle device discovery, initialization, and operation.
///
/// # Driver Lifecycle
///
/// 1. **Registration**: Driver registers with `on_register()`
/// 2. **Probing**: Driver's `probe()` method is called for each compatible device
/// 3. **Initialization**: Driver's `init()` method is called for bound devices
/// 4. **Operation**: Driver handles I/O and interrupts via `read()`, `write()`, `irq_handler()`
/// 5. **Removal**: Driver's `remove()` method is called when device is removed
///
/// # Thread Safety
///
/// All methods are marked as `Sync + Send` to ensure thread safety.
/// Drivers must be prepared to handle concurrent access to their methods.
///
/// # Error Handling
///
/// Most methods return `Result<(), &'static str>` where the error string
/// provides a human-readable description of what went wrong.
pub trait Driver: Sync + Send {
    /// Get the list of device classes this driver supports.
    ///
    /// This method is used by the driver manager to determine which devices
    /// to probe with this driver. Only devices whose class is in this list
    /// will be passed to the `probe()` method.
    ///
    /// # Returns
    /// A static slice of device classes this driver can handle
    fn supported_classes(&self) -> &'static [DeviceClass];

    /// Check if this driver supports a specific device class.
    ///
    /// This is a convenience method that checks if the given class is
    /// in the driver's supported classes list.
    ///
    /// # Arguments
    /// * `class` - Device class to check
    ///
    /// # Returns
    /// * `true` - Driver supports this class
    /// * `false` - Driver does not support this class
    fn supports_class(&self, class: DeviceClass) -> bool {
        self.supported_classes().contains(&class)
    }

    /// Called when the driver is registered with the driver manager.
    ///
    /// This method allows drivers to perform initialization tasks that
    /// don't require a specific device. It's called once when the driver
    /// is registered with the system.
    ///
    /// # Returns
    /// * `Ok(true)` - Driver should be probed immediately against existing devices
    /// * `Ok(false)` - Driver should be probed only when new devices are added
    /// * `Err(&'static str)` - Driver registration failed
    ///
    /// # Default Implementation
    /// Returns `Ok(true)` to enable immediate probing.
    fn on_register(&'static self) -> Result<bool, &'static str> {
        Ok(true)
    }

    /// Initialize a device that has been bound to this driver.
    ///
    /// This method is called after a successful `probe()` to perform
    /// device-specific initialization. The driver should:
    /// - Set up hardware registers
    /// - Configure interrupts
    /// - Initialize any internal state
    /// - Store device-specific data in `dev.driver_data`
    ///
    /// # Arguments
    /// * `dev` - Device descriptor (mutable for storing driver state)
    ///
    /// # Returns
    /// * `Ok(())` - Device initialized successfully
    /// * `Err(&'static str)` - Initialization failed
    ///
    /// # Default Implementation
    /// Does nothing and returns `Ok(())`.
    fn init(&'static self, _dev: &mut Device) -> Result<(), &'static str> {
        Ok(())
    }

    /// Attempt to bind to a device.
    ///
    /// This method is called by the driver manager to determine if this
    /// driver can handle a specific device. The driver should examine
    /// the device's properties (ID, class, resources) and decide whether
    /// it can provide support.
    ///
    /// If the driver accepts the device, it should store any necessary
    /// state in `dev.driver_data` and return `Ok(())`. The device will
    /// then be bound to this driver and `init()` will be called.
    ///
    /// # Arguments
    /// * `dev` - Device descriptor to examine and potentially bind to
    ///
    /// # Returns
    /// * `Ok(())` - Driver accepts this device (binding successful)
    /// * `Err(&'static str)` - Driver cannot handle this device
    ///
    /// # Note
    /// This method should be fast and not perform heavy initialization.
    /// Use `init()` for device-specific setup.
    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str>;

    /// Called when a device is removed or the driver is unloading.
    ///
    /// This method allows drivers to clean up device-specific resources
    /// when a device is removed from the system or when the driver is
    /// being unloaded.
    ///
    /// # Arguments
    /// * `dev` - Device descriptor being removed
    ///
    /// # Default Implementation
    /// Does nothing.
    fn remove(&'static self, _dev: &mut Device) {}

    /// Handle an interrupt for a device.
    ///
    /// This method is called when an interrupt occurs for a device bound
    /// to this driver. The driver should examine the device's interrupt
    /// status and handle the interrupt appropriately.
    ///
    /// # Arguments
    /// * `dev` - Device descriptor for the interrupting device
    /// * `irq` - Interrupt request number
    ///
    /// # Returns
    /// * `true` - Interrupt was handled by this driver
    /// * `false` - Interrupt was not handled (try other drivers)
    ///
    /// # Default Implementation
    /// Returns `false` (interrupt not handled).
    fn irq_handler(&'static self, _dev: &mut Device, _irq: u32) -> bool {
        false
    }

    /// Write data to a device.
    ///
    /// This method provides a character-device-like write interface for
    /// devices that support it (e.g., serial ports, network interfaces).
    /// The implementation should write the data to the device and return
    /// the number of bytes actually written.
    ///
    /// # Arguments
    /// * `dev` - Device descriptor
    /// * `buf` - Buffer containing data to write
    ///
    /// # Returns
    /// * `Ok(usize)` - Number of bytes written
    /// * `Err(&'static str)` - Write operation failed
    ///
    /// # Default Implementation
    /// Returns an error indicating write is not supported.
    fn write(&'static self, _dev: &mut Device, _buf: &[u8]) -> Result<usize, &'static str> {
        Err("write not supported")
    }

    /// Write a string to a device.
    ///
    /// This is a convenience method that converts a string to bytes
    /// and calls the `write()` method.
    ///
    /// # Arguments
    /// * `dev` - Device descriptor
    /// * `s` - String to write
    ///
    /// # Returns
    /// * `Ok(usize)` - Number of bytes written
    /// * `Err(&'static str)` - Write operation failed
    fn write_str(&'static self, dev: &mut Device, s: &str) -> Result<usize, &'static str> {
        self.write(dev, s.as_bytes())
    }

    /// Read data from a device.
    ///
    /// This method provides a character-device-like read interface for
    /// devices that support it (e.g., serial ports, network interfaces).
    /// The implementation should read data from the device into the
    /// provided buffer and return the number of bytes actually read.
    ///
    /// # Arguments
    /// * `dev` - Device descriptor
    /// * `buf` - Buffer to read data into
    ///
    /// # Returns
    /// * `Ok(usize)` - Number of bytes read
    /// * `Err(&'static str)` - Read operation failed
    ///
    /// # Default Implementation
    /// Returns an error indicating read is not supported.
    fn read(&'static self, _dev: &mut Device, _buf: &mut [u8]) -> Result<usize, &'static str> {
        Err("read not supported")
    }
}
