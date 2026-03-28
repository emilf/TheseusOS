//! Module: drivers::traits
//!
//! SOURCE OF TRUTH:
//! - docs/plans/drivers-and-io.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - `Device`, `DeviceId`, `DeviceClass`, and `Driver` define the stable kernel-side vocabulary used by the driver manager and monitor surfaces.
//! - Device descriptors may carry MMIO, IRQ, and opaque driver-owned state, but the framework itself does not reinterpret driver-private payloads.
//! - Driver binding remains trait-driven and class-filtered rather than hard-coded per subsystem.
//!
//! SAFETY:
//! - `driver_data` is an opaque raw payload; drivers are responsible for ensuring type, lifetime, aliasing, and cleanup expectations remain valid.
//! - Storing a pointer-like value in `driver_data` does not make the pointee safe to access concurrently from IRQ and non-IRQ contexts without the driver's own synchronization.
//! - Resource metadata in `Device` describes discovered hardware state; drivers must still validate that the resource layout matches their controller before programming it.
//!
//! PROGRESS:
//! - docs/plans/drivers-and-io.md
//!
//! Core driver traits and device descriptions.
//!
//! This module defines the stable vocabulary used by the kernel driver framework.

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::any::Any;
use core::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DeviceClass {
    Serial,
    UsbController,
    Storage,
    Network,
    Bridge,
    Display,
    Audio,
    Input,
    Wireless,
    Unknown,
}

/// Minimal device identifier.
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

/// Device descriptor tracked by the driver manager.
#[derive(Debug)]
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
    /// Opaque driver-defined binding state (type-safe via `Any`).
    pub driver_data: Option<Box<dyn Any + Send>>,
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
        // driver_data is not merged — binding state is per-device and
        // owned by the bound driver.
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

    /// Store typed driver state (replaces any previous state).
    pub fn set_driver_state<T: Any + Send + 'static>(&mut self, state: T) {
        self.driver_data = Some(Box::new(state));
    }

    /// Borrow the driver state as the requested type, or `None` if not set or wrong type.
    pub fn driver_state<T: Any>(&self) -> Option<&T> {
        self.driver_data.as_ref().and_then(|b| b.downcast_ref::<T>())
    }

    /// Mutably borrow the driver state as the requested type.
    pub fn driver_state_mut<T: Any>(&mut self) -> Option<&mut T> {
        self.driver_data
            .as_mut()
            .and_then(|b| b.downcast_mut::<T>())
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
