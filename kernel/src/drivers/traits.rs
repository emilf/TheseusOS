//! Core driver traits and device descriptions
//!
//! The driver system is intentionally small for the MVP. We model devices as
//! lightweight descriptors that hold identity, resources and driver-specific
//! opaque data. Drivers implement a trait with lifecycle hooks so the kernel can
//! probe and later remove devices, as well as deliver IRQs or simple I/O.

use core::fmt;

/// Minimal device identifier
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum DeviceId {
    /// ACPI hardware ID (HID) such as `"PNP0A08"`
    Acpi(&'static str),
    /// PCI Bus/Device/Function tuple
    Pci { bus: u8, device: u8, function: u8 },
    /// Generic/raw identifier for platform-specific devices
    Raw(&'static str),
}

impl fmt::Display for DeviceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DeviceId::Acpi(hid) => write!(f, "ACPI:{}", hid),
            DeviceId::Pci {
                bus,
                device,
                function,
            } => {
                write!(f, "PCI:{:02x}:{:02x}.{:x}", bus, device, function)
            }
            DeviceId::Raw(name) => write!(f, "RAW:{}", name),
        }
    }
}

/// Device descriptor tracked by the driver manager
#[derive(Debug, Clone)]
pub struct Device {
    /// Device identity (ACPI, PCI or raw)
    pub id: DeviceId,
    /// MMIO physical base (if applicable)
    pub phys_addr: Option<u64>,
    /// IRQ vector/global system interrupt (if assigned)
    pub irq: Option<u32>,
    /// Opaque driver-defined binding state stored as usize
    pub driver_data: Option<usize>,
}

impl Device {
    /// Construct a basic device descriptor
    pub const fn new(id: DeviceId) -> Self {
        Self {
            id,
            phys_addr: None,
            irq: None,
            driver_data: None,
        }
    }
}

/// Core driver trait used by the kernel driver manager
pub trait Driver: Sync + Send {
    /// Called at registration. Returning `Ok(true)` indicates the driver should
    /// be probed immediately.
    fn on_register(&'static self) -> Result<bool, &'static str> {
        Ok(true)
    }

    /// Attempt to bind to a device. Returning `Ok(())` means the device is now
    /// handled by this driver. The driver can stash state via
    /// `dev.driver_data`.
    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str>;

    /// Called when the device is removed or the driver is unloading.
    fn remove(&'static self, _dev: &mut Device) {}

    /// Optional IRQ handler. Return `true` when the IRQ was handled.
    fn irq_handler(&'static self, _dev: &mut Device, _irq: u32) -> bool {
        false
    }

    /// Optional character-device-like write path.
    fn write(&'static self, _dev: &mut Device, _buf: &[u8]) -> Result<usize, &'static str> {
        Err("write not supported")
    }

    /// Optional character-device-like read path.
    fn read(&'static self, _dev: &mut Device, _buf: &mut [u8]) -> Result<usize, &'static str> {
        Err("read not supported")
    }
}
