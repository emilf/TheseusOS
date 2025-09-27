//! UEFI Serial Driver
//!
//! Uses the UEFI Serial I/O protocol for output. Only works during boot services.

use crate::serial::serial_write_line;
use uefi::boot::SearchType;
use uefi::proto::console::serial::Serial;
use uefi::{Handle, Identify};

/// UEFI Serial driver implementation
pub struct UefiSerialDriver {
    handle: Option<Handle>,
}

impl UefiSerialDriver {
    /// Create a new UEFI serial driver
    /// This will automatically locate and initialize the serial protocol
    pub fn new() -> Self {
        let handle = uefi::boot::locate_handle_buffer(SearchType::ByProtocol(&Serial::GUID))
            .ok()
            .and_then(|buf| buf.first().copied());

        Self { handle }
    }

    /// Check if UEFI boot services are still active
    pub fn is_boot_services_active() -> bool {
        // Check if we can still access UEFI services by trying to get the system table
        uefi::table::system_table_raw().is_some()
    }

    /// Check if this driver is available
    pub fn is_available(&self) -> bool {
        Self::is_boot_services_active() && self.handle.is_some()
    }
}

impl crate::drivers::manager::Driver for UefiSerialDriver {
    fn write_line(&self, message: &str) -> bool {
        if !self.is_available() {
            return false;
        }

        serial_write_line(self.handle, message);
        true
    }

    fn is_available(&self) -> bool {
        self.is_available()
    }

    fn name(&self) -> &'static str {
        "UEFI Serial"
    }
}
