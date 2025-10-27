//! USB driver subsystem helpers.
//!
//! This module currently exposes primitives required for taking ownership of
//! USB host controllers from firmware so upcoming drivers can safely bind.

pub mod handoff;
mod xhci;

pub use handoff::ensure_legacy_usb_handoff;
pub use xhci::register_xhci_driver;

/// Register all USB-class drivers with the driver manager.
pub fn init() {
    xhci::register_xhci_driver();
}
