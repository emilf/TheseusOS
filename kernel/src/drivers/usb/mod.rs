//! USB driver subsystem helpers.
//!
//! This module currently exposes primitives required for taking ownership of
//! USB host controllers from firmware so upcoming drivers can safely bind.

pub mod handoff;
mod xhci;

pub use handoff::ensure_legacy_usb_handoff;
pub use xhci::register_xhci_driver;
pub use xhci::{
    diagnostics_snapshot, poll_runtime_events, poll_runtime_events_fallback, ControllerDiagnostics,
    HidEndpointSummary, PortDiagnostics, kick_msix_self_test, service_deferred_runtime,
    service_runtime_interrupt,
};

/// Register all USB-class drivers with the driver manager.
pub fn init() {
    xhci::register_xhci_driver();
}
