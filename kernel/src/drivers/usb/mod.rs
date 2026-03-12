//! Module: drivers::usb
//!
//! SOURCE OF TRUTH:
//! - docs/plans/drivers-and-io.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/memory.md#A2:-Physical-memory-is-accessed-through-a-fixed-PHYS_OFFSET-linear-mapping-after-paging-is-active
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//!
//! INVARIANTS:
//! - This module is the public USB-subsystem entrypoint layered over firmware handoff and xHCI support.
//! - USB ownership handoff happens before driver registration depends on the controller being under kernel control.
//! - Current runtime focus is xHCI-centric, not a claim of a broad finished USB stack.
//!
//! SAFETY:
//! - High-level USB entrypoint docs must not hide the MMIO/DMA/interrupt safety obligations implemented in the concrete submodules.
//! - Bringing up USB too early or without successful ownership handoff risks programming firmware-owned controller state.
//!
//! PROGRESS:
//! - docs/plans/drivers-and-io.md
//!
//! USB driver subsystem helpers.
//!
//! This module exposes primitives required for taking ownership of USB host
//! controllers from firmware and registering current USB-class drivers.

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
