//! USB driver subsystem helpers.
//!
//! This module currently exposes primitives required for taking ownership of
//! USB host controllers from firmware so upcoming drivers can safely bind.

pub mod handoff;

pub use handoff::ensure_legacy_usb_handoff;
