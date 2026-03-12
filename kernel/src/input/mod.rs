//! Module: input
//!
//! SOURCE OF TRUTH:
//! - docs/plans/drivers-and-io.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - The input subsystem provides shared event-distribution primitives for human-interface devices.
//! - Concrete device drivers publish decoded input events into this layer rather than exposing transport-specific details to higher consumers.
//! - Keyboard support is currently the active implemented path.
//!
//! SAFETY:
//! - Input publication and listener registration must preserve queue/listener validity across interrupt-driven producer paths and higher-level consumers.
//! - Device-specific decoding correctness remains the responsibility of the publishing driver; this layer does not validate raw HID/transport semantics.
//!
//! PROGRESS:
//! - docs/plans/drivers-and-io.md
//!
//! Input subsystem entry points.
//!
//! This module exposes shared services for human-interface devices as they come
//! online.

/// Keyboard-centric primitives such as event queues and listener registration.
pub mod keyboard;

pub use keyboard::{
    pop_event as keyboard_pop_event, publish_event as keyboard_publish_event,
    register_listener as keyboard_register_listener,
    unregister_listener as keyboard_unregister_listener, KeyEvent, KeyTransition, ListenerHandle,
    Modifiers, RegisterListenerError,
};
