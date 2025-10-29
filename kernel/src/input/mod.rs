//! Input subsystem entry points.
//!
//! The input module exposes shared services for keyboard, mouse, and other
//! human-interface devices as they come online.  Each submodule owns its own
//! buffering policy and fan-out strategy so teaching examples can focus on the
//! device driver while still providing ergonomic hooks for higher layers.

/// Keyboard-centric primitives such as event queues and listener registration.
pub mod keyboard;

pub use keyboard::{
    pop_event as keyboard_pop_event, publish_event as keyboard_publish_event,
    register_listener as keyboard_register_listener,
    unregister_listener as keyboard_unregister_listener, KeyEvent, KeyTransition,
    ListenerHandle, RegisterListenerError,
};
