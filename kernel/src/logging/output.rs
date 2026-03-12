//! Module: logging::output
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/logging.md
//! - docs/qemu-runner.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//! - docs/axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status
//!
//! INVARIANTS:
//! - Log output may be routed to QEMU debug port, serial, both, or nowhere on a per-level basis.
//! - Output routing is simple enough to keep working in early boot, panic, and interrupt-adjacent contexts.
//! - QEMU debug output and serial are first-class observability channels in the current workflow.
//!
//! SAFETY:
//! - Direct port I/O fallback paths must remain minimal and reliable because they are used when higher-level serial infrastructure is unavailable.
//! - Output helpers must not assume that serial driver state is fully initialized or safe for blocking behavior.
//! - Dropped bytes in fallback paths are an observability limitation, not a reason to make logging code more stateful or fragile.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//!
//! Log output routing and low-level write paths.
//!
//! This module owns per-level output routing plus the allocation-free QEMU debug
//! and serial write helpers used by the logging subsystem.

use super::LogLevel;
use core::sync::atomic::{AtomicU8, Ordering};
use x86_64::instructions::port::Port;

/// Output destination for log messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OutputTarget {
    /// Drop the message.
    None = 0,
    /// Route only to QEMU debug port `0xE9`.
    QemuDebug = 1,
    /// Route only to the serial path.
    Serial = 2,
    /// Route to both QEMU debug and serial.
    Both = 3,
}

impl OutputTarget {
    /// Parse a string into an output target.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "none" | "off" => Some(OutputTarget::None),
            "qemu" | "debug" => Some(OutputTarget::QemuDebug),
            "serial" | "com1" => Some(OutputTarget::Serial),
            "both" | "all" => Some(OutputTarget::Both),
            _ => None,
        }
    }

    /// Return the config-facing string form.
    pub const fn as_str(&self) -> &'static str {
        match self {
            OutputTarget::None => "none",
            OutputTarget::QemuDebug => "qemu",
            OutputTarget::Serial => "serial",
            OutputTarget::Both => "both",
        }
    }
}

/// Output target for each log level.
///
/// The array index matches the `LogLevel` discriminant order.
static OUTPUT_TARGETS: [AtomicU8; 5] = [
    AtomicU8::new(OutputTarget::Both as u8),      // Error -> both
    AtomicU8::new(OutputTarget::Both as u8),      // Warn -> both
    AtomicU8::new(OutputTarget::QemuDebug as u8), // Info -> qemu
    AtomicU8::new(OutputTarget::QemuDebug as u8), // Debug -> qemu
    AtomicU8::new(OutputTarget::QemuDebug as u8), // Trace -> qemu
];

/// Initialize per-level output routing from config defaults.
pub(super) fn init_default_targets() {
    use crate::config;

    // Load per-level output routing from config.
    set_output_target(LogLevel::Error, config::LOG_OUTPUT_ERROR);
    set_output_target(LogLevel::Warn, config::LOG_OUTPUT_WARN);
    set_output_target(LogLevel::Info, config::LOG_OUTPUT_INFO);
    set_output_target(LogLevel::Debug, config::LOG_OUTPUT_DEBUG);
    set_output_target(LogLevel::Trace, config::LOG_OUTPUT_TRACE);
}

/// Get the current output target for a log level.
pub fn get_output_target(level: LogLevel) -> OutputTarget {
    let idx = level as usize;
    if idx >= OUTPUT_TARGETS.len() {
        return OutputTarget::None;
    }

    let val = OUTPUT_TARGETS[idx].load(Ordering::Relaxed);
    u8_to_target(val)
}

/// Set the output target for a log level.
pub fn set_output_target(level: LogLevel, target: OutputTarget) {
    let idx = level as usize;
    if idx < OUTPUT_TARGETS.len() {
        OUTPUT_TARGETS[idx].store(target as u8, Ordering::Relaxed);
    }
}

/// Write bytes to the selected output target(s).
///
/// This path stays allocation-free so it can be used from panic and other
/// constrained diagnostic contexts.
pub fn write_bytes(target: OutputTarget, bytes: &[u8]) {
    match target {
        OutputTarget::None => {}
        OutputTarget::QemuDebug => write_qemu_debug(bytes),
        OutputTarget::Serial => write_serial(bytes),
        OutputTarget::Both => {
            write_qemu_debug(bytes);
            write_serial(bytes);
        }
    }
}

/// Write bytes directly to QEMU debug port `0xE9`.
fn write_qemu_debug(bytes: &[u8]) {
    unsafe {
        let mut port: Port<u8> = Port::new(0xE9);
        for &byte in bytes {
            port.write(byte);
        }
    }
}

/// Write bytes to the serial logging path.
///
/// This tries the serial driver first and falls back to direct COM1 port I/O if the
/// higher-level driver path is unavailable.
fn write_serial(bytes: &[u8]) {
    // Try serial driver first (if available)
    if crate::drivers::serial::write_bytes_direct(bytes).is_ok() {
        return;
    }

    // Fallback to direct COM1 port I/O (3F8h)
    unsafe {
        let mut port: Port<u8> = Port::new(0x3F8);
        for &byte in bytes {
            // Simple polling write (no FIFO check, may drop bytes if busy)
            port.write(byte);
        }
    }
}

/// Convert a stored `u8` value back into an `OutputTarget`.
fn u8_to_target(val: u8) -> OutputTarget {
    match val {
        0 => OutputTarget::None,
        1 => OutputTarget::QemuDebug,
        2 => OutputTarget::Serial,
        3 => OutputTarget::Both,
        _ => OutputTarget::None,
    }
}
