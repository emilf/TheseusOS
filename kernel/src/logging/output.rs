//! Log output target management
//!
//! This module manages where log messages are sent:
//! - QEMU debug port (0xE9) - for development/debugging
//! - Serial port (COM1) - for production logging
//! - Both targets - for comprehensive logging
//! - None - to disable output for a specific level
//!
//! Output targets can be configured per log level, allowing (for example)
//! errors to go to serial while debug messages only go to QEMU debug port.

use super::LogLevel;
use core::sync::atomic::{AtomicU8, Ordering};
use x86_64::instructions::port::Port;

/// Output target for log messages
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OutputTarget {
    /// No output (discard the message)
    None = 0,
    /// QEMU debug port (0xE9) only
    QemuDebug = 1,
    /// Serial port (COM1) only
    Serial = 2,
    /// Both QEMU debug and serial
    Both = 3,
}

impl OutputTarget {
    /// Convert string to output target
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "none" | "off" => Some(OutputTarget::None),
            "qemu" | "debug" => Some(OutputTarget::QemuDebug),
            "serial" | "com1" => Some(OutputTarget::Serial),
            "both" | "all" => Some(OutputTarget::Both),
            _ => None,
        }
    }

    /// Convert to string
    pub const fn as_str(&self) -> &'static str {
        match self {
            OutputTarget::None => "none",
            OutputTarget::QemuDebug => "qemu",
            OutputTarget::Serial => "serial",
            OutputTarget::Both => "both",
        }
    }
}

/// Output target for each log level
///
/// Index corresponds to LogLevel enum values (0-4)
static OUTPUT_TARGETS: [AtomicU8; 5] = [
    AtomicU8::new(OutputTarget::Both as u8),      // Error -> both
    AtomicU8::new(OutputTarget::Both as u8),      // Warn -> both
    AtomicU8::new(OutputTarget::QemuDebug as u8), // Info -> qemu
    AtomicU8::new(OutputTarget::QemuDebug as u8), // Debug -> qemu
    AtomicU8::new(OutputTarget::QemuDebug as u8), // Trace -> qemu
];

/// Initialize default output targets from config
pub(super) fn init_default_targets() {
    use crate::config;

    // Set output targets for each level from config
    set_output_target(LogLevel::Error, config::LOG_OUTPUT_ERROR);
    set_output_target(LogLevel::Warn, config::LOG_OUTPUT_WARN);
    set_output_target(LogLevel::Info, config::LOG_OUTPUT_INFO);
    set_output_target(LogLevel::Debug, config::LOG_OUTPUT_DEBUG);
    set_output_target(LogLevel::Trace, config::LOG_OUTPUT_TRACE);
}

/// Get output target for a log level
///
/// # Arguments
/// * `level` - Log level to query
///
/// # Returns
/// Output target for that level
pub fn get_output_target(level: LogLevel) -> OutputTarget {
    let idx = level as usize;
    if idx >= OUTPUT_TARGETS.len() {
        return OutputTarget::None;
    }

    let val = OUTPUT_TARGETS[idx].load(Ordering::Relaxed);
    u8_to_target(val)
}

/// Set output target for a log level
///
/// # Arguments
/// * `level` - Log level to configure
/// * `target` - Output target to use
///
/// # Examples
/// ```rust
/// set_output_target(LogLevel::Error, OutputTarget::Serial);
/// set_output_target(LogLevel::Debug, OutputTarget::QemuDebug);
/// ```
pub fn set_output_target(level: LogLevel, target: OutputTarget) {
    let idx = level as usize;
    if idx < OUTPUT_TARGETS.len() {
        OUTPUT_TARGETS[idx].store(target as u8, Ordering::Relaxed);
    }
}

/// Write bytes to the specified output target(s)
///
/// This function is allocation-free and safe to call from any context,
/// including panic handlers and interrupt handlers.
///
/// # Arguments
/// * `target` - Where to send the output
/// * `bytes` - Bytes to write
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

/// Write bytes to QEMU debug port (0xE9)
///
/// Uses direct I/O port access for maximum reliability.
/// Works in any context (panic, interrupts, early boot).
fn write_qemu_debug(bytes: &[u8]) {
    unsafe {
        let mut port: Port<u8> = Port::new(0xE9);
        for &byte in bytes {
            port.write(byte);
        }
    }
}

/// Write bytes to serial port (COM1)
///
/// Tries the serial driver first, falls back to direct port I/O if unavailable.
/// Safe to call from any context.
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

/// Convert u8 to OutputTarget
fn u8_to_target(val: u8) -> OutputTarget {
    match val {
        0 => OutputTarget::None,
        1 => OutputTarget::QemuDebug,
        2 => OutputTarget::Serial,
        3 => OutputTarget::Both,
        _ => OutputTarget::None,
    }
}
