//! Module: logging
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//! - docs/axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status
//!
//! INVARIANTS:
//! - Kernel logging initializes at `kernel_entry` and provides the common diagnostic surface for bring-up and failure reporting.
//! - Filtering and output-target routing are runtime-configurable, but the formatting path remains allocation-free.
//! - DEBUG/TRACE richness is a diagnostics feature, not an excuse to make default boot output noisy.
//!
//! SAFETY:
//! - Logging must remain usable in panic and interrupt-adjacent contexts, so formatting paths cannot assume heap allocation or heavyweight synchronization.
//! - Log routing policy is observability behavior, not a correctness boundary; subsystems must not rely on logs being visible to remain safe.
//! - Changes here should be checked against `docs/logging.md` and current runtime defaults so documentation and behavior do not drift apart.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//!
//! Unified kernel logging subsystem.
//!
//! This module owns log levels, per-module filtering, runtime output routing, and
//! allocation-free formatting/output for kernel diagnostics.

mod filter;
mod output;

// Macros must be declared before they're used
#[macro_use]
pub mod macros;

pub use filter::{get_module_level, list_module_levels, set_module_level, ModuleFilter};
pub use output::{get_output_target, set_output_target, OutputTarget};

use core::fmt;
use core::sync::atomic::{AtomicBool, Ordering};
extern crate alloc;

/// Log levels ordered from most to least severe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum LogLevel {
    /// Critical errors that may cause system failure
    Error = 0,
    /// Warning conditions that should be investigated
    Warn = 1,
    /// Informational messages about normal operation
    Info = 2,
    /// Debugging information for development
    Debug = 3,
    /// Detailed trace information for deep debugging
    Trace = 4,
}

impl LogLevel {
    /// Return the fixed-width string prefix for this level.
    pub const fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Error => "ERROR",
            LogLevel::Warn => "WARN ",
            LogLevel::Info => "INFO ",
            LogLevel::Debug => "DEBUG",
            LogLevel::Trace => "TRACE",
        }
    }

    /// Parse a string into a log level.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_uppercase().as_str() {
            "ERROR" | "ERR" | "E" => Some(LogLevel::Error),
            "WARN" | "WARNING" | "W" => Some(LogLevel::Warn),
            "INFO" | "I" => Some(LogLevel::Info),
            "DEBUG" | "DBG" | "D" => Some(LogLevel::Debug),
            "TRACE" | "TRC" | "T" => Some(LogLevel::Trace),
            _ => None,
        }
    }
}

impl fmt::Display for LogLevel {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

/// Tracks whether kernel logging has already been initialized.
static LOGGING_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Initialise kernel logging once.
pub fn init() {
    if LOGGING_INITIALIZED.swap(true, Ordering::SeqCst) {
        return; // Already initialized
    }

    // Load default filters from config
    filter::init_default_filters();

    // Set up default output targets
    output::init_default_targets();
}

/// Core logging implementation used by the macros.
#[doc(hidden)]
pub fn log_impl(
    level: LogLevel,
    module: &str,
    file: &str,
    line: u32,
    function: Option<&str>,
    message: &str,
) {
    // Check if this module/level combination should be logged
    if !filter::should_log(module, level) {
        return;
    }

    // Get output target for this level
    let target = output::get_output_target(level);
    if target == OutputTarget::None {
        return;
    }

    // Format log entry on stack (no heap allocation)
    let mut buf = [0u8; 512];
    let len = format_log_entry(&mut buf, level, module, file, line, function, message);

    // Output to appropriate target(s)
    output::write_bytes(target, &buf[..len]);
}

/// Format one log entry into the provided stack buffer.
///
/// INFO/WARN/ERROR lines use the short module form; DEBUG/TRACE also include the
/// function name and source location when available.
fn format_log_entry(
    buf: &mut [u8],
    level: LogLevel,
    module: &str,
    file: &str,
    line: u32,
    function: Option<&str>,
    message: &str,
) -> usize {
    let mut pos = 0;

    // Write "[LEVEL "
    buf[pos] = b'[';
    pos += 1;
    for byte in level.as_str().bytes() {
        if pos >= buf.len() {
            return pos;
        }
        buf[pos] = byte;
        pos += 1;
    }
    buf[pos] = b' ';
    pos += 1;

    // Write module name (strip crate name prefix to keep lines shorter)
    // "theseus_kernel::environment" -> "environment"
    // "theseus_kernel" -> "kernel" (for crate root)
    let module_to_write = if let Some(idx) = module.find("::") {
        // Has modules - strip crate name
        &module[idx + 2..]
    } else if module == "theseus_kernel" {
        // Crate root - use short name
        "kernel"
    } else {
        // Unknown format - use as-is
        module
    };

    for byte in module_to_write.bytes().take(64) {
        // Limit module name length
        if pos >= buf.len() {
            return pos;
        }
        buf[pos] = byte;
        pos += 1;
    }

    // For DEBUG/TRACE, add file/line/function
    if level >= LogLevel::Debug {
        if let Some(func) = function {
            // Write "::function"
            buf[pos] = b':';
            pos += 1;
            buf[pos] = b':';
            pos += 1;
            for byte in func.bytes().take(32) {
                if pos >= buf.len() {
                    return pos;
                }
                buf[pos] = byte;
                pos += 1;
            }
        }

        // Write "@file:line"
        buf[pos] = b'@';
        pos += 1;

        // Extract just filename from path
        let filename = file.rsplit('/').next().unwrap_or(file);
        for byte in filename.bytes().take(32) {
            if pos >= buf.len() {
                return pos;
            }
            buf[pos] = byte;
            pos += 1;
        }

        buf[pos] = b':';
        pos += 1;

        // Write line number (decimal)
        pos += write_u32_decimal(&mut buf[pos..], line);
    }

    // Write "] "
    buf[pos] = b']';
    pos += 1;
    buf[pos] = b' ';
    pos += 1;

    // Write message
    for byte in message.bytes() {
        if pos >= buf.len() - 2 {
            // Reserve space for \n
            break;
        }
        buf[pos] = byte;
        pos += 1;
    }

    // Write newline
    buf[pos] = b'\n';
    pos += 1;

    pos
}

/// Write a `u32` as decimal ASCII into the buffer.
fn write_u32_decimal(buf: &mut [u8], mut value: u32) -> usize {
    if value == 0 {
        buf[0] = b'0';
        return 1;
    }

    // Build digits in reverse
    let mut digits = [0u8; 10];
    let mut count = 0;

    while value > 0 && count < 10 {
        digits[count] = b'0' + (value % 10) as u8;
        value /= 10;
        count += 1;
    }

    // Write in forward order
    for i in 0..count {
        if i >= buf.len() {
            return i;
        }
        buf[i] = digits[count - 1 - i];
    }

    count
}

/// Stack-based string writer (no heap allocation)
///
/// Writes formatted output to a fixed-size stack buffer.
/// Used by logging macros to format messages without allocation.
pub struct StackWriter {
    buf: &'static mut [u8],
    pos: usize,
}

impl StackWriter {
    /// Create a new stack writer
    pub fn new(buf: &'static mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Get the written content as a string slice
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.pos]).unwrap_or("<invalid utf-8>")
    }
}

impl core::fmt::Write for StackWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.pos;
        let to_write = bytes.len().min(remaining);

        self.buf[self.pos..self.pos + to_write].copy_from_slice(&bytes[..to_write]);
        self.pos += to_write;

        Ok(())
    }
}
