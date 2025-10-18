//! Unified logging subsystem
//!
//! This module provides a comprehensive logging framework with:
//! - **Log Levels**: ERROR, WARN, INFO, DEBUG, TRACE
//! - **Per-Module Filtering**: Different log levels for different modules
//! - **Runtime Configuration**: Change log levels via monitor commands
//! - **Multiple Output Targets**: QEMU debug port, serial, both, or none
//! - **Allocation-Free**: Safe to use in error handlers and critical code
//! - **Rich Formatting**: Shows module, file, line, function for DEBUG/TRACE
//!
//! ## Usage
//!
//! ```rust
//! use crate::{log_error, log_warn, log_info, log_debug, log_trace};
//!
//! log_info!("Kernel initialization starting");
//! log_debug!("Mapping {} bytes at {:#x}", size, addr);
//! log_error!("Failed to allocate frame: {}", reason);
//! ```
//!
//! ## Output Format
//!
//! - **INFO/WARN/ERROR**: `[LEVEL module] message`
//! - **DEBUG/TRACE**: `[LEVEL module::function@file:line] message`
//!
//! ## Configuration
//!
//! Default levels are set in `config.rs`. Runtime changes via monitor:
//! ```text
//! loglevel kernel::memory DEBUG    # Set memory module to DEBUG
//! loglevel                         # Show all module levels
//! logoutput ERROR serial           # Send errors to serial port
//! ```

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

/// Log levels (ordered from most to least severe)
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
    /// Convert log level to string prefix
    pub const fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Error => "ERROR",
            LogLevel::Warn => "WARN ",
            LogLevel::Info => "INFO ",
            LogLevel::Debug => "DEBUG",
            LogLevel::Trace => "TRACE",
        }
    }

    /// Convert string to log level
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

/// Logging system initialization state
static LOGGING_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Initialize the logging subsystem
///
/// Sets up default log levels from config and prepares output targets.
/// Safe to call multiple times (idempotent).
pub fn init() {
    if LOGGING_INITIALIZED.swap(true, Ordering::SeqCst) {
        return; // Already initialized
    }

    // Load default filters from config
    filter::init_default_filters();

    // Set up default output targets
    output::init_default_targets();
}

/// Log a message (internal function used by macros)
///
/// This function is called by the logging macros with pre-captured context.
/// It performs filtering, formatting, and output.
///
/// # Arguments
/// * `level` - Log level
/// * `module` - Module path (from module_path!())
/// * `file` - Source file (from file!())
/// * `line` - Line number (from line!())
/// * `function` - Function name (optional, for DEBUG/TRACE)
/// * `message` - Pre-formatted message string
///
/// # Allocation-Free Guarantee
/// This function uses only stack-allocated buffers. It's safe to call from
/// panic handlers, interrupt handlers, and other critical code.
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

/// Format a log entry into a stack buffer
///
/// Format depends on log level:
/// - INFO/WARN/ERROR: `[LEVEL module] message\n`
/// - DEBUG/TRACE: `[LEVEL module::function@file:line] message\n`
///
/// # Returns
/// Number of bytes written to buffer
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

/// Write a u32 as decimal ASCII into buffer
///
/// # Returns
/// Number of bytes written
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
