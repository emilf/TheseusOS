//! Logging macros
//!
//! This module provides convenient logging macros with automatic context capture:
//! - `log_error!` - Critical errors that may cause system failure
//! - `log_warn!` - Warning conditions that should be investigated
//! - `log_info!` - Informational messages about normal operation
//! - `log_debug!` - Debugging information for development
//! - `log_trace!` - Detailed trace information for deep debugging
//!
//! ## Usage
//!
//! ```rust
//! use crate::{log_error, log_warn, log_info, log_debug, log_trace};
//!
//! log_error!("Failed to allocate frame");
//! log_warn!("Memory fragmentation detected");
//! log_info!("Kernel initialization complete");
//! log_debug!("Mapping {} bytes at {:#x}", size, addr);
//! log_trace!("Page table walk: level {}", level);
//! ```
//!
//! ## Context Capture
//!
//! All macros automatically capture:
//! - Module path (from `module_path!()`)
//! - File name (from `file!()`)
//! - Line number (from `line!()`)
//! - Function name (for DEBUG/TRACE levels, via type introspection)
//!
//! ## Allocation-Free
//!
//! All macros use stack-allocated 256-byte buffers for formatting.
//! This makes them safe to use in:
//! - Panic handlers
//! - Interrupt handlers
//! - Early boot code (before heap is initialized)
//! - Any critical code path

/// Log an ERROR level message
///
/// Use for critical errors that may cause system failure or indicate
/// serious problems requiring immediate attention.
///
/// # Examples
///
/// ```rust
/// log_error!("Failed to allocate frame");
/// log_error!("Invalid address: {:#x}", addr);
/// log_error!("Panic in {}: {}", module, message);
/// ```
///
/// # Output Format
///
/// ```text
/// [ERROR module] message
/// ```
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {{
        #[allow(static_mut_refs, unused_unsafe)]
        unsafe {
            static mut FORMAT_BUF: [u8; 256] = [0u8; 256];
            use core::fmt::Write;
            use core::ptr::addr_of_mut;
            let buf_ptr = addr_of_mut!(FORMAT_BUF);
            let buf_slice = &mut *buf_ptr;
            let mut writer = $crate::logging::StackWriter::new(buf_slice);
            let _ = write!(writer, $($arg)*);
            $crate::logging::log_impl(
                $crate::logging::LogLevel::Error,
                module_path!(),
                file!(),
                line!(),
                None,
                writer.as_str(),
            )
        }
    }};
}

/// Log a WARN level message
///
/// Use for warning conditions that should be investigated but don't
/// prevent normal operation.
///
/// # Examples
///
/// ```rust
/// log_warn!("Memory fragmentation detected");
/// log_warn!("Unexpected value: {}", value);
/// log_warn!("Retrying operation {} of {}", attempt, max);
/// ```
///
/// # Output Format
///
/// ```text
/// [WARN module] message
/// ```
#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {{
        #[allow(static_mut_refs, unused_unsafe)]
        unsafe {
            static mut FORMAT_BUF: [u8; 256] = [0u8; 256];
            use core::fmt::Write;
            use core::ptr::addr_of_mut;
            let buf_ptr = addr_of_mut!(FORMAT_BUF);
            let buf_slice = &mut *buf_ptr;
            let mut writer = $crate::logging::StackWriter::new(buf_slice);
            let _ = write!(writer, $($arg)*);
            $crate::logging::log_impl(
                $crate::logging::LogLevel::Warn,
                module_path!(),
                file!(),
                line!(),
                None,
                writer.as_str(),
            )
        }
    }};
}

/// Log an INFO level message
///
/// Use for informational messages about normal operation, major milestones,
/// and system state changes.
///
/// # Examples
///
/// ```rust
/// log_info!("Kernel initialization complete");
/// log_info!("Loaded {} drivers", count);
/// log_info!("System ready");
/// ```
///
/// # Output Format
///
/// ```text
/// [INFO module] message
/// ```
#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {{
        #[allow(static_mut_refs, unused_unsafe)]
        unsafe {
            static mut FORMAT_BUF: [u8; 256] = [0u8; 256];
            use core::fmt::Write;
            use core::ptr::addr_of_mut;
            let buf_ptr = addr_of_mut!(FORMAT_BUF);
            let buf_slice = &mut *buf_ptr;
            let mut writer = $crate::logging::StackWriter::new(buf_slice);
            let _ = write!(writer, $($arg)*);
            $crate::logging::log_impl(
                $crate::logging::LogLevel::Info,
                module_path!(),
                file!(),
                line!(),
                None,
                writer.as_str(),
            )
        }
    }};
}

/// Log a DEBUG level message (includes function name)
///
/// Use for debugging information during development. Includes function name
/// in the output for better context.
///
/// # Examples
///
/// ```rust
/// log_debug!("Entering high-half transition");
/// log_debug!("Frame allocated: {:#x}", frame_addr);
/// log_debug!("Mapping {} bytes at {:#x}", size, addr);
/// ```
///
/// # Output Format
///
/// ```text
/// [DEBUG module::function@file:line] message
/// ```
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {{
        #[allow(static_mut_refs, unused_unsafe)]
        unsafe {
            static mut FORMAT_BUF: [u8; 256] = [0u8; 256];
            use core::fmt::Write;
            use core::ptr::addr_of_mut;
            let buf_ptr = addr_of_mut!(FORMAT_BUF);
            let buf_slice = &mut *buf_ptr;
            let mut writer = $crate::logging::StackWriter::new(buf_slice);
            let _ = write!(writer, $($arg)*);

            // Try to extract function name from type_name
            fn f() {}
            fn type_name_of<T>(_: T) -> &'static str {
                core::any::type_name::<T>()
            }
            let name = type_name_of(f);
            let func = name.rsplit("::").nth(1).unwrap_or("unknown");

            $crate::logging::log_impl(
                $crate::logging::LogLevel::Debug,
                module_path!(),
                file!(),
                line!(),
                Some(func),
                writer.as_str(),
            )
        }
    }};
}

/// Log a TRACE level message (includes function name)
///
/// Use for detailed trace information during deep debugging. Most verbose level.
/// Includes function name for maximum context.
///
/// # Examples
///
/// ```rust
/// log_trace!("Page table walk: level {}", level);
/// log_trace!("Register value: CR3={:#x}", cr3);
/// log_trace!("Processing entry {} of {}", i, total);
/// ```
///
/// # Output Format
///
/// ```text
/// [TRACE module::function@file:line] message
/// ```
#[macro_export]
macro_rules! log_trace {
    ($($arg:tt)*) => {{
        #[allow(static_mut_refs, unused_unsafe)]
        unsafe {
            static mut FORMAT_BUF: [u8; 256] = [0u8; 256];
            use core::fmt::Write;
            use core::ptr::addr_of_mut;
            let buf_ptr = addr_of_mut!(FORMAT_BUF);
            let buf_slice = &mut *buf_ptr;
            let mut writer = $crate::logging::StackWriter::new(buf_slice);
            let _ = write!(writer, $($arg)*);

            // Try to extract function name from type_name
            fn f() {}
            fn type_name_of<T>(_: T) -> &'static str {
                core::any::type_name::<T>()
            }
            let name = type_name_of(f);
            let func = name.rsplit("::").nth(1).unwrap_or("unknown");

            $crate::logging::log_impl(
                $crate::logging::LogLevel::Trace,
                module_path!(),
                file!(),
                line!(),
                Some(func),
                writer.as_str(),
            )
        }
    }};
}
