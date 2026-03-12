//! Module: logging::macros
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//!
//! INVARIANTS:
//! - This module provides the user-facing logging macros and helper macro glue for the kernel logging system.
//! - Macro expansion captures call-site context without forcing callers to hand-write module/file/line plumbing.
//!
//! SAFETY:
//! - Macro convenience must not smuggle in allocation-heavy or context-sensitive behavior that would break panic/interrupt-adjacent logging assumptions.
//! - Changes here affect log callsites across the kernel, so even small macro tweaks can have wide observability fallout.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//!
//! Logging macros.
//!
//! This module provides the user-facing logging macros that capture call-site
//! context and forward into the core logging implementation without requiring
//! callers to hand-write module/file/line plumbing.

/// Log an ERROR-level message.
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

/// Log a WARN-level message.
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

/// Log an INFO-level message.
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

/// Log a DEBUG-level message.
///
/// DEBUG output includes the inferred function name when available.
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

/// Log a TRACE-level message.
///
/// TRACE is the most verbose logging level and includes function context when available.
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
