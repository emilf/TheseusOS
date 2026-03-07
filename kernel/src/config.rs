//! Kernel configuration constants
//!
//! Centralized configuration for kernel behavior and debug toggles. Other
//! modules should import these values from `theseus_kernel::config` to keep
//! configuration in a single place.

/// When `true` enable verbose kernel debug output (many debug traces).
///
/// Default is `false` to keep boot logs high-signal and conserve context when
/// iterating with AI tooling.
pub const VERBOSE_KERNEL_OUTPUT: bool = false;

/// When `true` the kernel will idle (keep running) after initialization.
/// When `false` the kernel will exit QEMU immediately (useful for CI).
pub const KERNEL_SHOULD_IDLE: bool = true;

/// When `true`, dump the UEFI hardware inventory entries during driver init.
///
/// Default is `false` to avoid logspam; use the monitor or enable verbose logs
/// when actively debugging device discovery.
pub const PRINT_HARDWARE_INVENTORY: bool = false;

/// When `true`, enable serial output to COM1.
pub const ENABLE_SERIAL_OUTPUT: bool = true;

/// When `true`, activate the interactive kernel monitor on COM1.
pub const ENABLE_KERNEL_MONITOR: bool = true;

/// When `true`, run the raw COM1 reverse-echo loop after high-half transition.
/// This is intended for temporary debugging; the kernel will not progress while enabled.
pub const RUN_POST_BOOT_SERIAL_REVERSE_ECHO: bool = false;

/// When `true`, keep the legacy 1 GiB PHYS_OFFSET identity mapping.
/// When `false`, rely on per-table ACPI mappings and tighter PHYS_OFFSET coverage.
pub const MAP_LEGACY_PHYS_OFFSET_1GIB: bool = true;

/// When `true`, allow the USB stack to service xHCI controllers via the idle-loop
/// fallback poll even after MSI/MSI-X delivery is configured. Disable to rely solely
/// on interrupt-driven completions (reduces QEMU trace chatter).
pub const USB_ENABLE_POLLING_FALLBACK: bool = false;

/// When `true`, the idle loop will periodically peek at `IMAN` to detect
/// "pending without delivery" cases even when fallback polling is disabled.
///
/// Leave this disabled when QEMU USB tracing (`trace:usb_*`) is enabled, as any
/// runtime register read will spam the host log.
pub const USB_IDLE_IMAN_DIAGNOSTICS: bool = false;

/// When `true`, the xHCI driver will enqueue a NOOP command immediately after
/// successfully enabling MSI/MSI-X and rely on the interrupt path to observe
/// the completion. This provides a boot-time "MVP" signal that MSI delivery is
/// wired correctly, without requiring user input.
pub const USB_RUN_MSIX_SELF_TEST: bool = true;

/// When `true`, log additional xHCI event-ring diagnostics (including warnings
/// and ring snapshots when cycle bits appear unexpected).
///
/// Leave this disabled for normal runs; enabling it is useful when debugging
/// ring desynchronisation or missed interrupts.
pub const USB_XHCI_EVENT_RING_DIAGNOSTICS: bool = false;

/// When `true`, execute a software `int 0x50` once after interrupts are enabled.
/// This validates the IDT handler wiring for the xHCI MSI vector independently
/// of PCI/MSI delivery.
pub const USB_RUN_SW_INT_SELF_TEST: bool = false;

// ============================================================================
// Logging Configuration
// ============================================================================

use crate::logging::{LogLevel, OutputTarget};

/// Default log level for modules without specific configuration.
///
/// Keep this relatively quiet by default; enable DEBUG/TRACE via the kernel
/// monitor (`loglevel ...`) or by changing this constant when doing deep bring-up.
pub const DEFAULT_LOG_LEVEL: LogLevel = LogLevel::Info;

/// Per-module log level overrides
///
/// Add entries here to set specific log levels for different modules.
/// Format: ("module::path", LogLevel::Level)
pub const MODULE_LOG_LEVELS: &[(&str, LogLevel)] = &[
    // Example overrides:
    // ("kernel::memory", LogLevel::Debug),
    // ("kernel::interrupts", LogLevel::Trace),
    // ("kernel::environment", LogLevel::Info),
];

/// Output target for ERROR level logs
pub const LOG_OUTPUT_ERROR: OutputTarget = OutputTarget::Both;

/// Output target for WARN level logs
pub const LOG_OUTPUT_WARN: OutputTarget = OutputTarget::Both;

/// Output target for INFO level logs
pub const LOG_OUTPUT_INFO: OutputTarget = OutputTarget::QemuDebug;

/// Output target for DEBUG level logs
pub const LOG_OUTPUT_DEBUG: OutputTarget = OutputTarget::QemuDebug;

/// Output target for TRACE level logs
pub const LOG_OUTPUT_TRACE: OutputTarget = OutputTarget::QemuDebug;
