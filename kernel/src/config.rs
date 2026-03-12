//! Module: config
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - This module is the centralized home for current kernel behavior/debug toggles.
//! - Config values here shape runtime behavior, but they do not override architectural truth documented in axioms/plans.
//! - Defaults should bias toward high-signal bring-up rather than maximum logspam or speculative feature enablement.
//!
//! SAFETY:
//! - A configuration flag can make behavior louder, slower, or more invasive; it does not make an unsafe subsystem magically safe.
//! - Comments here must stay aligned with actual consumers or config toggles become lying documentation.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//! - docs/plans/interrupts-and-platform.md
//!
//! Centralized kernel configuration constants.
//!
//! This module gathers the current behavior/debug toggles in one place.

/// When `true` enable verbose kernel debug output (many debug traces).
///
/// Default is `false` to keep boot logs high-signal and conserve context when
/// iterating with AI tooling.
pub const VERBOSE_KERNEL_OUTPUT: bool = false;

/// When `true`, keep the kernel idling after initialization.
/// When `false`, exit QEMU instead (useful for CI-style runs).
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

/// Per-module log level overrides.
///
/// Each entry is `(module_path, LogLevel)`.
pub const MODULE_LOG_LEVELS: &[(&str, LogLevel)] = &[
    // Example overrides:
    // ("kernel::memory", LogLevel::Debug),
    // ("kernel::interrupts", LogLevel::Trace),
    // ("kernel::environment", LogLevel::Info),
];

/// Output target for ERROR logs.
pub const LOG_OUTPUT_ERROR: OutputTarget = OutputTarget::Both;

/// Output target for WARN logs.
pub const LOG_OUTPUT_WARN: OutputTarget = OutputTarget::Both;

/// Output target for INFO logs.
pub const LOG_OUTPUT_INFO: OutputTarget = OutputTarget::QemuDebug;

/// Output target for DEBUG logs.
pub const LOG_OUTPUT_DEBUG: OutputTarget = OutputTarget::QemuDebug;

/// Output target for TRACE logs.
pub const LOG_OUTPUT_TRACE: OutputTarget = OutputTarget::QemuDebug;
