//! Kernel configuration constants
//!
//! Centralized configuration for kernel behavior and debug toggles. Other
//! modules should import these values from `theseus_kernel::config` to keep
//! configuration in a single place.

/// When `true` enable verbose kernel debug output (many debug traces).
pub const VERBOSE_KERNEL_OUTPUT: bool = true;

/// When `true` the kernel will idle (keep running) after initialization.
/// When `false` the kernel will exit QEMU immediately (useful for CI).
pub const KERNEL_SHOULD_IDLE: bool = false;

/// When `true`, dump the UEFI hardware inventory entries during driver init.
pub const PRINT_HARDWARE_INVENTORY: bool = true;

/// When `true`, enable serial output to COM1.
pub const ENABLE_SERIAL_OUTPUT: bool = true;

/// When `true`, activate the interactive kernel monitor on COM1.
pub const ENABLE_KERNEL_MONITOR: bool = false;

/// When `true`, run the raw COM1 reverse-echo loop after high-half transition.
/// This is intended for temporary debugging; the kernel will not progress while enabled.
pub const RUN_POST_BOOT_SERIAL_REVERSE_ECHO: bool = false;

/// When `true`, keep the legacy 1 GiB PHYS_OFFSET identity mapping.
/// When `false`, rely on per-table ACPI mappings and tighter PHYS_OFFSET coverage.
pub const MAP_LEGACY_PHYS_OFFSET_1GIB: bool = true;
