//! Kernel configuration constants
//!
//! Centralized configuration for kernel behavior and debug toggles. Other
//! modules should import these values from `theseus_kernel::config` to keep
//! configuration in a single place.

/// When `true` enable verbose kernel debug output (many debug traces).
pub const VERBOSE_KERNEL_OUTPUT: bool = false;

/// When `true` the kernel will idle (keep running) after initialization.
/// When `false` the kernel will exit QEMU immediately (useful for CI).
pub const KERNEL_SHOULD_IDLE: bool = false;

/// When `true`, dump the UEFI hardware inventory entries during driver init.
pub const PRINT_HARDWARE_INVENTORY: bool = false;

/// When `true`, keep the legacy 1 GiB PHYS_OFFSET identity mapping.
/// When `false`, rely on per-table ACPI mappings and tighter PHYS_OFFSET coverage.
pub const MAP_LEGACY_PHYS_OFFSET_1GIB: bool = true;
