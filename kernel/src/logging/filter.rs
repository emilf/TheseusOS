//! Module: logging::filter
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/logging.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//!
//! INVARIANTS:
//! - This module owns module-level log filtering decisions for the current logging subsystem.
//! - Default levels plus runtime overrides determine whether a log call is emitted before formatting/output work happens.
//!
//! SAFETY:
//! - Filtering bugs change observability, not core safety, but they can still hide failures or flood bring-up logs badly enough to mislead debugging.
//! - Runtime reconfiguration must keep its internal tables coherent without assuming heap-heavy policy machinery.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//!
//! Module-level log filtering support.

use super::LogLevel;
use core::sync::atomic::{AtomicU8, Ordering};

/// Number of module-filter slots.
const FILTER_SLOTS: usize = 64;

/// One module-filter table entry.
struct FilterEntry {
    /// Module name hash (0 = empty slot)
    hash: AtomicU32,
    /// Log level for this module (as u8)
    level: AtomicU8,
}

use core::sync::atomic::AtomicU32;

/// Global module-filter table.
///
/// The table uses open addressing with linear probing and stores only module-name
/// hashes plus their configured levels.
static MODULE_FILTERS: [FilterEntry; FILTER_SLOTS] = {
    const EMPTY: FilterEntry = FilterEntry {
        hash: AtomicU32::new(0),
        level: AtomicU8::new(LogLevel::Info as u8),
    };
    [EMPTY; FILTER_SLOTS]
};

/// Default log level for modules without explicit overrides.
static DEFAULT_LEVEL: AtomicU8 = AtomicU8::new(LogLevel::Info as u8);

/// Manager for module-level log filtering.
pub struct ModuleFilter;

impl ModuleFilter {
    /// Get the effective log level for a module.
    pub fn get(module: &str) -> LogLevel {
        let hash = hash_module_name(module);
        let idx = (hash as usize) % FILTER_SLOTS;

        // Linear probe to find a matching entry.
        for i in 0..FILTER_SLOTS {
            let probe_idx = (idx + i) % FILTER_SLOTS;
            let entry_hash = MODULE_FILTERS[probe_idx].hash.load(Ordering::Relaxed);

            if entry_hash == 0 {
                // Empty slot - use default level
                break;
            }

            if entry_hash == hash {
                // Found matching entry
                let level_u8 = MODULE_FILTERS[probe_idx].level.load(Ordering::Relaxed);
                return u8_to_level(level_u8);
            }
        }

        // Not found - use default
        u8_to_level(DEFAULT_LEVEL.load(Ordering::Relaxed))
    }

    /// Set the log level override for a module.
    pub fn set(module: &str, level: LogLevel) {
        let hash = hash_module_name(module);
        let idx = (hash as usize) % FILTER_SLOTS;

        // Linear probe to find an empty slot or existing entry.
        for i in 0..FILTER_SLOTS {
            let probe_idx = (idx + i) % FILTER_SLOTS;
            let entry_hash = MODULE_FILTERS[probe_idx].hash.load(Ordering::Relaxed);

            if entry_hash == 0 {
                // Empty slot - insert new entry
                MODULE_FILTERS[probe_idx]
                    .hash
                    .store(hash, Ordering::Relaxed);
                MODULE_FILTERS[probe_idx]
                    .level
                    .store(level as u8, Ordering::Relaxed);
                return;
            }

            if entry_hash == hash {
                // Found existing entry - update level
                MODULE_FILTERS[probe_idx]
                    .level
                    .store(level as u8, Ordering::Relaxed);
                return;
            }
        }

        // Table full - this shouldn't happen with 64 slots, but if it does,
        // just silently ignore (better than crashing)
    }

    /// Set the default log level for modules without explicit overrides.
    pub fn set_default(level: LogLevel) {
        DEFAULT_LEVEL.store(level as u8, Ordering::Relaxed);
    }

    /// Get the current default log level.
    pub fn get_default() -> LogLevel {
        u8_to_level(DEFAULT_LEVEL.load(Ordering::Relaxed))
    }
}

/// Decide whether a log call should be emitted.
pub fn should_log(module: &str, level: LogLevel) -> bool {
    let module_level = ModuleFilter::get(module);
    level <= module_level
}

/// Set the log level for a specific module.
pub fn set_module_level(module: &str, level: LogLevel) {
    ModuleFilter::set(module, level);
}

/// Get the current log level for a specific module.
pub fn get_module_level(module: &str) -> LogLevel {
    ModuleFilter::get(module)
}

/// List the configured module-level overrides.
///
/// Only module hashes are stored here, not the original names.
pub fn list_module_levels() -> [(u32, LogLevel); FILTER_SLOTS] {
    let mut result = [(0u32, LogLevel::Info); FILTER_SLOTS];

    for i in 0..FILTER_SLOTS {
        let hash = MODULE_FILTERS[i].hash.load(Ordering::Relaxed);
        if hash != 0 {
            let level_u8 = MODULE_FILTERS[i].level.load(Ordering::Relaxed);
            result[i] = (hash, u8_to_level(level_u8));
        }
    }

    result
}

/// Initialize default and per-module levels from config.
pub(super) fn init_default_filters() {
    use crate::config;

    // Set the default level for modules without explicit overrides.
    DEFAULT_LEVEL.store(config::DEFAULT_LOG_LEVEL as u8, Ordering::Relaxed);

    // Set per-module overrides from config
    for (module, level) in config::MODULE_LOG_LEVELS {
        ModuleFilter::set(module, *level);
    }
}

/// Hash a module name for the fixed filter table.
///
/// Uses FNV-1a and reserves `0` for empty slots.
fn hash_module_name(s: &str) -> u32 {
    const FNV_PRIME: u32 = 16777619;
    const FNV_OFFSET: u32 = 2166136261;

    let mut hash = FNV_OFFSET;
    for byte in s.bytes() {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(FNV_PRIME);
    }

    // Ensure hash is never 0 (reserved for empty)
    if hash == 0 {
        hash = 1;
    }

    hash
}

/// Convert a stored `u8` back into a `LogLevel`.
fn u8_to_level(val: u8) -> LogLevel {
    match val {
        0 => LogLevel::Error,
        1 => LogLevel::Warn,
        2 => LogLevel::Info,
        3 => LogLevel::Debug,
        4 => LogLevel::Trace,
        _ => LogLevel::Info, // Default for invalid values
    }
}
