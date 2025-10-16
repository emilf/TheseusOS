//! Per-module log level filtering
//!
//! This module provides runtime-configurable filtering of log messages by module.
//! Each module can have its own log level, allowing fine-grained control over
//! debug output.
//!
//! ## Design
//!
//! - **Fast Lookup**: O(1) using hash-indexed static array
//! - **No Allocation**: Uses only static arrays and atomics
//! - **Runtime Updates**: Can change levels via monitor commands
//! - **Default Levels**: Loaded from config.rs on init

use super::LogLevel;
use core::sync::atomic::{AtomicU8, Ordering};

/// Number of module filter slots (power of 2 for fast modulo)
const FILTER_SLOTS: usize = 64;

/// Module filter entry
struct FilterEntry {
    /// Module name hash (0 = empty slot)
    hash: AtomicU32,
    /// Log level for this module (as u8)
    level: AtomicU8,
}

use core::sync::atomic::AtomicU32;

/// Global module filter table
///
/// Uses open addressing with linear probing for collision resolution.
/// Each entry stores a module name hash and its corresponding log level.
static MODULE_FILTERS: [FilterEntry; FILTER_SLOTS] = {
    const EMPTY: FilterEntry = FilterEntry {
        hash: AtomicU32::new(0),
        level: AtomicU8::new(LogLevel::Info as u8),
    };
    [EMPTY; FILTER_SLOTS]
};

/// Default log level for modules without specific configuration
static DEFAULT_LEVEL: AtomicU8 = AtomicU8::new(LogLevel::Info as u8);

/// Module filter manager
pub struct ModuleFilter;

impl ModuleFilter {
    /// Get log level for a module
    pub fn get(module: &str) -> LogLevel {
        let hash = hash_module_name(module);
        let idx = (hash as usize) % FILTER_SLOTS;
        
        // Linear probe to find entry
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
    
    /// Set log level for a module
    pub fn set(module: &str, level: LogLevel) {
        let hash = hash_module_name(module);
        let idx = (hash as usize) % FILTER_SLOTS;
        
        // Linear probe to find empty slot or existing entry
        for i in 0..FILTER_SLOTS {
            let probe_idx = (idx + i) % FILTER_SLOTS;
            let entry_hash = MODULE_FILTERS[probe_idx].hash.load(Ordering::Relaxed);
            
            if entry_hash == 0 {
                // Empty slot - insert new entry
                MODULE_FILTERS[probe_idx].hash.store(hash, Ordering::Relaxed);
                MODULE_FILTERS[probe_idx].level.store(level as u8, Ordering::Relaxed);
                return;
            }
            
            if entry_hash == hash {
                // Found existing entry - update level
                MODULE_FILTERS[probe_idx].level.store(level as u8, Ordering::Relaxed);
                return;
            }
        }
        
        // Table full - this shouldn't happen with 64 slots, but if it does,
        // just silently ignore (better than crashing)
    }
    
    /// Set default log level for all modules without specific config
    pub fn set_default(level: LogLevel) {
        DEFAULT_LEVEL.store(level as u8, Ordering::Relaxed);
    }
    
    /// Get default log level
    pub fn get_default() -> LogLevel {
        u8_to_level(DEFAULT_LEVEL.load(Ordering::Relaxed))
    }
}

/// Check if a log message should be output
///
/// # Arguments
/// * `module` - Module path (e.g., "kernel::memory")
/// * `level` - Log level of the message
///
/// # Returns
/// * `true` - Message should be logged
/// * `false` - Message should be filtered out
pub fn should_log(module: &str, level: LogLevel) -> bool {
    let module_level = ModuleFilter::get(module);
    level <= module_level
}

/// Set log level for a specific module
///
/// # Arguments
/// * `module` - Module path (e.g., "kernel::memory")
/// * `level` - Log level to set
///
/// # Examples
/// ```rust
/// set_module_level("kernel::memory", LogLevel::Debug);
/// set_module_level("kernel::interrupts", LogLevel::Warn);
/// ```
pub fn set_module_level(module: &str, level: LogLevel) {
    ModuleFilter::set(module, level);
}

/// Get log level for a specific module
///
/// # Arguments
/// * `module` - Module path
///
/// # Returns
/// Current log level for the module (or default if not configured)
pub fn get_module_level(module: &str) -> LogLevel {
    ModuleFilter::get(module)
}

/// List all configured module log levels
///
/// Returns a list of (module_hash, level) pairs for modules with
/// non-default levels. Note: module names are not stored, only hashes.
///
/// # Returns
/// Array of (hash, level) tuples
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

/// Initialize default log levels from config
pub(super) fn init_default_filters() {
    use crate::config;
    
    // Set default level for all modules
    DEFAULT_LEVEL.store(config::DEFAULT_LOG_LEVEL as u8, Ordering::Relaxed);
    
    // Set per-module overrides from config
    for (module, level) in config::MODULE_LOG_LEVELS {
        ModuleFilter::set(module, *level);
    }
}

/// Simple hash function for module names
///
/// Uses FNV-1a hash algorithm (simple, fast, no allocation)
///
/// # Arguments
/// * `s` - String to hash
///
/// # Returns
/// 32-bit hash value (never returns 0, which is reserved for empty slots)
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

/// Convert u8 to LogLevel (with bounds checking)
fn u8_to_level(val: u8) -> LogLevel {
    match val {
        0 => LogLevel::Error,
        1 => LogLevel::Warn,
        2 => LogLevel::Info,
        3 => LogLevel::Debug,
        4 => LogLevel::Trace,
        _ => LogLevel::Info,  // Default for invalid values
    }
}

