//! Module: memory_inspect
//!
//! SOURCE OF TRUTH:
//! - docs/plans/memory-inspect.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//! - docs/axioms/memory.md#A2:-Physical-memory-is-accessed-through-a-fixed-PHYS_OFFSET-linear-mapping-after-paging-is-active
//!
//! INVARIANTS:
//! - This module provides safe, validated memory inspection utilities for debugging and diagnostics.
//! - All memory access is validated before execution to catch errors early and provide helpful diagnostics.
//! - The API supports both virtual and physical memory access, with clear separation between the two.
//! - Functions are designed to be usable in interrupt contexts where possible (no heap allocation, minimal stack).
//!
//! SAFETY:
//! - Memory inspection is inherently dangerous; this module provides safety through validation, not by making unsafe operations safe.
//! - Validation functions check for obvious errors (null pointers, overflow) but cannot guarantee memory is mapped or accessible.
//! - **IMPORTANT**: Memory access operations will PANIC on page faults. TheseusOS does not have a page fault handler that can recover from
//!   faults in kernel code. Use `validate_range()` heuristic to check memory accessibility before access.
//! - Physical memory access requires PHYS_OFFSET mapping to be active; the module checks this at runtime.
//! - Functions that perform actual memory access are marked `unsafe` and require the caller to ensure memory validity.
//!
//! PROGRESS:
//! - docs/plans/memory-inspect.md
//!
//! Unified memory inspection utilities for TheseusOS.
//!
//! This module provides safe, validated functions for inspecting and manipulating
//! memory from both kernel code and the monitor. It replaces the ad-hoc memory
//! access patterns currently used in monitor commands with a unified, safety-focused API.
//!
//! # Architecture
//!
//! The module is organized into submodules:
//! - `safety`: Validation functions and safety checks
//! - `access`: Core memory read/write operations
//! - `hexdump`: Hexdump formatting and output
//! - `physical`: Physical memory access (via PHYS_OFFSET)
//!
//! # Usage Examples
//!
//! ```rust
//! use crate::memory_inspect;
//!
//! // Safe memory read with validation
//! match memory_inspect::read_byte(0x1000) {
//!     Ok(byte) => log_info!("Read 0x{:02x} from 0x1000", byte),
//!     Err(e) => log_error!("Failed to read: {:?}", e),
//! }
//!
//! // Hexdump memory region
//! if let Err(e) = memory_inspect::hexdump_virtual(0x2000, 256, None) {
//!     log_error!("Hexdump failed: {:?}", e);
//! }
//!
//! // Physical memory access
//! let mut buffer = [0u8; 16];
//! match memory_inspect::read_physical(0xFEE00000, &mut buffer) {
//!     Ok(()) => log_info!("Read APIC registers"),
//!     Err(e) => log_error!("Failed to read physical memory: {:?}", e),
//! }
//! ```

// Re-export submodules
pub mod access;
pub mod error;
pub mod hexdump;
pub mod physical;
pub mod safety;

// Re-export commonly used types and functions
pub use access::{read_byte, read_bytes, write_byte, write_bytes, fill_bytes,
                 read_byte_skip_validation, read_bytes_skip_validation,
                 write_byte_skip_validation};
pub use error::{MemoryAccessError, MemoryResult};
pub use hexdump::{hexdump_virtual, hexdump_physical};
pub use safety::{validate_virtual_range, validate_physical_range, validate_range};



/// Check if we're running in an interrupt context.
///
/// This is used to skip expensive validation in interrupt handlers
/// where we need to minimize latency.
///
/// # Implementation
/// Uses the x86_64 RFLAGS register to check the interrupt flag.
/// When we're in an interrupt handler, the CPU automatically clears
/// the interrupt flag (IF), so interrupts_enabled() returns false.
///
/// **NOTE**: This is a basic check only. It returns true when interrupts
/// are disabled, which could be in a critical section, not just an
/// interrupt handler. Use `skip_validation` parameter for precise control.
pub fn in_interrupt_context() -> bool {
    use x86_64::registers::rflags::{self, RFlags};
    
    // Simple check: if interrupts are disabled, we might be in an interrupt
    // handler or in a critical section. For safety, assume interrupt context.
    !rflags::read().contains(RFlags::INTERRUPT_FLAG)
}

use spin::Mutex;

/// Simple fixed-size cache for memory inspection operations
///
/// Uses a fixed-size array on the stack, no heap allocation.
/// Implements a simple circular buffer for cache entries.
struct MemoryCache {
    // Fixed array of cache entries
    entries: [CacheEntry; CACHE_SIZE],
    // Current position in circular buffer
    current_index: usize,
    // Number of valid entries
    entry_count: usize,
}

/// Individual cache entry
#[derive(Clone, Copy)]
struct CacheEntry {
    address: u64,
    data: [u8; 64],
    valid: bool,
}

impl CacheEntry {
    const fn new() -> Self {
        Self {
            address: 0,
            data: [0; 64],
            valid: false,
        }
    }
}

/// Cache size - fixed at compile time
const CACHE_SIZE: usize = 8;

impl MemoryCache {
    const fn new() -> Self {
        Self {
            entries: [CacheEntry::new(); CACHE_SIZE],
            current_index: 0,
            entry_count: 0,
        }
    }
    
    fn get_virtual(&self, addr: u64, size: usize) -> Option<&[u8]> {
        // Find cache entry that contains this address
        for entry in &self.entries[0..self.entry_count] {
            if entry.valid && addr >= entry.address && addr + (size as u64) <= entry.address + 64 {
                let offset = (addr - entry.address) as usize;
                return Some(&entry.data[offset..offset + size]);
            }
        }
        None
    }
    
    fn set_virtual(&mut self, addr: u64, data: &[u8]) {
        // Only cache aligned 64-byte blocks
        if data.len() == 64 && addr % 64 == 0 {
            // Check if already cached
            for i in 0..self.entry_count {
                if self.entries[i].address == addr {
                    self.entries[i].data.copy_from_slice(data);
                    return;
                }
            }
            
            // Add new entry
            self.entries[self.current_index] = CacheEntry {
                address: addr,
                data: data.try_into().unwrap(), // Safe because we checked len == 64
                valid: true,
            };
            
            self.current_index = (self.current_index + 1) % CACHE_SIZE;
            if self.entry_count < CACHE_SIZE {
                self.entry_count += 1;
            }
        }
    }
    
    fn clear(&mut self) {
        for entry in &mut self.entries[0..self.entry_count] {
            entry.valid = false;
        }
        self.entry_count = 0;
        self.current_index = 0;
    }
}

static MEMORY_CACHE: Mutex<MemoryCache> = Mutex::new(MemoryCache::new());

fn get_cache() -> &'static Mutex<MemoryCache> {
    &MEMORY_CACHE
}

/// Initialize the memory inspection module.
///
/// This function performs any necessary setup, such as checking
/// if PHYS_OFFSET mapping is active for physical memory access.
pub fn init() {
    // Cache is already initialized statically, nothing to do
    // Check if PHYS_OFFSET is active for physical memory support
    // This will be implemented in the physical module
}

/// Clear the memory inspection cache.
///
/// This can be useful when memory contents are known to have changed
/// (e.g., after writing to memory).
pub fn clear_cache() {
    let mut cache = get_cache().lock();
    cache.clear();
}
