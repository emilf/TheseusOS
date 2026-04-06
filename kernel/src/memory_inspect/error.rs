//! Module: memory_inspect::error
//!
//! Error types for memory inspection operations.
//!
//! This module defines the `MemoryAccessError` enum and related types
//! to avoid circular dependencies between memory_inspect submodules.

use core::fmt;

/// Error type for memory inspection operations.
///
/// This enum provides detailed error information to help diagnose
/// memory access failures during debugging.
///
/// **NOTE**: TheseusOS does not handle page faults in kernel code.
/// Memory access operations will PANIC on page faults, not return
/// `PageFault` errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryAccessError {
    /// Null pointer encountered
    NullPointer,
    /// Invalid alignment for the operation
    InvalidAlignment,
    /// Address range exceeds bounds
    OutOfBounds,
    /// Memory is not mapped (page fault)
    NotMapped,
    /// Protection violation at specific address
    ProtectionViolation(u64),
    /// PHYS_OFFSET mapping not active
    PhysicalNotMapped,
    /// Cannot perform validation in interrupt context
    InterruptContext,
    /// Address is not canonical (invalid x86_64 virtual address)
    NonCanonicalAddress(u64),
    /// Unknown error (should not occur)
    Unknown,
}

impl fmt::Display for MemoryAccessError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            MemoryAccessError::NullPointer => write!(f, "null pointer"),
            MemoryAccessError::InvalidAlignment => write!(f, "invalid alignment"),
            MemoryAccessError::OutOfBounds => write!(f, "address out of bounds"),
            MemoryAccessError::NotMapped => write!(f, "memory not mapped (would cause PANIC)"),
            MemoryAccessError::ProtectionViolation(addr) => {
                write!(f, "protection violation at 0x{:016x}", addr)
            }
            MemoryAccessError::PhysicalNotMapped => {
                write!(f, "PHYS_OFFSET mapping not active")
            }
            MemoryAccessError::InterruptContext => {
                write!(f, "cannot validate in interrupt context")
            }
            MemoryAccessError::NonCanonicalAddress(addr) => {
                write!(f, "non-canonical address: 0x{:016x}", addr)
            }
            MemoryAccessError::Unknown => write!(f, "unknown error"),
        }
    }
}

/// Result type for memory inspection operations.
pub type MemoryResult<T> = Result<T, MemoryAccessError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_result_type() {
        let ok_result: MemoryResult<u8> = Ok(42);
        assert_eq!(ok_result.unwrap(), 42);

        let err_result: MemoryResult<u8> = Err(MemoryAccessError::OutOfBounds);
        assert!(err_result.is_err());
    }
}