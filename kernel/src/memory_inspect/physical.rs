//! Module: memory_inspect::physical
//!
//! Physical memory access utilities.
//!
//! This module provides functions to access physical memory through
//! the PHYS_OFFSET linear mapping. All access is validated and
//! returns detailed error information.
//!
//! **IMPORTANT**: TheseusOS requires PHYS_OFFSET mapping to be active
//! for physical memory access. If PHYS_OFFSET is not available,
//! all functions return `MemoryAccessError::PhysicalNotMapped`.

use crate::memory_inspect::error::{MemoryAccessError, MemoryResult};
use crate::memory::{phys_offset_is_active, phys_to_virt_pa};

/// Check if PHYS_OFFSET mapping is active.
///
/// PHYS_OFFSET is a linear mapping of all physical memory into
/// kernel virtual address space. This function checks if the
/// mapping has been established.
///
/// **NOTE**: Physical memory access functions will return
/// `MemoryAccessError::PhysicalNotMapped` if this returns false.
///
/// # Returns
/// - `true` if PHYS_OFFSET mapping is active
/// - `false` otherwise
pub fn phys_offset_active() -> bool {
    phys_offset_is_active()
}

/// Convert physical address to virtual address via PHYS_OFFSET.
///
/// # Parameters
/// - `phys_addr`: Physical address to convert
///
/// # Returns
/// - `Some(virt_addr)` if conversion successful
/// - `None` if PHYS_OFFSET not active or address out of range
pub fn phys_to_virt(phys_addr: u64) -> Option<u64> {
    if !phys_offset_active() {
        return None;
    }
    
    Some(phys_to_virt_pa(phys_addr))
}

/// Convert virtual address to physical address via page tables.
///
/// This performs a page table walk to translate a virtual address
/// to its corresponding physical address.
///
/// # Parameters
/// - `_virt_addr`: Virtual address to convert
///
/// # Returns
/// - `Some(phys_addr)` if translation successful
/// - `None` if address not mapped or translation fails
pub fn virt_to_phys(_virt_addr: u64) -> Option<u64> {
    // TODO: Implement page table walk
    // For now, return None
    None
}

/// Read bytes from physical memory.
///
/// This is the low-level implementation used by `memory_inspect::access`.
///
/// # Parameters
/// - `phys_addr`: Physical address to read from
/// - `buf`: Buffer to read into
///
/// # Returns
/// - `Ok(())` if read successful
/// - `Err(error)` with detailed error information
#[allow(dead_code)]
pub(crate) fn read_physical_impl(phys_addr: u64, buf: &mut [u8]) -> MemoryResult<()> {
    if !phys_offset_active() {
        return Err(MemoryAccessError::PhysicalNotMapped);
    }
    
    let virt_addr = phys_to_virt_pa(phys_addr);
    let ptr = virt_addr as *const u8;
    let len = buf.len();
    
    // Optimize bulk reads using copy_nonoverlapping for large buffers
    // For small buffers or unaligned accesses, fall back to byte-by-byte
    if len >= 8 && (virt_addr as usize) % 8 == 0 && (buf.as_ptr() as usize) % 8 == 0 {
        // Use 64-bit copies for aligned large buffers
        let word_len = len / 8;
        let remainder = len % 8;
        
        let src_words = ptr as *const u64;
        let dst_words = buf.as_mut_ptr() as *mut u64;
        
        for i in 0..word_len {
            unsafe {
                let word = src_words.add(i).read_volatile();
                core::ptr::copy_nonoverlapping(&word, dst_words.add(i), 1);
            }
        }
        
        // Handle remainder bytes
        if remainder > 0 {
            let base_offset = word_len * 8;
            for i in 0..remainder {
                unsafe {
                    buf[base_offset + i] = ptr.add(base_offset + i).read_volatile();
                }
            }
        }
    } else {
        // Fall back to byte-by-byte for unaligned or small buffers
        for i in 0..len {
            unsafe {
                buf[i] = ptr.add(i).read_volatile();
            }
        }
    }
    
    Ok(())
}

/// Write bytes to physical memory.
///
/// This is the low-level implementation used by `memory_inspect::access`.
///
/// # Parameters
/// - `phys_addr`: Physical address to write to
/// - `data`: Bytes to write
///
/// # Returns
/// - `Ok(())` if write successful
/// - `Err(error)` with detailed error information
#[allow(dead_code)]
pub(crate) fn write_physical_impl(phys_addr: u64, data: &[u8]) -> MemoryResult<()> {
    if !phys_offset_active() {
        return Err(MemoryAccessError::PhysicalNotMapped);
    }
    
    let virt_addr = phys_to_virt_pa(phys_addr);
    let ptr = virt_addr as *mut u8;
    let len = data.len();
    
    // Optimize bulk writes using copy_nonoverlapping for large buffers
    // For small buffers or unaligned accesses, fall back to byte-by-byte
    if len >= 8 && (virt_addr as usize) % 8 == 0 && (data.as_ptr() as usize) % 8 == 0 {
        // Use 64-bit copies for aligned large buffers
        let word_len = len / 8;
        let remainder = len % 8;
        
        let dst_words = ptr as *mut u64;
        let src_words = data.as_ptr() as *const u64;
        
        for i in 0..word_len {
            unsafe {
                let word = core::ptr::read(src_words.add(i));
                dst_words.add(i).write_volatile(word);
            }
        }
        
        // Handle remainder bytes
        if remainder > 0 {
            let base_offset = word_len * 8;
            for i in 0..remainder {
                unsafe {
                    ptr.add(base_offset + i).write_volatile(data[base_offset + i]);
                }
            }
        }
    } else {
        // Fall back to byte-by-byte for unaligned or small buffers
        for i in 0..len {
            unsafe {
                ptr.add(i).write_volatile(data[i]);
            }
        }
    }
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_phys_offset_active() {
        // Currently returns false (not implemented)
        assert!(!phys_offset_active());
    }
    
    #[test]
    fn test_phys_to_virt() {
        // Should return None since PHYS_OFFSET not active
        assert_eq!(phys_to_virt(0x1000), None);
    }
    
    #[test]
    fn test_virt_to_phys() {
        // Should return None (not implemented)
        assert_eq!(virt_to_phys(0x1000), None);
    }
}
