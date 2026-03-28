//! Module: memory::va_alloc
//!
//! SOURCE OF TRUTH:
//! - docs/plans/memory.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/memory.md#A1:-The-kernel-executes-from-a-higher-half-virtual-base
//!
//! INVARIANTS:
//! - `KernelVaAllocator` manages the VA range `0xFFFF900000000000..0xFFFFB00000000000`.
//! - This range is well clear of all existing hardcoded VA regions.
//! - Current implementation is a bump allocator; free is a stub.
//!
//! SAFETY:
//! - Returned VAs are not mapped — callers must map before access.
//! - The bump pointer never wraps or aliases existing regions.
//!
//! Kernel virtual address space allocator.

use spin::Mutex;
use alloc::vec::Vec;

/// Start of the kernel VA allocation region.
pub const VA_ALLOC_START: u64 = 0xFFFF_9000_0000_0000;

/// End (exclusive) of the kernel VA allocation region.
pub const VA_ALLOC_END: u64 = 0xFFFF_B000_0000_0000;

/// A free region in the VA allocator.
#[derive(Debug, Clone, Copy)]
struct FreeRegion {
    base: u64,
    size: u64,
}

/// Kernel virtual address allocator with free-list reclamation.
///
/// Manages VA ranges from `VA_ALLOC_START` to `VA_ALLOC_END`.
/// Uses a free-list to reclaim freed regions, falling back to bump allocation
/// when no suitable free region exists.
pub struct KernelVaAllocator {
    next: u64,
    free_list: Vec<FreeRegion>,
}

impl KernelVaAllocator {
    /// Create a new allocator starting at `VA_ALLOC_START`.
    pub const fn new() -> Self {
        Self {
            next: VA_ALLOC_START,
            free_list: Vec::new(),
        }
    }

    /// Allocate a contiguous VA region of `size` bytes, aligned to `align`.
    ///
    /// Returns `None` if the allocation would exceed the managed range.
    pub fn alloc_va(&mut self, size: u64, align: u64) -> Option<u64> {
        if size == 0 {
            return None;
        }
        let align = if align == 0 { 1 } else { align };
        
        // First, try to allocate from free list
        for (i, region) in self.free_list.iter_mut().enumerate() {
            let aligned = (region.base + align - 1) & !(align - 1);
            let end = aligned + size;
            
            // Check if this free region can satisfy the allocation
            if end <= region.base + region.size {
                let result = aligned;
                
                // Split the free region if there's leftover space
                let leftover_before = aligned - region.base;
                let leftover_after = region.size - (leftover_before + size);
                
                if leftover_before > 0 && leftover_after > 0 {
                    // Split into two regions
                    region.size = leftover_before;
                    self.free_list.insert(i + 1, FreeRegion {
                        base: end,
                        size: leftover_after,
                    });
                } else if leftover_before > 0 {
                    // Shrink from the start
                    region.size = leftover_before;
                } else if leftover_after > 0 {
                    // Shrink from the end
                    region.base = end;
                    region.size = leftover_after;
                } else {
                    // Exact fit, remove the region
                    self.free_list.remove(i);
                }
                
                return Some(result);
            }
        }
        
        // Fall back to bump allocation
        let aligned = (self.next + align - 1) & !(align - 1);
        let end = aligned.checked_add(size)?;
        if end > VA_ALLOC_END {
            return None;
        }
        self.next = end;
        Some(aligned)
    }

    /// Free a VA region for future reuse.
    ///
    /// The region is added to the free list and may be coalesced with adjacent
    /// free regions to reduce fragmentation.
    pub fn free_va(&mut self, base: u64, size: u64) {
        if size == 0 {
            return;
        }
        
        let new_region = FreeRegion { base, size };
        
        // Find insertion position and check for adjacency
        let mut i = 0;
        let mut merged = false;
        
        while i < self.free_list.len() {
            let region = &self.free_list[i];
            
            // Check if new region is adjacent to or overlaps with existing region
            if new_region.base + new_region.size == region.base {
                // New region immediately before existing region
                self.free_list[i].base = new_region.base;
                self.free_list[i].size += new_region.size;
                merged = true;
                break;
            } else if region.base + region.size == new_region.base {
                // New region immediately after existing region
                self.free_list[i].size += new_region.size;
                merged = true;
                break;
            } else if new_region.base < region.base {
                // Insert before this region
                break;
            }
            i += 1;
        }
        
        if !merged {
            self.free_list.insert(i, new_region);
        }
        
        // Try to coalesce with neighboring regions
        self.coalesce_free_regions();
    }
    
    /// Coalesce adjacent free regions to reduce fragmentation.
    fn coalesce_free_regions(&mut self) {
        if self.free_list.len() < 2 {
            return;
        }
        
        self.free_list.sort_by_key(|r| r.base);
        
        let mut i = 0;
        while i < self.free_list.len() - 1 {
            let current = self.free_list[i];
            let next = self.free_list[i + 1];
            
            if current.base + current.size == next.base {
                // Merge current and next
                self.free_list[i].size += next.size;
                self.free_list.remove(i + 1);
            } else {
                i += 1;
            }
        }
    }
    
    /// Get statistics about the allocator state.
    pub fn stats(&self) -> VaAllocStats {
        let total_free: u64 = self.free_list.iter().map(|r| r.size).sum();
        let largest_free = self.free_list.iter().map(|r| r.size).max().unwrap_or(0);
        
        VaAllocStats {
            next_bump: self.next,
            free_regions: self.free_list.len(),
            total_free,
            largest_free,
        }
    }
}

/// Statistics about the VA allocator state.
pub struct VaAllocStats {
    /// Next address for bump allocation.
    pub next_bump: u64,
    /// Number of free regions in the free list.
    pub free_regions: usize,
    /// Total free space in bytes.
    pub total_free: u64,
    /// Size of the largest free region in bytes.
    pub largest_free: u64,
}

/// Global kernel VA allocator.
static KERNEL_VA_ALLOCATOR: Mutex<KernelVaAllocator> = Mutex::new(KernelVaAllocator::new());

/// Access the global kernel VA allocator.
pub fn kernel_va_alloc() -> &'static Mutex<KernelVaAllocator> {
    &KERNEL_VA_ALLOCATOR
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_va_alloc_basic() {
        let mut alloc = KernelVaAllocator::new();
        
        // Allocate some regions
        let a1 = alloc.alloc_va(4096, 4096).unwrap();
        let a2 = alloc.alloc_va(8192, 4096).unwrap();
        let a3 = alloc.alloc_va(4096, 4096).unwrap();
        
        assert_eq!(a1, VA_ALLOC_START);
        assert_eq!(a2, VA_ALLOC_START + 4096);
        assert_eq!(a3, VA_ALLOC_START + 4096 + 8192);
        
        // Free middle region
        alloc.free_va(a2, 8192);
        
        // Allocate again - should reuse freed region
        let a4 = alloc.alloc_va(4096, 4096).unwrap();
        assert_eq!(a4, a2); // Should reuse freed region
        
        let stats = alloc.stats();
        assert_eq!(stats.free_regions, 1); // One free region (the leftover from split)
        assert_eq!(stats.total_free, 4096); // 8192 - 4096 = 4096 free
    }

    #[test]
    fn test_va_alloc_alignment() {
        let mut alloc = KernelVaAllocator::new();
        
        // Allocate with different alignments
        let a1 = alloc.alloc_va(100, 1).unwrap();
        assert_eq!(a1, VA_ALLOC_START);
        
        let a2 = alloc.alloc_va(100, 4096).unwrap();
        assert_eq!(a2 % 4096, 0);
        
        // Free and reallocate with alignment
        alloc.free_va(a1, 100);
        let a3 = alloc.alloc_va(50, 4096).unwrap();
        assert_eq!(a3 % 4096, 0);
    }

    #[test]
    fn test_va_alloc_coalesce() {
        let mut alloc = KernelVaAllocator::new();
        
        // Allocate three adjacent regions
        let a1 = alloc.alloc_va(4096, 4096).unwrap();
        let a2 = alloc.alloc_va(4096, 4096).unwrap();
        let a3 = alloc.alloc_va(4096, 4096).unwrap();
        
        // Free all three
        alloc.free_va(a1, 4096);
        alloc.free_va(a3, 4096);
        alloc.free_va(a2, 4096);
        
        // Should coalesce into one large region
        let stats = alloc.stats();
        assert_eq!(stats.free_regions, 1);
        assert_eq!(stats.total_free, 4096 * 3);
        
        // Should be able to allocate the full coalesced region
        let a4 = alloc.alloc_va(4096 * 3, 4096).unwrap();
        assert_eq!(a4, a1);
    }
}
