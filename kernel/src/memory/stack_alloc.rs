//! Module: memory::stack_alloc
//!
//! SOURCE OF TRUTH:
//! - docs/plans/memory.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/memory.md#A3.5:-The-kernel-VA-allocator-manages-a-dedicated-dynamic-region
//!
//! INVARIANTS:
//! - Kernel stacks are allocated from the VA allocator region.
//! - Each stack has a guard page left unmapped to catch overflow.
//! - Stack top is 16‑byte aligned for x86_64 ABI compliance.
//!
//! SAFETY:
//! - The VA allocator must be initialized before using this module.
//! - The runtime mapper must be initialized for mapping stack frames.
//! - Stacks must be freed before the VA allocator is destroyed.
//!
//! Kernel stack allocator.

use crate::memory::{
    runtime_mapper::{kernel_mapper, MapError},
    va_alloc::kernel_va_alloc,
};
use x86_64::structures::paging::PageTableFlags;
use crate::log_warn;

/// Size of a guard page (4 KiB).
const GUARD_PAGE_SIZE: u64 = 4096;

/// Errors that can occur during stack allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllocError {
    /// VA allocator could not allocate virtual address space.
    VaAllocFailed,
    /// Runtime mapper failed to map stack frames.
    MapFailed(MapError),
}

/// A kernel stack region descriptor.
#[derive(Debug)]
pub struct StackRegion {
    /// Top of the stack (16‑byte aligned, first usable address).
    pub top: u64,
    /// Bottom of the stack (first address after the guard page).
    pub bottom: u64,
    /// Start of the guard page (unmapped).
    pub guard: u64,
    /// Virtual base address of the allocated region (including guard).
    pub va_base: u64,
    /// Total size of the stack (excluding guard page).
    pub size: u64,
}

/// Allocate a kernel stack of `size` bytes.
///
/// The allocated stack includes a guard page below it. The guard page is left
/// unmapped; accessing it will cause a page fault.
///
/// Returns `Ok(StackRegion)` on success, `Err(AllocError)` on failure.
pub fn alloc_kernel_stack(size: u64) -> Result<StackRegion, AllocError> {
    if size == 0 || size % 4096 != 0 {
        return Err(AllocError::VaAllocFailed);
    }

    // Allocate VA for stack + guard page
    let total_va = size + GUARD_PAGE_SIZE;
    let mut va_alloc = kernel_va_alloc().lock();
    let va_base = va_alloc
        .alloc_va(total_va, 4096)
        .ok_or(AllocError::VaAllocFailed)?;

    // Guard page is at va_base, stack starts after guard
    let guard = va_base;
    let bottom = guard + GUARD_PAGE_SIZE;
    let top = bottom + size;

    // Align top to 16 bytes (x86_64 ABI requirement)
    let top = (top + 15) & !15;

    // Get kernel mapper
    let mut mapper_guard = kernel_mapper().lock();
    let mapper = mapper_guard.as_mut().ok_or(AllocError::MapFailed(MapError::FrameAllocFailed))?;
    
    // Map each stack page
    let mut current_va = bottom;
    for _ in 0..(size / 4096) {
        // Allocate a physical frame
        let pa = match crate::physical_memory::alloc_frame() {
            Ok(pa) => pa,
            Err(_) => return Err(AllocError::MapFailed(MapError::FrameAllocFailed)),
        };
        
        // Map the page with read/write permissions (no execute for W^X)
        let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE;
        if let Err(e) = mapper.map_page(current_va, pa, flags) {
            // Clean up: free the frame we just allocated
            let _ = crate::physical_memory::free_frame(pa);
            return Err(AllocError::MapFailed(e));
        }
        
        current_va += 4096;
    }
    
    Ok(StackRegion {
        top,
        bottom,
        guard,
        va_base,
        size,
    })
}

/// Free a kernel stack region.
///
/// Unmaps all stack pages, frees the physical frames, and returns the VA
/// to the allocator.
pub fn free_kernel_stack(region: StackRegion) -> Result<(), AllocError> {
    // Get kernel mapper
    let mut mapper_guard = kernel_mapper().lock();
    let mapper = mapper_guard.as_mut().ok_or(AllocError::MapFailed(MapError::FrameAllocFailed))?;
    
    // Unmap each stack page and free the physical frame
    let mut current_va = region.bottom;
    for _ in 0..(region.size / 4096) {
        match mapper.unmap_page(current_va) {
            Ok(pa) => {
                // Free the physical frame
                if let Err(e) = crate::physical_memory::free_frame(pa) {
                    log_warn!("Failed to free physical frame {:#x} from stack: {:?}", pa, e);
                }
            }
            Err(e) => {
                log_warn!("Failed to unmap stack page {:#x}: {:?}", current_va, e);
            }
        }
        current_va += 4096;
    }
    
    // Return VA to allocator
    let mut va_alloc = kernel_va_alloc().lock();
    va_alloc.free_va(region.va_base, region.size + GUARD_PAGE_SIZE);

    Ok(())
}
