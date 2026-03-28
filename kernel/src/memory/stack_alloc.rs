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
    runtime_mapper::MapError,
    va_alloc::kernel_va_alloc,
};

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
    let _top = (top + 15) & !15;

    // TODO: map stack frames with physical frame allocation
    // This requires integrating with the physical frame allocator.
    // For now, return an error; implementation will be completed
    // after physical allocator integration.
    Err(AllocError::MapFailed(MapError::FrameAllocFailed))
}

/// Free a kernel stack region.
///
/// Unmaps all stack pages, frees the physical frames, and returns the VA
/// to the allocator.
pub fn free_kernel_stack(region: StackRegion) -> Result<(), AllocError> {
    // TODO: unmap stack pages and free physical frames
    // Implementation pending physical allocator integration.

    // Return VA to allocator
    let mut va_alloc = kernel_va_alloc().lock();
    va_alloc.free_va(region.va_base, region.size + GUARD_PAGE_SIZE);

    Ok(())
}
