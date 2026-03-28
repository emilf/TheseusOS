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

/// Start of the kernel VA allocation region.
pub const VA_ALLOC_START: u64 = 0xFFFF_9000_0000_0000;

/// End (exclusive) of the kernel VA allocation region.
pub const VA_ALLOC_END: u64 = 0xFFFF_B000_0000_0000;

/// Bump-style kernel virtual address allocator.
///
/// Allocates VA ranges from `VA_ALLOC_START` upward. Free is currently a no-op;
/// a free-list will be added when runtime unmapping becomes common.
pub struct KernelVaAllocator {
    next: u64,
}

impl KernelVaAllocator {
    /// Create a new allocator starting at `VA_ALLOC_START`.
    pub const fn new() -> Self {
        Self {
            next: VA_ALLOC_START,
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
        let aligned = (self.next + align - 1) & !(align - 1);
        let end = aligned.checked_add(size)?;
        if end > VA_ALLOC_END {
            return None;
        }
        self.next = end;
        Some(aligned)
    }

    /// Stub: free a VA region. Currently a no-op (bump allocator).
    ///
    /// A free-list will be implemented when runtime unmapping becomes common.
    pub fn free_va(&mut self, _base: u64, _size: u64) {
        // TODO: implement free-list reclamation
    }
}

/// Global kernel VA allocator.
static KERNEL_VA_ALLOCATOR: Mutex<KernelVaAllocator> = Mutex::new(KernelVaAllocator::new());

/// Access the global kernel VA allocator.
pub fn kernel_va_alloc() -> &'static Mutex<KernelVaAllocator> {
    &KERNEL_VA_ALLOCATOR
}
