//! PageTableBuilder: batch mapping helper that applies a mapping policy
//!
//! Provides a small builder that lets callers add mapping ranges and applies
//! a centralized mapping policy (prefers 2MiB huge pages when aligned) which
//! simplifies higher-level mapping logic.

use crate::memory::{PageTable, PAGE_SIZE};
use crate::memory::frame_allocator::BootFrameAllocator;
use crate::memory::mapping::map_range_with_policy;

pub struct PageTableBuilder<'a> {
    pml4: &'a mut PageTable,
    fa: &'a mut BootFrameAllocator,
}

impl<'a> PageTableBuilder<'a> {
    /// Create a new builder bound to `pml4` and `fa`.
    pub fn new(pml4: &'a mut PageTable, fa: &'a mut BootFrameAllocator) -> Self {
        Self { pml4, fa }
    }

    /// Map a VA range to PA range using the centralized policy.
    ///
    /// This applies the mapping immediately using the policy implemented in
    /// `map_range_with_policy`.
    pub unsafe fn map_range(&mut self, va: u64, pa: u64, size: u64, flags: u64) {
        if size == 0 { return; }
        map_range_with_policy(self.pml4, va, pa, size, flags, self.fa);
    }

    /// Convenience: map many pages using page granularity.
    pub unsafe fn map_pages(&mut self, va: u64, pa: u64, pages: u64, flags: u64) {
        let size = pages * PAGE_SIZE as u64;
        self.map_range(va, pa, size, flags);
    }
}


