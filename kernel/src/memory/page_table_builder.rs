//! PageTableBuilder: batch mapping helper that applies a mapping policy
//!
//! Provides a small builder that lets callers add mapping ranges and applies
//! a centralized mapping policy (prefers 2MiB huge pages when aligned) which
//! simplifies higher-level mapping logic.

use crate::memory::mapping::map_range_with_policy;
use crate::memory::FrameSource;
use crate::memory::{PageTable, PAGE_SIZE};

pub struct PageTableBuilder<'a, F: FrameSource> {
    pml4: &'a mut PageTable,
    fa: &'a mut F,
}

impl<'a, F: FrameSource> PageTableBuilder<'a, F> {
    /// Create a new builder bound to `pml4` and `fa`.
    pub fn new(pml4: &'a mut PageTable, fa: &'a mut F) -> Self {
        Self { pml4, fa }
    }

    /// Map a VA range to PA range using the centralized policy.
    ///
    /// This applies the mapping immediately using the policy implemented in
    /// `map_range_with_policy`.
    ///
    /// # Examples
    /// ```rust,ignore
    /// use crate::memory::{page_table_builder::PageTableBuilder, FrameSource, PageTable,
    ///     PTE_PRESENT, PTE_WRITABLE};
    /// use x86_64::structures::paging::{FrameAllocator, PhysFrame, Size4KiB};
    /// use x86_64::PhysAddr;
    ///
    /// struct DummyAlloc {
    ///     frame: u64,
    ///     used: bool,
    /// }
    ///
    /// unsafe impl FrameAllocator<Size4KiB> for DummyAlloc {
    ///     fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
    ///         if self.used {
    ///             return None;
    ///         }
    ///         self.used = true;
    ///         Some(PhysFrame::containing_address(PhysAddr::new(self.frame)))
    ///     }
    /// }
    ///
    /// impl FrameSource for DummyAlloc {}
    ///
    /// let mut pml4 = PageTable::new();
    /// let mut allocator = DummyAlloc { frame: 0x2000, used: false };
    /// let mut builder = PageTableBuilder::new(&mut pml4, &mut allocator);
    /// unsafe {
    ///     builder.map_range(
    ///         0xFFFF_8000_0000_0000,
    ///         0x0,
    ///         4096,
    ///         PTE_PRESENT | PTE_WRITABLE,
    ///     );
    /// }
    /// ```
    pub unsafe fn map_range(&mut self, va: u64, pa: u64, size: u64, flags: u64) {
        if size == 0 {
            return;
        }
        map_range_with_policy(self.pml4, va, pa, size, flags, self.fa);
    }

    /// Convenience: map many pages using page granularity.
    pub unsafe fn map_pages(&mut self, va: u64, pa: u64, pages: u64, flags: u64) {
        let size = pages * PAGE_SIZE as u64;
        self.map_range(va, pa, size, flags);
    }
}
