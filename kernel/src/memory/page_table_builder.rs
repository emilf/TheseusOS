//! Module: memory::page_table_builder
//!
//! SOURCE OF TRUTH:
//! - docs/plans/memory.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/memory.md#A1:-The-kernel-executes-from-a-higher-half-virtual-base
//! - docs/axioms/memory.md#A2:-Physical-memory-is-accessed-through-a-fixed-PHYS_OFFSET-linear-mapping-after-paging-is-active
//!
//! INVARIANTS:
//! - `PageTableBuilder` is a convenience layer over the lower-level mapping helpers, not a separate mapping policy authority.
//! - Range mapping still follows the centralized policy implemented in `memory::mapping`.
//!
//! SAFETY:
//! - Builder ergonomics do not reduce the safety obligations of page-table mutation, alignment, or mapping correctness.
//! - Callers must still ensure the VA/PA ranges and flags reflect valid architectural intent.
//!
//! PROGRESS:
//! - docs/plans/memory.md
//!
//! `PageTableBuilder`: batch mapping helper that applies a centralized policy.

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

    /// Map a VA range to a PA range using the centralized policy.
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
