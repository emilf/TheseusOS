//! Module: memory::runtime_mapper
//!
//! SOURCE OF TRUTH:
//! - docs/plans/memory.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/memory.md#A1:-The-kernel-executes-from-a-higher-half-virtual-base
//! - docs/axioms/memory.md#A2:-Physical-memory-is-accessed-through-a-fixed-PHYS_OFFSET-linear-mapping
//! - docs/axioms/memory.md#A3.5:-The-kernel-VA-allocator-manages-a-dedicated-dynamic-region
//!
//! INVARIANTS:
//! - `KernelMapper` wraps an `OffsetPageTable` and a reference to a `PersistentFrameAllocator`.
//! - All mapping operations are single‑CPU for now; TLB shootdowns are deferred to SMP.
//! - `invlpg` is issued after every map/unmap.
//!
//! SAFETY:
//! - The `OffsetPageTable` must be constructed with the correct `PHYS_OFFSET`.
//! - The frame allocator must be initialized before using `KernelMapper`.
//! - `map_page` and `map_range` must be called with a VA that is not already mapped.
//! - `unmap_page` must be called with a VA that is currently mapped.
//!
//! Runtime page‑table manager for kernel‑internal dynamic mappings.

use spin::Mutex;
use x86_64::{
    instructions::tlb,
    structures::paging::{
        mapper::TranslateResult, Mapper, OffsetPageTable, Page, PageTableFlags, Size4KiB, Translate,
    },
    VirtAddr,
};

/// Errors that can occur during mapping operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapError {
    /// The virtual address is already mapped.
    AlreadyMapped,
    /// The virtual address is not mapped.
    NotMapped,
    /// The frame allocator could not allocate a physical frame.
    FrameAllocFailed,
}

/// Kernel‑side runtime page‑table mapper.
pub struct KernelMapper {
    mapper: OffsetPageTable<'static>,
}

impl KernelMapper {
    /// Create a new `KernelMapper`.
    ///
    /// # Safety
    /// - `mapper` must be a valid `OffsetPageTable` for the current `CR3`.
    pub unsafe fn new(mapper: OffsetPageTable<'static>) -> Self {
        Self { mapper }
    }

    /// Map a single page at virtual address `va` to physical address `pa` with `flags`.
    ///
    /// Returns `Ok(())` on success, `Err(MapError::AlreadyMapped)` if the page is already mapped,
    /// or `Err(MapError::FrameAllocFailed)` if a page‑table frame could not be allocated.
    pub fn map_page(
        &mut self,
        va: u64,
        pa: u64,
        flags: PageTableFlags,
    ) -> Result<(), MapError> {
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(va));
        let frame = x86_64::structures::paging::PhysFrame::containing_address(
            x86_64::PhysAddr::new(pa),
        );

        // Check if already mapped
        if self.translate(va).is_some() {
            return Err(MapError::AlreadyMapped);
        }

        unsafe {
            self.mapper
                .map_to(page, frame, flags, &mut crate::physical_memory::PersistentFrameAllocator)
                .map_err(|_| MapError::FrameAllocFailed)?
                .flush();
        }

        // Invalidate TLB entry for this page
        tlb::flush(VirtAddr::new(va));

        Ok(())
    }

    /// Map a contiguous range of pages.
    ///
    /// Maps `size` bytes starting at `va` to `pa`. The size must be page‑aligned.
    pub fn map_range(
        &mut self,
        va: u64,
        pa: u64,
        size: u64,
        flags: PageTableFlags,
    ) -> Result<(), MapError> {
        if size == 0 {
            return Ok(());
        }
        if size % 4096 != 0 {
            // For simplicity, require page‑aligned size
            return Err(MapError::FrameAllocFailed);
        }

        let mut current_va = va;
        let mut current_pa = pa;
        let pages = size / 4096;

        for _ in 0..pages {
            self.map_page(current_va, current_pa, flags)?;
            current_va += 4096;
            current_pa += 4096;
        }

        Ok(())
    }

    /// Unmap a single page at virtual address `va`.
    ///
    /// Returns the physical address of the unmapped page, or `Err(MapError::NotMapped)`.
    pub fn unmap_page(&mut self, va: u64) -> Result<u64, MapError> {
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(va));
        let (frame, flush) = self
            .mapper
            .unmap(page)
            .map_err(|_| MapError::NotMapped)?;

        flush.flush();

        // Invalidate TLB entry
        tlb::flush(VirtAddr::new(va));

        Ok(frame.start_address().as_u64())
    }

    /// Translate a virtual address to a physical address.
    ///
    /// Returns `Some(pa)` if the address is mapped, `None` otherwise.
    pub fn translate(&self, va: u64) -> Option<u64> {
        match self.mapper.translate(x86_64::VirtAddr::new(va)) {
            TranslateResult::Mapped { frame, .. } => {
                Some(frame.start_address().as_u64())
            }
            _ => None,
        }
    }
}

/// Global kernel mapper singleton.
static KERNEL_MAPPER: Mutex<Option<KernelMapper>> = Mutex::new(None);

/// Initialize the global kernel mapper.
///
/// # Safety
/// Must be called exactly once after the high‑half transition and after the
/// persistent frame allocator is ready.
pub unsafe fn init_kernel_mapper(mapper: OffsetPageTable<'static>) {
    let mut guard = KERNEL_MAPPER.lock();
    *guard = Some(KernelMapper::new(mapper));
}

/// Access the global kernel mapper.
///
/// # Panics
/// Panics if `init_kernel_mapper` has not been called.
pub fn kernel_mapper() -> &'static Mutex<Option<KernelMapper>> {
    &KERNEL_MAPPER
}
