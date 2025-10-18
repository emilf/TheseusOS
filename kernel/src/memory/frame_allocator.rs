//! Frame allocator built from UEFI memory map with reserved pool for critical structures
//!
//! This allocator iterates through the UEFI memory map descriptors and
//! allocates frames from `UEFI_CONVENTIONAL_MEMORY` regions. It maintains
//! state to track the current region and position within that region.
//!
//! The allocator skips physical frame 0 to avoid potential issues with
//! null pointer dereferences.
//!
//! # Reserved Frame Pool
//!
//! The allocator includes a small reserved pool (16 frames) to prevent critical
//! kernel structures from being starved by ephemeral allocations. This ensures
//! that page tables, IST stacks, and other essential structures always have
//! frames available even if the general pool is temporarily exhausted.
//!
//! - `ReservedPool`: fixed-capacity stack storing physical addresses of reserved frames
//! - LIFO allocation: Last frame reserved is first frame allocated
//! - Fallback behavior: If reserved pool empty, falls back to general pool

use crate::physical_memory;
use crate::{log_debug, log_trace};
use x86_64::{
    structures::paging::{FrameAllocator, PhysFrame, Size4KiB},
    PhysAddr,
};

const UEFI_CONVENTIONAL_MEMORY: u32 = 7; // UEFI spec: conventional memory type

/// Lightweight view over a raw UEFI `MemoryDescriptor`.
///
/// We avoid pulling in the `uefi` crate's definition here to keep the boot-time
/// allocator standalone; instead we read the handful of fields we care about
/// using this wrapper.
#[derive(Clone, Copy)]
struct MemoryDescriptorView {
    ptr: *const u8,
}

impl MemoryDescriptorView {
    unsafe fn new(ptr: *const u8) -> Self {
        Self { ptr }
    }

    unsafe fn kind(&self) -> u32 {
        read_u32(self.ptr, 0)
    }

    unsafe fn physical_start(&self) -> u64 {
        read_u64(self.ptr, 8)
    }

    unsafe fn page_count(&self) -> u64 {
        read_u64(self.ptr, 24)
    }
}

/// Fixed-capacity stack of reserved frames kept aside for critical boot-time needs.
#[derive(Clone, Copy)]
struct ReservedPool {
    slots: [u64; 16],
    count: usize,
}

impl ReservedPool {
    const fn new() -> Self {
        Self {
            slots: [0; 16],
            count: 0,
        }
    }

    fn push(&mut self, frame_pa: u64) -> bool {
        if self.count >= self.slots.len() {
            return false;
        }
        self.slots[self.count] = frame_pa;
        self.count += 1;
        true
    }

    fn pop(&mut self) -> Option<u64> {
        if self.count == 0 {
            return None;
        }
        self.count -= 1;
        Some(self.slots[self.count])
    }

    fn is_full(&self) -> bool {
        self.count >= self.slots.len()
    }
}

/// Trait abstracting over sources of physical frames for page-table building and
/// general allocations. Implemented by both the boot-time allocator and the
/// persistent allocator so mapping helpers can operate on either.
pub trait FrameSource {
    fn alloc_frame(&mut self) -> Option<PhysFrame<Size4KiB>>;

    fn alloc_page_table_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        self.alloc_frame()
    }
}

pub struct BootFrameAllocator {
    base_ptr: *const u8,
    desc_size: usize,
    count: usize,
    cur_index: usize,
    cur_next_addr: u64,
    cur_remaining_pages: u64,
    // Reserved frames pool for critical boot-time allocations.
    reserved: ReservedPool,
    tracking_enabled: bool,
}

impl BootFrameAllocator {
    pub fn empty() -> Self {
        Self {
            base_ptr: core::ptr::null(),
            desc_size: 0,
            count: 0,
            cur_index: 0,
            cur_next_addr: 0,
            cur_remaining_pages: 0,
            reserved: ReservedPool::new(),
            tracking_enabled: false,
        }
    }

    pub unsafe fn from_handoff(h: &theseus_shared::handoff::Handoff) -> Self {
        log_debug!("Frame allocator: from_handoff begin");
        let base_ptr = h.memory_map_buffer_ptr as *const u8;
        let desc_size = h.memory_map_descriptor_size as usize;
        let count = h.memory_map_entries as usize;
        log_trace!(
            "Frame allocator: base_ptr={:#x} desc_size={} count={}",
            base_ptr as u64,
            desc_size,
            count
        );
        // Memory map diagnostics: Compute summary statistics of the UEFI memory map
        // to verify the allocator sees the full system RAM. This helps debug memory
        // allocation issues and confirms we have sufficient conventional memory.
        let mut total_pages: u128 = 0;
        let mut conventional_pages: u128 = 0;
        for i in 0..count {
            let p = base_ptr.add(i * desc_size);
            let desc = MemoryDescriptorView::new(p);
            let typ = desc.kind();
            let num_pages = desc.page_count() as u128;
            total_pages = total_pages.wrapping_add(num_pages);
            if typ == UEFI_CONVENTIONAL_MEMORY {
                conventional_pages = conventional_pages.wrapping_add(num_pages);
            }
        }
        log_debug!(
            "Memory map: total_pages={:#x} conventional_pages={:#x}",
            total_pages,
            conventional_pages
        );

        let mut s = Self {
            base_ptr,
            desc_size,
            count,
            cur_index: 0,
            cur_next_addr: 0,
            cur_remaining_pages: 0,
            reserved: ReservedPool::new(),
            tracking_enabled: false,
        };
        s.advance_to_next_region();
        s
    }

    /// Enable boot-time allocation tracking. Each frame handed out after this
    /// call is immediately recorded as "consumed" in the persistent allocator
    /// bootstrap log so it will never be returned to the free pool later.
    pub fn enable_tracking(&mut self) {
        self.tracking_enabled = true;
    }

    /// Reserve up to `n` frames from the allocator and store them into the
    /// internal reserved pool for critical kernel use.
    pub fn reserve_frames(&mut self, mut n: usize) -> usize {
        let mut got = 0usize;
        while n > 0 && !self.reserved.is_full() {
            if let Some(frame) = self.allocate_frame() {
                let pa = frame.start_address().as_u64();
                if self.reserved.push(pa) {
                    got += 1;
                    n -= 1;
                } else {
                    // Pool filled between the loop condition and push; rewind allocator state
                    // so the frame is handed out normally on the next request.
                    self.cur_next_addr = self
                        .cur_next_addr
                        .saturating_sub(crate::memory::PAGE_SIZE as u64);
                    self.cur_remaining_pages = self.cur_remaining_pages.saturating_add(1);
                    break;
                }
            } else {
                break;
            }
        }
        got
    }

    /// Allocate a frame from the reserved pool (LIFO allocation).
    pub fn allocate_reserved_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        self.reserved
            .pop()
            .map(|pa| PhysFrame::containing_address(PhysAddr::new(pa)))
    }

    unsafe fn advance_to_next_region(&mut self) {
        while self.cur_index < self.count {
            let p = self.base_ptr.add(self.cur_index * self.desc_size);
            // UEFI memory descriptors are laid out as defined in the spec:
            // type @ 0, physical start @ 8, number of pages @ 24. We read the fields
            // using helper accessors to avoid struct definitions that would pull in `uefi`.
            let desc = MemoryDescriptorView::new(p);
            let typ = desc.kind();
            let phys_start = desc.physical_start();
            let num_pages = desc.page_count();
            if self.cur_index < 3 {
                log_trace!(
                    "desc[{}] @ {:#x}: type={} start={:#x} pages={}",
                    self.cur_index,
                    p as u64,
                    typ,
                    phys_start,
                    num_pages
                );
            }
            self.cur_index += 1;
            if typ == UEFI_CONVENTIONAL_MEMORY && num_pages > 0 {
                let aligned_start = phys_start & !((crate::memory::PAGE_SIZE as u64) - 1);
                let adj_pages = if aligned_start > phys_start {
                    // If aligning up skipped the first partial page, reduce the count so
                    // we never hand out a frame whose first bytes sit outside the descriptor.
                    num_pages.saturating_sub(1)
                } else {
                    num_pages
                };
                if adj_pages == 0 {
                    continue;
                }
                // Avoid returning physical frame 0 (page 0); skip the first page if region starts at 0
                if aligned_start == 0 {
                    if adj_pages <= 1 {
                        continue;
                    }
                    // Skip physical frame 0 entirely to avoid handing out a null-mappable
                    // page to subsystems that treat address 0 as special.
                    self.cur_next_addr = aligned_start + crate::memory::PAGE_SIZE as u64;
                    self.cur_remaining_pages = adj_pages.saturating_sub(1);
                } else {
                    self.cur_next_addr = aligned_start;
                    self.cur_remaining_pages = adj_pages;
                }
                log_trace!(
                    "Frame allocator region: start={:#x} pages={}",
                    self.cur_next_addr,
                    self.cur_remaining_pages
                );
                return;
            }
        }
        self.cur_remaining_pages = 0;
    }
}

unsafe impl FrameAllocator<Size4KiB> for BootFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        loop {
            if self.cur_remaining_pages == 0 {
                // Safe here because we only read from the UEFI memory map provided in handoff
                unsafe {
                    self.advance_to_next_region();
                }
                if self.cur_remaining_pages == 0 {
                    return None;
                }
            }
            let addr = self.cur_next_addr;
            self.cur_next_addr = self
                .cur_next_addr
                .saturating_add(crate::memory::PAGE_SIZE as u64);
            self.cur_remaining_pages = self.cur_remaining_pages.saturating_sub(1);
            if self.tracking_enabled {
                physical_memory::record_boot_consumed_region(physical_memory::consumed(
                    addr,
                    crate::memory::PAGE_SIZE as u64,
                ));
            }
            return Some(PhysFrame::containing_address(PhysAddr::new(addr)));
        }
    }
}

impl FrameSource for BootFrameAllocator {
    fn alloc_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        FrameAllocator::<Size4KiB>::allocate_frame(self)
    }

    fn alloc_page_table_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        self.allocate_reserved_frame()
            .or_else(|| FrameAllocator::<Size4KiB>::allocate_frame(self))
    }
}

#[inline(always)]
unsafe fn read_u32(ptr: *const u8, offset: usize) -> u32 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u32)
}

#[inline(always)]
unsafe fn read_u64(ptr: *const u8, offset: usize) -> u64 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u64)
}
