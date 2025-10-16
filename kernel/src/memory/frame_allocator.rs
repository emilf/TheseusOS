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
//! - `reserved`: Array storing physical addresses of reserved frames
//! - `reserved_count`: Number of frames currently in the reserved pool
//! - LIFO allocation: Last frame reserved is first frame allocated
//! - Fallback behavior: If reserved pool empty, falls back to general pool

use crate::{log_debug, log_trace};
use x86_64::{
    structures::paging::{FrameAllocator, PhysFrame, Size4KiB},
    PhysAddr,
};
use crate::physical_memory;

const UEFI_CONVENTIONAL_MEMORY: u32 = 7; // UEFI spec: conventional memory type

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
    // Reserved frames pool for critical boot-time allocations. Stores physical
    // addresses of frames that have been removed from the general pool and are
    // held back for critical kernel needs (page tables, IST stacks, etc.).
    reserved: [u64; 16],
    reserved_count: usize,
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
            reserved: [0u64; 16],
            reserved_count: 0,
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
            base_ptr as u64, desc_size, count
        );
        // Memory map diagnostics: Compute summary statistics of the UEFI memory map
        // to verify the allocator sees the full system RAM. This helps debug memory
        // allocation issues and confirms we have sufficient conventional memory.
        let mut total_pages: u128 = 0;
        let mut conventional_pages: u128 = 0;
        for i in 0..count {
            let p = base_ptr.add(i * desc_size);
            let typ = read_u32(p, 0);
            let num_pages = read_u64(p, 24) as u128;
            total_pages = total_pages.wrapping_add(num_pages);
            if typ == UEFI_CONVENTIONAL_MEMORY {
                conventional_pages = conventional_pages.wrapping_add(num_pages);
            }
        }
        log_debug!(
            "Memory map: total_pages={:#x} conventional_pages={:#x}",
            total_pages, conventional_pages
        );

        let mut s = Self {
            base_ptr,
            desc_size,
            count,
            cur_index: 0,
            cur_next_addr: 0,
            cur_remaining_pages: 0,
            reserved: [0u64; 16],
            reserved_count: 0,
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
        while n > 0 && self.reserved_count < self.reserved.len() {
            if let Some(frame) = self.allocate_frame() {
                let pa = frame.start_address().as_u64();
                self.reserved[self.reserved_count] = pa;
                self.reserved_count += 1;
                got += 1;
                n -= 1;
            } else {
                break;
            }
        }
        got
    }

    /// Allocate a frame from the reserved pool (LIFO allocation).
    pub fn allocate_reserved_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        if self.reserved_count == 0 {
            return None;
        }
        self.reserved_count -= 1;
        let pa = self.reserved[self.reserved_count];
        Some(PhysFrame::containing_address(PhysAddr::new(pa)))
    }

    unsafe fn advance_to_next_region(&mut self) {
        while self.cur_index < self.count {
            let p = self.base_ptr.add(self.cur_index * self.desc_size);
            let typ = read_u32(p, 0);
            let phys_start = read_u64(p, 8);
            let num_pages = read_u64(p, 24);
            if self.cur_index < 3 {
                log_trace!(
                    "desc[{}] @ {:#x}: type={} start={:#x} pages={}",
                    self.cur_index, p as u64, typ, phys_start, num_pages
                );
            }
            self.cur_index += 1;
            if typ == UEFI_CONVENTIONAL_MEMORY && num_pages > 0 {
                let aligned_start = phys_start & !((crate::memory::PAGE_SIZE as u64) - 1);
                let adj_pages = if aligned_start > phys_start {
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
                    self.cur_next_addr = aligned_start + crate::memory::PAGE_SIZE as u64;
                    self.cur_remaining_pages = adj_pages.saturating_sub(1);
                } else {
                    self.cur_next_addr = aligned_start;
                    self.cur_remaining_pages = adj_pages;
                }
                log_trace!("Frame allocator region: start={:#x} pages={}", self.cur_next_addr, self.cur_remaining_pages);
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
        self
            .allocate_reserved_frame()
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
