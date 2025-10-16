//! Persistent physical memory manager.
//!
//! During early boot we rely on `BootFrameAllocator`, which simply walks the
//! UEFI memory map and hands out frames monotonically. Once paging is active and
//! the kernel heap exists we need a reclaimable allocator that can both hand out
//! and free frames for the remainder of the kernel’s lifetime. This module owns
//! that allocator. At the moment the APIs are implemented but callers still need
//! to switch from the boot-time allocator—future work will wire that up.

use alloc::vec::Vec;
use spin::Mutex;
use theseus_shared::handoff::Handoff;

use crate::memory::{phys_offset_is_active, phys_to_virt_pa, PHYS_OFFSET, PAGE_SIZE};
use x86_64::{
    structures::paging::{FrameAllocator, PhysFrame, Size4KiB},
    PhysAddr,
};

/// Frame size (4 KiB) expressed as `u64`.
const FRAME_SIZE: u64 = PAGE_SIZE as u64;

/// Physical frame number helper (PFN = physical address / 4 KiB).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Pfn(u64);

impl Pfn {
    fn containing(addr: u64) -> Self {
        Self(addr / FRAME_SIZE)
    }

    fn start_address(self) -> u64 {
        self.0 * FRAME_SIZE
    }
}

/// Region descriptor derived from the UEFI memory map.
#[derive(Debug, Clone)]
struct Region {
    start: Pfn,
    end: Pfn,
    kind: RegionKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RegionKind {
    Free,
    Reserved,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllocError {
    OutOfMemory,
    DoubleFree,
    UnknownFrame,
    PhysOffsetInactive,
    NoMemoryMap,
}

pub type AllocResult<T> = Result<T, AllocError>;

/// Bitmap-backed physical frame allocator that supports allocate + free.
pub struct PhysicalMemoryManager {
    base_pfn: Pfn,
    bitmap: &'static mut [u64],
    total_frames: u64,
    free_frames: u64,
}

impl PhysicalMemoryManager {
    #[inline]
    fn bit_position(&self, idx: usize) -> Option<(usize, u32)> {
        if idx as u64 >= self.total_frames {
            return None;
        }
        Some((idx / 64, (idx % 64) as u32))
    }

    #[inline]
    fn test_bit(&self, idx: usize) -> bool {
        if let Some((word, bit)) = self.bit_position(idx) {
            (self.bitmap[word] & (1u64 << bit)) != 0
        } else {
            true
        }
    }

    #[inline]
    fn set_bit(&mut self, idx: usize) -> bool {
        if let Some((word, bit)) = self.bit_position(idx) {
            let mask = 1u64 << bit;
            let was_set = (self.bitmap[word] & mask) != 0;
            self.bitmap[word] |= mask;
            was_set
        } else {
            true
        }
    }

    #[inline]
    fn clear_bit(&mut self, idx: usize) -> bool {
        if let Some((word, bit)) = self.bit_position(idx) {
            let mask = 1u64 << bit;
            let was_set = (self.bitmap[word] & mask) != 0;
            self.bitmap[word] &= !mask;
            was_set
        } else {
            false
        }
    }

    fn new(base_pfn: Pfn, frame_count: u64, bitmap: &'static mut [u64]) -> Self {
        crate::display::kernel_write_line("[phys/manager] using provided bitmap storage");
        let required_words = ((frame_count + 63) / 64) as usize;
        assert!(bitmap.len() >= required_words, "bitmap storage too small");
        for word in bitmap.iter_mut().skip(required_words) {
            *word = u64::MAX;
        }
        if frame_count % 64 != 0 {
            let valid = (frame_count % 64) as u32;
            let mask = !((1u64 << valid) - 1);
            if let Some(last) = bitmap.get_mut(required_words - 1) {
                *last |= mask;
            }
        }
        crate::display::kernel_write_line("[phys/manager] bitmap ready");
        Self {
            base_pfn,
            bitmap,
            total_frames: frame_count,
            free_frames: frame_count,
        }
    }

    fn reserve_frame(&mut self, pfn: Pfn) {
        if pfn < self.base_pfn {
            return;
        }
        let idx = (pfn.0 - self.base_pfn.0) as usize;
        if idx as u64 >= self.total_frames {
            return;
        }
        if !self.set_bit(idx) {
            self.free_frames = self.free_frames.saturating_sub(1);
        }
    }

    fn reserve_range(&mut self, start: Pfn, pages: u64) {
        for offset in 0..pages {
            self.reserve_frame(Pfn(start.0 + offset));
        }
    }

    fn mark_free(&mut self, pfn: Pfn) -> AllocResult<()> {
        if pfn < self.base_pfn {
            return Err(AllocError::UnknownFrame);
        }
        let idx = (pfn.0 - self.base_pfn.0) as usize;
        if idx as u64 >= self.total_frames {
            return Err(AllocError::UnknownFrame);
        }
        if !self.test_bit(idx) {
            return Err(AllocError::DoubleFree);
        }
        self.clear_bit(idx);
        self.free_frames = self.free_frames.saturating_add(1);
        Ok(())
    }

    fn pop_first_free(&mut self) -> AllocResult<Pfn> {
        for (word_index, word) in self.bitmap.iter_mut().enumerate() {
            if *word == u64::MAX {
                continue;
            }
            let inverted = !*word;
            let bit = inverted.trailing_zeros() as usize;
            let idx = word_index * 64 + bit;
            if idx as u64 >= self.total_frames {
                break;
            }
            *word |= 1u64 << bit;
            self.free_frames = self.free_frames.saturating_sub(1);
            return Ok(Pfn(self.base_pfn.0 + idx as u64));
        }
        Err(AllocError::OutOfMemory)
    }
}

/// Global singleton guarding the allocator.
static PHYS_MANAGER: Mutex<Option<PhysicalMemoryManager>> = Mutex::new(None);

const BOOT_CONSUMED_CAPACITY_BYTES: usize = 16 * 1024;
const BOOT_CONSUMED_CAPACITY: usize = BOOT_CONSUMED_CAPACITY_BYTES / core::mem::size_of::<ConsumedRegion>();

#[derive(Clone, Copy)]
struct BootConsumedLog {
    regions: [ConsumedRegion; BOOT_CONSUMED_CAPACITY],
    len: usize,
}

impl BootConsumedLog {
    const fn new() -> Self {
        Self {
            regions: [ConsumedRegion::EMPTY; BOOT_CONSUMED_CAPACITY],
            len: 0,
        }
    }

    fn push(&mut self, region: ConsumedRegion) {
        if region.size == 0 {
            return;
        }
        if self.len >= BOOT_CONSUMED_CAPACITY {
            panic!("boot consumed log overflow");
        }
        self.regions[self.len] = region;
        self.len += 1;
    }

    fn extend(&mut self, regions: &[ConsumedRegion]) {
        for &region in regions {
            self.push(region);
        }
    }
}

static BOOT_CONSUMED: Mutex<BootConsumedLog> = Mutex::new(BootConsumedLog::new());

/// Initialise the persistent allocator using the UEFI memory map plus any
/// pre-consumed regions (page tables, stacks, etc.).
pub fn init_from_handoff<F>(
    handoff: &Handoff,
    consumed: &[ConsumedRegion],
    bitmap_alloc: F,
) -> AllocResult<()>
where
    F: FnOnce(usize) -> &'static mut [u64],
{
    if is_initialised() {
        return Err(AllocError::UnknownFrame);
    }
    let memmap_ptr = accessible_memmap_ptr(handoff)?;
    if memmap_ptr.is_null() {
        return Err(AllocError::NoMemoryMap);
    }
    crate::display::kernel_write_line("[phys/init] collecting regions");
    let regions = collect_regions(handoff, memmap_ptr);
    crate::display::kernel_write_line("[phys/init] regions collected");
    let (base_pfn, frame_count) = compute_bounds(&regions)?;

    crate::display::kernel_write_line("[phys/init] constructing bitmap allocator");
    crate::display::kernel_write_line("  base_pfn=");
    theseus_shared::print_hex_u64_0xe9!(base_pfn.start_address());
    crate::display::kernel_write_line("  frames=");
    theseus_shared::print_hex_u64_0xe9!(frame_count);
    crate::display::kernel_write_line("\n");
    let words = ((frame_count + 63) / 64) as usize;
    let bitmap_storage = bitmap_alloc(words);
    let mut manager = PhysicalMemoryManager::new(base_pfn, frame_count, bitmap_storage);
    crate::display::kernel_write_line("[phys/init] allocator constructed");
    for region in &regions {
        if matches!(region.kind, RegionKind::Reserved) {
            manager.reserve_range(region.start, region.end.0.saturating_sub(region.start.0));
        }
    }
    reserve_boot_consumed(&mut manager);
    for region in consumed {
        reserve_region(&mut manager, *region);
    }

    let mut guard = PHYS_MANAGER.lock();
    *guard = Some(manager);
    Ok(())
}

fn reserve_boot_consumed(manager: &mut PhysicalMemoryManager) {
    let mut boot = BOOT_CONSUMED.lock();
    if boot.len == 0 {
        return;
    }
    let len = boot.len;
    let merged = merge_regions_in_place(&mut boot.regions, len);
    for entry in boot.regions.iter_mut().take(len) {
        *entry = ConsumedRegion::EMPTY;
    }
    boot.len = 0;
    drop(boot);
    for region in merged {
        reserve_region(manager, region);
    }
}

fn merge_regions_in_place(regions: &mut [ConsumedRegion], len: usize) -> Vec<ConsumedRegion> {
    let mut entries: Vec<ConsumedRegion> = regions
        .iter()
        .take(len)
        .copied()
        .filter(|r| r.size != 0)
        .collect();
    if entries.is_empty() {
        return Vec::new();
    }
    entries.sort_by(|a, b| a.start.cmp(&b.start));

    let mut merged: Vec<ConsumedRegion> = Vec::with_capacity(entries.len());
    for region in entries {
        if let Some(last) = merged.last_mut() {
            let last_end = last.start.saturating_add(last.size);
            let region_end = region.start.saturating_add(region.size);
            if region.start <= last_end {
                let new_end = core::cmp::max(last_end, region_end);
                last.size = new_end.saturating_sub(last.start);
                continue;
            }
        }
        merged.push(region);
    }
    merged
}

pub fn record_boot_consumed(regions: Vec<ConsumedRegion>) {
    if regions.is_empty() {
        return;
    }
    if let Some(manager) = PHYS_MANAGER.lock().as_mut() {
        for region in regions {
            reserve_region(manager, region);
        }
    } else {
        BOOT_CONSUMED.lock().extend(&regions);
    }
}

pub fn record_boot_consumed_region(region: ConsumedRegion) {
    if let Some(manager) = PHYS_MANAGER.lock().as_mut() {
        reserve_region(manager, region);
    } else {
        BOOT_CONSUMED.lock().push(region);
    }
}

pub fn drain_boot_consumed() -> Vec<ConsumedRegion> {
    let mut boot = BOOT_CONSUMED.lock();
    let len = boot.len;
    if len == 0 {
        return Vec::new();
    }
    let merged = merge_regions_in_place(&mut boot.regions, len);
    for entry in boot.regions.iter_mut().take(len) {
        *entry = ConsumedRegion::EMPTY;
    }
    boot.len = 0;
    merged
}

pub fn is_initialised() -> bool {
    PHYS_MANAGER.lock().is_some()
}

pub fn alloc_frame() -> AllocResult<u64> {
    let mut guard = PHYS_MANAGER.lock();
    let manager = guard.as_mut().expect("physical allocator not initialised");
    manager.pop_first_free().map(Pfn::start_address)
}

pub fn free_frame(pa: u64) -> AllocResult<()> {
    let mut guard = PHYS_MANAGER.lock();
    let manager = guard.as_mut().expect("physical allocator not initialised");
    let pfn = Pfn::containing(pa);
    manager.mark_free(pfn)
}

pub fn dump_state() {
    if let Some(mgr) = PHYS_MANAGER.lock().as_ref() {
        crate::display::kernel_write_line("[phys] PhysicalMemoryManager state:");
        crate::display::kernel_write_line("  base_pfn=");
        theseus_shared::print_hex_u64_0xe9!(mgr.base_pfn.start_address());
        crate::display::kernel_write_line("  total_frames=");
        theseus_shared::print_hex_u64_0xe9!(mgr.total_frames);
        crate::display::kernel_write_line("  free_frames=");
        theseus_shared::print_hex_u64_0xe9!(mgr.free_frames);
        crate::display::kernel_write_line("\n");
    } else {
        crate::display::kernel_write_line("[phys] PhysicalMemoryManager not initialised\n");
    }
}

pub fn base_pfn() -> Option<u64> {
    PHYS_MANAGER
        .lock()
        .as_ref()
        .map(|mgr| mgr.base_pfn.start_address())
}

/// Snapshot of persistent allocator state.
pub struct Stats {
    pub base_pfn: u64,
    pub total_frames: u64,
    pub free_frames: u64,
}

pub fn stats() -> Option<Stats> {
    PHYS_MANAGER.lock().as_ref().map(|mgr| Stats {
        base_pfn: mgr.base_pfn.start_address(),
        total_frames: mgr.total_frames,
        free_frames: mgr.free_frames,
    })
}

pub struct PersistentFrameAllocator;

unsafe impl FrameAllocator<Size4KiB> for PersistentFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        alloc_frame()
            .ok()
            .map(|pa| PhysFrame::containing_address(PhysAddr::new(pa)))
    }
}

impl crate::memory::FrameSource for PersistentFrameAllocator {
    fn alloc_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        FrameAllocator::<Size4KiB>::allocate_frame(self)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ConsumedRegion {
    pub start: u64,
    pub size: u64,
}

impl ConsumedRegion {
    pub const EMPTY: Self = Self { start: 0, size: 0 };
}

pub fn consumed(start: u64, size: u64) -> ConsumedRegion {
    ConsumedRegion { start, size }
}

#[cfg(test)]
pub fn reset() {
    *PHYS_MANAGER.lock() = None;
    *BOOT_CONSUMED.lock() = BootConsumedLog::new();
}

fn accessible_memmap_ptr(handoff: &Handoff) -> AllocResult<*const u8> {
    let ptr = handoff.memory_map_buffer_ptr;
    if ptr == 0 {
        return Err(AllocError::NoMemoryMap);
    }
    if ptr >= PHYS_OFFSET {
        Ok(ptr as *const u8)
    } else if phys_offset_is_active() {
        Ok(phys_to_virt_pa(ptr) as *const u8)
    } else {
        // Identity mapping is still active in early boot, so the physical
        // pointer is accessible directly.
        Ok(ptr as *const u8)
    }
}

fn collect_regions(handoff: &Handoff, base_ptr: *const u8) -> Vec<Region> {
    const UEFI_CONVENTIONAL_MEMORY: u32 = 7;
    let mut regions = Vec::new();
    let desc_size = handoff.memory_map_descriptor_size as usize;
    let count = handoff.memory_map_entries as usize;
    for i in 0..count {
        let p = unsafe { base_ptr.add(i * desc_size) };
        let view = unsafe { MemoryDescriptorView::new(p, handoff.memory_map_descriptor_size) };
        let start = Pfn::containing(view.phys_start);
        let end = Pfn::containing(view.phys_start + view.num_pages * FRAME_SIZE);
        let kind = if view.ty == UEFI_CONVENTIONAL_MEMORY {
            RegionKind::Free
        } else {
            RegionKind::Reserved
        };
        regions.push(Region { start, end, kind });
    }
    regions
}

fn compute_bounds(regions: &[Region]) -> AllocResult<(Pfn, u64)> {
    let mut free_regions: Vec<&Region> = regions
        .iter()
        .filter(|r| matches!(r.kind, RegionKind::Free))
        .collect();
    if free_regions.is_empty() {
        return Err(AllocError::OutOfMemory);
    }
    free_regions.sort_by(|a, b| a.start.cmp(&b.start));
    let base = free_regions.first().unwrap().start;
    let end = free_regions
        .iter()
        .map(|r| r.end)
        .max()
        .unwrap_or(base);
    Ok((base, end.0.saturating_sub(base.0)))
}

struct MemoryDescriptorView {
    ty: u32,
    phys_start: u64,
    num_pages: u64,
}

impl MemoryDescriptorView {
    unsafe fn new(ptr: *const u8, _size: u32) -> Self {
        Self {
            ty: read_u32(ptr, 0),
            phys_start: read_u64(ptr, 8),
            num_pages: read_u64(ptr, 24),
        }
    }
}

unsafe fn read_u32(ptr: *const u8, offset: usize) -> u32 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u32)
}

unsafe fn read_u64(ptr: *const u8, offset: usize) -> u64 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u64)
}

fn reserve_region(manager: &mut PhysicalMemoryManager, region: ConsumedRegion) {
    if region.size == 0 {
        return;
    }
    let start_pfn = Pfn::containing(region.start);
    let pages = (region.size + FRAME_SIZE - 1) / FRAME_SIZE;
    manager.reserve_range(start_pfn, pages);
}
