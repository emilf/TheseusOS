//! Persistent physical memory manager.
//!
//! During early boot we rely on `BootFrameAllocator`, which simply walks the
//! UEFI memory map and hands out frames monotonically. Once paging is active and
//! the kernel heap exists we need a reclaimable allocator that can both hand out
//! and free frames for the remainder of the kernel's lifetime. This module owns
//! that allocator. At the moment the APIs are implemented but callers still need
//! to switch from the boot-time allocator—future work will wire that up.
//!
//! # Architecture
//!
//! The physical memory manager uses a bitmap-based allocation scheme:
//! - Each bit represents one physical frame (4 KiB page)
//! - Bit = 1 means the frame is allocated or reserved
//! - Bit = 0 means the frame is free
//!
//! The allocator tracks all usable memory from the UEFI memory map and reserves
//! regions that are in use (kernel code, page tables, stacks, etc.). It supports
//! both allocation and freeing of frames, making it suitable for long-term use.
//!
//! # Initialization
//!
//! Call [`init_from_handoff`] once during early boot after the heap is available.
//! This will parse the UEFI memory map, construct the bitmap, and mark all
//! consumed regions as allocated.
//!
//! # Usage
//!
//! - [`alloc_frame`]: Allocate a single physical frame
//! - [`free_frame`]: Return a frame to the free pool
//! - [`stats`]: Query allocator statistics
//! - [`record_boot_consumed_region`]: Mark early-boot allocations as consumed

use crate::{log_debug, log_error, log_info};
use alloc::vec::Vec;
use spin::Mutex;
use theseus_shared::handoff::Handoff;

use crate::memory::{phys_offset_is_active, phys_to_virt_pa, PAGE_SIZE, PHYS_OFFSET};
use x86_64::{
    structures::paging::{FrameAllocator, PhysFrame, Size4KiB},
    PhysAddr,
};

/// Frame size (4 KiB) expressed as `u64`.
const FRAME_SIZE: u64 = PAGE_SIZE as u64;

/// Physical frame number (PFN) — the index of a 4 KiB physical page.
///
/// A PFN is computed by dividing a physical address by the page size (4 KiB).
/// This allows us to represent physical frames compactly as simple indices,
/// which is especially useful for bitmap-based allocation schemes.
///
/// # Example
/// ```ignore
/// let pfn = Pfn::containing(0x1000);  // PFN = 1
/// assert_eq!(pfn.start_address(), 0x1000);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Pfn(u64);

impl Pfn {
    /// Returns the PFN containing the given physical address.
    ///
    /// This performs a simple division by the frame size (4 KiB).
    fn containing(addr: u64) -> Self {
        Self(addr / FRAME_SIZE)
    }

    /// Returns the physical address at the start of this frame.
    ///
    /// This is the inverse of `containing`: it multiplies the PFN
    /// by the frame size to get back to a physical address.
    fn start_address(self) -> u64 {
        self.0 * FRAME_SIZE
    }
}

/// Memory region descriptor derived from the UEFI memory map.
///
/// Each region represents a contiguous range of physical memory with a
/// specific type (free or reserved). During initialization, we parse the
/// UEFI memory map into a collection of these regions.
#[derive(Debug, Clone)]
struct Region {
    /// Starting PFN of this region (inclusive).
    start: Pfn,
    /// Ending PFN of this region (exclusive).
    end: Pfn,
    /// Whether this region is available for allocation.
    kind: RegionKind,
}

/// Type of memory region.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RegionKind {
    /// Free memory available for allocation (UEFI conventional memory).
    Free,
    /// Reserved memory not available for allocation (firmware, MMIO, etc.).
    Reserved,
}

/// Errors that can occur during physical memory allocation or deallocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllocError {
    /// No free frames available.
    OutOfMemory,
    /// Attempted to free a frame that was already free.
    DoubleFree,
    /// Frame address is outside the managed range.
    UnknownFrame,
    /// Physical offset mapping is not active (can't access memory map).
    PhysOffsetInactive,
    /// UEFI memory map is not available.
    NoMemoryMap,
}

/// Result type for allocation operations.
pub type AllocResult<T> = Result<T, AllocError>;

/// Bitmap-backed physical frame allocator that supports allocate + free.
///
/// This allocator manages all physical memory discovered from the UEFI memory
/// map using a bitmap where each bit represents one 4 KiB frame. The bitmap
/// is allocated from the kernel heap during initialization.
///
/// # Implementation Details
///
/// - The bitmap uses u64 words, so each word tracks 64 frames.
/// - Free frames are represented by 0 bits, allocated frames by 1 bits.
/// - Allocation uses a first-fit strategy: we scan the bitmap from the
///   beginning and return the first free frame found.
/// - The allocator tracks the base PFN to handle memory that doesn't start
///   at physical address 0.
///
/// # Thread Safety
///
/// The allocator is wrapped in a global `Mutex` to ensure thread-safe access.
pub struct PhysicalMemoryManager {
    /// The PFN of the first frame tracked by this allocator.
    base_pfn: Pfn,
    /// Bitmap storage: each bit represents one frame.
    /// - 0 = free, 1 = allocated/reserved
    bitmap: &'static mut [u64],
    /// Total number of frames tracked by this allocator.
    total_frames: u64,
    /// Number of frames currently available for allocation.
    free_frames: u64,
}

impl PhysicalMemoryManager {
    /// Computes the bitmap word index and bit position for a frame index.
    ///
    /// Returns `None` if the index is out of bounds.
    ///
    /// # Parameters
    /// - `idx`: Frame index relative to `base_pfn`
    ///
    /// # Returns
    /// - `Some((word_index, bit_position))` if valid
    /// - `None` if index >= total_frames
    #[inline]
    fn bit_position(&self, idx: usize) -> Option<(usize, u32)> {
        if idx as u64 >= self.total_frames {
            return None;
        }
        // Each u64 holds 64 bits, so word index = idx / 64
        // Bit position within the word = idx % 64
        Some((idx / 64, (idx % 64) as u32))
    }

    /// Tests whether a frame is allocated (bit = 1) or free (bit = 0).
    ///
    /// Returns `true` for out-of-bounds indices (treat as allocated).
    #[inline]
    fn test_bit(&self, idx: usize) -> bool {
        if let Some((word, bit)) = self.bit_position(idx) {
            (self.bitmap[word] & (1u64 << bit)) != 0
        } else {
            true // Out of bounds = treat as allocated
        }
    }

    /// Marks a frame as allocated by setting its bit to 1.
    ///
    /// # Returns
    /// - `true` if the bit was already set (frame was already allocated)
    /// - `false` if the bit was cleared (frame was free)
    #[inline]
    fn set_bit(&mut self, idx: usize) -> bool {
        if let Some((word, bit)) = self.bit_position(idx) {
            let mask = 1u64 << bit;
            let was_set = (self.bitmap[word] & mask) != 0;
            self.bitmap[word] |= mask;
            was_set
        } else {
            true // Out of bounds = pretend it was already set
        }
    }

    /// Marks a frame as free by clearing its bit to 0.
    ///
    /// # Returns
    /// - `true` if the bit was set (frame was allocated)
    /// - `false` if the bit was already cleared or out of bounds
    #[inline]
    fn clear_bit(&mut self, idx: usize) -> bool {
        if let Some((word, bit)) = self.bit_position(idx) {
            let mask = 1u64 << bit;
            let was_set = (self.bitmap[word] & mask) != 0;
            self.bitmap[word] &= !mask;
            was_set
        } else {
            false // Out of bounds = can't clear
        }
    }

    /// Constructs a new physical memory manager.
    ///
    /// # Parameters
    /// - `base_pfn`: The first PFN managed by this allocator
    /// - `frame_count`: Total number of frames to manage
    /// - `bitmap`: Pre-allocated bitmap storage (must be large enough)
    ///
    /// # Panics
    /// Panics if the bitmap is too small for the given frame count.
    ///
    /// # Initialization
    /// All frames start as free (bits = 0). Any excess bits beyond
    /// `frame_count` are marked as allocated to prevent allocating
    /// non-existent frames.
    fn new(base_pfn: Pfn, frame_count: u64, bitmap: &'static mut [u64]) -> Self {
        log_debug!("Physical memory manager: using provided bitmap storage");

        // Calculate required bitmap size
        let required_words = ((frame_count + 63) / 64) as usize;
        assert!(bitmap.len() >= required_words, "bitmap storage too small");

        // Mark any excess words beyond required_words as fully allocated
        for word in bitmap.iter_mut().skip(required_words) {
            *word = u64::MAX;
        }

        // If frame_count isn't a multiple of 64, mark the unused bits
        // in the last word as allocated to prevent invalid allocations
        if frame_count % 64 != 0 {
            let valid = (frame_count % 64) as u32;
            let mask = !((1u64 << valid) - 1); // Set upper bits to 1
            if let Some(last) = bitmap.get_mut(required_words - 1) {
                *last |= mask;
            }
        }

        log_debug!("Physical memory manager: bitmap ready");
        Self {
            base_pfn,
            bitmap,
            total_frames: frame_count,
            free_frames: frame_count, // All start free
        }
    }

    /// Reserves a single physical frame, marking it as unavailable for allocation.
    ///
    /// This is used during initialization to mark firmware-reserved regions,
    /// kernel code/data, and other pre-allocated structures.
    ///
    /// # Parameters
    /// - `pfn`: The physical frame number to reserve
    ///
    /// # Behavior
    /// - If the frame is outside the managed range, this is a no-op
    /// - If the frame was already reserved, the free count is not decremented
    fn reserve_frame(&mut self, pfn: Pfn) {
        if pfn < self.base_pfn {
            return; // Before our range
        }
        let idx = (pfn.0 - self.base_pfn.0) as usize;
        if idx as u64 >= self.total_frames {
            return; // After our range
        }
        // set_bit returns true if already set
        if !self.set_bit(idx) {
            // Frame was free, now allocated
            self.free_frames = self.free_frames.saturating_sub(1);
        }
    }

    /// Reserves a contiguous range of physical frames.
    ///
    /// # Parameters
    /// - `start`: Starting PFN
    /// - `pages`: Number of frames to reserve
    fn reserve_range(&mut self, start: Pfn, pages: u64) {
        for offset in 0..pages {
            self.reserve_frame(Pfn(start.0 + offset));
        }
    }

    /// Marks a previously-allocated frame as free.
    ///
    /// # Parameters
    /// - `pfn`: The physical frame number to free
    ///
    /// # Errors
    /// - [`AllocError::UnknownFrame`] if the frame is outside the managed range
    /// - [`AllocError::DoubleFree`] if the frame was already free
    fn mark_free(&mut self, pfn: Pfn) -> AllocResult<()> {
        if pfn < self.base_pfn {
            return Err(AllocError::UnknownFrame);
        }
        let idx = (pfn.0 - self.base_pfn.0) as usize;
        if idx as u64 >= self.total_frames {
            return Err(AllocError::UnknownFrame);
        }
        // Check if frame is currently allocated (bit = 1)
        if !self.test_bit(idx) {
            // Frame is already free (bit = 0)
            return Err(AllocError::DoubleFree);
        }
        // Clear the bit and increment free count
        self.clear_bit(idx);
        self.free_frames = self.free_frames.saturating_add(1);
        Ok(())
    }

    /// Allocates the first available free frame using a first-fit strategy.
    ///
    /// Scans the bitmap from the beginning and returns the first frame with
    /// a cleared bit. This is simple and fast for typical allocation patterns.
    ///
    /// # Returns
    /// - `Ok(pfn)` if a free frame was found and allocated
    /// - [`AllocError::OutOfMemory`] if no free frames are available
    fn pop_first_free(&mut self) -> AllocResult<Pfn> {
        for (word_index, word) in self.bitmap.iter_mut().enumerate() {
            // Skip words that are fully allocated (all bits = 1)
            if *word == u64::MAX {
                continue;
            }
            // Find the first zero bit in this word
            let inverted = !*word;
            let bit = inverted.trailing_zeros() as usize;
            let idx = word_index * 64 + bit;

            // Bounds check: ensure we don't allocate beyond total_frames
            if idx as u64 >= self.total_frames {
                break;
            }

            // Mark the frame as allocated
            *word |= 1u64 << bit;
            self.free_frames = self.free_frames.saturating_sub(1);
            return Ok(Pfn(self.base_pfn.0 + idx as u64));
        }
        Err(AllocError::OutOfMemory)
    }
}

/// Global singleton guarding the allocator.
///
/// The allocator is initialized once during boot and then accessed through
/// this mutex for the remainder of kernel execution.
static PHYS_MANAGER: Mutex<Option<PhysicalMemoryManager>> = Mutex::new(None);

/// Size of the boot-time consumed region log in bytes.
const BOOT_CONSUMED_CAPACITY_BYTES: usize = 16 * 1024;

/// Number of consumed region entries that can be logged before initialization.
const BOOT_CONSUMED_CAPACITY: usize =
    BOOT_CONSUMED_CAPACITY_BYTES / core::mem::size_of::<ConsumedRegion>();

/// Boot-time log of consumed memory regions.
///
/// Before the persistent allocator is initialized, we need to track which
/// physical memory regions have been consumed (e.g., for page tables, stacks).
/// This structure provides a fixed-capacity log that accumulates those regions.
///
/// Once the allocator is initialized, the log is drained and all consumed
/// regions are marked as reserved in the bitmap.
#[derive(Clone, Copy)]
struct BootConsumedLog {
    /// Fixed-capacity array of consumed regions.
    regions: [ConsumedRegion; BOOT_CONSUMED_CAPACITY],
    /// Number of valid entries in `regions`.
    len: usize,
}

impl BootConsumedLog {
    /// Creates a new empty log.
    const fn new() -> Self {
        Self {
            regions: [ConsumedRegion::EMPTY; BOOT_CONSUMED_CAPACITY],
            len: 0,
        }
    }

    /// Pushes a consumed region into the log.
    ///
    /// # Panics
    /// Panics if the log is full (capacity exceeded).
    fn push(&mut self, region: ConsumedRegion) {
        if region.size == 0 {
            return; // Ignore empty regions
        }
        if self.len >= BOOT_CONSUMED_CAPACITY {
            panic!("boot consumed log overflow");
        }
        self.regions[self.len] = region;
        self.len += 1;
    }

    /// Extends the log with multiple regions.
    fn extend(&mut self, regions: &[ConsumedRegion]) {
        for &region in regions {
            self.push(region);
        }
    }
}

/// Global log of consumed regions during early boot.
static BOOT_CONSUMED: Mutex<BootConsumedLog> = Mutex::new(BootConsumedLog::new());

/// Runs a closure with a view of the boot-time consumed region log.
///
/// This avoids allocating heap memory while still allowing callers to inspect
/// the regions recorded before the persistent allocator is initialised.
pub fn visit_boot_consumed<R, F>(f: F) -> R
where
    F: FnOnce(&[ConsumedRegion]) -> R,
{
    let log = BOOT_CONSUMED.lock();
    let slice = &log.regions[..log.len];
    f(slice)
}

/// Initialises the persistent allocator using the UEFI memory map plus any
/// pre-consumed regions (page tables, stacks, etc.).
///
/// This function should be called once during early boot after the kernel heap
/// is available. It parses the UEFI memory map, constructs a bitmap covering
/// all usable memory, and marks all consumed regions as allocated.
///
/// # Parameters
/// - `handoff`: Boot handoff structure containing UEFI memory map information
/// - `consumed`: Array of memory regions already consumed (kernel, page tables, etc.)
/// - `bitmap_alloc`: Callback to allocate bitmap storage (typically from the heap)
///
/// # Errors
/// - [`AllocError::UnknownFrame`] if already initialized
/// - [`AllocError::NoMemoryMap`] if the UEFI memory map is missing
/// - [`AllocError::OutOfMemory`] if no free regions are found
///
/// # Example
/// ```ignore
/// let consumed = vec![
///     consumed(kernel_start, kernel_size),
///     consumed(page_table_addr, page_table_size),
/// ];
/// init_from_handoff(&handoff, &consumed, |words| {
///     // Allocate bitmap from heap
///     vec![0u64; words].leak()
/// })?;
/// ```
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

    // Get a pointer to the UEFI memory map (either virtual or identity-mapped)
    let memmap_ptr = accessible_memmap_ptr(handoff)?;
    if memmap_ptr.is_null() {
        return Err(AllocError::NoMemoryMap);
    }

    log_debug!("Physical memory init: collecting regions");
    let regions = collect_regions(handoff, memmap_ptr);
    log_debug!("Physical memory init: regions collected");

    // Determine the range of memory we need to track
    let (base_pfn, frame_count) = compute_bounds(&regions)?;

    log_debug!(
        "Constructing bitmap allocator: base_pfn={:#x} frames={}",
        base_pfn.start_address(),
        frame_count
    );

    // Allocate bitmap storage and construct the manager
    let words = ((frame_count + 63) / 64) as usize;
    let bitmap_storage = bitmap_alloc(words);
    let mut manager = PhysicalMemoryManager::new(base_pfn, frame_count, bitmap_storage);

    log_info!("Physical memory allocator constructed");

    // Mark all reserved regions from the UEFI memory map
    for region in &regions {
        if matches!(region.kind, RegionKind::Reserved) {
            manager.reserve_range(region.start, region.end.0.saturating_sub(region.start.0));
        }
    }

    // Reserve regions consumed during early boot (before this allocator existed)
    reserve_boot_consumed(&mut manager);

    // Reserve explicitly provided consumed regions
    for region in consumed {
        reserve_region(&mut manager, *region);
    }

    // Install the manager as the global singleton
    let mut guard = PHYS_MANAGER.lock();
    *guard = Some(manager);
    Ok(())
}

/// Reserves all regions from the boot-time consumed log and clears the log.
///
/// This is called during initialization to mark all early-boot allocations
/// (page tables, stacks, etc.) as reserved in the bitmap.
fn reserve_boot_consumed(manager: &mut PhysicalMemoryManager) {
    let mut boot = BOOT_CONSUMED.lock();
    if boot.len == 0 {
        return; // Nothing to reserve
    }

    let len = boot.len;
    // Merge overlapping/adjacent regions to reduce reservation overhead
    let merged = merge_regions_in_place(&mut boot.regions, len);

    // Clear the log
    for entry in boot.regions.iter_mut().take(len) {
        *entry = ConsumedRegion::EMPTY;
    }
    boot.len = 0;
    drop(boot);

    // Reserve the merged regions
    for region in merged {
        reserve_region(manager, region);
    }
}

/// Merges overlapping or adjacent consumed regions to reduce fragmentation.
///
/// This function sorts the regions by start address and then merges any
/// that overlap or are adjacent. This is useful for reducing the number
/// of reservation operations during initialization.
///
/// # Parameters
/// - `regions`: Mutable slice of consumed regions to merge in-place
/// - `len`: Number of valid entries in `regions`
///
/// # Returns
/// A new `Vec` containing the merged regions (non-overlapping and sorted).
fn merge_regions_in_place(regions: &mut [ConsumedRegion], len: usize) -> Vec<ConsumedRegion> {
    // Collect non-empty regions into a vector
    let mut entries: Vec<ConsumedRegion> = regions
        .iter()
        .take(len)
        .copied()
        .filter(|r| r.size != 0)
        .collect();

    if entries.is_empty() {
        return Vec::new();
    }

    // Sort by start address
    entries.sort_by(|a, b| a.start.cmp(&b.start));

    let mut merged: Vec<ConsumedRegion> = Vec::with_capacity(entries.len());
    for region in entries {
        if let Some(last) = merged.last_mut() {
            let last_end = last.start.saturating_add(last.size);
            let region_end = region.start.saturating_add(region.size);

            // Check if this region overlaps or is adjacent to the last one
            if region.start <= last_end {
                // Merge: extend the last region to cover both
                let new_end = core::cmp::max(last_end, region_end);
                last.size = new_end.saturating_sub(last.start);
                continue;
            }
        }
        // No overlap: add as a new region
        merged.push(region);
    }
    merged
}

/// Records consumed memory regions, either immediately or for later processing.
///
/// If the persistent allocator is already initialized, the regions are marked
/// as reserved immediately. Otherwise, they are added to the boot-time log
/// for processing during initialization.
///
/// # Parameters
/// - `regions`: Vector of consumed regions to record
pub fn record_boot_consumed(regions: Vec<ConsumedRegion>) {
    if regions.is_empty() {
        return;
    }
    if let Some(manager) = PHYS_MANAGER.lock().as_mut() {
        // Allocator is initialized: reserve immediately
        for region in regions {
            reserve_region(manager, region);
        }
    } else {
        // Allocator not yet initialized: add to boot log
        BOOT_CONSUMED.lock().extend(&regions);
    }
}

/// Records a single consumed memory region.
///
/// This is a convenience wrapper around [`record_boot_consumed`] for single regions.
///
/// # Parameters
/// - `region`: The consumed region to record
pub fn record_boot_consumed_region(region: ConsumedRegion) {
    if let Some(manager) = PHYS_MANAGER.lock().as_mut() {
        reserve_region(manager, region);
    } else {
        BOOT_CONSUMED.lock().push(region);
    }
}

/// Drains the boot-time consumed log, returning all merged regions.
///
/// This is useful for transferring consumed regions between different
/// initialization stages. After calling this, the log is empty.
///
/// # Returns
/// A vector of merged consumed regions.
pub fn drain_boot_consumed() -> Vec<ConsumedRegion> {
    let mut boot = BOOT_CONSUMED.lock();
    let len = boot.len;
    if len == 0 {
        return Vec::new();
    }

    let merged = merge_regions_in_place(&mut boot.regions, len);

    // Clear the log
    for entry in boot.regions.iter_mut().take(len) {
        *entry = ConsumedRegion::EMPTY;
    }
    boot.len = 0;
    merged
}

/// Returns whether the persistent allocator has been initialized.
///
/// # Returns
/// `true` if [`init_from_handoff`] has been successfully called.
pub fn is_initialised() -> bool {
    PHYS_MANAGER.lock().is_some()
}

/// Allocates a single 4 KiB physical frame.
///
/// This is the primary allocation interface. It returns the physical address
/// of the start of the allocated frame.
///
/// # Returns
/// - `Ok(physical_address)` if a frame was successfully allocated
/// - [`AllocError::OutOfMemory`] if no free frames are available
///
/// # Panics
/// Panics if called before the allocator is initialized.
///
/// # Example
/// ```ignore
/// let frame_pa = alloc_frame()?;
/// // Use the frame at physical address `frame_pa`
/// // ...
/// // Later, return it:
/// free_frame(frame_pa)?;
/// ```
pub fn alloc_frame() -> AllocResult<u64> {
    let mut guard = PHYS_MANAGER.lock();
    let manager = guard.as_mut().expect("physical allocator not initialised");
    manager.pop_first_free().map(Pfn::start_address)
}

/// Frees a previously-allocated physical frame.
///
/// Returns the frame to the free pool, making it available for future allocations.
///
/// # Parameters
/// - `pa`: Physical address of the frame to free (must be frame-aligned)
///
/// # Returns
/// - `Ok(())` if the frame was successfully freed
/// - [`AllocError::DoubleFree`] if the frame was already free
/// - [`AllocError::UnknownFrame`] if the address is outside the managed range
///
/// # Panics
/// Panics if called before the allocator is initialized.
///
/// # Safety
/// The caller must ensure that the frame is no longer in use and that no
/// references to its contents exist. Freeing a frame that is still in use
/// will lead to memory corruption.
pub fn free_frame(pa: u64) -> AllocResult<()> {
    let mut guard = PHYS_MANAGER.lock();
    let manager = guard.as_mut().expect("physical allocator not initialised");
    let pfn = Pfn::containing(pa);
    manager.mark_free(pfn)
}

/// Dumps the current state of the physical memory manager to the debug output.
///
/// This is useful for debugging and monitoring memory usage.
pub fn dump_state() {
    if let Some(mgr) = PHYS_MANAGER.lock().as_ref() {
        log_info!(
            "PhysicalMemoryManager state: base_pfn={:#x} total_frames={} free_frames={}",
            mgr.base_pfn.start_address(),
            mgr.total_frames,
            mgr.free_frames
        );
    } else {
        log_error!("PhysicalMemoryManager not initialized");
    }
}

/// Returns the base PFN managed by the allocator.
///
/// # Returns
/// - `Some(physical_address)` if initialized
/// - `None` if not yet initialized
pub fn base_pfn() -> Option<u64> {
    PHYS_MANAGER
        .lock()
        .as_ref()
        .map(|mgr| mgr.base_pfn.start_address())
}

/// Snapshot of persistent allocator state.
///
/// This structure provides a point-in-time view of allocator statistics
/// without holding the allocator lock.
pub struct Stats {
    /// Physical address of the first managed frame.
    pub base_pfn: u64,
    /// Total number of frames managed.
    pub total_frames: u64,
    /// Number of frames currently free.
    pub free_frames: u64,
}

/// Returns current allocator statistics.
///
/// # Returns
/// - `Some(stats)` if the allocator is initialized
/// - `None` if not yet initialized
pub fn stats() -> Option<Stats> {
    PHYS_MANAGER.lock().as_ref().map(|mgr| Stats {
        base_pfn: mgr.base_pfn.start_address(),
        total_frames: mgr.total_frames,
        free_frames: mgr.free_frames,
    })
}

/// Persistent frame allocator implementing x86_64 traits.
///
/// This zero-sized type wraps the global [`PhysicalMemoryManager`] and
/// implements the `FrameAllocator` trait from the `x86_64` crate. This
/// allows it to be used with the `x86_64` paging API.
pub struct PersistentFrameAllocator;

unsafe impl FrameAllocator<Size4KiB> for PersistentFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        alloc_frame()
            .ok()
            .map(|pa| PhysFrame::containing_address(PhysAddr::new(pa)))
    }
}

impl crate::memory::FrameSource for PersistentFrameAllocator {}

/// A consumed memory region descriptor.
///
/// Represents a region of physical memory that has been consumed during
/// early boot (before the persistent allocator was initialized). These
/// regions need to be marked as reserved to prevent reallocation.
///
/// # Examples
/// - Kernel code and data sections
/// - Page tables
/// - Boot-time stacks
/// - Framebuffer
#[derive(Debug, Clone, Copy)]
pub struct ConsumedRegion {
    /// Physical address of the start of the region.
    pub start: u64,
    /// Size of the region in bytes.
    pub size: u64,
}

impl ConsumedRegion {
    /// An empty region (size = 0). Used as a sentinel value.
    pub const EMPTY: Self = Self { start: 0, size: 0 };
}

/// Convenience function to create a consumed region descriptor.
///
/// # Parameters
/// - `start`: Physical address of the start of the region
/// - `size`: Size of the region in bytes
///
/// # Returns
/// A new [`ConsumedRegion`] descriptor.
pub fn consumed(start: u64, size: u64) -> ConsumedRegion {
    ConsumedRegion { start, size }
}

/// Test-only function to reset the allocator state.
///
/// This is used by unit tests to ensure a clean state between test cases.
#[cfg(test)]
pub fn reset() {
    *PHYS_MANAGER.lock() = None;
    *BOOT_CONSUMED.lock() = BootConsumedLog::new();
}

/// Returns a virtual pointer to the UEFI memory map that is accessible in the current address space.
///
/// The memory map can be provided in three ways:
/// 1. As a virtual address (already >= PHYS_OFFSET)
/// 2. As a physical address that needs to be translated via the physical offset mapping
/// 3. As a physical address accessible via the identity mapping (early boot)
///
/// # Parameters
/// - `handoff`: Boot handoff structure containing the memory map pointer
///
/// # Returns
/// - `Ok(ptr)` if the memory map is accessible
/// - [`AllocError::NoMemoryMap`] if the memory map pointer is null or inaccessible
fn accessible_memmap_ptr(handoff: &Handoff) -> AllocResult<*const u8> {
    let ptr = handoff.memory_map_buffer_ptr;
    if ptr == 0 {
        return Err(AllocError::NoMemoryMap);
    }
    if ptr >= PHYS_OFFSET {
        // Already a virtual address
        Ok(ptr as *const u8)
    } else if phys_offset_is_active() {
        // Physical offset mapping is active: translate physical to virtual
        Ok(phys_to_virt_pa(ptr) as *const u8)
    } else {
        // Identity mapping is still active in early boot, so the physical
        // pointer is accessible directly.
        Ok(ptr as *const u8)
    }
}

/// Parses the UEFI memory map into a vector of memory regions.
///
/// Each UEFI memory descriptor is converted into a [`Region`] with a start PFN,
/// end PFN, and kind (free or reserved). The region kind is determined by the
/// UEFI memory type:
/// - Type 7 (EfiConventionalMemory) → Free
/// - All other types → Reserved
///
/// # Parameters
/// - `handoff`: Boot handoff structure with memory map metadata
/// - `base_ptr`: Virtual pointer to the start of the memory map buffer
///
/// # Returns
/// A vector of memory regions sorted by address.
fn collect_regions(handoff: &Handoff, base_ptr: *const u8) -> Vec<Region> {
    const UEFI_CONVENTIONAL_MEMORY: u32 = 7;
    let mut regions = Vec::new();
    let desc_size = handoff.memory_map_descriptor_size as usize;
    let count = handoff.memory_map_entries as usize;

    for i in 0..count {
        // Compute pointer to this descriptor
        let p = unsafe { base_ptr.add(i * desc_size) };
        // Parse the descriptor
        let view = unsafe { MemoryDescriptorView::new(p, handoff.memory_map_descriptor_size) };

        // Convert to PFN range
        let start = Pfn::containing(view.phys_start);
        let end = Pfn::containing(view.phys_start + view.num_pages * FRAME_SIZE);

        // Determine region kind
        let kind = if view.ty == UEFI_CONVENTIONAL_MEMORY {
            RegionKind::Free
        } else {
            RegionKind::Reserved
        };

        regions.push(Region { start, end, kind });
    }
    regions
}

/// Computes the base PFN and total frame count needed to cover all free regions.
///
/// This scans all free regions from the memory map and determines the smallest
/// contiguous range of PFNs that covers all of them. The allocator will then
/// manage this entire range, marking non-free regions as reserved.
///
/// # Parameters
/// - `regions`: Slice of memory regions from the UEFI memory map
///
/// # Returns
/// - `Ok((base_pfn, frame_count))` if free regions exist
/// - [`AllocError::OutOfMemory`] if no free regions are found
fn compute_bounds(regions: &[Region]) -> AllocResult<(Pfn, u64)> {
    let mut free_regions: Vec<&Region> = regions
        .iter()
        .filter(|r| matches!(r.kind, RegionKind::Free))
        .collect();

    if free_regions.is_empty() {
        return Err(AllocError::OutOfMemory);
    }

    // Sort by start address
    free_regions.sort_by(|a, b| a.start.cmp(&b.start));

    // Base PFN is the start of the first free region
    let base = free_regions.first().unwrap().start;

    // End PFN is the maximum end of any free region
    let end = free_regions.iter().map(|r| r.end).max().unwrap_or(base);

    Ok((base, end.0.saturating_sub(base.0)))
}

/// A lightweight view into a UEFI memory descriptor.
///
/// The UEFI memory map consists of descriptors with a variable size (usually 48+ bytes).
/// This structure extracts just the fields we need for allocation purposes.
struct MemoryDescriptorView {
    /// UEFI memory type (7 = conventional, others = reserved).
    ty: u32,
    /// Physical start address of the region.
    phys_start: u64,
    /// Number of 4 KiB pages in the region.
    num_pages: u64,
}

impl MemoryDescriptorView {
    /// Constructs a view from a raw pointer to a UEFI memory descriptor.
    ///
    /// # Safety
    /// The caller must ensure that `ptr` points to a valid UEFI memory descriptor
    /// of at least `_size` bytes (though we only read the first ~32 bytes).
    ///
    /// # UEFI Memory Descriptor Layout
    /// ```text
    /// Offset | Size | Field
    /// -------|------|------
    ///   0    |  4   | Type
    ///   4    |  4   | (padding)
    ///   8    |  8   | PhysicalStart
    ///  16    |  8   | VirtualStart
    ///  24    |  8   | NumberOfPages
    ///  32    |  8   | Attribute
    /// ```
    unsafe fn new(ptr: *const u8, _size: u32) -> Self {
        Self {
            ty: read_u32(ptr, 0),
            phys_start: read_u64(ptr, 8),
            num_pages: read_u64(ptr, 24),
        }
    }
}

/// Reads a u32 from an unaligned pointer with a byte offset.
///
/// # Safety
/// The caller must ensure that `ptr + offset` is a valid, readable memory location.
unsafe fn read_u32(ptr: *const u8, offset: usize) -> u32 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u32)
}

/// Reads a u64 from an unaligned pointer with a byte offset.
///
/// # Safety
/// The caller must ensure that `ptr + offset` is a valid, readable memory location.
unsafe fn read_u64(ptr: *const u8, offset: usize) -> u64 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u64)
}

/// Reserves a consumed region in the physical memory manager.
///
/// Converts a byte-based region descriptor into PFN ranges and marks all
/// overlapping frames as allocated. This ensures that consumed memory
/// (kernel, page tables, etc.) is not reused.
///
/// # Parameters
/// - `manager`: The physical memory manager
/// - `region`: The consumed region to reserve
fn reserve_region(manager: &mut PhysicalMemoryManager, region: ConsumedRegion) {
    if region.size == 0 {
        return; // Empty region, nothing to do
    }

    // Convert byte address to PFN (rounding down)
    let start_pfn = Pfn::containing(region.start);

    // Compute number of pages, rounding up to cover partial pages
    let pages = (region.size + FRAME_SIZE - 1) / FRAME_SIZE;

    manager.reserve_range(start_pfn, pages);
}
