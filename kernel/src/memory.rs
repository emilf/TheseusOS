//! Memory management module
//!
//! This module contains the early-boot memory-management primitives used during
//! kernel bring-up. It is intentionally self-contained and provides a small
//! custom page-table representation (for early identity mapping and boot-time
//! page-table setup) as well as helpers that integrate with the `x86_64` crate
//! once paging is active.
//!
//! Major responsibilities:
//! - Building a PML4 and mapping an identity region used during early boot
//! - Mapping the kernel into a higher-half and establishing `PHYS_OFFSET`
//! - Providing a `BootFrameAllocator` built from the UEFI handoff memory map
//!   with reserved frame pool for critical kernel structures
//! - Utilities for mapping single pages and 2MiB huge pages using frame-backed
//!   page-table allocation helpers
//! - A small `TemporaryWindow` API to perform single-frame temporary mappings
//!   without relying on identity access once the PHYS_OFFSET mapping is active
//!
//! # Recent Improvements (Latest Session)
//!
//! ## Reserved Frame Pool
//! - Added `BootFrameAllocator` with reserved frame pool (16 frames) to prevent
//!   critical kernel structures (page tables, IST stacks) from being starved by
//!   ephemeral allocations.
//! - `reserve_frames()` method reserves frames from the general pool.
//! - `allocate_reserved_frame()` provides LIFO allocation from reserved pool.
//! - Page table allocation prefers reserved frames, falling back to general pool.
//!
//! ## Temporary Mapping Window
//! - `TemporaryWindow` struct provides safe temporary mapping of arbitrary physical
//!   frames to a fixed virtual address without relying on identity mapping.
//! - Used for zeroing newly allocated page tables safely after PHYS_OFFSET is active.
//! - Methods: `new()`, `map_phys_frame()`, `map_and_zero_frame()`, `unmap()`.
//!
//! ## PHYS_OFFSET Integration
//! - Enhanced `zero_frame_safely()` to use PHYS_OFFSET mapping when active,
//!   with identity fallback for early boot.
//! - `phys_offset_is_active()` tracks when PHYS_OFFSET mapping is available.
//! - `zero_phys_range()` performs volatile writes through PHYS_OFFSET mapping.
//!
//! ## Memory Map Diagnostics
//! - Added diagnostics showing total pages and conventional pages from UEFI memory map.
//! - Confirms allocator sees full system RAM (typically 1GB+ in QEMU).
//! - Helps debug memory allocation issues.
//!
//! Safety notes:
//! - Many functions in this module are `unsafe` because they manipulate raw
//!   page tables, write to control registers (CR3) or access physical memory
//!   via virtual translations. Callers must ensure the required invariants for
//!   these operations hold (correct PML4, appropriate mapping state, etc.).
#![allow(dead_code)]
#![allow(static_mut_refs)]


/// Page table entry flags
pub const PTE_PRESENT: u64 = 1 << 0;      // Present
pub const PTE_WRITABLE: u64 = 1 << 1;     // Writable
pub const PTE_USER: u64 = 1 << 2;         // User accessible
pub const PTE_PWT: u64 = 1 << 3;          // Page Write Through
pub const PTE_PCD: u64 = 1 << 4;          // Page Cache Disable
pub const PTE_ACCESSED: u64 = 1 << 5;     // Accessed
pub const PTE_DIRTY: u64 = 1 << 6;        // Dirty
pub const PTE_PS: u64 = 1 << 7;           // Page Size (for PDE)
pub const PTE_GLOBAL: u64 = 1 << 8;       // Global
pub const PTE_NO_EXEC: u64 = 1 << 63;     // No Execute

/// Page table levels
pub const PML4_LEVEL: usize = 0;
pub const PDPT_LEVEL: usize = 1;
pub const PD_LEVEL: usize = 2;
pub const PT_LEVEL: usize = 3;

/// Page size constants
pub const PAGE_SIZE: usize = 4096;
pub const PAGE_MASK: u64 = 0xFFF;

/// Virtual memory layout
pub const KERNEL_VIRTUAL_BASE: u64 = 0xFFFFFFFF80000000;
pub const KERNEL_HEAP_BASE: u64 = 0xFFFFFFFF80010000;
pub const KERNEL_HEAP_SIZE: usize = 0x100000; // 1MB
/// Fixed virtual base where the temporary boot heap is mapped
pub const TEMP_HEAP_VIRTUAL_BASE: u64 = 0xFFFFFFFFA0000000;
/// Physical memory linear mapping base (maps [0..N) -> [PHYS_OFFSET..PHYS_OFFSET+N))
pub const PHYS_OFFSET: u64 = 0xFFFF800000000000;

/// Convert a physical address into the kernel's high-half virtual address using
/// the fixed `PHYS_OFFSET` mapping.
///
/// Arguments:
/// - `pa`: physical address to convert
///
/// Returns:
/// - virtual address corresponding to `pa` in the kernel's PHYS_OFFSET window

/// Page table entry
/// 
/// Represents a single entry in a page table, containing a physical address
/// and various flags that control memory access and caching behavior.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct PageTableEntry(pub u64);

impl PageTableEntry {
    /// Create a new page table entry
    /// 
    /// # Parameters
    /// 
    /// * `physical_addr` - The physical address to map
    /// * `flags` - The flags to set for this entry
    /// 
    /// # Returns
    /// 
    /// A new PageTableEntry with the specified address and flags
    pub const fn new(physical_addr: u64, flags: u64) -> Self {
        Self(physical_addr & !PAGE_MASK | flags)
    }
    
    /// Get the physical address from the entry
    /// 
    /// # Returns
    /// 
    /// The physical address stored in this entry
    pub fn physical_addr(&self) -> u64 { self.0 & !PAGE_MASK }
    
    /// Check if the entry is present
    /// 
    /// # Returns
    /// 
    /// * `true` - If the entry is present and valid
    /// * `false` - If the entry is not present
    pub fn is_present(&self) -> bool {
        self.0 & PTE_PRESENT != 0
    }
    
    /// Set the entry as present
    /// 
    /// This marks the entry as present and valid.
    pub fn set_present(&mut self) {
        self.0 |= PTE_PRESENT;
    }
    
    /// Get the flags
    /// 
    /// # Returns
    /// 
    /// The flags stored in this entry
    pub fn flags(&self) -> u64 { self.0 & PAGE_MASK }
}

/// Page table (512 entries)
/// 
/// Represents a page table containing 512 entries, each mapping a 4KB page
/// or pointing to another level of page tables.
#[repr(align(4096))]
#[derive(Clone, Copy)]
pub struct PageTable {
    pub entries: [PageTableEntry; 512],
}

impl PageTable {
    /// Create a new empty page table
    /// 
    /// # Returns
    /// 
    /// A new PageTable with all entries set to zero (not present)
    pub const fn new() -> Self {
        Self {
            entries: [PageTableEntry(0); 512],
        }
    }
    
    /// Get a page table entry by index
    /// 
    /// # Parameters
    /// 
    /// * `index` - The index of the entry to retrieve (0-511)
    /// 
    /// # Returns
    /// 
    /// A mutable reference to the page table entry
    pub fn get_entry(&mut self, index: usize) -> &mut PageTableEntry {
        &mut self.entries[index]
    }
}

/// Memory manager
pub struct MemoryManager {
    pub pml4: &'static mut PageTable,
    pml4_phys: u64,
    pub kernel_heap_start: u64,
    pub kernel_heap_end: u64,
    pub frame_allocator: BootFrameAllocator,
}
// Simple global virt<->phys offset model for early paging
static mut VIRT_PHYS_OFFSET: u64 = 0;
// Indicates that the PHYS_OFFSET linear mapping is active (so phys_to_virt_pa can be used)
static mut PHYS_OFFSET_ACTIVE: bool = false;
// Physical base of kernel image for page-table pool address translation
static mut KERNEL_PHYS_BASE_FOR_POOL: u64 = 0;

// Early virt/phys helpers no longer used in HH path; keep for legacy/debug if needed
#[allow(dead_code)]
fn virt_to_phys(va: u64) -> u64 { unsafe { va.wrapping_add(VIRT_PHYS_OFFSET) } }

#[allow(dead_code)]
fn phys_to_virt(pa: u64) -> u64 { unsafe { pa.wrapping_add(VIRT_PHYS_OFFSET) } }

/// Convert a physical address into the kernel's high-half virtual address using
/// the fixed `PHYS_OFFSET` mapping.
#[allow(dead_code)]
pub fn phys_to_virt_pa(pa: u64) -> u64 { PHYS_OFFSET.wrapping_add(pa) }

/// Mark the PHYS_OFFSET linear mapping as active. Call this after CR3 is loaded
/// and the kernel's PHYS_OFFSET mapping is established.
pub fn set_phys_offset_active() { unsafe { PHYS_OFFSET_ACTIVE = true; } }

/// Query whether the PHYS_OFFSET mapping is active
pub fn phys_offset_is_active() -> bool { unsafe { PHYS_OFFSET_ACTIVE } }

/// Zero a range of physical memory by writing through the kernel's PHYS_OFFSET
/// mapping. This is used for zeroing page-table frames after the PHYS_OFFSET
/// mapping is established.
///
/// # Safety
///
/// The caller must ensure that the PHYS_OFFSET mapping is valid and maps the
/// physical frame range `[pa, pa + size)` into the kernel virtual address space.

#[allow(dead_code)]
fn set_virt_phys_offset(offset: u64) { unsafe { VIRT_PHYS_OFFSET = offset; } }

/// Zero a physical address range by converting to the kernel's PHYS_OFFSET-mapped virtual
/// address and performing volatile writes. Used for zeroing page-table frames safely.
#[allow(dead_code)]
pub unsafe fn zero_phys_range(pa: u64, size: usize) {
    let va = phys_to_virt_pa(pa);
    let mut off: usize = 0;
    // write u64 words for speed
    while off + 8 <= size {
        core::ptr::write_volatile((va + off as u64) as *mut u64, 0u64);
        off += 8;
    }
    // tail bytes if any
    while off < size {
        core::ptr::write_volatile((va + off as u64) as *mut u8, 0u8);
        off += 1;
    }
}

/// Zero a single page/frame safely. If paging is already enabled and the
/// PHYS_OFFSET mapping is available we use that; otherwise fall back to
/// identity physical writes which are valid early in boot.
#[allow(dead_code)]
pub unsafe fn zero_frame_safely(pa: u64) {
    // Zero a single frame safely. When the PHYS_OFFSET mapping is active we
    // call `zero_phys_range` which writes via the kernel virtual mapping; when
    // PHYS_OFFSET is not available (very early boot) we fall back to identity
    // writes into the physical address directly.
    if phys_offset_is_active() {
        zero_phys_range(pa, PAGE_SIZE as usize);
    } else {
        // Early boot path: identity write into physical memory. This is only
        // valid because early boot is still identity-addressable.
        core::ptr::write_bytes(pa as *mut u8, 0, PAGE_SIZE);
    }
}

/// Expose current virt->phys offset (phys - virt)
#[allow(dead_code)]
pub fn virt_phys_offset() -> u64 { unsafe { VIRT_PHYS_OFFSET } }


impl MemoryManager {
    /// Create a new memory manager
    ///
    /// Construct a new MemoryManager and build an initial PML4 with the
    /// following mappings established:
    /// - identity mapping for the first 1 GiB (2MiB pages)
    /// - kernel image mapped into the high-half (4KiB pages)
    /// - a PHYS_OFFSET linear mapping for the first 1 GiB
    /// - framebuffer and temporary heap mappings if provided by the handoff
    ///
    /// # Arguments
    /// - `handoff`: pointer to the bootloader handoff structure containing the
    ///   UEFI memory map, kernel physical base/size, framebuffer info and temp
    ///   heap location.
    ///
    /// # Returns
    /// - `MemoryManager` containing a pointer to the PML4 (virtual), the
    ///   physical PML4 frame, and an initialized early `BootFrameAllocator`.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it writes raw page tables and performs
    /// identity physical memory writes while paging is not yet active. The
    /// caller must ensure the provided `handoff` is valid.
    pub unsafe fn new(handoff: &theseus_shared::handoff::Handoff) -> Self {
            crate::display::kernel_write_line("  [vm/new] start");
            // Make kernel phys base available (legacy var no longer used after pool removal)
            KERNEL_PHYS_BASE_FOR_POOL = handoff.kernel_physical_base;
        // Initialize early frame allocator and allocate a fresh PML4 frame
        let mut early_frame_alloc = BootFrameAllocator::from_handoff(handoff);
        // Reserve a small pool of frames for critical kernel structures (page
        // tables, IST stacks, etc.) so ephemeral allocations won't starve them.
        // This ensures that essential boot-time structures always have frames
        // available even if the general pool is temporarily exhausted during
        // heavy mapping operations.
        const RESERVED_FOR_CRITICAL: usize = 16;
        let reserved = early_frame_alloc.reserve_frames(RESERVED_FOR_CRITICAL);
        crate::display::kernel_write_line("  [fa] reserved frames="); theseus_shared::print_hex_u64_0xe9!(reserved as u64); crate::display::kernel_write_line("\n");
        let pml4_frame = early_frame_alloc.allocate_frame().expect("Out of frames for PML4");
        let pml4_phys = pml4_frame.start_address().as_u64();
        // Zero the new PML4 frame using a helper that picks the correct method
        // depending on whether paging is active.
        zero_frame_safely(pml4_phys);
        
        let pml4: &mut PageTable = &mut *(pml4_phys as *mut PageTable);
        crate::display::kernel_write_line("  [vm/new] got pml4");

        // Identity map first 1 GiB
        {
            crate::display::kernel_write_line("  [vm/new] id-map 1GiB begin");
            // Bootstrap identity map using frame-backed tables
        identity_map_first_1gb_2mb_alloc(pml4, &mut early_frame_alloc);
            crate::display::kernel_write_line("  [vm/new] id-map 1GiB done");
        }
        // legacy path removed

        // Do not clone bootloader HH entry; create our own HH structures deterministically

        // Map the kernel image high-half
        {
            crate::display::kernel_write_line("  [vm/new] map kernel HH begin");
            // Map kernel code/data into the high-half using 4KiB pages with frame allocator
            map_kernel_high_half_4k_alloc(pml4, handoff, &mut early_frame_alloc);
            crate::display::kernel_write_line("  [vm/new] map kernel HH done");
            // Debug: print PML4[HH] entry after mapping
            let hh_index = ((KERNEL_VIRTUAL_BASE >> 39) & 0x1FF) as usize;
            let pml4_addr = pml4 as *mut PageTable as u64;
            let entry_val = core::ptr::read_volatile((pml4_addr as *const u64).add(hh_index));
            // PML4[HH] -- high-half mapping for kernel (debug left minimal)
            theseus_shared::print_hex_u64_0xe9!(entry_val);
            if entry_val == 0 {
                // Force-create HH PDPT so the entry is present
                let _ = get_or_create_page_table_alloc(pml4.get_entry(hh_index), &mut early_frame_alloc);
                let entry_val2 = core::ptr::read_volatile((pml4_addr as *const u64).add(hh_index));
                crate::display::kernel_write_line("  [vm/new] PML4[HH] forced=");
                theseus_shared::print_hex_u64_0xe9!(entry_val2);
                crate::display::kernel_write_line("\n");
            }
        }
        // legacy path removed

        // Map framebuffer and temp heap using frame allocator
        {
            crate::display::kernel_write_line("  [vm/new] map fb/heap begin");
            if handoff.gop_fb_base != 0 { map_framebuffer_alloc(pml4, handoff, &mut early_frame_alloc); }
            if handoff.temp_heap_base != 0 { map_temporary_heap_alloc(pml4, handoff, &mut early_frame_alloc); }
            crate::display::kernel_write_line("  [vm/new] map fb/heap done");
        }
        // Map a linear physical mapping for first 1 GiB at PHYS_OFFSET
        {
            crate::display::kernel_write_line("  [vm/new] map phys_offset 1GiB begin");
            map_phys_offset_1gb_2mb_alloc(pml4, &mut early_frame_alloc);
            crate::display::kernel_write_line("  [vm/new] map phys_offset 1GiB done");
        }

        // Map LAPIC MMIO region (0xFEE00000-0xFEEFFFFF)
        {
            crate::display::kernel_write_line("  [vm/new] map LAPIC MMIO begin");
            map_lapic_mmio_alloc(pml4, &mut early_frame_alloc);
            crate::display::kernel_write_line("  [vm/new] map LAPIC MMIO done");
        }

        let kernel_heap_start = KERNEL_HEAP_BASE;
        let kernel_heap_end = kernel_heap_start + KERNEL_HEAP_SIZE as u64;

        // Quick integrity checks (optional): ensure first entries are present
        if !pml4.entries[0].is_present() { panic!("PML4[0] not present"); }

        // Done
        let s = Self {
            pml4,
            pml4_phys,
            kernel_heap_start,
            kernel_heap_end,
            frame_allocator: early_frame_alloc,
        };
        s
    }
    
    /// Get the page table root (CR3 value)
    pub fn page_table_root(&self) -> u64 { self.pml4_phys }
}

/// Set up identity mapping for first 1 GiB using 2 MiB pages
unsafe fn identity_map_first_1gb_2mb_alloc(pml4: &mut PageTable, fa: &mut BootFrameAllocator) {
    // Identity-map the first 1 GiB of physical memory using 2MiB pages.
    //
    // This is used during early boot so that low-VA virtual accesses still
    // find the expected physical frames while we continue building the
    // higher-half mappings.
    //
    // Arguments:
    // - `pml4`: mutable reference to the newly-created PML4 table
    // - `fa`: frame allocator used to allocate intermediate page tables
    //
    // Returns: nothing (maps are installed in `pml4`)
    // Identity map using 2MiB pages with NX cleared (executable)
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_PS;
    let gigabyte: u64 = 1 << 30;
    let two_mb: u64 = 2 * 1024 * 1024;
    let mut addr: u64 = 0;
    let mut count: u32 = 0;
    while addr < gigabyte {
        if count < 4 { crate::display::kernel_write_line("    [vm] map2m"); theseus_shared::print_hex_u64_0xe9!(addr); crate::display::kernel_write_line(" -> "); theseus_shared::print_hex_u64_0xe9!(addr); crate::display::kernel_write_line("\n"); }
        map_2mb_page_alloc(pml4, addr, addr, flags, fa);
        addr += two_mb;
        count += 1;
    }
}

/// Map kernel to high-half using a single 2 MiB page
unsafe fn map_high_half_1gb_2mb(pml4: &mut PageTable, fa: &mut BootFrameAllocator) {
    // Map [0 .. 1GiB) physical -> [KERNEL_VIRTUAL_BASE .. +1GiB) virtual using 2MiB pages
    let two_mb: u64 = 2 * 1024 * 1024;
    let one_gb: u64 = 1024 * 1024 * 1024;
    let virt_base = KERNEL_VIRTUAL_BASE & !(two_mb - 1);
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL;
    let mut offset: u64 = 0;
    while offset < one_gb {
        let pa = offset;
        let va = virt_base + offset;
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        offset += two_mb;
    }
}

/// Map a 1GiB linear physical mapping at PHYS_OFFSET using 2MiB pages
unsafe fn map_phys_offset_1gb_2mb_alloc(pml4: &mut PageTable, fa: &mut BootFrameAllocator) {
    let two_mb: u64 = 2 * 1024 * 1024;
    let one_gb: u64 = 1024 * 1024 * 1024;
    let mut offset: u64 = 0;
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL;
    while offset < one_gb {
        let pa = offset;
        let va = phys_to_virt_pa(offset);
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        offset += two_mb;
    }
}

/// Map LAPIC MMIO region (0xFEE00000-0xFEEFFFFF) at a dedicated virtual address
unsafe fn map_lapic_mmio_alloc(pml4: &mut PageTable, fa: &mut BootFrameAllocator) {
    const LAPIC_PHYS_BASE: u64 = 0xFEE00000;
    const LAPIC_VIRT_BASE: u64 = 0xFFFF800000000000 + 0xFEE00000; // Use PHYS_OFFSET + LAPIC_PHYS_BASE
    const LAPIC_SIZE: u64 = 0x100000; // 1MB
    
    // Map LAPIC MMIO region using 4KB pages (it's only 1MB, so 4KB pages are fine)
    for i in 0..(LAPIC_SIZE / PAGE_SIZE as u64) {
        let virt_addr = LAPIC_VIRT_BASE + (i * PAGE_SIZE as u64);
        let phys_addr = LAPIC_PHYS_BASE + (i * PAGE_SIZE as u64);
        map_page_alloc(pml4, virt_addr, phys_addr, PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_PCD | PTE_PWT, fa);
    }
}

/// Map the kernel's physical image range into the high-half window using 2MiB pages
unsafe fn map_kernel_high_half_2mb(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff, fa: &mut BootFrameAllocator) {
    let two_mb: u64 = 2 * 1024 * 1024;
    let phys_base = handoff.kernel_physical_base;
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 { return; }

    // Align physical range to 2MiB boundaries
    let phys_start = phys_base & !(two_mb - 1);
    let phys_end = (phys_base + phys_size + two_mb - 1) & !(two_mb - 1);

    // Compute the corresponding virtual start so that phys_base maps to KERNEL_VIRTUAL_BASE
    let va_start = KERNEL_VIRTUAL_BASE.wrapping_sub(phys_base.wrapping_sub(phys_start));

    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL; // executable (no NX)
    let mut pa = phys_start;
    let mut va = va_start;
    while pa < phys_end {
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        pa += two_mb;
        va += two_mb;
    }
}

/// Map the kernel's physical image range into the high-half window using 4KiB pages
unsafe fn map_kernel_high_half_4k_alloc(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff, fa: &mut BootFrameAllocator) {
    let phys_base = handoff.kernel_physical_base;
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 { return; }

    // Map with padding to cover .bss/.stack placed beyond reported image size,
    // but intentionally leave a 1-page guard unmapped at the end to catch overruns.
    const KERNEL_IMAGE_PAD: u64 = 8 * 1024 * 1024; // 8 MiB cushion
    let mut total_bytes = phys_size + KERNEL_IMAGE_PAD;
    if total_bytes >= PAGE_SIZE as u64 {
        total_bytes -= PAGE_SIZE as u64; // leave one unmapped guard page at the end
    }
    let pages: u64 = (total_bytes + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL; // executable

    for i in 0..pages {
        let pa = phys_base + i * PAGE_SIZE as u64;
        let va = KERNEL_VIRTUAL_BASE + i * PAGE_SIZE as u64;
        map_page_alloc(pml4, va, pa, flags, fa);
    }
}

/// Map an existing kernel VA range to its corresponding PA using the kernel base translation.
/// VA->PA translation: pa = handoff.kernel_physical_base + (va - KERNEL_VIRTUAL_BASE)
pub unsafe fn map_existing_region_va_to_its_pa(
    pml4_phys: u64,
    handoff: &theseus_shared::handoff::Handoff,
    start_va: u64,
    size: u64,
    flags: u64,
    fa: &mut BootFrameAllocator,
) {
    if size == 0 { return; }
    let pml4: &mut PageTable = &mut *(pml4_phys as *mut PageTable);
    let mut va = start_va & !((PAGE_SIZE as u64) - 1);
    let end = (start_va + size + (PAGE_SIZE as u64) - 1) & !((PAGE_SIZE as u64) - 1);
    while va < end {
        let offset = va.wrapping_sub(KERNEL_VIRTUAL_BASE);
        let pa = handoff.kernel_physical_base.wrapping_add(offset);
        map_page_alloc(pml4, va, pa, flags, fa);
        va = va.wrapping_add(PAGE_SIZE as u64);
    }
}

/// Map framebuffer
unsafe fn map_framebuffer_alloc(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff, fa: &mut BootFrameAllocator) {
    let fb_physical = handoff.gop_fb_base;
    let fb_virtual = 0xFFFFFFFF90000000; // Map framebuffer to high memory
    let fb_size = handoff.gop_fb_size;
    let pages = (fb_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    
    for page in 0..pages {
        let physical_addr = fb_physical + page * PAGE_SIZE as u64;
        let virtual_addr = fb_virtual + page * PAGE_SIZE as u64;
        
        map_page_alloc(pml4, virtual_addr, physical_addr,
                PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC, fa);
    }
}

/// Map temporary heap
unsafe fn map_temporary_heap_alloc(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff, fa: &mut BootFrameAllocator) {
    // Map the temporary heap provided by the bootloader into a fixed high-half
    // virtual region. The kernel uses this temporary heap prior to switching to
    // its permanent heap.
    //
    // Arguments:
    // - `pml4`: mutable reference to PML4
    // - `handoff`: bootloader handoff containing `temp_heap_base` and `temp_heap_size`
    // - `fa`: frame allocator for page-table allocation
    let heap_physical = handoff.temp_heap_base;
    let heap_virtual = 0xFFFFFFFFA0000000; // Map heap to high memory
    let heap_size = handoff.temp_heap_size;
    let pages = (heap_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    
    for page in 0..pages {
        let physical_addr = heap_physical + page * PAGE_SIZE as u64;
        let virtual_addr = heap_virtual + page * PAGE_SIZE as u64;
        
        map_page_alloc(pml4, virtual_addr, physical_addr,
                PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC, fa);
    }
}

/// Map a single page
// legacy map_page removed; use map_page_alloc

/// Map a single page using frame-backed table allocation
unsafe fn map_page_alloc(pml4: &mut PageTable, virtual_addr: u64, physical_addr: u64, flags: u64, fa: &mut BootFrameAllocator) {
    // Map a single 4KiB page into `pml4`, creating intermediate page-table
    // levels as necessary using `fa`.
    //
    // Arguments:
    // - `pml4`: mutable reference to the root PML4 table
    // - `virtual_addr`: virtual address to map
    // - `physical_addr`: corresponding physical address
    // - `flags`: page entry flags
    // - `fa`: frame allocator for allocating intermediate page tables
    //
    // Returns: nothing; the mapping is written into page tables.
    // Extract page table indices
    let pml4_index = ((virtual_addr >> 39) & 0x1FF) as usize;
    let pdpt_index = ((virtual_addr >> 30) & 0x1FF) as usize;
    let pd_index = ((virtual_addr >> 21) & 0x1FF) as usize;
    let pt_index = ((virtual_addr >> 12) & 0x1FF) as usize;
    
    // Get or create PDPT
    let pdpt = get_or_create_page_table_alloc(pml4.get_entry(pml4_index), fa);
    
    // Get or create PD
    let pd = get_or_create_page_table_alloc(pdpt.get_entry(pdpt_index), fa);
    
    // Get or create PT
    let pt = get_or_create_page_table_alloc(pd.get_entry(pd_index), fa);
    
    // Set the page table entry
    *pt.get_entry(pt_index) = PageTableEntry::new(physical_addr, flags);
}

/// Map a single 2 MiB page by setting a PD entry with PS
// legacy map_2mb_page removed; use map_2mb_page_alloc

/// Map a single 2 MiB page using frame-backed table allocation
unsafe fn map_2mb_page_alloc(pml4: &mut PageTable, virtual_addr: u64, physical_addr: u64, flags: u64, fa: &mut BootFrameAllocator) {
    let pml4_index = ((virtual_addr >> 39) & 0x1FF) as usize;
    let pdpt_index = ((virtual_addr >> 30) & 0x1FF) as usize;
    let pd_index = ((virtual_addr >> 21) & 0x1FF) as usize;

    // Ensure next-level tables exist up to PD
    let pdpt = get_or_create_page_table_alloc(pml4.get_entry(pml4_index), fa);
    let pd = get_or_create_page_table_alloc(pdpt.get_entry(pdpt_index), fa);

    // Set PD entry with PS bit
    *pd.get_entry(pd_index) = PageTableEntry::new(physical_addr, flags | PTE_PS);
}

/// Get or create a page table
// Legacy helper removed; use get_or_create_page_table_alloc instead

/// Get or create a page table using a frame allocator
unsafe fn get_or_create_page_table_alloc(entry: &mut PageTableEntry, fa: &mut BootFrameAllocator) -> &'static mut PageTable {
    // Get the existing page table referenced by `entry`, or allocate, zero and
    // install a new page-table frame and return a mutable reference to it.
    //
    // Arguments:
    // - `entry`: mutable reference to a page-table entry where the PT/PD/PDPT
    //   pointer would be stored
    // - `fa`: frame allocator used to obtain a new physical frame if needed
    //
    // Returns:
    // - `&'static mut PageTable` pointing to the created or existing table
    if entry.is_present() {
        let pa = entry.physical_addr();
        // Prefer PHYS_OFFSET mapping if active, otherwise fall back to identity
        if phys_offset_is_active() {
            let va = phys_to_virt_pa(pa) as *mut PageTable;
            &mut *va
        } else {
            &mut *(pa as *mut PageTable)
        }
    } else {
        // Prefer a reserved frame if available to avoid consuming the general
        // pool used for non-critical allocations. This ensures that page table
        // allocation doesn't compete with ephemeral allocations for frames.
        let frame_opt = fa.allocate_reserved_frame().or_else(|| fa.allocate_frame());
        if let Some(frame) = frame_opt {
            let phys = frame.start_address().as_u64();
            // If PHYS_OFFSET mapping is active, map the frame into a temporary
            // window and zero it through the window; otherwise fall back to identity.
            if phys_offset_is_active() {
                if let Some(mut tw) = TemporaryWindow::new_from_current_pml4() {
                    tw.map_and_zero_frame(phys, fa);
                } else {
                    // Fallback to identity if we couldn't obtain current PML4
                    core::ptr::write_bytes(phys as *mut u8, 0, PAGE_SIZE);
                }
                *entry = PageTableEntry::new(phys, PTE_PRESENT | PTE_WRITABLE);
                let va = phys_to_virt_pa(phys) as *mut PageTable;
                &mut *va
            } else {
                core::ptr::write_bytes(phys as *mut u8, 0, PAGE_SIZE);
        *entry = PageTableEntry::new(phys, PTE_PRESENT | PTE_WRITABLE);
                &mut *(phys as *mut PageTable)
            }
        } else {
            panic!("Out of frames for page tables");
        }
    }
}

/// A tiny temporary page mapper that maps a single physical frame to a fixed
/// virtual window. Useful for safely accessing arbitrary physical memory
/// (e.g., page tables) without relying on arbitrary identity writes.
pub const TEMP_WINDOW_VA: u64 = 0xFFFF_FFFE_0000_0000u64;

pub struct TemporaryWindow {
    /// Virtual address of the temporary window
    pub window_va: u64,
    /// PML4 virtual pointer (PHYS_OFFSET translated)
    pml4: *mut PageTable,
}

impl TemporaryWindow {
    /// Create a new TemporaryWindow backed by the provided PML4 physical address.
    ///
    /// # Safety
    ///
    /// Caller must ensure `pml4_phys` is the physical address of the current PML4
    /// and that PHYS_OFFSET mapping is available (or identity mapping exists for low
    /// physical frames during early boot). The window VA is chosen from a high unused
    /// virtual address range.
    ///
    /// # Arguments
    /// - `pml4_phys`: physical address of the active PML4 (CR3)
    ///
    /// # Safety
    /// Caller must ensure `phys_to_virt_pa(pml4_phys)` yields a valid virtual
    /// address (PHYS_OFFSET mapping is active).
    pub unsafe fn new(pml4_phys: u64) -> Self {
        // Use a high, unlikely-to-collide virtual address for temporary mappings
        const TEMP_WINDOW_VA: u64 = 0xFFFF_FFFE_0000_0000u64;
        let pml4_va = phys_to_virt_pa(pml4_phys) as *mut PageTable;
        Self { window_va: TEMP_WINDOW_VA, pml4: pml4_va }
    }

    /// Map a single 4KiB physical frame into the temporary window and return the VA.
    ///
    /// # Safety
    ///
    /// `fa` must be a valid BootFrameAllocator used to allocate intermediate page
    /// tables if they are missing. This function will overwrite any existing mapping
    /// at the temporary window.
    pub unsafe fn map_phys_frame(&mut self, phys_frame_pa: u64, fa: &mut BootFrameAllocator) -> u64 {
        let pml4 = &mut *self.pml4;
        // If an existing mapping is present, simply overwrite the PT entry
        map_page_alloc(pml4, self.window_va, phys_frame_pa, PTE_PRESENT | PTE_WRITABLE, fa);
        self.window_va
    }

    /// Convenience constructor that reads CR3 to determine the current PML4 and
    /// returns a new TemporaryWindow. Returns `None` if CR3 appears invalid.
    ///
    /// # Safety
    ///
    /// Requires that the PHYS_OFFSET mapping is active.
    pub unsafe fn new_from_current_pml4() -> Option<Self> {
        use x86_64::registers::control::Cr3;
        let (frame, _flags) = Cr3::read();
        let pml4_pa = frame.start_address().as_u64();
        if pml4_pa == 0 { return None; }
        Some(Self::new(pml4_pa))
    }

    /// Map then zero a frame: convenience wrapper that maps, zeroes through VA,
    /// and unmaps the window.
    pub unsafe fn map_and_zero_frame(&mut self, phys_frame_pa: u64, fa: &mut BootFrameAllocator) {
        let va = self.map_phys_frame(phys_frame_pa, fa);
        core::ptr::write_bytes(va as *mut u8, 0, PAGE_SIZE);
        self.unmap();
    }

    /// Unmap the temporary window (clears the leaf PTE). Does not free page tables.
    pub unsafe fn unmap(&mut self) {
        let pml4 = &mut *self.pml4;
        let pml4_index = ((self.window_va >> 39) & 0x1FF) as usize;
        let pdpt_index = ((self.window_va >> 30) & 0x1FF) as usize;
        let pd_index = ((self.window_va >> 21) & 0x1FF) as usize;
        let pt_index = ((self.window_va >> 12) & 0x1FF) as usize;

        if !pml4.entries[pml4_index].is_present() { return; }
        let pdpt = &mut *(pml4.entries[pml4_index].physical_addr() as *mut PageTable);
        if !pdpt.entries[pdpt_index].is_present() { return; }
        let pd = &mut *(pdpt.entries[pdpt_index].physical_addr() as *mut PageTable);
        if !pd.entries[pd_index].is_present() { return; }
        let pt = &mut *(pd.entries[pd_index].physical_addr() as *mut PageTable);
        // Clear the PT entry
        *pt.get_entry(pt_index) = PageTableEntry::new(0, 0);
    }
}

/// Activate virtual memory by loading the page table root into CR3
/// 
/// This function performs the critical step of enabling virtual memory by loading
/// the physical address of the PML4 (Page Map Level 4) table into the CR3 register.
/// This makes the CPU use our page tables for address translation.
/// 
/// # Assembly Details
/// - `mov cr3, {page_table_root}`: Load the PML4 physical address into CR3 register
/// 
/// # Parameters
/// 
/// * `page_table_root` - Physical address of the PML4 page table
/// 
/// # Safety
/// 
/// This function is unsafe because it:
/// - Modifies the CR3 control register
/// - Assumes the page table is properly set up
/// - Must be called when identity mapping is active (before high-half transition)
/// 
/// The caller must ensure:
/// - The page table is properly initialized and valid
/// - Identity mapping is active for the current code
/// - The page table root address is valid and accessible
/// - No other code is modifying CR3 concurrently
pub unsafe fn activate_virtual_memory(page_table_root: u64) {
    // Debug: print the page_table_root being loaded and current CR3
    {
        use x86_64::registers::control::Cr3;
        crate::display::kernel_write_line("[dbg] activate_virtual_memory: loading CR3 with=");
        theseus_shared::print_hex_u64_0xe9!(page_table_root);
        crate::display::kernel_write_line("\n");
        let (old, _f) = Cr3::read();
        crate::display::kernel_write_line("[dbg] activate_virtual_memory: CR3 before=");
        theseus_shared::print_hex_u64_0xe9!(old.start_address().as_u64());
        crate::display::kernel_write_line("\n");
    }
    // Load our page table root into CR3 to enable virtual memory
    core::arch::asm!(
        "mov cr3, {}",  // Load PML4 physical address into CR3 register
        in(reg) page_table_root,
        options(nomem, nostack, preserves_flags)
    );
    // Confirm CR3 after loading
    {
        use x86_64::registers::control::Cr3;
        let (new, _f2) = Cr3::read();
        crate::display::kernel_write_line("[dbg] activate_virtual_memory: CR3 after=");
        theseus_shared::print_hex_u64_0xe9!(new.start_address().as_u64());
        crate::display::kernel_write_line("\n");
    }
}

// ===================== x86_64 paging integration (gated) =====================
use x86_64::{PhysAddr, VirtAddr, structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, PhysFrame, Size4KiB, OffsetPageTable}};
// control registers used elsewhere; no local Cr0 flags needed here

const UEFI_CONVENTIONAL_MEMORY: u32 = 7; // UEFI spec: conventional memory type

/// Frame allocator built from UEFI memory map with reserved pool for critical structures
///
/// This allocator iterates through the UEFI memory map descriptors and
/// allocates frames from `UEFI_CONVENTIONAL_MEMORY` regions. It maintains
/// state to track the current region and position within that region.
///
/// The allocator skips physical frame 0 to avoid potential issues with
/// null pointer dereferences.
///
/// # Reserved Frame Pool
/// 
/// The allocator includes a small reserved pool (16 frames) to prevent critical
/// kernel structures from being starved by ephemeral allocations. This ensures
/// that page tables, IST stacks, and other essential structures always have
/// frames available even if the general pool is temporarily exhausted.
///
/// - `reserved`: Array storing physical addresses of reserved frames
/// - `reserved_count`: Number of frames currently in the reserved pool
/// - LIFO allocation: Last frame reserved is first frame allocated
/// - Fallback behavior: If reserved pool empty, falls back to general pool
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
}

impl BootFrameAllocator {
    pub fn empty() -> Self {
        Self { base_ptr: core::ptr::null(), desc_size: 0, count: 0, cur_index: 0, cur_next_addr: 0, cur_remaining_pages: 0, reserved: [0u64; 16], reserved_count: 0 }
    }

    pub unsafe fn from_handoff(h: &theseus_shared::handoff::Handoff) -> Self {
        crate::display::kernel_write_line("  [fa] from_handoff begin");
        let base_ptr = h.memory_map_buffer_ptr as *const u8;
        let desc_size = h.memory_map_descriptor_size as usize;
        let count = h.memory_map_entries as usize;
        crate::display::kernel_write_line("  [fa] base_ptr="); theseus_shared::print_hex_u64_0xe9!(base_ptr as u64);
        crate::display::kernel_write_line(" desc_size="); theseus_shared::print_hex_u64_0xe9!(desc_size as u64);
        crate::display::kernel_write_line(" count="); theseus_shared::print_hex_u64_0xe9!(count as u64); crate::display::kernel_write_line("\n");
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
            if typ == UEFI_CONVENTIONAL_MEMORY { conventional_pages = conventional_pages.wrapping_add(num_pages); }
        }
        crate::display::kernel_write_line("  [fa] memmap total_pages="); theseus_shared::print_hex_u64_0xe9!(total_pages as u64); crate::display::kernel_write_line(" conv_pages="); theseus_shared::print_hex_u64_0xe9!(conventional_pages as u64); crate::display::kernel_write_line("\n");

        let mut s = Self { base_ptr, desc_size, count, cur_index: 0, cur_next_addr: 0, cur_remaining_pages: 0, reserved: [0u64; 16], reserved_count: 0 };
        s.advance_to_next_region();
        s
    }

    /// Reserve up to `n` frames from the allocator and store them into the
    /// internal reserved pool for critical kernel use.
    ///
    /// This method allocates frames from the general pool and moves them into
    /// the reserved pool. The reserved pool is used for critical kernel structures
    /// like page tables and IST stacks to prevent them from being starved by
    /// ephemeral allocations.
    ///
    /// # Arguments
    /// - `n`: Maximum number of frames to reserve
    ///
    /// # Returns
    /// - The number of frames actually reserved (may be less than `n` if
    ///   insufficient frames available or reserved pool is full)
    ///
    /// # Example
    /// ```rust
    /// let mut allocator = BootFrameAllocator::from_handoff(handoff);
    /// let reserved_count = allocator.reserve_frames(16);
    /// println!("Reserved {} frames for critical structures", reserved_count);
    /// ```
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
    ///
    /// This method provides frames from the reserved pool using Last In, First Out
    /// (LIFO) allocation. This is used for critical kernel structures that need
    /// guaranteed frame availability.
    ///
    /// # Returns
    /// - `Some(PhysFrame)` if frames are available in the reserved pool
    /// - `None` if the reserved pool is empty
    ///
    /// # Example
    /// ```rust
    /// // Try to get a reserved frame first, fall back to general pool
    /// let frame = allocator.allocate_reserved_frame()
    ///     .or_else(|| allocator.allocate_frame())
    ///     .expect("No frames available");
    /// ```
    pub fn allocate_reserved_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        if self.reserved_count == 0 { return None; }
        self.reserved_count -= 1;
        let pa = self.reserved[self.reserved_count];
        Some(PhysFrame::containing_address(PhysAddr::new(pa)))
    }

    unsafe fn advance_to_next_region(&mut self) {
        while self.cur_index < self.count {
            let p = self.base_ptr.add(self.cur_index * self.desc_size);
            if self.cur_index < 3 {
                crate::display::kernel_write_line("  [fa] desc[");
                let d = self.cur_index as u32; let mut buf=[0u8;3]; let mut n=d; let mut c=0usize; if n==0 { theseus_shared::out_char_0xe9!(b'0'); } else { while n>0 { buf[c]=b'0'+(n%10) as u8; n/=10; c+=1; } while c>0 { c-=1; theseus_shared::out_char_0xe9!(buf[c]); } }
                crate::display::kernel_write_line("] @"); theseus_shared::print_hex_u64_0xe9!(p as u64); crate::display::kernel_write_line("\n");
            }
            let typ = read_u32(p, 0);
            let phys_start = read_u64(p, 8);
            let num_pages = read_u64(p, 24);
            if self.cur_index < 3 {
                crate::display::kernel_write_line("    type="); theseus_shared::print_hex_u64_0xe9!(typ as u64);
                crate::display::kernel_write_line(" start="); theseus_shared::print_hex_u64_0xe9!(phys_start);
                crate::display::kernel_write_line(" pages="); theseus_shared::print_hex_u64_0xe9!(num_pages); crate::display::kernel_write_line("\n");
            }
            self.cur_index += 1;
            if typ == UEFI_CONVENTIONAL_MEMORY && num_pages > 0 {
                let aligned_start = phys_start & !((PAGE_SIZE as u64) - 1);
                let adj_pages = if aligned_start > phys_start { num_pages.saturating_sub(1) } else { num_pages };
                if adj_pages == 0 { continue; }
                // Avoid returning physical frame 0 (page 0); skip the first page if region starts at 0
                if aligned_start == 0 {
                    if adj_pages <= 1 { continue; }
                    self.cur_next_addr = aligned_start + PAGE_SIZE as u64;
                    self.cur_remaining_pages = adj_pages.saturating_sub(1);
                } else {
                    self.cur_next_addr = aligned_start;
                    self.cur_remaining_pages = adj_pages;
                }
                crate::display::kernel_write_line("  [fa] region start="); theseus_shared::print_hex_u64_0xe9!(self.cur_next_addr);
                crate::display::kernel_write_line(" pages="); theseus_shared::print_hex_u64_0xe9!(self.cur_remaining_pages); crate::display::kernel_write_line("\n");
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
                unsafe { self.advance_to_next_region(); }
                if self.cur_remaining_pages == 0 { return None; }
            }
            let addr = self.cur_next_addr;
            self.cur_next_addr = self.cur_next_addr.saturating_add(PAGE_SIZE as u64);
            self.cur_remaining_pages = self.cur_remaining_pages.saturating_sub(1);
            // Quiet: suppress per-frame allocation logging
            return Some(PhysFrame::containing_address(PhysAddr::new(addr)));
        }
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

/// Map the kernel's physical image range into the high-half window using 4KiB pages (x86_64 API)
fn map_kernel_high_half_x86<M>(mapper: &mut M, frame_alloc: &mut BootFrameAllocator, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
    let phys_base = handoff.kernel_physical_base;
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 { return; }

    let pages = ((phys_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE; // executable by default
    for i in 0..pages {
        let pa = PhysAddr::new(phys_base + i * PAGE_SIZE as u64);
        let frame = PhysFrame::<Size4KiB>::containing_address(pa);
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(KERNEL_VIRTUAL_BASE + i * PAGE_SIZE as u64));
                let _ = unsafe { mapper.map_to(page, frame, flags, frame_alloc) }.map(|flush| flush.flush());
    }
}

/// Map framebuffer using x86_64 paging API (4KiB pages)
fn map_framebuffer_x86<M>(mapper: &mut M, frame_alloc: &mut BootFrameAllocator, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
    let fb_physical = handoff.gop_fb_base;
    let fb_virtual = 0xFFFFFFFF90000000u64;
    let fb_size = handoff.gop_fb_size;
    let pages = ((fb_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_EXECUTE;
    for i in 0..pages {
        let pa = PhysAddr::new(fb_physical + i * PAGE_SIZE as u64);
        let frame = PhysFrame::<Size4KiB>::containing_address(pa);
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(fb_virtual + i * PAGE_SIZE as u64));
                // Ignore AlreadyMapped errors by simply continuing
                let _ = unsafe { mapper.map_to(page, frame, flags, frame_alloc) }.map(|flush| flush.flush());
    }
}

/// Map temporary heap using x86_64 paging API (4KiB pages)
fn map_temporary_heap_x86<M>(mapper: &mut M, frame_alloc: &mut BootFrameAllocator, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
    let heap_physical = handoff.temp_heap_base;
    let heap_virtual = 0xFFFFFFFFA0000000u64;
    let heap_size = handoff.temp_heap_size;
    let pages = ((heap_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_EXECUTE;
    for i in 0..pages {
        let pa = PhysAddr::new(heap_physical + i * PAGE_SIZE as u64);
        let frame = PhysFrame::<Size4KiB>::containing_address(pa);
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(heap_virtual + i * PAGE_SIZE as u64));
        let _ = unsafe { mapper.map_to(page, frame, flags, frame_alloc) } .map(|flush| flush.flush());
    }
}

/// Map the permanent kernel heap at KERNEL_HEAP_BASE using fresh frames
pub fn map_kernel_heap_x86<M>(mapper: &mut M, frame_alloc: &mut BootFrameAllocator)
where
    M: Mapper<Size4KiB>,
{
    use x86_64::structures::paging::PageTableFlags as F;
    let pages = (KERNEL_HEAP_SIZE as u64 + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    let flags = F::PRESENT | F::WRITABLE | F::NO_EXECUTE;
    for i in 0..pages {
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(KERNEL_HEAP_BASE + i * PAGE_SIZE as u64));
        if let Some(frame) = frame_alloc.allocate_frame() {
            let _ = unsafe { mapper.map_to(page, frame, flags, frame_alloc) }.map(|flush| flush.flush());
        } else {
            crate::display::kernel_write_line("  [vm] kernel heap map: out of frames");
            break;
        }
    }
}

/// Unmap the temporary boot heap that was mapped at TEMP_HEAP_VIRTUAL_BASE
pub fn unmap_temporary_heap_x86<M>(mapper: &mut M, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
    crate::display::kernel_write_line("[dbg] unmap_temporary_heap_x86: begin\n");
    // Trait not needed for direct calls here
    let heap_virtual = TEMP_HEAP_VIRTUAL_BASE;
    let heap_size = handoff.temp_heap_size;
    if heap_size == 0 { return; }
    let pages = ((heap_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    for i in 0..pages {
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(heap_virtual + i * PAGE_SIZE as u64));
        if let Ok((_frame, flush)) = mapper.unmap(page) {
            flush.flush();
        }
    }
    crate::display::kernel_write_line("[dbg] unmap_temporary_heap_x86: done\n");
}

/// Unmap the identity-mapped kernel image range at low VA to catch stale low-VA uses
pub fn unmap_identity_kernel_x86<M>(mapper: &mut M, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
    crate::display::kernel_write_line("[dbg] unmap_identity_kernel_x86: begin\n");
    // Trait not needed for direct calls here
    let phys_base = handoff.kernel_physical_base;
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 { return; }
    let pages = ((phys_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    for i in 0..pages {
        let va = VirtAddr::new(phys_base + i * PAGE_SIZE as u64);
        let page = Page::<Size4KiB>::containing_address(va);
        if let Ok((_frame, flush)) = mapper.unmap(page) {
            flush.flush();
        }
    }
    crate::display::kernel_write_line("[dbg] unmap_identity_kernel_x86: done\n");
}

/// Identity map first 1 GiB using 2 MiB pages with x86_64 API
fn identity_map_first_1gb_x86(mapper: &mut OffsetPageTable<'static>, frame_alloc: &mut BootFrameAllocator) {
    use x86_64::structures::paging::{PageTableFlags as F, Size2MiB, Mapper};
    let flags = F::PRESENT | F::WRITABLE | F::GLOBAL;
    let two_mb: u64 = 2 * 1024 * 1024;
    let one_gb: u64 = 1024 * 1024 * 1024;
    let mut addr: u64 = 0;
    while addr < one_gb {
        let page = Page::<Size2MiB>::containing_address(VirtAddr::new(addr));
        let frame = PhysFrame::<Size2MiB>::containing_address(PhysAddr::new(addr));
        // Safe under our invariants: we identity map known RAM, providing existing frame
        let _ = unsafe { mapper.map_to(page, frame, flags, frame_alloc) }
            .map(|flush| flush.flush());
        addr += two_mb;
    }
}

