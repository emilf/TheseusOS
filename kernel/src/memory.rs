//! Memory management module
//!
//! Early paging and simple identity/high-half mappings used during kernel bring-up.
//! Many helpers and constants are defined for future expansion and may be unused
//! temporarily while the VM system is still being built out.
#![allow(dead_code)]
#![allow(static_mut_refs)]
//! 
//! This module provides page table creation, virtual memory mapping,
//! and memory management for the kernel.


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

/// Page table entry
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct PageTableEntry(u64);

impl PageTableEntry {
    /// Create a new page table entry
    pub const fn new(physical_addr: u64, flags: u64) -> Self {
        Self(physical_addr & !PAGE_MASK | flags)
    }
    
    /// Get the physical address from the entry
    pub fn physical_addr(&self) -> u64 { self.0 & !PAGE_MASK }
    
    /// Check if the entry is present
    pub fn is_present(&self) -> bool {
        self.0 & PTE_PRESENT != 0
    }
    
    /// Set the entry as present
    pub fn set_present(&mut self) {
        self.0 |= PTE_PRESENT;
    }
    
    /// Get the flags
    pub fn flags(&self) -> u64 { self.0 & PAGE_MASK }
}

/// Page table (512 entries)
#[repr(align(4096))]
#[derive(Clone, Copy)]
pub struct PageTable {
    pub entries: [PageTableEntry; 512],
}

impl PageTable {
    /// Create a new empty page table
    pub const fn new() -> Self {
        Self {
            entries: [PageTableEntry(0); 512],
        }
    }
    
    /// Get a page table entry by index
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
    #[cfg(feature = "new_arch")]
    pub frame_allocator: BootFrameAllocator,
}
// Simple global virt<->phys offset model for early paging
static mut VIRT_PHYS_OFFSET: i64 = 0;

fn virt_to_phys(va: u64) -> u64 {
    unsafe { (va as i64 + VIRT_PHYS_OFFSET) as u64 }
}

fn phys_to_virt(pa: u64) -> u64 {
    unsafe { (pa as i64 - VIRT_PHYS_OFFSET) as u64 }
}

fn set_virt_phys_offset(offset: i64) {
    unsafe { VIRT_PHYS_OFFSET = offset; }
}

/// Expose current virt->phys offset (phys - virt)
pub fn virt_phys_offset() -> i64 { unsafe { VIRT_PHYS_OFFSET } }

/// Compute phys->virt offset (virt - phys)
pub fn virt_offset() -> i64 { unsafe { -VIRT_PHYS_OFFSET } }


impl MemoryManager {
    /// Create a new memory manager
    pub unsafe fn new(handoff: &theseus_shared::handoff::Handoff) -> Self {
        #[cfg(feature = "new_arch")]
        crate::display::kernel_write_line("  [vm/new] start");
        // Establish virt->phys offset for kernel image tables
        set_virt_phys_offset(handoff.kernel_physical_base as i64 - KERNEL_VIRTUAL_BASE as i64);
        // Allocate PML4 from a static pool and get its physical address
        let (pml4, pml4_phys) = alloc_table_from_pool();
        #[cfg(feature = "new_arch")]
        crate::display::kernel_write_line("  [vm/new] got pml4");
        // Use kernel VA for table writes; bootloader already mapped high-half

        // Identity map first 1 GiB
        #[cfg(feature = "new_arch")]
        {
            // Bootstrap with legacy identity map so the x86_64 mapper can access page tables
            identity_map_first_1gb_2mb(pml4);
        }
        #[cfg(not(feature = "new_arch"))]
        {
            // Identity map first 1 GiB using 2MiB pages to cover kernel physical area
            identity_map_first_1gb_2mb(pml4);
        }

        // Map the kernel image high-half
        #[cfg(feature = "new_arch")]
        {
            // defer to x86_64 mapper below
        }
        #[cfg(not(feature = "new_arch"))]
        {
            // Map the kernel image physical range into the high-half at KERNEL_VIRTUAL_BASE using 4KiB pages
            map_kernel_high_half_4k(pml4, handoff);
        }

        // Map framebuffer and temp heap if available (4KiB pages are fine here)
        #[cfg(feature = "new_arch")]
        {
            // Post-CR3, mappings are handled via x86_64 mapper (see environment setup)
        }
        #[cfg(not(feature = "new_arch"))]
        {
            if handoff.gop_fb_base != 0 { map_framebuffer(pml4, handoff); }
            if handoff.temp_heap_base != 0 { map_temporary_heap(pml4, handoff); }
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
            #[cfg(feature = "new_arch")]
            frame_allocator: BootFrameAllocator { regions: Vec::new(), region_index: 0 },
        };
        s
    }

    /// Get the page table root (CR3 value)
    pub fn page_table_root(&self) -> u64 { self.pml4_phys }
}

/// Allocate a page table using the temporary heap
// Static pool of page tables linked into the kernel image
#[repr(align(4096))]
#[derive(Copy, Clone)]
struct RawPage([u8; PAGE_SIZE]);

static mut PAGE_POOL: [RawPage; 64] = [const { RawPage([0; PAGE_SIZE]) }; 64];
static mut PAGE_POOL_NEXT: usize = 0;

unsafe fn alloc_table_from_pool() -> (&'static mut PageTable, u64) {
    if PAGE_POOL_NEXT >= PAGE_POOL.len() { panic!("Out of static page pool"); }
    let ptr = PAGE_POOL[PAGE_POOL_NEXT].0.as_mut_ptr();
    PAGE_POOL_NEXT += 1;
    // Zero via current VA; we're identity-mapped pre-CR3
    core::ptr::write_bytes(ptr, 0, PAGE_SIZE);
    let pa = ptr as u64;
    let table = &mut *(ptr as *mut PageTable);
    (table, pa)
}

/// Set up identity mapping for first 1 GiB using 2 MiB pages
unsafe fn identity_map_first_1gb_2mb(pml4: &mut PageTable) {
    // Identity map using 2MiB pages with NX cleared (executable)
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_PS;
    let gigabyte: u64 = 1 << 30;
    let two_mb: u64 = 2 * 1024 * 1024;
    let mut addr: u64 = 0;
    while addr < gigabyte {
        map_2mb_page(pml4, addr, addr, flags);
        addr += two_mb;
    }
}

/// Map kernel to high-half using a single 2 MiB page
unsafe fn map_high_half_1gb_2mb(pml4: &mut PageTable) {
    // Map [0 .. 1GiB) physical -> [KERNEL_VIRTUAL_BASE .. +1GiB) virtual using 2MiB pages
    let two_mb: u64 = 2 * 1024 * 1024;
    let one_gb: u64 = 1024 * 1024 * 1024;
    let virt_base = KERNEL_VIRTUAL_BASE & !(two_mb - 1);
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL;
    let mut offset: u64 = 0;
    while offset < one_gb {
        let pa = offset;
        let va = virt_base + offset;
        map_2mb_page(pml4, va, pa, flags);
        offset += two_mb;
    }
}

/// Map the kernel's physical image range into the high-half window using 2MiB pages
unsafe fn map_kernel_high_half_2mb(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff) {
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
        map_2mb_page(pml4, va, pa, flags);
        pa += two_mb;
        va += two_mb;
    }
}

/// Map the kernel's physical image range into the high-half window using 4KiB pages
unsafe fn map_kernel_high_half_4k(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff) {
    let phys_base = handoff.kernel_physical_base;
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 { return; }

    let pages: u64 = (phys_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL; // executable

    for i in 0..pages {
        let pa = phys_base + i * PAGE_SIZE as u64;
        let va = KERNEL_VIRTUAL_BASE + i * PAGE_SIZE as u64;
        map_page(pml4, va, pa, flags);
    }
}

/// Map framebuffer
unsafe fn map_framebuffer(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff) {
    let fb_physical = handoff.gop_fb_base;
    let fb_virtual = 0xFFFFFFFF90000000; // Map framebuffer to high memory
    let fb_size = handoff.gop_fb_size;
    let pages = (fb_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    
    for page in 0..pages {
        let physical_addr = fb_physical + page * PAGE_SIZE as u64;
        let virtual_addr = fb_virtual + page * PAGE_SIZE as u64;
        
        map_page(pml4, virtual_addr, physical_addr,
                PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC);
    }
}

/// Map temporary heap
unsafe fn map_temporary_heap(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff) {
    let heap_physical = handoff.temp_heap_base;
    let heap_virtual = 0xFFFFFFFFA0000000; // Map heap to high memory
    let heap_size = handoff.temp_heap_size;
    let pages = (heap_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    
    for page in 0..pages {
        let physical_addr = heap_physical + page * PAGE_SIZE as u64;
        let virtual_addr = heap_virtual + page * PAGE_SIZE as u64;
        
        map_page(pml4, virtual_addr, physical_addr,
                PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC);
    }
}

/// Map a single page
unsafe fn map_page(pml4: &mut PageTable, virtual_addr: u64, physical_addr: u64, flags: u64) {
    // Extract page table indices
    let pml4_index = ((virtual_addr >> 39) & 0x1FF) as usize;
    let pdpt_index = ((virtual_addr >> 30) & 0x1FF) as usize;
    let pd_index = ((virtual_addr >> 21) & 0x1FF) as usize;
    let pt_index = ((virtual_addr >> 12) & 0x1FF) as usize;
    
    // Get or create PDPT
    let pdpt = get_or_create_page_table(pml4.get_entry(pml4_index));
    
    // Get or create PD
    let pd = get_or_create_page_table(pdpt.get_entry(pdpt_index));
    
    // Get or create PT
    let pt = get_or_create_page_table(pd.get_entry(pd_index));
    
    // Set the page table entry
    *pt.get_entry(pt_index) = PageTableEntry::new(physical_addr, flags);
}

/// Map a single 2 MiB page by setting a PD entry with PS
unsafe fn map_2mb_page(pml4: &mut PageTable, virtual_addr: u64, physical_addr: u64, flags: u64) {
    let pml4_index = ((virtual_addr >> 39) & 0x1FF) as usize;
    let pdpt_index = ((virtual_addr >> 30) & 0x1FF) as usize;
    let pd_index = ((virtual_addr >> 21) & 0x1FF) as usize;

    // Ensure next-level tables exist up to PD
    let pdpt = get_or_create_page_table(pml4.get_entry(pml4_index));
    let pd = get_or_create_page_table(pdpt.get_entry(pdpt_index));

    // Set PD entry with PS bit
    *pd.get_entry(pd_index) = PageTableEntry::new(physical_addr, flags | PTE_PS);
}

/// Get or create a page table
unsafe fn get_or_create_page_table(entry: &mut PageTableEntry) -> &mut PageTable {
    if entry.is_present() {
        // Access the existing next-level table via identity VA (pre-CR3)
        (entry.physical_addr() as *mut PageTable).as_mut().unwrap()
    } else {
        // Allocate a new table; write through identity VA
        let (_table, phys) = alloc_table_from_pool();
        *entry = PageTableEntry::new(phys, PTE_PRESENT | PTE_WRITABLE);
        (phys as *mut PageTable).as_mut().unwrap()
    }
}

/// Activate virtual memory
pub unsafe fn activate_virtual_memory(page_table_root: u64) {
    #[cfg(feature = "new_arch")]
    {
        use x86_64::registers::control::Cr3;
        use x86_64::{PhysAddr, structures::paging::PhysFrame};
        Cr3::write(PhysFrame::containing_address(PhysAddr::new(page_table_root)), x86_64::registers::control::Cr3Flags::empty());
        return;
    }
    // Legacy path
    core::arch::asm!(
        "mov cr3, {}",
        in(reg) page_table_root,
        options(nomem, nostack, preserves_flags)
    );
}

// ===================== x86_64 paging integration (gated) =====================
#[cfg(feature = "new_arch")]
use alloc::vec::Vec;

#[cfg(feature = "new_arch")]
use x86_64::{PhysAddr, VirtAddr, structures::paging::{FrameAllocator, Mapper, Page, PageSize, PageTableFlags, PhysFrame, Size4KiB, OffsetPageTable}};

#[cfg(feature = "new_arch")]
const UEFI_CONVENTIONAL_MEMORY: u32 = 7; // UEFI spec: conventional memory type

#[cfg(feature = "new_arch")]
struct MemoryRegion {
    next_addr: u64,
    remaining_pages: u64,
}

#[cfg(feature = "new_arch")]
pub struct BootFrameAllocator {
    regions: Vec<MemoryRegion>,
    region_index: usize,
}

#[cfg(feature = "new_arch")]
impl BootFrameAllocator {
    pub fn empty() -> Self {
        Self { regions: Vec::new(), region_index: 0 }
    }

    pub unsafe fn from_handoff(h: &theseus_shared::handoff::Handoff) -> Self {
        let mut regions: Vec<MemoryRegion> = Vec::new();
        let base_ptr = h.memory_map_buffer_ptr as *const u8;
        let desc_size = h.memory_map_descriptor_size as usize;
        let count = h.memory_map_entries as usize;
        for i in 0..count {
            let p = base_ptr.add(i * desc_size);
            let typ = read_u32(p, 0);
            if typ == UEFI_CONVENTIONAL_MEMORY {
                let phys_start = read_u64(p, 8);
                let num_pages = read_u64(p, 24);
                if num_pages == 0 { continue; }
                // Align start to 4KiB just in case
                let aligned_start = phys_start & !((PAGE_SIZE as u64) - 1);
                let adj_pages = if aligned_start > phys_start {
                    num_pages.saturating_sub(1)
                } else { num_pages };
                if adj_pages == 0 { continue; }
                regions.push(MemoryRegion { next_addr: aligned_start, remaining_pages: adj_pages });
            }
        }
        Self { regions, region_index: 0 }
    }
}

#[cfg(feature = "new_arch")]
unsafe impl FrameAllocator<Size4KiB> for BootFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        while self.region_index < self.regions.len() {
            let region = &mut self.regions[self.region_index];
            if region.remaining_pages == 0 {
                self.region_index += 1;
                continue;
            }
            let addr = region.next_addr;
            region.next_addr = region.next_addr.saturating_add(PAGE_SIZE as u64);
            region.remaining_pages = region.remaining_pages.saturating_sub(1);
            return Some(PhysFrame::containing_address(PhysAddr::new(addr)));
        }
        None
    }
}

#[cfg(feature = "new_arch")]
#[inline(always)]
unsafe fn read_u32(ptr: *const u8, offset: usize) -> u32 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u32)
}

#[cfg(feature = "new_arch")]
#[inline(always)]
unsafe fn read_u64(ptr: *const u8, offset: usize) -> u64 {
    core::ptr::read_unaligned(ptr.add(offset) as *const u64)
}

/// Map the kernel's physical image range into the high-half window using 4KiB pages (x86_64 API)
#[cfg(feature = "new_arch")]
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
        let _ = unsafe { mapper.map_to(page, frame, flags, frame_alloc) } .map(|flush| flush.flush());
    }
}

/// Map framebuffer using x86_64 paging API (4KiB pages)
#[cfg(feature = "new_arch")]
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
        let _ = unsafe { mapper.map_to(page, frame, flags, frame_alloc) } .map(|flush| flush.flush());
    }
}

/// Map temporary heap using x86_64 paging API (4KiB pages)
#[cfg(feature = "new_arch")]
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

/// Identity map first 1 GiB using 2 MiB pages with x86_64 API
#[cfg(feature = "new_arch")]
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
