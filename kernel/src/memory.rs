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
/// Fixed virtual base where the temporary boot heap is mapped
pub const TEMP_HEAP_VIRTUAL_BASE: u64 = 0xFFFFFFFFA0000000;
/// Physical memory linear mapping base (maps [0..N) -> [PHYS_OFFSET..PHYS_OFFSET+N))
pub const PHYS_OFFSET: u64 = 0xFFFF800000000000;

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
    pub frame_allocator: BootFrameAllocator,
}
// Simple global virt<->phys offset model for early paging
static mut VIRT_PHYS_OFFSET: u64 = 0;
// Physical base of kernel image for page-table pool address translation
static mut KERNEL_PHYS_BASE_FOR_POOL: u64 = 0;

// Early virt/phys helpers no longer used in HH path; keep for legacy/debug if needed
#[allow(dead_code)]
fn virt_to_phys(va: u64) -> u64 { unsafe { va.wrapping_add(VIRT_PHYS_OFFSET) } }

#[allow(dead_code)]
fn phys_to_virt(pa: u64) -> u64 { unsafe { pa.wrapping_sub(VIRT_PHYS_OFFSET) } }

#[allow(dead_code)]
fn set_virt_phys_offset(offset: u64) { unsafe { VIRT_PHYS_OFFSET = offset; } }

/// Expose current virt->phys offset (phys - virt)
#[allow(dead_code)]
pub fn virt_phys_offset() -> u64 { unsafe { VIRT_PHYS_OFFSET } }


impl MemoryManager {
    /// Create a new memory manager
    pub unsafe fn new(handoff: &theseus_shared::handoff::Handoff) -> Self {
            crate::display::kernel_write_line("  [vm/new] start");
            // Make kernel phys base available (legacy var no longer used after pool removal)
            KERNEL_PHYS_BASE_FOR_POOL = handoff.kernel_physical_base;
        // Initialize early frame allocator and allocate a fresh PML4 frame
        let mut early_frame_alloc = BootFrameAllocator::from_handoff(handoff);
        let pml4_frame = early_frame_alloc.allocate_frame().expect("Out of frames for PML4");
        let pml4_phys = pml4_frame.start_address().as_u64();
        core::ptr::write_bytes(pml4_phys as *mut u8, 0, PAGE_SIZE);
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
            crate::display::kernel_write_line("  [vm/new] PML4[HH]=");
            theseus_shared::print_hex_u64_0xe9!(entry_val);
            crate::display::kernel_write_line("\n");
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
        // legacy path removed

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

// Removed static page-table pool; all tables are allocated from BootFrameAllocator

/// Set up identity mapping for first 1 GiB using 2 MiB pages
unsafe fn identity_map_first_1gb_2mb_alloc(pml4: &mut PageTable, fa: &mut BootFrameAllocator) {
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
        let va = PHYS_OFFSET + offset;
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        offset += two_mb;
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

    let pages: u64 = (phys_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL; // executable

    for i in 0..pages {
        let pa = phys_base + i * PAGE_SIZE as u64;
        let va = KERNEL_VIRTUAL_BASE + i * PAGE_SIZE as u64;
        map_page_alloc(pml4, va, pa, flags, fa);
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
    if entry.is_present() {
        let pa = entry.physical_addr();
        let va = pa as *mut PageTable; // identity mapping expected for early boot
        &mut *va
    } else {
        if let Some(frame) = fa.allocate_frame() {
            let phys = frame.start_address().as_u64();
            // Zero via identity mapping (frames currently allocated in low memory)
            core::ptr::write_bytes(phys as *mut u8, 0, PAGE_SIZE);
            *entry = PageTableEntry::new(phys, PTE_PRESENT | PTE_WRITABLE);
            &mut *(phys as *mut PageTable)
        } else {
            panic!("Out of frames for page tables");
        }
    }
}

/// Activate virtual memory
pub unsafe fn activate_virtual_memory(page_table_root: u64) {
    // Switch to our freshly created PML4 (pre-CR3 identity makes this safe)
    core::arch::asm!(
        "mov cr3, {}",
        in(reg) page_table_root,
        options(nomem, nostack, preserves_flags)
    );
}

// ===================== x86_64 paging integration (gated) =====================
use x86_64::{PhysAddr, VirtAddr, structures::paging::{FrameAllocator, Mapper, Page, PageTableFlags, PhysFrame, Size4KiB, OffsetPageTable}};

const UEFI_CONVENTIONAL_MEMORY: u32 = 7; // UEFI spec: conventional memory type

pub struct BootFrameAllocator {
    base_ptr: *const u8,
    desc_size: usize,
    count: usize,
    cur_index: usize,
    cur_next_addr: u64,
    cur_remaining_pages: u64,
}

impl BootFrameAllocator {
    pub fn empty() -> Self {
        Self { base_ptr: core::ptr::null(), desc_size: 0, count: 0, cur_index: 0, cur_next_addr: 0, cur_remaining_pages: 0 }
    }

    pub unsafe fn from_handoff(h: &theseus_shared::handoff::Handoff) -> Self {
        crate::display::kernel_write_line("  [fa] from_handoff begin");
        let base_ptr = h.memory_map_buffer_ptr as *const u8;
        let desc_size = h.memory_map_descriptor_size as usize;
        let count = h.memory_map_entries as usize;
        crate::display::kernel_write_line("  [fa] base_ptr="); theseus_shared::print_hex_u64_0xe9!(base_ptr as u64);
        crate::display::kernel_write_line(" desc_size="); theseus_shared::print_hex_u64_0xe9!(desc_size as u64);
        crate::display::kernel_write_line(" count="); theseus_shared::print_hex_u64_0xe9!(count as u64); crate::display::kernel_write_line("\n");
        let mut s = Self { base_ptr, desc_size, count, cur_index: 0, cur_next_addr: 0, cur_remaining_pages: 0 };
        s.advance_to_next_region();
        s
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
                self.cur_next_addr = aligned_start;
                self.cur_remaining_pages = adj_pages;
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
                if self.cur_remaining_pages == 0 { crate::display::kernel_write_line("  [fa] no more regions\n"); return None; }
            }
            let addr = self.cur_next_addr;
            self.cur_next_addr = self.cur_next_addr.saturating_add(PAGE_SIZE as u64);
            self.cur_remaining_pages = self.cur_remaining_pages.saturating_sub(1);
            crate::display::kernel_write_line("  [fa] alloc frame="); theseus_shared::print_hex_u64_0xe9!(addr); crate::display::kernel_write_line("\n");
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
}

/// Unmap the identity-mapped kernel image range at low VA to catch stale low-VA uses
pub fn unmap_identity_kernel_x86<M>(mapper: &mut M, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
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
