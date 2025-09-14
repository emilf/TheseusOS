//! Memory management module
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
    pub fn physical_addr(&self) -> u64 {
        self.0 & !PAGE_MASK
    }
    
    /// Check if the entry is present
    pub fn is_present(&self) -> bool {
        self.0 & PTE_PRESENT != 0
    }
    
    /// Set the entry as present
    pub fn set_present(&mut self) {
        self.0 |= PTE_PRESENT;
    }
    
    /// Get the flags
    pub fn flags(&self) -> u64 {
        self.0 & PAGE_MASK
    }
}

/// Page table (512 entries)
#[repr(align(4096))]
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
    pub kernel_heap_start: u64,
    pub kernel_heap_end: u64,
}

impl MemoryManager {
    /// Create a new memory manager
    pub unsafe fn new(handoff: &theseus_shared::handoff::Handoff) -> Self {
        // Allocate PML4 table
        let pml4 = allocate_page_table();
        
        // Set up identity mapping for low memory (first 2MB)
        identity_map_low_memory(pml4);
        
        // Map kernel sections to high memory
        map_kernel_sections(pml4, handoff);
        
        // Set up kernel heap
        let kernel_heap_start = KERNEL_HEAP_BASE;
        let kernel_heap_end = kernel_heap_start + KERNEL_HEAP_SIZE as u64;
        
        Self {
            pml4,
            kernel_heap_start,
            kernel_heap_end,
        }
    }
    
    /// Get the page table root (CR3 value)
    pub fn page_table_root(&self) -> u64 {
        self.pml4 as *const PageTable as u64
    }
}

/// Allocate a page table (simplified - uses temporary heap)
unsafe fn allocate_page_table() -> &'static mut PageTable {
    // For now, use a static allocation
    // In a real implementation, this would use a proper allocator
    static mut PAGE_TABLE_STORAGE: [PageTable; 4] = [
        PageTable::new(),
        PageTable::new(),
        PageTable::new(),
        PageTable::new(),
    ];
    
    static mut ALLOCATION_INDEX: usize = 0;
    
    if ALLOCATION_INDEX >= PAGE_TABLE_STORAGE.len() {
        panic!("Out of page table storage");
    }
    
    let table = &mut PAGE_TABLE_STORAGE[ALLOCATION_INDEX];
    ALLOCATION_INDEX += 1;
    
    table
}

/// Set up identity mapping for low memory
unsafe fn identity_map_low_memory(pml4: &mut PageTable) {
    // Map first 2MB (512 pages) with identity mapping
    // This allows safe transition from physical to virtual addressing
    
    for page in 0..512 {
        let physical_addr = (page * PAGE_SIZE) as u64;
        let virtual_addr = physical_addr; // Identity mapping
        
        map_page(pml4, virtual_addr, physical_addr, 
                PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC);
    }
}

/// Map kernel sections to high memory
unsafe fn map_kernel_sections(pml4: &mut PageTable, handoff: &theseus_shared::handoff::Handoff) {
    let kernel_physical_base = handoff.kernel_physical_base;
    let kernel_virtual_base = KERNEL_VIRTUAL_BASE;
    
    // Map kernel code and data sections
    // For simplicity, map 2MB of kernel space
    for page in 0..512 {
        let physical_addr = kernel_physical_base + (page * PAGE_SIZE) as u64;
        let virtual_addr = kernel_virtual_base + (page * PAGE_SIZE) as u64;
        
        map_page(pml4, virtual_addr, physical_addr,
                PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC);
    }
    
    // Map framebuffer if available
    if handoff.gop_fb_base != 0 {
        map_framebuffer(pml4, handoff);
    }
    
    // Map temporary heap if available
    if handoff.temp_heap_base != 0 {
        map_temporary_heap(pml4, handoff);
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

/// Get or create a page table
unsafe fn get_or_create_page_table(entry: &mut PageTableEntry) -> &mut PageTable {
    if entry.is_present() {
        // Return existing page table
        (entry.physical_addr() as *mut PageTable).as_mut().unwrap()
    } else {
        // Create new page table
        let new_table = allocate_page_table();
        *entry = PageTableEntry::new(new_table as *const PageTable as u64, 
                                   PTE_PRESENT | PTE_WRITABLE | PTE_USER);
        new_table
    }
}

/// Activate virtual memory
pub unsafe fn activate_virtual_memory(page_table_root: u64) {
    // Load page table root into CR3
    core::arch::asm!(
        "mov cr3, {}",
        in(reg) page_table_root,
        options(nomem, nostack, preserves_flags)
    );
    
    // Paging is already enabled in CR0, so we're done
}
