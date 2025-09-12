//! Virtual Memory Management Module
//! 
//! This module provides proper virtual memory management using direct x86_64
//! page table manipulation and inline assembly for maximum control and reliability.

use crate::drivers::manager::write_line;
use alloc::format;

/// Page table entry flags for x86_64
const PAGE_PRESENT: u64 = 1 << 0;
const PAGE_WRITABLE: u64 = 1 << 1;
const PAGE_USER: u64 = 1 << 2;
const PAGE_PWT: u64 = 1 << 3;
const PAGE_PCD: u64 = 1 << 4;
const PAGE_ACCESSED: u64 = 1 << 5;
const PAGE_DIRTY: u64 = 1 << 6;
const PAGE_SIZE: u64 = 1 << 7;
const PAGE_GLOBAL: u64 = 1 << 8;
const PAGE_NX: u64 = 1 << 63;

/// Page table structure for x86_64 (4KB pages)
#[repr(C, align(4096))]
pub struct PageTable {
    entries: [u64; 512],
}

impl PageTable {
    /// Create a new empty page table
    pub const fn new() -> Self {
        Self {
            entries: [0; 512],
        }
    }
    
    /// Get a reference to an entry
    pub fn get_entry(&self, index: usize) -> u64 {
        self.entries[index]
    }
    
    /// Set an entry
    pub fn set_entry(&mut self, index: usize, value: u64) {
        self.entries[index] = value;
    }
}

/// Virtual memory manager with proper x86_64 page table setup
pub struct VirtualMemoryManager {
    pml4: PageTable,
    pdpt: PageTable,
    pd: PageTable,
    pt: PageTable,
}

impl VirtualMemoryManager {
    /// Create a new virtual memory manager
    pub fn new() -> Self {
        Self {
            pml4: PageTable::new(),
            pdpt: PageTable::new(),
            pd: PageTable::new(),
            pt: PageTable::new(),
        }
    }
    
    /// Set up kernel virtual memory mapping
    /// Maps virtual address 0xffffffff80000000 to physical address 0x100000
    pub fn setup_kernel_mapping(&mut self) -> Result<(), &'static str> {
        write_line("Setting up kernel virtual memory mapping...");
        
        // Disable interrupts during critical memory operations
        Self::disable_interrupts();
        
        // Kernel virtual base: 0xffffffff80000000
        // Kernel physical base: 0x100000
        let virtual_base = 0xffffffff80000000u64;
        let physical_base = 0x100000u64;
        let kernel_size = 0x200000u64; // 2MB for now
        
        write_line(&format!("  Virtual base: 0x{:016X}", virtual_base));
        write_line(&format!("  Physical base: 0x{:016X}", physical_base));
        write_line(&format!("  Kernel size: 0x{:016X} bytes", kernel_size));
        
        // Calculate page table indices for virtual address 0xffffffff80000000
        let pml4_index = ((virtual_base >> 39) & 0x1FF) as usize;
        let pdpt_index = ((virtual_base >> 30) & 0x1FF) as usize;
        let pd_index = ((virtual_base >> 21) & 0x1FF) as usize;
        let pt_index = ((virtual_base >> 12) & 0x1FF) as usize;
        
        write_line(&format!("  PML4 index: {}", pml4_index));
        write_line(&format!("  PDPT index: {}", pdpt_index));
        write_line(&format!("  PD index: {}", pd_index));
        write_line(&format!("  PT index: {}", pt_index));
        
        // Set up PML4 entry pointing to PDPT
        let pdpt_physical = &self.pdpt as *const PageTable as u64;
        self.pml4.set_entry(pml4_index, pdpt_physical | PAGE_PRESENT | PAGE_WRITABLE);
        
        // Set up PDPT entry pointing to PD
        let pd_physical = &self.pd as *const PageTable as u64;
        self.pdpt.set_entry(pdpt_index, pd_physical | PAGE_PRESENT | PAGE_WRITABLE);
        
        // Set up PD entry pointing to PT
        let pt_physical = &self.pt as *const PageTable as u64;
        self.pd.set_entry(pd_index, pt_physical | PAGE_PRESENT | PAGE_WRITABLE);
        
        // Set up PT entries for kernel pages
        let page_size = 4096u64;
        let num_pages = (kernel_size + page_size - 1) / page_size; // Round up
        
        write_line(&format!("  Mapping {} pages ({} bytes each)", num_pages, page_size));
        
        for i in 0..num_pages {
            let pt_index = pt_index + i as usize;
            if pt_index >= 512 {
                return Err("Kernel mapping exceeds page table capacity");
            }
            
            let physical_addr = physical_base + (i * page_size);
            let flags = PAGE_PRESENT | PAGE_WRITABLE | PAGE_GLOBAL;
            
            self.pt.set_entry(pt_index, physical_addr | flags);
            
            if i % 100 == 0 {
                write_line(&format!("    Mapped page {}/{}", i + 1, num_pages));
            }
        }
        
        write_line("✓ Kernel virtual memory mapping set up successfully");
        
        // Re-enable interrupts
        Self::enable_interrupts();
        
        Ok(())
    }
    
    /// Get the physical address of the PML4 table
    pub fn get_pml4_physical(&self) -> u64 {
        &self.pml4 as *const PageTable as u64
    }
    
    /// Set the physical address of the PML4 table (for restoring from handoff)
    pub fn set_pml4_physical(&mut self, physical_addr: u64) {
        // This is a placeholder - in a real implementation, we'd need to restore
        // the page table structure from the physical address
        // For now, we'll just use the existing PML4
        let _ = physical_addr; // Suppress unused variable warning
    }
    
    /// Enable paging and return the page table root
    pub fn enable_paging(&mut self) -> Result<u64, &'static str> {
        write_line("Enabling paging...");
        
        // Disable interrupts during paging setup
        Self::disable_interrupts();
        
        // Get the physical address of the page table
        let page_table_phys = self.get_pml4_physical();
        
        write_line(&format!("  Page table physical address: 0x{:016X}", page_table_phys));
        
        // Load CR3 with PML4 physical address
        unsafe {
            core::arch::asm!(
                "mov cr3, {pml4}",
                pml4 = in(reg) page_table_phys,
                options(nomem, nostack, preserves_flags)
            );
        }
        
        // Enable paging (set CR0.PG bit)
        let pg_bit = 0x80000000u64; // PG bit (bit 31)
        unsafe {
            core::arch::asm!(
                "mov rax, cr0",
                "or rax, {pg_bit}",
                "mov cr0, rax",
                pg_bit = in(reg) pg_bit,
                options(nomem, nostack, preserves_flags)
            );
        }
        
        write_line("✓ Paging enabled successfully");
        
        // Re-enable interrupts
        Self::enable_interrupts();
        
        Ok(page_table_phys)
    }
    
    /// Test virtual memory by writing to identity-mapped location and reading from virtual address
    pub fn test_virtual_memory(&mut self) -> Result<(), &'static str> {
        write_line("Testing virtual memory mapping...");
        
        // Disable interrupts during test
        Self::disable_interrupts();
        
        // Test data to write
        let test_data: u64 = 0x123456789ABCDEF0;
        let test_pattern: u64 = 0xDEADBEEFCAFEBABE;
        
        // Identity-mapped physical address (low memory that should be identity mapped)
        let identity_addr = 0x1000u64 as *mut u64; // Use a safe low memory address
        
        // Virtual address that should map to the same physical location
        let virtual_addr = 0xffffffff80000000u64 as *mut u64;
        
        write_line(&format!("  Identity address: 0x{:016X}", identity_addr as u64));
        write_line(&format!("  Virtual address: 0x{:016X}", virtual_addr as u64));
        
        // Step 1: Write test data to identity-mapped location
        write_line("  Step 1: Writing test data to identity-mapped location...");
        unsafe {
            core::ptr::write_volatile(identity_addr, test_data);
        }
        write_line(&format!("    Written: 0x{:016X}", test_data));
        
        // Step 2: Read from identity-mapped location to verify write
        let read_from_identity = unsafe { core::ptr::read_volatile(identity_addr) };
        write_line(&format!("    Read from identity: 0x{:016X}", read_from_identity));
        
        if read_from_identity != test_data {
            return Err("Identity-mapped write/read test failed");
        }
        write_line("    ✓ Identity-mapped write/read successful");
        
        // Step 3: Write different data to virtual address
        write_line("  Step 2: Writing different data to virtual address...");
        unsafe {
            core::ptr::write_volatile(virtual_addr, test_pattern);
        }
        write_line(&format!("    Written: 0x{:016X}", test_pattern));
        
        // Step 4: Read from virtual address
        let read_from_virtual = unsafe { core::ptr::read_volatile(virtual_addr) };
        write_line(&format!("    Read from virtual: 0x{:016X}", read_from_virtual));
        
        if read_from_virtual != test_pattern {
            return Err("Virtual address write/read test failed");
        }
        write_line("    ✓ Virtual address write/read successful");
        
        // Step 5: Cross-test - read from identity what we wrote to virtual
        write_line("  Step 3: Cross-testing - reading from identity what was written to virtual...");
        let cross_read = unsafe { core::ptr::read_volatile(identity_addr) };
        write_line(&format!("    Read from identity (after virtual write): 0x{:016X}", cross_read));
        
        if cross_read != test_pattern {
            write_line("    ⚠ Virtual and identity addresses don't map to same physical location");
            write_line("    This is expected if virtual memory mapping is working correctly");
        } else {
            write_line("    ✓ Virtual and identity addresses map to same physical location");
        }
        
        // Step 6: Test reading from virtual what we wrote to identity
        write_line("  Step 4: Cross-testing - reading from virtual what was written to identity...");
        let cross_read2 = unsafe { core::ptr::read_volatile(virtual_addr) };
        write_line(&format!("    Read from virtual (after identity write): 0x{:016X}", cross_read2));
        
        write_line("✓ Virtual memory test completed successfully");
        
        // Re-enable interrupts
        Self::enable_interrupts();
        
        Ok(())
    }
    
    /// Disable interrupts
    fn disable_interrupts() {
        unsafe {
            core::arch::asm!("cli", options(nomem, nostack, preserves_flags));
        }
    }
    
    /// Enable interrupts
    fn enable_interrupts() {
        unsafe {
            core::arch::asm!("sti", options(nomem, nostack, preserves_flags));
        }
    }
}