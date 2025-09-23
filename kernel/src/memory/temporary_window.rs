//! TemporaryWindow scoped helper moved out of `memory.rs` to clarify
//! responsibilities. This module provides a small temporary mapper that maps a
//! single physical frame into a fixed virtual window for safe access (e.g., to
//! zero newly allocated page-table frames).

use crate::memory::{PageTable, PageTableEntry, PAGE_SIZE};
use crate::memory::frame_allocator::BootFrameAllocator;

/// Fixed virtual address used for temporary mappings
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
    /// Caller must ensure `pml4_phys` is the physical address of the current PML4
    /// and that PHYS_OFFSET mapping is available (or identity mapping exists for
    /// low physical frames during early boot).
    pub unsafe fn new(pml4_phys: u64) -> Self {
        let pml4_va = crate::memory::phys_to_virt_pa(pml4_phys) as *mut PageTable;
        Self { window_va: TEMP_WINDOW_VA, pml4: pml4_va }
    }

    /// Map a single 4KiB physical frame into the temporary window and return the VA.
    ///
    /// # Safety
    /// `fa` must be a valid BootFrameAllocator used to allocate intermediate page
    /// tables if they are missing. This function will overwrite any existing mapping
    /// at the temporary window.
    pub unsafe fn map_phys_frame(&mut self, phys_frame_pa: u64, fa: &mut BootFrameAllocator) -> u64 {
        let pml4 = &mut *self.pml4;
        // If an existing mapping is present, simply overwrite the PT entry
        super::map_page_alloc(pml4, self.window_va, phys_frame_pa, super::PTE_PRESENT | super::PTE_WRITABLE, fa);
        self.window_va
    }

    /// Convenience constructor that reads CR3 to determine the current PML4 and
    /// returns a new TemporaryWindow. Returns `None` if CR3 appears invalid.
    ///
    /// # Safety
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

    /// Map a physical frame into the temporary window, execute `f` with the
    /// mapped virtual address, then unmap the window. This provides a scoped
    /// API to avoid leaking temporary mappings across early returns or panics.
    ///
    /// # Safety
    /// Caller must ensure `fa` is valid and that mapping this frame is safe.
    pub unsafe fn with_mapped_frame<F, R>(&mut self, phys_frame_pa: u64, fa: &mut BootFrameAllocator, f: F) -> R
    where F: FnOnce(u64) -> R
    {
        let va = self.map_phys_frame(phys_frame_pa, fa);
        let res = f(va);
        // Ensure we unmap before returning
        self.unmap();
        res
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

impl Drop for TemporaryWindow {
    fn drop(&mut self) {
        // SAFETY: unmap is safe to call during drop; it checks presence bits
        // and will no-op if already unmapped. We ignore any errors here as
        // Drop cannot fail.
        unsafe { TemporaryWindow::unmap(self); }
    }

}


