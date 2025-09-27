//! Page table types and helpers moved out of `memory.rs` to reduce file size
//! and separate low-level table layout from mapping logic.

use super::{
    phys_offset_is_active, phys_to_virt_pa, PAGE_MASK, PAGE_SIZE, PTE_PRESENT, PTE_WRITABLE,
};
use crate::memory::frame_allocator::BootFrameAllocator;
use x86_64::structures::paging::FrameAllocator;

/// Page table entry
///
/// Represents a single 64-bit page-table entry storing a physical address and
/// flag bits. This is a thin wrapper over the raw u64 used by the page tables.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct PageTableEntry(pub u64);

impl PageTableEntry {
    pub const fn new(physical_addr: u64, flags: u64) -> Self {
        Self(physical_addr & !PAGE_MASK | flags)
    }
    pub fn physical_addr(&self) -> u64 {
        self.0 & !PAGE_MASK
    }
    pub fn is_present(&self) -> bool {
        self.0 & PTE_PRESENT != 0
    }
    pub fn set_present(&mut self) {
        self.0 |= PTE_PRESENT;
    }
    pub fn flags(&self) -> u64 {
        self.0 & PAGE_MASK
    }
}

/// Page table (512 entries)
///
/// 4KiB-aligned page table containing 512 `PageTableEntry`s.
#[repr(align(4096))]
#[derive(Clone, Copy)]
pub struct PageTable {
    pub entries: [PageTableEntry; 512],
}

impl PageTable {
    pub const fn new() -> Self {
        Self {
            entries: [PageTableEntry(0); 512],
        }
    }
    pub fn get_entry(&mut self, index: usize) -> &mut PageTableEntry {
        &mut self.entries[index]
    }
}

/// Get or create a page table using a frame allocator
///
/// If `entry` already references a present table this returns a mutable
/// reference to that table; otherwise this function allocates a fresh frame
/// (preferring the reserved pool), zeros it, installs the entry and returns
/// a mutable reference to the newly-created table.
///
/// # Safety
/// This function manipulates raw page-table entries and returns a `'static`
/// mutable reference into physical memory. Callers must ensure the returned
/// pointer is used under the correct paging/translation context.
pub unsafe fn get_or_create_page_table_alloc(
    entry: &mut PageTableEntry,
    fa: &mut BootFrameAllocator,
) -> &'static mut PageTable {
    if entry.is_present() {
        let pa = entry.physical_addr();
        if phys_offset_is_active() {
            let va = phys_to_virt_pa(pa) as *mut PageTable;
            &mut *va
        } else {
            &mut *(pa as *mut PageTable)
        }
    } else {
        // Prefer a reserved frame first, falling back to the general pool.
        let frame_opt = fa.allocate_reserved_frame().or_else(|| fa.allocate_frame());
        if let Some(frame) = frame_opt {
            let phys = frame.start_address().as_u64();
            if phys_offset_is_active() {
                if let Some(mut tw) = super::TemporaryWindow::new_from_current_pml4() {
                    let _ = tw.with_mapped_frame(phys, fa, |va| {
                        core::ptr::write_bytes(va as *mut u8, 0, PAGE_SIZE);
                    });
                } else {
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
