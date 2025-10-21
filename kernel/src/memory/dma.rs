//! DMA-friendly contiguous buffer helpers.
//!
//! This module builds on the persistent physical allocator to hand out
//! physically contiguous, cache-coherent buffers and ensure they are mapped in
//! the kernel address space.  Device drivers can use `DmaBuffer` to obtain
//! aligned memory suitable for descriptor rings or bounce buffers.

use crate::memory::{
    current_pml4_phys, map_range_with_policy, phys_offset_is_active, phys_to_virt_pa, PageTable,
    PTE_PRESENT, PTE_WRITABLE,
};
use crate::physical_memory::{self, AllocError, PersistentFrameAllocator};

/// Contiguous DMA buffer tracking its physical and virtual address.
pub struct DmaBuffer {
    phys_addr: u64,
    virt_addr: u64,
    size: usize,
    mapped_size: usize,
}

impl DmaBuffer {
    /// Allocate a contiguous buffer with the requested size and alignment.
    ///
    /// `align` is specified in bytes and must be a power of two. The buffer is
    /// zeroed on allocation.
    pub fn allocate(size: usize, align: usize) -> Result<Self, AllocError> {
        if size == 0 {
            return Err(AllocError::OutOfMemory);
        }

        let alignment = align.max(1);
        let phys = physical_memory::alloc_contiguous(size as u64, alignment as u64)?;
        let (virt, mapped) = map_dma_region(phys, size);

        unsafe {
            core::ptr::write_bytes(virt as *mut u8, 0, size);
        }

        Ok(Self {
            phys_addr: phys,
            virt_addr: virt,
            size,
            mapped_size: mapped,
        })
    }

    /// Physical address of the buffer.
    pub fn phys_addr(&self) -> u64 {
        self.phys_addr
    }

    /// Virtual address usable by the kernel.
    pub fn virt_addr(&self) -> u64 {
        self.virt_addr
    }

    /// Length in bytes.
    pub fn len(&self) -> usize {
        self.size
    }

    /// Returns a mutable slice view over the buffer.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.virt_addr as *mut u8, self.size) }
    }
}

impl Drop for DmaBuffer {
    fn drop(&mut self) {
        let _ = physical_memory::free_contiguous(self.phys_addr, self.size as u64);
        if !phys_offset_is_active() {
            // If PHYS_OFFSET is inactive this buffer was mapped through the DMA window;
            // we intentionally leave the mapping in place because tearing it down would
            // require a dedicated unmap helper. Future improvements can reclaim the VA.
        }
    }
}

fn map_dma_region(phys_addr: u64, size: usize) -> (u64, usize) {
    let page_size = crate::memory::PAGE_SIZE as u64;
    let phys_base = phys_addr & !(page_size - 1);
    let offset = phys_addr - phys_base;
    let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;
    let virt_base = phys_to_virt_pa(phys_base);

    unsafe {
        let pml4_pa = current_pml4_phys();
        let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
        let pml4 = &mut *pml4_ptr;
        let mut allocator = PersistentFrameAllocator;
        map_range_with_policy(
            pml4,
            virt_base,
            phys_base,
            size_aligned,
            PTE_PRESENT | PTE_WRITABLE,
            &mut allocator,
        );
    }

    (virt_base + offset, size_aligned as usize)
}
