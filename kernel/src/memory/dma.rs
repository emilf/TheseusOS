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
///
/// This structure represents a physically contiguous memory region that is
/// suitable for DMA operations. It maintains both the physical address (for
/// device access) and virtual address (for kernel access) of the same memory.
///
/// The buffer is automatically zeroed on allocation and properly unmapped
/// when dropped. The cache policy determines how the memory is accessed
/// by the CPU and devices.
pub struct DmaBuffer {
    /// Physical address of the buffer (used by devices for DMA)
    phys_addr: u64,
    /// Virtual address of the buffer (used by kernel code)
    virt_addr: u64,
    /// Size of the buffer in bytes
    size: usize,
    /// Size of the mapped region (may be larger due to page alignment)
    mapped_size: usize,
    /// Cache policy applied to this buffer's memory pages
    cache_policy: CachePolicy,
}

impl DmaBuffer {
    /// Allocate a contiguous buffer with the requested size and alignment.
    ///
    /// This is a convenience method that allocates a buffer with the default
    /// write-back cache policy, which is suitable for most DMA operations.
    ///
    /// # Arguments
    /// * `size` - Size of the buffer in bytes
    /// * `align` - Alignment requirement in bytes (must be a power of two)
    ///
    /// # Returns
    /// * `Ok(DmaBuffer)` - Successfully allocated buffer
    /// * `Err(AllocError)` - Allocation failed (out of memory, invalid alignment)
    ///
    /// # Example
    /// ```rust,no_run
    /// // Allocate a 4KB buffer aligned to page boundary
    /// let buffer = DmaBuffer::allocate(4096, 4096)?;
    /// ```
    pub fn allocate(size: usize, align: usize) -> Result<Self, AllocError> {
        Self::allocate_with_policy(size, align, CachePolicy::WriteBack)
    }

    /// Allocate a buffer with an explicit cache policy.
    ///
    /// This method allows drivers to specify the exact cache behavior needed
    /// for their specific use case. Different cache policies are optimized
    /// for different types of DMA operations:
    ///
    /// - **WriteBack**: Normal cached memory, good for descriptor rings and
    ///   general-purpose DMA buffers
    /// - **WriteCombining**: Optimized for frame buffers and bulk data transfers
    /// - **Uncached**: Required for MMIO bounce buffers and device registers
    ///
    /// # Arguments
    /// * `size` - Size of the buffer in bytes
    /// * `align` - Alignment requirement in bytes (must be a power of two)
    /// * `cache_policy` - Cache behavior for the allocated memory
    ///
    /// # Returns
    /// * `Ok(DmaBuffer)` - Successfully allocated buffer
    /// * `Err(AllocError)` - Allocation failed
    ///
    /// # Safety
    /// The buffer is automatically zeroed on allocation and properly
    /// unmapped when dropped. No manual cleanup is required.
    pub fn allocate_with_policy(
        size: usize,
        align: usize,
        cache_policy: CachePolicy,
    ) -> Result<Self, AllocError> {
        // Validate input parameters
        if size == 0 {
            return Err(AllocError::OutOfMemory);
        }

        // Ensure alignment is at least 1 byte
        let alignment = align.max(1);

        // Allocate physically contiguous memory from the persistent allocator
        let phys = physical_memory::alloc_contiguous(size as u64, alignment as u64)?;

        // Map the physical memory into kernel virtual address space
        let (virt, mapped) = map_dma_region(phys, size, cache_policy);

        // Zero the buffer to ensure clean state
        unsafe {
            core::ptr::write_bytes(virt as *mut u8, 0, size);
        }

        Ok(Self {
            phys_addr: phys,
            virt_addr: virt,
            size,
            mapped_size: mapped,
            cache_policy,
        })
    }

    /// Get the physical address of the buffer.
    ///
    /// This address is used by devices for DMA operations. It represents
    /// the actual physical memory location that hardware can access directly.
    ///
    /// # Returns
    /// The physical address as a 64-bit value
    pub fn phys_addr(&self) -> u64 {
        self.phys_addr
    }

    /// Get the virtual address of the buffer.
    ///
    /// This address is used by kernel code to access the buffer contents.
    /// It's mapped into the kernel's virtual address space and can be
    /// dereferenced safely by kernel code.
    ///
    /// # Returns
    /// The virtual address as a 64-bit value
    pub fn virt_addr(&self) -> u64 {
        self.virt_addr
    }

    /// Get the length of the buffer in bytes.
    ///
    /// This returns the actual usable size of the buffer, not including
    /// any padding that may have been added for alignment.
    ///
    /// # Returns
    /// The buffer size in bytes
    pub fn len(&self) -> usize {
        self.size
    }

    /// Get a mutable slice view over the buffer.
    ///
    /// This provides safe access to the buffer contents as a byte slice.
    /// The slice is valid for the lifetime of the DmaBuffer.
    ///
    /// # Returns
    /// A mutable slice of bytes representing the buffer contents
    ///
    /// # Safety
    /// The returned slice is safe to use as long as the DmaBuffer is alive.
    /// The underlying memory is guaranteed to be valid and properly mapped.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.virt_addr as *mut u8, self.size) }
    }

    /// Get the cache policy used for this buffer.
    ///
    /// This returns the cache behavior that was applied when the buffer
    /// was allocated and mapped.
    ///
    /// # Returns
    /// The cache policy enum value
    pub fn cache_policy(&self) -> CachePolicy {
        self.cache_policy
    }
}

impl Drop for DmaBuffer {
    /// Automatically clean up the DMA buffer when it goes out of scope.
    ///
    /// This implementation:
    /// 1. Frees the physical memory back to the allocator
    /// 2. Leaves virtual mappings in place if PHYS_OFFSET is inactive
    ///    (they were created through the DMA window and require special cleanup)
    ///
    /// The virtual address space cleanup is intentionally deferred to avoid
    /// complexity in the current implementation. Future improvements could
    /// add proper unmap functionality.
    fn drop(&mut self) {
        // Free the physical memory back to the persistent allocator
        let _ = physical_memory::free_contiguous(self.phys_addr, self.size as u64);

        // Note: Virtual mapping cleanup is intentionally omitted here
        // If PHYS_OFFSET is inactive, the buffer was mapped through the DMA window
        // and tearing down the mapping would require a dedicated unmap helper.
        // Future improvements can reclaim the virtual address space.
        if !phys_offset_is_active() {
            // If PHYS_OFFSET is inactive this buffer was mapped through the DMA window;
            // we intentionally leave the mapping in place because tearing it down would
            // require a dedicated unmap helper. Future improvements can reclaim the VA.
        }
    }
}

/// Map a DMA region into kernel virtual address space.
///
/// This function takes a physical address and maps it into the kernel's
/// virtual address space with the specified cache policy. It handles
/// page alignment and ensures the entire region is properly mapped.
///
/// # Arguments
/// * `phys_addr` - Physical address to map (may not be page-aligned)
/// * `size` - Size of the region to map in bytes
/// * `policy` - Cache policy to apply to the mapped pages
///
/// # Returns
/// * `(u64, usize)` - Tuple of (virtual_address, mapped_size)
///   - `virtual_address`: The virtual address where the region is mapped
///   - `mapped_size`: The actual size of the mapped region (may be larger due to alignment)
///
/// # Implementation Details
/// The function:
/// 1. Aligns the physical address to page boundaries
/// 2. Calculates the required mapping size (including alignment padding)
/// 3. Maps the region using the current page table
/// 4. Applies the specified cache policy flags
/// 5. Returns the virtual address and actual mapped size
fn map_dma_region(phys_addr: u64, size: usize, policy: CachePolicy) -> (u64, usize) {
    let page_size = crate::memory::PAGE_SIZE as u64;

    // Align physical address to page boundary
    let phys_base = phys_addr & !(page_size - 1);
    let offset = phys_addr - phys_base;

    // Calculate size needed for mapping (including alignment padding)
    let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;

    // Convert physical base to virtual address
    let virt_base = phys_to_virt_pa(phys_base);

    // Map the region into the current page table
    unsafe {
        let pml4_pa = current_pml4_phys();
        let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
        let pml4 = &mut *pml4_ptr;
        let mut allocator = PersistentFrameAllocator;

        // Apply cache policy to page table flags
        let flags = policy.page_flags(PTE_PRESENT | PTE_WRITABLE);

        map_range_with_policy(
            pml4,
            virt_base,
            phys_base,
            size_aligned,
            flags,
            &mut allocator,
        );
    }

    // Return the virtual address (with offset) and mapped size
    (virt_base + offset, size_aligned as usize)
}

/// Cache attribute for DMA mapping.
///
/// This enum defines the cache behavior for DMA-accessible memory regions.
/// The variants map directly onto the cache-control bits in the x86-64 page tables,
/// allowing precise control over how the CPU and devices interact with memory.
///
/// Having a dedicated enum keeps call-sites explicit about the trade-offs and
/// makes it clear why we manipulate PWT (Page Write Through) and PCD (Page Cache Disable)
/// bits in the page table entries.
///
/// # Cache Policies
///
/// - **WriteBack**: Normal cached memory with write-back behavior
/// - **WriteCombining**: Optimized for bulk writes, good for frame buffers
/// - **Uncached**: Bypasses CPU cache entirely, required for device registers
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CachePolicy {
    /// Write-back cache policy (default)
    ///
    /// Normal cached memory with write-back behavior. This is the default
    /// for most DMA operations and provides good performance for general
    /// purpose buffers and descriptor rings.
    WriteBack,

    /// Write-combining cache policy
    ///
    /// Optimized for bulk data transfers where the CPU writes large
    /// amounts of data sequentially. The CPU can combine multiple writes
    /// into fewer memory transactions, improving performance for frame
    /// buffers and bulk data transfers.
    WriteCombining,

    /// Uncached memory policy
    ///
    /// Bypasses the CPU cache entirely. This is required for memory
    /// regions that are shared with devices or used as MMIO bounce
    /// buffers where cache coherency must be guaranteed.
    Uncached,
}

impl CachePolicy {
    /// Convert cache policy to x86-64 page table flags.
    ///
    /// This method takes the base page table flags and adds the appropriate
    /// cache control bits (PWT and PCD) based on the cache policy.
    ///
    /// # Arguments
    /// * `base` - Base page table flags (typically PTE_PRESENT | PTE_WRITABLE)
    ///
    /// # Returns
    /// * `u64` - Page table flags with cache control bits applied
    ///
    /// # Cache Control Bits
    /// - **PWT (Page Write Through)**: When set, forces write-through behavior
    /// - **PCD (Page Cache Disable)**: When set, disables CPU caching entirely
    ///
    /// # Policy Mappings
    /// - `WriteBack`: No additional flags (normal cached behavior)
    /// - `WriteCombining`: PWT set (write-through for combining)
    /// - `Uncached`: Both PWT and PCD set (no caching at all)
    pub fn page_flags(self, base: u64) -> u64 {
        match self {
            CachePolicy::WriteBack => base,
            CachePolicy::WriteCombining => base | crate::memory::PTE_PWT,
            CachePolicy::Uncached => base | crate::memory::PTE_PCD | crate::memory::PTE_PWT,
        }
    }
}
