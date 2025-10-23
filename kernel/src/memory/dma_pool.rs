//! Fixed-size DMA buffer pool backed by `DmaBuffer` slabs.
//!
//! This module provides a memory pool allocator specifically designed for DMA operations.
//! It allocates large slabs of contiguous DMA memory and carves them into fixed-size
//! blocks, providing efficient allocation and deallocation of DMA buffers.
//!
//! ## Design Philosophy
//!
//! The pool is designed to minimize fragmentation and allocation overhead:
//! - **Slab-based**: Allocates large contiguous slabs and subdivides them
//! - **Fixed-size blocks**: All blocks in a pool are the same size
//! - **Fast allocation**: O(1) allocation from a free list
//! - **Automatic cleanup**: Blocks are automatically returned to the pool when dropped
//!
//! ## Use Cases
//!
//! This pool is ideal for:
//! - Network packet buffers
//! - Storage I/O buffers
//! - Device descriptor rings
//! - Any scenario requiring many small, fixed-size DMA buffers
//!
//! ## Example Usage
//!
//! ```rust,no_run
//! // Create a pool for 1KB blocks
//! let mut pool = DmaPool::new(1024, CachePolicy::WriteBack);
//!
//! // Allocate a block
//! let block = pool.allocate()?;
//! let phys_addr = block.phys_addr();
//! let size = block.len();
//!
//! // Block is automatically returned to pool when dropped
//! ```

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::marker::PhantomData;

use super::dma::{CachePolicy, DmaBuffer};

/// Simple pool that hands out fixed-size DMA blocks carved from larger slabs.
///
/// This structure manages a collection of DMA buffers (slabs) and provides
/// efficient allocation of fixed-size blocks from those slabs. When a block
/// is allocated, it's removed from the free list. When the block is dropped,
/// it's automatically returned to the free list for reuse.
///
/// The pool grows dynamically by allocating new slabs when the free list
/// is exhausted. This provides good performance for both small and large
/// numbers of concurrent allocations.
pub struct DmaPool {
    /// Size of each block in bytes
    block_size: usize,
    /// Cache policy applied to all slabs in this pool
    cache_policy: CachePolicy,
    /// Number of blocks per slab (configurable for tuning)
    blocks_per_slab: usize,
    /// Collection of DMA buffer slabs
    slabs: Vec<DmaBuffer>,
    /// Free list of available blocks: (slab_index, block_index)
    free_list: VecDeque<(usize, usize)>,
}

impl DmaPool {
    /// Create a new DMA pool with default slab size.
    ///
    /// This creates a pool with 8 blocks per slab, which provides a good
    /// balance between memory usage and allocation performance for most
    /// use cases.
    ///
    /// # Arguments
    /// * `block_size` - Size of each block in bytes
    /// * `cache_policy` - Cache policy for all slabs in this pool
    ///
    /// # Returns
    /// A new DMA pool ready for allocation
    pub fn new(block_size: usize, cache_policy: CachePolicy) -> Self {
        Self::with_slab(block_size, cache_policy, 8)
    }

    /// Create a new DMA pool with custom slab configuration.
    ///
    /// This allows fine-tuning of the pool's memory usage characteristics.
    /// Larger slabs reduce allocation overhead but use more memory upfront.
    /// Smaller slabs use less memory but may require more frequent allocations.
    ///
    /// # Arguments
    /// * `block_size` - Size of each block in bytes
    /// * `cache_policy` - Cache policy for all slabs in this pool
    /// * `blocks_per_slab` - Number of blocks to allocate per slab
    ///
    /// # Returns
    /// A new DMA pool ready for allocation
    ///
    /// # Panics
    /// Panics if `block_size` or `blocks_per_slab` is zero
    pub fn with_slab(block_size: usize, cache_policy: CachePolicy, blocks_per_slab: usize) -> Self {
        assert!(block_size > 0);
        assert!(blocks_per_slab > 0);
        Self {
            block_size,
            cache_policy,
            blocks_per_slab,
            slabs: Vec::new(),
            free_list: VecDeque::new(),
        }
    }

    /// Allocate a block from the pool.
    ///
    /// This method provides O(1) allocation by removing a block from the
    /// free list. If no blocks are available, it automatically allocates
    /// a new slab and adds all blocks from that slab to the free list.
    ///
    /// # Returns
    /// * `Ok(DmaPoolBlock)` - Successfully allocated block
    /// * `Err(&'static str)` - Allocation failed (out of memory)
    ///
    /// # Allocation Strategy
    /// 1. If free blocks are available, return one immediately
    /// 2. If no free blocks, allocate a new slab
    /// 3. Add all blocks from the new slab to the free list
    /// 4. Return the first block from the new slab
    ///
    /// # Example
    /// ```rust,no_run
    /// let mut pool = DmaPool::new(1024, CachePolicy::WriteBack);
    /// let block = pool.allocate()?;
    /// // Use block...
    /// // Block is automatically returned to pool when dropped
    /// ```
    pub fn allocate(&mut self) -> Result<DmaPoolBlock<'_>, &'static str> {
        // If no free blocks available, allocate a new slab
        if self.free_list.is_empty() {
            let total = self.block_size * self.blocks_per_slab;
            let slab = DmaBuffer::allocate_with_policy(total, self.block_size, self.cache_policy)
                .map_err(|_| "dma slab allocation failed")?;
            let slab_index = self.slabs.len();
            self.slabs.push(slab);
            
            // Add all blocks from the new slab to the free list
            for block_index in 0..self.blocks_per_slab {
                self.free_list.push_back((slab_index, block_index));
            }
        }

        // Remove a block from the free list
        let (slab_idx, block_idx) = self.free_list.pop_front().expect("pool must provide block");
        let phys_base = self.slabs[slab_idx].phys_addr();
        
        Ok(DmaPoolBlock {
            pool: self,
            slab_idx,
            block_idx,
            phys_base,
            offset: block_idx * self.block_size,
            size: self.block_size,
            _marker: PhantomData,
        })
    }

    /// Release a block back to the pool.
    ///
    /// This is an internal method called by `DmaPoolBlock::drop`. It adds
    /// the block back to the free list so it can be reused for future
    /// allocations.
    ///
    /// # Arguments
    /// * `slab_idx` - Index of the slab containing the block
    /// * `block_idx` - Index of the block within the slab
    fn release(&mut self, slab_idx: usize, block_idx: usize) {
        self.free_list.push_back((slab_idx, block_idx));
    }

    /// Get the size of blocks in this pool.
    ///
    /// # Returns
    /// The size of each block in bytes
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// Get the cache policy used by this pool.
    ///
    /// # Returns
    /// The cache policy applied to all slabs in this pool
    pub fn cache_policy(&self) -> CachePolicy {
        self.cache_policy
    }
}

/// A block allocated from a DMA pool.
///
/// This represents a single block of DMA memory that has been allocated
/// from a `DmaPool`. The block is automatically returned to the pool when
/// it goes out of scope, ensuring proper resource management.
///
/// The block provides access to the physical address and size, which are
/// the primary pieces of information needed for DMA operations. The actual
/// memory content can be accessed through the pool's underlying DMA buffers
/// if needed.
///
/// # Lifetime
/// The block is tied to the lifetime of the pool (`'pool`) to ensure it
/// cannot outlive the pool that allocated it.
pub struct DmaPoolBlock<'pool> {
    /// Raw pointer to the pool (used for returning the block)
    pool: *mut DmaPool,
    /// Index of the slab containing this block
    slab_idx: usize,
    /// Index of this block within the slab
    block_idx: usize,
    /// Physical base address of the slab
    phys_base: u64,
    /// Offset of this block within the slab
    offset: usize,
    /// Size of this block in bytes
    size: usize,
    /// Phantom data to tie the block to the pool's lifetime
    _marker: PhantomData<&'pool mut DmaPool>,
}

impl<'pool> DmaPoolBlock<'pool> {
    /// Get the physical address of this block.
    ///
    /// This is the address that devices use for DMA operations. It's calculated
    /// as the base address of the slab plus the offset of this block within
    /// the slab.
    ///
    /// # Returns
    /// The physical address of the block
    pub fn phys_addr(&self) -> u64 {
        self.phys_base + self.offset as u64
    }

    /// Get the length of this block in bytes.
    ///
    /// All blocks in a pool have the same size, so this will always return
    /// the same value for blocks from the same pool.
    ///
    /// # Returns
    /// The size of the block in bytes
    pub fn len(&self) -> usize {
        self.size
    }
}

impl<'pool> Drop for DmaPoolBlock<'pool> {
    /// Automatically return the block to the pool when it goes out of scope.
    ///
    /// This implementation ensures that blocks are properly returned to the
    /// pool's free list when they are no longer needed, preventing memory
    /// leaks and enabling reuse of the allocated memory.
    ///
    /// # Safety
    /// This method uses unsafe code to access the pool through a raw pointer.
    /// This is safe because:
    /// 1. The pool is guaranteed to be alive (due to the lifetime parameter)
    /// 2. The block is only dropped when it's no longer being used
    /// 3. The pool's release method is thread-safe for this use case
    fn drop(&mut self) {
        unsafe {
            (*self.pool).release(self.slab_idx, self.block_idx);
        }
    }
}
