//! Fixed-size DMA buffer pool backed by `DmaBuffer` slabs.

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::marker::PhantomData;

use super::dma::{CachePolicy, DmaBuffer};

/// Simple pool that hands out fixed-size DMA blocks carved from larger slabs.
pub struct DmaPool {
    block_size: usize,
    cache_policy: CachePolicy,
    blocks_per_slab: usize,
    slabs: Vec<DmaBuffer>,
    free_list: VecDeque<(usize, usize)>, // (slab_index, block_index)
}

impl DmaPool {
    pub fn new(block_size: usize, cache_policy: CachePolicy) -> Self {
        Self::with_slab(block_size, cache_policy, 8)
    }

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

    pub fn allocate(&mut self) -> Result<DmaPoolBlock<'_>, &'static str> {
        if self.free_list.is_empty() {
            let total = self.block_size * self.blocks_per_slab;
            let slab = DmaBuffer::allocate_with_policy(total, self.block_size, self.cache_policy)
                .map_err(|_| "dma slab allocation failed")?;
            let slab_index = self.slabs.len();
            self.slabs.push(slab);
            for block_index in 0..self.blocks_per_slab {
                self.free_list.push_back((slab_index, block_index));
            }
        }

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

    fn release(&mut self, slab_idx: usize, block_idx: usize) {
        self.free_list.push_back((slab_idx, block_idx));
    }

    pub fn block_size(&self) -> usize {
        self.block_size
    }

    pub fn cache_policy(&self) -> CachePolicy {
        self.cache_policy
    }
}

pub struct DmaPoolBlock<'pool> {
    pool: *mut DmaPool,
    slab_idx: usize,
    block_idx: usize,
    phys_base: u64,
    offset: usize,
    size: usize,
    _marker: PhantomData<&'pool mut DmaPool>,
}

impl<'pool> DmaPoolBlock<'pool> {
    pub fn phys_addr(&self) -> u64 {
        self.phys_base + self.offset as u64
    }

    pub fn len(&self) -> usize {
        self.size
    }
}

impl<'pool> Drop for DmaPoolBlock<'pool> {
    fn drop(&mut self) {
        unsafe {
            (*self.pool).release(self.slab_idx, self.block_idx);
        }
    }
}
