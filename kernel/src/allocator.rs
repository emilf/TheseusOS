//! Kernel allocator module
//! 
//! This module sets up the kernel's global allocator. We use
//! `linked_list_allocator::LockedHeap` after switching to the high-half.

use linked_list_allocator::LockedHeap;

#[global_allocator]
pub(crate) static ALLOCATOR_LINKED: LockedHeap = LockedHeap::empty();

/// Initialize the global allocator based on bootloader-provided handoff info.
/// The actual initialization is deferred until we're in the high-half, where
/// the temporary heap is mapped at a fixed high VA.
pub fn initialize_heap_from_handoff(handoff_addr: u64) {
    let _ = handoff_addr; // kept for signature stability
    crate::display::kernel_write_line("  Deferring heap init until high-half");
}
