//! Kernel allocator module
//!
//! This module manages the kernel's heap allocator in the unified single-binary boot.
//! The global allocator is provided by UEFI (via the `global_allocator` feature)
//! and remains active throughout boot. We maintain our own internal heap allocator
//! for use after higher-half transition.

use linked_list_allocator::LockedHeap;

/// Internal heap allocator instance
///
/// This is used by the kernel after transitioning to higher-half and setting up
/// permanent kernel heap mappings. It's not marked as the global allocator since
/// UEFI provides that; instead we initialize this allocator and it's used internally.
pub(crate) static ALLOCATOR_LINKED: LockedHeap = LockedHeap::empty();

/// Initialize the global allocator based on bootloader-provided handoff info.
///
/// This function is called early in kernel initialization but defers the actual
/// allocator setup until after virtual memory is configured. The actual initialization
/// happens in the high-half environment where the temporary heap is properly mapped.
///
/// # Parameters
///
/// * `handoff_addr` - Physical address of the handoff structure (currently unused
///   but kept for API stability)
///
/// # Safety
///
/// This function is safe to call during early kernel initialization. The actual
/// allocator initialization happens later in a controlled environment.
pub fn initialize_heap_from_handoff(handoff_addr: u64) {
    let _ = handoff_addr; // kept for signature stability
    crate::display::kernel_write_line("  Deferring heap init until high-half");
}

/// Initialize the permanent kernel heap once the high-half mappings are ready.
///
/// This function initializes the internal kernel heap allocator with the permanent
/// heap region. Called after higher-half mapping is established.
pub fn initialize_permanent_heap(phys_base: u64, size: usize) {
    unsafe { ALLOCATOR_LINKED.lock().init(phys_base as *mut u8, size) }
}
