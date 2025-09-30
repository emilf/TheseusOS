//! Kernel allocator module
//!
//! This module sets up the kernel's global allocator. We use
//! `linked_list_allocator::LockedHeap` after switching to the high-half.
//!
//! The allocator is initialized in two phases:
//! 1. Early initialization with a temporary heap from the bootloader
//! 2. Migration to a permanent high-half heap after virtual memory setup

use linked_list_allocator::LockedHeap;

/// Global allocator instance
///
/// This is the kernel's global allocator that handles all dynamic memory allocation.
/// It uses a linked list allocator with a mutex for thread safety.
// Global allocator is provided by the UEFI app at the binary level in the
// single-binary configuration. We keep a linked allocator instance here for
// future post-UEFI usage but do not mark it as the global allocator.
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
