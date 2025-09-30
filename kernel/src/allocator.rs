//! Kernel allocator module
//!
//! This module sets up the kernel's global allocator. We use
//! `linked_list_allocator::LockedHeap` after switching to the high-half.
//!
//! The allocator is initialized in two phases:
//! 1. Early initialization with a temporary heap from the bootloader
//! 2. Migration to a permanent high-half heap after virtual memory setup

use core::alloc::GlobalAlloc;
use core::sync::atomic::{AtomicBool, Ordering};
use linked_list_allocator::LockedHeap;

/// Global allocator instance
///
/// This is the kernel's global allocator that handles all dynamic memory allocation.
/// It uses a linked list allocator with a mutex for thread safety.
// Global allocator is provided by the UEFI app at the binary level in the
// single-binary configuration. We keep a linked allocator instance here for
// future post-UEFI usage but do not mark it as the global allocator.
pub(crate) static ALLOCATOR_LINKED: LockedHeap = LockedHeap::empty();

/// Shim that delegates to UEFI allocations until we initialize our own heap,
/// then forwards to the permanent kernel heap. This allows restoring the
/// original split-allocator behavior.
pub struct SplitAllocator;

static HEAP_READY: AtomicBool = AtomicBool::new(false);

unsafe impl GlobalAlloc for SplitAllocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        if HEAP_READY.load(Ordering::Acquire) {
            return ALLOCATOR_LINKED.alloc(layout);
        }
        // Before heap init: return null to avoid accidental allocations.
        core::ptr::null_mut()
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: core::alloc::Layout) {
        if HEAP_READY.load(Ordering::Acquire) {
            ALLOCATOR_LINKED.dealloc(ptr, layout);
        }
    }
}

// Use UEFI's global allocator during boot; we'll switch to our permanent heap
// internally via initialize_permanent_heap without changing the global symbol.

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
pub fn initialize_permanent_heap(phys_base: u64, size: usize) {
    unsafe { ALLOCATOR_LINKED.lock().init(phys_base as *mut u8, size) }
    HEAP_READY.store(true, Ordering::Release);
}
