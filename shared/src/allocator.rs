#![allow(clippy::missing_safety_doc)]

//! Module: shared::allocator
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//! - docs/axioms/memory.md#A3:-The-boot-path-keeps-a-temporary-heap-before-switching-to-a-permanent-kernel-heap
//!
//! INVARIANTS:
//! - This crate exposes the single global allocator shim shared by bootloader and kernel code.
//! - Before `ExitBootServices`, allocation forwards through bootloader-installed firmware callbacks.
//! - After the kernel maps a writable heap and calls `switch_to_kernel_heap`, allocation uses a local `linked_list_allocator::Heap`.
//! - No crate in this workspace should install a competing global allocator.
//!
//! SAFETY:
//! - Bootloader-installed pre-exit function pointers must remain valid for the entire pre-exit phase.
//! - `init_kernel_heap` is only sound when the supplied region is already mapped, writable, and exclusively owned as heap backing.
//! - Switching to the kernel heap too early or reusing the same backing for another purpose corrupts allocator state globally.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//!
//! Global allocator shim with pre-/post-`ExitBootServices` backends.

use core::alloc::{GlobalAlloc, Layout};
use core::ptr::null_mut;
use core::sync::atomic::{AtomicBool, Ordering};
use linked_list_allocator::Heap;
use spin::Mutex;

/// Signature of pre-exit allocation functions provided by the bootloader.
pub type PreAllocFn = unsafe fn(Layout) -> *mut u8;
pub type PreDeallocFn = unsafe fn(*mut u8, Layout);

struct ShimAllocator {
    ready_post_exit: AtomicBool,
    heap: Mutex<Option<Heap>>,                // post-exit kernel heap
    pre_alloc: Mutex<Option<PreAllocFn>>,     // pre-exit allocate
    pre_dealloc: Mutex<Option<PreDeallocFn>>, // pre-exit deallocate
}

unsafe impl GlobalAlloc for ShimAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if self.ready_post_exit.load(Ordering::Acquire) {
            // Post-ExitBootServices: use kernel heap
            if let Some(heap) = self.heap.lock().as_mut() {
                if let Ok(ptr) = heap.allocate_first_fit(layout) {
                    return ptr.as_ptr();
                }
            }
            return null_mut();
        }
        // Pre-ExitBootServices: forward to UEFI
        if let Some(func) = *self.pre_alloc.lock() {
            return func(layout);
        }
        null_mut()
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if self.ready_post_exit.load(Ordering::Acquire) {
            if let Some(heap) = self.heap.lock().as_mut() {
                if let Some(nn) = core::ptr::NonNull::new(ptr) {
                    heap.deallocate(nn, layout);
                }
            }
            return;
        }
        if let Some(func) = *self.pre_dealloc.lock() {
            func(ptr, layout);
        }
    }
}

#[global_allocator]
static GLOBAL_ALLOCATOR: ShimAllocator = ShimAllocator {
    ready_post_exit: AtomicBool::new(false),
    heap: Mutex::new(None),
    pre_alloc: Mutex::new(None),
    pre_dealloc: Mutex::new(None),
};

/// Install pre-exit allocation functions (bootloader must call early).
pub fn install_pre_exit_allocators(alloc: PreAllocFn, dealloc: PreDeallocFn) {
    *GLOBAL_ALLOCATOR.pre_alloc.lock() = Some(alloc);
    *GLOBAL_ALLOCATOR.pre_dealloc.lock() = Some(dealloc);
}

/// Initialize the post-exit kernel heap (kernel calls after mapping).
///
/// # Safety
/// Caller must ensure `heap_start..heap_start+heap_size` is a valid, mapped writable region.
pub unsafe fn init_kernel_heap(heap_start: *mut u8, heap_size: usize) {
    let mut guard = GLOBAL_ALLOCATOR.heap.lock();
    // Allow reinitialisation while we're still running with the pre-exit allocator.
    // Once we've switched to the permanent heap, leave the existing backing intact.
    if GLOBAL_ALLOCATOR.ready_post_exit.load(Ordering::Acquire) {
        return;
    }
    let mut heap = Heap::empty();
    heap.init(heap_start, heap_size);
    *guard = Some(heap);
}

/// Switch the allocator to use the kernel heap (post-ExitBootServices).
pub fn switch_to_kernel_heap() {
    GLOBAL_ALLOCATOR
        .ready_post_exit
        .store(true, Ordering::Release);
}

// Note: OOM handler is provided by binaries (bootloader/kernel) if needed.
