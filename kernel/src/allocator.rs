//! Kernel allocator module
//! 
//! This module provides the temporary bump allocator used during kernel initialization.
//! It uses memory pre-allocated by the bootloader and stored in the handoff structure.

use theseus_shared::handoff::Handoff;

// Global heap state
static mut HEAP_START: *mut u8 = core::ptr::null_mut();
static mut HEAP_END: *mut u8 = core::ptr::null_mut();
static mut HEAP_NEXT: *mut u8 = core::ptr::null_mut();
static mut HEAP_INITIALIZED: bool = false;

/// Temporary bump allocator for kernel setup phase
/// 
/// This allocator uses memory pre-allocated by the bootloader and stored in the handoff structure.
/// It's designed for use during kernel initialization before proper page tables and memory management
/// are set up. The allocator provides a simple bump allocation strategy without deallocation support.
#[global_allocator]
pub static ALLOCATOR: BumpAllocator = BumpAllocator;

pub struct BumpAllocator;

unsafe impl core::alloc::GlobalAlloc for BumpAllocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        // Initialize heap if not already done
        if !HEAP_INITIALIZED {
            init_heap_from_memory_map();
        }
        
        if HEAP_START.is_null() || HEAP_END.is_null() || HEAP_NEXT.is_null() {
            return core::ptr::null_mut();
        }
        
        // Align the next pointer to the required alignment
        let align = layout.align();
        let size = layout.size();
        
        // Calculate aligned address
        let addr = HEAP_NEXT as usize;
        let aligned_addr = (addr + align - 1) & !(align - 1);
        let new_next = aligned_addr + size;
        
        // Check if we have enough space
        if new_next <= HEAP_END as usize {
            let result = aligned_addr as *mut u8;
            // Update the next pointer
            HEAP_NEXT = new_next as *mut u8;
            result
        } else {
            // Out of memory
            core::ptr::null_mut()
        }
    }

    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: core::alloc::Layout) {
        // Bump allocator doesn't support deallocation
        // All memory is freed when the allocator is reset
    }
}

/// Initialize heap using temporary heap from handoff structure
/// 
/// This function sets up the kernel's heap using memory pre-allocated by the bootloader.
/// The bootloader allocates a chunk of safe memory and stores its address and size in the
/// handoff structure. If no temporary heap is available, falls back to a fixed safe region.
pub fn initialize_heap_from_handoff(handoff_addr: u64) {
    if handoff_addr == 0 {
        crate::display::kernel_write_line("  No handoff structure provided, using fallback heap");
        unsafe {
            init_heap_fallback();
        }
        return;
    }
    
    unsafe {
        let handoff_ptr = handoff_addr as *const Handoff;
        let handoff = &*handoff_ptr;
        
        // Check if bootloader allocated a temporary heap for us
        if handoff.temp_heap_base != 0 && handoff.temp_heap_size != 0 {
            crate::display::kernel_write_line("  Using temporary heap from bootloader");
            
            // Set up the heap using the bootloader-allocated memory
            HEAP_START = handoff.temp_heap_base as *mut u8;
            HEAP_END = (handoff.temp_heap_base + handoff.temp_heap_size) as *mut u8;
            HEAP_NEXT = handoff.temp_heap_base as *mut u8;
            HEAP_INITIALIZED = true;
            
            crate::display::kernel_write_line("  Heap initialized using bootloader-allocated memory");
        } else {
            crate::display::kernel_write_line("  No temporary heap available, using fallback");
            init_heap_fallback();
        }
    }
}

/// Initialize heap from memory map in handoff structure (called by allocator)
unsafe fn init_heap_from_memory_map() {
    // This is called by the allocator if heap is not initialized
    // We'll use a fallback approach
    init_heap_fallback();
}

/// Fallback heap initialization using fixed safe memory region
unsafe fn init_heap_fallback() {
    // Use a safe memory region starting at 1MB
    // This is typically safe conventional memory
    let heap_start_addr = 0x100000usize;  // 1MB
    let heap_size = 0x100000;  // 1MB heap
    
    HEAP_START = heap_start_addr as *mut u8;
    HEAP_END = (heap_start_addr + heap_size) as *mut u8;
    HEAP_NEXT = heap_start_addr as *mut u8;
    HEAP_INITIALIZED = true;
    
    crate::display::kernel_write_line("  Heap initialized using fallback method");
}

/// Test the temporary allocator with basic memory allocations
/// 
/// This function verifies that the bump allocator is working correctly by performing
/// simple string and vector allocations. It uses minimal allocations to avoid complex
/// formatting that could cause issues during early kernel initialization.
#[allow(dead_code)]
pub fn test_allocator() {
    crate::display::kernel_write_line("  Testing bump allocator with heap allocations...");
    
    // Test basic allocation without formatting
    crate::display::kernel_write_line("  Testing basic allocation...");
    let _test_string = alloc::format!("Test");
    crate::display::kernel_write_line("  Basic allocation successful");
    
    crate::display::kernel_write_line("  Testing vector allocation...");
    let _vec: alloc::vec::Vec<u32> = alloc::vec![1, 2, 3];
    crate::display::kernel_write_line("  Vector allocation successful");
    
    crate::display::kernel_write_line("✓ Bump allocator is working correctly");
}
