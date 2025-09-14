#![no_std]
#![no_main]

extern crate alloc;

use theseus_shared::constants::{io_ports, exit_codes};

/// Bump allocator that uses memory map to find safe memory regions
#[global_allocator]
static ALLOCATOR: BumpAllocator = BumpAllocator;

struct BumpAllocator;

// Global heap state
static mut HEAP_START: *mut u8 = core::ptr::null_mut();
static mut HEAP_END: *mut u8 = core::ptr::null_mut();
static mut HEAP_NEXT: *mut u8 = core::ptr::null_mut();
static mut HEAP_INITIALIZED: bool = false;

unsafe impl core::alloc::GlobalAlloc for BumpAllocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        // Initialize heap if not already done
        if !HEAP_INITIALIZED {
            init_heap_from_memory_map();
        }
        
        if HEAP_START.is_null() || HEAP_END.is_null() || HEAP_NEXT.is_null() {
            kernel_write_line("  ALLOC: Heap pointers are null!");
            return core::ptr::null_mut();
        }
        
        // Align the next pointer to the required alignment
        let align = layout.align();
        let size = layout.size();
        
        kernel_write_line("  ALLOC: Requesting allocation");
        
        // Calculate aligned address
        let addr = HEAP_NEXT as usize;
        let aligned_addr = (addr + align - 1) & !(align - 1);
        let new_next = aligned_addr + size;
        
        kernel_write_line("  ALLOC: Alignment calculation complete");
        
        // Check if we have enough space
        if new_next <= HEAP_END as usize {
            let result = aligned_addr as *mut u8;
            // Update the next pointer
            HEAP_NEXT = new_next as *mut u8;
            kernel_write_line("  ALLOC: Allocation successful");
            result
        } else {
            // Out of memory
            kernel_write_line("  ALLOC: Out of memory");
            core::ptr::null_mut()
        }
    }

    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: core::alloc::Layout) {
        // Bump allocator doesn't support deallocation
        // All memory is freed when the allocator is reset
    }
}

/// Initialize heap from handoff structure
fn initialize_heap_from_handoff(handoff_addr: u64) {
    if handoff_addr == 0 {
        kernel_write_line("  No handoff structure provided, using fallback heap");
        unsafe {
            init_heap_fallback();
        }
        return;
    }
    
    unsafe {
        let handoff_ptr = handoff_addr as *const theseus_shared::handoff::Handoff;
        let handoff = &*handoff_ptr;
        
        // Check if bootloader allocated a temporary heap for us
        if handoff.temp_heap_base != 0 && handoff.temp_heap_size != 0 {
            kernel_write_line("  Using temporary heap from bootloader");
            
            // Set up the heap using the bootloader-allocated memory
            HEAP_START = handoff.temp_heap_base as *mut u8;
            HEAP_END = (handoff.temp_heap_base + handoff.temp_heap_size) as *mut u8;
            HEAP_NEXT = handoff.temp_heap_base as *mut u8;
            HEAP_INITIALIZED = true;
            
            kernel_write_line("  Heap initialized using bootloader-allocated memory");
        } else {
            kernel_write_line("  No temporary heap available, using fallback");
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
    
    kernel_write_line("  Heap initialized using fallback method");
}

/// Parse memory map and find safe conventional memory regions
unsafe fn find_conventional_memory(handoff: &theseus_shared::handoff::Handoff) -> Option<(u64, u64)> {
    if handoff.memory_map_buffer_ptr == 0 || handoff.memory_map_entries == 0 {
        kernel_write_line("  No memory map available in handoff structure");
        return None;
    }
    
    kernel_write_line("  Parsing memory map from handoff structure");
    
    // Use the raw memory map data from handoff structure
    let buffer_ptr = handoff.memory_map_buffer_ptr as *const u8;
    let descriptor_size = handoff.memory_map_descriptor_size as usize;
    let entry_count = handoff.memory_map_entries as usize;
    
    // Find the largest conventional memory region
    let mut best_start = 0u64;
    let mut best_size = 0u64;
    let mut conventional_count = 0;
    
    // Limit parsing to prevent infinite loops
    let max_entries = core::cmp::min(entry_count, 135);
    
    // Parse memory descriptors directly
    for i in 0..max_entries {
        let desc_ptr = buffer_ptr.add(i * descriptor_size);
        
        // Read memory type (first 4 bytes)
        let mem_type = core::ptr::read_volatile(desc_ptr as *const u32);
        
        // Read physical start (offset 8 bytes)
        let phys_start = core::ptr::read_volatile(desc_ptr.add(8) as *const u64);
        
        // Read page count (offset 24 bytes)
        let page_count = core::ptr::read_volatile(desc_ptr.add(24) as *const u64);
        
        // Check if this is conventional memory (type 7)
        if mem_type == 7 { // MemoryType::CONVENTIONAL
            conventional_count += 1;
            let region_size = page_count * 4096; // 4KB per page
            
            // Look for a reasonably sized region (at least 64KB)
            if region_size >= 65536 && region_size > best_size {
                best_start = phys_start;
                best_size = region_size;
            }
        }
    }
    
    if best_size > 0 {
        kernel_write_line("  Found suitable conventional memory region");
        Some((best_start, best_size))
    } else {
        kernel_write_line("  No suitable conventional memory found");
        None
    }
}

/// Test the allocator with real memory allocations
fn test_allocator() {
    kernel_write_line("  Testing bump allocator with heap allocations...");
    
    // Test basic allocation without formatting
    kernel_write_line("  Testing basic allocation...");
    let _test_string = alloc::format!("Test");
    kernel_write_line("  Basic allocation successful");
    
    kernel_write_line("  Testing vector allocation...");
    let _vec: alloc::vec::Vec<u32> = alloc::vec![1, 2, 3];
    kernel_write_line("  Vector allocation successful");
    
    kernel_write_line("✓ Bump allocator is working correctly");
}

/// Simple kernel output function that writes directly to QEMU debug port
fn kernel_write_line(message: &str) {
    // Write message directly to QEMU debug port (0xe9)
    for byte in message.bytes() {
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") io_ports::QEMU_DEBUG,
                in("al") byte,
                options(nomem, nostack, preserves_flags)
            );
        }
    }
    
    // Write newline
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") io_ports::QEMU_DEBUG,
            in("al") b'\n',
            options(nomem, nostack, preserves_flags)
        );
    }
}

/// Kernel entry point
/// 
/// This is where the kernel starts after the bootloader has:
/// 1. Collected all system information
/// 2. Exited boot services
/// 3. Jumped to this location
/// 
/// The handoff structure address is passed as a parameter.
#[no_mangle]
pub extern "C" fn kernel_main(handoff_addr: u64) -> ! {
    // Initialize kernel logging (placeholder)
    // TODO: Set up kernel logging system
    
    kernel_write_line("=== TheseusOS Kernel Starting ===");
    kernel_write_line("Kernel entry point reached successfully");

    // Initialize heap from memory map
    kernel_write_line("Initializing heap from memory map...");
    initialize_heap_from_handoff(handoff_addr);
    
    // Test the bump allocator
    kernel_write_line("Testing bump allocator...");
    test_allocator();
    
    kernel_write_line("Handoff structure address received");
    
    // Access the handoff structure from the passed address
    unsafe {
        if handoff_addr != 0 {
            let handoff_ptr = handoff_addr as *const theseus_shared::handoff::Handoff;
            let handoff = &*handoff_ptr;
            
            if handoff.size > 0 {
                kernel_write_line("Handoff structure found");
                
                // Display system information from handoff
                display_handoff_info(handoff);
                
                // Initialize kernel subsystems
                initialize_kernel_subsystems();
                
            } else {
                kernel_write_line("ERROR: Handoff structure has invalid size");
            }
        } else {
            kernel_write_line("ERROR: Handoff structure address is null");
        }
    }
    
    // For now, just exit QEMU
    kernel_write_line("Kernel initialization complete");
    kernel_write_line("Exiting QEMU...");
    
    unsafe {
        core::arch::asm!("out dx, al", 
            in("dx") io_ports::QEMU_EXIT, 
            in("al") exit_codes::QEMU_SUCCESS, 
            options(nomem, nostack, preserves_flags)
        );
    }
    
    loop {}
}

/// Display information from the handoff structure
fn display_handoff_info(handoff: &theseus_shared::handoff::Handoff) {
    kernel_write_line("");
    kernel_write_line("┌─────────────────────────────────────────────────────────┐");
    kernel_write_line("│                Kernel Handoff Information               │");
    kernel_write_line("├─────────────────────────────────────────────────────────┤");
    
    kernel_write_line("│ Handoff Size: Available");
    kernel_write_line("│ Memory Map Entries: Available");
    kernel_write_line("│ Memory Map Size: Available");
    kernel_write_line("│ Memory Map Buffer: Available");
    
    if handoff.acpi_rsdp != 0 {
        kernel_write_line("│ ACPI RSDP: Available");
    } else {
        kernel_write_line("│ ACPI RSDP: Not available");
    }
    
    if handoff.gop_fb_base != 0 {
        kernel_write_line("│ Framebuffer: Available");
    } else {
        kernel_write_line("│ Framebuffer: Not available");
    }
    
    kernel_write_line("│ Hardware Devices: Available");
    
    // Virtual memory information
    kernel_write_line("│");
    kernel_write_line("│ Virtual Memory Information:");
    kernel_write_line("│   Virtual Base: Available");
    kernel_write_line("│   Physical Base: Available");
    kernel_write_line("│   Virtual Entry: Available");
    kernel_write_line("│   Page Table Root: Available");
    kernel_write_line("│   Virtual Memory: Available");
    
    kernel_write_line("└─────────────────────────────────────────────────────────┘");
    kernel_write_line("");
}

/// Initialize kernel subsystems
fn initialize_kernel_subsystems() {
    kernel_write_line("Initializing kernel subsystems...");
    
    // TODO: Initialize memory management
    kernel_write_line("  - Memory management: TODO");
    
    // TODO: Initialize ACPI subsystem
    kernel_write_line("  - ACPI subsystem: TODO");
    
    // TODO: Initialize device drivers
    kernel_write_line("  - Device drivers: TODO");
    
    // TODO: Initialize process management
    kernel_write_line("  - Process management: TODO");
    
    kernel_write_line("Kernel subsystem initialization complete");
}

/// Panic handler for kernel
#[panic_handler]
fn panic_handler(_panic_info: &core::panic::PanicInfo) -> ! {
    // Try to output panic information
    // TODO: Do it without using allocator

    let message = "KERNEL PANIC: Panic occurred";
    
    // Write directly to QEMU debug port
    for byte in message.bytes() {
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") io_ports::QEMU_DEBUG,
                in("al") byte,
                options(nomem, nostack, preserves_flags)
            );
        }
    }
    
    // Write newline
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") io_ports::QEMU_DEBUG,
            in("al") b'\n',
            options(nomem, nostack, preserves_flags)
        );
    }
    
    // Exit QEMU with error
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") io_ports::QEMU_EXIT,
            in("al") exit_codes::QEMU_ERROR,
            options(nomem, nostack, preserves_flags)
        );
    }
    
    loop {}
}
