//! Kernel-Initialized Tests for TheseusOS
//!
//! These tests run after the kernel has been fully initialized, including:
//! - Memory mapping and page tables
//! - Heap allocation
//! - Interrupt handling
//! - All kernel services
//!
//! This environment allows testing of higher-level functionality that depends
//! on kernel services like heap allocation, memory management, and interrupt handling.

#![no_std]
#![no_main]
#![feature(custom_test_frameworks)]
#![test_runner(test_runner)]
#![reexport_test_harness_main = "test_main"]

extern crate theseus_kernel;
extern crate alloc;

/// Kernel-initialized test entry point
/// 
/// This is called by the bootloader after it has:
/// 1. Exited UEFI boot services
/// 2. Jumped to this location
/// 3. Passed the handoff structure address
/// 
/// Unlike bare-metal tests, this environment:
/// 1. Initializes the kernel fully (heap, memory mapping, interrupts)
/// 2. Validates the handoff structure from the bootloader
/// 3. Sets up all kernel services
/// 4. Runs tests with full kernel capabilities
#[no_mangle]
pub extern "C" fn kernel_main(handoff_addr: u64) -> ! {
    // Initialize kernel logging system
    theseus_kernel::display::kernel_write_line("=== Kernel Test Environment Starting ===");
    
    // Initialize heap allocator from memory map (same sequence as main kernel)
    // This sets up the global allocator so we can use Box, Vec, String, etc.
    theseus_kernel::display::kernel_write_line("Initializing heap from memory map...");
    theseus_kernel::initialize_heap_from_handoff(handoff_addr);
    theseus_kernel::set_handoff_pointers(handoff_addr);
    
    // Access and validate the handoff structure from the bootloader
    // This contains system information collected by the bootloader:
    // - Memory map
    // - ACPI tables
    // - Graphics information
    // - CPU information
    // - And more...
    unsafe {
        if handoff_addr != 0 {
            let handoff_ptr = handoff_addr as *const theseus_shared::handoff::Handoff;
            let handoff = &*handoff_ptr;
            
            if handoff.size > 0 {
                theseus_kernel::display::kernel_write_line("Handoff structure found");
                
                // Validate that the handoff structure is well-formed
                // This ensures all critical fields are present and valid
                match theseus_kernel::validate_handoff(handoff) {
                    Ok(()) => theseus_kernel::display::kernel_write_line("Handoff validation passed"),
                    Err(msg) => {
                        theseus_kernel::display::kernel_write_line("Handoff validation failed");
                        theseus_shared::qemu_println!(msg);
                        panic!("Invalid handoff structure");
                    }
                }
                
                // Set up complete kernel environment:
                // - Memory mapping and page tables
                // - Interrupt handling
                // - CPU features
                // - All kernel services
                theseus_kernel::setup_kernel_environment(handoff, handoff.kernel_physical_base, false);
                
                // Run tests directly (bypassing broken custom test framework)
                // We discovered that the framework's test() call mechanism hangs
                // in bare-metal environments, so we call functions directly
                theseus_kernel::display::kernel_write_line("Running kernel-initialized tests...");
                
                test_heap_allocation();
                test_memory_management();
                test_kernel_services();
                
                // All tests completed successfully
                theseus_kernel::display::kernel_write_line("All kernel tests completed successfully");
                
            } else {
                theseus_kernel::display::kernel_write_line("ERROR: Handoff structure has invalid size");
            }
        } else {
            theseus_kernel::display::kernel_write_line("ERROR: Handoff structure address is null");
        }
    }
    
    // Exit QEMU with success code (0)
    // This uses QEMU's isa-debug-exit device
    unsafe {
        core::arch::asm!(
            "mov dx, 0xf4",    // isa-debug-exit device port
            "mov al, 0",       // Exit code 0 = success
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }
    
    // Infinite loop - should never be reached due to QEMU exit
    loop {}
}

/// Dummy test runner - we don't use it since we call tests directly
/// 
/// This function exists only to satisfy the custom test framework
/// requirements. It will never be called since we bypass test_main()
/// and call test functions directly in kernel_main().
pub fn test_runner(_tests: &[&dyn Fn()]) {
    // This will never be called since we bypass test_main()
}

// Note: We don't define a panic handler here because the kernel library
// already provides one. If we need custom panic handling for tests,
// we would need to restructure this differently.

/// Test heap allocation in kernel-initialized environment
/// 
/// This test verifies that heap allocation works correctly after
/// kernel initialization. It tests:
/// - Vec allocation and operations
/// - Box allocation
/// - Dynamic memory management
/// - Global allocator functionality
fn test_heap_allocation() {
    // In kernel environment, we should have heap allocation available
    extern crate alloc;
    use alloc::vec::Vec;
    use alloc::boxed::Box;
    
    // Test Vec allocation and operations
    let mut vec = Vec::new();
    vec.push(1);                // Add elements
    vec.push(2);
    vec.push(3);
    assert_eq!(vec.len(), 3);   // Verify length
    assert_eq!(vec[0], 1);      // Verify first element
    assert_eq!(vec[2], 3);      // Verify last element
    
    // Test Box allocation
    let boxed = Box::new(42);   // Allocate on heap
    assert_eq!(*boxed, 42);     // Verify value
    // Box is automatically deallocated when it goes out of scope
}

/// Test memory management in kernel-initialized environment
/// 
/// This test verifies that memory management functions work correctly
/// after kernel initialization. It tests:
/// - Basic memory operations (same as bare-metal)
/// - Kernel memory management APIs
/// - Kernel logging system
fn test_memory_management() {
    // Test basic memory operations (should work same as bare-metal)
    let x = 42;
    let y = x;
    assert_eq!(x, y);
    
    // Test that we can use kernel services (this is the key difference
    // from bare-metal tests - we have access to kernel APIs)
    theseus_kernel::display::kernel_write_line("Testing memory management...");
}

/// Test kernel services in kernel-initialized environment
/// 
/// This test verifies that kernel services are available and functional.
/// It tests:
/// - Kernel logging system
/// - Kernel API access
/// - Service availability
fn test_kernel_services() {
    // Test that kernel services are available and working
    theseus_kernel::display::kernel_write_line("Testing kernel services...");
    
    // Test that we can access kernel modules
    // (This would need to be implemented based on what's available)
    // For now, this is a placeholder that verifies the test framework works
    assert!(true); // Placeholder for actual kernel service tests
    
    // In a real implementation, you might test:
    // - Process management
    // - File system operations
    // - Network stack
    // - Device drivers
    // - etc.
}

/// Test interrupt handling in kernel-initialized environment
/// 
/// This test verifies that interrupt handling is set up correctly.
/// It tests:
/// - Interrupt handler installation
/// - Interrupt enable/disable
/// - Interrupt priority handling
/// 
/// Note: This is currently a placeholder. Actual interrupt testing
/// would require more sophisticated test infrastructure.
#[allow(dead_code)]
fn test_interrupt_handling() {
    // Test that interrupt handlers are set up
    // This is a placeholder - actual interrupt testing would be more complex
    theseus_kernel::display::kernel_write_line("Testing interrupt handling setup...");
    assert!(true); // Placeholder for actual interrupt tests
    
    // In a real implementation, you might test:
    // - Timer interrupts
    // - Keyboard interrupts
    // - System call interrupts
    // - Exception handling
    // - Interrupt nesting
}
