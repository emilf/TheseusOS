//! Example usage of hexdump utility

use crate::log_info;
use crate::util;

/// Example function demonstrating hexdump usage
pub fn demonstrate_hexdump() {
    log_info!("=== Hexdump Utility Example ===");
    
    // Example 1: Dump a string
    let message = b"TheseusOS kernel debugging utility";
    log_info!("Example 1: Dumping a string");
    unsafe {
        util::hexdump_labeled("Test Message", message.as_ptr(), message.len(), None);
    }
    
    // Example 2: Dump an array with specific base address
    let data = [0xDEu8, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE];
    log_info!("Example 2: Dumping array with base address 0x1000");
    unsafe {
        util::hexdump(data.as_ptr(), data.len(), Some(0x1000));
    }
    
    // Example 3: Using safe wrapper
    log_info!("Example 3: Using safe wrapper");
    match util::hexdump_safe(data.as_ptr(), data.len(), None) {
        Ok(()) => log_info!("Hexdump completed successfully"),
        Err(e) => log_info!("Hexdump failed: {}", e),
    }
    
    // Example 4: Invalid pointer (demonstrating error handling)
    log_info!("Example 4: Testing error handling");
    match util::hexdump_safe(core::ptr::null(), 100, None) {
        Ok(()) => log_info!("Unexpected success"),
        Err(e) => log_info!("Expected error: {}", e),
    }
    
    log_info!("=== End Hexdump Example ===");
}

/// Example of using hexdump for memory inspection
pub fn inspect_memory_region(start: *const u8, size: usize, label: &str) {
    log_info!("Inspecting memory region: {} ({} bytes)", label, size);
    
    if util::ptr_likely_valid(start, size) {
        unsafe {
            util::hexdump_labeled(label, start, size.min(128), None); // Limit to 128 bytes for demo
        }
    } else {
        log_info!("Warning: Pointer validation failed for {}", label);
    }
}