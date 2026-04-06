//! Test module for hexdump utility

use super::*;

/// Test basic hexdump functionality
pub fn test_hexdump_basic() {
    // Create a test buffer with known content
    let test_data = b"Hello, TheseusOS! This is a test buffer for hexdump.";
    
    unsafe {
        util::hexdump(test_data.as_ptr(), test_data.len(), None);
    }
    
    // Test safe wrapper
    let result = util::hexdump_safe(test_data.as_ptr(), test_data.len(), None);
    assert!(result.is_ok(), "hexdump_safe should succeed for valid pointer");
    
    // Test labeled version
    unsafe {
        util::hexdump_labeled("Test Buffer", test_data.as_ptr(), test_data.len(), None);
    }
    
    // Test zero-length region
    let result = util::hexdump_safe(test_data.as_ptr(), 0, None);
    assert!(result.is_ok(), "hexdump_safe should handle zero length");
    
    // Test null pointer (should fail)
    let result = util::hexdump_safe(core::ptr::null(), 100, None);
    assert!(result.is_err(), "hexdump_safe should fail for null pointer");
    
    // Test ptr_likely_valid
    assert!(util::ptr_likely_valid(test_data.as_ptr(), test_data.len()));
    assert!(!util::ptr_likely_valid(core::ptr::null(), 100));
    assert!(!util::ptr_likely_valid(0xFFFFFFFF as *const u8, usize::MAX)); // Overflow
}

/// Test hexdump with specific base address
pub fn test_hexdump_with_base() {
    let test_data = [0x00u8, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
    
    unsafe {
        // Test with explicit base address
        util::hexdump(test_data.as_ptr(), test_data.len(), Some(0x1000));
        
        // Test with pointer value as base (default)
        util::hexdump(test_data.as_ptr(), test_data.len(), None);
    }
}

/// Test hexdump with various buffer sizes
pub fn test_hexdump_various_sizes() {
    // Test small buffer (less than 16 bytes)
    let small = b"Short";
    unsafe {
        util::hexdump(small.as_ptr(), small.len(), None);
    }
    
    // Test exact 16-byte buffer
    let exact = [0u8; 16];
    unsafe {
        util::hexdump(exact.as_ptr(), exact.len(), None);
    }
    
    // Test larger buffer (multiple lines)
    let mut large = [0u8; 48];
    for i in 0..large.len() {
        large[i] = i as u8;
    }
    unsafe {
        util::hexdump(large.as_ptr(), large.len(), None);
    }
}

/// Run all hexdump tests
pub fn run_all_tests() {
    test_hexdump_basic();
    test_hexdump_with_base();
    test_hexdump_various_sizes();
    
    crate::log_info!("All hexdump tests passed!");
}