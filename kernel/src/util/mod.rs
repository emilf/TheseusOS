//! Module: util
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation
//!
//! INVARIANTS:
//! - This module provides general-purpose utility functions for kernel debugging and diagnostics.
//! - Utilities must remain allocation-free and safe for use in panic/interrupt contexts.
//! - Functions should be simple, focused, and follow existing kernel coding patterns.
//!
//! SAFETY:
//! - Raw pointer operations must validate bounds and handle null pointers gracefully.
//! - Memory inspection utilities must not assume the target memory is valid or mapped.
//! - No heap allocation or complex synchronization allowed.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//!
//! General-purpose kernel utilities.
//!
//! This module contains utility functions for debugging, diagnostics, and common
//! operations that don't belong in a specific subsystem.

use crate::log_trace;

/// Dump a memory region in hex+ASCII format to the serial console.
///
/// This function prints a hex dump of the specified memory region, showing both
/// hexadecimal byte values and ASCII representation (with non-printable characters
/// replaced with '.').
///
/// # Parameters
/// - `ptr`: Pointer to the start of the memory region
/// - `len`: Length of the memory region in bytes
/// - `base_addr`: Optional base address to display in the offset column (defaults to pointer value)
///
/// # Safety
/// - The caller must ensure the memory region `[ptr, ptr+len)` is valid and readable.
/// - This function performs raw pointer reads and should only be used for debugging.
pub unsafe fn hexdump(ptr: *const u8, len: usize, base_addr: Option<u64>) {
    if len == 0 {
        log_trace!("hexdump: empty region");
        return;
    }

    let base = base_addr.unwrap_or(ptr as u64);
    let mut offset = 0;

    while offset < len {
        // Print offset
        log_trace!("{:08x}: ", base.wrapping_add(offset as u64));

        // Print hex bytes (16 bytes per line)
        let mut ascii = [b'.'; 16];
        let mut hex_chars = [b' '; 49]; // 16*3 = 48 chars + null terminator
        let mut hex_pos = 0;

        for i in 0..16 {
            if offset + i < len {
                let byte = ptr.add(offset + i).read_volatile();
                
                // Convert byte to hex digits
                let high = (byte >> 4) & 0xF;
                let low = byte & 0xF;
                
                hex_chars[hex_pos] = if high < 10 { b'0' + high } else { b'A' + (high - 10) };
                hex_chars[hex_pos + 1] = if low < 10 { b'0' + low } else { b'A' + (low - 10) };
                hex_chars[hex_pos + 2] = b' ';
                hex_pos += 3;

                // Store ASCII representation
                ascii[i] = if byte.is_ascii_graphic() || byte == b' ' {
                    byte
                } else {
                    b'.'
                };
            } else {
                // Pad with spaces
                hex_chars[hex_pos] = b' ';
                hex_chars[hex_pos + 1] = b' ';
                hex_chars[hex_pos + 2] = b' ';
                hex_pos += 3;
                ascii[i] = b' ';
            }

            // Add extra space after 8 bytes for readability
            if i == 7 {
                hex_chars[hex_pos] = b' ';
                hex_pos += 1;
            }
        }

        // Print hex line
        log_trace!("{}", core::str::from_utf8_unchecked(&hex_chars[..hex_pos]));

        // Print ASCII representation
        log_trace!(" |{}|", core::str::from_utf8_unchecked(&ascii));

        offset += 16;
    }
}

/// Safe wrapper for `hexdump` that validates pointer and length.
///
/// This function performs basic validation before calling the unsafe `hexdump`.
/// It checks for null pointers and handles zero-length regions gracefully.
///
/// # Parameters
/// - `ptr`: Pointer to the start of the memory region
/// - `len`: Length of the memory region in bytes
/// - `base_addr`: Optional base address to display in the offset column
///
/// # Returns
/// - `Ok(())` if the dump was successful
/// - `Err(&'static str)` if validation failed
pub fn hexdump_safe(ptr: *const u8, len: usize, base_addr: Option<u64>) -> Result<(), &'static str> {
    if ptr.is_null() {
        return Err("null pointer");
    }

    if len == 0 {
        log_trace!("hexdump_safe: empty region");
        return Ok(());
    }

    // Note: We can't fully validate the memory range without knowing the mapping,
    // but we at least check for obvious issues.
    unsafe {
        hexdump(ptr, len, base_addr);
    }
    Ok(())
}

/// Dump memory region with a descriptive label.
///
/// This is a convenience wrapper that adds a label to the hex dump output.
///
/// # Parameters
/// - `label`: Descriptive label for the memory region
/// - `ptr`: Pointer to the start of the memory region
/// - `len`: Length of the memory region in bytes
/// - `base_addr`: Optional base address to display in the offset column
///
/// # Safety
/// - Same as `hexdump`
pub unsafe fn hexdump_labeled(
    label: &str,
    ptr: *const u8,
    len: usize,
    base_addr: Option<u64>,
) {
    log_trace!("=== {} ({} bytes) ===", label, len);
    hexdump(ptr, len, base_addr);
    log_trace!("=== end {} ===", label);
}

/// Safe wrapper for `hexdump_labeled`.
///
/// # Parameters
/// - `label`: Descriptive label for the memory region
/// - `ptr`: Pointer to the start of the memory region
/// - `len`: Length of the memory region in bytes
/// - `base_addr`: Optional base address to display in the offset column
///
/// # Returns
/// - `Ok(())` if the dump was successful
/// - `Err(&'static str)` if validation failed
pub fn hexdump_labeled_safe(
    label: &str,
    ptr: *const u8,
    len: usize,
    base_addr: Option<u64>,
) -> Result<(), &'static str> {
    if ptr.is_null() {
        return Err("null pointer");
    }

    if len == 0 {
        log_trace!("hexdump_labeled_safe: empty region for '{}'", label);
        return Ok(());
    }

    unsafe {
        hexdump_labeled(label, ptr, len, base_addr);
    }
    Ok(())
}

/// Check if a pointer is likely valid for reading.
///
/// This is a best-effort check that doesn't guarantee the memory is actually
/// readable or mapped. It only checks for obviously invalid pointers.
///
/// # Parameters
/// - `ptr`: Pointer to check
/// - `len`: Length of the region to check
///
/// # Returns
/// - `true` if the pointer passes basic validation
/// - `false` if the pointer is obviously invalid
pub fn ptr_likely_valid(ptr: *const u8, len: usize) -> bool {
    if ptr.is_null() {
        return false;
    }

    // Check for overflow in pointer arithmetic
    let ptr_usize = ptr as usize;
    if let Some(end) = ptr_usize.checked_add(len) {
        // Simple sanity check: pointer shouldn't wrap around to low addresses
        end >= ptr_usize
    } else {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ptr_likely_valid() {
        // Valid cases
        assert!(ptr_likely_valid(0x1000 as *const u8, 100));
        assert!(ptr_likely_valid(0xFFFFFFFF as *const u8, 1));

        // Invalid cases
        assert!(!ptr_likely_valid(core::ptr::null(), 100));
        assert!(!ptr_likely_valid(0xFFFFFFFF as *const u8, usize::MAX)); // Overflow
    }

    #[test]
    fn test_hexdump_safe_validation() {
        // Null pointer should fail
        assert_eq!(hexdump_safe(core::ptr::null(), 100, None), Err("null pointer"));

        // Zero length should succeed (no-op)
        assert!(hexdump_safe(0x1000 as *const u8, 0, None).is_ok());
    }
}

// Test module for integration testing
#[cfg(test)]
pub mod test_hexdump;