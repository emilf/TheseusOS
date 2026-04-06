//! Module: memory_inspect::access
//!
//! Core memory access functions with safety validation.
//!
//! This module provides safe wrappers around raw memory access operations.
//! All functions validate addresses before attempting access and return
//! detailed error information on failure.

use crate::memory_inspect::error::{MemoryAccessError, MemoryResult};
use crate::memory_inspect::in_interrupt_context;
use super::safety::{validate_virtual_range, validate_range, ValidationResult};

/// Read a single byte from memory with validation.
///
/// This function validates the address before attempting to read,
/// catching obvious errors like null pointers and overflow.
///
/// **WARNING**: TheseusOS will PANIC on page faults. Use `validate_range()`
/// heuristic to check memory accessibility before calling this function.
///
/// # Parameters
/// - `addr`: Virtual address to read from
///
/// # Returns
/// - `Ok(byte)` if read successful
/// - `Err(error)` with detailed error information
///
/// # Safety
/// The function validates the address but cannot guarantee the memory
/// is actually mapped or accessible. A page fault will cause a PANIC.
pub fn read_byte(addr: u64) -> MemoryResult<u8> {
    read_byte_skip_validation(addr, false)
}

/// Read a single byte from memory with optional validation.
///
/// Internal function with `skip_validation` parameter for precise control.
/// Use this in interrupt handlers where validation overhead is unacceptable.
///
/// # Parameters
/// - `addr`: Virtual address to read from
/// - `skip_validation`: If true, skip all validation checks
///
/// # Returns
/// - `Ok(byte)` if read successful
/// - `Err(error)` with detailed error information
///
/// # Safety
/// When `skip_validation` is true, the caller must ensure the address
/// is valid and accessible. A page fault will cause a PANIC.
pub fn read_byte_skip_validation(addr: u64, skip_validation: bool) -> MemoryResult<u8> {
    let ptr = addr as *const u8;
    
    // Skip validation if requested or in interrupt context
    if !skip_validation && !in_interrupt_context() {
        match validate_range(ptr, 1) {
            ValidationResult::Valid => (),
            ValidationResult::Invalid(err) => return Err(err),
        }
    }
    
    // Perform the actual read
    // Use read_volatile() to ensure the compiler doesn't reorder or optimize away the read
    unsafe {
        // Note: read_volatile() can still page fault; TheseusOS will PANIC on page faults
        Ok(ptr.read_volatile())
    }
}

/// Read multiple bytes from memory into a buffer with validation.
///
/// **WARNING**: TheseusOS will PANIC on page faults. Use `validate_range()`
/// heuristic to check memory accessibility before calling this function.
///
/// # Parameters
/// - `addr`: Starting virtual address
/// - `buf`: Buffer to read into
///
/// # Returns
/// - `Ok(())` if read successful
/// - `Err(error)` with detailed error information
///
/// # Safety
/// Same as `read_byte()` - validation catches obvious errors but
/// cannot guarantee all memory in the range is accessible.
/// A page fault will cause a PANIC.
pub fn read_bytes(addr: u64, buf: &mut [u8]) -> MemoryResult<()> {
    read_bytes_skip_validation(addr, buf, false)
}

/// Read multiple bytes from memory into a buffer with optional validation.
///
/// Internal function with `skip_validation` parameter for precise control.
/// Use this in interrupt handlers where validation overhead is unacceptable.
///
/// # Parameters
/// - `addr`: Starting virtual address
/// - `buf`: Buffer to read into
/// - `skip_validation`: If true, skip all validation checks
///
/// # Returns
/// - `Ok(())` if read successful
/// - `Err(error)` with detailed error information
///
/// # Safety
/// When `skip_validation` is true, the caller must ensure the address
/// range is valid and accessible. A page fault will cause a PANIC.
pub fn read_bytes_skip_validation(addr: u64, buf: &mut [u8], skip_validation: bool) -> MemoryResult<()> {
    let ptr = addr as *const u8;
    let len = buf.len();
    
    if len == 0 {
        return Ok(());
    }
    
    // Skip validation if requested or in interrupt context
    if !skip_validation && !in_interrupt_context() {
        match validate_range(ptr, len) {
            ValidationResult::Valid => (),
            ValidationResult::Invalid(err) => return Err(err),
        }
    }
    
    // Use chunking for very large reads to avoid long loops
    const CHUNK_SIZE: usize = 4096; // 4KB chunks
    
    if len > CHUNK_SIZE * 4 {
        // For very large reads, process in chunks
        let mut remaining = len;
        let mut current_addr = addr;
        let mut buf_offset = 0;
        
        while remaining > 0 {
            let chunk_len = core::cmp::min(CHUNK_SIZE, remaining);
            
            // Read this chunk
            read_bytes_chunk(current_addr, &mut buf[buf_offset..buf_offset + chunk_len])?;
            
            remaining -= chunk_len;
            current_addr += chunk_len as u64;
            buf_offset += chunk_len;
        }
    } else {
        // For smaller reads, process directly
        read_bytes_chunk(addr, buf)?;
    }
    
    Ok(())
}

/// Read a structure from memory with validation.
///
/// This is a convenience wrapper for reading typed data from memory.
/// The structure must be `Copy` and have no padding or references.
///
/// # Type Parameters
/// - `T`: Type to read (must be `Copy` and `Sized`)
///
/// # Parameters
/// - `addr`: Virtual address to read from
///
/// # Returns
/// - `Ok(value)` if read successful
/// - `Err(error)` with detailed error information
///
/// # Safety
/// The address must be properly aligned for type `T`.
pub fn read_struct<T: Copy + Sized>(addr: u64) -> MemoryResult<T> {
    let ptr = addr as *const T;
    let size = core::mem::size_of::<T>();
    
    // Check alignment
    if (addr as usize) % core::mem::align_of::<T>() != 0 {
        return Err(MemoryAccessError::InvalidAlignment);
    }
    
    // Skip validation in interrupt context
    if !in_interrupt_context() {
        match validate_virtual_range(ptr as *const u8, size) {
            ValidationResult::Valid => (),
            ValidationResult::Invalid(err) => return Err(err),
        }
    }
    
    unsafe {
        Ok(ptr.read_volatile())
    }
}

/// Write a single byte to memory with validation.
///
/// **WARNING**: TheseusOS will PANIC on page faults. Use `validate_range()`
/// heuristic to check memory accessibility before calling this function.
///
/// # Parameters
/// - `addr`: Virtual address to write to
/// - `value`: Byte value to write
///
/// # Returns
/// - `Ok(())` if write successful
/// - `Err(error)` with detailed error information
///
/// # Safety
/// The function validates the address but cannot guarantee the memory
/// is actually mapped or writable. A page fault will cause a PANIC.
pub fn write_byte(addr: u64, value: u8) -> MemoryResult<()> {
    write_byte_skip_validation(addr, value, false)
}

/// Write a single byte to memory with optional validation.
///
/// Internal function with `skip_validation` parameter for precise control.
/// Use this in interrupt handlers where validation overhead is unacceptable.
///
/// # Parameters
/// - `addr`: Virtual address to write to
/// - `value`: Byte value to write
/// - `skip_validation`: If true, skip all validation checks
///
/// # Returns
/// - `Ok(())` if write successful
/// - `Err(error)` with detailed error information
///
/// # Safety
/// When `skip_validation` is true, the caller must ensure the address
/// is valid and writable. A page fault will cause a PANIC.
pub fn write_byte_skip_validation(addr: u64, value: u8, skip_validation: bool) -> MemoryResult<()> {
    let ptr = addr as *mut u8;
    
    // Skip validation if requested or in interrupt context
    if !skip_validation && !in_interrupt_context() {
        match validate_range(ptr as *const u8, 1) {
            ValidationResult::Valid => (),
            ValidationResult::Invalid(err) => return Err(err),
        }
    }
    
    unsafe {
        ptr.write_volatile(value);
    }
    
    Ok(())
}

/// Write multiple bytes to memory with validation.
///
/// # Parameters
/// - `addr`: Starting virtual address
/// - `data`: Bytes to write
///
/// # Returns
/// - `Ok(())` if write successful
/// - `Err(error)` with detailed error information
pub fn write_bytes(addr: u64, data: &[u8]) -> MemoryResult<()> {
    let ptr = addr as *mut u8;
    let len = data.len();
    
    if len == 0 {
        return Ok(());
    }
    
    // Skip validation in interrupt context
    if !in_interrupt_context() {
        match validate_virtual_range(ptr as *const u8, len) {
            ValidationResult::Valid => (),
            ValidationResult::Invalid(err) => return Err(err),
        }
    }
    
    // Invalidate cache for this address range
    if len >= 64 {
        super::clear_cache();
    }
    
    // Use chunking for very large writes to avoid long loops
    const CHUNK_SIZE: usize = 4096; // 4KB chunks
    
    if len > CHUNK_SIZE * 4 {
        // For very large writes, process in chunks
        let mut remaining = len;
        let mut current_addr = addr;
        let mut data_offset = 0;
        
        while remaining > 0 {
            let chunk_len = core::cmp::min(CHUNK_SIZE, remaining);
            
            // Write this chunk
            write_bytes_chunk(current_addr, &data[data_offset..data_offset + chunk_len])?;
            
            remaining -= chunk_len;
            current_addr += chunk_len as u64;
            data_offset += chunk_len;
        }
    } else {
        // For smaller writes, process directly
        write_bytes_chunk(addr, data)?;
    }
    
    Ok(())
}

/// Fill a memory region with a byte value.
///
/// # Parameters
/// - `addr`: Starting virtual address
/// - `len`: Number of bytes to fill
/// - `value`: Byte value to write
///
/// # Returns
/// - `Ok(())` if fill successful
/// - `Err(error)` with detailed error information
pub fn fill_bytes(addr: u64, len: usize, value: u8) -> MemoryResult<()> {
    if len == 0 {
        return Ok(());
    }
    
    // Skip validation in interrupt context
    if !in_interrupt_context() {
        let ptr = addr as *const u8;
        match validate_virtual_range(ptr, len) {
            ValidationResult::Valid => (),
            ValidationResult::Invalid(err) => return Err(err),
        }
    }
    
    // Invalidate cache for this address range
    if len >= 64 {
        super::clear_cache();
    }
    
    // Use chunking for very large fills to avoid long loops
    const CHUNK_SIZE: usize = 4096; // 4KB chunks
    
    if len > CHUNK_SIZE * 4 {
        // For very large fills, process in chunks
        let mut remaining = len;
        let mut current_addr = addr;
        
        while remaining > 0 {
            let chunk_len = core::cmp::min(CHUNK_SIZE, remaining);
            
            // Fill this chunk
            fill_bytes_chunk(current_addr, chunk_len, value)?;
            
            remaining -= chunk_len;
            current_addr += chunk_len as u64;
        }
    } else {
        // For smaller fills, process directly
        fill_bytes_chunk(addr, len, value)?;
    }
    
    Ok(())
}

/// Internal helper to read a chunk of memory
fn read_bytes_chunk(addr: u64, buf: &mut [u8]) -> MemoryResult<()> {
    let ptr = addr as *const u8;
    let len = buf.len();
    
    // Check cache first for aligned 64-byte reads
    if len == 64 && addr % 64 == 0 {
        let cache = super::get_cache().lock();
        if let Some(cached_data) = cache.get_virtual(addr, len) {
            buf.copy_from_slice(cached_data);
            return Ok(());
        }
    }
    
    // Optimize bulk reads using copy_nonoverlapping for large buffers
    // For small buffers or unaligned accesses, fall back to byte-by-byte
    if len >= 8 && (addr as usize) % 8 == 0 && (buf.as_ptr() as usize) % 8 == 0 {
        // Use 64-bit copies for aligned large buffers
        let word_len = len / 8;
        let remainder = len % 8;
        
        let src_words = ptr as *const u64;
        let dst_words = buf.as_mut_ptr() as *mut u64;
        
        for i in 0..word_len {
            unsafe {
                let word = src_words.add(i).read_volatile();
                core::ptr::copy_nonoverlapping(&word, dst_words.add(i), 1);
            }
        }
        
        // Handle remainder bytes
        if remainder > 0 {
            let base_offset = word_len * 8;
            for i in 0..remainder {
                unsafe {
                    buf[base_offset + i] = ptr.add(base_offset + i).read_volatile();
                }
            }
        }
    } else {
        // Fall back to byte-by-byte for unaligned or small buffers
        for i in 0..len {
            unsafe {
                buf[i] = ptr.add(i).read_volatile();
            }
        }
    }
    
    // Cache the result if it's an aligned 64-byte read
    if len == 64 && addr % 64 == 0 {
        let mut cache = super::get_cache().lock();
        cache.set_virtual(addr, buf);
    }
    
    Ok(())
}

/// Internal helper to write a chunk of memory
fn write_bytes_chunk(addr: u64, data: &[u8]) -> MemoryResult<()> {
    let ptr = addr as *mut u8;
    let len = data.len();
    
    // Optimize bulk writes using copy_nonoverlapping for large buffers
    // For small buffers or unaligned accesses, fall back to byte-by-byte
    if len >= 8 && (addr as usize) % 8 == 0 && (data.as_ptr() as usize) % 8 == 0 {
        // Use 64-bit copies for aligned large buffers
        let word_len = len / 8;
        let remainder = len % 8;
        
        let dst_words = ptr as *mut u64;
        let src_words = data.as_ptr() as *const u64;
        
        for i in 0..word_len {
            unsafe {
                let word = core::ptr::read(src_words.add(i));
                dst_words.add(i).write_volatile(word);
            }
        }
        
        // Handle remainder bytes
        if remainder > 0 {
            let base_offset = word_len * 8;
            for i in 0..remainder {
                unsafe {
                    ptr.add(base_offset + i).write_volatile(data[base_offset + i]);
                }
            }
        }
    } else {
        // Fall back to byte-by-byte for unaligned or small buffers
        for i in 0..len {
            unsafe {
                ptr.add(i).write_volatile(data[i]);
            }
        }
    }
    
    Ok(())
}

/// Internal helper to fill a chunk of memory
fn fill_bytes_chunk(addr: u64, len: usize, value: u8) -> MemoryResult<()> {
    let ptr = addr as *mut u8;
    
    // Optimize fill operation using 64-bit pattern for large buffers
    if len >= 32 {
        // Create a 64-bit pattern from the byte value
        let pattern = (value as u64) * 0x0101010101010101u64;
        
        // Fill in 64-bit chunks
        let word_len = len / 8;
        let remainder = len % 8;
        
        let dst_words = ptr as *mut u64;
        
        for i in 0..word_len {
            unsafe {
                dst_words.add(i).write_volatile(pattern);
            }
        }
        
        // Handle remainder bytes
        if remainder > 0 {
            let base_offset = word_len * 8;
            for i in 0..remainder {
                unsafe {
                    ptr.add(base_offset + i).write_volatile(value);
                }
            }
        }
    } else {
        // Fall back to byte-by-byte for small buffers
        for i in 0..len {
            unsafe {
                ptr.add(i).write_volatile(value);
            }
        }
    }
    
    Ok(())
}

/// Read bytes from physical memory (via PHYS_OFFSET mapping).
///
/// # Parameters
/// - `phys_addr`: Physical address to read from
/// - `buf`: Buffer to read into
///
/// # Returns
/// - `Ok(())` if read successful
/// - `Err(error)` with detailed error information
///
/// # Note
/// Requires PHYS_OFFSET mapping to be active. If not, returns
/// `MemoryAccessError::PhysicalNotMapped`.
pub fn read_physical(phys_addr: u64, buf: &mut [u8]) -> MemoryResult<()> {
    super::physical::read_physical_impl(phys_addr, buf)
}

/// Write bytes to physical memory (via PHYS_OFFSET mapping).
///
/// # Parameters
/// - `phys_addr`: Physical address to write to
/// - `data`: Bytes to write
///
/// # Returns
/// - `Ok(())` if write successful
/// - `Err(error)` with detailed error information
pub fn write_physical(phys_addr: u64, data: &[u8]) -> MemoryResult<()> {
    super::physical::write_physical_impl(phys_addr, data)
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_read_byte_valid() {
        // Test reading from a known valid address (stack variable)
        let test_value: u8 = 0x42;
        let addr = &test_value as *const u8 as u64;
        
        match read_byte(addr) {
            Ok(byte) => assert_eq!(byte, 0x42),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_read_byte_null() {
        // Null pointer should fail validation
        match read_byte(0) {
            Ok(_) => panic!("Should have failed on null pointer"),
            Err(MemoryAccessError::NullPointer) => (), // Expected
            Err(e) => panic!("Wrong error: {:?}", e),
        }
    }
    
    #[test]
    fn test_read_byte_non_canonical() {
        // Non-canonical address should fail validation
        // 0x0000800000000000 is non-canonical (bits 48-63 are 0x0000, not 0x0000 or 0xFFFF)
        match read_byte(0x0000800000000000) {
            Ok(_) => panic!("Should have failed on non-canonical address"),
            Err(MemoryAccessError::NonCanonicalAddress(_)) => (), // Expected
            Err(e) => panic!("Wrong error: {:?}", e),
        }
    }
    
    #[test]
    fn test_write_byte() {
        // Test writing to stack memory
        let mut test_value: u8 = 0;
        let addr = &mut test_value as *mut u8 as u64;
        
        match write_byte(addr, 0x55) {
            Ok(()) => assert_eq!(test_value, 0x55),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_write_byte_null() {
        // Null pointer should fail validation
        match write_byte(0, 0x42) {
            Ok(_) => panic!("Should have failed on null pointer"),
            Err(MemoryAccessError::NullPointer) => (), // Expected
            Err(e) => panic!("Wrong error: {:?}", e),
        }
    }
    
    #[test]
    fn test_read_bytes() {
        // Test reading array
        let test_array = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let addr = test_array.as_ptr() as u64;
        let mut buffer = [0u8; 8];
        
        match read_bytes(addr, &mut buffer) {
            Ok(()) => assert_eq!(buffer, test_array),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_read_bytes_aligned_optimization() {
        // Test reading aligned array (should use optimized path)
        let mut test_array = [0u8; 32];
        for i in 0..32 {
            test_array[i] = i as u8;
        }
        
        let addr = test_array.as_ptr() as u64;
        let mut buffer = [0u8; 32];
        
        // Ensure alignment
        assert_eq!(addr as usize % 8, 0);
        assert_eq!(buffer.as_ptr() as usize % 8, 0);
        
        match read_bytes(addr, &mut buffer) {
            Ok(()) => assert_eq!(buffer, test_array),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_read_bytes_unaligned() {
        // Test reading unaligned array (should use fallback path)
        let test_array = [0x11, 0x22, 0x33, 0x44, 0x55];
        let addr = test_array.as_ptr() as u64 + 1; // Make it unaligned
        let mut buffer = [0u8; 4];
        
        match read_bytes(addr, &mut buffer) {
            Ok(()) => assert_eq!(buffer, [0x22, 0x33, 0x44, 0x55]),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_write_bytes() {
        // Test writing array
        let mut test_array = [0u8; 8];
        let addr = test_array.as_mut_ptr() as u64;
        let data = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22];
        
        match write_bytes(addr, &data) {
            Ok(()) => assert_eq!(test_array, data),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_write_bytes_aligned_optimization() {
        // Test writing aligned array (should use optimized path)
        let mut test_array = [0u8; 32];
        let addr = test_array.as_mut_ptr() as u64;
        let mut data = [0u8; 32];
        for i in 0..32 {
            data[i] = i as u8;
        }
        
        // Ensure alignment
        assert_eq!(addr as usize % 8, 0);
        assert_eq!(data.as_ptr() as usize % 8, 0);
        
        match write_bytes(addr, &data) {
            Ok(()) => assert_eq!(test_array, data),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_fill_bytes() {
        // Test filling memory
        let mut test_array = [0u8; 8];
        let addr = test_array.as_mut_ptr() as u64;
        
        match fill_bytes(addr, 8, 0xFF) {
            Ok(()) => assert_eq!(test_array, [0xFF; 8]),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_fill_bytes_large_optimization() {
        // Test filling large buffer (should use optimized path)
        let mut test_array = [0u8; 64];
        let addr = test_array.as_mut_ptr() as u64;
        
        match fill_bytes(addr, 64, 0xAA) {
            Ok(()) => assert_eq!(test_array, [0xAA; 64]),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_fill_bytes_small() {
        // Test filling small buffer (should use fallback path)
        let mut test_array = [0u8; 7];
        let addr = test_array.as_mut_ptr() as u64;
        
        match fill_bytes(addr, 7, 0x55) {
            Ok(()) => assert_eq!(test_array, [0x55; 7]),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_read_struct() {
        #[derive(Debug, Clone, Copy, PartialEq)]
        struct TestStruct {
            a: u32,
            b: u16,
            c: u8,
        }
        
        let test_value = TestStruct { a: 0xDEADBEEF, b: 0xCAFE, c: 0x42 };
        let addr = &test_value as *const TestStruct as u64;
        
        match read_struct::<TestStruct>(addr) {
            Ok(read_value) => assert_eq!(read_value, test_value),
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }
    
    #[test]
    fn test_read_struct_alignment() {
        // Test reading with misaligned address
        let buffer = [0u8; 16];
        let addr = buffer.as_ptr() as u64 + 1; // Misaligned for u32
        
        // This should fail due to alignment
        match read_struct::<u32>(addr) {
            Ok(_) => panic!("Should have failed on alignment"),
            Err(MemoryAccessError::InvalidAlignment) => (), // Expected
            Err(e) => panic!("Wrong error: {:?}", e),
        }
    }
    
    #[test]
    fn test_zero_length_operations() {
        // Test zero-length read
        let mut buffer = [0u8; 0];
        assert!(read_bytes(0x1000, &mut buffer).is_ok());
        
        // Test zero-length write
        let data: [u8; 0] = [];
        assert!(write_bytes(0x1000, &data).is_ok());
        
        // Test zero-length fill
        assert!(fill_bytes(0x1000, 0, 0xFF).is_ok());
    }
    
    #[test]
    fn test_overflow_validation() {
        // Test address overflow
        let max_addr = u64::MAX;
        let mut buffer = [0u8; 16];
        
        match read_bytes(max_addr, &mut buffer) {
            Ok(_) => panic!("Should have failed on overflow"),
            Err(MemoryAccessError::OutOfBounds) => (), // Expected
            Err(e) => panic!("Wrong error: {:?}", e),
        }
    }
}
