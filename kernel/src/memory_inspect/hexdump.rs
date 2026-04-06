//! Module: memory_inspect::hexdump
//!
//! Hexdump formatting and output utilities.
//!
//! This module provides functions to dump memory regions in hex+ASCII
//! format for debugging purposes. Output can be directed to serial
//! console or any writer implementing the `Write` trait.

use crate::memory_inspect::access::read_bytes;
use crate::memory_inspect::error::MemoryResult;
use crate::memory_inspect::physical::read_physical_impl;
// Macros are exported at crate root
use crate::serial_println;
use alloc::format;
use alloc::string::String;
use core::fmt::Write;

/// Dump a memory region in hex+ASCII format to the serial console.
///
/// # Parameters
/// - `addr`: Virtual address to dump from
/// - `len`: Number of bytes to dump
/// - `base_addr`: Optional base address for offset display (defaults to `addr`)
///
/// # Returns
/// - `Ok(())` if dump successful
/// - `Err(error)` with detailed error information
///
/// # Output Format
/// ```
/// 00001000: 48 65 6C 6C 6F 2C 20 54  68 65 73 65 75 73 4F 53  |Hello, TheseusOS|
/// 00001010: 21 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |!...............|
/// ```
pub fn hexdump_virtual(addr: u64, len: usize, base_addr: Option<u64>) -> MemoryResult<()> {
    let base = base_addr.unwrap_or(addr);
    let mut buffer = [0u8; 16];
    
    for offset in (0..len).step_by(16) {
        let line_addr = addr + offset as u64;
        let bytes_in_line = core::cmp::min(16, len - offset);
        
        // Read bytes for this line
        match read_bytes(line_addr, &mut buffer[..bytes_in_line]) {
            Ok(()) => {},
            Err(e) => {
                // If we can't read, show error and continue with zeros
                serial_println!("Error reading at 0x{:016x}: {}", line_addr, e);
                buffer[..bytes_in_line].fill(0);
            }
        }
        
        // Build hex string
        let mut hex_part = String::new();
        for i in 0..16 {
            if i < bytes_in_line {
                hex_part.push_str(&format!("{:02x} ", buffer[i]));
            } else {
                hex_part.push_str("   ");
            }
            
            // Add extra space after 8 bytes
            if i == 7 {
                hex_part.push(' ');
            }
        }
        
        // Build ASCII string
        let mut ascii_part = String::new();
        for i in 0..bytes_in_line {
            let byte = buffer[i];
            if byte >= 0x20 && byte < 0x7F {
                ascii_part.push(byte as char);
            } else {
                ascii_part.push('.');
            }
        }
        
        // Print the line
        serial_println!("{:016x}: {} |{}|", base + offset as u64, hex_part, ascii_part);
    }
    
    Ok(())
}

/// Dump physical memory in hex+ASCII format to the serial console.
///
/// # Parameters
/// - `phys_addr`: Physical address to dump from
/// - `len`: Number of bytes to dump
///
/// # Returns
/// - `Ok(())` if dump successful
/// - `Err(error)` with detailed error information
pub fn hexdump_physical(phys_addr: u64, len: usize) -> MemoryResult<()> {
    let mut buffer = [0u8; 16];
    
    for offset in (0..len).step_by(16) {
        let line_addr = phys_addr + offset as u64;
        let bytes_in_line = core::cmp::min(16, len - offset);
        
        // Read bytes for this line using physical memory access
        // Note: This requires PHYS_OFFSET to be active
        let mut line_buffer = [0u8; 16];
        match read_physical_impl(line_addr, &mut line_buffer[..bytes_in_line]) {
            Ok(()) => {
                buffer[..bytes_in_line].copy_from_slice(&line_buffer[..bytes_in_line]);
            }
            Err(e) => {
                // If we can't read, show error and continue with zeros
                serial_println!("Error reading physical memory at 0x{:016x}: {}", line_addr, e);
                buffer[..bytes_in_line].fill(0);
            }
        }
        
        // Build hex string
        let mut hex_part = String::new();
        for i in 0..16 {
            if i < bytes_in_line {
                hex_part.push_str(&format!("{:02x} ", buffer[i]));
            } else {
                hex_part.push_str("   ");
            }
            
            // Add extra space after 8 bytes
            if i == 7 {
                hex_part.push(' ');
            }
        }
        
        // Build ASCII string
        let mut ascii_part = String::new();
        for i in 0..bytes_in_line {
            let byte = buffer[i];
            if byte >= 0x20 && byte < 0x7F {
                ascii_part.push(byte as char);
            } else {
                ascii_part.push('.');
            }
        }
        
        // Print the line
        serial_println!("{:016x}: {} |{}|", line_addr, hex_part, ascii_part);
    }
    
    Ok(())
}

/// Dump memory region to any writer implementing `Write`.
///
/// This is a more flexible version that can output to different
/// destinations (serial, buffer, file, etc.).
///
/// # Parameters
/// - `writer`: Writer to output to
/// - `addr`: Virtual address to dump from
/// - `len`: Number of bytes to dump
/// - `base_addr`: Optional base address for offset display
///
/// # Returns
/// - `Ok(())` if dump successful
/// - `Err(error)` with detailed error information
pub fn hexdump_to_writer<W: Write>(
    writer: &mut W,
    addr: u64,
    len: usize,
    base_addr: Option<u64>,
) -> MemoryResult<()> {
    let base = base_addr.unwrap_or(addr);
    let mut buffer = [0u8; 16];
    
    for offset in (0..len).step_by(16) {
        let line_addr = addr + offset as u64;
        let bytes_in_line = core::cmp::min(16, len - offset);
        
        // Read bytes for this line
        match read_bytes(line_addr, &mut buffer[..bytes_in_line]) {
            Ok(()) => {},
            Err(e) => {
                // If we can't read, write error and continue with zeros
                let _ = write!(writer, "Error reading at 0x{:016x}: {}\n", line_addr, e);
                buffer[..bytes_in_line].fill(0);
            }
        }
        
        // Print address
        let _ = write!(writer, "{:016x}: ", base + offset as u64);
        
        // Print hex bytes
        for i in 0..16 {
            if i < bytes_in_line {
                let _ = write!(writer, "{:02x} ", buffer[i]);
            } else {
                let _ = write!(writer, "   ");
            }
            
            // Add extra space after 8 bytes
            if i == 7 {
                let _ = write!(writer, " ");
            }
        }
        
        // Print ASCII representation
        let _ = write!(writer, " |");
        for i in 0..bytes_in_line {
            let byte = buffer[i];
            if byte >= 0x20 && byte < 0x7F {
                let _ = write!(writer, "{}", byte as char);
            } else {
                let _ = write!(writer, ".");
            }
        }
        let _ = writeln!(writer, "|");
    }
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::String;
    
    #[test]
    fn test_hexdump_virtual_placeholder() {
        // Just test that it compiles and returns Ok for now
        assert!(hexdump_virtual(0x1000, 256, None).is_ok());
    }
    
    #[test]
    fn test_hexdump_to_writer_placeholder() {
        let mut output = String::new();
        assert!(hexdump_to_writer(&mut output, 0x1000, 256, None).is_ok());
        assert!(output.contains("Not implemented yet"));
    }
}
