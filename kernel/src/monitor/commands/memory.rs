//! Memory operation commands
//!
//! This module implements commands for examining and modifying memory:
//! - `mem`: Examine memory at an address (16 bytes, with continuation)
//! - `dump`: Dump a larger memory region
//! - `write`: Write a single byte to memory
//! - `fill`: Fill a memory region with a value

use crate::monitor::Monitor;
use crate::monitor::parsing::parse_number;
use alloc::format;

impl Monitor {
    /// Examine memory at specified address
    ///
    /// Shows 16 bytes in hex + ASCII format. If no address is provided,
    /// continues from the last examined address (Wozmon-style continuation).
    ///
    /// # Arguments
    /// * `args` - Command arguments (optional address)
    ///
    /// # Examples
    /// ```text
    /// mem 0x1000        # Examine 16 bytes at 0x1000
    /// mem               # Continue from last address + 16
    /// ```
    ///
    /// # Safety
    /// Uses volatile reads to access arbitrary memory addresses. The caller
    /// must ensure the address is valid and accessible.
    pub(in crate::monitor) fn cmd_memory(&mut self, args: &[&str]) {
        if args.is_empty() {
            // Continue from last address
            if self.last_addr == 0 {
                self.writeln("Usage: mem ADDR");
                self.writeln("  ADDR can be hex (0x...) or decimal");
                return;
            }
        } else {
            // Parse new address
            if let Some(addr) = parse_number(args[0]) {
                self.last_addr = addr;
            } else {
                self.writeln("Invalid address");
                return;
            }
        }

        // Display 16 bytes in hex + ASCII
        unsafe {
            let ptr = self.last_addr as *const u8;
            self.write(&format!("{:016X}: ", self.last_addr));

            // Hex values
            for i in 0..16 {
                let byte = core::ptr::read_volatile(ptr.add(i));
                self.write(&format!("{:02X} ", byte));
            }

            // ASCII representation
            self.write(" |");
            for i in 0..16 {
                let byte = core::ptr::read_volatile(ptr.add(i));
                if byte >= 0x20 && byte < 0x7F {
                    self.write(&format!("{}", byte as char));
                } else {
                    self.write(".");
                }
            }
            self.writeln("|");
        }

        self.last_addr += 16;
    }

    /// Dump memory region
    ///
    /// Displays a larger memory region in hex + ASCII format with multiple lines.
    ///
    /// # Arguments
    /// * `args` - Command arguments: ADDRESS [LENGTH]
    ///   - ADDRESS: Starting address (hex or decimal)
    ///   - LENGTH: Number of bytes to dump (default: 256)
    ///
    /// # Examples
    /// ```text
    /// dump 0x1000           # Dump 256 bytes starting at 0x1000
    /// dump 0x1000 1024      # Dump 1024 bytes
    /// ```
    pub(in crate::monitor) fn cmd_dump(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: dump ADDR [LENGTH]");
            self.writeln("  ADDR   - Starting address (hex or decimal)");
            self.writeln("  LENGTH - Number of bytes to dump (default: 256)");
            return;
        }

        let addr = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid address");
                return;
            }
        };

        let len = if args.len() > 1 {
            parse_number(args[1]).unwrap_or(256)
        } else {
            256
        };

        self.writeln(&format!("Memory dump at 0x{:X}, {} bytes:", addr, len));

        unsafe {
            for offset in (0..len).step_by(16) {
                let line_addr = addr + offset;
                let ptr = line_addr as *const u8;

                self.write(&format!("{:016X}: ", line_addr));

                // Hex values
                for i in 0..16usize {
                    if offset + (i as u64) < len {
                        let byte = core::ptr::read_volatile(ptr.add(i));
                        self.write(&format!("{:02X} ", byte));
                    } else {
                        self.write("   ");
                    }
                }

                // ASCII representation
                self.write(" |");
                for i in 0..16usize {
                    if offset + (i as u64) < len {
                        let byte = core::ptr::read_volatile(ptr.add(i));
                        if byte >= 0x20 && byte < 0x7F {
                            self.write(&format!("{}", byte as char));
                        } else {
                            self.write(".");
                        }
                    }
                }
                self.writeln("|");
            }
        }
    }

    /// Write a byte to memory
    ///
    /// Writes a single byte value to the specified memory address.
    ///
    /// # Arguments
    /// * `args` - Command arguments: ADDRESS VALUE
    ///   - ADDRESS: Memory address to write to
    ///   - VALUE: Byte value (0-255, hex or decimal)
    ///
    /// # Examples
    /// ```text
    /// write 0x1000 0xFF     # Write 0xFF to address 0x1000
    /// write 0x1000 42       # Write 42 (decimal) to address 0x1000
    /// ```
    ///
    /// # Safety
    /// Uses volatile write to arbitrary memory. May crash if address is invalid
    /// or protected. Use with caution!
    pub(in crate::monitor) fn cmd_write(&self, args: &[&str]) {
        if args.len() < 2 {
            self.writeln("Usage: write ADDR VALUE");
            self.writeln("  ADDR  - Memory address (hex or decimal)");
            self.writeln("  VALUE - Byte value to write (hex or decimal)");
            return;
        }

        let addr = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid address");
                return;
            }
        };

        let value = match parse_number(args[1]) {
            Some(v) => (v & 0xFF) as u8,
            None => {
                self.writeln("Invalid value");
                return;
            }
        };

        unsafe {
            core::ptr::write_volatile(addr as *mut u8, value);
            self.writeln(&format!("Wrote 0x{:02X} to 0x{:016X}", value, addr));
        }
    }

    /// Fill memory region with a value
    ///
    /// Fills a contiguous memory region with a specified byte value.
    ///
    /// # Arguments
    /// * `args` - Command arguments: ADDRESS LENGTH VALUE
    ///   - ADDRESS: Starting address
    ///   - LENGTH: Number of bytes to fill
    ///   - VALUE: Byte value to write (0-255)
    ///
    /// # Examples
    /// ```text
    /// fill 0x1000 256 0x00  # Zero 256 bytes starting at 0x1000
    /// fill 0x2000 16 0xCC   # Fill 16 bytes with 0xCC pattern
    /// ```
    ///
    /// # Safety
    /// Performs multiple volatile writes to arbitrary memory. May crash or
    /// corrupt data if the range overlaps with kernel structures.
    pub(in crate::monitor) fn cmd_fill(&self, args: &[&str]) {
        if args.len() < 3 {
            self.writeln("Usage: fill ADDR LENGTH VALUE");
            self.writeln("  ADDR   - Starting address");
            self.writeln("  LENGTH - Number of bytes to fill");
            self.writeln("  VALUE  - Byte value to write");
            return;
        }

        let addr = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid address");
                return;
            }
        };

        let len = match parse_number(args[1]) {
            Some(l) => l as usize,
            None => {
                self.writeln("Invalid length");
                return;
            }
        };

        let value = match parse_number(args[2]) {
            Some(v) => (v & 0xFF) as u8,
            None => {
                self.writeln("Invalid value");
                return;
            }
        };

        unsafe {
            let ptr = addr as *mut u8;
            for i in 0..len {
                core::ptr::write_volatile(ptr.add(i), value);
            }
            self.writeln(&format!(
                "Filled {} bytes at 0x{:016X} with 0x{:02X}",
                len, addr, value
            ));
        }
    }
}

