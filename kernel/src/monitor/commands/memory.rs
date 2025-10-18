//! Memory operation commands
//!
//! This module implements commands for examining and modifying memory:
//! - `mem`: Examine memory at an address (16 bytes, with continuation)
//! - `dump`: Dump a larger memory region
//! - `write`: Write a single byte to memory
//! - `fill`: Fill a memory region with a value

use crate::memory::{
    self, PageTable, PAGE_SIZE, PTE_ACCESSED, PTE_DIRTY, PTE_GLOBAL, PTE_NO_EXEC, PTE_PCD,
    PTE_PRESENT, PTE_PS, PTE_PWT, PTE_USER, PTE_WRITABLE,
};
use crate::monitor::parsing::parse_number;
use crate::monitor::Monitor;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

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

    /// Walk the current page tables and display each level involved in translating `va`.
    ///
    /// # Arguments
    /// * `args` - Command arguments: VIRTUAL_ADDRESS
    ///
    /// # Examples
    /// ```text
    /// ptwalk 0xFFFFFFFF80000000
    /// pt 0x7FFFF000000
    /// ```
    pub(in crate::monitor) fn cmd_ptwalk(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: ptwalk|pt VIRT_ADDR");
            self.writeln("  VIRT_ADDR - Virtual address to translate");
            return;
        }

        let va = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid virtual address");
                return;
            }
        };

        let (frame, _) = x86_64::registers::control::Cr3::read();
        let mut table_phys = frame.start_address().as_u64();
        let indices = [
            ((va >> 39) & 0x1FF) as usize,
            ((va >> 30) & 0x1FF) as usize,
            ((va >> 21) & 0x1FF) as usize,
            ((va >> 12) & 0x1FF) as usize,
        ];

        self.writeln(&format!(
            "Page table walk for VA 0x{:016X} (CR3=0x{:016X}):",
            va, table_phys
        ));

        let mut final_pa: Option<u64> = None;
        let mut page_size: Option<u64> = None;

        for (depth, &index) in indices.iter().enumerate() {
            let level = 4 - depth;
            let table = match self.page_table_from_phys(table_phys) {
                Some(t) => t,
                None => {
                    self.writeln(&format!(
                        "  L{} table at 0x{:016X} is not accessible (PHYS_OFFSET inactive?)",
                        level, table_phys
                    ));
                    return;
                }
            };

            let entry = table.entries[index];
            let value = entry.0;
            let flags = Self::entry_flags_to_string(value);
            let next_phys = entry.physical_addr();

            self.writeln(&format!(
                "  L{}[{:03X}] = 0x{:016X} [{}]",
                level, index, value, flags
            ));

            if value & PTE_PRESENT == 0 {
                self.writeln("    ↳ entry not present");
                return;
            }

            match level {
                4 | 3 => {
                    if level == 3 && (value & PTE_PS) != 0 {
                        // 1 GiB page
                        let offset = va & ((1u64 << 30) - 1);
                        final_pa = Some(next_phys + offset);
                        page_size = Some(1u64 << 30);
                        break;
                    }
                    table_phys = next_phys;
                }
                2 => {
                    if (value & PTE_PS) != 0 {
                        // 2 MiB page
                        let offset = va & ((1u64 << 21) - 1);
                        final_pa = Some(next_phys + offset);
                        page_size = Some(1u64 << 21);
                        break;
                    }
                    table_phys = next_phys;
                }
                1 => {
                    let offset = va & ((PAGE_SIZE as u64) - 1);
                    final_pa = Some(next_phys + offset);
                    page_size = Some(PAGE_SIZE as u64);
                    break;
                }
                _ => {}
            }
        }

        match (final_pa, page_size) {
            (Some(pa), Some(sz)) => {
                self.writeln(&format!(
                    "  ↳ VA 0x{:016X} → PA 0x{:016X} (page size {} KiB)",
                    va,
                    pa,
                    sz / 1024
                ));
            }
            _ => {
                self.writeln("  ↳ translation terminated early");
            }
        }
    }

    /// Dump entries from a specific page-table level to help browse mappings.
    ///
    /// # Arguments
    /// * `args` - `LEVEL [PML4] [PDPT] [PD] [START [COUNT]]`
    ///   - LEVEL: 4 (PML4), 3 (PDPT), 2 (PD), 1 (PT)
    ///   - PML4/PDPT/PD: Indices needed to reach desired level
    ///   - START: Optional starting slot index (default 0)
    ///   - COUNT: Optional number of entries to show (default 16)
    ///
    /// # Examples
    /// ```text
    /// ptdump 4 0 32          # Show 32 PML4 entries
    /// ptdump 3 0x1FF 0 16    # Show 16 PDPTEs under PML4[0x1FF]
    /// ptdump 2 0x1FF 0 0 32  # Dump PDEs under PML4[0x1FF], PDPT[0]
    /// ptdump 1 0x1FF 0 0x40 0x80  # Walk to PT and show entries starting at 0x80
    /// ```
    pub(in crate::monitor) fn cmd_ptdump(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: ptdump LEVEL [PML4] [PDPT] [PD] [START [COUNT]]");
            return;
        }

        let level = match parse_number(args[0]) {
            Some(l) if (1..=4).contains(&l) => l as u8,
            _ => {
                self.writeln("LEVEL must be 1 (PT) through 4 (PML4)");
                return;
            }
        };

        let required_path = match level {
            4 => 0,
            3 => 1,
            2 => 2,
            1 => 3,
            _ => unreachable!(),
        };

        if args.len() < 1 + required_path {
            self.writeln("Not enough indices supplied to reach requested level");
            return;
        }

        let mut path: [usize; 3] = [0; 3];
        for i in 0..required_path {
            path[i] = match parse_number(args[1 + i]) {
                Some(v) if v < 512 => v as usize,
                Some(_) => {
                    self.writeln("Index must be < 512");
                    return;
                }
                None => {
                    self.writeln("Invalid index");
                    return;
                }
            };
        }

        let mut arg_idx = 1 + required_path;
        let start = if args.len() > arg_idx {
            match parse_number(args[arg_idx]) {
                Some(v) if v < 512 => v as usize,
                Some(_) => {
                    self.writeln("START must be < 512");
                    return;
                }
                None => {
                    self.writeln("Invalid START value");
                    return;
                }
            }
        } else {
            0
        };
        if args.len() > arg_idx {
            arg_idx += 1;
        }

        let count = if args.len() > arg_idx {
            match parse_number(args[arg_idx]) {
                Some(v) if v > 0 => core::cmp::min(v as usize, 512 - start),
                Some(_) => {
                    self.writeln("COUNT must be > 0");
                    return;
                }
                None => {
                    self.writeln("Invalid COUNT value");
                    return;
                }
            }
        } else {
            core::cmp::min(16, 512 - start)
        };

        let (frame, _) = x86_64::registers::control::Cr3::read();
        let mut table_phys = frame.start_address().as_u64();
        let mut current_level = 4;

        for depth in 0..required_path {
            let index = path[depth];
            let table = match self.page_table_from_phys(table_phys) {
                Some(t) => t,
                None => {
                    self.writeln(&format!(
                        "Unable to access L{} table at 0x{:016X}",
                        current_level, table_phys
                    ));
                    return;
                }
            };

            let entry = table.entries[index];
            let value = entry.0;

            self.writeln(&format!(
                "  L{}[{:03X}] = 0x{:016X} [{}]",
                current_level,
                index,
                value,
                Self::entry_flags_to_string(value)
            ));

            if value & PTE_PRESENT == 0 {
                self.writeln("    ↳ entry not present");
                return;
            }

            if current_level == 3 && (value & PTE_PS) != 0 && level < 3 {
                self.writeln("    ↳ 1 GiB huge page prevents descending further");
                return;
            }
            if current_level == 2 && (value & PTE_PS) != 0 && level < 2 {
                self.writeln("    ↳ 2 MiB huge page prevents descending further");
                return;
            }

            table_phys = entry.physical_addr();
            current_level -= 1;
        }

        let table = match self.page_table_from_phys(table_phys) {
            Some(t) => t,
            None => {
                self.writeln(&format!(
                    "Unable to access L{} table at 0x{:016X}",
                    level, table_phys
                ));
                return;
            }
        };
        let table_va = table as *const PageTable as u64;

        self.writeln(&format!(
            "Dumping L{} table @ phys 0x{:016X} (virt 0x{:016X}), entries {}..{}",
            level,
            table_phys,
            table_va,
            start,
            start + count - 1
        ));

        for slot in start..(start + count) {
            let entry = table.entries[slot];
            let value = entry.0;
            if value == 0 {
                self.writeln(&format!("  [{:03X}] --", slot));
                continue;
            }

            let flags = Self::entry_flags_to_string(value);
            let phys = entry.physical_addr();

            if (level == 2 && (value & PTE_PS) != 0) || (level == 3 && (value & PTE_PS) != 0) {
                let size = if level == 3 { "1GiB" } else { "2MiB" };
                self.writeln(&format!(
                    "  [{:03X}] phys=0x{:016X} [{}] ({})",
                    slot, phys, flags, size
                ));
            } else {
                self.writeln(&format!(
                    "  [{:03X}] phys=0x{:016X} [{}]",
                    slot, phys, flags
                ));
            }
        }
    }

    fn page_table_from_phys(&self, phys: u64) -> Option<&'static PageTable> {
        if phys == 0 {
            return None;
        }
        let va = if memory::phys_offset_is_active() {
            memory::phys_to_virt_pa(phys)
        } else {
            phys
        };
        Some(unsafe { &*(va as *const PageTable) })
    }

    fn entry_flags_to_string(value: u64) -> String {
        if value == 0 {
            return String::from("empty");
        }

        let mut parts: Vec<&'static str> = Vec::new();
        if value & PTE_PRESENT != 0 {
            parts.push("P");
        } else {
            parts.push("NP");
        }
        if value & PTE_WRITABLE != 0 {
            parts.push("RW");
        }
        if value & PTE_USER != 0 {
            parts.push("US");
        }
        if value & PTE_PWT != 0 {
            parts.push("WT");
        }
        if value & PTE_PCD != 0 {
            parts.push("CD");
        }
        if value & PTE_ACCESSED != 0 {
            parts.push("A");
        }
        if value & PTE_DIRTY != 0 {
            parts.push("D");
        }
        if value & PTE_GLOBAL != 0 {
            parts.push("G");
        }
        if value & PTE_PS != 0 {
            parts.push("PS");
        }
        if value & PTE_NO_EXEC != 0 {
            parts.push("NX");
        }

        parts.join("|")
    }
}
