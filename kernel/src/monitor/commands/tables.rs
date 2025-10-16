//! Table inspection commands (IDT, GDT, memory map)
//!
//! This module implements commands for inspecting system tables:
//! - `idt`: Display Interrupt Descriptor Table
//! - `gdt`: Display Global Descriptor Table
//! - `mmap`: Display UEFI memory map

use crate::monitor::Monitor;
use crate::monitor::parsing::parse_number;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::cmp::min;

const EFI_PAGE_SIZE: u64 = 4096;

impl Monitor {
    /// Display IDT information
    ///
    /// Shows entries from the Interrupt Descriptor Table (IDT), including:
    /// - IDTR base and limit
    /// - Entry count
    /// - Individual IDT entries with handler addresses, selectors, and attributes
    ///
    /// # Arguments
    /// * `args` - Optional vector number to limit display (default: first 16)
    ///
    /// # Examples
    /// ```text
    /// idt               # Show first 16 IDT entries
    /// idt 32            # Show first 32 entries
    /// ```
    pub(in crate::monitor) fn cmd_idt(&self, args: &[&str]) {
        self.writeln("Interrupt Descriptor Table:");

        use x86_64::instructions::tables::sidt;
        let idtr = sidt();
        let base = idtr.base.as_u64();
        let total_bytes = (idtr.limit as usize) + 1;
        let entry_size = core::mem::size_of::<RawIdtEntry>();
        let entry_count = total_bytes / entry_size;

        self.writeln(&format!("  IDTR Base:  0x{:016X}", base));
        self.writeln(&format!(
            "  IDTR Limit: 0x{:04X} ({} bytes, {} entries)",
            idtr.limit, total_bytes, entry_count
        ));

        if entry_count == 0 {
            self.writeln("  (IDT empty)");
            return;
        }

        let max_entries = args
            .get(0)
            .and_then(|s| parse_number(s))
            .map(|n| n as usize)
            .unwrap_or(16);

        let ptr = base as *const RawIdtEntry;
        let count = min(entry_count, max_entries);

        for idx in 0..count {
            let entry = unsafe { core::ptr::read_unaligned(ptr.add(idx)) };
            self.print_idt_entry(idx, &entry);
        }

        if entry_count > count {
            self.writeln(&format!("  ({} entries not shown)", entry_count - count));
        }
    }

    /// Print a single IDT entry with decoded fields
    ///
    /// Extracts and displays:
    /// - Handler offset (64-bit address)
    /// - Segment selector
    /// - IST (Interrupt Stack Table) index
    /// - Gate type (Interrupt Gate, Trap Gate, etc.)
    /// - DPL (Descriptor Privilege Level)
    /// - Present bit
    fn print_idt_entry(&self, index: usize, entry: &RawIdtEntry) {
        let entry = *entry;
        let empty = entry.offset_low == 0
            && entry.offset_mid == 0
            && entry.offset_high == 0
            && entry.type_attr == 0
            && entry.selector == 0;
        if empty {
            self.writeln(&format!("  [{:02}] <empty>", index));
            return;
        }

        let selector = entry.selector;
        let type_attr = entry.type_attr;
        let ist = entry.ist & 0x7;  // IST index is only 3 bits (0-7)
        
        // Reconstruct 64-bit handler offset from three 16/32-bit pieces
        let offset = (entry.offset_low as u64)          // Bits 0-15
            | ((entry.offset_mid as u64) << 16)         // Bits 16-31
            | ((entry.offset_high as u64) << 32);       // Bits 32-63
        
        let gate_type = type_attr & 0x0F;               // Type field (bits 0-3)
        let present = (type_attr & 0x80) != 0;          // Present bit (bit 7)
        let dpl = (type_attr >> 5) & 0x3;               // DPL (bits 5-6)

        self.writeln(&format!(
            "  [{:02}] selector=0x{:04X} offset=0x{:016X} type:{} dpl:{} ist:{} attr=0x{:02X} {}",
            index,
            selector,
            offset,
            describe_idt_gate(gate_type),
            dpl,
            ist,
            type_attr,
            if present { "present" } else { "absent" }
        ));
    }

    /// Display GDT information
    ///
    /// Shows entries from the Global Descriptor Table (GDT), including:
    /// - GDTR base and limit
    /// - Individual GDT entries with base, limit, type, and flags
    /// - Support for both 8-byte and 16-byte descriptors (TSS, LDT)
    ///
    /// # Arguments
    /// * `args` - Optional entry count to display (default: all entries)
    ///
    /// # Examples
    /// ```text
    /// gdt               # Show all GDT entries
    /// gdt 8             # Show first 8 entries
    /// ```
    pub(in crate::monitor) fn cmd_gdt(&self, args: &[&str]) {
        self.writeln("Global Descriptor Table:");

        use x86_64::instructions::tables::sgdt;
        let gdtr = sgdt();
        let base = gdtr.base.as_u64();
        let total_bytes = (gdtr.limit as usize) + 1;
        let mut entry_count = total_bytes / 8;
        if total_bytes % 8 != 0 {
            entry_count += 1;
        }

        self.writeln(&format!("  GDTR Base:  0x{:016X}", base));
        self.writeln(&format!(
            "  GDTR Limit: 0x{:04X} ({} bytes, {} entries)",
            gdtr.limit, total_bytes, entry_count
        ));

        if entry_count == 0 {
            self.writeln("  (GDT empty)");
            return;
        }

        let max_entries = args
            .get(0)
            .and_then(|s| parse_number(s))
            .map(|n| n as usize)
            .unwrap_or(entry_count);

        let ptr = base as *const u8;
        let mut idx = 0usize;
        let mut shown = 0usize;

        while idx < entry_count && shown < max_entries {
            let entry_ptr = unsafe { ptr.add(idx * 8) } as *const u64;
            let raw_low = unsafe { core::ptr::read_unaligned(entry_ptr) };
            let next = if idx + 1 < entry_count {
                Some(unsafe { core::ptr::read_unaligned(entry_ptr.add(1)) })
            } else {
                None
            };
            let consumed = self.print_gdt_entry(idx, raw_low, next);
            shown += 1;
            idx += if consumed { 2 } else { 1 };
        }

        if idx < entry_count {
            self.writeln(&format!("  ({} descriptors not shown)", entry_count - idx));
        }
    }

    /// Print a single GDT entry with decoded fields
    ///
    /// Decodes an x86-64 segment descriptor (8 or 16 bytes) and displays:
    /// - Base address (24-bit for code/data, 64-bit for system descriptors)
    /// - Limit (20-bit, possibly page-granular)
    /// - Segment type (Code, Data, TSS, LDT, etc.)
    /// - DPL (Descriptor Privilege Level: 0-3)
    /// - Flags (G=Granularity, L=Long mode, DB=Default size, AVL=Available)
    ///
    /// # Returns
    /// * `true` - Entry consumed 16 bytes (system descriptor)
    /// * `false` - Entry consumed 8 bytes (code/data segment)
    fn print_gdt_entry(&self, index: usize, raw_low: u64, raw_next: Option<u64>) -> bool {
        let uses_second = gdt_entry_needs_extra(raw_low);
        let raw_high = if uses_second { raw_next } else { None };

        if raw_low == 0 && raw_high.unwrap_or(0) == 0 {
            self.writeln(&format!("  [{:02}] <null>", index));
            return uses_second;
        }

        // Decode access byte (bits 40-47)
        let access = ((raw_low >> 40) & 0xFF) as u8;
        let s = (access & 0x10) != 0;        // S bit: 1=code/data, 0=system
        let typ = access & 0x0F;             // Type field (bits 0-3)
        let dpl = (access >> 5) & 0x03;      // Descriptor Privilege Level
        let present = (access & 0x80) != 0;  // Present bit

        // Decode limit (20-bit: bits 0-15 and 48-51)
        let limit_low = (raw_low & 0xFFFF) as u32;
        let limit_high = ((raw_low >> 48) & 0xF) as u32;
        let limit = (limit_low | (limit_high << 16)) as u32;

        // Decode base address (32-bit in low quadword: bits 16-39 and 56-63)
        let base_low = ((raw_low >> 16) & 0xFFFFFF) as u32;
        let base_high = ((raw_low >> 56) & 0xFF) as u32;
        let mut base = ((base_high as u64) << 24) | base_low as u64;

        // Decode flags (bits 52-55)
        let flags = ((raw_low >> 52) & 0xF) as u8;
        let avl = (flags & 0x1) != 0;  // Available for system use
        let l = (flags & 0x2) != 0;    // Long mode (64-bit code segment)
        let db = (flags & 0x4) != 0;   // Default operation size (0=16bit, 1=32bit)
        let g = (flags & 0x8) != 0;    // Granularity (0=byte, 1=4KiB pages)

        // For system descriptors (TSS, LDT), use upper 64 bits for extended base
        if let Some(high) = raw_high {
            base |= ((high & 0xFFFF_FFFF) as u64) << 32;
        }

        // Calculate actual limit in bytes
        // If granularity bit set, limit is in 4KiB pages
        let limit_bytes = if g {
            ((limit as u64) << 12) | 0xFFF  // Multiply by 4096 and add 4095
        } else {
            limit as u64
        };

        let mut flag_parts = Vec::new();
        if g {
            flag_parts.push("G");
        }
        if l {
            flag_parts.push("L");
        }
        if db {
            flag_parts.push("DB");
        }
        if avl {
            flag_parts.push("AVL");
        }

        let type_desc = describe_segment_type(s, typ);

        self.writeln(&format!(
            "  [{:02}] base=0x{:016X} limit=0x{:016X} type:{} dpl:{} attr=0x{:02X} {} flags:{}",
            index,
            base,
            limit_bytes,
            type_desc,
            dpl,
            access,
            if present { "present" } else { "not-present" },
            if flag_parts.is_empty() {
                "-".into()
            } else {
                flag_parts.join("|")
            }
        ));

        uses_second
    }

    /// Display UEFI memory map
    ///
    /// Shows the UEFI firmware memory map with all memory regions reported
    /// during boot. Supports multiple display modes:
    /// - Summary + sample (default): Statistics and first 8 entries
    /// - Summary only: Just per-type totals
    /// - All entries: Complete listing
    /// - Single entry: Detailed view of one descriptor
    ///
    /// # Arguments
    /// * `args` - Display mode and parameters:
    ///   - No args: Summary + first 8 entries
    ///   - `summary`: Show only per-type totals
    ///   - `entries [N]`: Show all (or N) entries
    ///   - `entry INDEX`: Show single entry
    ///   - `NUMBER`: Show single entry by index
    ///
    /// # Examples
    /// ```text
    /// mmap              # Summary + first 8 entries
    /// mmap summary      # Only type summary
    /// mmap entries      # Show all entries
    /// mmap entries 20   # Show first 20 entries
    /// mmap entry 5      # Show entry #5 in detail
    /// mmap 5            # Same as above
    /// ```
    pub(in crate::monitor) fn cmd_memory_map(&self, args: &[&str]) {
        self.writeln("UEFI Memory Map:");

        let handoff = unsafe {
            &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff)
        };

        let desc_size = handoff.memory_map_descriptor_size as usize;
        let entry_count = handoff.memory_map_entries as usize;
        let buffer_ptr = handoff.memory_map_buffer_ptr;

        if buffer_ptr == 0 || entry_count == 0 || desc_size == 0 {
            self.writeln("  (memory map not available in handoff)");
            return;
        }

        if desc_size < 32 {
            self.writeln(&format!(
                "  Descriptor size {} is unexpectedly small (< 32)",
                desc_size
            ));
            return;
        }

        let buffer = buffer_ptr as *const u8;
        let mut total_pages: u128 = 0;
        let mut type_totals: Vec<TypeSummary> = Vec::new();
        let mut descriptors: Vec<UefiMemoryDescriptor> = Vec::with_capacity(entry_count);

        // Parse all descriptors and build summary statistics
        for idx in 0..entry_count {
            let desc = unsafe { read_uefi_descriptor(buffer, desc_size, idx) };
            total_pages = total_pages.wrapping_add(desc.num_pages as u128);

            // Accumulate per-type totals
            if let Some(entry) = type_totals.iter_mut().find(|e| e.typ == desc.typ) {
                entry.entries += 1;
                entry.pages = entry.pages.wrapping_add(desc.num_pages);
            } else {
                type_totals.push(TypeSummary {
                    typ: desc.typ,
                    entries: 1,
                    pages: desc.num_pages,
                });
            }

            descriptors.push(desc);
        }

        // Sort types by page count (largest first) for summary display
        type_totals.sort_by(|a, b| b.pages.cmp(&a.pages));

        let total_bytes = total_pages.saturating_mul(EFI_PAGE_SIZE as u128);

        // Determine display mode based on arguments
        enum MemoryMapMode {
            SummaryOnly,                      // Just per-type totals
            SummaryAndSample { sample: usize }, // Summary + first N entries
            Entries { limit: Option<usize> }, // All entries (or limited)
            Single { index: usize },          // Single entry detail
        }

        let mode = if let Some(first) = args.get(0) {
            match *first {
                "summary" => MemoryMapMode::SummaryOnly,
                "entries" | "all" => {
                    let limit = args
                        .get(1)
                        .and_then(|s| parse_number(s))
                        .map(|n| n as usize);
                    MemoryMapMode::Entries { limit }
                }
                "entry" => {
                    if args.len() < 2 {
                        self.writeln("Usage: mmap entry INDEX");
                        return;
                    }
                    match parse_number(args[1]) {
                        Some(idx) => MemoryMapMode::Single {
                            index: idx as usize,
                        },
                        None => {
                            self.writeln("Invalid index");
                            return;
                        }
                    }
                }
                other => match parse_number(other) {
                    Some(idx) => MemoryMapMode::Single {
                        index: idx as usize,
                    },
                    None => {
                        self.writeln("Usage: mmap [summary | entries [N] | entry INDEX]");
                        return;
                    }
                },
            }
        } else {
            MemoryMapMode::SummaryAndSample { sample: 8 }
        };

        self.writeln(&format!(
            "  Entries: {}  Descriptor Size: {} bytes",
            entry_count, desc_size
        ));
        self.writeln(&format!("  Total reported: {}", format_bytes(total_bytes)));
        self.writeln("  Per-type summary:");
        for summary in &type_totals {
            let bytes = (summary.pages as u128).saturating_mul(EFI_PAGE_SIZE as u128);
            self.writeln(&format!(
                "    {:>2} {:<20} entries:{:>2} pages:{:>8} size:{}",
                summary.typ,
                uefi_memory_type_to_str(summary.typ),
                summary.entries,
                summary.pages,
                format_bytes(bytes)
            ));
        }

        match mode {
            MemoryMapMode::SummaryOnly => {}
            MemoryMapMode::SummaryAndSample { sample } => {
                self.writeln("");
                let count = min(sample, descriptors.len());
                self.writeln(&format!("  First {} descriptors:", count));
                for (idx, desc) in descriptors.iter().take(count).enumerate() {
                    self.print_memory_descriptor(idx, desc);
                }
                if descriptors.len() > count {
                    self.writeln(&format!(
                        "  ({} additional descriptors hidden; use 'mmap entries' to show all)",
                        descriptors.len() - count
                    ));
                }
            }
            MemoryMapMode::Entries { limit } => {
                self.writeln("");
                let max = limit.unwrap_or(descriptors.len());
                let count = min(max, descriptors.len());
                self.writeln(&format!("  Listing {} descriptors:", count));
                for (idx, desc) in descriptors.iter().take(count).enumerate() {
                    self.print_memory_descriptor(idx, desc);
                }
                if descriptors.len() > count {
                    self.writeln(&format!(
                        "  ({} additional descriptors not shown)",
                        descriptors.len() - count
                    ));
                }
            }
            MemoryMapMode::Single { index } => {
                self.writeln("");
                if let Some(desc) = descriptors.get(index) {
                    self.writeln(&format!("  Descriptor {}:", index));
                    self.print_memory_descriptor(index, desc);
                } else {
                    self.writeln("  Descriptor index out of range");
                }
            }
        }
    }

    /// Print a single UEFI memory descriptor with all details
    ///
    /// Displays physical/virtual range, page count, memory type, size, and attributes.
    fn print_memory_descriptor(&self, index: usize, desc: &UefiMemoryDescriptor) {
        let size_bytes = (desc.num_pages as u128).saturating_mul(EFI_PAGE_SIZE as u128);
        let phys_end = if desc.num_pages == 0 {
            desc.phys_start
        } else {
            desc.phys_start
                .saturating_add(desc.num_pages.saturating_mul(EFI_PAGE_SIZE) - 1)
        };
        let attrs = describe_memory_attributes(desc.attributes);
        self.writeln(&format!(
            "  [{:03}] {:<20} phys:0x{:016X}-0x{:016X} pages:{:>6} size:{} attrs:{} (0x{:016X})",
            index,
            uefi_memory_type_to_str(desc.typ),
            desc.phys_start,
            phys_end,
            desc.num_pages,
            format_bytes(size_bytes),
            attrs,
            desc.attributes
        ));
        if desc.virt_start != 0 {
            self.writeln(&format!("        virt:0x{:016X}", desc.virt_start));
        }
    }
}

// Helper structures and functions

/// Raw IDT entry structure (16 bytes)
///
/// Matches the x86-64 interrupt gate descriptor format.
/// Layout:
/// - Bytes 0-1: offset_low (bits 0-15 of handler address)
/// - Bytes 2-3: selector (code segment selector)
/// - Byte 4: ist (Interrupt Stack Table index, bits 0-2)
/// - Byte 5: type_attr (gate type and attributes)
/// - Bytes 6-7: offset_mid (bits 16-31 of handler address)
/// - Bytes 8-11: offset_high (bits 32-63 of handler address)
/// - Bytes 12-15: zero (reserved, must be 0)
#[repr(C, packed)]
#[derive(Copy, Clone)]
struct RawIdtEntry {
    offset_low: u16,
    selector: u16,
    ist: u8,
    type_attr: u8,
    offset_mid: u16,
    offset_high: u32,
    zero: u32,
}

/// Describe IDT gate type
///
/// Converts the gate type field (bits 0-3 of type_attr) to a human-readable string.
///
/// # Arguments
/// * `typ` - Gate type value (0-15)
///
/// # Returns
/// * Human-readable gate type description
fn describe_idt_gate(typ: u8) -> &'static str {
    match typ {
        0x5 => "Task Gate",
        0x6 => "16-bit Interrupt",
        0x7 => "16-bit Trap",
        0xE => "Interrupt Gate",
        0xF => "Trap Gate",
        _ => "Reserved",
    }
}

/// Check if a GDT entry uses 16 bytes (system descriptor) vs 8 bytes (code/data)
///
/// System descriptors (TSS, LDT, Call/Interrupt/Trap gates) in 64-bit mode
/// use 16 bytes to accommodate a 64-bit base address.
///
/// # Arguments
/// * `raw_low` - Lower 8 bytes of the descriptor
///
/// # Returns
/// * `true` - Descriptor uses 16 bytes
/// * `false` - Descriptor uses 8 bytes
fn gdt_entry_needs_extra(raw_low: u64) -> bool {
    let access = ((raw_low >> 40) & 0xFF) as u8;
    let is_code_or_data = (access & 0x10) != 0;
    if is_code_or_data {
        false
    } else {
        match access & 0x0F {
            0x2 | 0x9 | 0xB | 0xC | 0xE | 0xF => true,
            _ => false,
        }
    }
}

/// Describe segment type based on S bit and type field
///
/// Segments are either code/data (S=1) or system (S=0), with different
/// type field interpretations for each.
///
/// # Arguments
/// * `is_code_or_data` - S bit value (true = code/data, false = system)
/// * `typ` - Type field (bits 0-3 of access byte)
///
/// # Returns
/// * Human-readable segment type description
fn describe_segment_type(is_code_or_data: bool, typ: u8) -> String {
    if is_code_or_data {
        let is_code = (typ & 0x8) != 0;
        if is_code {
            let readable = (typ & 0x2) != 0;
            let conforming = (typ & 0x4) != 0;
            format!(
                "Code{}{}",
                if readable { "+R" } else { "" },
                if conforming { " (conf)" } else { "" }
            )
        } else {
            let writable = (typ & 0x2) != 0;
            let expand_down = (typ & 0x4) != 0;
            format!(
                "Data{}{}",
                if writable { "+W" } else { "" },
                if expand_down { " (exp-down)" } else { "" }
            )
        }
    } else {
        match typ {
            0x2 => "LDT".into(),
            0x9 => "TSS (available)".into(),
            0xB => "TSS (busy)".into(),
            0xC => "Call Gate".into(),
            0xE => "Interrupt Gate".into(),
            0xF => "Trap Gate".into(),
            _ => "System".into(),
        }
    }
}

/// UEFI memory descriptor (simplified)
///
/// Represents a single region in the UEFI memory map.
/// Each descriptor describes a contiguous region of physical memory
/// with a specific type and attributes.
#[derive(Copy, Clone)]
struct UefiMemoryDescriptor {
    typ: u32,
    phys_start: u64,
    virt_start: u64,
    num_pages: u64,
    attributes: u64,
}

/// Summary of memory regions by type
///
/// Used to aggregate memory map entries by type for summary display.
#[derive(Clone)]
struct TypeSummary {
    typ: u32,
    entries: usize,
    pages: u64,
}

/// Read a UEFI memory descriptor from the memory map buffer
///
/// Safely reads descriptor fields handling variable descriptor sizes.
/// UEFI allows descriptor sizes >= 32 bytes, so we check sizes before reading.
///
/// # Arguments
/// * `buffer` - Pointer to memory map buffer
/// * `desc_size` - Size of each descriptor in bytes
/// * `index` - Index of the descriptor to read
///
/// # Returns
/// * Parsed memory descriptor
///
/// # Safety
/// Caller must ensure buffer is valid and contains at least (index+1) * desc_size bytes.
unsafe fn read_uefi_descriptor(
    buffer: *const u8,
    desc_size: usize,
    index: usize,
) -> UefiMemoryDescriptor {
    let ptr = buffer.add(index * desc_size);
    
    // UEFI memory descriptor layout (minimum 32 bytes, may be larger):
    // Offset 0-3:   Type (u32)
    // Offset 4-7:   Padding
    // Offset 8-15:  PhysicalStart (u64)
    // Offset 16-23: VirtualStart (u64)
    // Offset 24-31: NumberOfPages (u64)
    // Offset 32-39: Attribute (u64)
    
    let typ = core::ptr::read_unaligned(ptr as *const u32);
    let phys_start = if desc_size >= 16 {
        core::ptr::read_unaligned(ptr.add(8) as *const u64)
    } else {
        0
    };
    let virt_start = if desc_size >= 24 {
        core::ptr::read_unaligned(ptr.add(16) as *const u64)
    } else {
        0
    };
    let num_pages = if desc_size >= 32 {
        core::ptr::read_unaligned(ptr.add(24) as *const u64)
    } else {
        0
    };
    let attributes = if desc_size >= 40 {
        core::ptr::read_unaligned(ptr.add(32) as *const u64)
    } else {
        0
    };
    UefiMemoryDescriptor {
        typ,
        phys_start,
        virt_start,
        num_pages,
        attributes,
    }
}

/// Convert UEFI memory type code to human-readable string
///
/// # Arguments
/// * `typ` - UEFI memory type code (0-15, or higher for vendor-specific)
///
/// # Returns
/// * Human-readable memory type name
fn uefi_memory_type_to_str(typ: u32) -> &'static str {
    match typ {
        0 => "Reserved",
        1 => "LoaderCode",
        2 => "LoaderData",
        3 => "BootServicesCode",
        4 => "BootServicesData",
        5 => "RuntimeServicesCode",
        6 => "RuntimeServicesData",
        7 => "ConventionalMemory",
        8 => "UnusableMemory",
        9 => "ACPIReclaim",
        10 => "ACPINVS",
        11 => "MMIO",
        12 => "MMIOPort",
        13 => "PalCode",
        14 => "PersistentMemory",
        15 => "Unaccepted",
        _ => "Unknown",
    }
}

/// Describe UEFI memory attributes as a pipe-separated flag string
///
/// Decodes the attribute bits into readable cache/access flags:
/// - UC: Uncacheable
/// - WC: Write Combining
/// - WT: Write Through
/// - WP: Write Protected
/// - WB: Write Back
/// - UCE: Uncacheable Exported
/// - XP: Execute Protected (no-execute)
/// - RT: Runtime (used by firmware runtime services)
///
/// # Arguments
/// * `attrs` - 64-bit attribute bitfield from UEFI memory descriptor
///
/// # Returns
/// * Pipe-separated string of active flags (e.g., "WB|RT")
fn describe_memory_attributes(attrs: u64) -> String {
    const EFI_MEMORY_UC: u64 = 0x0000_0000_0000_0001;
    const EFI_MEMORY_WC: u64 = 0x0000_0000_0000_0002;
    const EFI_MEMORY_WT: u64 = 0x0000_0000_0000_0004;
    const EFI_MEMORY_WP: u64 = 0x0000_0000_0000_0008;
    const EFI_MEMORY_WB: u64 = 0x0000_0000_0000_0010;
    const EFI_MEMORY_UCE: u64 = 0x0000_0000_0000_0020;
    const EFI_MEMORY_RUNTIME: u64 = 0x8000_0000_0000_0000;
    const EFI_MEMORY_XP: u64 = 0x1000_0000_0000_0000;

    let mut flags = Vec::new();
    if attrs & EFI_MEMORY_UC != 0 {
        flags.push("UC");
    }
    if attrs & EFI_MEMORY_WC != 0 {
        flags.push("WC");
    }
    if attrs & EFI_MEMORY_WT != 0 {
        flags.push("WT");
    }
    if attrs & EFI_MEMORY_WP != 0 {
        flags.push("WP");
    }
    if attrs & EFI_MEMORY_WB != 0 {
        flags.push("WB");
    }
    if attrs & EFI_MEMORY_UCE != 0 {
        flags.push("UCE");
    }
    if attrs & EFI_MEMORY_XP != 0 {
        flags.push("XP");
    }
    if attrs & EFI_MEMORY_RUNTIME != 0 {
        flags.push("RT");
    }

    if flags.is_empty() {
        "-".into()
    } else {
        flags.join("|")
    }
}

/// Format byte count in human-readable units
///
/// Converts a byte count to appropriate units (B, KiB, MiB, GiB, TiB, PiB)
/// using binary (1024) scaling.
///
/// # Arguments
/// * `value` - Number of bytes to format
///
/// # Returns
/// * Formatted string with both exact bytes and scaled value
/// * Example: "268435456 bytes (~256 MiB)"
fn format_bytes(value: u128) -> String {
    const UNITS: [&str; 6] = ["B", "KiB", "MiB", "GiB", "TiB", "PiB"];
    let mut scaled = value;
    let mut unit = 0usize;
    while scaled >= 1024 && unit < UNITS.len() - 1 {
        scaled /= 1024;
        unit += 1;
    }
    if unit == 0 {
        format!("{} {}", scaled, UNITS[unit])
    } else {
        format!("{} bytes (~{} {})", value, scaled, UNITS[unit])
    }
}
