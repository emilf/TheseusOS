//! Debugging utilities for interrupt subsystem
//!
//! This module provides functions for debugging and diagnosing interrupt-related issues:
//! - IDT (Interrupt Descriptor Table) inspection
//! - GDT (Global Descriptor Table) inspection
//! - Low-level QEMU debug port output (0xE9)
//!
//! ## QEMU Debug Port (0xE9)
//!
//! Port 0xE9 is a special QEMU feature that writes bytes directly to stdout.
//! This is useful for early boot debugging before serial drivers are initialized.
//!
//! Enable in QEMU with: `-debugcon file:debug.log -global isa-debugcon.iobase=0xE9`

use super::{IdtEntry, APIC_TIMER_VECTOR};
use x86_64::instructions::port::Port;
use x86_64::instructions::tables::sgdt;

/// Write a single byte to QEMU debug port (0xE9)
///
/// # Arguments
/// * `byte` - Byte to write
///
/// # Note
/// Only works in QEMU. On real hardware, writes to 0xE9 are ignored.
#[inline(always)]
pub(super) unsafe fn out_char_0xe9(byte: u8) {
    let mut port: Port<u8> = Port::new(0xE9);
    port.write(byte);
}

/// Write a string to QEMU debug port
///
/// # Arguments
/// * `s` - String to write (sent byte-by-byte)
pub(super) unsafe fn print_str_0xe9(s: &str) {
    for b in s.bytes() {
        out_char_0xe9(b);
    }
}

/// Print a 64-bit value as hex to QEMU debug port
///
/// Format: "0x" followed by 16 hex digits (uppercase)
///
/// # Arguments
/// * `v` - 64-bit value to print
///
/// # Examples
/// ```rust
/// unsafe { print_hex_u64_0xe9(0xDEADBEEF); }  // Prints: 0x00000000DEADBEEF
/// ```
pub(super) unsafe fn print_hex_u64_0xe9(v: u64) {
    out_char_0xe9(b'0');
    out_char_0xe9(b'x');

    // Print all 16 hex digits (4 bits each)
    for i in (0..16).rev() {
        let nib = ((v >> (i * 4)) & 0xF) as u8;
        let ch = if nib < 10 {
            b'0' + nib // 0-9
        } else {
            b'A' + (nib - 10) // A-F
        };
        out_char_0xe9(ch);
    }
}

/// Compute full 64-bit handler address from an IDT entry
///
/// IDT entries store the handler address in three pieces:
/// - offset_low: bits 0-15
/// - offset_mid: bits 16-31
/// - offset_high: bits 32-63
///
/// # Arguments
/// * `e` - IDT entry to extract address from
///
/// # Returns
/// * Full 64-bit handler address
fn idt_entry_addr(e: &IdtEntry) -> u64 {
    (e.offset_low as u64) | ((e.offset_mid as u64) << 16) | ((e.offset_high as u64) << 32)
}

/// Print one IDT entry in diagnostic format
///
/// Shows index, selector, type/attributes, and handler address.
///
/// # Arguments
/// * `idx` - Entry index (vector number)
/// * `e` - IDT entry to print
#[allow(dead_code)]
unsafe fn print_idt_entry(idx: usize, e: &IdtEntry) {
    print_str_0xe9("IDT[");

    // Print index as decimal
    let d = idx as u32;
    let mut buf = [0u8; 3];
    let mut n = d;
    let mut c = 0usize;

    if n == 0 {
        out_char_0xe9(b'0');
    } else {
        // Build digits in reverse
        while n > 0 {
            buf[c] = b'0' + (n % 10) as u8;
            n /= 10;
            c += 1;
        }
        // Print in forward order
        while c > 0 {
            c -= 1;
            out_char_0xe9(buf[c]);
        }
    }

    print_str_0xe9("] sel=");
    print_hex_u64_0xe9(e.selector as u64);
    print_str_0xe9(" typ=");
    print_hex_u64_0xe9(e.type_attr as u64);
    print_str_0xe9(" addr=");
    print_hex_u64_0xe9(idt_entry_addr(e));
    out_char_0xe9(b'\n');
}

/// Print a compact summary of key IDT handlers
///
/// Shows handler addresses for important exception vectors:
/// - Vector 0: Divide Error
/// - Vector 3: Breakpoint
/// - Vector 6: Invalid Opcode
/// - Vector 13: General Protection Fault
/// - Vector 14: Page Fault
/// - Vector 0x40: APIC Timer
///
/// # Output Format
/// ```text
/// IDT addrs: 0=0xFFFF... 3=0xFFFF... 6=0xFFFF... 13=0xFFFF... 14=0xFFFF...
/// IDT[0x40] sel=0x...
/// ```
///
/// # Safety
/// Reads from the IDT using SIDT instruction.
pub unsafe fn print_idt_summary_compact() {
    let indices = [0usize, 3, 6, 13, 14]; // Key exception vectors

    print_str_0xe9("IDT addrs: ");

    use x86_64::instructions::tables::sidt;
    let idtr = sidt();
    let base = idtr.base.as_u64();

    // Print key exception handler addresses
    for (k, &idx) in indices.iter().enumerate() {
        if k > 0 {
            print_str_0xe9(" ");
        }

        // Print vector number in decimal
        let d = idx as u32;
        let mut buf = [0u8; 3];
        let mut n = d;
        let mut c = 0usize;

        if n == 0 {
            out_char_0xe9(b'0');
        } else {
            while n > 0 {
                buf[c] = b'0' + (n % 10) as u8;
                n /= 10;
                c += 1;
            }
            while c > 0 {
                c -= 1;
                out_char_0xe9(buf[c]);
            }
        }

        out_char_0xe9(b'=');

        // Read IDT entry and print handler address
        let e = &*(base.wrapping_add((idx * core::mem::size_of::<IdtEntry>()) as u64)
            as *const IdtEntry);
        print_hex_u64_0xe9(idt_entry_addr(e));
    }
    out_char_0xe9(b'\n');

    // Also print selector for timer vector to verify it's correct
    let e40 = &*(base
        .wrapping_add(((APIC_TIMER_VECTOR as usize) * core::mem::size_of::<IdtEntry>()) as u64)
        as *const IdtEntry);
    print_str_0xe9("IDT[0x40] sel=");
    print_hex_u64_0xe9(e40.selector as u64);
    out_char_0xe9(b'\n');
}

/// Print basic GDT information for debugging
///
/// Shows:
/// - GDTR base and limit
/// - First 8 GDT entries (or fewer if GDT is smaller)
///
/// # Output Format
/// ```text
/// GDT base=0x... limit=0x...
/// GDT[0]=0x...
/// GDT[1]=0x...
/// ...
/// ```
///
/// # Safety
/// Reads from the GDT using SGDT instruction.
pub unsafe fn print_gdt_summary_basic() {
    let gdtr = sgdt();
    let base = gdtr.base.as_u64();
    let limit = gdtr.limit as u64;

    print_str_0xe9("GDT base=");
    print_hex_u64_0xe9(base);
    print_str_0xe9(" limit=");
    print_hex_u64_0xe9(limit);
    out_char_0xe9(b'\n');

    // Dump first 8 descriptors (8 bytes each)
    // Note: TSS descriptors span 16 bytes in 64-bit mode
    let count = core::cmp::min(((limit + 1) / 8) as usize, 8usize);

    for i in 0..count {
        let lo = core::ptr::read_unaligned((base + (i as u64) * 8) as *const u64);
        print_str_0xe9("GDT[");

        // Print index as decimal (manual implementation to avoid alloc)
        let d = i as u32;
        let mut buf = [0u8; 3];
        let mut n = d;
        let mut c = 0usize;

        if n == 0 {
            out_char_0xe9(b'0');
        } else {
            while n > 0 {
                buf[c] = b'0' + (n % 10) as u8;
                n /= 10;
                c += 1;
            }
            while c > 0 {
                c -= 1;
                out_char_0xe9(buf[c]);
            }
        }

        print_str_0xe9("]=");
        print_hex_u64_0xe9(lo);
        out_char_0xe9(b'\n');
    }
}
