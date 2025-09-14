//! Global Descriptor Table (GDT) module
//! 
//! This module provides the GDT setup required for x86-64 long mode.
//! It creates the necessary segment descriptors for kernel and user mode operation.

use core::mem::size_of;

/// GDT Entry structure
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct GdtEntry {
    /// Low 16 bits of limit
    limit_low: u16,
    /// Low 16 bits of base address
    base_low: u16,
    /// Middle 8 bits of base address
    base_middle: u8,
    /// Access byte (P, DPL, S, Type)
    access: u8,
    /// Flags and high 4 bits of limit
    flags_limit: u8,
    /// High 8 bits of base address
    base_high: u8,
}

impl GdtEntry {
    /// Create a new GDT entry
    const fn new(base: u32, limit: u32, access: u8, flags: u8) -> Self {
        Self {
            limit_low: limit as u16,
            base_low: base as u16,
            base_middle: (base >> 16) as u8,
            access,
            flags_limit: (((limit >> 16) & 0xF) as u8) | (flags << 4),
            base_high: (base >> 24) as u8,
        }
    }
}

/// GDT Pointer structure
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct GdtPointer {
    /// Size of GDT in bytes minus 1
    limit: u16,
    /// Base address of GDT
    base: u64,
}

impl GdtPointer {
    /// Create a new GDT pointer
    pub fn new(gdt: &[GdtEntry]) -> Self {
        Self {
            limit: (gdt.len() * size_of::<GdtEntry>() - 1) as u16,
            base: gdt.as_ptr() as u64,
        }
    }

    /// Load the GDT
    pub unsafe fn load(&self) {
        core::arch::asm!(
            "lgdt [{}]",
            in(reg) self,
            options(readonly, nostack, preserves_flags)
        );
    }
}

/// Segment selectors
pub const NULL_SELECTOR: u16 = 0x00;
pub const KERNEL_CS: u16 = 0x08;
pub const KERNEL_DS: u16 = 0x10;
pub const USER_CS: u16 = 0x18;
pub const USER_DS: u16 = 0x20;
pub const TSS_SELECTOR: u16 = 0x28;

/// Access byte flags
const ACCESS_PRESENT: u8 = 1 << 7;
const ACCESS_PRIVILEGE_RING0: u8 = 0 << 5;
const ACCESS_PRIVILEGE_RING3: u8 = 3 << 5;
const ACCESS_SYSTEM: u8 = 0 << 4;
const ACCESS_CODE_DATA: u8 = 1 << 4;
const ACCESS_EXECUTABLE: u8 = 1 << 3;
const ACCESS_CONFORMING: u8 = 1 << 2;
const ACCESS_READABLE: u8 = 1 << 1;
const ACCESS_WRITABLE: u8 = 1 << 1;
const ACCESS_ACCESSED: u8 = 1 << 0;

/// Flags
const FLAGS_64BIT: u8 = 1 << 1;
const FLAGS_32BIT: u8 = 1 << 0;
const FLAGS_GRANULARITY: u8 = 1 << 3;

/// Global GDT
static mut GDT: [GdtEntry; 6] = [
    // Null descriptor
    GdtEntry::new(0, 0, 0, 0),
    // Kernel code segment (64-bit, ring 0, executable)
    GdtEntry::new(0, 0, 
        ACCESS_PRESENT | ACCESS_PRIVILEGE_RING0 | ACCESS_CODE_DATA | ACCESS_EXECUTABLE | ACCESS_READABLE,
        FLAGS_64BIT | FLAGS_GRANULARITY
    ),
    // Kernel data segment (64-bit, ring 0, writable)
    GdtEntry::new(0, 0,
        ACCESS_PRESENT | ACCESS_PRIVILEGE_RING0 | ACCESS_CODE_DATA | ACCESS_WRITABLE,
        FLAGS_64BIT | FLAGS_GRANULARITY
    ),
    // User code segment (64-bit, ring 3, executable)
    GdtEntry::new(0, 0,
        ACCESS_PRESENT | ACCESS_PRIVILEGE_RING3 | ACCESS_CODE_DATA | ACCESS_EXECUTABLE | ACCESS_READABLE,
        FLAGS_64BIT | FLAGS_GRANULARITY
    ),
    // User data segment (64-bit, ring 3, writable)
    GdtEntry::new(0, 0,
        ACCESS_PRESENT | ACCESS_PRIVILEGE_RING3 | ACCESS_CODE_DATA | ACCESS_WRITABLE,
        FLAGS_64BIT | FLAGS_GRANULARITY
    ),
    // TSS segment (placeholder for now)
    GdtEntry::new(0, 0, 0, 0),
];

/// Set up and load the GDT
pub unsafe fn setup_gdt() {
    // Create GDT pointer
    let gdt_ptr = GdtPointer::new(&GDT);
    
    // Load GDT
    gdt_ptr.load();
    
    // Reload segment registers
    reload_segments();
}

/// Reload segment registers with new selectors
unsafe fn reload_segments() {
    core::arch::asm!(
        "mov ds, ax",
        "mov es, ax", 
        "mov fs, ax",
        "mov gs, ax",
        "mov ss, ax",
        in("ax") KERNEL_DS,
        options(nomem, nostack, preserves_flags)
    );
    
    // Far jump to reload CS
    core::arch::asm!(
        "push {0:x}",
        "lea {1}, [rip + 2]",
        "push {1}",
        "retfq",
        "2:",
        in(reg_abcd) KERNEL_CS,
        out(reg) _,
        options(nomem, nostack)
    );
}
