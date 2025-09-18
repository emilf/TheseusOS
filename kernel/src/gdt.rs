//! Global Descriptor Table (GDT) module
//! 
//! This module provides the GDT setup required for x86-64 long mode.
//! It creates the necessary segment descriptors for kernel and user mode operation.

use core::mem::size_of;
#[cfg(feature = "new_arch")]
use alloc::boxed::Box;

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
            flags_limit: ((limit >> 16) & 0xF) as u8 | (flags << 4),
            base_high: (base >> 24) as u8,
        }
    }
}

/// GDT Pointer structure
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
struct GdtPointer {
    /// Size of GDT in bytes minus 1
    limit: u16,
    /// Base address of GDT
    base: u64,
}

impl GdtPointer {
    /// Create a new GDT pointer
    fn new(gdt: &[GdtEntry]) -> Self {
        Self {
            limit: (gdt.len() * size_of::<GdtEntry>() - 1) as u16,
            base: gdt.as_ptr() as u64,
        }
    }
    
    /// Load the GDT using lgdt instruction
    unsafe fn load(&self) {
        core::arch::asm!(
            "lgdt [{0}]",
            in(reg) self as *const Self,
            options(nomem, nostack, preserves_flags)
        );
    }
}

/// GDT entries
static GDT: [GdtEntry; 6] = [
    // Entry 0: Null descriptor (required)
    GdtEntry::new(0, 0, 0x00, 0x00),
    
    // Entry 1: Kernel code segment (64-bit)
    // flags nibble: G=1, D/B=0, L=1, AVL=0 => 0xA
    GdtEntry::new(0, 0, 0x9A, 0x0A),
    
    // Entry 2: Kernel data segment
    // In long mode, data segment flags are ignored; use 0x0
    GdtEntry::new(0, 0, 0x92, 0x00),
    
    // Entry 3: User code segment (64-bit)
    GdtEntry::new(0, 0, 0xFA, 0x0A),
    
    // Entry 4: User data segment
    GdtEntry::new(0, 0, 0xF2, 0x00),
    
    // Entry 5: TSS (placeholder for future use)
    GdtEntry::new(0, 0, 0x00, 0x00),
];

/// Selectors
pub const KERNEL_CS: u16 = 0x08; // Entry 1 (kernel code)
#[allow(dead_code)]
const KERNEL_DS: u16 = 0x10; // Entry 2 (kernel data)
#[allow(dead_code)]
const USER_CS: u16 = 0x18;   // Entry 3 (user code)
#[allow(dead_code)]
const USER_DS: u16 = 0x20;   // Entry 4 (user data)

/// Set up and load the GDT
pub unsafe fn setup_gdt() {
    #[cfg(feature = "new_arch")]
    {
        use x86_64::structures::gdt::{Descriptor, GlobalDescriptorTable};
        use x86_64::structures::tss::TaskStateSegment;
        use x86_64::instructions::segmentation::{CS, DS, ES, FS, GS};
        use x86_64::instructions::segmentation::Segment;
        use x86_64::structures::gdt::SegmentSelector;

        // Build a minimal GDT: kernel code + kernel data, optionally TSS later
        let mut gdt = GlobalDescriptorTable::new();
        let kernel_code: SegmentSelector = gdt.add_entry(Descriptor::kernel_code_segment());
        let kernel_data: SegmentSelector = gdt.add_entry(Descriptor::kernel_data_segment());

        // Leak to static so it lives while active
        let gdt_ref: &'static GlobalDescriptorTable = alloc::boxed::Box::leak(alloc::boxed::Box::new(gdt));
        gdt_ref.load();

        // Reload segment registers
        unsafe {
            // Set data segments to null selector in long mode is allowed, but we set to kernel data for clarity
            DS::set_reg(kernel_data);
            ES::set_reg(kernel_data);
            FS::set_reg(kernel_data);
            GS::set_reg(kernel_data);
            // Reload CS using far return via CS::set_reg
            unsafe { CS::set_reg(kernel_code) };
            // SS is typically left alone in long mode unless needed
        }
        return;
    }

    // Legacy path: Create GDT pointer using raw pointer to avoid static_mut_refs warning
    let gdt_ptr = core::ptr::addr_of!(GDT) as *const GdtEntry;
    let gdt_slice = core::slice::from_raw_parts(gdt_ptr, 6);
    let gdt_ptr = GdtPointer::new(gdt_slice);
    
    // Load GDT
    gdt_ptr.load();
    
    // Reload segment registers
    reload_segments();
}

/// Reload segment registers with new selectors
unsafe fn reload_segments() {
    // ds := 0 (null selector is permitted for data segs in long mode)
    core::arch::asm!("xor eax, eax", "mov ds, ax", options(nomem, nostack, preserves_flags));

    // es := 0
    core::arch::asm!("xor eax, eax", "mov es, ax", options(nomem, nostack, preserves_flags));

    // fs := 0 (FS base will be set via MSR when needed)
    core::arch::asm!("xor eax, eax", "mov fs, ax", options(nomem, nostack, preserves_flags));

    // gs := 0 (GS base via MSR later if needed)
    core::arch::asm!("xor eax, eax", "mov gs, ax", options(nomem, nostack, preserves_flags));

    // skip ss for now (can cause #GP in long mode if not needed)
    // core::arch::asm!("mov ss, ax", in("ax") KERNEL_DS, options(nomem, nostack, preserves_flags));
    
    // Reload CS using the cleaner approach
    reload_cs();
}

/// Reload CS register using the cleaner approach
/// 
/// This uses the pattern (converted from AT&T to Intel syntax):
/// sub rsp, 16
/// mov qword ptr [rsp + 8], 8     ; CS selector
/// mov rax, return_addr           ; Load return address
/// mov qword ptr [rsp], rax       ; RIP
/// retfq
unsafe fn reload_cs() {
    core::arch::asm!(
        // build far-return frame: [RIP]=label 2f, [CS]=KERNEL_CS
        "sub rsp, 16",
        "mov qword ptr [rsp + 8], {cs}",
        "lea rax, [rip + 2f]",
        "mov qword ptr [rsp], rax",
        "retfq",
        // return lands here
        "2:",
        cs = const (KERNEL_CS as u64),
        lateout("rax") _,
    );
}
