//! Interrupt control module
//! 
//! This module provides functions to disable all interrupts including NMI,
//! and control various interrupt sources during kernel initialization.

/// Minimal IDT structures for exception handling
#[repr(C, packed)]
#[derive(Copy, Clone)]
struct IdtEntry {
    offset_low: u16,
    selector: u16,
    options: u16,
    offset_mid: u32,
    offset_high: u32,
    zero: u32,
}

impl IdtEntry {
    const fn missing() -> Self {
        Self { offset_low: 0, selector: 0, options: 0, offset_mid: 0, offset_high: 0, zero: 0 }
    }

    fn set_handler(&mut self, handler: extern "C" fn() -> !) {
        let addr = handler as u64;
        self.offset_low = addr as u16;
        self.selector = super::gdt::KERNEL_CS as u16;
        // present=1, DPL=0, type=0b1110 (interrupt gate)
        self.options = 0b1000_1110_00000000u16;
        self.offset_mid = (addr >> 16) as u32;
        self.offset_high = (addr >> 32) as u32;
        self.zero = 0;
    }
}

#[repr(C, packed)]
struct IdtPointer {
    limit: u16,
    base: u64,
}

static mut IDT: [IdtEntry; 256] = [const { IdtEntry::missing() }; 256];

#[inline(always)]
unsafe fn out_char_0xe9(byte: u8) {
    core::arch::asm!(
        "out dx, al",
        in("dx") 0xE9u16,
        in("al") byte,
        options(nomem, nostack, preserves_flags)
    );
}

unsafe fn print_str_0xe9(s: &str) {
    for b in s.bytes() { out_char_0xe9(b); }
}

unsafe fn print_hex_u64_0xe9(mut v: u64) {
    // print 0xXXXXXXXXXXXXXXX
    out_char_0xe9(b'0');
    out_char_0xe9(b'x');
    for i in (0..16).rev() {
        let nib = ((v >> (i * 4)) & 0xF) as u8;
        let ch = if nib < 10 { b'0' + nib } else { b'A' + (nib - 10) };
        out_char_0xe9(ch);
    }
}

/// Set up a basic IDT with exception handlers
pub unsafe fn setup_idt() {
    // Install a few critical exception handlers
    IDT[0].set_handler(isr_de);
    IDT[13].set_handler(isr_gp);
    IDT[14].set_handler(isr_pf);

    let idt_ptr = IdtPointer {
        limit: (core::mem::size_of_val(&IDT) - 1) as u16,
        base: &IDT as *const _ as u64,
    };
    core::arch::asm!("lidt [{}]", in(reg) &idt_ptr, options(readonly, nostack, preserves_flags));
}

extern "C" fn isr_de() -> ! {
    unsafe {
        print_str_0xe9("DE\n");
        core::arch::asm!(
            "mov dx, 0xF4",
            "mov al, 0x01",
            "out dx, al",
            "cli",
            "2: hlt",
            "jmp 2b",
            options(noreturn)
        );
    }
}

extern "C" fn isr_gp() -> ! {
    unsafe {
        print_str_0xe9("GP\n");
        core::arch::asm!(
            "mov dx, 0xF4",
            "mov al, 0x01",
            "out dx, al",
            "cli",
            "2: hlt",
            "jmp 2b",
            options(noreturn)
        );
    }
}

extern "C" fn isr_pf() -> ! {
    unsafe {
        let mut cr2v: u64;
        core::arch::asm!("mov {}, cr2", out(reg) cr2v, options(nomem, nostack, preserves_flags));
        print_str_0xe9("PF CR2=");
        print_hex_u64_0xe9(cr2v);
        out_char_0xe9(b'\n');
        core::arch::asm!(
            "mov dx, 0xF4",
            "mov al, 0x01",
            "out dx, al",
            "cli",
            "2: hlt",
            "jmp 2b",
            options(noreturn)
        );
    }
}

/// Disable all interrupts including NMI
pub unsafe fn disable_all_interrupts() {
    // Disable regular interrupts (CLI)
    core::arch::asm!("cli", options(nomem, nostack, preserves_flags));
    
    // Disable NMI
    disable_nmi();
    
    // Disable local APIC interrupts
    disable_local_apic();
    
    // Mask all PIC interrupts
    mask_pic_interrupts();
}

/// Disable Non-Maskable Interrupts (NMI)
unsafe fn disable_nmi() {
    // Read from CMOS register to disable NMI
    core::arch::asm!(
        "out dx, al",
        in("dx") 0x70u16,
        in("al") 0x80u8,
        options(nomem, nostack, preserves_flags)
    );
}

/// Disable local APIC interrupts
unsafe fn disable_local_apic() {
    // Check if APIC is available
    if has_apic() {
        // Disable APIC by clearing the APIC enable bit
        let apic_base = get_apic_base();
        if apic_base != 0 {
            let mut apic_sivr = read_apic_register(apic_base, 0xF0); // SIVR register
            apic_sivr &= !0x100; // Clear APIC enable bit
            write_apic_register(apic_base, 0xF0, apic_sivr);
        }
    }
}

/// Mask all PIC interrupts
unsafe fn mask_pic_interrupts() {
    // Mask all interrupts on master PIC
    core::arch::asm!(
        "out dx, al",
        in("dx") 0x21u16,
        in("al") 0xFFu8,
        options(nomem, nostack, preserves_flags)
    );
    
    // Mask all interrupts on slave PIC
    core::arch::asm!(
        "out dx, al",
        in("dx") 0xA1u16,
        in("al") 0xFFu8,
        options(nomem, nostack, preserves_flags)
    );
}

/// Check if APIC is available
unsafe fn has_apic() -> bool {
    // Check CPUID feature flags for APIC
    let _eax: u32;
    let _ecx: u32;
    let edx: u32;
    
    core::arch::asm!(
        "cpuid",
        inout("eax") 1 => _eax,
        out("ecx") _ecx,
        out("edx") edx,
        options(nomem, nostack, preserves_flags)
    );
    
    // Check APIC bit in EDX
    (edx & (1 << 9)) != 0
}

/// Get APIC base address from IA32_APIC_BASE MSR
unsafe fn get_apic_base() -> u64 {
    let mut eax: u32;
    let mut edx: u32;
    
    core::arch::asm!(
        "rdmsr",
        in("ecx") 0x1Bu32, // IA32_APIC_BASE MSR
        out("eax") eax,
        out("edx") edx,
        options(nomem, nostack, preserves_flags)
    );
    
    ((edx as u64) << 32) | (eax as u64)
}

/// Read APIC register
unsafe fn read_apic_register(apic_base: u64, offset: u32) -> u32 {
    let addr = (apic_base & 0xFFFFF000) | (offset as u64);
    core::ptr::read_volatile(addr as *const u32)
}

/// Write APIC register
unsafe fn write_apic_register(apic_base: u64, offset: u32, value: u32) {
    let addr = (apic_base & 0xFFFFF000) | (offset as u64);
    core::ptr::write_volatile(addr as *mut u32, value);
}

/// Enable interrupts (for future use)
pub unsafe fn enable_interrupts() {
    core::arch::asm!("sti", options(nomem, nostack, preserves_flags));
}

/// Check if interrupts are enabled
pub fn interrupts_enabled() -> bool {
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "pushfq",
            "pop {}",
            out(reg) flags,
            options(nomem, nostack, preserves_flags)
        );
    }
    (flags & 0x200) != 0 // IF flag
}
