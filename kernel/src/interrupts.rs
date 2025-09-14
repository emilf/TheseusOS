//! Interrupt control module
//! 
//! This module provides functions to disable all interrupts including NMI,
//! and control various interrupt sources during kernel initialization.

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
