//! Module: interrupts::apic
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//! - docs/axioms/memory.md#A2:-Physical-memory-is-accessed-through-a-fixed-PHYS_OFFSET-linear-mapping-after-paging-is-active
//!
//! INVARIANTS:
//! - This module provides LAPIC register access and the early interrupt-masking path used during kernel bring-up.
//! - APIC MMIO access assumes the LAPIC region is mapped according to the kernel’s documented memory setup.
//! - Legacy PIC masking is part of the transition to APIC-based delivery, not a claim that every architectural interrupt source disappears forever.
//!
//! SAFETY:
//! - APIC register reads/writes are raw MMIO and must only target the mapped LAPIC window.
//! - Early “disable interrupts” helpers are ordering-sensitive bring-up tools, not magical absolute guarantees about all NMI-class behavior.
//! - Any future switch away from the fixed LAPIC base assumption must update both code and docs together.
//!
//! PROGRESS:
//! - docs/plans/interrupts-and-platform.md
//!
//! APIC MMIO access and early bring-up helpers.
//!
//! This module contains the LAPIC accessors plus the early interrupt-masking path
//! used during kernel bring-up.

use crate::log_trace;
use x86_64::instructions::port::Port;

/// APIC error interrupt vector
pub(super) const APIC_ERROR_VECTOR: u8 = 0xFE;

/// Disable the current early-boot interrupt sources.
///
/// This clears IF, masks the legacy PIC, and disables NMI through the CMOS port.
/// It is an early bring-up helper, not a claim that every architectural interrupt
/// source has vanished forever.
pub unsafe fn disable_all_interrupts() {
    // Disable regular interrupts (clear IF flag in RFLAGS)
    x86_64::instructions::interrupts::disable();

    // Disable Non-Maskable Interrupts
    disable_nmi();

    // Mask all legacy PIC interrupts
    mask_pic_interrupts();
}

/// Disable NMI through the CMOS index port.
unsafe fn disable_nmi() {
    // Write to CMOS index port with NMI disable bit (bit 7 = 0x80)
    let mut port: Port<u8> = Port::new(0x70);
    port.write(0x80u8);
}

/// Mask all legacy 8259 PIC interrupts.
unsafe fn mask_pic_interrupts() {
    let mut master: Port<u8> = Port::new(0x21);
    let mut slave: Port<u8> = Port::new(0xA1);
    master.write(0xFFu8); // Mask all master PIC interrupts
    slave.write(0xFFu8); // Mask all slave PIC interrupts
}

/// Return whether CPUID reports APIC support.
#[allow(dead_code)]
unsafe fn has_apic() -> bool {
    if let Some(fi) = raw_cpuid::CpuId::new().get_feature_info() {
        return fi.has_apic();
    }
    false
}

/// Return the LAPIC physical base address assumed by the current bring-up path.
///
/// Today this is the fixed legacy base `0xFEE0_0000`; the function does not yet
/// query `IA32_APIC_BASE`.
pub unsafe fn get_apic_base() -> u64 {
    // Use standard LAPIC base address
    // Could be read from IA32_APIC_BASE MSR (0x1B) if needed:
    // let mut msr = Msr::new(0x1B);
    // let val = msr.read();
    // (val & 0xFFFF_F000) as u64
    0xFEE0_0000u64
}

/// Read a LAPIC register through the `PHYS_OFFSET` mapping.
pub unsafe fn read_apic_register(apic_base: u64, offset: u32) -> u32 {
    // The LAPIC MMIO is mapped at PHYS_OFFSET + physical address
    let vbase = crate::memory::phys_to_virt_pa(apic_base & 0xFFFFF000);
    let addr = vbase + (offset as u64);

    // Use volatile read to ensure the hardware access isn't optimized away
    let val = core::ptr::read_volatile(addr as *const u32);

    // Debug MMIO operations if needed (disabled for performance)
    const APIC_MMIO_DEBUG: bool = false;
    if APIC_MMIO_DEBUG {
        log_trace!("LAPIC MMIO READ addr={:#x} val={:#x}", addr, val);
    }

    val
}

/// Write a LAPIC register through the `PHYS_OFFSET` mapping.
pub(super) unsafe fn write_apic_register(apic_base: u64, offset: u32, value: u32) {
    let vbase = crate::memory::phys_to_virt_pa(apic_base & 0xFFFFF000);
    let addr = vbase + (offset as u64);

    // Use volatile write to ensure the hardware access happens
    core::ptr::write_volatile(addr as *mut u32, value);

    // Debug MMIO operations if needed (disabled for performance)
    const APIC_MMIO_DEBUG: bool = false;
    if APIC_MMIO_DEBUG {
        log_trace!("LAPIC MMIO WRITE addr={:#x} val={:#x}", addr, value);
    }
}

/// Set the CPU interrupt flag once the runtime is ready for maskable IRQs.
pub unsafe fn enable_interrupts() {
    x86_64::instructions::interrupts::enable();
}
