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
//! - The current runtime still assumes xAPIC-style MMIO register access even though the LAPIC base now comes from `IA32_APIC_BASE`; future x2APIC enablement must update both code and docs together.
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
use x86_64::registers::model_specific::Msr;

/// APIC error interrupt vector
pub(super) const APIC_ERROR_VECTOR: u8 = 0xFE;

const IA32_APIC_BASE_MSR: u32 = 0x1B;
const IA32_APIC_BASE_BSP_BIT: u64 = 1 << 8;
const IA32_APIC_BASE_X2APIC_ENABLE_BIT: u64 = 1 << 10;
const IA32_APIC_BASE_GLOBAL_ENABLE_BIT: u64 = 1 << 11;
const IA32_APIC_BASE_PHYS_MASK: u64 = 0xFFFF_F000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApicAccessMode {
    Disabled,
    XApic,
    X2Apic,
}

impl ApicAccessMode {
    pub const fn as_str(self) -> &'static str {
        match self {
            ApicAccessMode::Disabled => "disabled",
            ApicAccessMode::XApic => "xapic",
            ApicAccessMode::X2Apic => "x2apic",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ApicBaseInfo {
    pub raw_msr: u64,
    pub phys_base: u64,
    pub global_enabled: bool,
    pub x2apic_enabled: bool,
    pub bsp: bool,
}

impl ApicBaseInfo {
    pub const fn access_mode(self) -> ApicAccessMode {
        if !self.global_enabled {
            ApicAccessMode::Disabled
        } else if self.x2apic_enabled {
            ApicAccessMode::X2Apic
        } else {
            ApicAccessMode::XApic
        }
    }
}

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

/// Read and decode `IA32_APIC_BASE`.
pub unsafe fn apic_base_info() -> ApicBaseInfo {
    let raw = Msr::new(IA32_APIC_BASE_MSR).read();
    ApicBaseInfo {
        raw_msr: raw,
        phys_base: raw & IA32_APIC_BASE_PHYS_MASK,
        global_enabled: (raw & IA32_APIC_BASE_GLOBAL_ENABLE_BIT) != 0,
        x2apic_enabled: (raw & IA32_APIC_BASE_X2APIC_ENABLE_BIT) != 0,
        bsp: (raw & IA32_APIC_BASE_BSP_BIT) != 0,
    }
}

/// Return the LAPIC physical base address reported by `IA32_APIC_BASE`.
///
/// The current runtime still uses xAPIC/MMIO register access even when CPUID advertises
/// x2APIC capability; callers that need mode information should inspect [`apic_base_info`].
pub unsafe fn get_apic_base() -> u64 {
    apic_base_info().phys_base
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
