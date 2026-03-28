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

use crate::log_info;
use spin::Once;
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
    crate::cpu_features::CpuFeatures::get().apic
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
pub(super) unsafe fn get_apic_base() -> u64 {
    apic_base_info().phys_base
}

/// Cached APIC access mode, initialized once during early LAPIC setup.
static APIC_MODE: Once<ApicAccessMode> = Once::new();

/// Detect and cache the APIC access mode. Call once before first LAPIC access.
pub unsafe fn init_apic_mode() {
    let info = apic_base_info();
    let mode = info.access_mode();
    APIC_MODE.call_once(|| mode);
    log_info!("APIC mode: {}", mode.as_str());
}

/// Get the cached APIC access mode.
fn cached_apic_mode() -> ApicAccessMode {
    *APIC_MODE.get().unwrap_or(&ApicAccessMode::XApic)
}

/// Read a LAPIC register through the `PHYS_OFFSET` mapping (xAPIC MMIO path).
pub(super) unsafe fn read_apic_register(apic_base: u64, offset: u32) -> u32 {
    let vbase = crate::memory::phys_to_virt_pa(apic_base & 0xFFFFF000);
    let addr = vbase + (offset as u64);
    core::ptr::read_volatile(addr as *const u32)
}

/// Write a LAPIC register through the `PHYS_OFFSET` mapping (xAPIC MMIO path).
pub(super) unsafe fn write_apic_register(apic_base: u64, offset: u32, value: u32) {
    let vbase = crate::memory::phys_to_virt_pa(apic_base & 0xFFFFF000);
    let addr = vbase + (offset as u64);
    core::ptr::write_volatile(addr as *mut u32, value);
}

/// Read a LAPIC register via x2APIC MSR.
///
/// x2APIC registers are at MSR `0x800 + (offset / 16)`.
/// ICR (offset 0x300) is special: in x2APIC it's a single 64-bit MSR at 0x830.
unsafe fn x2apic_read(offset: u32) -> u32 {
    let msr_addr = 0x800 + (offset >> 4);
    Msr::new(msr_addr).read() as u32
}

/// Write a LAPIC register via x2APIC MSR.
///
/// For ICR (offset 0x300/0x310): in x2APIC mode, ICR is a single 64-bit write
/// at MSR 0x830. The caller must use `x2apic_write_icr` for ICR writes.
unsafe fn x2apic_write(offset: u32, value: u32) {
    let msr_addr = 0x800 + (offset >> 4);
    Msr::new(msr_addr).write(value as u64);
}

/// Write the x2APIC ICR as a single 64-bit MSR write.
///
/// `low` is the ICR low half (vector, delivery mode, etc.).
/// `high` is the destination APIC ID (full 32-bit in x2APIC).
#[allow(dead_code)]
unsafe fn x2apic_write_icr(low: u32, high: u32) {
    let val = ((high as u64) << 32) | (low as u64);
    Msr::new(0x830).write(val);
}

/// Return the current processor's APIC ID.
pub unsafe fn local_apic_id() -> u32 {
    match cached_apic_mode() {
        ApicAccessMode::XApic => {
            let base = get_apic_base();
            read_apic_register(base, 0x20) >> 24
        }
        ApicAccessMode::X2Apic => {
            // x2APIC ID is the full 32-bit value (no shift needed)
            x2apic_read(0x20)
        }
        ApicAccessMode::Disabled => {
            panic!("LAPIC access while APIC disabled (local_apic_id)");
        }
    }
}

/// Signal end-of-interrupt to the Local APIC.
pub unsafe fn local_apic_eoi() {
    match cached_apic_mode() {
        ApicAccessMode::XApic => {
            let base = get_apic_base();
            write_apic_register(base, 0xB0, 0);
        }
        ApicAccessMode::X2Apic => {
            x2apic_write(0xB0, 0);
        }
        ApicAccessMode::Disabled => {
            panic!("LAPIC access while APIC disabled (local_apic_eoi)");
        }
    }
}

/// Read one Local APIC register through the mode-appropriate path.
pub unsafe fn local_apic_read(offset: u32) -> u32 {
    match cached_apic_mode() {
        ApicAccessMode::XApic => {
            let base = get_apic_base();
            read_apic_register(base, offset)
        }
        ApicAccessMode::X2Apic => x2apic_read(offset),
        ApicAccessMode::Disabled => {
            panic!("LAPIC access while APIC disabled (local_apic_read)");
        }
    }
}

/// Write one Local APIC register through the mode-appropriate path.
///
/// For ICR writes in x2APIC mode, offset 0x300 is handled specially:
/// the high half (0x310 destination) must be written first via `local_apic_write(0x310, dest)`,
/// then this function combines them for the 64-bit MSR write.
pub unsafe fn local_apic_write(offset: u32, value: u32) {
    match cached_apic_mode() {
        ApicAccessMode::XApic => {
            let base = get_apic_base();
            write_apic_register(base, offset, value);
        }
        ApicAccessMode::X2Apic => {
            // ICR low (0x300) in x2APIC is special — it triggers the IPI.
            // We write it as a regular MSR for now; full 64-bit ICR writes
            // should use x2apic_write_icr() when IPI support is implemented.
            x2apic_write(offset, value);
        }
        ApicAccessMode::Disabled => {
            panic!("LAPIC access while APIC disabled (local_apic_write)");
        }
    }
}

/// Set the CPU interrupt flag once the runtime is ready for maskable IRQs.
pub unsafe fn enable_interrupts() {
    x86_64::instructions::interrupts::enable();
}
