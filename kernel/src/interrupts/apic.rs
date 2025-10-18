//! APIC (Advanced Programmable Interrupt Controller) management
//!
//! This module provides functions for configuring and controlling the Local APIC:
//! - APIC base address detection
//! - MMIO register access (read/write)
//! - APIC initialization and configuration
//! - Interrupt disabling (APIC, NMI, PIC)
//!
//! ## Local APIC Overview
//!
//! The Local APIC is a per-CPU interrupt controller in modern x86-64 systems.
//! It handles:
//! - Timer interrupts
//! - Inter-processor interrupts (IPI)
//! - Local interrupt sources (LINT0, LINT1)
//! - Error interrupts
//! - Spurious interrupts
//!
//! ## MMIO Access
//!
//! The APIC registers are accessed via Memory-Mapped I/O (MMIO) at a physical
//! address specified by the IA32_APIC_BASE MSR (typically 0xFEE00000).
//!
//! Common APIC registers (offset from base):
//! - 0x020: Local APIC ID
//! - 0x030: Local APIC Version
//! - 0x080: Task Priority Register (TPR)
//! - 0x0B0: End of Interrupt (EOI)
//! - 0x0F0: Spurious Interrupt Vector Register (SIVR)
//! - 0x280: Error Status Register (ESR)
//! - 0x320: LVT Timer
//! - 0x350: LVT LINT0
//! - 0x360: LVT LINT1
//! - 0x370: LVT Error
//! - 0x380: Timer Initial Count
//! - 0x390: Timer Current Count
//! - 0x3E0: Timer Divide Configuration

use crate::{log_debug, log_trace};
use x86_64::instructions::port::Port;

/// APIC error interrupt vector
pub(super) const APIC_ERROR_VECTOR: u8 = 0xFE;

/// Disable all interrupts including NMI
///
/// Comprehensively disables all interrupt sources for a clean kernel environment:
/// 1. Regular interrupts (IF flag) via CLI
/// 2. Non-Maskable Interrupts (NMI) via CMOS port
/// 3. Local APIC interrupts
/// 4. Legacy PIC (8259) interrupts
///
/// # Safety
/// Should only be called during early kernel initialization before interrupt
/// handlers are set up.
///
/// # Examples
/// Called during kernel boot sequence:
/// ```rust
/// unsafe { disable_all_interrupts(); }
/// // Now safe to set up GDT, IDT, etc.
/// ```
pub unsafe fn disable_all_interrupts() {
    // Disable regular interrupts (clear IF flag in RFLAGS)
    x86_64::instructions::interrupts::disable();

    // Disable Non-Maskable Interrupts
    disable_nmi();

    // Mask all legacy PIC interrupts
    mask_pic_interrupts();
}

/// Disable Non-Maskable Interrupts (NMI)
///
/// NMI is disabled by setting bit 7 of the CMOS index port (0x70).
/// This prevents NMI from firing during critical kernel operations.
///
/// # Note
/// NMI is used for critical hardware events (memory errors, watchdog),
/// so it should be re-enabled after kernel initialization if needed.
unsafe fn disable_nmi() {
    // Write to CMOS index port with NMI disable bit (bit 7 = 0x80)
    let mut port: Port<u8> = Port::new(0x70);
    port.write(0x80u8);
}

/// Mask all legacy PIC (8259) interrupts
///
/// The 8259 PIC was used in older systems before APIC. In modern systems
/// with APIC, we mask all PIC interrupts to prevent conflicts.
///
/// PIC ports:
/// - 0x21: Master PIC data port (IRQ 0-7)
/// - 0xA1: Slave PIC data port (IRQ 8-15)
///
/// Writing 0xFF masks all 8 interrupt lines on each controller.
unsafe fn mask_pic_interrupts() {
    let mut master: Port<u8> = Port::new(0x21);
    let mut slave: Port<u8> = Port::new(0xA1);
    master.write(0xFFu8);  // Mask all master PIC interrupts
    slave.write(0xFFu8);   // Mask all slave PIC interrupts
}

/// Check if CPU supports APIC
///
/// Uses CPUID to detect APIC support.
///
/// # Returns
/// * `true` - APIC is available
/// * `false` - APIC not supported (ancient CPU)
#[allow(dead_code)]
unsafe fn has_apic() -> bool {
    if let Some(fi) = raw_cpuid::CpuId::new().get_feature_info() {
        return fi.has_apic();
    }
    false
}

/// Get Local APIC base address
///
/// Returns the physical base address of the Local APIC MMIO region.
///
/// # Implementation Note
/// Currently returns the standard APIC base address (0xFEE00000) to avoid
/// potential issues with RDMSR during early boot. In the future, this could
/// read the IA32_APIC_BASE MSR (0x1B) to support relocated APICs.
///
/// # Returns
/// * Physical address of LAPIC (typically 0xFEE00000)
///
/// # Safety
/// The returned address must be mapped in virtual memory before accessing
/// APIC registers.
/// 
/// # TODO: Make it actually return the base address of the LAPIC MMIO region
pub unsafe fn get_apic_base() -> u64 {
    // Use standard LAPIC base address
    // Could be read from IA32_APIC_BASE MSR (0x1B) if needed:
    // let mut msr = Msr::new(0x1B);
    // let val = msr.read();
    // (val & 0xFFFF_F000) as u64
    0xFEE0_0000u64
}

/// Read a Local APIC register via MMIO
///
/// Accesses an APIC register by reading from the mapped MMIO address.
///
/// # Arguments
/// * `apic_base` - Physical base address of LAPIC (from `get_apic_base()`)
/// * `offset` - Register offset in bytes (e.g., 0x20 for APIC ID)
///
/// # Returns
/// * 32-bit register value
///
/// # Safety
/// Requires:
/// - APIC MMIO region to be mapped via PHYS_OFFSET
/// - Valid register offset (must be 16-byte aligned for most registers)
/// - Paging to be active
///
/// # Common Offsets
/// - 0x020: APIC ID
/// - 0x030: APIC Version  
/// - 0x080: Task Priority (TPR)
/// - 0x0F0: Spurious Interrupt Vector
/// - 0x320: LVT Timer
/// - 0x380: Timer Initial Count
/// - 0x390: Timer Current Count
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

/// Write to a Local APIC register via MMIO
///
/// Accesses an APIC register by writing to the mapped MMIO address.
///
/// # Arguments
/// * `apic_base` - Physical base address of LAPIC (from `get_apic_base()`)
/// * `offset` - Register offset in bytes
/// * `value` - 32-bit value to write
///
/// # Safety
/// Requires:
/// - APIC MMIO region to be mapped via PHYS_OFFSET
/// - Valid register offset
/// - Paging to be active
/// - Appropriate value for the register (incorrect values can cause system instability)
///
/// # Warning
/// Writing to APIC registers can have immediate hardware effects:
/// - EOI (0xB0): Acknowledges interrupt
/// - SIVR (0xF0): Enable/disable APIC
/// - Timer registers: Start/stop timer
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

/// Enable CPU interrupts
///
/// Sets the IF (Interrupt Flag) in RFLAGS, allowing maskable interrupts
/// to be delivered to the CPU.
///
/// # Safety
/// Should only be called when:
/// - IDT is properly set up with handlers
/// - Stack is valid and has sufficient space
/// - System is ready to handle interrupts
pub unsafe fn enable_interrupts() {
    x86_64::instructions::interrupts::enable();
}

