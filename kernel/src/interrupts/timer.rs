//! Module: interrupts::timer
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - The LAPIC timer is the current kernel timer source used during bring-up for verification and later periodic work.
//! - Timer vector programming must stay aligned with the IDT/interrupt-module ownership of `APIC_TIMER_VECTOR`.
//! - Tick counting here is shared runtime state consumed by observability and simple timing paths.
//!
//! SAFETY:
//! - Timer programming is raw LAPIC register manipulation and assumes APIC access is already valid.
//! - One-shot/periodic examples in comments must not overstate the timing precision; the actual cadence still depends on hardware/QEMU behavior and divider choices.
//! - Bringing up a timer before the handler/vector path is coherent risks spurious or misleading failures.
//!
//! PROGRESS:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/observability.md
//!
//! LAPIC timer configuration and control.
//!
//! This module owns the current LAPIC timer bring-up helpers, including one-shot,
//! periodic, and shared tick-counter support.

use super::{local_apic_read, local_apic_write};
use crate::{log_debug, log_trace};
use core::sync::atomic::Ordering;
// Debug output functions kept for potential low-level debugging
use super::apic::APIC_ERROR_VECTOR;

#[allow(unused_imports)]
use super::{out_char_0xe9, print_hex_u64_0xe9, print_str_0xe9};
use super::{APIC_TIMER_VECTOR, TIMER_TICKS};

/// Configure the LAPIC timer without starting it.
///
/// The current setup enables the APIC via SIVR, applies the `/16` divider,
/// installs the masked timer vector, masks LINT0/LINT1, and sets the APIC error
/// vector.
pub unsafe fn lapic_timer_configure() {
    // Register the timer IRQ handler in the kernel IRQ registry.
    // This must happen before the LAPIC unmasked the timer vector.
    if let Err(e) = super::irq_registry::register_irq_handler(
        APIC_TIMER_VECTOR,
        "apic-timer",
        super::handlers::irq_timer,
    ) {
        // "already registered" is fine on re-init; anything else is a bug.
        if e != "already registered" {
            panic!("lapic_timer_configure: failed to register IRQ handler: {}", e);
        }
    }

    log_debug!("LAPIC TPR=0");
    // Set Task Priority Register to 0 (allow all interrupt priorities)
    local_apic_write(0x80, 0x00);

    // Enable APIC via SIVR (Spurious Interrupt Vector Register)
    // Bits 0-7: Spurious vector (0xFF)
    // Bit 8: APIC Software Enable (1 = enabled)
    let siv = (local_apic_read(0xF0) & !0xFF) | 0xFF | 0x100;
    local_apic_write(0xF0, siv);

    // Set Timer Divide Configuration to /16
    // This divides the bus frequency by 16 before counting
    log_debug!("LAPIC set divide");
    local_apic_write(0x3E0, 0x3); // 0x3 = divide by 16

    // Program LVT Timer register
    // Bits 0-7: Vector number (0x40)
    // Bit 16: Mask (1 = masked/disabled)
    // Bit 17: Timer mode (0 = one-shot, 1 = periodic)
    log_debug!("LAPIC program LVT timer (masked)");
    let lvt_timer = (APIC_TIMER_VECTOR as u32) | (1 << 16); // Masked one-shot
    local_apic_write(0x320, lvt_timer);

    // Mask external interrupts LINT0 and LINT1 (legacy INT/NMI pins)
    let mut lint0 = local_apic_read(0x350);
    lint0 |= 1 << 16; // Set mask bit
    local_apic_write(0x350, lint0);

    let mut lint1 = local_apic_read(0x360);
    lint1 |= 1 << 16; // Set mask bit
    local_apic_write(0x360, lint1);

    // Set error LVT vector
    let lvt_err = (local_apic_read(0x370) & !0xFF) | (APIC_ERROR_VECTOR as u32);
    local_apic_write(0x370, lvt_err);

    // Clear Error Status Register (ESR) by writing then reading
    local_apic_write(0x280, 0);
    let esr = local_apic_read(0x280);

    // Read and display APIC configuration for debugging
    let id = local_apic_read(0x20);
    let ver = local_apic_read(0x30);
    let sivr = local_apic_read(0xF0);
    let tpr = local_apic_read(0x80);
    let lvt = local_apic_read(0x320);

    log_trace!(
        "LAPIC registers: ID={:#x} VER={:#x} SIVR={:#x} TPR={:#x} LVT={:#x} ESR={:#x}",
        id,
        ver,
        sivr,
        tpr,
        lvt,
        esr
    );
}

/// Start the LAPIC timer in one-shot mode.
pub unsafe fn lapic_timer_start_oneshot(initial_count: u32) {
    // Unmask timer (clear bit 16) for one-shot mode
    // Bit 17 is 0 for one-shot mode
    let mut lvt = local_apic_read(0x320);
    lvt &= !(1 << 16); // Clear mask bit
    local_apic_write(0x320, lvt);

    // Write initial count to start countdown
    local_apic_write(0x380, initial_count);

    // Read current count for debugging
    let cur = local_apic_read(0x390);
    log_debug!("LAPIC current count={:#x}", cur);
}

/// Start the LAPIC timer in periodic mode.
pub unsafe fn lapic_timer_start_periodic(initial_count: u32) {
    // Configure LVT Timer for periodic mode
    let mut lvt = local_apic_read(0x320);
    lvt |= 1 << 17; // Set periodic mode (bit 17)
    lvt &= !(1 << 16); // Clear mask bit (enable timer)
    local_apic_write(0x320, lvt);

    // Write initial count to start periodic countdown
    local_apic_write(0x380, initial_count);
}

/// Mask/stop the LAPIC timer.
pub unsafe fn lapic_timer_mask() {
    // Debug: print CR3 while masking to help diagnose page-table issues.
    use x86_64::registers::control::Cr3;
    let (frame, _f) = Cr3::read();
    let cr3pa = frame.start_address().as_u64();

    log_trace!("LAPIC mask: CR3={:#x}", cr3pa);

    // Read current LVT value
    let val = local_apic_read(0x320);
    log_trace!("LAPIC LVT before mask={:#x}", val);

    // Set mask bit to disable timer
    let new = val | (1 << 16);
    local_apic_write(0x320, new);

    // Verify the write succeeded
    let val2 = local_apic_read(0x320);
    log_trace!("LAPIC LVT after mask={:#x}", val2);

    // Verify CR3 didn't change (helps debug page table issues)
    let (frame2, _f2) = Cr3::read();
    log_trace!(
        "LAPIC mask CR3 after={:#x}",
        frame2.start_address().as_u64()
    );
}

/// Return the LAPIC timer tick counter.
pub fn timer_tick_count() -> u32 {
    TIMER_TICKS.load(Ordering::Relaxed)
}

// install_timer_vector_runtime() removed — the IDT stub for the timer vector
// is installed via IRQ_STUB_TABLE during init_idt(), so no runtime patching
// is needed. The timer handler is registered in the IRQ registry instead.
