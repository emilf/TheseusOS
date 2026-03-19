//! Module: interrupts::handlers
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A1:-The-kernel-is-x86_64-no_std-code-using-the-x86-interrupt-ABI
//! - docs/axioms/arch-x86_64.md#A2:-GDT/TSS-setup-provides-dedicated-IST-stacks-for-critical-faults
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//! - docs/axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status
//!
//! INVARIANTS:
//! - This module contains the concrete exception and hardware-interrupt handlers installed into the IDT.
//! - Critical fault handlers rely on dedicated IST stacks configured by the GDT/TSS path.
//! - Hardware interrupt handlers are expected to perform bounded work and cooperate with the APIC/monitor/driver subsystems.
//!
//! SAFETY:
//! - Handler code executes under interrupt/exception constraints and must not casually assume normal runtime context or allocator availability.
//! - Debug-printing inside fatal handlers is best-effort diagnostic behavior, not a substitute for coherent interrupt-state management.
//! - EOI behavior and side effects must stay aligned with APIC ownership rules or handler success/failure reporting becomes misleading.
//!
//! PROGRESS:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/observability.md
//!
//! Exception and hardware-interrupt handlers.
//!
//! This module contains the concrete `extern "x86-interrupt"` handlers installed
//! into the current kernel IDT.

use x86_64::registers::control::Cr2;
use x86_64::structures::idt::{InterruptStackFrame, PageFaultErrorCode};

// Import shared state from parent module
use super::{DOUBLE_FAULT_CONTEXT, TIMER_TICKS};

// Import helper functions from parent module
use super::get_handoff_for_timer;
use super::local_apic_eoi;
use super::{out_char_0xe9, print_hex_u64_0xe9, print_str_0xe9};
use crate::drivers::usb;
use crate::log_trace;
use core::sync::atomic::AtomicBool;

// ============================================================================
// Exception Handlers
// ============================================================================

/// Divide-error (`#DE`, vector 0) handler.
pub(super) extern "x86-interrupt" fn handler_de(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("DE\n");
    }
    loop {
        x86_64::instructions::hlt();
    }
}

/// Breakpoint (`#BP`, vector 3) handler.
pub(super) extern "x86-interrupt" fn handler_bp(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("BP\n");
    }
    loop {
        x86_64::instructions::hlt();
    }
}

/// Invalid-opcode (`#UD`, vector 6) handler.
pub(super) extern "x86-interrupt" fn handler_ud(stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("UD RIP=");
        print_hex_u64_0xe9(stack.instruction_pointer.as_u64());
        print_str_0xe9(" CS=");
        print_hex_u64_0xe9(stack.code_segment as u64);
        print_str_0xe9(" RFLAGS=");
        print_hex_u64_0xe9(stack.cpu_flags);
        out_char_0xe9(b'\n');
        theseus_shared::qemu_exit_ok!();
    }
    loop {
        x86_64::instructions::hlt();
    }
}

/// General-protection-fault (`#GP`, vector 13) handler.
pub(super) extern "x86-interrupt" fn handler_gp(stack: InterruptStackFrame, code: u64) {
    unsafe {
        print_str_0xe9("GP EC=");
        print_hex_u64_0xe9(code);
        print_str_0xe9(" RIP=");
        print_hex_u64_0xe9(stack.instruction_pointer.as_u64());
        print_str_0xe9(" CS=");
        print_hex_u64_0xe9(stack.code_segment as u64);
        print_str_0xe9(" RFLAGS=");
        print_hex_u64_0xe9(stack.cpu_flags);
        print_str_0xe9(" TR=");

        // Read task register to help debug TSS-related issues
        let tr: u16;
        core::arch::asm!("str {0:x}", out(reg) tr, options(nomem, nostack, preserves_flags));
        print_hex_u64_0xe9(tr as u64);

        // Dump 8 bytes at RIP to see what instruction caused the fault
        print_str_0xe9(" INS=");
        let rip = stack.instruction_pointer.as_u64();
        let ins = core::ptr::read_volatile(rip as *const u64);
        print_hex_u64_0xe9(ins);

        // Dump top of stack for context
        print_str_0xe9(" STK:");
        let rsp = stack.stack_pointer.as_u64();
        for i in 0..4u64 {
            let val = core::ptr::read_volatile((rsp + i * 8) as *const u64);
            out_char_0xe9(b' ');
            print_hex_u64_0xe9(val);
        }
        out_char_0xe9(b'\n');
        theseus_shared::qemu_exit_ok!();
    }
    loop {
        x86_64::instructions::hlt();
    }
}

/// Page-fault (`#PF`, vector 14) handler.
///
/// This prints the fault address, error code, and a small stack snapshot before
/// halting on the dedicated IST stack.
pub(super) extern "x86-interrupt" fn handler_pf(
    stack: InterruptStackFrame,
    code: PageFaultErrorCode,
) {
    let cr2 = Cr2::read().as_u64(); // CR2 holds the faulting virtual address
    let ec = code.bits() as u64;

    unsafe {
        print_str_0xe9("PF EC=");
        print_hex_u64_0xe9(ec);
        print_str_0xe9(" CR2=");
        print_hex_u64_0xe9(cr2);
        print_str_0xe9(" RIP=");
        print_hex_u64_0xe9(stack.instruction_pointer.as_u64());
        print_str_0xe9(" CS=");
        print_hex_u64_0xe9(stack.code_segment as u64);
        print_str_0xe9(" RFLAGS=");
        print_hex_u64_0xe9(stack.cpu_flags);
        print_str_0xe9(" RSP=");
        let rsp = stack.stack_pointer.as_u64();
        print_hex_u64_0xe9(rsp);

        // Dump top 6 stack entries for debugging
        print_str_0xe9(" STK:");
        for i in 0..6u64 {
            let val = core::ptr::read_volatile((rsp + i * 8) as *const u64);
            out_char_0xe9(b' ');
            print_hex_u64_0xe9(val);
        }
        out_char_0xe9(b'\n');
    }
    loop {
        x86_64::instructions::hlt();
    }
}

/// Double-fault (`#DF`, vector 8) handler.
///
/// This prints the saved pre-fault context when available and then exits.
pub(super) extern "x86-interrupt" fn handler_df(_stack: InterruptStackFrame, _code: u64) -> ! {
    unsafe {
        print_str_0xe9("DF at");

        // Try to print saved context if it was captured before the fault
        if let Some(ctx) = DOUBLE_FAULT_CONTEXT {
            print_str_0xe9(" RIP=");
            print_hex_u64_0xe9(ctx.rip);
            print_str_0xe9(" CR2=");
            print_hex_u64_0xe9(ctx.cr2);
            print_str_0xe9(" RSP=");
            print_hex_u64_0xe9(ctx.rsp);
            print_str_0xe9(" STK:");
            for word in ctx.stack.iter() {
                out_char_0xe9(b' ');
                print_hex_u64_0xe9(*word);
            }
            out_char_0xe9(b'\n');
        } else {
            print_str_0xe9(" (no context)\n");
        }
    }
    theseus_shared::qemu_exit_ok!();
    loop {
        x86_64::instructions::hlt();
    }
}

/// Non-maskable interrupt (NMI, vector 2) handler.
pub(super) extern "x86-interrupt" fn handler_nmi(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("NMI\n");
    }
    theseus_shared::qemu_exit_ok!();
    loop {
        x86_64::instructions::hlt();
    }
}

/// Machine-check (`#MC`, vector 18) handler.
pub(super) extern "x86-interrupt" fn handler_mc(_stack: InterruptStackFrame) -> ! {
    unsafe {
        print_str_0xe9("MC\n");
    }
    theseus_shared::qemu_exit_ok!();
    loop {
        x86_64::instructions::hlt();
    }
}

// ============================================================================
// Hardware Interrupt Handlers
// ============================================================================

/// APIC timer interrupt handler (vector `0x40`).
pub(super) extern "x86-interrupt" fn handler_timer(_stack: InterruptStackFrame) {
    // Acknowledge LAPIC EOI first to avoid stuck-in-service.
    unsafe {
        local_apic_eoi();
    }

    // Record the tick atomically
    TIMER_TICKS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

    // Update the simple framebuffer animation if the transitional handoff pointer is available.
    unsafe {
        if let Some(handoff) = get_handoff_for_timer() {
            crate::framebuffer::update_heart_animation(handoff);
        }
    }
}

/// Serial RX interrupt handler (vector `0x41`).
pub(super) extern "x86-interrupt" fn handler_serial_rx(_stack: InterruptStackFrame) {
    let mut handled = false;

    // Ask the driver manager to handle the IRQ
    if let Some(irq) = crate::drivers::serial::current_irq_number() {
        let mut mgr = crate::drivers::manager::driver_manager().lock();
        handled = mgr.handle_irq(irq);
    }

    // Send EOI to LAPIC
    unsafe {
        local_apic_eoi();
    }

    // Log if interrupt was unhandled (debugging)
    if !handled {
        unsafe {
            print_str_0xe9("[serial] irq but no handler\n");
        }
    }
}

/// xHCI MSI interrupt handler (vector `0x50`).
pub(super) extern "x86-interrupt" fn handler_usb_xhci(_stack: InterruptStackFrame) {
    unsafe {
        local_apic_eoi();
    }

    static FIRST_XHCI_MSI: AtomicBool = AtomicBool::new(true);
    if FIRST_XHCI_MSI.swap(false, core::sync::atomic::Ordering::AcqRel) {
        // This is intentionally INFO-level and one-shot so it's visible even
        // when TRACE output is noisy or filtered.
        crate::log_info!("xHCI MSI handler invoked (first delivery)");
    } else {
        log_trace!("xHCI MSI handler invoked");
    }
    usb::service_runtime_interrupt();
}

/// Spurious interrupt handler (vector `0xFF` and APIC error vector `0xFE`).
pub(super) extern "x86-interrupt" fn handler_spurious(_stack: InterruptStackFrame) {
    // Log spurious interrupt entry for debugging nested/extra interrupts
    unsafe {
        print_str_0xe9("[INT] spurious\n");
    }
    // Return to interrupted context without EOI
}
