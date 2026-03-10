//! Exception and interrupt handlers
//!
//! This module contains all the x86-interrupt handlers for exceptions and hardware interrupts:
//! - **Exception handlers**: DE, BP, UD, GP, PF, DF, NMI, MC
//! - **Hardware interrupts**: APIC timer, serial RX, spurious
//!
//! ## Handler Responsibilities
//!
//! Each handler must:
//! 1. Acknowledge the interrupt (EOI for APIC interrupts)
//! 2. Perform minimal processing
//! 3. Return quickly to avoid blocking other interrupts
//!
//! ## IST (Interrupt Stack Table)
//!
//! Critical handlers use dedicated stacks via IST to prevent stack corruption:
//! - Page Fault (PF): IST stack to handle faults during stack operations
//! - Double Fault (DF): IST stack to recover from stack overflow
//! - NMI: IST stack for non-maskable interrupts
//! - Machine Check (MC): IST stack for hardware errors

use x86_64::registers::control::Cr2;
use x86_64::structures::idt::{InterruptStackFrame, PageFaultErrorCode};

// Import shared state from parent module
use super::{DOUBLE_FAULT_CONTEXT, TIMER_TICKS};

// Import helper functions from parent module
use super::get_handoff_for_timer;
use super::{get_apic_base, write_apic_register};
use super::{out_char_0xe9, print_hex_u64_0xe9, print_str_0xe9}; // # TODO: Remove this and get framebuffer via driver subsystem
use crate::drivers::usb;
use crate::log_trace;
use core::sync::atomic::AtomicBool;

// ============================================================================
// Exception Handlers
// ============================================================================

/// Divide Error (#DE, Vector 0) handler
///
/// Triggered when:
/// - Division by zero
/// - Division overflow (quotient too large for destination)
///
/// # Behavior
/// Prints "DE" and halts the system. This is a fatal error.
pub(super) extern "x86-interrupt" fn handler_de(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("DE\n");
    }
    loop {
        x86_64::instructions::hlt();
    }
}

/// Breakpoint (#BP, Vector 3) handler
///
/// Triggered by INT3 instruction, commonly used by debuggers.
///
/// # Behavior
/// Prints "BP" and halts. In a full debugger implementation, this would
/// pause execution and allow inspection of state.
pub(super) extern "x86-interrupt" fn handler_bp(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("BP\n");
    }
    loop {
        x86_64::instructions::hlt();
    }
}

/// Invalid Opcode (#UD, Vector 6) handler
///
/// Triggered when the CPU encounters an instruction it doesn't recognize.
///
/// Common causes:
/// - Executing data as code
/// - Using unsupported CPU instructions
/// - Corrupted code segment
///
/// # Behavior
/// Prints instruction pointer, code segment, and RFLAGS, then exits QEMU.
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

/// General Protection Fault (#GP, Vector 13) handler
///
/// Triggered by various protection violations:
/// - Accessing a null selector
/// - Executing privileged instructions in user mode
/// - Writing to a read-only segment
/// - Accessing beyond segment limit
///
/// # Behavior
/// Prints error code, RIP, CS, RFLAGS, task register, instruction bytes at RIP,
/// and top of stack for debugging. Then exits QEMU.
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

/// Page Fault (#PF, Vector 14) handler
///
/// Triggered when:
/// - Accessing a non-present page (P=0)
/// - Page protection violation (write to read-only, user accessing kernel, etc.)
/// - Instruction fetch from no-execute page
///
/// # Error Code Bits
/// - Bit 0 (P): 0=non-present page, 1=protection violation
/// - Bit 1 (W/R): 0=read, 1=write
/// - Bit 2 (U/S): 0=supervisor, 1=user mode
/// - Bit 3 (RSVD): 1=reserved bit violation
/// - Bit 4 (I/D): 1=instruction fetch
///
/// # Behavior
/// Prints CR2 (faulting address), error code, RIP, and stack, then halts.
/// Uses IST stack to prevent recursive faults during stack access.
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

/// Double Fault (#DF, Vector 8) handler
///
/// Triggered when an exception occurs while handling another exception.
/// This is a critical error indicating severe system instability.
///
/// Common causes:
/// - Stack overflow (exception during exception handler on corrupted stack)
/// - Missing exception handler
/// - Invalid TSS or IST configuration
///
/// # Behavior
/// Prints saved context (if available) and exits. Uses dedicated IST stack
/// to ensure we can handle the fault even if the main stack is corrupted.
///
/// # Note
/// This handler never returns (`!`) - double faults are unrecoverable.
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

/// Non-Maskable Interrupt (NMI, Vector 2) handler
///
/// NMI cannot be disabled by CLI and is used for critical hardware events:
/// - Memory parity errors
/// - Hardware watchdog timeouts
/// - Debug/profiling interrupts
///
/// # Behavior
/// Prints "NMI", exits QEMU, and halts. Uses IST stack for safety.
pub(super) extern "x86-interrupt" fn handler_nmi(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("NMI\n");
    }
    theseus_shared::qemu_exit_ok!();
    loop {
        x86_64::instructions::hlt();
    }
}

/// Machine Check (#MC, Vector 18) handler
///
/// Triggered by severe hardware errors detected by the CPU:
/// - CPU internal errors
/// - Bus errors
/// - Cache errors
/// - TLB errors
///
/// # Behavior
/// Prints "MC", exits QEMU, and halts. This is a fatal hardware error.
/// Uses IST stack. Never returns.
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

/// APIC Timer interrupt handler (Vector 0x40)
///
/// Called periodically when the Local APIC timer fires.
///
/// # Responsibilities
/// 1. Send EOI (End of Interrupt) to LAPIC
/// 2. Increment global tick counter
/// 3. Update heart animation framebuffer (if handoff available)
///
/// # Performance
/// This runs frequently (e.g., 100Hz), so it must be fast.
/// All work should be minimal and non-blocking.
pub(super) extern "x86-interrupt" fn handler_timer(_stack: InterruptStackFrame) {
    // Acknowledge LAPIC EOI first to avoid stuck-in-service
    unsafe {
        let apic_base = get_apic_base();
        write_apic_register(apic_base, 0xB0, 0); // Write to EOI register
    }

    // Record the tick atomically
    TIMER_TICKS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

    // Update heart animation if handoff is available
    // # TODO: Update this to get framebuffer via driver subsystem
    unsafe {
        if let Some(handoff) = get_handoff_for_timer() {
            crate::framebuffer::update_heart_animation(handoff);
        }
    }
}

/// Serial RX interrupt handler (Vector 0x41)
///
/// Called when data arrives on the COM1 serial port (IRQ 4).
///
/// # Responsibilities
/// 1. Delegate to the driver manager to handle the IRQ
/// 2. Send EOI to LAPIC
/// 3. Log if no driver handled the interrupt
///
/// # Note
/// The actual serial processing is done by the serial driver,
/// not directly in this handler.
pub(super) extern "x86-interrupt" fn handler_serial_rx(_stack: InterruptStackFrame) {
    let mut handled = false;

    // Ask the driver manager to handle the IRQ
    if let Some(irq) = crate::drivers::serial::current_irq_number() {
        let mut mgr = crate::drivers::manager::driver_manager().lock();
        handled = mgr.handle_irq(irq);
    }

    // Send EOI to LAPIC
    unsafe {
        let apic_base = get_apic_base();
        write_apic_register(apic_base, 0xB0, 0);
    }

    // Log if interrupt was unhandled (debugging)
    if !handled {
        unsafe {
            print_str_0xe9("[serial] irq but no handler\n");
        }
    }
}

/// xHCI MSI interrupt handler (Vector 0x50)
///
/// This handler services message-signalled interrupts delivered by xHCI host controllers.
/// It clears the LAPIC in-service state and forwards processing to the USB driver so
/// controller completions are drained promptly.
pub(super) extern "x86-interrupt" fn handler_usb_xhci(_stack: InterruptStackFrame) {
    unsafe {
        let apic_base = get_apic_base();
        write_apic_register(apic_base, 0xB0, 0);
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

/// Spurious interrupt handler (Vector 0xFF and APIC error vector 0xFE)
///
/// Spurious interrupts can occur due to:
/// - APIC hardware race conditions
/// - Interrupt masking timing
/// - External interrupt controller issues
///
/// # Behavior
/// Logs the spurious interrupt and returns immediately.
/// Does NOT send EOI (spurious interrupts don't require acknowledgment).
pub(super) extern "x86-interrupt" fn handler_spurious(_stack: InterruptStackFrame) {
    // Log spurious interrupt entry for debugging nested/extra interrupts
    unsafe {
        print_str_0xe9("[INT] spurious\n");
    }
    // Return to interrupted context without EOI
}
