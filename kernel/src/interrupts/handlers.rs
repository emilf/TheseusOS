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
// ============================================================================
// IRQ handler functions — registered with IRQ_REGISTRY, called via stubs
// ============================================================================
//
// These are plain `fn()` functions, not `extern "x86-interrupt"`. They are
// called by irq_dispatch_common after the stub has already handled the
// interrupt-frame mechanics. Each handler is responsible for its own EOI.

/// APIC timer IRQ handler (vector 0x40). Registered by `lapic_timer_configure`.
pub(crate) fn irq_timer() {
    // Acknowledge LAPIC EOI first to avoid stuck-in-service.
    unsafe { local_apic_eoi(); }

    TIMER_TICKS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    super::calibration::tick();

    unsafe {
        if let Some(handoff) = get_handoff_for_timer() {
            crate::framebuffer::update_heart_animation(handoff);
        }
    }
}

/// Serial RX IRQ handler (vector 0x41). Registered by `init_serial`.
pub(crate) fn irq_serial_rx() {
    let mut handled = false;

    if let Some(irq) = crate::drivers::serial::current_irq_number() {
        let mut mgr = crate::drivers::manager::driver_manager().lock();
        handled = mgr.handle_irq(irq);
    }

    unsafe { local_apic_eoi(); }

    if !handled {
        unsafe { print_str_0xe9("[serial] irq but no handler\n"); }
    }
}

/// xHCI MSI IRQ handler (vector 0x50). Registered by `usb::init`.
pub(crate) fn irq_usb_xhci() {
    unsafe { local_apic_eoi(); }

    static FIRST_XHCI_MSI: AtomicBool = AtomicBool::new(true);
    if FIRST_XHCI_MSI.swap(false, core::sync::atomic::Ordering::AcqRel) {
        crate::log_info!("xHCI MSI handler invoked (first delivery)");
    } else {
        log_trace!("xHCI MSI handler invoked");
    }
    usb::service_runtime_interrupt();
}

// ============================================================================
// Per-vector interrupt stubs
// ============================================================================
//
// Design rationale (see docs/plans/interrupts-and-platform.md):
//
// The x86-interrupt ABI gives us no way to pass the vector number to a shared
// Rust handler — the CPU only pushes SS/RSP/RFLAGS/CS/RIP (and optionally an
// error code) before jumping to the IDT entry. There is no "which vector was
// this" slot on the interrupt frame.
//
// The canonical OS solution (Linux, FreeBSD, seL4, most hobby OSes) is a table
// of tiny assembly stubs — one per vector — that each push a literal vector
// byte, then tail-call a common C/Rust dispatcher. The vector is baked into the
// code at compile time, so the dispatcher receives it in a register with no
// shared-state access.
//
// SMP notes: each stub is pure code (no data). The common dispatcher reads the
// IRQ registry, which is a global Mutex — that is already SMP-safe.
// Per-CPU interrupt stacks (IST) are a separate concern handled by the GDT/TSS
// path and are unaffected by this design.
//
// The stubs live in `global_asm!` because Rust currently has no stable way to
// generate per-vector named extern "x86-interrupt" functions using macros
// (identifiers cannot be concatenated in declarative macros). A `global_asm!`
// loop is the idiomatic Rust/LLVM equivalent of the Linux `.irq_entries_start`
// table.
//
// The stub table `IRQ_STUB_TABLE` maps vector → stub address so the IDT
// setup loop can install each stub with `set_handler_addr`.

use core::arch::global_asm;

/// Common Rust-side IRQ dispatcher called from all per-vector stubs.
///
/// Receives the vector number in RDI (System V AMD64 ABI first integer
/// argument). The stub has already saved no caller-saved registers —
/// `extern "C"` calling convention means the compiler handles that.
///
/// # Safety
/// Must only be called from the assembly stubs below, inside an interrupt
/// context with interrupts disabled by the CPU.
#[no_mangle]
pub extern "C" fn irq_dispatch_common(vector: u8) {
    unsafe {
        super::irq_registry::dispatch_irq(vector);
    }
}

// Generate one stub per IRQ vector (0x20 – 0xFD).
// Excluded: 0x40 (timer), 0x41 (serial), 0x50 (xHCI), 0xFE (APIC error),
//           0xFF (spurious) — those have dedicated Rust handlers.
// Vectors 0x00–0x1F are CPU exceptions handled separately.
//
// Each stub:
//   1. Aligns to 16 bytes (improves icache packing; matches Linux IDT_ALIGN).
//   2. `push $vector` — encodes the vector as an immediate; becomes the first
//      argument (RDI) for irq_dispatch_common via the `mov rdi, [rsp]` below.
//   3. Jumps to the shared thunk `irq_stub_common` which loads RDI and calls
//      irq_dispatch_common, then does IRETQ.
//
// irq_stub_common is a naked function (no prologue/epilogue) that:
//   - pops the vector off the stack into RDI
//   - calls irq_dispatch_common (which handles EOI internally via dispatch_irq)
//   - executes IRETQ to return to interrupted code
//
// Stack state at irq_stub_common entry (after push $vector by stub):
//   [RSP+0]  vector byte (pushed by stub)
//   [RSP+8]  RIP  (pushed by CPU on interrupt)
//   [RSP+16] CS
//   [RSP+24] RFLAGS
//   [RSP+32] RSP (only present if privilege change)
//   [RSP+40] SS  (only present if privilege change)
//
// We're kernel-only for now, so no privilege change → no RSP/SS on stack.

global_asm!(
    // Shared thunk: vector byte is on top of stack.
    ".global irq_stub_common",
    "irq_stub_common:",
    "    pop  rdi",          // vector → first C argument
    "    call irq_dispatch_common",
    "    iretq",

    // Macro: emit one 16-byte-aligned stub.
    // Using .set to define vector values avoids the assembler complaining
    // about immediate sizes.
    ".macro irq_stub vec",
    "    .align 16",
    "    .global irq_stub_\\vec",
    "    irq_stub_\\vec:",
    "    push \\vec",
    "    jmp  irq_stub_common",
    ".endm",

    // Emit stubs 0x20–0xFF (all hardware IRQ vectors)
    // Exceptions 0x00-0x1F stay as dedicated extern "x86-interrupt" handlers.
    // Spurious (0xFF) and APIC error (0xFE) are special: spurious must NOT
    // send EOI per Intel SDM; they keep dedicated Rust handlers.
    // Everything else — including timer (0x40), serial (0x41), xHCI (0x50)
    // — goes through the stub system and is registered in the IRQ registry.
    "irq_stub 0x20", "irq_stub 0x21", "irq_stub 0x22", "irq_stub 0x23",
    "irq_stub 0x24", "irq_stub 0x25", "irq_stub 0x26", "irq_stub 0x27",
    "irq_stub 0x28", "irq_stub 0x29", "irq_stub 0x2a", "irq_stub 0x2b",
    "irq_stub 0x2c", "irq_stub 0x2d", "irq_stub 0x2e", "irq_stub 0x2f",
    "irq_stub 0x30", "irq_stub 0x31", "irq_stub 0x32", "irq_stub 0x33",
    "irq_stub 0x34", "irq_stub 0x35", "irq_stub 0x36", "irq_stub 0x37",
    "irq_stub 0x38", "irq_stub 0x39", "irq_stub 0x3a", "irq_stub 0x3b",
    "irq_stub 0x3c", "irq_stub 0x3d", "irq_stub 0x3e", "irq_stub 0x3f",
    // 0x40 (timer), 0x41 (serial), 0x50 (xHCI) now go through the stub
    // system too — their handlers are registered in the IRQ registry at init.
    "irq_stub 0x40", "irq_stub 0x41", "irq_stub 0x42", "irq_stub 0x43", "irq_stub 0x44",
    "irq_stub 0x45", "irq_stub 0x46", "irq_stub 0x47", "irq_stub 0x48",
    "irq_stub 0x49", "irq_stub 0x4a", "irq_stub 0x4b", "irq_stub 0x4c",
    "irq_stub 0x4d", "irq_stub 0x4e", "irq_stub 0x4f",
    "irq_stub 0x50", "irq_stub 0x51", "irq_stub 0x52", "irq_stub 0x53", "irq_stub 0x54",
    "irq_stub 0x55", "irq_stub 0x56", "irq_stub 0x57", "irq_stub 0x58",
    "irq_stub 0x59", "irq_stub 0x5a", "irq_stub 0x5b", "irq_stub 0x5c",
    "irq_stub 0x5d", "irq_stub 0x5e", "irq_stub 0x5f",
    "irq_stub 0x60", "irq_stub 0x61", "irq_stub 0x62", "irq_stub 0x63",
    "irq_stub 0x64", "irq_stub 0x65", "irq_stub 0x66", "irq_stub 0x67",
    "irq_stub 0x68", "irq_stub 0x69", "irq_stub 0x6a", "irq_stub 0x6b",
    "irq_stub 0x6c", "irq_stub 0x6d", "irq_stub 0x6e", "irq_stub 0x6f",
    "irq_stub 0x70", "irq_stub 0x71", "irq_stub 0x72", "irq_stub 0x73",
    "irq_stub 0x74", "irq_stub 0x75", "irq_stub 0x76", "irq_stub 0x77",
    "irq_stub 0x78", "irq_stub 0x79", "irq_stub 0x7a", "irq_stub 0x7b",
    "irq_stub 0x7c", "irq_stub 0x7d", "irq_stub 0x7e", "irq_stub 0x7f",
    "irq_stub 0x80", "irq_stub 0x81", "irq_stub 0x82", "irq_stub 0x83",
    "irq_stub 0x84", "irq_stub 0x85", "irq_stub 0x86", "irq_stub 0x87",
    "irq_stub 0x88", "irq_stub 0x89", "irq_stub 0x8a", "irq_stub 0x8b",
    "irq_stub 0x8c", "irq_stub 0x8d", "irq_stub 0x8e", "irq_stub 0x8f",
    "irq_stub 0x90", "irq_stub 0x91", "irq_stub 0x92", "irq_stub 0x93",
    "irq_stub 0x94", "irq_stub 0x95", "irq_stub 0x96", "irq_stub 0x97",
    "irq_stub 0x98", "irq_stub 0x99", "irq_stub 0x9a", "irq_stub 0x9b",
    "irq_stub 0x9c", "irq_stub 0x9d", "irq_stub 0x9e", "irq_stub 0x9f",
    "irq_stub 0xa0", "irq_stub 0xa1", "irq_stub 0xa2", "irq_stub 0xa3",
    "irq_stub 0xa4", "irq_stub 0xa5", "irq_stub 0xa6", "irq_stub 0xa7",
    "irq_stub 0xa8", "irq_stub 0xa9", "irq_stub 0xaa", "irq_stub 0xab",
    "irq_stub 0xac", "irq_stub 0xad", "irq_stub 0xae", "irq_stub 0xaf",
    "irq_stub 0xb0", "irq_stub 0xb1", "irq_stub 0xb2", "irq_stub 0xb3",
    "irq_stub 0xb4", "irq_stub 0xb5", "irq_stub 0xb6", "irq_stub 0xb7",
    "irq_stub 0xb8", "irq_stub 0xb9", "irq_stub 0xba", "irq_stub 0xbb",
    "irq_stub 0xbc", "irq_stub 0xbd", "irq_stub 0xbe", "irq_stub 0xbf",
    "irq_stub 0xc0", "irq_stub 0xc1", "irq_stub 0xc2", "irq_stub 0xc3",
    "irq_stub 0xc4", "irq_stub 0xc5", "irq_stub 0xc6", "irq_stub 0xc7",
    "irq_stub 0xc8", "irq_stub 0xc9", "irq_stub 0xca", "irq_stub 0xcb",
    "irq_stub 0xcc", "irq_stub 0xcd", "irq_stub 0xce", "irq_stub 0xcf",
    "irq_stub 0xd0", "irq_stub 0xd1", "irq_stub 0xd2", "irq_stub 0xd3",
    "irq_stub 0xd4", "irq_stub 0xd5", "irq_stub 0xd6", "irq_stub 0xd7",
    "irq_stub 0xd8", "irq_stub 0xd9", "irq_stub 0xda", "irq_stub 0xdb",
    "irq_stub 0xdc", "irq_stub 0xdd", "irq_stub 0xde", "irq_stub 0xdf",
    "irq_stub 0xe0", "irq_stub 0xe1", "irq_stub 0xe2", "irq_stub 0xe3",
    "irq_stub 0xe4", "irq_stub 0xe5", "irq_stub 0xe6", "irq_stub 0xe7",
    "irq_stub 0xe8", "irq_stub 0xe9", "irq_stub 0xea", "irq_stub 0xeb",
    "irq_stub 0xec", "irq_stub 0xed", "irq_stub 0xee", "irq_stub 0xef",
    "irq_stub 0xf0", "irq_stub 0xf1", "irq_stub 0xf2", "irq_stub 0xf3",
    "irq_stub 0xf4", "irq_stub 0xf5", "irq_stub 0xf6", "irq_stub 0xf7",
    "irq_stub 0xf8", "irq_stub 0xf9", "irq_stub 0xfa", "irq_stub 0xfb",
    "irq_stub 0xfc", "irq_stub 0xfd",
    // 0xfe = APIC error, 0xff = spurious — dedicated Rust handlers
);

// Declare the stub symbols so Rust can take their addresses.
extern "C" {
    fn irq_stub_0x20(); fn irq_stub_0x21(); fn irq_stub_0x22(); fn irq_stub_0x23();
    fn irq_stub_0x24(); fn irq_stub_0x25(); fn irq_stub_0x26(); fn irq_stub_0x27();
    fn irq_stub_0x28(); fn irq_stub_0x29(); fn irq_stub_0x2a(); fn irq_stub_0x2b();
    fn irq_stub_0x2c(); fn irq_stub_0x2d(); fn irq_stub_0x2e(); fn irq_stub_0x2f();
    fn irq_stub_0x30(); fn irq_stub_0x31(); fn irq_stub_0x32(); fn irq_stub_0x33();
    fn irq_stub_0x34(); fn irq_stub_0x35(); fn irq_stub_0x36(); fn irq_stub_0x37();
    fn irq_stub_0x38(); fn irq_stub_0x39(); fn irq_stub_0x3a(); fn irq_stub_0x3b();
    fn irq_stub_0x3c(); fn irq_stub_0x3d(); fn irq_stub_0x3e(); fn irq_stub_0x3f();
    fn irq_stub_0x40(); fn irq_stub_0x41(); fn irq_stub_0x42(); fn irq_stub_0x43(); fn irq_stub_0x44();
    fn irq_stub_0x45(); fn irq_stub_0x46(); fn irq_stub_0x47(); fn irq_stub_0x48();
    fn irq_stub_0x49(); fn irq_stub_0x4a(); fn irq_stub_0x4b(); fn irq_stub_0x4c();
    fn irq_stub_0x4d(); fn irq_stub_0x4e(); fn irq_stub_0x4f();
    fn irq_stub_0x50(); fn irq_stub_0x51(); fn irq_stub_0x52(); fn irq_stub_0x53(); fn irq_stub_0x54();
    fn irq_stub_0x55(); fn irq_stub_0x56(); fn irq_stub_0x57(); fn irq_stub_0x58();
    fn irq_stub_0x59(); fn irq_stub_0x5a(); fn irq_stub_0x5b(); fn irq_stub_0x5c();
    fn irq_stub_0x5d(); fn irq_stub_0x5e(); fn irq_stub_0x5f();
    fn irq_stub_0x60(); fn irq_stub_0x61(); fn irq_stub_0x62(); fn irq_stub_0x63();
    fn irq_stub_0x64(); fn irq_stub_0x65(); fn irq_stub_0x66(); fn irq_stub_0x67();
    fn irq_stub_0x68(); fn irq_stub_0x69(); fn irq_stub_0x6a(); fn irq_stub_0x6b();
    fn irq_stub_0x6c(); fn irq_stub_0x6d(); fn irq_stub_0x6e(); fn irq_stub_0x6f();
    fn irq_stub_0x70(); fn irq_stub_0x71(); fn irq_stub_0x72(); fn irq_stub_0x73();
    fn irq_stub_0x74(); fn irq_stub_0x75(); fn irq_stub_0x76(); fn irq_stub_0x77();
    fn irq_stub_0x78(); fn irq_stub_0x79(); fn irq_stub_0x7a(); fn irq_stub_0x7b();
    fn irq_stub_0x7c(); fn irq_stub_0x7d(); fn irq_stub_0x7e(); fn irq_stub_0x7f();
    fn irq_stub_0x80(); fn irq_stub_0x81(); fn irq_stub_0x82(); fn irq_stub_0x83();
    fn irq_stub_0x84(); fn irq_stub_0x85(); fn irq_stub_0x86(); fn irq_stub_0x87();
    fn irq_stub_0x88(); fn irq_stub_0x89(); fn irq_stub_0x8a(); fn irq_stub_0x8b();
    fn irq_stub_0x8c(); fn irq_stub_0x8d(); fn irq_stub_0x8e(); fn irq_stub_0x8f();
    fn irq_stub_0x90(); fn irq_stub_0x91(); fn irq_stub_0x92(); fn irq_stub_0x93();
    fn irq_stub_0x94(); fn irq_stub_0x95(); fn irq_stub_0x96(); fn irq_stub_0x97();
    fn irq_stub_0x98(); fn irq_stub_0x99(); fn irq_stub_0x9a(); fn irq_stub_0x9b();
    fn irq_stub_0x9c(); fn irq_stub_0x9d(); fn irq_stub_0x9e(); fn irq_stub_0x9f();
    fn irq_stub_0xa0(); fn irq_stub_0xa1(); fn irq_stub_0xa2(); fn irq_stub_0xa3();
    fn irq_stub_0xa4(); fn irq_stub_0xa5(); fn irq_stub_0xa6(); fn irq_stub_0xa7();
    fn irq_stub_0xa8(); fn irq_stub_0xa9(); fn irq_stub_0xaa(); fn irq_stub_0xab();
    fn irq_stub_0xac(); fn irq_stub_0xad(); fn irq_stub_0xae(); fn irq_stub_0xaf();
    fn irq_stub_0xb0(); fn irq_stub_0xb1(); fn irq_stub_0xb2(); fn irq_stub_0xb3();
    fn irq_stub_0xb4(); fn irq_stub_0xb5(); fn irq_stub_0xb6(); fn irq_stub_0xb7();
    fn irq_stub_0xb8(); fn irq_stub_0xb9(); fn irq_stub_0xba(); fn irq_stub_0xbb();
    fn irq_stub_0xbc(); fn irq_stub_0xbd(); fn irq_stub_0xbe(); fn irq_stub_0xbf();
    fn irq_stub_0xc0(); fn irq_stub_0xc1(); fn irq_stub_0xc2(); fn irq_stub_0xc3();
    fn irq_stub_0xc4(); fn irq_stub_0xc5(); fn irq_stub_0xc6(); fn irq_stub_0xc7();
    fn irq_stub_0xc8(); fn irq_stub_0xc9(); fn irq_stub_0xca(); fn irq_stub_0xcb();
    fn irq_stub_0xcc(); fn irq_stub_0xcd(); fn irq_stub_0xce(); fn irq_stub_0xcf();
    fn irq_stub_0xd0(); fn irq_stub_0xd1(); fn irq_stub_0xd2(); fn irq_stub_0xd3();
    fn irq_stub_0xd4(); fn irq_stub_0xd5(); fn irq_stub_0xd6(); fn irq_stub_0xd7();
    fn irq_stub_0xd8(); fn irq_stub_0xd9(); fn irq_stub_0xda(); fn irq_stub_0xdb();
    fn irq_stub_0xdc(); fn irq_stub_0xdd(); fn irq_stub_0xde(); fn irq_stub_0xdf();
    fn irq_stub_0xe0(); fn irq_stub_0xe1(); fn irq_stub_0xe2(); fn irq_stub_0xe3();
    fn irq_stub_0xe4(); fn irq_stub_0xe5(); fn irq_stub_0xe6(); fn irq_stub_0xe7();
    fn irq_stub_0xe8(); fn irq_stub_0xe9(); fn irq_stub_0xea(); fn irq_stub_0xeb();
    fn irq_stub_0xec(); fn irq_stub_0xed(); fn irq_stub_0xee(); fn irq_stub_0xef();
    fn irq_stub_0xf0(); fn irq_stub_0xf1(); fn irq_stub_0xf2(); fn irq_stub_0xf3();
    fn irq_stub_0xf4(); fn irq_stub_0xf5(); fn irq_stub_0xf6(); fn irq_stub_0xf7();
    fn irq_stub_0xf8(); fn irq_stub_0xf9(); fn irq_stub_0xfa(); fn irq_stub_0xfb();
    fn irq_stub_0xfc(); fn irq_stub_0xfd();
}

/// Table mapping vector number → stub address.
/// Entry is `None` for vectors that have dedicated Rust handlers (timer, serial,
/// xHCI, APIC error, spurious). The IDT setup loop uses this to install each stub
/// with `set_handler_addr`.
pub(super) static IRQ_STUB_TABLE: [Option<unsafe extern "C" fn()>; 256] = {
    let mut table: [Option<unsafe extern "C" fn()>; 256] = [None; 256];
    // Vectors 0x00–0x1F are CPU exceptions — not general IRQs.
    table[0x20] = Some(irq_stub_0x20); table[0x21] = Some(irq_stub_0x21);
    table[0x22] = Some(irq_stub_0x22); table[0x23] = Some(irq_stub_0x23);
    table[0x24] = Some(irq_stub_0x24); table[0x25] = Some(irq_stub_0x25);
    table[0x26] = Some(irq_stub_0x26); table[0x27] = Some(irq_stub_0x27);
    table[0x28] = Some(irq_stub_0x28); table[0x29] = Some(irq_stub_0x29);
    table[0x2a] = Some(irq_stub_0x2a); table[0x2b] = Some(irq_stub_0x2b);
    table[0x2c] = Some(irq_stub_0x2c); table[0x2d] = Some(irq_stub_0x2d);
    table[0x2e] = Some(irq_stub_0x2e); table[0x2f] = Some(irq_stub_0x2f);
    table[0x30] = Some(irq_stub_0x30); table[0x31] = Some(irq_stub_0x31);
    table[0x32] = Some(irq_stub_0x32); table[0x33] = Some(irq_stub_0x33);
    table[0x34] = Some(irq_stub_0x34); table[0x35] = Some(irq_stub_0x35);
    table[0x36] = Some(irq_stub_0x36); table[0x37] = Some(irq_stub_0x37);
    table[0x38] = Some(irq_stub_0x38); table[0x39] = Some(irq_stub_0x39);
    table[0x3a] = Some(irq_stub_0x3a); table[0x3b] = Some(irq_stub_0x3b);
    table[0x3c] = Some(irq_stub_0x3c); table[0x3d] = Some(irq_stub_0x3d);
    table[0x3e] = Some(irq_stub_0x3e); table[0x3f] = Some(irq_stub_0x3f);
    table[0x40] = Some(irq_stub_0x40); table[0x41] = Some(irq_stub_0x41);
    table[0x42] = Some(irq_stub_0x42); table[0x43] = Some(irq_stub_0x43);
    table[0x44] = Some(irq_stub_0x44); table[0x45] = Some(irq_stub_0x45);
    table[0x46] = Some(irq_stub_0x46); table[0x47] = Some(irq_stub_0x47);
    table[0x48] = Some(irq_stub_0x48); table[0x49] = Some(irq_stub_0x49);
    table[0x4a] = Some(irq_stub_0x4a); table[0x4b] = Some(irq_stub_0x4b);
    table[0x4c] = Some(irq_stub_0x4c); table[0x4d] = Some(irq_stub_0x4d);
    table[0x4e] = Some(irq_stub_0x4e); table[0x4f] = Some(irq_stub_0x4f);
    table[0x50] = Some(irq_stub_0x50); table[0x51] = Some(irq_stub_0x51); table[0x52] = Some(irq_stub_0x52);
    table[0x53] = Some(irq_stub_0x53); table[0x54] = Some(irq_stub_0x54);
    table[0x55] = Some(irq_stub_0x55); table[0x56] = Some(irq_stub_0x56);
    table[0x57] = Some(irq_stub_0x57); table[0x58] = Some(irq_stub_0x58);
    table[0x59] = Some(irq_stub_0x59); table[0x5a] = Some(irq_stub_0x5a);
    table[0x5b] = Some(irq_stub_0x5b); table[0x5c] = Some(irq_stub_0x5c);
    table[0x5d] = Some(irq_stub_0x5d); table[0x5e] = Some(irq_stub_0x5e);
    table[0x5f] = Some(irq_stub_0x5f);
    table[0x60] = Some(irq_stub_0x60); table[0x61] = Some(irq_stub_0x61);
    table[0x62] = Some(irq_stub_0x62); table[0x63] = Some(irq_stub_0x63);
    table[0x64] = Some(irq_stub_0x64); table[0x65] = Some(irq_stub_0x65);
    table[0x66] = Some(irq_stub_0x66); table[0x67] = Some(irq_stub_0x67);
    table[0x68] = Some(irq_stub_0x68); table[0x69] = Some(irq_stub_0x69);
    table[0x6a] = Some(irq_stub_0x6a); table[0x6b] = Some(irq_stub_0x6b);
    table[0x6c] = Some(irq_stub_0x6c); table[0x6d] = Some(irq_stub_0x6d);
    table[0x6e] = Some(irq_stub_0x6e); table[0x6f] = Some(irq_stub_0x6f);
    table[0x70] = Some(irq_stub_0x70); table[0x71] = Some(irq_stub_0x71);
    table[0x72] = Some(irq_stub_0x72); table[0x73] = Some(irq_stub_0x73);
    table[0x74] = Some(irq_stub_0x74); table[0x75] = Some(irq_stub_0x75);
    table[0x76] = Some(irq_stub_0x76); table[0x77] = Some(irq_stub_0x77);
    table[0x78] = Some(irq_stub_0x78); table[0x79] = Some(irq_stub_0x79);
    table[0x7a] = Some(irq_stub_0x7a); table[0x7b] = Some(irq_stub_0x7b);
    table[0x7c] = Some(irq_stub_0x7c); table[0x7d] = Some(irq_stub_0x7d);
    table[0x7e] = Some(irq_stub_0x7e); table[0x7f] = Some(irq_stub_0x7f);
    table[0x80] = Some(irq_stub_0x80); table[0x81] = Some(irq_stub_0x81);
    table[0x82] = Some(irq_stub_0x82); table[0x83] = Some(irq_stub_0x83);
    table[0x84] = Some(irq_stub_0x84); table[0x85] = Some(irq_stub_0x85);
    table[0x86] = Some(irq_stub_0x86); table[0x87] = Some(irq_stub_0x87);
    table[0x88] = Some(irq_stub_0x88); table[0x89] = Some(irq_stub_0x89);
    table[0x8a] = Some(irq_stub_0x8a); table[0x8b] = Some(irq_stub_0x8b);
    table[0x8c] = Some(irq_stub_0x8c); table[0x8d] = Some(irq_stub_0x8d);
    table[0x8e] = Some(irq_stub_0x8e); table[0x8f] = Some(irq_stub_0x8f);
    table[0x90] = Some(irq_stub_0x90); table[0x91] = Some(irq_stub_0x91);
    table[0x92] = Some(irq_stub_0x92); table[0x93] = Some(irq_stub_0x93);
    table[0x94] = Some(irq_stub_0x94); table[0x95] = Some(irq_stub_0x95);
    table[0x96] = Some(irq_stub_0x96); table[0x97] = Some(irq_stub_0x97);
    table[0x98] = Some(irq_stub_0x98); table[0x99] = Some(irq_stub_0x99);
    table[0x9a] = Some(irq_stub_0x9a); table[0x9b] = Some(irq_stub_0x9b);
    table[0x9c] = Some(irq_stub_0x9c); table[0x9d] = Some(irq_stub_0x9d);
    table[0x9e] = Some(irq_stub_0x9e); table[0x9f] = Some(irq_stub_0x9f);
    table[0xa0] = Some(irq_stub_0xa0); table[0xa1] = Some(irq_stub_0xa1);
    table[0xa2] = Some(irq_stub_0xa2); table[0xa3] = Some(irq_stub_0xa3);
    table[0xa4] = Some(irq_stub_0xa4); table[0xa5] = Some(irq_stub_0xa5);
    table[0xa6] = Some(irq_stub_0xa6); table[0xa7] = Some(irq_stub_0xa7);
    table[0xa8] = Some(irq_stub_0xa8); table[0xa9] = Some(irq_stub_0xa9);
    table[0xaa] = Some(irq_stub_0xaa); table[0xab] = Some(irq_stub_0xab);
    table[0xac] = Some(irq_stub_0xac); table[0xad] = Some(irq_stub_0xad);
    table[0xae] = Some(irq_stub_0xae); table[0xaf] = Some(irq_stub_0xaf);
    table[0xb0] = Some(irq_stub_0xb0); table[0xb1] = Some(irq_stub_0xb1);
    table[0xb2] = Some(irq_stub_0xb2); table[0xb3] = Some(irq_stub_0xb3);
    table[0xb4] = Some(irq_stub_0xb4); table[0xb5] = Some(irq_stub_0xb5);
    table[0xb6] = Some(irq_stub_0xb6); table[0xb7] = Some(irq_stub_0xb7);
    table[0xb8] = Some(irq_stub_0xb8); table[0xb9] = Some(irq_stub_0xb9);
    table[0xba] = Some(irq_stub_0xba); table[0xbb] = Some(irq_stub_0xbb);
    table[0xbc] = Some(irq_stub_0xbc); table[0xbd] = Some(irq_stub_0xbd);
    table[0xbe] = Some(irq_stub_0xbe); table[0xbf] = Some(irq_stub_0xbf);
    table[0xc0] = Some(irq_stub_0xc0); table[0xc1] = Some(irq_stub_0xc1);
    table[0xc2] = Some(irq_stub_0xc2); table[0xc3] = Some(irq_stub_0xc3);
    table[0xc4] = Some(irq_stub_0xc4); table[0xc5] = Some(irq_stub_0xc5);
    table[0xc6] = Some(irq_stub_0xc6); table[0xc7] = Some(irq_stub_0xc7);
    table[0xc8] = Some(irq_stub_0xc8); table[0xc9] = Some(irq_stub_0xc9);
    table[0xca] = Some(irq_stub_0xca); table[0xcb] = Some(irq_stub_0xcb);
    table[0xcc] = Some(irq_stub_0xcc); table[0xcd] = Some(irq_stub_0xcd);
    table[0xce] = Some(irq_stub_0xce); table[0xcf] = Some(irq_stub_0xcf);
    table[0xd0] = Some(irq_stub_0xd0); table[0xd1] = Some(irq_stub_0xd1);
    table[0xd2] = Some(irq_stub_0xd2); table[0xd3] = Some(irq_stub_0xd3);
    table[0xd4] = Some(irq_stub_0xd4); table[0xd5] = Some(irq_stub_0xd5);
    table[0xd6] = Some(irq_stub_0xd6); table[0xd7] = Some(irq_stub_0xd7);
    table[0xd8] = Some(irq_stub_0xd8); table[0xd9] = Some(irq_stub_0xd9);
    table[0xda] = Some(irq_stub_0xda); table[0xdb] = Some(irq_stub_0xdb);
    table[0xdc] = Some(irq_stub_0xdc); table[0xdd] = Some(irq_stub_0xdd);
    table[0xde] = Some(irq_stub_0xde); table[0xdf] = Some(irq_stub_0xdf);
    table[0xe0] = Some(irq_stub_0xe0); table[0xe1] = Some(irq_stub_0xe1);
    table[0xe2] = Some(irq_stub_0xe2); table[0xe3] = Some(irq_stub_0xe3);
    table[0xe4] = Some(irq_stub_0xe4); table[0xe5] = Some(irq_stub_0xe5);
    table[0xe6] = Some(irq_stub_0xe6); table[0xe7] = Some(irq_stub_0xe7);
    table[0xe8] = Some(irq_stub_0xe8); table[0xe9] = Some(irq_stub_0xe9);
    table[0xea] = Some(irq_stub_0xea); table[0xeb] = Some(irq_stub_0xeb);
    table[0xec] = Some(irq_stub_0xec); table[0xed] = Some(irq_stub_0xed);
    table[0xee] = Some(irq_stub_0xee); table[0xef] = Some(irq_stub_0xef);
    table[0xf0] = Some(irq_stub_0xf0); table[0xf1] = Some(irq_stub_0xf1);
    table[0xf2] = Some(irq_stub_0xf2); table[0xf3] = Some(irq_stub_0xf3);
    table[0xf4] = Some(irq_stub_0xf4); table[0xf5] = Some(irq_stub_0xf5);
    table[0xf6] = Some(irq_stub_0xf6); table[0xf7] = Some(irq_stub_0xf7);
    table[0xf8] = Some(irq_stub_0xf8); table[0xf9] = Some(irq_stub_0xf9);
    table[0xfa] = Some(irq_stub_0xfa); table[0xfb] = Some(irq_stub_0xfb);
    table[0xfc] = Some(irq_stub_0xfc); table[0xfd] = Some(irq_stub_0xfd);
    table
};

/// Spurious interrupt handler (vector `0xFF` and APIC error vector `0xFE`).
pub(super) extern "x86-interrupt" fn handler_spurious(_stack: InterruptStackFrame) {
    // Log spurious interrupt entry for debugging nested/extra interrupts
    unsafe {
        print_str_0xe9("[INT] spurious\n");
    }
    // Return to interrupted context without EOI
}
