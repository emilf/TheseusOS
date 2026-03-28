//! Module: interrupts
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A1:-The-kernel-is-x86_64-no_std-code-using-the-x86-interrupt-ABI
//! - docs/axioms/arch-x86_64.md#A2:-GDT/TSS-setup-provides-dedicated-IST-stacks-for-critical-faults
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - This module owns IDT setup, exception/interrupt handler registration, APIC-facing vectors, and shared interrupt-time state used during bring-up.
//! - Hardware interrupt delivery is APIC-based in the current kernel path, with the documented timer/serial/xHCI vectors layered on top.
//! - Debug and monitor tooling may inspect interrupt state, but the source of truth for vector behavior remains the live IDT/APIC code here.
//!
//! SAFETY:
//! - Interrupt state changes here are system-wide and ordering-sensitive: descriptor tables, APIC state, and handler installation must agree before interrupts are enabled.
//! - Raw MMIO and privileged-instruction usage in submodules require valid mappings, valid IST/GDT state, and correct vector ownership assumptions.
//! - Comments that overclaim “everything is masked” or otherwise simplify architectural interrupt behavior are dangerous and should be treated as bugs.
//!
//! PROGRESS:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/observability.md
//!
//! Interrupt handling subsystem.
//!
//! This module owns IDT setup, exception/interrupt handler installation, APIC
//! integration, timer support, and interrupt-oriented debug helpers.

// Submodules
mod apic;
pub mod calibration;
mod debug;
pub(crate) mod handlers;
mod irq_registry;
mod timer;

// Re-export commonly used items from submodules
pub use apic::{
    apic_base_info, disable_all_interrupts, enable_interrupts, init_apic_mode, local_apic_eoi,
    local_apic_id, local_apic_read, local_apic_write, try_enable_x2apic, ApicAccessMode,
    ApicBaseInfo,
};
pub use debug::{print_gdt_summary_basic, print_idt_summary_compact};
pub use irq_registry::{dispatch_irq, list_irq_handlers, register_irq_handler, unregister_irq_handler};
pub use timer::{
    lapic_timer_configure, lapic_timer_mask,
    lapic_timer_start_oneshot, lapic_timer_start_periodic, timer_tick_count,
};

// Make debug functions available to submodules
use debug::{out_char_0xe9, print_hex_u64_0xe9, print_str_0xe9};

// Make handlers available to submodules and IDT setup
use handlers::{
    handler_bp, handler_de, handler_df, handler_gp, handler_mc, handler_nmi, handler_pf,
    handler_spurious, handler_ud,
    IRQ_STUB_TABLE,
};

use core::sync::atomic::AtomicU32;
use spin::Once as SpinOnce;
use x86_64::structures::idt::InterruptDescriptorTable;

// ============================================================================
// Public Constants
// ============================================================================

/// APIC timer interrupt vector number.
pub const APIC_TIMER_VECTOR: u8 = 0x40; // 64

/// Serial RX interrupt vector number.
pub const SERIAL_RX_VECTOR: u8 = APIC_TIMER_VECTOR + 1; // 65

/// Default MSI vector assigned to xHCI controllers.
pub const XHCI_MSI_VECTOR: u8 = 0x50;

/// APIC error interrupt vector.
const APIC_ERROR_VECTOR: u8 = 0xFE; // 254

// ============================================================================
// Shared State
// ============================================================================

/// Interrupt Descriptor Table (initialized once via SpinOnce)
static IDT_X86: SpinOnce<InterruptDescriptorTable> = SpinOnce::new();

/// Global timer tick counter.
///
/// Incremented by the APIC timer interrupt handler on every tick.
pub static TIMER_TICKS: AtomicU32 = AtomicU32::new(0);

/// Double-fault context captured before the fault escalates.
pub static mut DOUBLE_FAULT_CONTEXT: Option<DoubleFaultContext> = None;

/// Global handoff pointer used by the timer path for framebuffer animation access.
///
/// This is still transitional plumbing rather than the final long-term interface.
static mut HANDOFF_FOR_TIMER: Option<&'static theseus_shared::handoff::Handoff> = None;

// ============================================================================
// Public Data Structures
// ============================================================================

/// Best-effort context captured for double-fault debugging.
#[derive(Copy, Clone)]
pub struct DoubleFaultContext {
    /// Instruction pointer where fault occurred
    pub rip: u64,
    /// CR2 register (faulting address for page faults)
    pub cr2: u64,
    /// Stack pointer at time of fault
    pub rsp: u64,
    /// Top 6 quadwords from stack
    pub stack: [u64; 6],
}

impl DoubleFaultContext {
    /// Construct a double-fault context value.
    pub const fn new(rip: u64, cr2: u64, rsp: u64, stack: [u64; 6]) -> Self {
        Self {
            rip,
            cr2,
            rsp,
            stack,
        }
    }
}

/// Packed IDT entry layout used by the debug helpers.
#[repr(C, packed)]
#[derive(Copy, Clone)]
struct IdtEntry {
    offset_low: u16,
    selector: u16,
    ist: u8,
    type_attr: u8,
    offset_mid: u16,
    offset_high: u32,
    zero: u32,
}

impl IdtEntry {
    #[allow(dead_code)]
    const fn missing() -> Self {
        Self {
            offset_low: 0,
            selector: 0,
            ist: 0,
            type_attr: 0,
            offset_mid: 0,
            offset_high: 0,
            zero: 0,
        }
    }
}

// ============================================================================
// Public API Functions
// ============================================================================

/// Set double fault context for debugging
///
/// Call this before an operation that might cause a double fault to save
/// diagnostic information.
///
/// # Arguments
/// * `rip` - Instruction pointer
/// * `cr2` - CR2 register value
/// * `rsp` - Stack pointer
/// * `stack` - Top 6 stack values
pub fn set_double_fault_context(rip: u64, cr2: u64, rsp: u64, stack: [u64; 6]) {
    unsafe {
        DOUBLE_FAULT_CONTEXT = Some(DoubleFaultContext::new(rip, cr2, rsp, stack));
    }
}

/// Build and load the runtime IDT.
///
/// The current IDT installs the documented exception handlers plus the APIC timer,
/// serial RX, xHCI MSI, spurious, and APIC-error vectors. Critical faults are wired
/// to dedicated IST stacks supplied by the GDT/TSS path.
pub unsafe fn setup_idt() {
    let idt = IDT_X86.call_once(|| {
        let mut idt = InterruptDescriptorTable::new();

        // Install exception handlers
        idt.divide_error.set_handler_fn(handler_de);
        idt.breakpoint.set_handler_fn(handler_bp);
        idt.invalid_opcode.set_handler_fn(handler_ud);
        idt.general_protection_fault.set_handler_fn(handler_gp);
        idt.page_fault.set_handler_fn(handler_pf);

        // Spurious and APIC error keep dedicated handlers: the Intel SDM
        // requires that spurious interrupts (0xFF) do NOT receive an EOI.
        idt[0xFF].set_handler_fn(handler_spurious);
        idt[APIC_ERROR_VECTOR as usize].set_handler_fn(handler_spurious);

        // All other hardware IRQs (0x20–0xFD) — including timer (0x40),
        // serial (0x41), and xHCI (0x50) — go through the uniform stub system.
        // Handlers for those vectors are registered in the IRQ registry below.
        for (vector, stub_opt) in IRQ_STUB_TABLE.iter().enumerate() {
            if let Some(stub) = stub_opt {
                unsafe {
                    idt[vector]
                        .set_handler_addr(x86_64::VirtAddr::new(*stub as u64));
                }
            }
        }

        // IRQ handler registrations are done by each driver/subsystem during
        // their own init — not here. init_idt only sets up the IDT structure.

        // Assign IST indices for critical exceptions
        // These use dedicated stacks to prevent recursive faults
        {
            use crate::gdt::{IST_INDEX_DF, IST_INDEX_MC, IST_INDEX_NMI, IST_INDEX_PF};
            unsafe {
                idt.double_fault
                    .set_handler_fn(handler_df)
                    .set_stack_index(IST_INDEX_DF);
                idt.non_maskable_interrupt
                    .set_handler_fn(handler_nmi)
                    .set_stack_index(IST_INDEX_NMI);
                idt.machine_check
                    .set_handler_fn(handler_mc)
                    .set_stack_index(IST_INDEX_MC);
                idt.page_fault
                    .set_handler_fn(handler_pf)
                    .set_stack_index(IST_INDEX_PF);
            }
        }

        idt
    });

    // Load the IDT into the CPU
    idt.load();
}

/// Perform a small sanity check on the loaded IDT.
pub unsafe fn validate_idt_basic() -> bool {
    use x86_64::instructions::tables::sidt;
    let idtr = sidt();

    // Check that IDTR limit is reasonable (at least 256 entries = 4095 bytes)
    if idtr.limit < 4095 {
        return false;
    }

    // Verify key exception handlers are non-zero
    let base = idtr.base.as_u64();
    for idx in [0, 3, 6, 13, 14] {
        let entry_ptr = base + (idx * 16);
        let low = core::ptr::read_unaligned(entry_ptr as *const u64);

        // Extract handler address from low quadword
        let offset_low = low & 0xFFFF;
        let offset_mid = (low >> 48) & 0xFFFF;

        if offset_low == 0 && offset_mid == 0 {
            return false; // Handler address is zero
        }
    }

    true
}

/// Trigger a breakpoint exception with `int3`.
pub fn trigger_breakpoint() {
    unsafe {
        core::arch::asm!("int3");
    }
}

/// Check whether the CPU interrupt flag is currently enabled.
pub fn interrupts_enabled() -> bool {
    use x86_64::registers::rflags::{self, RFlags};
    rflags::read().contains(RFlags::INTERRUPT_FLAG)
}

/// Set the transitional handoff pointer used by the timer handler.
///
/// This exists only because the timer-driven framebuffer animation still reaches
/// through handoff data instead of a more final subsystem boundary.
pub unsafe fn set_handoff_for_timer(handoff: &'static theseus_shared::handoff::Handoff) {
    HANDOFF_FOR_TIMER = Some(handoff);
}

/// Get the transitional handoff pointer used by the timer handler.
pub unsafe fn get_handoff_for_timer() -> Option<&'static theseus_shared::handoff::Handoff> {
    HANDOFF_FOR_TIMER
}

/// No-op function for testing
///
/// Used to test whether function calls work correctly in various contexts.
/// Can be called safely without side effects.
#[no_mangle]
pub extern "C" fn noop_for_test() {}
