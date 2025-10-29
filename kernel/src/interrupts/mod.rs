//! Interrupt handling subsystem
//!
//! This module provides comprehensive interrupt management for the x86-64 kernel:
//!
//! ## Core Components
//!
//! - **IDT Setup**: Initialize and load the Interrupt Descriptor Table
//! - **Exception Handlers**: CPU exceptions (page faults, general protection, etc.)
//! - **Hardware Interrupts**: APIC timer, serial port, spurious interrupts
//! - **APIC Management**: Local APIC configuration and control
//! - **Timer Control**: LAPIC timer for periodic interrupts
//! - **Debug Utilities**: IDT/GDT inspection tools
//!
//! ## Module Organization
//!
//! - `handlers`: Exception and interrupt handler implementations
//! - `apic`: APIC configuration, MMIO access, interrupt disabling
//! - `timer`: LAPIC timer setup and control
//! - `debug`: Debugging utilities (IDT/GDT printing, QEMU debug port)
//!
//! ## Interrupt Vectors
//!
//! - 0-31: CPU exceptions (reserved by Intel)
//! - 0x40 (64): APIC timer interrupt
//! - 0x41 (65): Serial RX interrupt (COM1, IRQ 4)
//! - 0xFE (254): APIC error interrupt
//! - 0xFF (255): Spurious interrupt vector
//!
//! ## Safety
//!
//! Most functions in this module are `unsafe` because they:
//! - Modify system-wide interrupt state
//! - Access hardware registers (APIC MMIO)
//! - Execute privileged instructions (CLI, STI, LIDT, etc.)
//!
//! Callers must ensure the system is in an appropriate state for these operations.

// Submodules
mod apic;
mod debug;
mod handlers;
mod timer;

// Re-export commonly used items from submodules
pub use apic::{disable_all_interrupts, enable_interrupts, get_apic_base, read_apic_register};
pub use debug::{print_gdt_summary_basic, print_idt_summary_compact};
pub use timer::{
    install_timer_vector_runtime, lapic_timer_configure, lapic_timer_mask,
    lapic_timer_start_oneshot, lapic_timer_start_periodic, timer_tick_count,
};

// Make APIC functions available to submodules
use apic::write_apic_register;

// Make debug functions available to submodules
use debug::{out_char_0xe9, print_hex_u64_0xe9, print_str_0xe9};

// Make handlers available to submodules and IDT setup
use handlers::{
    handler_bp, handler_de, handler_df, handler_gp, handler_mc, handler_nmi, handler_pf,
    handler_serial_rx, handler_spurious, handler_timer, handler_ud, handler_usb_xhci,
};

use core::sync::atomic::AtomicU32;
use spin::Once as SpinOnce;
use x86_64::structures::idt::InterruptDescriptorTable;

// ============================================================================
// Public Constants
// ============================================================================

/// APIC timer interrupt vector number
pub const APIC_TIMER_VECTOR: u8 = 0x40; // 64

/// Serial RX interrupt vector number  
pub const SERIAL_RX_VECTOR: u8 = APIC_TIMER_VECTOR + 1; // 65

/// Default MSI vector assigned to xHCI controllers.
pub const XHCI_MSI_VECTOR: u8 = 0x50;

/// APIC error interrupt vector
const APIC_ERROR_VECTOR: u8 = 0xFE; // 254

// ============================================================================
// Shared State
// ============================================================================

/// Interrupt Descriptor Table (initialized once via SpinOnce)
static IDT_X86: SpinOnce<InterruptDescriptorTable> = SpinOnce::new();

/// Global timer tick counter
///
/// Incremented by the APIC timer interrupt handler on every tick.
/// Used for timing, delays, and animation.
pub static TIMER_TICKS: AtomicU32 = AtomicU32::new(0);

/// Double fault context (captured before double fault occurs)
///
/// When a double fault happens, this contains saved state from the
/// fault that led to the double fault.
pub static mut DOUBLE_FAULT_CONTEXT: Option<DoubleFaultContext> = None;

/// Global handoff pointer for timer interrupt access
///
/// The timer interrupt handler uses this to access framebuffer info
/// for heart animation updates.
///
/// # TODO: Remove this and get framebuffer via driver subsystem
static mut HANDOFF_FOR_TIMER: Option<&'static theseus_shared::handoff::Handoff> = None;

// ============================================================================
// Public Data Structures
// ============================================================================

/// Context information for double fault debugging
///
/// Contains register state and stack snapshot from when a fault occurred
/// that led to a double fault.
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
    /// Create a new double fault context
    pub const fn new(rip: u64, cr2: u64, rsp: u64, stack: [u64; 6]) -> Self {
        Self {
            rip,
            cr2,
            rsp,
            stack,
        }
    }
}

/// IDT entry structure (for debugging with SIDT)
///
/// Represents a single 16-byte entry in the IDT.
/// Used by debug utilities to inspect the IDT.
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

/// Set up the Interrupt Descriptor Table
///
/// Creates and loads the IDT with handlers for all critical exceptions and
/// hardware interrupts. Sets up IST (Interrupt Stack Table) indices for
/// critical handlers that need dedicated stacks.
///
/// # Handlers Installed
///
/// **Exceptions (using IST stacks)**:
/// - Vector 2 (NMI): Non-maskable interrupt
/// - Vector 8 (DF): Double fault
/// - Vector 14 (PF): Page fault
/// - Vector 18 (MC): Machine check
///
/// **Exceptions (using main stack)**:
/// - Vector 0 (DE): Divide error
/// - Vector 3 (BP): Breakpoint
/// - Vector 6 (UD): Invalid opcode
/// - Vector 13 (GP): General protection fault
///
/// **Hardware Interrupts**:
/// - Vector 0x40 (64): APIC timer
/// - Vector 0x41 (65): Serial RX (COM1)
/// - Vector 0xFE (254): APIC error
/// - Vector 0xFF (255): Spurious interrupt
///
/// # Safety
/// - Should be called during kernel initialization
/// - Must be called before enabling interrupts
/// - IDT must remain valid for the lifetime of the kernel
/// - IST stacks must be properly configured in TSS before calling
///
/// # Examples
/// ```rust
/// unsafe {
///     setup_gdt();          // Set up GDT and TSS with IST stacks
///     setup_idt();          // Set up IDT with exception handlers
///     enable_interrupts();  // Enable IF flag
/// }
/// ```
pub unsafe fn setup_idt() {
    let idt = IDT_X86.call_once(|| {
        let mut idt = InterruptDescriptorTable::new();

        // Install exception handlers
        idt.divide_error.set_handler_fn(handler_de);
        idt.breakpoint.set_handler_fn(handler_bp);
        idt.invalid_opcode.set_handler_fn(handler_ud);
        idt.general_protection_fault.set_handler_fn(handler_gp);
        idt.page_fault.set_handler_fn(handler_pf);

        // Install hardware interrupt handlers
        idt[APIC_TIMER_VECTOR as usize].set_handler_fn(handler_timer);
        idt[SERIAL_RX_VECTOR as usize].set_handler_fn(handler_serial_rx);
        idt[XHCI_MSI_VECTOR as usize].set_handler_fn(handler_usb_xhci);
        idt[0xFF].set_handler_fn(handler_spurious);
        idt[APIC_ERROR_VECTOR as usize].set_handler_fn(handler_spurious);

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

/// Validate basic IDT setup
///
/// Performs sanity checks on the IDT to ensure it's properly configured.
/// Checks that key exception handlers are installed.
///
/// # Returns
/// * `true` - IDT appears valid
/// * `false` - IDT has issues
///
/// # Safety
/// Uses SIDT to read IDT base, then inspects entries.
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

/// Trigger a breakpoint exception
///
/// Executes INT3 instruction to trigger vector 3 (breakpoint).
/// Useful for testing exception handling.
///
/// # Examples
/// ```rust
/// trigger_breakpoint();  // Will invoke BP handler
/// ```
pub fn trigger_breakpoint() {
    unsafe {
        core::arch::asm!("int3");
    }
}

/// Check if interrupts are enabled
///
/// Reads the IF (Interrupt Flag) from RFLAGS.
///
/// # Returns
/// * `true` - Interrupts enabled (IF=1)
/// * `false` - Interrupts disabled (IF=0)
pub fn interrupts_enabled() -> bool {
    use x86_64::registers::rflags::{self, RFlags};
    rflags::read().contains(RFlags::INTERRUPT_FLAG)
}

/// Set handoff pointer for timer interrupt handler
///
/// The timer handler needs access to the handoff structure to update
/// the heart animation framebuffer.
///
/// # Arguments
/// * `handoff` - Static reference to handoff structure
///
/// # Safety
/// - Handoff reference must be valid for 'static lifetime
/// - Should only be called once during initialization
///
/// # TODO: Remove this and get framebuffer via driver subsystem
pub unsafe fn set_handoff_for_timer(handoff: &'static theseus_shared::handoff::Handoff) {
    HANDOFF_FOR_TIMER = Some(handoff);
}

/// Get handoff pointer for timer interrupt handler
///
/// # Returns
/// * `Some(&Handoff)` - Handoff is available
/// * `None` - Handoff not yet set
///
/// # TODO: Remove this and get framebuffer via driver subsystem
pub unsafe fn get_handoff_for_timer() -> Option<&'static theseus_shared::handoff::Handoff> {
    HANDOFF_FOR_TIMER
}

/// No-op function for testing
///
/// Used to test whether function calls work correctly in various contexts.
/// Can be called safely without side effects.
#[no_mangle]
pub extern "C" fn noop_for_test() {}
