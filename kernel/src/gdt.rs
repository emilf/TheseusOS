//! Global Descriptor Table (GDT) module
//! 
//! This module provides the GDT setup required for x86-64 long mode.
//! It creates the necessary segment descriptors for kernel and user mode operation.
//! 
//! The GDT is essential for x86-64 operation and provides:
//! - Code and data segment descriptors
//! - Task State Segment (TSS) for interrupt handling
//! - Interrupt Stack Table (IST) for critical exceptions
//! 
//! This module uses the x86_64 crate for GDT management and provides
//! the necessary setup functions for kernel initialization.

use core::mem::MaybeUninit;
use spin::Once as SpinOnce;
use x86_64::{
    VirtAddr,
    instructions::{segmentation::{CS, Segment}, tables::load_tss},
    structures::{
        gdt::{Descriptor, GlobalDescriptorTable, SegmentSelector},
        tss::TaskStateSegment,
    },
};

// Legacy manual GDT removed; we use x86_64's GDT entirely

// TSS and GDT built using x86_64 crate
static mut TSS: TaskStateSegment = TaskStateSegment::new();
static mut GDT_RT: MaybeUninit<GlobalDescriptorTable> = MaybeUninit::uninit();
static SELECTORS: SpinOnce<Selectors> = SpinOnce::new();

/// Structure to hold GDT selectors
#[repr(C)]
struct Selectors {
    code: SegmentSelector,
    tss: SegmentSelector,
}

// IST stacks (aligned in .bss)
/// Double Fault interrupt stack (16KB)
#[link_section = ".bss.stack"]
static mut IST_DF_STACK: [u8; 16 * 4096] = [0; 16 * 4096];
/// Non-Maskable Interrupt stack (16KB)
#[link_section = ".bss.stack"]
static mut IST_NMI_STACK: [u8; 16 * 4096] = [0; 16 * 4096];
/// Machine Check interrupt stack (16KB)
#[link_section = ".bss.stack"]
static mut IST_MC_STACK: [u8; 16 * 4096] = [0; 16 * 4096];

/// Interrupt Stack Table indices
pub const IST_INDEX_DF: u16 = 0; // IST1 - Double Fault
pub const IST_INDEX_NMI: u16 = 1; // IST2 - Non-Maskable Interrupt
pub const IST_INDEX_MC: u16 = 2; // IST3 - Machine Check

/// Initialize the Task State Segment with IST stacks
/// 
/// This function sets up the TSS with dedicated interrupt stacks for critical
/// exceptions. Each stack is 16KB and properly aligned.
/// 
/// # Returns
/// 
/// A reference to the initialized TSS
/// 
/// # Safety
/// 
/// This function is safe to call during kernel initialization.
unsafe fn init_tss() -> &'static TaskStateSegment {
    // Set IST stack pointers to top of stacks (16-byte aligned)
    let df_top = (core::ptr::addr_of!(IST_DF_STACK) as u64) + (16 * 4096) as u64;
    let nmi_top = (core::ptr::addr_of!(IST_NMI_STACK) as u64) + (16 * 4096) as u64;
    let mc_top = (core::ptr::addr_of!(IST_MC_STACK) as u64) + (16 * 4096) as u64;
    TSS.interrupt_stack_table[IST_INDEX_DF as usize] = VirtAddr::new(df_top & !0xFu64);
    TSS.interrupt_stack_table[IST_INDEX_NMI as usize] = VirtAddr::new(nmi_top & !0xFu64);
    TSS.interrupt_stack_table[IST_INDEX_MC as usize] = VirtAddr::new(mc_top & !0xFu64);
    core::mem::transmute::<*const TaskStateSegment, &'static TaskStateSegment>(&raw const TSS as *const _)
}

/// Kernel code segment selector
/// 
/// This is the selector for the kernel code segment in the GDT.
/// It's used for verification and debugging purposes.
pub const KERNEL_CS: u16 = 0x08;

/// Set up and load the GDT
/// 
/// This function creates and loads the Global Descriptor Table with the necessary
/// segments for kernel operation. It sets up code segments, data segments, and
/// the Task State Segment for interrupt handling.
/// 
/// # Safety
/// 
/// This function modifies CPU segment registers and should only be called
/// during kernel initialization when it's safe to modify these registers.
pub unsafe fn setup_gdt() {
    // Build and load runtime GDT with TSS
    let tss = init_tss();
    GDT_RT = MaybeUninit::new(GlobalDescriptorTable::new());
    let gdt_rt = unsafe { GDT_RT.assume_init_mut() };
    let code_sel = gdt_rt.add_entry(Descriptor::kernel_code_segment());
    let tss_sel = gdt_rt.add_entry(Descriptor::tss_segment(tss));
    gdt_rt.load();
    SELECTORS.call_once(|| Selectors { code: code_sel, tss: tss_sel });
    // Reload CS and load TSS
    CS::set_reg(code_sel);
    load_tss(tss_sel);
    // Also reload data segments to 0 as before
    reload_data_segments();
}

/// Reload segment registers with new selectors
/// 
/// This function reloads the data segment registers (DS, ES, FS, GS) with
/// null selectors, which is the standard practice in x86-64 long mode.
/// 
/// # Safety
/// 
/// This function modifies CPU segment registers and should only be called
/// during kernel initialization when it's safe to modify these registers.
/// Reload data segment registers with null selectors
/// 
/// In x86-64 long mode, data segment registers (DS, ES, FS, GS) can be set to
/// null selectors (0) which effectively disable segmentation for those segments.
/// This is the standard approach in 64-bit mode where segmentation is largely
/// unused except for FS and GS which can have custom base addresses via MSRs.
/// 
/// # Assembly Details
/// - `xor eax, eax`: Clear EAX register (sets it to 0)
/// - `mov ds, ax`: Load DS segment register with null selector
/// - `mov es, ax`: Load ES segment register with null selector  
/// - `mov fs, ax`: Load FS segment register with null selector
/// - `mov gs, ax`: Load GS segment register with null selector
/// 
/// # Safety
/// 
/// This function is unsafe because it modifies segment registers directly.
/// It should only be called during kernel initialization when it's safe to
/// modify these critical registers.
/// 
/// The caller must ensure:
/// - We are in kernel mode with sufficient privileges
/// - No other code is using the segment registers concurrently
/// - The system is in a stable state for segment register modification
/// - This is called during proper kernel initialization sequence
unsafe fn reload_data_segments() {
    // Set DS to null selector (0) - null selectors are valid for data segments in long mode
    core::arch::asm!(
        "xor eax, eax",  // Clear EAX to 0
        "mov ds, ax",    // Load DS with null selector
        options(nomem, nostack, preserves_flags)
    );

    // Set ES to null selector (0)
    core::arch::asm!(
        "xor eax, eax",  // Clear EAX to 0
        "mov es, ax",    // Load ES with null selector
        options(nomem, nostack, preserves_flags)
    );

    // Set FS to null selector (0) - base address can be set via MSR later if needed
    core::arch::asm!(
        "xor eax, eax",  // Clear EAX to 0
        "mov fs, ax",    // Load FS with null selector
        options(nomem, nostack, preserves_flags)
    );

    // Set GS to null selector (0) - base address can be set via MSR later if needed
    core::arch::asm!(
        "xor eax, eax",  // Clear EAX to 0
        "mov gs, ax",    // Load GS with null selector
        options(nomem, nostack, preserves_flags)
    );

    // Skip SS (stack segment) for now as it can cause #GP in long mode if not properly configured
    // The stack segment is typically not used in 64-bit mode
    // core::arch::asm!("mov ss, ax", in("ax") KERNEL_DS, options(nomem, nostack, preserves_flags));
}

// CS is reloaded via x86_64 crate APIs; legacy helper removed
