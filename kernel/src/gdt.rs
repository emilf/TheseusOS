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

use spin::Once as SpinOnce;
use x86_64::{
    instructions::{
        segmentation::{Segment, CS, DS, ES, SS},
        tables::load_tss,
    },
    structures::{
        gdt::{Descriptor, GlobalDescriptorTable, SegmentSelector},
        tss::TaskStateSegment,
    },
    VirtAddr,
};

struct GdtState {
    gdt: GlobalDescriptorTable,
    code_sel: SegmentSelector,
    data_sel: SegmentSelector,
    tss_sel: SegmentSelector,
}

static GDT_STATE: SpinOnce<GdtState> = SpinOnce::new();

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
/// Page Fault interrupt stack (16KB)
#[link_section = ".bss.stack"]
static mut IST_PF_STACK: [u8; 16 * 4096] = [0; 16 * 4096];

/// Interrupt Stack Table indices
pub const IST_INDEX_DF: u16 = 0; // IST1 - Double Fault
pub const IST_INDEX_NMI: u16 = 1; // IST2 - Non-Maskable Interrupt
pub const IST_INDEX_MC: u16 = 2; // IST3 - Machine Check
pub const IST_INDEX_PF: u16 = 3; // IST4 - Page Fault

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
// Expose TSS storage so we can refresh IST pointers at runtime if needed
static mut TSS_STATIC: TaskStateSegment = TaskStateSegment::new();

unsafe fn build_gdt_state() -> GdtState {
    // Create TSS and set IST stack pointers to top of stacks (16-byte aligned)
    let mut tss = TaskStateSegment::new();
    let df_top = (core::ptr::addr_of!(IST_DF_STACK) as u64) + (16 * 4096) as u64;
    let nmi_top = (core::ptr::addr_of!(IST_NMI_STACK) as u64) + (16 * 4096) as u64;
    let mc_top = (core::ptr::addr_of!(IST_MC_STACK) as u64) + (16 * 4096) as u64;
    let pf_top = (core::ptr::addr_of!(IST_PF_STACK) as u64) + (16 * 4096) as u64;
    tss.interrupt_stack_table[IST_INDEX_DF as usize] = VirtAddr::new(df_top & !0xFu64);
    tss.interrupt_stack_table[IST_INDEX_NMI as usize] = VirtAddr::new(nmi_top & !0xFu64);
    tss.interrupt_stack_table[IST_INDEX_MC as usize] = VirtAddr::new(mc_top & !0xFu64);
    tss.interrupt_stack_table[IST_INDEX_PF as usize] = VirtAddr::new(pf_top & !0xFu64);

    let mut gdt = GlobalDescriptorTable::new();
    let code_sel = gdt.add_entry(Descriptor::kernel_code_segment());
    let data_sel = gdt.add_entry(Descriptor::kernel_data_segment());
    TSS_STATIC = tss;
    let tss_ref: &'static TaskStateSegment = core::mem::transmute::<
        *const TaskStateSegment,
        &'static TaskStateSegment,
    >(&raw const TSS_STATIC as *const _);
    let tss_sel = gdt.add_entry(Descriptor::tss_segment(tss_ref));
    GdtState {
        gdt,
        code_sel,
        data_sel,
        tss_sel,
    }
}

/// Expose IST stack base addresses and sizes for explicit mapping
pub fn ist_stack_ranges() -> [(u64, u64); 4] {
    [
        (core::ptr::addr_of!(IST_DF_STACK) as u64, (16 * 4096) as u64),
        (
            core::ptr::addr_of!(IST_NMI_STACK) as u64,
            (16 * 4096) as u64,
        ),
        (core::ptr::addr_of!(IST_MC_STACK) as u64, (16 * 4096) as u64),
        (core::ptr::addr_of!(IST_PF_STACK) as u64, (16 * 4096) as u64),
    ]
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
    let state = GDT_STATE.call_once(|| unsafe { build_gdt_state() });
    state.gdt.load();
    // Reload CS and load TSS
    CS::set_reg(state.code_sel);
    load_tss(state.tss_sel);
    reload_data_segments(state.data_sel);
}

/// Refresh TSS IST pointers and reload TSS
///
/// Useful to ensure PF/DF/NMI/MC IST pointers are correct after relocation.
pub unsafe fn refresh_tss_ist() {
    let df_top = (core::ptr::addr_of!(IST_DF_STACK) as u64) + (16 * 4096) as u64;
    let nmi_top = (core::ptr::addr_of!(IST_NMI_STACK) as u64) + (16 * 4096) as u64;
    let mc_top = (core::ptr::addr_of!(IST_MC_STACK) as u64) + (16 * 4096) as u64;
    let pf_top = (core::ptr::addr_of!(IST_PF_STACK) as u64) + (16 * 4096) as u64;
    TSS_STATIC.interrupt_stack_table[IST_INDEX_DF as usize] = VirtAddr::new(df_top & !0xFu64);
    TSS_STATIC.interrupt_stack_table[IST_INDEX_NMI as usize] = VirtAddr::new(nmi_top & !0xFu64);
    TSS_STATIC.interrupt_stack_table[IST_INDEX_MC as usize] = VirtAddr::new(mc_top & !0xFu64);
    TSS_STATIC.interrupt_stack_table[IST_INDEX_PF as usize] = VirtAddr::new(pf_top & !0xFu64);
    let _ = GDT_STATE.get();
}

/// Read current PF IST pointer from TSS
pub fn get_pf_ist_top() -> u64 {
    unsafe { TSS_STATIC.interrupt_stack_table[IST_INDEX_PF as usize].as_u64() }
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
unsafe fn reload_data_segments(data_sel: SegmentSelector) {
    DS::set_reg(data_sel);
    ES::set_reg(data_sel);
    SS::set_reg(data_sel);

    // Leave FS/GS at 0 for now; TLS via GS base MSR is configured elsewhere
    // This only marks the segments as disabled, but doesn't actually clear them.
    core::arch::asm!(
        "xor eax, eax",
        "mov fs, ax",
        options(nomem, nostack, preserves_flags)
    );
    core::arch::asm!(
        "xor eax, eax",
        "mov gs, ax",
        options(nomem, nostack, preserves_flags)
    );
}
