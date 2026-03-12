//! Module: gdt
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A2:-GDT/TSS-setup-provides-dedicated-IST-stacks-for-critical-faults
//!
//! INVARIANTS:
//! - This module owns the runtime GDT/TSS setup used by the kernel in long mode.
//! - Dedicated IST stacks exist for the documented critical-fault paths.
//! - GDT/TSS state here must stay aligned with IDT handler expectations and the stack-mapping work performed during bring-up.
//!
//! SAFETY:
//! - Descriptor-table and TSS changes are CPU-global state for the running core and must only be performed in the intended bring-up sequence.
//! - IST stack pointers must reference mapped writable stack memory before fault handlers rely on them.
//! - Comments that describe x86-64 segmentation too casually can hide the fact that the TSS/IST path is still operationally critical.
//!
//! PROGRESS:
//! - docs/plans/interrupts-and-platform.md
//!
//! Runtime GDT/TSS setup for long-mode kernel execution.
//!
//! This module owns the descriptor-table state and dedicated IST stacks used by
//! the current bring-up path.

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
/// Double-fault IST stack (16 KiB).
#[link_section = ".bss.stack"]
static mut IST_DF_STACK: [u8; 16 * 4096] = [0; 16 * 4096];
/// NMI IST stack (16 KiB).
#[link_section = ".bss.stack"]
static mut IST_NMI_STACK: [u8; 16 * 4096] = [0; 16 * 4096];
/// Machine-check IST stack (16 KiB).
#[link_section = ".bss.stack"]
static mut IST_MC_STACK: [u8; 16 * 4096] = [0; 16 * 4096];
/// Page-fault IST stack (16 KiB).
#[link_section = ".bss.stack"]
static mut IST_PF_STACK: [u8; 16 * 4096] = [0; 16 * 4096];

/// Interrupt Stack Table indices.
pub const IST_INDEX_DF: u16 = 0; // IST1 - Double Fault
pub const IST_INDEX_NMI: u16 = 1; // IST2 - Non-Maskable Interrupt
pub const IST_INDEX_MC: u16 = 2; // IST3 - Machine Check
pub const IST_INDEX_PF: u16 = 3; // IST4 - Page Fault

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

/// Return IST stack base addresses and sizes for explicit mapping.
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

/// Kernel code-segment selector used by the runtime GDT.
pub const KERNEL_CS: u16 = 0x08;

/// Build and load the runtime GDT/TSS state.
pub unsafe fn setup_gdt() {
    // Build and load runtime GDT with TSS
    let state = GDT_STATE.call_once(|| unsafe { build_gdt_state() });
    state.gdt.load();
    // Reload CS and load TSS
    CS::set_reg(state.code_sel);
    load_tss(state.tss_sel);
    reload_data_segments(state.data_sel);
}

/// Refresh the IST pointers stored in the TSS.
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

/// Read the current page-fault IST pointer from the TSS.
pub fn get_pf_ist_top() -> u64 {
    unsafe { TSS_STATIC.interrupt_stack_table[IST_INDEX_PF as usize].as_u64() }
}

/// Reload the data-segment registers for the current runtime GDT state.
///
/// In long mode, DS/ES/SS are reloaded from the runtime selector while FS/GS are
/// cleared here and configured elsewhere as needed.
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
