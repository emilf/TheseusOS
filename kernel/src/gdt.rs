//! Global Descriptor Table (GDT) module
//! 
//! This module provides the GDT setup required for x86-64 long mode.
//! It creates the necessary segment descriptors for kernel and user mode operation.

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

#[repr(C)]
struct Selectors {
    code: SegmentSelector,
    tss: SegmentSelector,
}

// IST stacks (aligned in .bss)
#[link_section = ".bss.stack"]
static mut IST_DF_STACK: [u8; 16 * 4096] = [0; 16 * 4096];
#[link_section = ".bss.stack"]
static mut IST_NMI_STACK: [u8; 16 * 4096] = [0; 16 * 4096];
#[link_section = ".bss.stack"]
static mut IST_MC_STACK: [u8; 16 * 4096] = [0; 16 * 4096];

pub const IST_INDEX_DF: u16 = 0; // IST1
pub const IST_INDEX_NMI: u16 = 1; // IST2
pub const IST_INDEX_MC: u16 = 2; // IST3

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

/// Selectors (informational): x86_64 crate manages actual selectors; keep KERNEL_CS for checks
pub const KERNEL_CS: u16 = 0x08;

/// Set up and load the GDT
pub unsafe fn setup_gdt() {
    // Build and load runtime GDT with TSS
    let tss = init_tss();
    GDT_RT = MaybeUninit::new(GlobalDescriptorTable::new());
    let gdt_rt = &mut *GDT_RT.as_mut_ptr();
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
unsafe fn reload_data_segments() {
    // ds := 0 (null selector is permitted for data segs in long mode)
    core::arch::asm!("xor eax, eax", "mov ds, ax", options(nomem, nostack, preserves_flags));

    // es := 0
    core::arch::asm!("xor eax, eax", "mov es, ax", options(nomem, nostack, preserves_flags));

    // fs := 0 (FS base will be set via MSR when needed)
    core::arch::asm!("xor eax, eax", "mov fs, ax", options(nomem, nostack, preserves_flags));

    // gs := 0 (GS base via MSR later if needed)
    core::arch::asm!("xor eax, eax", "mov gs, ax", options(nomem, nostack, preserves_flags));

    // skip ss for now (can cause #GP in long mode if not needed)
    // core::arch::asm!("mov ss, ax", in("ax") KERNEL_DS, options(nomem, nostack, preserves_flags));
}

// CS is reloaded via x86_64 crate APIs; legacy helper removed
