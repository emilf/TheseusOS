//! Kernel environment setup module
//! 
//! This module provides functions for setting up the complete kernel environment
//! including interrupts, GDT, CPU features, and virtual memory.

use theseus_shared::handoff::Handoff;
use crate::interrupts::{disable_all_interrupts, setup_idt};
use crate::gdt::setup_gdt;
use crate::cpu::{setup_control_registers, detect_cpu_features, setup_floating_point, setup_msrs};
use crate::memory::{MemoryManager, activate_virtual_memory, KERNEL_VIRTUAL_BASE};

/// Set up complete kernel environment (correct order)
/// 
/// This function performs the setup sequence in the correct order to establish kernel control:
/// 1. Exit boot services FIRST (to prevent firmware interference)
/// 2. Disable all interrupts including NMI
/// 3. Set up GDT and TSS
/// 4. Configure control registers
/// 5. Set up CPU features
/// 6. Test basic operations
pub fn setup_kernel_environment(_handoff: &Handoff) {
    crate::display::kernel_write_line("=== Setting up Kernel Environment ===");
    
    // 1. Disable all interrupts first (including NMI)
    crate::display::kernel_write_line("1. Disabling all interrupts...");
    unsafe {
        disable_all_interrupts();
    }
    crate::display::kernel_write_line("  ✓ All interrupts disabled");
    
    // 2. Set up GDT and TSS
    crate::display::kernel_write_line("2. Setting up GDT...");
    unsafe {
        setup_gdt();
    }
    crate::display::kernel_write_line("  ✓ GDT loaded and segments reloaded");
    
    // 3. Configure control registers (PAE etc.)
    crate::display::kernel_write_line("3. Configuring control registers...");
    unsafe { setup_control_registers(); }
    crate::display::kernel_write_line("  ✓ Control registers configured");

    // 3.5 Set up paging (identity map + high-half kernel) and load CR3
    crate::display::kernel_write_line("3.5. Setting up paging...");
    unsafe {
        crate::display::kernel_write_line("  [vm] before new");
        let mm = MemoryManager::new(_handoff);
        crate::display::kernel_write_line("  [vm] after new; loading CR3");
        activate_virtual_memory(mm.page_table_root());
        crate::display::kernel_write_line("  [vm] after CR3");
    }
    crate::display::kernel_write_line("  ✓ Paging enabled (identity + high-half kernel)");
    
    // 4. Install low-half IDT, then jump to high-half, then reinstall IDT and continue
    crate::display::kernel_write_line("4. Setting up CPU features...");
    unsafe { setup_idt(); }
    crate::display::kernel_write_line("  IDT (low-half) installed");
    crate::display::kernel_write_line("  [hh] preparing jump to high-half...");
    // Compute addresses and verify bytes before jumping to high-half
    let virt_base: u64 = KERNEL_VIRTUAL_BASE;
    let rip_now: u64; unsafe { core::arch::asm!("lea {}, [rip + 0]", out(reg) rip_now, options(nostack)); }
    if rip_now >= virt_base {
        crate::display::kernel_write_line("  [hh] already in high-half, skipping jump\n");
    } else {
        let target: u64 = virt_base.wrapping_add(rip_now);
        // Dump debug info and compare 8 bytes at low_rip vs target
        crate::display::kernel_write_line("  hh dbg: low_rip="); theseus_shared::print_hex_u64_0xe9!(rip_now);
        crate::display::kernel_write_line(" virt_base="); theseus_shared::print_hex_u64_0xe9!(virt_base);
        crate::display::kernel_write_line(" target="); theseus_shared::print_hex_u64_0xe9!(target); crate::display::kernel_write_line("\n");
        let low_q: u64 = unsafe { core::ptr::read_volatile(rip_now as *const u64) };
        let hi_q: u64  = unsafe { core::ptr::read_volatile(target as *const u64) };
        crate::display::kernel_write_line("  hh dbg: low_q="); theseus_shared::print_hex_u64_0xe9!(low_q);
        crate::display::kernel_write_line(" hi_q="); theseus_shared::print_hex_u64_0xe9!(hi_q);
        crate::display::kernel_write_line(if low_q == hi_q { " equal\n" } else { " DIFF\n" });
        if low_q == hi_q {
            unsafe { core::arch::asm!("jmp rax", in("rax") target, options(noreturn)); }
        } else {
            crate::display::kernel_write_line("  hh dbg: ABORT high-half jump due to mismatch\n");
        }
    }

    unsafe { setup_idt(); }
    crate::display::kernel_write_line("  IDT installed");
    crate::display::kernel_write_line("  Detecting CPU features...");
    unsafe {
        let features = detect_cpu_features();
        crate::display::kernel_write_line("  CPU features detected");
        crate::display::kernel_write_line("  Setting up floating point...");
        setup_floating_point(&features);
        crate::display::kernel_write_line("  Floating point setup complete");
        crate::display::kernel_write_line("  Setting up MSRs...");
        setup_msrs();
        crate::display::kernel_write_line("  MSR setup complete");
    }
    crate::display::kernel_write_line("  ✓ CPU features configured");
    
    crate::display::kernel_write_line("=== Kernel environment setup complete ===");
    crate::display::kernel_write_line("Kernel environment test completed successfully");
    
    // For testing interrupts: trigger #DE, then #GP, then #PF in separate runs
    #[allow(unreachable_code)] unsafe {
        // Select which to trigger by changing this constant index (0=DE,1=GP,2=PF)
        const WHICH: u8 = 3;
        if WHICH == 0 { // #BP via int3
            crate::display::kernel_write_line("Triggering #BP test (int3)...");
            core::arch::asm!(
                "int3",
                options(noreturn)
            );
        } else if WHICH == 1 { // #UD via ud2
            crate::display::kernel_write_line("Triggering #UD test (ud2)...");
            core::arch::asm!(
                "ud2",
                options(noreturn)
            );
        } else if WHICH == 2 { // #GP: load invalid segment selector into DS
            core::arch::asm!(
                "mov ax, 0xFFFF",
                "mov ds, ax",
                options(noreturn)
            );
        } else if WHICH == 3 { // #DE: divide by zero
            crate::display::kernel_write_line("Triggering #DE test (divide by zero)...");
            core::arch::asm!(
                "xor rax, rax",
                "mov rdx, 1",
                "div rax",
                options(noreturn)
            );
        } else { // #PF: access unmapped VA
            crate::display::kernel_write_line("Triggering #PF test (read from 0x50000000)...");
            core::arch::asm!(
                "mov rax, 0x50000000",
                "mov rax, [rax]",
                options(noreturn)
            );
        }
    }
}
