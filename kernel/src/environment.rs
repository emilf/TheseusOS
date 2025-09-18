//! Kernel environment setup module
//! 
//! This module provides functions for setting up the complete kernel environment
//! including interrupts, GDT, CPU features, and virtual memory.

use theseus_shared::handoff::Handoff;
use crate::interrupts::{disable_all_interrupts, setup_idt};
use crate::gdt::setup_gdt;
use crate::cpu::{setup_control_registers, detect_cpu_features, setup_floating_point, setup_msrs};
use crate::memory::{MemoryManager, activate_virtual_memory, KERNEL_VIRTUAL_BASE};

unsafe extern "C" fn after_high_half_entry() -> ! {
    // Reinstall IDT and continue setup now that we're in high-half
    setup_idt();
    crate::display::kernel_write_line("  IDT installed");
    crate::display::kernel_write_line("  Detecting CPU features...");
    let features = detect_cpu_features();
    crate::display::kernel_write_line("  CPU features detected");
    crate::display::kernel_write_line("  Setting up floating point...");
    setup_floating_point(&features);
    crate::display::kernel_write_line("  Floating point setup complete");
    crate::display::kernel_write_line("  Setting up MSRs...");
    setup_msrs();
    crate::display::kernel_write_line("  MSR setup complete");
    crate::display::kernel_write_line("  ✓ CPU features configured");
    
    crate::display::kernel_write_line("=== Kernel environment setup complete ===");
    crate::display::kernel_write_line("Kernel environment test completed successfully");
    
    // Exit QEMU for now; replace with scheduler/idle loop later
    theseus_shared::qemu_exit_ok!();
    loop {}
}

/// Set up complete kernel environment (correct order)
/// 
/// This function performs the setup sequence in the correct order to establish kernel control:
/// 1. Exit boot services FIRST (to prevent firmware interference)
/// 2. Disable all interrupts including NMI
/// 3. Set up GDT and TSS
/// 4. Configure control registers
/// 5. Set up CPU features
/// 6. Test basic operations
pub fn setup_kernel_environment(_handoff: &Handoff, kernel_physical_base: u64) {
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
    let phys_base: u64 = kernel_physical_base;
    let rip_now: u64; unsafe { core::arch::asm!("lea {}, [rip + 0]", out(reg) rip_now, options(nostack)); }
    if rip_now >= virt_base {
        crate::display::kernel_write_line("  [hh] already in high-half, skipping jump\n");
    } else {
        // Prefer the established virt<->phys offset from memory module
        let virt_off = crate::memory::virt_offset() as u64; // virt - phys
        // Compute target as the high-half address of our dedicated continuation function
        let cont_low: u64 = after_high_half_entry as usize as u64;
        let target: u64 = cont_low.wrapping_add(virt_off);
        let offset = rip_now.wrapping_sub(phys_base);
        // Dump debug info and compare 8 bytes at low_rip vs target (use current rip for consistency)
        crate::display::kernel_write_line("  hh dbg: phys_base="); theseus_shared::print_hex_u64_0xe9!(phys_base);
        crate::display::kernel_write_line(" low_rip="); theseus_shared::print_hex_u64_0xe9!(rip_now);
        crate::display::kernel_write_line(" offset="); theseus_shared::print_hex_u64_0xe9!(offset);
        crate::display::kernel_write_line(" virt_base="); theseus_shared::print_hex_u64_0xe9!(virt_base);
        crate::display::kernel_write_line(" target="); theseus_shared::print_hex_u64_0xe9!(target); crate::display::kernel_write_line("\n");
        let low_q: u64 = unsafe { core::ptr::read_volatile(rip_now as *const u64) };
        let hi_q: u64  = unsafe { core::ptr::read_volatile((rip_now.wrapping_add(virt_off)) as *const u64) };
        crate::display::kernel_write_line("  hh dbg: low_q="); theseus_shared::print_hex_u64_0xe9!(low_q);
        crate::display::kernel_write_line(" hi_q="); theseus_shared::print_hex_u64_0xe9!(hi_q);
        crate::display::kernel_write_line(if low_q == hi_q { " equal\n" } else { " DIFF\n" });
        if low_q == hi_q {
            unsafe { core::arch::asm!("jmp rax", in("rax") target, options(noreturn)); }
        } else {
            // Provide a clear error message using our debug macros before panicking
            theseus_shared::qemu_println!("PANIC: High-half jump verification failed (bytes mismatch)");
            theseus_shared::qemu_print!("  phys_base="); theseus_shared::print_hex_u64_0xe9!(phys_base); theseus_shared::qemu_println!("");
            theseus_shared::qemu_print!("  low_rip=");   theseus_shared::print_hex_u64_0xe9!(rip_now);   theseus_shared::qemu_println!("");
            theseus_shared::qemu_print!("  offset=");    theseus_shared::print_hex_u64_0xe9!(offset);    theseus_shared::qemu_println!("");
            theseus_shared::qemu_print!("  virt_base="); theseus_shared::print_hex_u64_0xe9!(virt_base); theseus_shared::qemu_println!("");
            theseus_shared::qemu_print!("  target=");    theseus_shared::print_hex_u64_0xe9!(target);    theseus_shared::qemu_println!("");
            theseus_shared::qemu_print!("  low_q=");     theseus_shared::print_hex_u64_0xe9!(low_q);     theseus_shared::qemu_println!("");
            theseus_shared::qemu_print!("  hi_q=");      theseus_shared::print_hex_u64_0xe9!(hi_q);      theseus_shared::qemu_println!("");
            panic!("High-half jump verification failed: bytes mismatch at target");
        }
    }

    // Unreachable if jump succeeds; if we get here, panic to avoid continuing in low-half
    theseus_shared::qemu_println!("PANIC: High-half jump did not transfer control");
    panic!("High-half jump did not transfer control");
}
