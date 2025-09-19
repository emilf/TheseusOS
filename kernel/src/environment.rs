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
    crate::display::kernel_write_line("  [hh] entered high-half");
    setup_idt();
    crate::display::kernel_write_line("  IDT installed");
    // Explicitly verify IDT entries in high-half before continuing
    crate::display::kernel_write_line("  Verifying IDT entries (high-half)...");
    unsafe { crate::interrupts::print_idt_summary_compact(); }
    crate::display::kernel_write_line("  IDT verification (high-half) complete");
    // Skip CPU feature detection and SSE for now to keep high-half path stable
    // Re-enable safe CR4 bits now that paging is active
    #[cfg(feature = "new_arch")]
    {
        use x86_64::registers::control::{Cr4, Cr4Flags};
        let mut f4 = Cr4::read();
        f4.insert(Cr4Flags::OSFXSR);
        f4.insert(Cr4Flags::OSXMMEXCPT_ENABLE);
        f4.insert(Cr4Flags::PAGE_GLOBAL);
        Cr4::write(f4);
        crate::display::kernel_write_line("  [cr] CR4: re-enabled OSFXSR, OSXMMEXCPT, PAGE_GLOBAL");
    }
    // Enable SSE unconditionally (AVX/MSRs remain disabled for now)
    {
        let mut f = crate::cpu::CpuFeatures::new();
        f.sse = true;
        unsafe { setup_floating_point(&f); }
        crate::display::kernel_write_line("  SSE enabled");
    }
    // Configure MSRs that are safe to enable now (e.g., EFER.SCE)
    unsafe {
        setup_msrs();
    }
    crate::display::kernel_write_line("  MSRs configured");
    
    crate::display::kernel_write_line("=== Kernel environment setup complete ===");
    crate::display::kernel_write_line("Kernel environment test completed successfully");
    // Small delay to ensure all debug bytes are emitted before exit
    for _ in 0..1_000_00 { core::hint::spin_loop(); }
    
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
        // Load CR3 earlier using the PML4 phys from the new manager
        crate::display::kernel_write_line("  [vm] after new; loading CR3");
        activate_virtual_memory(mm.page_table_root());
        crate::display::kernel_write_line("  [vm] after CR3");

        // Defer IDT installation to step 4 (pre-jump) to avoid early faults

        #[cfg(feature = "new_arch")]
        {
            use x86_64::{VirtAddr, structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate}};
            // Initialize mapper after CR3 load using identity phys-mem offset (0);
            // rely on pre-CR3 legacy mappings for framebuffer, heap, and kernel image.
            let l4: &mut X86PageTable = &mut *(mm.pml4 as *mut _ as *mut X86PageTable);
            let mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(0)) };
            let _ = mapper.translate_addr(VirtAddr::new(KERNEL_VIRTUAL_BASE));
        }
    }
    crate::display::kernel_write_line("  ✓ Paging enabled (identity + high-half kernel)");
    
    // 4. Install low-half IDT, then jump to high-half, then reinstall IDT and continue
    crate::display::kernel_write_line("4. Setting up CPU features...");
    // Install low-half IDT and proceed directly to high-half
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
        // Compute target HH VA from current symbol address: (sym - phys_base) + KERNEL_VIRTUAL_BASE
        let sym: u64 = after_high_half_entry as usize as u64;
        let target: u64 = sym.wrapping_sub(phys_base).wrapping_add(KERNEL_VIRTUAL_BASE);
        let offset = rip_now.wrapping_sub(phys_base);
        // Dump debug info and compare 8 bytes at low_rip vs target (use current rip for consistency)
        crate::display::kernel_write_line("  hh dbg: phys_base="); theseus_shared::print_hex_u64_0xe9!(phys_base);
        crate::display::kernel_write_line(" low_rip="); theseus_shared::print_hex_u64_0xe9!(rip_now);
        crate::display::kernel_write_line(" offset="); theseus_shared::print_hex_u64_0xe9!(offset);
        crate::display::kernel_write_line(" virt_base="); theseus_shared::print_hex_u64_0xe9!(virt_base);
        crate::display::kernel_write_line(" target="); theseus_shared::print_hex_u64_0xe9!(target); crate::display::kernel_write_line("\n");
        #[cfg(feature = "new_arch")]
        {
            // Verify mapping exists for target VA before jumping
            use x86_64::{VirtAddr, registers::control::Cr3, structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate}};
            let (_frame, _flags) = Cr3::read();
            let pml4_pa = _frame.start_address().as_u64();
            let l4: &mut X86PageTable = unsafe { &mut *(pml4_pa as *mut X86PageTable) };
            let mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(0)) };
            let phys = mapper.translate_addr(VirtAddr::new(target));
            crate::display::kernel_write_line("  [hh] target phys=");
            if let Some(pa) = phys { theseus_shared::print_hex_u64_0xe9!(pa.as_u64()); } else { theseus_shared::qemu_println!("NONE"); }
            crate::display::kernel_write_line("\n");
            crate::display::kernel_write_line("  [hh] jumping to high-half (via virt_off)");
            unsafe { core::arch::asm!("jmp rax", in("rax") target, options(noreturn)); }
        }
        #[cfg(not(feature = "new_arch"))]
        {
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
    }

    // Unreachable if jump succeeds; if we get here, panic to avoid continuing in low-half
    theseus_shared::qemu_println!("PANIC: High-half jump did not transfer control");
    panic!("High-half jump did not transfer control");
}
