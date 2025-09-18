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
        // Load CR3 earlier using the PML4 phys from the new manager
        crate::display::kernel_write_line("  [vm] after new; loading CR3");
        activate_virtual_memory(mm.page_table_root());
        crate::display::kernel_write_line("  [vm] after CR3");

        #[cfg(feature = "new_arch")]
        {
            use x86_64::{VirtAddr, PhysAddr, structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate, Page, PhysFrame, Size4KiB, PageTableFlags, Mapper}};
            use crate::memory::BootFrameAllocator;
            // Initialize mapper after CR3 load using identity phys-mem offset (0)
            let l4: &mut X86PageTable = &mut *(mm.pml4 as *mut _ as *mut X86PageTable);
            let mapper = OffsetPageTable::new(l4, VirtAddr::new(0));
            let _ = mapper.translate_addr(VirtAddr::new(KERNEL_VIRTUAL_BASE));

            // Gradually migrate framebuffer mapping using the mapper (idempotent)
            if _handoff.gop_fb_base != 0 && _handoff.gop_fb_size != 0 {
                let mut mapper = OffsetPageTable::new(l4, VirtAddr::new(0));
                let mut frame_alloc = BootFrameAllocator::from_handoff(_handoff);
                let fb_pa = _handoff.gop_fb_base;
                let fb_va = 0xFFFFFFFF90000000u64;
                let pages = ((_handoff.gop_fb_size + 4095) / 4096) as u64;
                let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_EXECUTE;
                let mut mapped_count: u64 = 0;
                for i in 0..pages {
                    let pa = PhysAddr::new(fb_pa + i * 4096);
                    let frame = PhysFrame::<Size4KiB>::containing_address(pa);
                    let page = Page::<Size4KiB>::containing_address(VirtAddr::new(fb_va + i * 4096));
                    match mapper.map_to(page, frame, flags, &mut frame_alloc) {
                        Ok(flush) => { flush.flush(); mapped_count += 1; },
                        Err(_e) => { /* Already mapped or parent huge page; keep legacy mapping */ }
                    }
                }
                let _ = mapped_count;
            }

            // Migrate temporary heap mapping via mapper (idempotent)
            // Migrate kernel image mapping via mapper (idempotent)
            {
                let mut mapper = OffsetPageTable::new(l4, VirtAddr::new(0));
                let mut frame_alloc = BootFrameAllocator::from_handoff(_handoff);
                let phys_base = _handoff.kernel_physical_base;
                let size = _handoff.kernel_image_size;
                if phys_base != 0 && size != 0 {
                    let pages = ((size + 4095) / 4096) as u64;
                    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE; // executable
                    let mut mapped_count: u64 = 0;
                    for i in 0..pages {
                        let pa = PhysAddr::new(phys_base + i * 4096);
                        let frame = PhysFrame::<Size4KiB>::containing_address(pa);
                        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(crate::memory::KERNEL_VIRTUAL_BASE + i * 4096));
                        match mapper.map_to(page, frame, flags, &mut frame_alloc) {
                            Ok(flush) => { flush.flush(); mapped_count += 1; },
                            Err(_e) => { /* Already mapped; fine */ }
                        }
                    }
                    let _ = mapped_count;
                }
            }
            if _handoff.temp_heap_base != 0 && _handoff.temp_heap_size != 0 {
                let mut mapper = OffsetPageTable::new(l4, VirtAddr::new(0));
                let mut frame_alloc = BootFrameAllocator::from_handoff(_handoff);
                let heap_pa = _handoff.temp_heap_base;
                let heap_va = 0xFFFFFFFFA0000000u64;
                let pages = ((_handoff.temp_heap_size + 4095) / 4096) as u64;
                let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_EXECUTE;
                let mut mapped_count: u64 = 0;
                for i in 0..pages {
                    let pa = PhysAddr::new(heap_pa + i * 4096);
                    let frame = PhysFrame::<Size4KiB>::containing_address(pa);
                    let page = Page::<Size4KiB>::containing_address(VirtAddr::new(heap_va + i * 4096));
                    match mapper.map_to(page, frame, flags, &mut frame_alloc) {
                        Ok(flush) => { flush.flush(); mapped_count += 1; },
                        Err(_e) => { /* Already mapped; fine */ }
                    }
                }
                let _ = mapped_count;
            }
        }
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
