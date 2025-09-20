//! Kernel environment setup module
//! 
//! This module provides functions for setting up the complete kernel environment
//! including interrupts, GDT, CPU features, and virtual memory.

use theseus_shared::handoff::Handoff;
use crate::interrupts::{disable_all_interrupts, setup_idt};
use crate::gdt::setup_gdt;
use crate::cpu::{setup_control_registers, setup_floating_point, setup_msrs};
use crate::memory::{MemoryManager, activate_virtual_memory, KERNEL_VIRTUAL_BASE, TEMP_HEAP_VIRTUAL_BASE};

#[link_section = ".bss.stack"]
static mut KERNEL_STACK: [u8; 64 * 1024] = [0; 64 * 1024];

#[inline(never)]
unsafe fn switch_to_high_stack_and_continue() -> ! {
    extern "C" fn high_stack_main() -> ! { unsafe { continue_after_stack_switch() } }
    let base = core::ptr::addr_of!(KERNEL_STACK) as u64;
    let size = core::mem::size_of::<[u8; 64 * 1024]>() as u64;
    let top_aligned = (base + size) & !0xFu64;
    core::arch::asm!(
        "mov rsp, {stack_top}",
        "jmp {cont}",
        stack_top = in(reg) top_aligned,
        cont = sym high_stack_main,
        options(noreturn)
    );
}

unsafe extern "C" fn after_high_half_entry() -> ! {
    // Switch stack immediately to a high-half kernel stack
    switch_to_high_stack_and_continue();
}

pub(super) unsafe fn continue_after_stack_switch() -> ! {
    // Reinstall IDT and continue setup now that we're in high-half
    crate::display::kernel_write_line("  [hh] entered high-half");
    // Debug: print current CS and expected KERNEL_CS
    {
        let cs_val: u16; unsafe { core::arch::asm!("mov {0:x}, cs", out(reg) cs_val, options(nomem, nostack, preserves_flags)); }
        crate::display::kernel_write_line("  [dbg] CS="); theseus_shared::print_hex_u64_0xe9!(cs_val as u64); crate::display::kernel_write_line(" expected="); theseus_shared::print_hex_u64_0xe9!(crate::gdt::KERNEL_CS as u64); crate::display::kernel_write_line("\n");
        // Print IDT[14] selector/type_attr to ensure correct gate
        unsafe {
            use x86_64::instructions::tables::sidt;
            let idtr = sidt();
            let base = idtr.base.as_u64();
            let ent = base + (14 * 16) as u64;
            let sel = core::ptr::read_unaligned((ent + 2) as *const u16) as u64;
            let ty  = core::ptr::read_unaligned((ent + 5) as *const u8) as u64;
            crate::display::kernel_write_line("  [dbg] IDT14 sel="); theseus_shared::print_hex_u64_0xe9!(sel);
            crate::display::kernel_write_line(" type="); theseus_shared::print_hex_u64_0xe9!(ty); crate::display::kernel_write_line("\n");
        }
    }
    setup_idt();
    crate::display::kernel_write_line("  IDT installed");
    // Explicitly verify IDT entries in high-half before continuing
    crate::display::kernel_write_line("  Verifying IDT entries (high-half)...");
    unsafe { crate::interrupts::print_idt_summary_compact(); }
    crate::display::kernel_write_line("  IDT verification (high-half) complete");
    // Skip CPU feature detection and SSE for now to keep high-half path stable
    // Re-enable safe CR4 bits now that paging is active
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

    // Initialize global allocator on a high-half VA range (mapped temp heap), then migrate to permanent heap
    {
        use crate::handoff::handoff_phys_ptr;
        let h = unsafe { &*(handoff_phys_ptr() as *const Handoff) };
        if h.temp_heap_base != 0 && h.temp_heap_size != 0 {
            // Rebase the physical temp heap to its mapped high VA
            let base = TEMP_HEAP_VIRTUAL_BASE as *mut u8;
            let size = h.temp_heap_size as usize;
            unsafe { crate::allocator::ALLOCATOR_LINKED.lock().init(base, size); }
            crate::display::kernel_write_line("  High-half heap initialized");
            // Quick allocation probe to validate the heap works before heavier use
            {
                use alloc::boxed::Box;
                let bx = Box::new(0xDEADBEEFu64);
                crate::display::kernel_write_line("  [alloc] Box<u64> at ");
                theseus_shared::print_hex_u64_0xe9!((&*bx as *const u64) as u64);
                crate::display::kernel_write_line("\n");
                core::mem::drop(bx);
            }
        } else {
            crate::display::kernel_write_line("  No temp heap available for high-half allocator");
        }
    }

    // Create a permanent heap at KERNEL_HEAP_BASE and switch allocator to it
    {
        use x86_64::{VirtAddr, registers::control::Cr3, structures::paging::{OffsetPageTable, PageTable as X86PageTable}};
        use crate::memory::{BootFrameAllocator, map_kernel_heap_x86, unmap_temporary_heap_x86, unmap_identity_kernel_x86};
        use crate::handoff::handoff_phys_ptr;
        let h = unsafe { &*(handoff_phys_ptr() as *const Handoff) };
        // Build a frame allocator from the handoff and map the permanent heap first
        crate::display::kernel_write_line("  [perm] begin");
        {
            crate::display::kernel_write_line("  [perm] frame_alloc from handoff...");
            let mut frame_alloc = unsafe { BootFrameAllocator::from_handoff(h) };
            crate::display::kernel_write_line("  [perm] frame_alloc ready");
            let (_frame, _flags) = Cr3::read();
            let pml4_pa = _frame.start_address().as_u64();
            crate::display::kernel_write_line("  [perm] PML4 pa=");
            theseus_shared::print_hex_u64_0xe9!(pml4_pa);
            crate::display::kernel_write_line("\n");
            let l4: &mut X86PageTable = unsafe { &mut *(pml4_pa as *mut X86PageTable) };
            let mut mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(0)) };
            crate::display::kernel_write_line("  [perm] mapper ready");
            crate::display::kernel_write_line("  [perm] map kernel heap...");
            map_kernel_heap_x86(&mut mapper, &mut frame_alloc);
            crate::display::kernel_write_line("  [perm] map kernel heap done");
            // frame_alloc drops here while the allocator still points to temp heap → safe
        }
        crate::display::kernel_write_line("  [perm] switching allocator to permanent heap...");
        // Now switch the global allocator to the permanent heap
        let perm_base = crate::memory::KERNEL_HEAP_BASE as *mut u8;
        let perm_size = crate::memory::KERNEL_HEAP_SIZE as usize;
        unsafe { crate::allocator::ALLOCATOR_LINKED.lock().init(perm_base, perm_size); }
        crate::display::kernel_write_line("  Permanent kernel heap initialized");

        // Optionally unmap the temporary heap to catch stale references
        const UNMAP_TEMP_HEAP: bool = true;
        if UNMAP_TEMP_HEAP {
            let (_frame, _flags) = Cr3::read();
            let pml4_pa = _frame.start_address().as_u64();
            let l4: &mut X86PageTable = unsafe { &mut *(pml4_pa as *mut X86PageTable) };
            let mut mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(0)) };
            unmap_temporary_heap_x86(&mut mapper, h);
            crate::display::kernel_write_line("  Temporary heap unmapped");
            // Also unmap identity of kernel image to catch stale low-VA code/data
            unmap_identity_kernel_x86(&mut mapper, h);
            crate::display::kernel_write_line("  Identity-mapped kernel image unmapped");
        }
    }
    
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

        {
            use x86_64::{VirtAddr, structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate}};
            // Initialize mapper after CR3 load using PHYS_OFFSET for physical mapping base
            let l4: &mut X86PageTable = &mut *(mm.pml4 as *mut _ as *mut X86PageTable);
            let mapper = OffsetPageTable::new(l4, VirtAddr::new(crate::memory::PHYS_OFFSET));
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
        {
            // Verify mapping exists for target VA before jumping
            use x86_64::{VirtAddr, registers::control::Cr3, structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate}};
            let (_frame, _flags) = Cr3::read();
            let pml4_pa = _frame.start_address().as_u64();
            let l4: &mut X86PageTable = unsafe { &mut *(pml4_pa as *mut X86PageTable) };
            let mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(crate::memory::PHYS_OFFSET)) };
            // Debug: dump PML4[HH] entry value
            let hh_index = ((KERNEL_VIRTUAL_BASE >> 39) & 0x1FF) as usize;
            let pml4_entry_val = unsafe { core::ptr::read_volatile((pml4_pa as *const u64).add(hh_index)) };
            crate::display::kernel_write_line("  [hh] PML4[HH]=");
            theseus_shared::print_hex_u64_0xe9!(pml4_entry_val);
            crate::display::kernel_write_line("\n");
            let phys = mapper.translate_addr(VirtAddr::new(target));
            crate::display::kernel_write_line("  [hh] target phys=");
            if let Some(pa) = phys { theseus_shared::print_hex_u64_0xe9!(pa.as_u64()); } else { theseus_shared::qemu_println!("NONE"); }
            crate::display::kernel_write_line("\n");
            crate::display::kernel_write_line("  [hh] jumping to high-half (via virt_off)");
            unsafe { core::arch::asm!("jmp rax", in("rax") target, options(noreturn)); }
        }
        // legacy path removed
    }

    // Unreachable if jump succeeds; if we get here, panic to avoid continuing in low-half
    theseus_shared::qemu_println!("PANIC: High-half jump did not transfer control");
    panic!("High-half jump did not transfer control");
}
