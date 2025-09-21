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

/// Switch to high-half kernel stack and continue execution
/// 
/// This function performs a critical transition from the low-half (identity-mapped)
/// kernel to the high-half (virtual-mapped) kernel. It switches to a dedicated
/// kernel stack and jumps to the high-half continuation function.
/// 
/// # Assembly Details
/// - `mov rsp, {stack_top}`: Load the stack pointer with the top of our kernel stack
/// - `jmp {cont}`: Jump to the high-half continuation function
/// 
/// # Safety
/// 
/// This function is unsafe because it:
/// - Modifies the stack pointer directly
/// - Performs a non-returning jump
/// - Assumes the kernel stack is properly aligned and accessible
/// 
/// The caller must ensure:
/// - The kernel stack is properly allocated and aligned
/// - The stack memory is accessible and not corrupted
/// - The high-half continuation function is valid and properly mapped
/// - No other code is using the stack during the transition
#[inline(never)]
unsafe fn switch_to_high_stack_and_continue() -> ! {
    extern "C" fn high_stack_main() -> ! { unsafe { continue_after_stack_switch() } }
    let base = core::ptr::addr_of!(KERNEL_STACK) as u64;
    let size = core::mem::size_of::<[u8; 64 * 1024]>() as u64;
    let top_aligned = (base + size) & !0xFu64;
    core::arch::asm!(
        "mov rsp, {stack_top}",  // Set stack pointer to top of kernel stack
        "jmp {cont}",            // Jump to high-half continuation function
        stack_top = in(reg) top_aligned,
        cont = sym high_stack_main,
        options(noreturn)
    );
}

/// High-half entry point function
/// 
/// This function is called after jumping to the high-half virtual address space.
/// It immediately switches to the high-half kernel stack and continues execution.
/// 
/// # Safety
/// 
/// This function is unsafe because it:
/// - Calls unsafe functions that modify the stack pointer
/// - Assumes the high-half environment is properly set up
/// - Performs non-returning operations
/// 
/// The caller must ensure:
/// - Virtual memory is properly configured
/// - High-half mappings are active
/// - The kernel stack is accessible in high-half space
unsafe extern "C" fn after_high_half_entry() -> ! {
    // Switch stack immediately to a high-half kernel stack
    switch_to_high_stack_and_continue();
}

/// Continue kernel setup after switching to high-half stack
/// 
/// This function completes the kernel initialization process after transitioning
/// to the high-half virtual address space. It sets up the IDT, configures CPU
/// features, initializes memory management, and prepares the system for normal operation.
/// 
/// # Safety
/// 
/// This function is unsafe because it:
/// - Modifies global system state (IDT, CR4, MSRs)
/// - Performs memory allocation operations
/// - Assumes the high-half environment is properly configured
/// 
/// The caller must ensure:
/// - We are running in high-half virtual address space
/// - Virtual memory mappings are active and correct
/// - The kernel stack is properly set up
/// - No other code is modifying system state concurrently
pub(super) unsafe fn continue_after_stack_switch() -> ! {
    // Reinstall IDT and continue setup now that we're in high-half
    crate::display::kernel_write_line("  [hh] entered high-half");
    // Debug: Verify we're running in the correct code segment and IDT is properly set up
    {
        // Read current code segment selector to verify we're using the kernel CS
        let cs_val: u16; 
        unsafe { 
            core::arch::asm!(
                "mov {0:x}, cs",  // Read code segment register
                out(reg) cs_val, 
                options(nomem, nostack, preserves_flags)
            ); 
        }
        crate::display::kernel_write_line("  [dbg] CS="); 
        theseus_shared::print_hex_u64_0xe9!(cs_val as u64); 
        crate::display::kernel_write_line(" expected="); 
        theseus_shared::print_hex_u64_0xe9!(crate::gdt::KERNEL_CS as u64); 
        crate::display::kernel_write_line("\n");
        
        // Verify IDT entry 14 (Page Fault) is properly configured
        unsafe {
            use x86_64::instructions::tables::sidt;
            let idtr = sidt();
            let base = idtr.base.as_u64();
            let ent = base + (14 * 16) as u64;  // IDT entry 14 is 16 bytes each
            let sel = core::ptr::read_unaligned((ent + 2) as *const u16) as u64;  // Selector at offset 2
            let ty  = core::ptr::read_unaligned((ent + 5) as *const u8) as u64;   // Type/attributes at offset 5
            crate::display::kernel_write_line("  [dbg] IDT14 sel="); 
            theseus_shared::print_hex_u64_0xe9!(sel);
            crate::display::kernel_write_line(" type="); 
            theseus_shared::print_hex_u64_0xe9!(ty); 
            crate::display::kernel_write_line("\n");
        }
    }
    setup_idt();
    crate::display::kernel_write_line("  IDT installed");
    // Ensure timer vector (0x40) has a full 64-bit handler address
    unsafe { crate::interrupts::install_timer_vector_runtime(); }
    // Explicitly verify IDT entries in high-half before continuing
    crate::display::kernel_write_line("  Verifying IDT entries (high-half)...");
    unsafe { crate::interrupts::print_idt_summary_compact(); }
    crate::display::kernel_write_line("  IDT verification (high-half) complete");
    // Also show GDT summary to correlate selectors
    unsafe { crate::interrupts::print_gdt_summary_basic(); }
    // Inspect timer vector (0x40) raw entry
    unsafe {
        use x86_64::instructions::tables::sidt;
        let idtr = sidt();
        let base = idtr.base.as_u64();
        let ent = base + (0x40 * 16) as u64; // 16-byte entries
        let lo = core::ptr::read_unaligned(ent as *const u64);
        let hi = core::ptr::read_unaligned((ent + 8) as *const u64);
        crate::display::kernel_write_line("  [dbg] IDT[0x40] lo=");
        theseus_shared::print_hex_u64_0xe9!(lo);
        crate::display::kernel_write_line(" hi=");
        theseus_shared::print_hex_u64_0xe9!(hi);
        crate::display::kernel_write_line("\n");
    }
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
    
    // Set up basic TLS: IA32_GS_BASE MSR and enable CR4.FSGSBASE
    use x86_64::VirtAddr;
    use x86_64::registers::control::{Cr4, Cr4Flags};
    
    // Set a dummy GS base for now (we'll use this for per-CPU data later)
    let dummy_gs_base = VirtAddr::new(0xFFFF800000000000); // Use our PHYS_OFFSET for now
    unsafe {
        // Use the correct MSR constant for IA32_GS_BASE
        core::arch::asm!(
            "wrmsr",
            in("ecx") 0xC0000101u32,  // IA32_GS_BASE MSR
            in("eax") (dummy_gs_base.as_u64() & 0xFFFFFFFF) as u32,
            in("edx") (dummy_gs_base.as_u64() >> 32) as u32,
            options(nostack, preserves_flags)
        );
        crate::display::kernel_write_line("  [tls] IA32_GS_BASE set");
    }
    
    // Now enable CR4.FSGSBASE safely
    let mut cr4 = Cr4::read();
    cr4.insert(Cr4Flags::FSGSBASE);
    unsafe { Cr4::write(cr4); }
    crate::display::kernel_write_line("  [tls] CR4.FSGSBASE enabled");
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

    // Verify LAPIC timer delivery (one-shot hardware interrupt)
    const ENABLE_LAPIC_TIMER_TEST: bool = true;
    if ENABLE_LAPIC_TIMER_TEST {
        use x86_64::instructions::interrupts;
        crate::display::kernel_write_line("  [lapic] configuring timer");
        crate::interrupts::lapic_timer_configure();
        let before = crate::interrupts::timer_tick_count();
        crate::display::kernel_write_line("  [lapic] ticks(before)=");
        theseus_shared::print_hex_u64_0xe9!(before as u64);
        crate::display::kernel_write_line("\n");
        crate::display::kernel_write_line("  [lapic] arming one-shot timer");
        // Use a smaller initial count to avoid long waits if timer is slow
        unsafe { crate::interrupts::lapic_timer_start_oneshot(100_000); }
        crate::display::kernel_write_line("  [lapic] enabling IF");
        interrupts::enable();
        let mut ok = false;
        for _ in 0..2_000_000 {
            if crate::interrupts::timer_tick_count() > before { ok = true; break; }
            core::hint::spin_loop();
        }
        interrupts::disable();
        crate::display::kernel_write_line("  [lapic] disabled IF");
        let after = crate::interrupts::timer_tick_count();
        crate::display::kernel_write_line("  [lapic] ticks(after)=");
        theseus_shared::print_hex_u64_0xe9!(after as u64);
        crate::display::kernel_write_line("\n");
        if ok { crate::display::kernel_write_line("  [lapic] timer interrupt received"); }
        else { crate::display::kernel_write_line("  [lapic] timer interrupt NOT received"); }
        unsafe { crate::interrupts::lapic_timer_mask(); }
    }

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
/// 
/// # Safety
/// 
/// This function is unsafe because it:
/// - Modifies global system state (interrupts, GDT, control registers)
/// - Performs memory management operations
/// - Assumes the handoff structure is valid and properly initialized
/// 
/// The caller must ensure:
/// - The handoff structure contains valid system information
/// - No other code is running concurrently
/// - The kernel physical base address is correct
/// - UEFI boot services have been exited
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

        // Optional: probe LAPIC MMIO mapping safely (ID/Version) after paging
        const PROBE_LAPIC_AFTER_PAGING: bool = false;
        if PROBE_LAPIC_AFTER_PAGING {
            crate::interrupts::mask_lapic_after_paging();
        }

        // LAPIC timer setup deferred until after IDT install (to keep boot stable)

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
    // Prepare for high-half transition: compute addresses and verify mappings
    let virt_base: u64 = KERNEL_VIRTUAL_BASE;
    let phys_base: u64 = kernel_physical_base;
    
    // Get current instruction pointer to determine if we're already in high-half
    let rip_now: u64; 
    unsafe { 
        core::arch::asm!(
            "lea {}, [rip + 0]",  // Load effective address of current instruction
            out(reg) rip_now, 
            options(nostack)
        ); 
    }
    
    if rip_now >= virt_base {
        crate::display::kernel_write_line("  [hh] already in high-half, skipping jump\n");
    } else {
        // Calculate the virtual address of our high-half entry point
        // Formula: (physical_symbol - physical_base) + virtual_base
        let sym: u64 = after_high_half_entry as usize as u64;
        let target: u64 = sym.wrapping_sub(phys_base).wrapping_add(KERNEL_VIRTUAL_BASE);
        let offset = rip_now.wrapping_sub(phys_base);
        
        // Debug output: show address translation details
        crate::display::kernel_write_line("  hh dbg: phys_base="); 
        theseus_shared::print_hex_u64_0xe9!(phys_base);
        crate::display::kernel_write_line(" low_rip="); 
        theseus_shared::print_hex_u64_0xe9!(rip_now);
        crate::display::kernel_write_line(" offset="); 
        theseus_shared::print_hex_u64_0xe9!(offset);
        crate::display::kernel_write_line(" virt_base="); 
        theseus_shared::print_hex_u64_0xe9!(virt_base);
        crate::display::kernel_write_line(" target="); 
        theseus_shared::print_hex_u64_0xe9!(target); 
        crate::display::kernel_write_line("\n");
        
        {
            // Verify that the target virtual address is properly mapped before jumping
            use x86_64::{VirtAddr, registers::control::Cr3, structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate}};
            let (_frame, _flags) = Cr3::read();
            let pml4_pa = _frame.start_address().as_u64();
            let l4: &mut X86PageTable = unsafe { &mut *(pml4_pa as *mut X86PageTable) };
            let mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(crate::memory::PHYS_OFFSET)) };
            
            // Debug: Check PML4 entry for high-half region (bits 47:39 of virtual address)
            let hh_index = ((KERNEL_VIRTUAL_BASE >> 39) & 0x1FF) as usize;
            let pml4_entry_val = unsafe { core::ptr::read_volatile((pml4_pa as *const u64).add(hh_index)) };
            crate::display::kernel_write_line("  [hh] PML4[HH]=");
            theseus_shared::print_hex_u64_0xe9!(pml4_entry_val);
            crate::display::kernel_write_line("\n");
            
            // Verify the target address translates to a valid physical address
            let phys = mapper.translate_addr(VirtAddr::new(target));
            crate::display::kernel_write_line("  [hh] target phys=");
            if let Some(pa) = phys { 
                theseus_shared::print_hex_u64_0xe9!(pa.as_u64()); 
            } else { 
                theseus_shared::qemu_println!("NONE"); 
            }
            crate::display::kernel_write_line("\n");
            
            // Perform the jump to high-half virtual address
            crate::display::kernel_write_line("  [hh] jumping to high-half (via virt_off)");
            unsafe { 
                core::arch::asm!(
                    "jmp rax",  // Jump to the calculated virtual address
                    in("rax") target, 
                    options(noreturn)
                ); 
            }
        }
    }

    // Unreachable if jump succeeds; if we get here, panic to avoid continuing in low-half
    theseus_shared::qemu_println!("PANIC: High-half jump did not transfer control");
    panic!("High-half jump did not transfer control");
}
