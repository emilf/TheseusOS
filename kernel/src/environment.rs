//! Kernel environment setup module
//!
//! This module provides functions for setting up the complete kernel environment
//! including interrupts, GDT, CPU features, and virtual memory.

use crate::cpu::{setup_control_registers, setup_floating_point, setup_msrs};
use crate::gdt::setup_gdt;
use crate::interrupts::{disable_all_interrupts, setup_idt};
use crate::memory::{
    activate_virtual_memory, MemoryManager, KERNEL_VIRTUAL_BASE, TEMP_HEAP_VIRTUAL_BASE,
};
use theseus_shared::handoff::Handoff;

// Small helpers to keep unsafe/asm in tiny, reviewable boundaries.
#[allow(dead_code)]
#[inline]
fn current_rip() -> u64 {
    let rip: u64;
    unsafe { core::arch::asm!("lea {}, [rip + 0]", out(reg) rip, options(nostack)) }
    rip
}

// Use centralized abort helper in `crate::boot`.
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
// Stack switching is centralized in `crate::stack::switch_to_kernel_stack_and_jump`

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
extern "C" fn after_high_half_entry() -> ! {
    // Switch stack immediately to a high-half kernel stack using the
    // centralized stack helper. Build the stack top address and jump to
    // `continue_after_stack_switch` there.
    let base = core::ptr::addr_of!(KERNEL_STACK) as u64;
    let size = core::mem::size_of::<[u8; 64 * 1024]>() as u64;
    let top_aligned = (base + size) & !0xFu64;

    // Sanity-check: ensure the kernel stack and IST stacks are mapped in high-half
    use crate::memory::{PTE_PRESENT, PTE_WRITABLE};
    if !crate::memory::virt_range_has_flags(base, size as usize, PTE_PRESENT | PTE_WRITABLE) {
        crate::boot::abort_with_context(
            "PANIC: kernel stack region not mapped or not writable in high-half",
            file!(),
            line!(),
            Some(base),
        );
    }
    for (ist_base, ist_size) in crate::gdt::ist_stack_ranges().iter().copied() {
        if !crate::memory::virt_range_has_flags(
            ist_base,
            ist_size as usize,
            PTE_PRESENT | PTE_WRITABLE,
        ) {
            crate::boot::abort_with_context(
                "PANIC: IST stack region not mapped or not writable in high-half",
                file!(),
                line!(),
                Some(ist_base),
            );
        }
    }

    unsafe {
        crate::stack::switch_to_kernel_stack_and_jump(
            top_aligned,
            continue_after_stack_switch as usize as u64,
        )
    }
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
    // Get verbose setting from the centralized kernel config
    let verbose = crate::config::VERBOSE_KERNEL_OUTPUT;
    // Reinstall IDT and continue setup now that we're in high-half
    if verbose {
        crate::display::kernel_write_line("  [hh] entered high-half");
    }
    // Debug: Verify we're running in the correct code segment and IDT is properly set up
    if verbose {
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
                let ent = base + (14 * 16) as u64; // IDT entry 14 is 16 bytes each
                let sel = core::ptr::read_unaligned((ent + 2) as *const u16) as u64; // Selector at offset 2
                let ty = core::ptr::read_unaligned((ent + 5) as *const u8) as u64; // Type/attributes at offset 5
                crate::display::kernel_write_line("  [dbg] IDT14 sel=");
                theseus_shared::print_hex_u64_0xe9!(sel);
                crate::display::kernel_write_line(" type=");
                theseus_shared::print_hex_u64_0xe9!(ty);
                crate::display::kernel_write_line("\n");
            }
        }
    }
    // Ensure TSS IST pointers are correct before installing IDT
    unsafe {
        crate::gdt::refresh_tss_ist();
    }
    setup_idt();
    if verbose {
        crate::display::kernel_write_line("  IDT installed");
    }
    // Ensure high-half runtime stacks are mapped explicitly before enabling more subsystems
    {
        use crate::handoff::handoff_phys_ptr;
        use crate::memory::{
            map_existing_region_va_to_its_pa, BootFrameAllocator, PTE_GLOBAL, PTE_NO_EXEC,
            PTE_PRESENT, PTE_WRITABLE,
        };
        use x86_64::registers::control::Cr3;
        let h = unsafe { &*(handoff_phys_ptr() as *const Handoff) };
        let (_frame, _flags) = Cr3::read();
        let pml4_pa = _frame.start_address().as_u64();
        let mut fa = unsafe { BootFrameAllocator::from_handoff(h) };
        // Map kernel main stack
        let ks_base = core::ptr::addr_of!(KERNEL_STACK) as u64;
        let ks_size = core::mem::size_of::<[u8; 64 * 1024]>() as u64;
        unsafe {
            map_existing_region_va_to_its_pa(
                pml4_pa,
                h,
                ks_base,
                ks_size,
                PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC,
                &mut fa,
            );
        }
        // Map IST stacks
        for (base, size) in crate::gdt::ist_stack_ranges().iter().copied() {
            unsafe {
                map_existing_region_va_to_its_pa(
                    pml4_pa,
                    h,
                    base,
                    size,
                    PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC,
                    &mut fa,
                );
            }
        }
    }
    // Early LAPIC timer smoke test (before enabling SSE/MSRs/allocators) to isolate issues
    {
        const EARLY_TIMER_TEST: bool = false;
        if EARLY_TIMER_TEST {
            use x86_64::instructions::interrupts;
            crate::display::kernel_write_line("  [lapic] configuring timer (early)");
            crate::interrupts::lapic_timer_configure();
            let before = crate::interrupts::timer_tick_count();
            crate::display::kernel_write_line("  [lapic] arming one-shot timer (early)");
            unsafe {
                crate::interrupts::lapic_timer_start_oneshot(100_000);
            }
            crate::display::kernel_write_line("  [lapic] enabling IF (early)");
            interrupts::enable();
            let mut ticked = false;
            for _ in 0..2_000_000 {
                if crate::interrupts::timer_tick_count() > before {
                    ticked = true;
                    break;
                }
                core::hint::spin_loop();
            }
            interrupts::disable();
            if ticked {
                crate::display::kernel_write_line("  [lapic] timer ticked (early)");
            } else {
                crate::display::kernel_write_line("  [lapic] timer did NOT tick (early)");
            }
            // Exit immediately to prevent later subsystems from masking the symptom during debugging
            theseus_shared::qemu_exit_ok!();
        }
    }
    // Ensure timer vector (0x40) has a full 64-bit handler address
    unsafe {
        crate::interrupts::install_timer_vector_runtime();
    }
    // Debug: print PF IST pointer
    if verbose {
        {
            let pf_ist = crate::gdt::get_pf_ist_top();
            crate::display::kernel_write_line("  [dbg] PF IST=");
            theseus_shared::print_hex_u64_0xe9!(pf_ist);
            crate::display::kernel_write_line("\n");
        }
    }
    // Explicitly verify IDT entries in high-half before continuing (toggle for noise control)
    const DEBUG_VERIFY_IDT_GDT: bool = false;
    if DEBUG_VERIFY_IDT_GDT {
        crate::display::kernel_write_line("  Verifying IDT entries (high-half)...");
        unsafe {
            crate::interrupts::print_idt_summary_compact();
        }
        crate::display::kernel_write_line("  IDT verification (high-half) complete");
        // Also show GDT summary to correlate selectors
        unsafe {
            crate::interrupts::print_gdt_summary_basic();
        }
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
        if verbose {
            crate::display::kernel_write_line(
                "  [cr] CR4: re-enabled OSFXSR, OSXMMEXCPT, PAGE_GLOBAL",
            );
        }
    }

    // Set up basic TLS: IA32_GS_BASE MSR and enable CR4.FSGSBASE
    const ENABLE_TLS_GS: bool = false;
    if ENABLE_TLS_GS {
        use x86_64::registers::control::{Cr4, Cr4Flags};
        use x86_64::VirtAddr;
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
        unsafe {
            Cr4::write(cr4);
        }
        crate::display::kernel_write_line("  [tls] CR4.FSGSBASE enabled");
    }

    // Enable SSE unconditionally (AVX/MSRs remain disabled for now)
    {
        let mut f = crate::cpu::CpuFeatures::new();
        f.sse = true;
        unsafe {
            setup_floating_point(&f);
        }
        if verbose {
            crate::display::kernel_write_line("  SSE enabled");
        }
    }
    // Configure MSRs that are safe to enable now (e.g., EFER.SCE)
    unsafe {
        setup_msrs();
    }
    if verbose {
        crate::display::kernel_write_line("  MSRs configured");
    }

    // Verify LAPIC timer delivery (later test)
    const ENABLE_LAPIC_TIMER_TEST: bool = true;
    if ENABLE_LAPIC_TIMER_TEST {
        use x86_64::instructions::interrupts;
        if verbose {
            crate::display::kernel_write_line("  [lapic] configuring timer");
        }
        crate::interrupts::lapic_timer_configure();
        let before = crate::interrupts::timer_tick_count();
        if verbose {
            crate::display::kernel_write_line("  [lapic] arming one-shot timer");
        }
        // Place a small stack canary near the top of the kernel stack to detect
        // whether an interrupt/handler corrupts the stack.
        let ks_base = core::ptr::addr_of!(KERNEL_STACK) as u64;
        let ks_size = core::mem::size_of::<[u8; 64 * 1024]>() as u64;
        let ks_top = ks_base.wrapping_add(ks_size);
        const STACK_CANARY: u64 = 0xDEADBEEFCAFEBABEu64;
        unsafe {
            core::ptr::write_volatile((ks_top - 8) as *mut u64, STACK_CANARY);
        }
        // Use a smaller initial count to avoid long waits if timer is slow
        unsafe {
            crate::interrupts::lapic_timer_start_oneshot(100_000);
        }
        if verbose {
            crate::display::kernel_write_line("  [lapic] enabling IF");
        }
        // Print CR3 before enabling interrupts
        if verbose {
            {
                use x86_64::registers::control::Cr3;
                let (frame_before, _f) = Cr3::read();
                crate::display::kernel_write_line("  [dbg] CR3 before IF=");
                theseus_shared::print_hex_u64_0xe9!(frame_before.start_address().as_u64());
                crate::display::kernel_write_line("\n");
            }
        }
        interrupts::enable();
        let mut _ok = false;
        for _ in 0..2_000_000 {
            if crate::interrupts::timer_tick_count() > before {
                _ok = true;
                break;
            }
            core::hint::spin_loop();
        }
        interrupts::disable();
        // Print CR3 after disabling interrupts
        if verbose {
            {
                use x86_64::registers::control::Cr3;
                let (frame_after, _f) = Cr3::read();
                crate::display::kernel_write_line("  [dbg] CR3 after disable=");
                theseus_shared::print_hex_u64_0xe9!(frame_after.start_address().as_u64());
                crate::display::kernel_write_line("\n");
            }
        }
        if verbose {
            crate::display::kernel_write_line("  [lapic] disabled IF");
            let after = crate::interrupts::timer_tick_count();
            crate::display::kernel_write_line("  [lapic] ticks(before/after)=");
            theseus_shared::print_hex_u64_0xe9!(before as u64);
            crate::display::kernel_write_line("/");
            theseus_shared::print_hex_u64_0xe9!(after as u64);
            crate::display::kernel_write_line("\n");
            if after > before {
                crate::display::kernel_write_line("  [lapic] timer interrupt received");
            } else {
                crate::display::kernel_write_line("  [lapic] timer interrupt NOT received");
            }
        }
        // Optionally mask the timer post-tick; enable to reproduce PF and test whether
        // the CALL to lapic_timer_mask() is the culprit. We perform an inline MMIO
        // write here to avoid calling into another function (which may change the
        // stack or registers) and to isolate whether the CALL itself triggers the PF.
        // Disable masking after tick while we continue debugging root cause
        const DO_MASK_AFTER_TICK: bool = false;
        if DO_MASK_AFTER_TICK {
            let rsp_before: u64;
            unsafe {
                core::arch::asm!("mov {}, rsp", out(reg) rsp_before, options(nomem, preserves_flags));
            }
            crate::display::kernel_write_line("  [lapic] rsp(before mask)=");
            theseus_shared::print_hex_u64_0xe9!(rsp_before);
            crate::display::kernel_write_line("\n");

            // Call the real mask function to reproduce the PF (if present)
            unsafe {
                crate::interrupts::lapic_timer_mask();
            }

            let rsp_after: u64;
            unsafe {
                core::arch::asm!("mov {}, rsp", out(reg) rsp_after, options(nomem, preserves_flags));
            }
            crate::display::kernel_write_line("  [lapic] rsp(after mask)=");
            theseus_shared::print_hex_u64_0xe9!(rsp_after);
            crate::display::kernel_write_line("\n");

            // Check stack canary
            let canary_read: u64 = unsafe { core::ptr::read_volatile((ks_top - 8) as *const u64) };
            if canary_read != STACK_CANARY {
                crate::display::kernel_write_line("  [lapic] STACK CANARY CORRUPTED\n");
                theseus_shared::print_hex_u64_0xe9!(canary_read);
                crate::display::kernel_write_line("\n");
                theseus_shared::qemu_exit_error!();
            } else {
                crate::display::kernel_write_line("  [lapic] stack canary intact\n");
            }
        }
        // Stress test: repeatedly arm the LAPIC one-shot timer to exercise IRQ handling
        const STRESS_TIMER: bool = false; // disabled during quick boot to ensure completion
        if STRESS_TIMER {
            use x86_64::instructions::interrupts;
            crate::display::kernel_write_line("  [lapic] starting stress test");
            let ticks_before = crate::interrupts::timer_tick_count();
            const STRESS_ITER: usize = 20; // number of re-arms (reduced for quicker boot)
            for i in 0..STRESS_ITER {
                // small count to make test quick
                unsafe {
                    crate::interrupts::lapic_timer_start_oneshot(50_000);
                }
                interrupts::enable();
                // wait for tick or timeout
                let start = crate::interrupts::timer_tick_count();
                let mut waited = 0usize;
                while crate::interrupts::timer_tick_count() == start && waited < 1_000_000 {
                    waited += 1;
                    core::hint::spin_loop();
                }
                interrupts::disable();
                if i % 100 == 0 {
                    crate::display::kernel_write_line("  [lapic] stress progress: ");
                    theseus_shared::print_hex_u64_0xe9!(i as u64);
                    crate::display::kernel_write_line("\n");
                }
            }
            let ticks_after = crate::interrupts::timer_tick_count();
            crate::display::kernel_write_line("  [lapic] stress done ticks(before/after)=");
            theseus_shared::print_hex_u64_0xe9!(ticks_before as u64);
            crate::display::kernel_write_line("/");
            theseus_shared::print_hex_u64_0xe9!(ticks_after as u64);
            crate::display::kernel_write_line("\n");
            crate::display::kernel_write_line("  [lapic] stress test complete\n");
        }
    }

    // Validate current RSP lies within our high-half kernel stack to catch bogus stack usage
    {
        let rsp_now: u64;
        unsafe {
            core::arch::asm!("mov {}, rsp", out(reg) rsp_now, options(nomem, preserves_flags));
        }
        let ks_base = core::ptr::addr_of!(KERNEL_STACK) as u64;
        let ks_size = core::mem::size_of::<[u8; 64 * 1024]>() as u64;
        let ks_top = ks_base.wrapping_add(ks_size);
        if !(rsp_now >= ks_base && rsp_now <= ks_top) {
            crate::display::kernel_write_line("  [stk] RSP outside kernel stack range");
            crate::display::kernel_write_line("  [stk] rsp=");
            theseus_shared::print_hex_u64_0xe9!(rsp_now);
            crate::display::kernel_write_line(" base=");
            theseus_shared::print_hex_u64_0xe9!(ks_base);
            crate::display::kernel_write_line(" top=");
            theseus_shared::print_hex_u64_0xe9!(ks_top);
            crate::display::kernel_write_line("\n");
            theseus_shared::qemu_exit_error!();
        }
    }

    // Initialize global allocator on a high-half VA range (mapped temp heap), then migrate to permanent heap
    {
        use crate::handoff::handoff_phys_ptr;
        let h = unsafe { &*(handoff_phys_ptr() as *const Handoff) };
        if h.temp_heap_base != 0 && h.temp_heap_size != 0 {
            // Rebase the physical temp heap to its mapped high VA and arm the global allocator shim
            let base = TEMP_HEAP_VIRTUAL_BASE as *mut u8;
            let size = h.temp_heap_size as usize;
            unsafe {
                theseus_shared::allocator::init_kernel_heap(base, size);
                // Do not switch yet; keep using pre-exit backend until permanent heap mapped
            }
            if verbose {
                crate::display::kernel_write_line(
                    "  High-half temp heap mapped for allocator shim",
                );
            }
        } else if verbose {
            crate::display::kernel_write_line("  No temp heap available for high-half allocator");
        }
    }

    // Create a permanent heap at KERNEL_HEAP_BASE and switch allocator to it
    {
        use crate::handoff::handoff_phys_ptr;
        use crate::memory::{
            map_kernel_heap_x86, unmap_identity_kernel_x86, unmap_temporary_heap_x86,
            BootFrameAllocator,
        };
        use x86_64::{
            registers::control::Cr3,
            structures::paging::{OffsetPageTable, PageTable as X86PageTable},
            VirtAddr,
        };
        let h = unsafe { &*(handoff_phys_ptr() as *const Handoff) };
        // Build a frame allocator from the handoff and map the permanent heap first
        if verbose {
            crate::display::kernel_write_line("  [perm] begin");
        }
        {
            if verbose {
                crate::display::kernel_write_line("  [perm] frame_alloc from handoff...");
            }
            let mut frame_alloc = unsafe { BootFrameAllocator::from_handoff(h) };
            if verbose {
                crate::display::kernel_write_line("  [perm] frame_alloc ready");
            }
            let (_frame, _flags) = Cr3::read();
            let pml4_pa = _frame.start_address().as_u64();
            if verbose {
                crate::display::kernel_write_line("  [perm] PML4 pa=");
                theseus_shared::print_hex_u64_0xe9!(pml4_pa);
                crate::display::kernel_write_line("\n");
            }
            let l4_va = crate::memory::phys_to_virt_pa(pml4_pa) as *mut X86PageTable;
            let l4: &mut X86PageTable = unsafe { &mut *l4_va };
            let mut mapper =
                unsafe { OffsetPageTable::new(l4, VirtAddr::new(crate::memory::PHYS_OFFSET)) };
            if verbose {
                crate::display::kernel_write_line("  [perm] mapper ready");
                crate::display::kernel_write_line("  [perm] map kernel heap...");
            }
            map_kernel_heap_x86(&mut mapper, &mut frame_alloc);
            if verbose {
                crate::display::kernel_write_line("  [perm] map kernel heap done");
            }
            // frame_alloc drops here while the allocator still points to temp heap → safe
        }
        if verbose {
            crate::display::kernel_write_line(
                "  [perm] initializing global allocator on permanent heap...",
            );
        }
        // Initialize and switch the global allocator shim to the permanent heap
        let perm_base = crate::memory::KERNEL_HEAP_BASE as *mut u8;
        let perm_size = crate::memory::KERNEL_HEAP_SIZE as usize;
        unsafe {
            theseus_shared::allocator::init_kernel_heap(perm_base, perm_size);
        }
        theseus_shared::allocator::switch_to_kernel_heap();
        if verbose {
            crate::display::kernel_write_line("  Global allocator switched to permanent heap");
        }

        // Optionally unmap the temporary heap to catch stale references
        const UNMAP_TEMP_HEAP: bool = true;
        if UNMAP_TEMP_HEAP {
            let (_frame, _flags) = Cr3::read();
            let pml4_pa = _frame.start_address().as_u64();
            let l4_va = (crate::memory::PHYS_OFFSET + pml4_pa) as *mut X86PageTable;
            let l4: &mut X86PageTable = unsafe { &mut *l4_va };
            let mut mapper =
                unsafe { OffsetPageTable::new(l4, VirtAddr::new(crate::memory::PHYS_OFFSET)) };
            unmap_temporary_heap_x86(&mut mapper, h);
            if verbose {
                crate::display::kernel_write_line("  Temporary heap unmapped");
            }
            // Also unmap identity of kernel image to catch stale low-VA code/data
            unmap_identity_kernel_x86(&mut mapper, h);
            if verbose {
                crate::display::kernel_write_line("  Identity-mapped kernel image unmapped");
            }
        }
    }

    // Initialize ACPI and driver subsystem
    {
        use crate::display::kernel_write_line;
        use crate::drivers::system;

        match system::init() {
            Ok(platform_info) => {
                kernel_write_line("[driver] platform summary");
                kernel_write_line("  CPUs: ");
                theseus_shared::print_hex_u64_0xe9!(platform_info.cpu_count as u64);
                kernel_write_line("");
                kernel_write_line("  IO APICs: ");
                theseus_shared::print_hex_u64_0xe9!(platform_info.io_apic_count as u64);
                kernel_write_line("");
                kernel_write_line("  Local APIC: 0x");
                theseus_shared::print_hex_u64_0xe9!(platform_info.local_apic_address);
                kernel_write_line("");
            }
            Err(e) => {
                kernel_write_line("[driver] initialization skipped: ");
                kernel_write_line(e);
            }
        }
    }

    crate::display::kernel_write_line("=== Kernel environment setup complete ===");
    crate::display::kernel_write_line("Kernel environment test completed successfully");

    // Set up framebuffer drawing and timer
    crate::display::kernel_write_line("Setting up framebuffer drawing...");
    crate::framebuffer::init_framebuffer_drawing();

    // Draw initial heart pattern
    unsafe {
        // Get handoff from the global static that was set in main
        if let Some(handoff) = crate::interrupts::get_handoff_for_timer() {
            crate::framebuffer::draw_initial_heart(handoff, verbose);
        }
    }

    // Configure and start the APIC timer for heart animation
    crate::display::kernel_write_line("Configuring APIC timer for heart animation...");
    unsafe {
        crate::interrupts::lapic_timer_configure();
        // Set timer to fire every ~10ms (100Hz) for smooth animation
        crate::interrupts::lapic_timer_start_periodic(100_000);
    }

    // Enable interrupts to start the timer
    x86_64::instructions::interrupts::enable();

    crate::display::kernel_write_line("Timer configured and interrupts enabled");

    // Small delay to ensure all debug bytes are emitted
    for _ in 0..1_000_00 {
        core::hint::spin_loop();
    }

    // Choose behavior based on centralized kernel configuration
    crate::display::kernel_write_line("Kernel initialization complete");
    crate::display::kernel_write_serial("Kernel initialization complete");

    // Activate serial monitor (if enabled)
    crate::monitor::init();

    if crate::config::KERNEL_SHOULD_IDLE {
        crate::display::kernel_write_line("Entering idle loop - heart animation active");
        crate::display::kernel_write_line("Kill QEMU to stop the kernel");

        // Idle loop - the timer interrupt will handle the heart animation
        loop {
            // Use halt instruction to reduce CPU usage while waiting for interrupts
            x86_64::instructions::hlt();
        }
    } else {
        crate::display::kernel_write_line("Exiting QEMU immediately...");
        theseus_shared::qemu_exit_ok!();

        loop {}
    }
}

/// Set up complete kernel environment in single-binary boot
///
/// This function performs the complete kernel initialization sequence after the
/// UEFI bootloader has exited boot services and called into the kernel library.
///
/// ## Initialization Sequence
///
/// 1. **Disable all interrupts** (including NMI) to prevent firmware interference
/// 2. **Set up GDT and TSS** for proper segmentation and task state
/// 3. **Configure control registers** (CR0, CR4) for long mode and paging
/// 4. **Establish virtual memory**:
///    - Create identity mapping for low 1 GiB
///    - Map kernel to higher-half (0xFFFFFFFF80000000)
///    - Map framebuffer to fixed virtual address
///    - Map PHYS_OFFSET linear mapping (0xFFFF800000000000)
///    - Map LAPIC MMIO region
/// 5. **Switch to new page tables** (load CR3)
/// 6. **Install IDT** for exception and interrupt handling
/// 7. **Jump to higher-half** virtual addresses
/// 8. **Initialize permanent heap** at KERNEL_HEAP_BASE
/// 9. **Enable CPU features** (SSE, MSRs)
/// 10. **Configure LAPIC timer** for interrupt testing and animation
/// 11. **Set up framebuffer** and draw initial pattern
/// 12. **Enter idle loop** or exit based on kernel configuration
///
/// # Arguments
///
/// * `_handoff` - Reference to validated handoff structure from bootloader
/// * `kernel_physical_base` - Physical base address where kernel image is loaded
/// * `verbose` - Enable detailed debug output during initialization
///
/// # Safety
///
/// This function is unsafe because it:
/// - Modifies global system state (interrupts, GDT, control registers, page tables)
/// - Performs direct memory manipulation and MMIO access
/// - Assumes handoff structure is valid and boot services are exited
/// - Switches stack and jumps to higher-half addresses
///
/// The caller must ensure:
/// - Handoff structure is properly validated
/// - UEFI boot services have been exited (no firmware calls after this)
/// - Kernel physical base address is accurate
/// - No concurrent execution (single-threaded environment)
pub fn setup_kernel_environment(_handoff: &Handoff, kernel_physical_base: u64, verbose: bool) {
    crate::display::kernel_write_line("=== Setting up Kernel Environment ===");

    // 1. Disable all interrupts first (including NMI)
    if verbose {
        crate::display::kernel_write_line("1. Disabling all interrupts...");
    }
    unsafe {
        disable_all_interrupts();
    }
    if verbose {
        crate::display::kernel_write_line("  ✓ All interrupts disabled");
    }

    // 2. Set up GDT and TSS
    if verbose {
        crate::display::kernel_write_line("2. Setting up GDT...");
    }
    unsafe {
        setup_gdt();
    }
    if verbose {
        crate::display::kernel_write_line("  ✓ GDT loaded and segments reloaded");
    }

    // 3. Configure control registers (PAE etc.)
    if verbose {
        crate::display::kernel_write_line("3. Configuring control registers...");
    }
    unsafe {
        setup_control_registers();
    }
    if verbose {
        crate::display::kernel_write_line("  ✓ Control registers configured");
    }

    // 3.5 Set up paging (identity map + high-half kernel) and load CR3
    if verbose {
        crate::display::kernel_write_line("3.5. Setting up paging...");
    }
    // Create the MemoryManager (unsafe) in an outer scope so we can use it
    // later when performing the high-half jump.
    let mm = unsafe { MemoryManager::new(_handoff) };
    unsafe {
        if verbose {
            crate::display::kernel_write_line("  [vm] before new");
        }
        if verbose {
            crate::display::kernel_write_line("  [dbg] mm returned from MemoryManager::new");
        }
        // Load CR3 earlier using the PML4 phys from the new manager
        if verbose {
            crate::display::kernel_write_line("  [vm] after new; loading CR3");
        }
        // Sanity-check the memory manager returned values before loading CR3
        {
            let pml4_pa = mm.page_table_root();
            if verbose {
                crate::display::kernel_write_line("  [chk] mm.page_table_root=");
                theseus_shared::print_hex_u64_0xe9!(pml4_pa);
                crate::display::kernel_write_line("\n");
            }
            if pml4_pa == 0 {
                crate::display::kernel_write_line("  [ERR] mm.page_table_root == 0; aborting\n");
                theseus_shared::qemu_exit_error!();
            }
            let pml4_va = crate::memory::phys_to_virt_pa(pml4_pa);
            if verbose {
                crate::display::kernel_write_line("  [chk] mm.pml4_va=");
                theseus_shared::print_hex_u64_0xe9!(pml4_va);
                crate::display::kernel_write_line("\n");
            }
            if (pml4_va as *const u8).is_null() {
                crate::display::kernel_write_line("  [ERR] pml4_va is null; aborting\n");
                theseus_shared::qemu_exit_error!();
            }
        }
        // Debug: print mm internals before loading CR3
        if verbose {
            // minimal debug: pml4 physical
            theseus_shared::print_hex_u64_0xe9!(mm.page_table_root());
        }
        activate_virtual_memory(mm.page_table_root());
        // Mark PHYS_OFFSET mapping active for later helpers
        crate::memory::set_phys_offset_active();

        // (temporary smoke test removed to avoid consuming early frames)
        if verbose {
            crate::display::kernel_write_line("  [vm] after CR3");
        }

        // Optional: probe LAPIC MMIO mapping safely (ID/Version) after paging
        const PROBE_LAPIC_AFTER_PAGING: bool = false;
        if PROBE_LAPIC_AFTER_PAGING {
            crate::interrupts::mask_lapic_after_paging();
        }

        // LAPIC timer setup deferred until after IDT install (to keep boot stable)

        // Defer IDT installation to step 4 (pre-jump) to avoid early faults

        {
            use x86_64::{
                structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate},
                VirtAddr,
            };
            // Initialize mapper after CR3 load using PHYS_OFFSET for physical mapping base
            let l4: &mut X86PageTable = &mut *(mm.pml4 as *mut _ as *mut X86PageTable);
            let mapper = OffsetPageTable::new(l4, VirtAddr::new(crate::memory::PHYS_OFFSET));
            let _ = mapper.translate_addr(VirtAddr::new(KERNEL_VIRTUAL_BASE));
        }
    }
    if verbose {
        crate::display::kernel_write_line("  ✓ Paging enabled (identity + high-half kernel)");
    }

    // 4. Install low-half IDT, then jump to high-half, then reinstall IDT and continue
    if verbose {
        crate::display::kernel_write_line("4. Setting up CPU features...");
    }
    // Install low-half IDT and proceed directly to high-half
    unsafe {
        setup_idt();
    }
    if verbose {
        crate::display::kernel_write_line("  IDT (low-half) installed");
    }
    if verbose {
        crate::display::kernel_write_line("  [hh] preparing jump to high-half...");
    }
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
        if verbose {
            crate::display::kernel_write_line("  [hh] already in high-half, skipping jump\n");
        }
    } else {
        // Delegate the translation/verification/jump to the memory subsystem.
        unsafe {
            mm.jump_to_high_half(phys_base, after_high_half_entry);
        }
    }

    // Unreachable if jump succeeds; if we get here, panic to avoid continuing in low-half
    theseus_shared::qemu_println!("PANIC: High-half jump did not transfer control");
    panic!("High-half jump did not transfer control");
}
