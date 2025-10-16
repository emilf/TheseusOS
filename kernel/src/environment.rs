//! Kernel environment setup module
//!
//! This module provides functions for setting up the complete kernel environment
//! including interrupts, GDT, CPU features, virtual memory, and UEFI runtime hooks.

use alloc::{format, vec::Vec};
use alloc::string::String;
use core::slice;
use crate::{log_debug, log_error, log_info, log_warn};
use crate::cpu::{setup_control_registers, setup_floating_point, setup_msrs};
use crate::gdt::setup_gdt;
use crate::interrupts::{disable_all_interrupts, setup_idt};
use crate::memory::{
    activate_virtual_memory, BootFrameAllocator, MemoryManager, KERNEL_VIRTUAL_BASE,
    TEMP_HEAP_VIRTUAL_BASE,
};
use crate::serial_debug;
use theseus_shared::handoff::Handoff;
use uefi::mem::memory_map::{MemoryAttribute, MemoryDescriptor};
use uefi::Status;
use x86_64::structures::paging::FrameAllocator;

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
#[no_mangle]
pub extern "C" fn after_high_half_entry() -> ! {
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
#[no_mangle]
pub unsafe extern "C" fn continue_after_stack_switch() -> ! {
    // Reinstall IDT and continue setup now that we're in high-half
    log_debug!("Entered high-half virtual memory");
    
    // Ensure TSS IST pointers are correct before installing IDT
    unsafe {
        crate::gdt::refresh_tss_ist();
    }
    setup_idt();
    log_info!("IDT installed");
    // Ensure high-half runtime stacks are mapped explicitly before enabling more subsystems
    {
        use crate::handoff::handoff_phys_ptr;
        use crate::memory::{
            map_existing_region_va_to_its_pa, BootFrameAllocator, PTE_GLOBAL, PTE_NO_EXEC,
            PTE_PRESENT, PTE_WRITABLE,
        };
        use crate::physical_memory::{self, ConsumedRegion};
        use x86_64::registers::control::Cr3;

        let h = unsafe { &*(handoff_phys_ptr() as *const Handoff) };
        let (frame, _) = Cr3::read();
        let pml4_pa = frame.start_address().as_u64();

        let kernel_stack_region = ConsumedRegion {
            start: core::ptr::addr_of!(KERNEL_STACK) as u64,
            size: core::mem::size_of::<[u8; 64 * 1024]>() as u64,
        };
        physical_memory::record_boot_consumed_region(kernel_stack_region);
        unsafe {
            map_existing_region_va_to_its_pa(
                pml4_pa,
                h,
                kernel_stack_region.start,
                kernel_stack_region.size,
                PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC,
                &mut BootFrameAllocator::empty(),
            );
        }

        for (base, size) in crate::gdt::ist_stack_ranges().iter().copied() {
            physical_memory::record_boot_consumed_region(ConsumedRegion { start: base, size });
            unsafe {
                map_existing_region_va_to_its_pa(
                    pml4_pa,
                    h,
                    base,
                    size,
                    PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC,
                    &mut BootFrameAllocator::empty(),
                );
            }
        }
    }
    // Ensure timer vector (0x40) has a full 64-bit handler address
    unsafe {
        crate::interrupts::install_timer_vector_runtime();
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
        log_debug!("CR4: re-enabled OSFXSR, OSXMMEXCPT, PAGE_GLOBAL");
    }


    // Enable SSE unconditionally (AVX/MSRs remain disabled for now)
    {
        let mut f = crate::cpu::CpuFeatures::new();
        f.sse = true;
        unsafe {
            setup_floating_point(&f);
        }
        log_debug!("SSE enabled");
    }
    // Configure MSRs that are safe to enable now (e.g., EFER.SCE)
    unsafe {
        setup_msrs();
    }
    log_debug!("MSRs configured");

    // Verify LAPIC timer delivery (later test)
    const ENABLE_LAPIC_TIMER_TEST: bool = true;
    if ENABLE_LAPIC_TIMER_TEST {
        use x86_64::instructions::interrupts;
        log_debug!("Configuring LAPIC timer");
        crate::interrupts::lapic_timer_configure();
        let before = crate::interrupts::timer_tick_count();
        log_debug!("Arming LAPIC one-shot timer");
        unsafe {
            crate::interrupts::lapic_timer_start_oneshot(100_000);
        }
        log_debug!("Enabling interrupts (IF flag)");
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
        log_debug!("Disabled interrupts (IF flag)");
        let after = crate::interrupts::timer_tick_count();
        if after > before {
            log_info!("LAPIC timer interrupt received successfully");
        } else {
            log_warn!("LAPIC timer interrupt NOT received");
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
            log_debug!("High-half temp heap mapped for allocator shim");
        } else {
            log_warn!("No temp heap available for high-half allocator");
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
        log_debug!("Setting up permanent allocator");
        let mut frame_alloc = unsafe { BootFrameAllocator::from_handoff(h) };
        frame_alloc.enable_tracking();
        let (_frame, _flags) = Cr3::read();
        let pml4_pa = _frame.start_address().as_u64();
        let l4_va = crate::memory::phys_to_virt_pa(pml4_pa) as *mut X86PageTable;
        let l4: &mut X86PageTable = unsafe { &mut *l4_va };
        let mut mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(crate::memory::PHYS_OFFSET)) };
        map_kernel_heap_x86(&mut mapper, &mut frame_alloc);
        log_debug!("Kernel heap mapped");
        let perm_base = crate::memory::KERNEL_HEAP_BASE as *mut u8;
        let perm_size = crate::memory::KERNEL_HEAP_SIZE as usize;
        unsafe {
            theseus_shared::allocator::init_kernel_heap(perm_base, perm_size);
        }
        theseus_shared::allocator::switch_to_kernel_heap();

        let consumed = crate::physical_memory::drain_boot_consumed();
        log_debug!("Initializing persistent physical allocator");
        let bitmap_provider = |words: usize| allocate_bitmap_storage(words, &mut frame_alloc);
        crate::physical_memory::init_from_handoff(h, &consumed, bitmap_provider)
            .expect("failed to initialise persistent physical memory manager");
        log_info!("Persistent physical allocator ready");

        // With the allocator live, hand runtime services their virtual map
        match unsafe { set_virtual_address_map_runtime(handoff_phys_ptr()) } {
            Ok(()) => {
                log_info!("UEFI SetVirtualAddressMap completed");
            }
            Err(status) => {
                log_warn!("UEFI SetVirtualAddressMap failed: {:#x}", status.0 as u64);
                crate::boot::abort_with_context(
                    "UEFI SetVirtualAddressMap failed",
                    file!(),
                    line!(),
                    Some(status.0 as u64),
                );
            }
        }

        // Runtime services now operate via virtual addresses; capture firmware RTC output.
        log_uefi_firmware_time();

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
            log_debug!("Temporary heap unmapped");
            // Also unmap identity of kernel image to catch stale low-VA code/data
            unmap_identity_kernel_x86(&mut mapper, h);
            log_debug!("Identity-mapped kernel image unmapped");
        }
    }

    // Initialize ACPI and driver subsystem
    {
        use crate::drivers::system;

        match system::init() {
            Ok(platform_info) => {
                log_info!("Driver platform summary:");
                log_info!("  CPUs: {}", platform_info.cpu_count);
                log_info!("  IO APICs: {}", platform_info.io_apic_count);
                log_info!("  Local APIC: {:#x}", platform_info.local_apic_address);
            }
            Err(e) => {
                log_warn!("Driver initialization skipped: {}", e);
            }
        }
    }

    // Log as warning because this is what we use to measure success of the kernel environment setup.
    log_warn!("=== Kernel environment setup complete ===");
    log_warn!("Kernel initialization complete");

    // Set up framebuffer drawing and timer
    log_debug!("Setting up framebuffer drawing...");
    crate::framebuffer::init_framebuffer_drawing();

    // Draw initial heart pattern
    unsafe {
        // Get handoff from the global static that was set in main
        // TODO: Make it not use handoff for this.
        if let Some(handoff) = crate::interrupts::get_handoff_for_timer() {
            crate::framebuffer::draw_initial_heart(handoff, false);
        }
    }

    // Configure and start the APIC timer for heart animation
    log_debug!("Configuring APIC timer for heart animation...");
    unsafe {
        crate::interrupts::lapic_timer_configure();
        // Set timer to fire every ~5ms (50Hz) for smooth animation
        crate::interrupts::lapic_timer_start_periodic(50_000);
    }

    // Enable interrupts to start the timer
    x86_64::instructions::interrupts::enable();

    log_info!("Timer configured and interrupts enabled");

    // Small delay to ensure all debug bytes are emitted
    for _ in 0..1_000_00 {
        core::hint::spin_loop();
    }

    // Choose behavior based on centralized kernel configuration

    if crate::config::ENABLE_KERNEL_MONITOR {
        crate::monitor::init();
    }

    if crate::config::RUN_POST_BOOT_SERIAL_REVERSE_ECHO {
        log_warn!("⚠ Kernel COM1 reverse echo enabled");
        serial_debug::run_reverse_echo_session();
    }

    if crate::config::KERNEL_SHOULD_IDLE {
        log_warn!("Entering idle loop - heart animation active");
        log_warn!("Kill QEMU to stop the kernel");

        // Idle loop - the timer interrupt will handle the heart animation
        loop {
            // Use halt instruction to reduce CPU usage while waiting for interrupts
            x86_64::instructions::hlt();
        }
    } else {
        log_error!("Exiting QEMU immediately...");
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
pub fn setup_kernel_environment(_handoff: &Handoff, _kernel_physical_base: u64, _verbose: bool) {
    log_info!("=== Setting up Kernel Environment ===");

    // 1. Disable all interrupts first (including NMI)
    log_debug!("1. Disabling all interrupts...");
    unsafe {
        disable_all_interrupts();
    }
    log_debug!("  ✓ All interrupts disabled");

    // 2. Set up GDT and TSS
    log_debug!("2. Setting up GDT...");
    unsafe {
        setup_gdt();
    }
    log_debug!("  ✓ GDT loaded and segments reloaded");

    // 3. Configure control registers (PAE etc.)
    log_debug!("3. Configuring control registers...");
    unsafe {
        setup_control_registers();
    }
    log_debug!("  ✓ Control registers configured");

    // 3.5 Set up paging (identity map + high-half kernel) and load CR3
    log_debug!("3.5. Setting up paging...");
    // Create the MemoryManager (unsafe) in an outer scope so we can use it
    // later when performing the high-half jump.
    let mm = unsafe { MemoryManager::new(_handoff) };
    unsafe {
        // Sanity-check the memory manager returned values before loading CR3
        {
            let pml4_pa = mm.page_table_root();
            if pml4_pa == 0 {
                log_error!("mm.page_table_root == 0; aborting");
                theseus_shared::qemu_exit_error!();
            }
            let pml4_va = crate::memory::phys_to_virt_pa(pml4_pa);
            if (pml4_va as *const u8).is_null() {
                log_error!("pml4_va is null; aborting");
                theseus_shared::qemu_exit_error!();
            }
        }
        activate_virtual_memory(mm.page_table_root());
        // Mark PHYS_OFFSET mapping active for later helpers
        crate::memory::set_phys_offset_active();


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
    log_debug!("  ✓ Paging enabled (identity + high-half kernel)");

    // 4. Install low-half IDT, then jump to high-half, then reinstall IDT and continue
    log_debug!("4. Setting up CPU features...");
    // Install low-half IDT and proceed directly to high-half
    unsafe {
        setup_idt();
    }
    log_debug!("  IDT (low-half) installed");
    log_debug!("  [hh] preparing jump to high-half...");
    // Prepare for high-half transition: compute addresses and verify mappings
    let virt_base: u64 = KERNEL_VIRTUAL_BASE;
    let phys_base: u64 = crate::memory::runtime_kernel_phys_base(_handoff);

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
        log_debug!("Already in high-half, skipping jump");
    } else {
        // Delegate the translation/verification/jump to the memory subsystem.
        unsafe {
            mm.jump_to_high_half(phys_base, after_high_half_entry);
        }
    }

    // Unreachable if jump succeeds; if we get here, panic to avoid continuing in low-half
    log_error!("PANIC: High-half jump did not transfer control");
    panic!("High-half jump did not transfer control");
}

/// Query the firmware's RTC via UEFI runtime services and log the result.
///
/// This runs after we transition runtime services to virtual addressing, giving
/// immediate feedback that the virtual mapping is valid.
fn log_uefi_firmware_time() {
    match uefi::runtime::get_time() {
        Ok(time) => {
            let tz_display = match time.time_zone() {
                Some(offset_minutes) => {
                    let sign = if offset_minutes >= 0 { '+' } else { '-' };
                    let mut minutes = offset_minutes;
                    if minutes < 0 {
                        minutes = -minutes;
                    }
                    let hours = minutes / 60;
                    let mins = minutes % 60;
                    format!("UTC{}{:02}:{:02}", sign, hours, mins)
                }
                None => String::from("local"),
            };
            let daylight_bits = time.daylight().bits();
            log_debug!(
                "Firmware time {year:04}-{month:02}-{day:02} \
                 {hour:02}:{minute:02}:{second:02}.{nanos:09} {tz} daylight=0x{dl:02X}",
                year = time.year(),
                month = time.month(),
                day = time.day(),
                hour = time.hour(),
                minute = time.minute(),
                second = time.second(),
                nanos = time.nanosecond(),
                tz = tz_display,
                dl = daylight_bits,
            );
        }
        Err(err) => {
            let status = err.status();
            log_debug!("Firmware time unavailable (status {:?})", status);
        }
    }
}

fn allocate_bitmap_storage(
    words: usize,
    allocator: &mut BootFrameAllocator,
) -> &'static mut [u64] {
    let bytes = words
        .checked_mul(core::mem::size_of::<u64>())
        .expect("bitmap size overflow");
    let page_size = crate::memory::PAGE_SIZE;
    let pages = (bytes + page_size - 1) / page_size;
    if pages == 0 {
        return &mut [];
    }
    let first_frame = allocator
        .allocate_frame()
        .expect("no frames available for bitmap");
    let first_phys = first_frame.start_address().as_u64();
    for i in 1..pages {
        let frame = allocator
            .allocate_frame()
            .expect("no frames available for bitmap");
        let expected = first_phys + (i as u64) * page_size as u64;
        if frame.start_address().as_u64() != expected {
            log_error!("Bitmap allocation requires contiguous frames");
            theseus_shared::qemu_exit_error!();
        }
    }
    let size_bytes = pages * page_size;
    crate::physical_memory::record_boot_consumed_region(crate::physical_memory::consumed(
        first_phys,
        size_bytes as u64,
    ));
    let virt = crate::memory::phys_to_virt_pa(first_phys);
    unsafe {
        core::ptr::write_bytes(virt as *mut u8, 0, size_bytes);
        core::slice::from_raw_parts_mut(virt as *mut u64, words)
    }
}

/// Call UEFI's `SetVirtualAddressMap` runtime service after the kernel has established
/// permanent mappings for all runtime regions.
unsafe fn set_virtual_address_map_runtime(
    handoff_phys: u64,
) -> Result<(), Status> {
    use core::mem::{align_of, size_of};

    let handoff = &mut *(handoff_phys as *mut Handoff);
    if handoff.uefi_system_table == 0 {
        log_debug!("No UEFI system table; skipping SetVirtualAddressMap");
        return Ok(());
    }
    if handoff.uefi_system_table >= crate::memory::PHYS_OFFSET {
        // Already virtualised previously.
        log_debug!("System table already virtual; skipping");
        return Ok(());
    }
    if handoff.memory_map_buffer_ptr == 0
        || handoff.memory_map_entries == 0
        || handoff.memory_map_descriptor_size == 0
    {
        return Err(Status::UNSUPPORTED);
    }

    let desc_size = handoff.memory_map_descriptor_size as usize;
    let count = handoff.memory_map_entries as usize;
    let page_size = crate::memory::PAGE_SIZE as u64;
    let mut runtime_ranges: Vec<(u64, u64)> = Vec::new();
    let raw_ptr = handoff.memory_map_buffer_ptr;
    let phys_ptr = if raw_ptr >= crate::memory::PHYS_OFFSET {
        raw_ptr.wrapping_sub(crate::memory::PHYS_OFFSET)
    } else {
        raw_ptr
    };
    let phys_base = phys_ptr as usize;
    let memmap_len = handoff.memory_map_size as u64;

    // Update runtime descriptors to point at PHYS_OFFSET and track coverage.
    for idx in 0..count {
        let desc_ptr = (phys_base + idx * desc_size) as *mut MemoryDescriptor;
        let desc = unsafe { &mut *desc_ptr };
        if desc.att.contains(MemoryAttribute::RUNTIME) {
            let bytes = desc.page_count.saturating_mul(page_size);
            runtime_ranges.push((desc.phys_start, bytes));
            desc.virt_start = crate::memory::PHYS_OFFSET.wrapping_add(desc.phys_start);
        } else {
            desc.virt_start = desc.phys_start;
        }
    }
    log_debug!("Runtime descriptors prepared");

    let virt_ptr = crate::memory::PHYS_OFFSET.wrapping_add(phys_ptr);
    let map_base = virt_ptr as usize;
    handoff.memory_map_buffer_ptr = virt_ptr;
    log_debug!("Memory map pointer updated");

    // Ensure PHYS_OFFSET covers the memory map buffer and each runtime range before we
    // attempt to access them via the new virtual addresses.
    {
        use crate::memory::{map_range_with_policy, PageTable, PTE_PRESENT, PTE_WRITABLE};
        use x86_64::registers::control::Cr3;

        let mut frame_alloc = crate::physical_memory::PersistentFrameAllocator;
        let (frame, _) = Cr3::read();
        let pml4_pa = frame.start_address().as_u64();
        let pml4: &mut PageTable = unsafe { &mut *(pml4_pa as *mut PageTable) };

        if memmap_len != 0 {
            let phys_page_base = phys_ptr & !(page_size - 1);
            let offset = phys_ptr - phys_page_base;
            let total = ((offset + memmap_len + page_size - 1) / page_size) * page_size;
            unsafe {
                map_range_with_policy(
                    pml4,
                    crate::memory::PHYS_OFFSET.wrapping_add(phys_page_base),
                    phys_page_base,
                    total,
                    PTE_PRESENT | PTE_WRITABLE,
                    &mut frame_alloc,
                );
            }
        }

        for (phys_start, bytes) in runtime_ranges.iter().copied() {
            if bytes == 0 {
                continue;
            }
            unsafe {
                map_range_with_policy(
                    pml4,
                    crate::memory::PHYS_OFFSET.wrapping_add(phys_start),
                    phys_start,
                    bytes,
                    PTE_PRESENT | PTE_WRITABLE,
                    &mut frame_alloc,
                );
            }
        }
    }
    log_debug!("Runtime mapping ensured");

    let mut temp_vec: Vec<MemoryDescriptor> = Vec::new();
    let descriptors_slice: &mut [MemoryDescriptor] =
        if desc_size == size_of::<MemoryDescriptor>()
            && map_base % align_of::<MemoryDescriptor>() == 0
        {
            slice::from_raw_parts_mut(map_base as *mut MemoryDescriptor, count)
        } else {
            temp_vec.reserve_exact(count);
            for idx in 0..count {
                let src =
                    (map_base + idx * desc_size) as *const MemoryDescriptor;
            temp_vec.push(*src);
        }
        temp_vec.as_mut_slice()
    };
    log_debug!("Descriptor slice ready");

    let system_table_phys = handoff.uefi_system_table;
    let system_table_virt = crate::memory::PHYS_OFFSET.wrapping_add(system_table_phys)
        as *const uefi_raw::table::system::SystemTable;

    log_debug!("Calling SetVirtualAddressMap...");
    match uefi::runtime::set_virtual_address_map(descriptors_slice, system_table_virt) {
        Ok(()) => {
            handoff.uefi_system_table = system_table_virt as u64;
            log_info!("Runtime services remapped");
            Ok(())
        }
        Err(err) => Err(err.status()),
    }
}
