//! Memory management module
//!
//! This module contains the early-boot memory-management primitives used during
//! kernel bring-up. It is intentionally self-contained and provides a small
//! custom page-table representation (for early identity mapping and boot-time
//! page-table setup) as well as helpers that integrate with the `x86_64` crate
//! once paging is active.
//!
//! Major responsibilities:
//! - Building a PML4 and mapping an identity region used during early boot
//! - Mapping the kernel into a higher-half and establishing `PHYS_OFFSET`
//! - Providing a `BootFrameAllocator` built from the UEFI handoff memory map
//!   with reserved frame pool for critical kernel structures
//! - Utilities for mapping single pages and 2MiB huge pages using frame-backed
//!   page-table allocation helpers
//! - A small `TemporaryWindow` API to perform single-frame temporary mappings
//!   without relying on identity access once the PHYS_OFFSET mapping is active
//!
//! # Recent Improvements (Latest Session)
//!
//! ## Reserved Frame Pool
//! - Added `BootFrameAllocator` with reserved frame pool (16 frames) to prevent
//!   critical kernel structures (page tables, IST stacks) from being starved by
//!   ephemeral allocations.
//! - `reserve_frames()` method reserves frames from the general pool.
//! - `allocate_reserved_frame()` provides LIFO allocation from reserved pool.
//! - Page table allocation prefers reserved frames, falling back to general pool.
//!
//! ## Temporary Mapping Window
//! - `TemporaryWindow` struct provides safe temporary mapping of arbitrary physical
//!   frames to a fixed virtual address without relying on identity mapping.
//! - Used for zeroing newly allocated page tables safely after PHYS_OFFSET is active.
//! - Methods: `new()`, `map_phys_frame()`, `map_and_zero_frame()`, `unmap()`.
//!
//! ## PHYS_OFFSET Integration
//! - Enhanced `zero_frame_safely()` to use PHYS_OFFSET mapping when active,
//!   with identity fallback for early boot.
//! - `phys_offset_is_active()` tracks when PHYS_OFFSET mapping is available.
//! - `zero_phys_range()` performs volatile writes through PHYS_OFFSET mapping.
//!
//! ## Memory Map Diagnostics
//! - Added diagnostics showing total pages and conventional pages from UEFI memory map.
//! - Confirms allocator sees full system RAM (typically 1GB+ in QEMU).
//! - Helps debug memory allocation issues.
//!
//! Safety notes:
//! - Many functions in this module are `unsafe` because they manipulate raw
//!   page tables, write to control registers (CR3) or access physical memory
//!   via virtual translations. Callers must ensure the required invariants for
//!   these operations hold (correct PML4, appropriate mapping state, etc.).
#![allow(dead_code)]
#![allow(static_mut_refs)]

use crate::{log_debug, log_error, log_trace};
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

extern "C" {
    fn after_high_half_entry() -> !;
    fn continue_after_stack_switch() -> !;
}

/// Page table entry flags
pub const PTE_PRESENT: u64 = 1 << 0; // Present
pub const PTE_WRITABLE: u64 = 1 << 1; // Writable
pub const PTE_USER: u64 = 1 << 2; // User accessible
pub const PTE_PWT: u64 = 1 << 3; // Page Write Through
pub const PTE_PCD: u64 = 1 << 4; // Page Cache Disable
pub const PTE_ACCESSED: u64 = 1 << 5; // Accessed
pub const PTE_DIRTY: u64 = 1 << 6; // Dirty
pub const PTE_PS: u64 = 1 << 7; // Page Size (for PDE)
pub const PTE_GLOBAL: u64 = 1 << 8; // Global
pub const PTE_NO_EXEC: u64 = 1 << 63; // No Execute

/// Page table levels
pub const PML4_LEVEL: usize = 0;
pub const PDPT_LEVEL: usize = 1;
pub const PD_LEVEL: usize = 2;
pub const PT_LEVEL: usize = 3;

/// Page size constants
pub const PAGE_SIZE: usize = 4096;
pub const PAGE_MASK: u64 = 0xFFF;

/// Virtual memory layout
pub const KERNEL_VIRTUAL_BASE: u64 = 0xFFFFFFFF80000000;
pub const KERNEL_HEAP_BASE: u64 = 0xFFFFFFFFB0000000; // 768MB offset from kernel base to avoid framebuffer overlap
pub const KERNEL_HEAP_SIZE: usize = 0x100000; // 1MB
/// Fixed virtual base where the temporary boot heap is mapped
pub const TEMP_HEAP_VIRTUAL_BASE: u64 = 0xFFFFFFFFA0000000;
/// Physical memory linear mapping base (maps [0..N) -> [PHYS_OFFSET..PHYS_OFFSET+N))
pub const PHYS_OFFSET: u64 = 0xFFFF800000000000;
const HANDOFF_MEMMAP_WINDOW_BASE: u64 = 0xFFFF_FF88_0000_0000;

/// Convert a physical address into the kernel's high-half virtual address using
/// the fixed `PHYS_OFFSET` mapping.
///
/// Arguments:
/// - `pa`: physical address to convert
///
/// Returns:
/// - virtual address corresponding to `pa` in the kernel's PHYS_OFFSET window

/// Page table entry
///
/// Represents a single entry in a page table, containing a physical address
/// and various flags that control memory access and caching behavior.
mod page_tables;
pub use page_tables::{get_or_create_page_table_alloc, PageTable, PageTableEntry};

/// Memory manager
pub struct MemoryManager {
    pub pml4: &'static mut PageTable,
    pml4_phys: u64,
    pub kernel_heap_start: u64,
    pub kernel_heap_end: u64,
}
// Simple global virt<->phys offset model for early paging
static mut VIRT_PHYS_OFFSET: u64 = 0;
// Indicates that the PHYS_OFFSET linear mapping is active (so phys_to_virt_pa can be used)
static mut PHYS_OFFSET_ACTIVE: bool = false;
// Physical base of kernel image for page-table pool address translation
static mut KERNEL_PHYS_BASE_FOR_POOL: u64 = 0;
static ACTUAL_KERNEL_PHYS_BASE: AtomicU64 = AtomicU64::new(0);
static RUNTIME_BASE_LOGGED: AtomicBool = AtomicBool::new(false);
static ACTUAL_KERNEL_LOWER_GUARD: AtomicU64 = AtomicU64::new(0);

const LOWER_IMAGE_GUARD_BYTES: u64 = 0x10000; // 64 KiB guard window

/// Compute the runtime physical base of the kernel image.
///
/// The bootloader records a 2 MiB-aligned physical base in the handoff, but
/// depending on relocation the actual runtime address of `kernel_entry` may
/// fall below that alignment. This helper derives the true base by subtracting
/// the entry’s offset within the image (computed during boot) from the current
/// physical address of `kernel_entry`.
pub fn runtime_kernel_phys_base(handoff: &theseus_shared::handoff::Handoff) -> u64 {
    let cached = ACTUAL_KERNEL_PHYS_BASE.load(Ordering::Relaxed);
    if cached != 0 {
        return cached;
    }

    let raw_base = handoff.kernel_physical_base;
    let virt_base = handoff.kernel_virtual_base;
    let virt_entry = handoff.kernel_virtual_entry;

    let entry_phys = crate::kernel_entry as usize as u64;
    let stack_switch_phys = crate::stack::switch_to_kernel_stack_and_jump as usize as u64;
    let disable_irqs_phys = crate::interrupts::disable_all_interrupts as usize as u64;
    let setup_idt_phys = crate::interrupts::setup_idt as usize as u64;
    let setup_gdt_phys = crate::gdt::setup_gdt as usize as u64;
    let setup_ctrl_phys = crate::cpu::setup_control_registers as usize as u64;
    let after_entry_phys = after_high_half_entry as usize as u64;
    let cont_entry_phys = continue_after_stack_switch as usize as u64;

    let mut min_phys = u64::MAX;
    let mut consider = |addr: u64| {
        if addr != 0 && addr < min_phys {
            min_phys = addr;
        }
    };

    consider(raw_base);
    consider(entry_phys);
    consider(after_entry_phys);
    consider(cont_entry_phys);
    consider(stack_switch_phys);
    consider(disable_irqs_phys);
    consider(setup_idt_phys);
    consider(setup_gdt_phys);
    consider(setup_ctrl_phys);

    if min_phys == u64::MAX {
        min_phys = raw_base;
    }

    let guard_limit = core::cmp::min(LOWER_IMAGE_GUARD_BYTES, min_phys);
    let guard_pages = (guard_limit + (PAGE_SIZE as u64) - 1) / (PAGE_SIZE as u64);
    let guard_bytes = core::cmp::min(guard_pages * PAGE_SIZE as u64, min_phys);

    // Align down to 4KiB to match paging granularity and ensure we cover whole pages.
    let base = min_phys
        .saturating_sub(guard_bytes)
        & !((PAGE_SIZE as u64) - 1);
    if RUNTIME_BASE_LOGGED
        .compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed)
        .is_ok()
    {
        log_trace!(
            "Runtime phys base: raw={:#x} guard={:#x} virt_base={:#x} virt_entry={:#x} \
             entry_phys={:#x} after_phys={:#x} cont_phys={:#x} stack_switch_phys={:#x} \
             disable_irqs_phys={:#x} setup_idt_phys={:#x} setup_gdt_phys={:#x} \
             setup_ctrl_phys={:#x} min_phys={:#x} final={:#x}",
            raw_base, guard_bytes, virt_base, virt_entry, entry_phys, after_entry_phys,
            cont_entry_phys, stack_switch_phys, disable_irqs_phys, setup_idt_phys,
            setup_gdt_phys, setup_ctrl_phys, min_phys, base
        );
    }
    ACTUAL_KERNEL_LOWER_GUARD.store(guard_bytes, Ordering::Relaxed);
    ACTUAL_KERNEL_PHYS_BASE.store(base, Ordering::Relaxed);
    base
}

pub fn runtime_kernel_lower_guard() -> u64 {
    ACTUAL_KERNEL_LOWER_GUARD.load(Ordering::Relaxed)
}

// Early virt/phys helpers no longer used in HH path; keep for legacy/debug if needed
#[allow(dead_code)]
fn virt_to_phys(va: u64) -> u64 {
    unsafe { va.wrapping_add(VIRT_PHYS_OFFSET) }
}

#[allow(dead_code)]
fn phys_to_virt(pa: u64) -> u64 {
    unsafe { pa.wrapping_add(VIRT_PHYS_OFFSET) }
}

/// Convert a physical address into the kernel's high-half virtual address using
/// the fixed `PHYS_OFFSET` mapping.
#[allow(dead_code)]
pub fn phys_to_virt_pa(pa: u64) -> u64 {
    PHYS_OFFSET.wrapping_add(pa)
}

/// Convert a physical address into the kernel's PHYS_OFFSET-mapped virtual address.
///
/// # Examples
///
/// Basic translation math (pure computation):
///
/// ```
/// let pa: u64 = 0x1234;
/// let va = 0xFFFF800000000000u64.wrapping_add(pa);
/// assert_eq!(crate::memory::phys_to_virt_pa(pa), va);
/// ```

/// Mark the PHYS_OFFSET linear mapping as active. Call this after CR3 is loaded
/// and the kernel's PHYS_OFFSET mapping is established.
pub fn set_phys_offset_active() {
    unsafe {
        PHYS_OFFSET_ACTIVE = true;
    }
}

/// Query whether the PHYS_OFFSET mapping is active
pub fn phys_offset_is_active() -> bool {
    unsafe { PHYS_OFFSET_ACTIVE }
}

/// Zero a range of physical memory by writing through the kernel's PHYS_OFFSET
/// mapping. This is used for zeroing page-table frames after the PHYS_OFFSET
/// mapping is established.
///
/// # Safety
///
/// The caller must ensure that the PHYS_OFFSET mapping is valid and maps the
/// physical frame range `[pa, pa + size)` into the kernel virtual address space.

#[allow(dead_code)]
fn set_virt_phys_offset(offset: u64) {
    unsafe {
        VIRT_PHYS_OFFSET = offset;
    }
}

/// Zero a physical address range by converting to the kernel's PHYS_OFFSET-mapped virtual
/// address and performing volatile writes. Used for zeroing page-table frames safely.
#[allow(dead_code)]
pub unsafe fn zero_phys_range(pa: u64, size: usize) {
    let va = phys_to_virt_pa(pa);
    let mut off: usize = 0;
    // write u64 words for speed
    while off + 8 <= size {
        core::ptr::write_volatile((va + off as u64) as *mut u64, 0u64);
        off += 8;
    }
    // tail bytes if any
    while off < size {
        core::ptr::write_volatile((va + off as u64) as *mut u8, 0u8);
        off += 1;
    }
}

/// Zero a single page/frame safely. If paging is already enabled and the
/// PHYS_OFFSET mapping is available we use that; otherwise fall back to
/// identity physical writes which are valid early in boot.
#[allow(dead_code)]
pub unsafe fn zero_frame_safely(pa: u64) {
    // Zero a single frame safely. When the PHYS_OFFSET mapping is active we
    // call `zero_phys_range` which writes via the kernel virtual mapping; when
    // PHYS_OFFSET is not available (very early boot) we fall back to identity
    // writes into the physical address directly.
    if phys_offset_is_active() {
        zero_phys_range(pa, PAGE_SIZE as usize);
    } else {
        // Early boot path: identity write into physical memory. This is only
        // valid because early boot is still identity-addressable.
        core::ptr::write_bytes(pa as *mut u8, 0, PAGE_SIZE);
    }
}

/// Expose current virt->phys offset (phys - virt)
#[allow(dead_code)]
pub fn virt_phys_offset() -> u64 {
    unsafe { VIRT_PHYS_OFFSET }
}

impl MemoryManager {
    /// Create a new memory manager
    ///
    /// Construct a new MemoryManager and build an initial PML4 with the
    /// following mappings established:
    /// - identity mapping for the first 1 GiB (2MiB pages)
    /// - kernel image mapped into the high-half (4KiB pages)
    /// - a PHYS_OFFSET linear mapping for the first 1 GiB
    /// - framebuffer and temporary heap mappings if provided by the handoff
    ///
    /// # Arguments
    /// - `handoff`: pointer to the bootloader handoff structure containing the
    ///   UEFI memory map, kernel physical base/size, framebuffer info and temp
    ///   heap location.
    ///
    /// # Returns
    /// - `MemoryManager` containing a pointer to the PML4 (virtual), the
    ///   physical PML4 frame, and an initialized early `BootFrameAllocator`.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it writes raw page tables and performs
    /// identity physical memory writes while paging is not yet active. The
    /// caller must ensure the provided `handoff` is valid.
    pub unsafe fn new(handoff: &theseus_shared::handoff::Handoff) -> Self {
        log_debug!("MemoryManager::new() start");
        let runtime_kernel_base = runtime_kernel_phys_base(handoff);
        // Make kernel phys base available (legacy var no longer used after pool removal)
        KERNEL_PHYS_BASE_FOR_POOL = runtime_kernel_base;
        // Initialize early frame allocator and allocate a fresh PML4 frame
        let mut early_frame_alloc = BootFrameAllocator::from_handoff(handoff);
        early_frame_alloc.enable_tracking();
        // Reserve a small pool of frames for critical kernel structures (page
        // tables, IST stacks, etc.) so ephemeral allocations won't starve them.
        // This ensures that essential boot-time structures always have frames
        // available even if the general pool is temporarily exhausted during
        // heavy mapping operations.
        const RESERVED_FOR_CRITICAL: usize = 16;
        let reserved = early_frame_alloc.reserve_frames(RESERVED_FOR_CRITICAL);
        log_trace!("Reserved frames: {}", reserved);
        let pml4_frame = early_frame_alloc
            .allocate_frame()
            .expect("Out of frames for PML4");
        crate::physical_memory::record_boot_consumed_region(crate::physical_memory::consumed(
            pml4_frame.start_address().as_u64(),
            PAGE_SIZE as u64,
        ));
        let pml4_phys = pml4_frame.start_address().as_u64();
        // Zero the new PML4 frame using a helper that picks the correct method
        // depending on whether paging is active.
        zero_frame_safely(pml4_phys);

        let pml4: &mut PageTable = &mut *(pml4_phys as *mut PageTable);
        log_trace!("Got PML4");

        // Identity map first 1 GiB
        {
            log_debug!("Identity-mapping 1GiB begin");
            // Bootstrap identity map using frame-backed tables
            identity_map_first_1gb_2mb_alloc(pml4, &mut early_frame_alloc);
            log_debug!("Identity-mapping 1GiB done");
        }
        // legacy path removed

        // Do not clone bootloader HH entry; create our own HH structures deterministically

        // Map the kernel image high-half
        {
            log_debug!("Mapping kernel high-half begin");
            // Map kernel code/data into the high-half using 4KiB pages with frame allocator
            map_kernel_high_half_4k_alloc(pml4, handoff, &mut early_frame_alloc);
            log_debug!("Mapping kernel high-half done");
            let hh_index = ((KERNEL_VIRTUAL_BASE >> 39) & 0x1FF) as usize;
            let pml4_addr = pml4 as *mut PageTable as u64;
            let entry_val = core::ptr::read_volatile((pml4_addr as *const u64).add(hh_index));
            log_trace!("PML4[HH] entry={:#x}", entry_val);
            if entry_val == 0 {
                // Force-create HH PDPT so the entry is present
                let _ = get_or_create_page_table_alloc(
                    pml4.get_entry(hh_index),
                    &mut early_frame_alloc,
                );
                let entry_val2 = core::ptr::read_volatile((pml4_addr as *const u64).add(hh_index));
                log_trace!("PML4[HH] forced={:#x}", entry_val2);
            }
        }
        // legacy path removed

        // Map framebuffer and temp heap using frame allocator
        {
            log_debug!("Mapping framebuffer/heap begin");
            if handoff.gop_fb_base != 0 {
                map_framebuffer_alloc(pml4, handoff, &mut early_frame_alloc);
            }
            if handoff.temp_heap_base != 0 {
                map_temporary_heap_alloc(pml4, handoff, &mut early_frame_alloc);
            }
            log_debug!("Mapping framebuffer/heap done");
        }
        if crate::config::MAP_LEGACY_PHYS_OFFSET_1GIB {
            log_debug!("Mapping PHYS_OFFSET 1GiB begin");
            mapping::map_phys_offset_1gb_2mb_alloc(pml4, &mut early_frame_alloc);
            log_debug!("Mapping PHYS_OFFSET 1GiB done");
        } else {
            log_debug!("Mapping PHYS_OFFSET 64MiB begin");
            mapping::map_phys_offset_range_2mb_alloc(
                pml4,
                &mut early_frame_alloc,
                64 * 1024 * 1024,
            );
            log_debug!("Mapping PHYS_OFFSET 64MiB done");
            let memmap_len = handoff.memory_map_size as usize;
            if memmap_len > 0 {
                let page_size = PAGE_SIZE as u64;
                let phys_base = handoff.memory_map_buffer_ptr & !(page_size - 1);
                let offset = handoff.memory_map_buffer_ptr - phys_base;
                let size_aligned =
                    ((offset + memmap_len as u64 + page_size - 1) / page_size) * page_size;
                mapping::map_range_with_policy(
                    pml4,
                    HANDOFF_MEMMAP_WINDOW_BASE,
                    phys_base,
                    size_aligned,
                    PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC,
                    &mut early_frame_alloc,
                );
                let new_ptr = HANDOFF_MEMMAP_WINDOW_BASE + offset;
                let handoff_mut: *mut theseus_shared::handoff::Handoff =
                    handoff as *const _ as *mut _;
                unsafe {
                    (*handoff_mut).memory_map_buffer_ptr = new_ptr;
                }
            }
        }

        // Map LAPIC MMIO region (0xFEE00000-0xFEEFFFFF)
        {
            log_debug!("Mapping LAPIC MMIO begin");
            map_lapic_mmio_alloc(pml4, &mut early_frame_alloc);
            log_debug!("Mapping LAPIC MMIO done");
        }

        // Map IO APIC MMIO region (0xFEC00000-0xFECFFFFF)
        {
            log_debug!("Mapping IO APIC MMIO begin");
            mapping::map_io_apic_mmio_alloc(pml4, &mut early_frame_alloc);
            log_debug!("Mapping IO APIC MMIO done");
        }

        let kernel_heap_start = KERNEL_HEAP_BASE;
        let kernel_heap_end = kernel_heap_start + KERNEL_HEAP_SIZE as u64;

        // Collect frames allocated during boot so they can be marked used in the
        // persistent allocator via `record_boot_consumed_region` during allocation.

        // Quick integrity checks (optional): ensure first entries are present
        if !pml4.entries[0].is_present() {
            panic!("PML4[0] not present");
        }

        // Done
        Self {
            pml4,
            pml4_phys,
            kernel_heap_start,
            kernel_heap_end,
        }
    }

    /// Get the page table root (CR3 value)
    pub fn page_table_root(&self) -> u64 {
        self.pml4_phys
    }

    /// Compute and perform the high-half jump to `entry`.
    ///
    /// Safety: caller must ensure that paging is active and the high-half
    /// mappings for the kernel image are present. This function performs a
    /// non-returning jump into the high-half virtual address of `entry`.
    pub unsafe fn jump_to_high_half(&self, phys_base: u64, entry: extern "C" fn() -> !) -> ! {
        use x86_64::{
            registers::control::Cr3,
            structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate},
            VirtAddr,
        };

        let virt_base: u64 = KERNEL_VIRTUAL_BASE;
        let verbose = crate::config::VERBOSE_KERNEL_OUTPUT;

        // Get current RIP to determine whether we are already running in HH
        let rip_now: u64;
        core::arch::asm!("lea {}, [rip + 0]", out(reg) rip_now, options(nostack));

        if rip_now >= virt_base {
            log_debug!("Already in high-half, skipping jump");
            // Safety: caller shouldn't call this when already in high-half; abort
            log_error!("PANIC: jump_to_high_half invoked while already in high-half");
            theseus_shared::qemu_exit_error!();
            panic!("jump_to_high_half invoked while already in high-half");
        }

        // Compute target virtual address of the provided entry symbol
        let sym: u64 = entry as usize as u64;
        let target: u64 = sym
            .wrapping_sub(phys_base)
            .wrapping_add(KERNEL_VIRTUAL_BASE);

        if verbose {
            log_trace!(
                "Jump info: phys_base={:#x} rip_now={:#x} sym={:#x} target={:#x}",
                phys_base, rip_now, sym, target
            );
        }

        // Verify mapping for target before performing jump
        let (_frame, _flags) = Cr3::read();
        let pml4_pa = _frame.start_address().as_u64();
        let l4: &mut X86PageTable = unsafe { &mut *(pml4_pa as *mut X86PageTable) };
        let mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(PHYS_OFFSET)) };

        // Check PML4 entry for high-half
        let hh_index = ((KERNEL_VIRTUAL_BASE >> 39) & 0x1FF) as usize;
        let pml4_entry_val =
            unsafe { core::ptr::read_volatile((pml4_pa as *const u64).add(hh_index)) };
        if pml4_entry_val == 0 {
            log_error!("PANIC: PML4[HH] entry is zero; high-half may not be mapped");
            log_error!("PML4 physical={:#x}", pml4_pa);
            theseus_shared::qemu_exit_error!();
            panic!("PML4[HH] entry is zero");
        }

        let phys = mapper.translate_addr(VirtAddr::new(target));
        if let Some(pa) = phys {
            log_trace!("Target physical={:#x}", pa.as_u64());
        } else {
            log_error!("PANIC: high-half target translation returned NONE");
            log_error!("Target virtual={:#x}", target);
            theseus_shared::qemu_exit_error!();
            panic!("high-half target not mapped");
        }

        log_debug!("Jumping to high-half (verified)");

        // Perform the non-returning jump into high-half
        core::arch::asm!(
            "jmp rax",
            in("rax") target,
            options(noreturn)
        );
    }
}

/// Check whether a virtual address is currently mapped by the active page tables.
/// Uses the current CR3 and the `PHYS_OFFSET` linear mapping to build a mapper.
pub fn virt_addr_is_mapped(va: u64) -> bool {
    use x86_64::{
        registers::control::Cr3,
        structures::paging::{OffsetPageTable, PageTable as X86PageTable, Translate},
        VirtAddr,
    };
    let (_frame, _flags) = Cr3::read();
    let pml4_pa = _frame.start_address().as_u64();
    let l4: &mut X86PageTable = unsafe { &mut *(pml4_pa as *mut X86PageTable) };
    let mapper = unsafe { OffsetPageTable::new(l4, VirtAddr::new(PHYS_OFFSET)) };
    mapper.translate_addr(VirtAddr::new(va)).is_some()
}

/// Check whether a virtual address is mapped and has the requested page-table flags.
/// `flags_mask` is compared against the final page-table entry flags (PTE bits).
pub fn virt_addr_has_flags(va: u64, flags_mask: u64) -> bool {
    use x86_64::registers::control::Cr3;
    // Read CR3
    let (frame, _f) = Cr3::read();
    let mut table_pa = frame.start_address().as_u64();

    // Helper to read an entry at given table physical address and index
    unsafe fn read_entry(table_pa: u64, index: usize) -> u64 {
        let entry_pa = table_pa + (index * 8) as u64;
        let entry_va = phys_to_virt_pa(entry_pa) as *const u64;
        core::ptr::read_volatile(entry_va)
    }

    // Extract indices
    let pml4_index = ((va >> 39) & 0x1FF) as usize;
    let pdpt_index = ((va >> 30) & 0x1FF) as usize;
    let pd_index = ((va >> 21) & 0x1FF) as usize;
    let pt_index = ((va >> 12) & 0x1FF) as usize;

    // Walk PML4
    let pml4e = unsafe { read_entry(table_pa, pml4_index) };
    if pml4e & PTE_PRESENT == 0 {
        return false;
    }
    // Next level
    table_pa = pml4e & 0x000ffffffffff000u64;

    let pdpte = unsafe { read_entry(table_pa, pdpt_index) };
    if pdpte & PTE_PRESENT == 0 {
        return false;
    }
    // Check PS (1GiB) at PDPT
    if pdpte & PTE_PS != 0 {
        return (pdpte & flags_mask) == flags_mask;
    }
    table_pa = pdpte & 0x000ffffffffff000u64;

    let pde = unsafe { read_entry(table_pa, pd_index) };
    if pde & PTE_PRESENT == 0 {
        return false;
    }
    // Check PS (2MiB) at PD
    if pde & PTE_PS != 0 {
        return (pde & flags_mask) == flags_mask;
    }
    table_pa = pde & 0x000ffffffffff000u64;

    let pte = unsafe { read_entry(table_pa, pt_index) };
    if pte & PTE_PRESENT == 0 {
        return false;
    }
    (pte & flags_mask) == flags_mask
}

/// Check whether a full virtual range `[va, va+size)` is mapped and each page
/// has the requested `flags_mask` bits set. Returns `true` only if every page
/// in the range passes the flags check.
pub fn virt_range_has_flags(mut va: u64, size: usize, flags_mask: u64) -> bool {
    if size == 0 {
        return true;
    }
    let page_size: u64 = PAGE_SIZE as u64;
    let end = va.wrapping_add(size as u64);
    while va < end {
        if !virt_addr_has_flags(va, flags_mask) {
            return false;
        }
        va = va.wrapping_add(page_size);
    }
    true
}

mod mapping;
pub use mapping::identity_map_first_1gb_2mb_alloc;
pub use mapping::map_2mb_page_alloc;
pub use mapping::map_existing_region_va_to_its_pa;
pub use mapping::map_framebuffer_alloc;
pub use mapping::map_high_half_1gb_2mb;
pub use mapping::map_kernel_high_half_2mb;
pub use mapping::map_kernel_high_half_4k_alloc;
pub use mapping::map_lapic_mmio_alloc;
pub use mapping::map_page_alloc;
pub use mapping::map_phys_offset_range_2mb_alloc;
pub use mapping::map_range_with_policy;
pub use mapping::map_temporary_heap_alloc;

// Use the implementation in `page_tables.rs` instead

mod temporary_window;
pub use temporary_window::TemporaryWindow;
pub use temporary_window::TEMP_WINDOW_VA;

mod page_table_builder;
pub use page_table_builder::PageTableBuilder;

/// Activate virtual memory by loading the page table root into CR3
///
/// This function performs the critical step of enabling virtual memory by loading
/// the physical address of the PML4 (Page Map Level 4) table into the CR3 register.
/// This makes the CPU use our page tables for address translation.
///
/// # Assembly Details
/// - `mov cr3, {page_table_root}`: Load the PML4 physical address into CR3 register
///
/// # Parameters
///
/// * `page_table_root` - Physical address of the PML4 page table
///
/// # Safety
///
/// This function is unsafe because it:
/// - Modifies the CR3 control register
/// - Assumes the page table is properly set up
/// - Must be called when identity mapping is active (before high-half transition)
///
/// The caller must ensure:
/// - The page table is properly initialized and valid
/// - Identity mapping is active for the current code
/// - The page table root address is valid and accessible
/// - No other code is modifying CR3 concurrently
pub unsafe fn activate_virtual_memory(page_table_root: u64) {
    {
        use x86_64::registers::control::Cr3;
        log_debug!("activate_virtual_memory: loading CR3 with={:#x}", page_table_root);
        let (old, _f) = Cr3::read();
        log_trace!("activate_virtual_memory: CR3 before={:#x}", old.start_address().as_u64());
    }
    // Load our page table root into CR3 to enable virtual memory
    core::arch::asm!(
        "mov cr3, {}",  // Load PML4 physical address into CR3 register
        in(reg) page_table_root,
        options(nomem, nostack, preserves_flags)
    );
    // Confirm CR3 after loading
    {
        use x86_64::registers::control::Cr3;
        let (new, _f2) = Cr3::read();
        log_debug!("activate_virtual_memory: CR3 after={:#x}", new.start_address().as_u64());
    }
}

// ===================== x86_64 paging integration (gated) =====================
use x86_64::{
    structures::paging::{
        FrameAllocator, Mapper, OffsetPageTable, Page, PageTableFlags, PhysFrame, Size4KiB,
    },
    PhysAddr, VirtAddr,
};
// control registers used elsewhere; no local Cr0 flags needed here

// Extract the BootFrameAllocator implementation into its own module to
// separate frame-allocation responsibilities from page-table construction
// and mapping helpers. This improves maintainability and keeps a clear API
// boundary between allocation and mapping logic.
mod frame_allocator;
pub use frame_allocator::{BootFrameAllocator, FrameSource};

// Public API summary
// ------------------
// Exported types and helpers for other kernel modules to use. Prefer the
// high-level helpers here; internal helpers remain in submodules.

/// Map the kernel's physical image range into the high-half window using 4KiB pages (x86_64 API)
fn map_kernel_high_half_x86<M>(
    mapper: &mut M,
    frame_alloc: &mut BootFrameAllocator,
    handoff: &theseus_shared::handoff::Handoff,
) where
    M: Mapper<Size4KiB>,
{
    let phys_base = runtime_kernel_phys_base(handoff);
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 {
        return;
    }

    let pages = ((phys_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE; // executable by default
    for i in 0..pages {
        let pa = PhysAddr::new(phys_base + i * PAGE_SIZE as u64);
        let frame = PhysFrame::<Size4KiB>::containing_address(pa);
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(
            KERNEL_VIRTUAL_BASE + i * PAGE_SIZE as u64,
        ));
        let _ =
            unsafe { mapper.map_to(page, frame, flags, frame_alloc) }.map(|flush| flush.flush());
    }
}

/// Map framebuffer using x86_64 paging API (4KiB pages)
fn map_framebuffer_x86<M>(
    mapper: &mut M,
    frame_alloc: &mut BootFrameAllocator,
    handoff: &theseus_shared::handoff::Handoff,
) where
    M: Mapper<Size4KiB>,
{
    let fb_physical = handoff.gop_fb_base;
    let fb_virtual = 0xFFFFFFFF90000000u64;
    let fb_size = handoff.gop_fb_size;
    let pages = ((fb_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_EXECUTE;
    for i in 0..pages {
        let pa = PhysAddr::new(fb_physical + i * PAGE_SIZE as u64);
        let frame = PhysFrame::<Size4KiB>::containing_address(pa);
        let page =
            Page::<Size4KiB>::containing_address(VirtAddr::new(fb_virtual + i * PAGE_SIZE as u64));
        // Ignore AlreadyMapped errors by simply continuing
        let _ =
            unsafe { mapper.map_to(page, frame, flags, frame_alloc) }.map(|flush| flush.flush());
    }
}

/// Map temporary heap using x86_64 paging API (4KiB pages)
fn map_temporary_heap_x86<M>(
    mapper: &mut M,
    frame_alloc: &mut BootFrameAllocator,
    handoff: &theseus_shared::handoff::Handoff,
) where
    M: Mapper<Size4KiB>,
{
    let heap_physical = handoff.temp_heap_base;
    let heap_virtual = 0xFFFFFFFFA0000000u64;
    let heap_size = handoff.temp_heap_size;
    let pages = ((heap_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    let flags = PageTableFlags::PRESENT | PageTableFlags::WRITABLE | PageTableFlags::NO_EXECUTE;
    for i in 0..pages {
        let pa = PhysAddr::new(heap_physical + i * PAGE_SIZE as u64);
        let frame = PhysFrame::<Size4KiB>::containing_address(pa);
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(
            heap_virtual + i * PAGE_SIZE as u64,
        ));
        let _ =
            unsafe { mapper.map_to(page, frame, flags, frame_alloc) }.map(|flush| flush.flush());
    }
}

/// Map the permanent kernel heap at KERNEL_HEAP_BASE using fresh frames
pub fn map_kernel_heap_x86<M, F>(mapper: &mut M, frame_alloc: &mut F)
where
    M: Mapper<Size4KiB>,
    F: FrameAllocator<Size4KiB>,
{
    use x86_64::structures::paging::PageTableFlags as F;
    let pages = (KERNEL_HEAP_SIZE as u64 + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64;
    let flags = F::PRESENT | F::WRITABLE | F::NO_EXECUTE;

    for i in 0..pages {
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(
            KERNEL_HEAP_BASE + i * PAGE_SIZE as u64,
        ));
        if let Some(frame) = frame_alloc.allocate_frame() {
            match unsafe { mapper.map_to(page, frame, flags, frame_alloc) } {
                Ok(flush) => {
                    flush.flush();
                }
                Err(_e) => {
                    log_error!("Kernel heap map: mapping failed");
                    break;
                }
            }
        } else {
            log_error!("Kernel heap map: out of frames");
            break;
        }
    }
}

/// Unmap the temporary boot heap that was mapped at TEMP_HEAP_VIRTUAL_BASE
pub fn unmap_temporary_heap_x86<M>(mapper: &mut M, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
    log_debug!("unmap_temporary_heap_x86: begin");
    // Trait not needed for direct calls here
    let heap_virtual = TEMP_HEAP_VIRTUAL_BASE;
    let heap_size = handoff.temp_heap_size;
    if heap_size == 0 {
        return;
    }
    let pages = ((heap_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    for i in 0..pages {
        let page = Page::<Size4KiB>::containing_address(VirtAddr::new(
            heap_virtual + i * PAGE_SIZE as u64,
        ));
        if let Ok((_frame, flush)) = mapper.unmap(page) {
            flush.flush();
        }
    }
    log_debug!("unmap_temporary_heap_x86: done");
}

/// Unmap the identity-mapped kernel image range at low VA to catch stale low-VA uses
pub fn unmap_identity_kernel_x86<M>(mapper: &mut M, handoff: &theseus_shared::handoff::Handoff)
where
    M: Mapper<Size4KiB>,
{
    log_debug!("unmap_identity_kernel_x86: begin");
    // Trait not needed for direct calls here
    let phys_base = runtime_kernel_phys_base(handoff);
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 {
        return;
    }
    let pages = ((phys_size + PAGE_SIZE as u64 - 1) / PAGE_SIZE as u64) as u64;
    for i in 0..pages {
        let va = VirtAddr::new(phys_base + i * PAGE_SIZE as u64);
        let page = Page::<Size4KiB>::containing_address(va);
        if let Ok((_frame, flush)) = mapper.unmap(page) {
            flush.flush();
        }
    }
    log_debug!("unmap_identity_kernel_x86: done");
}

/// Identity map first 1 GiB using 2 MiB pages with x86_64 API
fn identity_map_first_1gb_x86(
    mapper: &mut OffsetPageTable<'static>,
    frame_alloc: &mut BootFrameAllocator,
) {
    use x86_64::structures::paging::{Mapper, PageTableFlags as F, Size2MiB};
    let flags = F::PRESENT | F::WRITABLE | F::GLOBAL;
    let two_mb: u64 = 2 * 1024 * 1024;
    let one_gb: u64 = 1024 * 1024 * 1024;
    let mut addr: u64 = 0;
    while addr < one_gb {
        let page = Page::<Size2MiB>::containing_address(VirtAddr::new(addr));
        let frame = PhysFrame::<Size2MiB>::containing_address(PhysAddr::new(addr));
        // Safe under our invariants: we identity map known RAM, providing existing frame
        let _ =
            unsafe { mapper.map_to(page, frame, flags, frame_alloc) }.map(|flush| flush.flush());
        addr += two_mb;
    }
}

pub fn current_pml4_phys() -> u64 {
    use x86_64::registers::control::Cr3;
    let (frame, _) = Cr3::read();
    frame.start_address().as_u64()
}
