//! Mapping helpers moved out of `memory.rs` to isolate page-table construction
//! and mapping policies.

use super::{
    runtime_kernel_phys_base, PageTable, PageTableEntry, KERNEL_VIRTUAL_BASE, PAGE_SIZE,
    PHYS_OFFSET, PTE_ACCESSED, PTE_DIRTY, PTE_GLOBAL, PTE_NO_EXEC, PTE_PCD, PTE_PRESENT, PTE_PS,
    PTE_PWT, PTE_USER, PTE_WRITABLE, TEMP_HEAP_VIRTUAL_BASE,
};
use crate::log_debug;
use crate::memory::page_table_builder::PageTableBuilder;
use crate::memory::FrameSource;
use core::fmt;

/// Captures the essential ranges needed to mirror the kernel image into the high half.
#[derive(Clone, Copy, Debug)]
struct KernelMappingExtents {
    phys_base: u64,
    phys_size: u64,
    guard: u64,
}

impl KernelMappingExtents {
    /// Physical start after extending downward by the guard window.
    fn guarded_phys_start(&self) -> u64 {
        self.phys_base.saturating_sub(self.guard)
    }

    /// Virtual start after extending downward by the guard window.
    fn guarded_va_start(&self) -> u64 {
        KERNEL_VIRTUAL_BASE.saturating_sub(self.guard)
    }
}

fn kernel_mapping_extents(
    handoff: &theseus_shared::handoff::Handoff,
) -> Option<KernelMappingExtents> {
    let phys_base = runtime_kernel_phys_base(handoff);
    let phys_size = handoff.kernel_image_size;
    if phys_base == 0 || phys_size == 0 {
        return None;
    }
    let guard = core::cmp::min(crate::memory::runtime_kernel_lower_guard(), phys_base);
    Some(KernelMappingExtents {
        phys_base,
        phys_size,
        guard,
    })
}

/// Set up identity mapping for first 1 GiB using 2 MiB pages
/// Identity-map the first 1 GiB of physical memory using 2MiB pages.
///
/// This is used during early boot so that low-VA virtual accesses still find
/// expected physical frames while the higher-half mappings are established.
pub unsafe fn identity_map_first_1gb_2mb_alloc<F: FrameSource>(pml4: &mut PageTable, fa: &mut F) {
    debug_assert!(
        PAGE_SIZE.is_power_of_two(),
        "PAGE_SIZE must be a power of two"
    );
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_PS;
    let gigabyte: u64 = 1 << 30;
    let two_mb: u64 = 2 * 1024 * 1024;
    let mut addr: u64 = 0;
    let mut count: u32 = 0;
    while addr < gigabyte {
        if count < 4 {
            log_debug!("map2m {:#x} -> {:#x}", addr, addr);
        }
        super::map_2mb_page_alloc(pml4, addr, addr, flags, fa);
        addr += two_mb;
        count += 1;
    }
}

/// Map a single page using frame-backed table allocation
/// Map a single 4KiB page into `pml4`, creating intermediate page-table
/// levels as necessary using `fa`.
pub unsafe fn map_page_alloc<F: FrameSource>(
    pml4: &mut PageTable,
    virtual_addr: u64,
    physical_addr: u64,
    flags: u64,
    fa: &mut F,
) {
    debug_assert_eq!(
        virtual_addr & ((PAGE_SIZE as u64) - 1),
        0,
        "virtual address must be 4 KiB aligned"
    );
    debug_assert_eq!(
        physical_addr & ((PAGE_SIZE as u64) - 1),
        0,
        "physical address must be 4 KiB aligned"
    );
    let pml4_index = ((virtual_addr >> 39) & 0x1FF) as usize;
    let pdpt_index = ((virtual_addr >> 30) & 0x1FF) as usize;
    let pd_index = ((virtual_addr >> 21) & 0x1FF) as usize;
    let pt_index = ((virtual_addr >> 12) & 0x1FF) as usize;
    let pdpt = super::get_or_create_page_table_alloc(pml4.get_entry(pml4_index), fa);
    let pd = super::get_or_create_page_table_alloc(pdpt.get_entry(pdpt_index), fa);
    let pt = super::get_or_create_page_table_alloc(pd.get_entry(pd_index), fa);
    *pt.get_entry(pt_index) = PageTableEntry::new(physical_addr, flags);
}

/// Map a single 2 MiB page using frame-backed table allocation
/// Map a single 2MiB page by installing a PD entry with the PS bit set.
pub unsafe fn map_2mb_page_alloc<F: FrameSource>(
    pml4: &mut PageTable,
    virtual_addr: u64,
    physical_addr: u64,
    flags: u64,
    fa: &mut F,
) {
    const TWO_MB: u64 = 2 * 1024 * 1024;
    debug_assert_eq!(
        virtual_addr & (TWO_MB - 1),
        0,
        "virtual address must be 2 MiB aligned"
    );
    debug_assert_eq!(
        physical_addr & (TWO_MB - 1),
        0,
        "physical address must be 2 MiB aligned"
    );
    let pml4_index = ((virtual_addr >> 39) & 0x1FF) as usize;
    let pdpt_index = ((virtual_addr >> 30) & 0x1FF) as usize;
    let pd_index = ((virtual_addr >> 21) & 0x1FF) as usize;
    let pdpt = super::get_or_create_page_table_alloc(pml4.get_entry(pml4_index), fa);
    let pd = super::get_or_create_page_table_alloc(pdpt.get_entry(pdpt_index), fa);
    *pd.get_entry(pd_index) = PageTableEntry::new(physical_addr, flags | PTE_PS);
}

/// Map kernel HH using 4KiB pages
/// Map the kernel's physical image range into the high-half using 4KiB pages.
pub unsafe fn map_kernel_high_half_4k_alloc<F: FrameSource>(
    pml4: &mut PageTable,
    handoff: &theseus_shared::handoff::Handoff,
    fa: &mut F,
) {
    let Some(extents) = kernel_mapping_extents(handoff) else {
        return;
    };
    const KERNEL_IMAGE_PAD: u64 = 8 * 1024 * 1024;
    let mut total_bytes = extents.phys_size + KERNEL_IMAGE_PAD;
    if total_bytes >= PAGE_SIZE as u64 {
        // Avoid mapping an unnecessary trailing guard page when the padded size
        // already falls on a page boundary.
        total_bytes -= PAGE_SIZE as u64;
    }
    let page_bytes = PAGE_SIZE as u64;
    debug_assert!(
        page_bytes.is_power_of_two(),
        "PAGE_SIZE must be a power of two"
    );
    let pages = total_bytes.div_ceil(page_bytes);
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL;
    let mut builder = PageTableBuilder::new(pml4, fa);
    let guard = extents.guard;
    // Extend the mapping downward by `guard` bytes so that low trampoline code or
    // relocation thunks that executed before entering the high-half remain reachable.
    let va_start = extents.guarded_va_start();
    let pa_start = extents.guarded_phys_start();
    let total = guard + pages * PAGE_SIZE as u64;
    builder.map_range(va_start, pa_start, total, flags);
}

/// Map framebuffer
/// Map the framebuffer into a fixed high-half virtual region.
pub unsafe fn map_framebuffer_alloc<F: FrameSource>(
    pml4: &mut PageTable,
    handoff: &theseus_shared::handoff::Handoff,
    fa: &mut F,
) {
    let fb_physical = handoff.gop_fb_base;
    let fb_virtual = 0xFFFFFFFF90000000u64;
    let fb_size = handoff.gop_fb_size;
    let page_bytes = PAGE_SIZE as u64;
    let pages = fb_size.div_ceil(page_bytes);
    let mut builder = PageTableBuilder::new(pml4, fa);
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC;
    log_debug!(
        "Mapping framebuffer: phys=0x{:016X} virt=0x{:016X} pages={} flags={} ({:#x})",
        fb_physical,
        fb_virtual,
        pages,
        FlagSummary(flags),
        flags
    );
    builder.map_range(fb_virtual, fb_physical, pages * page_bytes, flags);
}

/// Map temporary heap
/// Map the temporary heap provided by the bootloader into the fixed virtual
/// region `TEMP_HEAP_VIRTUAL_BASE`.
pub unsafe fn map_temporary_heap_alloc<F: FrameSource>(
    pml4: &mut PageTable,
    handoff: &theseus_shared::handoff::Handoff,
    fa: &mut F,
) {
    let heap_physical = handoff.temp_heap_base;
    let heap_virtual = TEMP_HEAP_VIRTUAL_BASE;
    let heap_size = handoff.temp_heap_size;
    let page_bytes = PAGE_SIZE as u64;
    let pages = heap_size.div_ceil(page_bytes);
    let mut builder = PageTableBuilder::new(pml4, fa);
    builder.map_range(
        heap_virtual,
        heap_physical,
        pages * page_bytes,
        PTE_PRESENT | PTE_WRITABLE | PTE_NO_EXEC,
    );
}

/// Map a 1GiB linear physical mapping at PHYS_OFFSET using 2MiB pages
/// Map a linear mapping of the first 1GiB of physical memory at `PHYS_OFFSET`
/// using 2MiB pages.
pub unsafe fn map_phys_offset_range_2mb_alloc<F: FrameSource>(
    pml4: &mut PageTable,
    fa: &mut F,
    max_phys_addr: u64,
) {
    let two_mb: u64 = 2 * 1024 * 1024;
    if max_phys_addr == 0 {
        return;
    }
    let mut offset: u64 = 0;
    let limit = max_phys_addr.saturating_add(two_mb - 1) & !(two_mb - 1);
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL;
    log_debug!(
        "Mapping PHYS_OFFSET window: base=0x{:016X} limit=0x{:016X} step=2MiB flags={} ({:#x})",
        PHYS_OFFSET,
        PHYS_OFFSET + limit,
        FlagSummary(flags),
        flags
    );
    while offset < limit {
        let pa = offset;
        let va = PHYS_OFFSET + offset;
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        offset = offset.saturating_add(two_mb);
    }
}

pub unsafe fn map_phys_offset_1gb_2mb_alloc<F: FrameSource>(pml4: &mut PageTable, fa: &mut F) {
    map_phys_offset_range_2mb_alloc(pml4, fa, 1 << 30);
}

/// Map LAPIC MMIO region
/// Map the LAPIC MMIO region at a virtual address derived from `PHYS_OFFSET`.
pub unsafe fn map_lapic_mmio_alloc<F: FrameSource>(pml4: &mut PageTable, fa: &mut F) {
    const LAPIC_PHYS_BASE: u64 = 0xFEE00000;
    const LAPIC_VIRT_BASE: u64 = PHYS_OFFSET + LAPIC_PHYS_BASE;
    const LAPIC_SIZE: u64 = 0x100000;
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_PCD | PTE_PWT;
    log_debug!(
        "Mapping LAPIC MMIO: phys=0x{:08X} virt=0x{:016X} size={} KiB flags={} ({:#x})",
        LAPIC_PHYS_BASE,
        LAPIC_VIRT_BASE,
        LAPIC_SIZE / 1024,
        FlagSummary(flags),
        flags
    );
    for i in 0..(LAPIC_SIZE / PAGE_SIZE as u64) {
        let virt_addr = LAPIC_VIRT_BASE + (i * PAGE_SIZE as u64);
        let phys_addr = LAPIC_PHYS_BASE + (i * PAGE_SIZE as u64);
        map_page_alloc(pml4, virt_addr, phys_addr, flags, fa);
    }
}

/// Map the IO APIC MMIO region at a high-half address derived from `PHYS_OFFSET`.
/// Most PC platforms place the first IO APIC at physical 0xFEC0_0000. We map an entire
/// 1MiB window so that all redirection table registers are covered.
pub unsafe fn map_io_apic_mmio_alloc<F: FrameSource>(pml4: &mut PageTable, fa: &mut F) {
    const IOAPIC_PHYS_BASE: u64 = 0xFEC0_0000;
    const IOAPIC_VIRT_BASE: u64 = PHYS_OFFSET + IOAPIC_PHYS_BASE;
    const IOAPIC_SIZE: u64 = 0x100000;
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_PCD | PTE_PWT;
    log_debug!(
        "Mapping IOAPIC MMIO: phys=0x{:08X} virt=0x{:016X} size={} KiB flags={} ({:#x})",
        IOAPIC_PHYS_BASE,
        IOAPIC_VIRT_BASE,
        IOAPIC_SIZE / 1024,
        FlagSummary(flags),
        flags
    );
    for i in 0..(IOAPIC_SIZE / PAGE_SIZE as u64) {
        let virt_addr = IOAPIC_VIRT_BASE + (i * PAGE_SIZE as u64);
        let phys_addr = IOAPIC_PHYS_BASE + (i * PAGE_SIZE as u64);
        map_page_alloc(pml4, virt_addr, phys_addr, flags, fa);
    }
}

/// Map kernel to high-half using a single 2 MiB page
/// Map the first 1GiB into the kernel high-half using 2MiB pages.
pub unsafe fn map_high_half_1gb_2mb<F: FrameSource>(pml4: &mut PageTable, fa: &mut F) {
    let two_mb: u64 = 2 * 1024 * 1024;
    let one_gb: u64 = 1024 * 1024 * 1024;
    let virt_base = KERNEL_VIRTUAL_BASE & !(two_mb - 1);
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL;
    let mut offset: u64 = 0;
    while offset < one_gb {
        let pa = offset;
        let va = virt_base + offset;
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        offset += two_mb;
    }
}

/// Map the kernel's physical image range into the high-half window using 2MiB pages
/// Map the kernel image into the high-half using 2MiB pages when possible.
pub unsafe fn map_kernel_high_half_2mb<F: FrameSource>(
    pml4: &mut PageTable,
    handoff: &theseus_shared::handoff::Handoff,
    fa: &mut F,
) {
    let two_mb: u64 = 2 * 1024 * 1024;
    let Some(extents) = kernel_mapping_extents(handoff) else {
        return;
    };
    // Mirror the low-guard region below the recorded kernel base so early boot code that
    // lives before the official start stays mapped once paging flips.
    let phys_guarded = extents.guarded_phys_start();
    let va_guarded = extents.guarded_va_start();

    let phys_start = phys_guarded & !(two_mb - 1);
    let phys_end = (extents.phys_base + extents.phys_size + two_mb - 1) & !(two_mb - 1);
    let va_start = va_guarded.wrapping_sub(phys_guarded.wrapping_sub(phys_start));
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL;
    let mut pa = phys_start;
    let mut va = va_start;
    while pa < phys_end {
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        pa += two_mb;
        va += two_mb;
    }
}

/// Map an existing kernel VA range to its corresponding PA using the kernel base translation.
/// Map an existing virtual region (assumed to correspond to the kernel image)
/// to the underlying physical addresses using the handoff's kernel base.
pub unsafe fn map_existing_region_va_to_its_pa<F: FrameSource>(
    pml4_phys: u64,
    handoff: &theseus_shared::handoff::Handoff,
    start_va: u64,
    size: u64,
    flags: u64,
    fa: &mut F,
) {
    if size == 0 {
        return;
    }
    let pml4: &mut PageTable = &mut *(pml4_phys as *mut PageTable);
    let offset = start_va.wrapping_sub(KERNEL_VIRTUAL_BASE);
    let phys_base = runtime_kernel_phys_base(handoff);
    let pa = phys_base.wrapping_add(offset);
    let mut builder = PageTableBuilder::new(pml4, fa);
    builder.map_range(start_va, pa, size, flags);
}

/// Map a range using a simple policy: prefer 2MiB huge pages when both VA and
/// PA are 2MiB-aligned and remaining size >= 2MiB, otherwise fall back to 4KiB
/// page mappings. This helper centralizes page-size decisions in one place.
/// Map a VA/PA range using a simple policy that prefers 2MiB pages when
/// possible and falls back to 4KiB pages for unaligned/tail regions.
///
/// The caller should supply base flags that do *not* include `PTE_PS`; the helper
/// sets the bit automatically when it materialises a huge-page leaf entry.
pub unsafe fn map_range_with_policy<F: FrameSource>(
    pml4: &mut PageTable,
    mut va: u64,
    mut pa: u64,
    mut size: u64,
    flags: u64,
    fa: &mut F,
) {
    const TWO_MB: u64 = 2 * 1024 * 1024;
    debug_assert_eq!(
        TWO_MB % PAGE_SIZE as u64,
        0,
        "2 MiB huge pages must be divisible by PAGE_SIZE"
    );
    // Handle leading unaligned portion to reach 2MiB alignment
    while size > 0 && (va & (TWO_MB - 1) != 0 || pa & (TWO_MB - 1) != 0) {
        // Bump each address in lockstep until both sides are aligned for a huge-page insert.
        map_page_alloc(pml4, va, pa, flags, fa);
        va = va.wrapping_add(PAGE_SIZE as u64);
        pa = pa.wrapping_add(PAGE_SIZE as u64);
        size = size.saturating_sub(PAGE_SIZE as u64);
    }
    // Map as many 2MiB pages as possible
    while size >= TWO_MB {
        map_2mb_page_alloc(pml4, va, pa, flags, fa);
        va = va.wrapping_add(TWO_MB);
        pa = pa.wrapping_add(TWO_MB);
        size = size.saturating_sub(TWO_MB);
    }
    // Tail with 4KiB pages
    while size > 0 {
        // Whatever remains is smaller than 2 MiB, so fall back to standard 4 KiB leaves.
        map_page_alloc(pml4, va, pa, flags, fa);
        va = va.wrapping_add(PAGE_SIZE as u64);
        pa = pa.wrapping_add(PAGE_SIZE as u64);
        size = size.saturating_sub(PAGE_SIZE as u64);
    }
}

/// Compact formatter for common x86_64 page-table flags.
struct FlagSummary(u64);

impl fmt::Display for FlagSummary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flags = self.0;
        let parts = [
            (PTE_PRESENT, "P"),
            (PTE_WRITABLE, "RW"),
            (PTE_USER, "US"),
            (PTE_PWT, "WT"),
            (PTE_PCD, "CD"),
            (PTE_ACCESSED, "A"),
            (PTE_DIRTY, "D"),
            (PTE_GLOBAL, "G"),
            (PTE_PS, "PS"),
            (PTE_NO_EXEC, "NX"),
        ];
        let mut first = true;
        for (bit, label) in parts {
            if flags & bit != 0 {
                if !first {
                    f.write_str("|")?;
                }
                f.write_str(label)?;
                first = false;
            }
        }
        if first {
            f.write_str("∅")?;
        }
        Ok(())
    }
}
