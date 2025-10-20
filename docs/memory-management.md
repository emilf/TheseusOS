# Memory Management

TheseusOS spends a large portion of its boot bringing up a predictable virtual memory layout. The `kernel::memory` module owns that responsibility and exposes a set of helpers that other subsystems can rely on when they need to look at physical memory, map device MMIO, or zero frames safely. This page explains the architecture, highlights the key data structures, and provides concrete examples of how to use the APIs from elsewhere in the kernel.

- New to the boot flow? Start with [Kernel Architecture](kernel-architecture.md) for the timeline that surrounds these calls.
- Want to dive into historical design notes? See `docs/archive/memory_manager_review.md`.

## Architecture Overview

The memory subsystem is deliberately split into three phases that mirror the boot lifecycle:

1. **Boot construction**
   - `MemoryManager::new` builds fresh page tables using the firmware-provided handoff.
   - Mapping helpers in `memory/mapping.rs` populate the identity region, higher-half mirror, PHYS_OFFSET window, framebuffer, LAPIC, IOAPIC, and bootloader-provided heaps.
   - `BootFrameAllocator` feeds physical frames to the mapper, keeping a reserved stack for critical tables.

2. **High-half transition**
   - `MemoryManager::jump_to_high_half` switches stacks and jumps into `after_high_half_entry`.
   - `set_phys_offset_active` flips a global flag so address conversion helpers become safe to use.

3. **Runtime services**
   - The temporary heap is remapped and then replaced with a permanent heap.
   - `physical_memory::init_from_handoff` turns the firmware memory map into the long-lived bitmap allocator.
   - Identity mappings are optionally torn down to catch stale low-half pointers.

All of the helpers discussed below live in `kernel/src/memory.rs` unless otherwise noted.

## Virtual Address Layout

| Region | Constant | Default Virtual Base | Purpose |
| --- | --- | --- | --- |
| Identity map | — | `0x0000_0000_0000_0000` | Low 1 GiB, used only during the transition away from firmware addresses. |
| Higher-half kernel | `KERNEL_VIRTUAL_BASE` | `0xFFFF_FFFF_8000_0000` | Where kernel code/data execute once paging flips. |
| Temporary heap | `TEMP_HEAP_VIRTUAL_BASE` | `0xFFFF_FFFF_A000_0000` | Firmware-allocated heap rebased into the high half. |
| Permanent heap | `KERNEL_HEAP_BASE` | `0xFFFF_FFFF_B000_0000` | The allocator’s long-term arena (configurable size). |
| Physical window | `PHYS_OFFSET` | `0xFFFF_8000_0000_0000` | Linear mapping: `virt = PHYS_OFFSET + phys`. |
| Handoff window | `HANDOFF_MEMMAP_WINDOW_BASE` | `0xFFFF_FF88_0000_0000` | Scratch space for inspecting firmware descriptors. |

Page-table flag bit definitions (`PTE_*`) sit next to these constants; reach for them instead of hand-coding bitmasks.

## Key Data Structures

### `MemoryManager`
Located in `kernel/src/memory.rs`.

- **Constructor**: `unsafe fn new(handoff: &Handoff) -> MemoryManager`
  - Consumes the firmware handoff, clones the memory map, and builds a ready-to-load PML4.
  - Returns a manager that tracks the root table (both virtually and physically) plus heap bounds.
- **Important methods**:
  - `fn page_table_root(&self) -> u64` — physical address for CR3.
  - `fn kernel_heap_bounds(&self) -> (u64, u64)` — useful for sanity checks before enabling the allocator.
  - `unsafe fn jump_to_high_half(&self, kernel_phys_base: u64, cont: extern "C" fn() -> !) -> !` — switch to the kernel stack and continue execution in the higher half.

Because these methods touch page tables and raw pointers, they are intentionally `unsafe`. Modules that consume them must uphold the documented invariants (validated handoff, interrupts disabled, etc.).

### `BootFrameAllocator`

- Wraps the firmware memory map descriptors, filtering for usable frames.
- Maintains a **reserved stack** (`reserve_frames`, `allocate_reserved_frame`) to guarantee that page-table allocations never fail while the rest of the system is still bootstrapping.
- Implements `x86_64::structures::paging::FrameAllocator`. Any code that requires a `FrameAllocator` before the persistent allocator comes online can borrow it.
- Once the permanent allocator is initialized, call `drain_boot_consumed()` so those early allocations are marked as in-use and never returned to the free pool.

### `PersistentFrameAllocator`

Defined in `kernel/src/physical_memory.rs`.

- Zero-sized adapter that implements `FrameAllocator<Size4KiB>` and `memory::FrameSource` by delegating to the global persistent allocator (`physical_memory::alloc_frame`).
- Use this *after* `physical_memory::init_from_handoff` completes; it is the preferred way for runtime code to obtain frames for new mappings.
- Pairs with `physical_memory::free_frame` when a mapping is torn down.

### `TemporaryWindow`

`TemporaryWindow` (see `memory.rs` near the bottom) maps arbitrary frames at a fixed virtual address so you can inspect or zero them without relying on identity mappings.

- `fn new(pml4: u64) -> TemporaryWindow` — takes the physical address of the PML4.
- `unsafe fn map_phys_frame(&mut self, pa: u64)` — installs the mapping and flushes the relevant TLB entry.
- `unsafe fn map_and_zero_frame(&mut self, pa: u64)` — convenience wrapper that also zeroes the frame.
- `fn unmap(&mut self)` — removes the mapping and flushes again.

Use this when `PHYS_OFFSET` is not yet active or when you want to keep the identity map disabled for guard purposes.

### Mapping Helpers (`memory/mapping.rs`)

- `map_page_alloc`, `map_2mb_page_alloc` — low-level insertions for 4 KiB and 2 MiB entries.
- `map_range_with_policy` — automatically prefers huge pages when aligned, falls back to 4 KiB leaves otherwise.
- `map_framebuffer_alloc`, `map_lapic_mmio_alloc`, `map_io_apic_mmio_alloc` — ready-made recipes for specific devices.
- `map_existing_region_va_to_its_pa` — mirror an already-virtual region (e.g., kernel image) using the recorded physical base.

`PageTableBuilder` (inside `page_tables.rs`) powers most of these routines. If you need to batch map a contiguous range, it offers `map_range` and `map_flags` helpers.

## Initialization Flow

The sequence below is the canonical order enforced by `environment::setup_kernel_environment` and `continue_after_stack_switch` (see `kernel/src/environment.rs`).

1. `MemoryManager::new(handoff)` constructs new tables and returns `mm`.
2. `activate_virtual_memory(mm.page_table_root())` loads CR3. From here on, only the new PML4 is active.
3. `set_phys_offset_active()` flips the global flag, making address helpers safe.
4. `mm.jump_to_high_half(runtime_kernel_phys_base(handoff), after_high_half_entry)` switches stacks and enters the high half.
5. Once running in `continue_after_stack_switch`:
   - `map_existing_region_va_to_its_pa` is used to pin IST stacks and kernel stack pages.
   - The temporary heap is mapped through `map_temporary_heap_alloc`.
   - `map_kernel_heap_x86` maps the permanent heap using the runtime frame allocator.
6. `physical_memory::init_from_handoff` constructs the persistent allocator.
7. Optional: `unmap_temporary_heap_x86` and `unmap_identity_kernel_x86` remove bootstrap-era mappings to expose stale references quickly.

If you need to insert additional mappings during this phase (for example, to bring up a driver earlier in boot), follow the same pattern: obtain a mapper (`OffsetPageTable`), use the mapping helpers, and record those page-table frames via the boot allocator (pre-init) or the persistent allocator (post-init) so bookkeeping stays accurate.

## API Surface for Other Modules

Most subsystems interact with `crate::memory` through a small set of safe wrappers. Use these instead of duplicating paging logic.

### Address Conversion

```rust
use crate::memory::{phys_to_virt_pa, virt_to_phys_pa, set_phys_offset_active, PHYS_OFFSET};

// After set_phys_offset_active() has been called:
let phys = 0x0012_3400;
let virt = phys_to_virt_pa(phys);
assert_eq!(virt, PHYS_OFFSET + phys);

let back_to_phys = virt_to_phys_pa(virt).expect("mapping present");
```

- `set_phys_offset_active()` must run once the PHYS_OFFSET mapping exists; `environment::setup_kernel_environment` handles this automatically.
- `virt_to_phys_pa` returns `Option<u64>` to signal unmapped pages. Check it before dereferencing.

### Mapping Existing Regions

```rust
use crate::memory::{map_existing_region_va_to_its_pa, PTE_PRESENT, PTE_WRITABLE};
use crate::physical_memory::PersistentFrameAllocator;

unsafe {
    let mut frames = PersistentFrameAllocator;
    map_existing_region_va_to_its_pa(
        pml4_phys,
        handoff,
        some_virtual_base,
        size_bytes,
        PTE_PRESENT | PTE_WRITABLE,
        &mut frames,
    );
}
```

Reach for this when the virtual region already exists but you want to refresh or tighten the flags (e.g., remapping the kernel stack with `PTE_NO_EXEC`).

### Mapping New Device Memory

```rust
use crate::memory::{map_range_with_policy, PTE_PCD, PTE_PRESENT, PTE_PWT, PTE_WRITABLE};
use crate::physical_memory::PersistentFrameAllocator;
use x86_64::structures::paging::{OffsetPageTable, PageTable as X86PageTable};
use x86_64::VirtAddr;

unsafe fn map_device_mmio(
    mapper: &mut OffsetPageTable<'static>,
    frame_alloc: &mut PersistentFrameAllocator,
    virt_base: u64,
    phys_base: u64,
    size: u64,
) {
    let l4_ptr = mapper.level_4_table() as *mut X86PageTable;
    let flags = PTE_PRESENT | PTE_WRITABLE | PTE_PCD | PTE_PWT;
    map_range_with_policy(&mut *l4_ptr, virt_base, phys_base, size, flags, frame_alloc);
}

// Example call inside a driver init function:
unsafe {
    let (frame, _) = x86_64::registers::control::Cr3::read();
    let pml4_pa = frame.start_address().as_u64();
    let l4_va = crate::memory::phys_to_virt_pa(pml4_pa) as *mut X86PageTable;
    let mut mapper = OffsetPageTable::new(&mut *l4_va, VirtAddr::new(crate::memory::PHYS_OFFSET));
    let mut frames = PersistentFrameAllocator;
    map_device_mmio(
        &mut mapper,
        &mut frames,
        0xFFFF_FF90_0000_0000,
        device_phys,
        device_size,
    );
}
```

Guidelines:
- Always choose the correct cacheability flags (`PTE_PCD`/`PTE_PWT`) for MMIO regions.
- Use the persistent allocator once it is initialized so `physical_memory` can reclaim frames later.
- When tearing down mappings, return frames with `physical_memory::free_frame`.

### Zeroing and Inspecting Frames

```rust
use crate::memory::{zero_phys_range, TemporaryWindow};

// When PHYS_OFFSET is active:
zero_phys_range(phys_start, length_bytes);

// Before PHYS_OFFSET is available:
unsafe {
    let mut window = TemporaryWindow::new(pml4_phys);
    window.map_and_zero_frame(phys_frame);
    window.unmap();
}
```

`zero_phys_range` automatically chooses between PHYS_OFFSET and the temporary window depending on the current boot stage, so prefer it when possible.

### Checking Mappings

```rust
use crate::memory::{virt_range_has_flags, PTE_PRESENT, PTE_WRITABLE};

if !virt_range_has_flags(stack_base, stack_size, PTE_PRESENT | PTE_WRITABLE) {
    crate::boot_abort!("kernel stack missing mapping", stack_base);
}
```

This helper is frequently used before entering critical sections (stack switch, IST installation) to catch configuration problems early.

## Working Pattern Examples

### Bringing Up a New Device Driver
1. Ensure the device’s MMIO range or frame buffer is described in the handoff or ACPI tables.
2. Inside your driver’s `init`:
   - Acquire an `OffsetPageTable` rooted at the active PML4 (`phys_to_virt_pa` + `OffsetPageTable::new`).
   - Request frames via `PersistentFrameAllocator` (or call `physical_memory::alloc_frame` directly for single frames). During the earliest boot phase, fall back to `BootFrameAllocator`.
   - Call `map_range_with_policy` or the purpose-built helper that matches your device.
   - When a mapping is removed, release frames with `physical_memory::free_frame`.
3. Use `phys_to_virt_pa` to convert any physical pointers you received from firmware into usable virtual addresses.

### Extending the Kernel Heap
If you grow `KERNEL_HEAP_SIZE`:
1. Update the constant in `kernel/src/memory.rs`.
2. Adjust `map_kernel_heap_x86` (in the same file) to cover the larger range, preferably using `map_range_with_policy`.
3. Ensure `allocate_bitmap_storage` in `environment.rs` still reserves enough metadata pages.
4. Verify at runtime using the monitor: `> heap` summarizes both temporary and permanent heap usage.

## Debugging Tips

- Enable verbose logging (`config::VERBOSE_KERNEL_OUTPUT = true`) to see each mapping stage summarized during boot.
- The monitor command `memlayout` prints all mapped ranges along with their flags using live page-table walks.
- Use `log::trace!` calls inside mapping helpers if you suspect a specific range is missing (be mindful of log volume).
- When a mapping appears correct but accesses fault, check CR3 and the relevant PTE flags with `virt_range_has_flags`.

## Safety Checklist

When calling `unsafe` memory helpers from other modules:
- Validate and sanitize all physical addresses you consume (e.g., ensure MMIO ranges are page-aligned).
- Disable interrupts around critical remapping so ISRs do not see partially updated tables.
- Flush the TLB (`x86_64::instructions::tlb::flush()`) if you manually modify PTEs without going through the provided helpers.
- Free any frames you allocate once the mapping is torn down (`physical_memory::free_frame`) to avoid leaks; reserve logging via `record_boot_consumed_region` is only for the pre-initialization window.

## Continue Exploring

- Review the broader boot context in [Kernel Architecture](kernel-architecture.md).
- Learn about device drivers and interrupt wiring in [Hardware & Drivers](hardware-and-drivers.md).
- For development workflows (debugging `MemoryManager`, using the monitor), see [Development & Debugging](development-and-debugging.md).
