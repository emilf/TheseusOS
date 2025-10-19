# Memory Management

TheseusOS establishes its virtual memory layout during early kernel initialization and then transitions to a persistent allocator backed by the firmware memory map. This page covers the moving pieces so you can follow (and modify) the paging story confidently.

## Virtual Address Layout
Defined in `kernel/src/memory.rs`:

| Region | Virtual Base | Purpose |
| --- | --- | --- |
| Identity map | `0x0000_0000_0000_0000` | Low 1 GiB identity map used during the transition out of firmware mode. |
| Higher-half kernel | `KERNEL_VIRTUAL_BASE = 0xFFFF_FFFF_8000_0000` | Kernel text/data once paging is active. |
| Temporary heap | `TEMP_HEAP_VIRTUAL_BASE = 0xFFFF_FFFF_A000_0000` | Rebased copy of the bootloader-allocated heap. |
| Permanent heap | `KERNEL_HEAP_BASE = 0xFFFF_FFFF_B000_0000` (1 MiB) | Long-lived allocations once the allocator switches. |
| Physical window | `PHYS_OFFSET = 0xFFFF_8000_0000_0000` | Linear mapping: `virt = PHYS_OFFSET + phys`. |

Flags (`PTE_*`) are defined alongside these constants to make intent explicit when assembling PTEs.

## `MemoryManager`
Constructor: `MemoryManager::new(handoff: &Handoff) -> MemoryManager`.

Responsibilities:
- Clone the firmware memory map to identify usable RAM.
- Build a TLB-friendly PML4 with:
  - Identity map for `[0, 1 GiB)`.
  - Higher-half kernel image with guard pages (`runtime_kernel_phys_base` adjusts for relocations).
  - Framebuffer, LAPIC MMIO, and handoff windows.
  - Optional 1 GiB PHYS_OFFSET alias based on `config::MAP_LEGACY_PHYS_OFFSET_1GIB`.
- Track the physical address of the PML4 so CR3 can be loaded later (`MemoryManager::page_table_root`).
- Provide `jump_to_high_half(phys_base, after_high_half_entry)` which sets up the stack switch and transfers control.

Helpers live in `kernel/src/memory/mapping.rs` and `kernel/src/memory/page_tables.rs`. Notable routines:
- `map_range_with_policy` — maps arbitrary ranges with fine-grained flag control.
- `map_2mb_page_alloc` — 2 MiB huge-page builder that pulls frames from the reserved pool when possible.
- `TemporaryWindow` — temporarily maps frames at a fixed VA for zeroing or inspection without identity access.

## Frame Allocation During Boot
`BootFrameAllocator` (in `kernel/src/memory.rs`) interprets the UEFI memory descriptors:
- Filters for usable RAM, excluding firmware code/data and MMIO ranges.
- Maintains a small reserved stack of frames for critical allocations (page tables, IST stacks).
- Implements `FrameAllocator` from the `x86_64` crate so higher-level helpers can reuse it.
- Exposes `enable_tracking()` plus `drain_boot_consumed()` so that `physical_memory` knows which frames were already used during bring-up.

## Persistent Physical Allocator
After the permanent heap is ready, `physical_memory::init_from_handoff` takes over:
- Consumes the drained boot allocations to avoid double-freeing critical regions (stacks, page tables, framebuffer).
- Builds a bitmap allocator sized via `allocate_bitmap_storage` in `environment.rs`. The bitmap is allocated from fresh frames and zeroed via the PHYS_OFFSET window.
- Provides APIs for future subsystems to allocate and free physical pages safely once the system is running.

## PHYS_OFFSET Utilities
- `phys_to_virt_pa` and `virt_to_phys_pa` convert between address spaces once `set_phys_offset_active()` has been called.
- `virt_range_has_flags` verifies that specific virtual ranges (e.g., kernel stack, IST stacks) have expected permissions before running critical code.
- `zero_phys_range` and `zero_frame_safely` let code zero frames through PHYS_OFFSET or via the temporary window, depending on what is active.

## Runtime Tuning
- Toggle `config::MAP_LEGACY_PHYS_OFFSET_1GIB` to choose between a broad identity map and the leaner per-region mappings.
- `MemoryManager::log_layout_summary()` (enabled under verbose logging) prints the mapped ranges with flag summaries, which is invaluable when auditing new mappings.
- The monitor command `memlayout` (see [Hardware & Drivers](hardware-and-drivers.md)) walks the live tables for inspection.

## Suggested Reading
- Follow the boot path in [Kernel Architecture](kernel-architecture.md) to see how `MemoryManager` integrates with the broader environment setup.
- Inspect archived design notes in `docs/archive/memory_manager_review.md` for historical context and future improvement ideas.
