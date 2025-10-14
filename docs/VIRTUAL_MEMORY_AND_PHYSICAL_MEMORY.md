# TheseusOS Virtual & Physical Memory Overview

This guide explains how the TheseusOS kernel establishes its virtual address
space, transitions into the high half, and manages physical frames during and
after boot. It reflects the current state of the memory subsystem as implemented
in the Rust sources under `kernel/src/memory`.

## Boot-Time Page Table Construction

The early boot code constructs all initial mappings inside
`MemoryManager::new()` (`kernel/src/memory.rs`). The sequence is:

1. **Frame allocator bootstrap** – `BootFrameAllocator::from_handoff()` walks the
   UEFI memory map supplied in the `Handoff` structure and prepares a pool of
   usable 4 KiB frames. It attempts to reserve up to 16 frames for critical work
   so page-table creation will never starve even if the general pool is
   temporarily exhausted.
2. **PML4 allocation** – A fresh frame is taken from the allocator, zeroed via
   `zero_frame_safely()`, and treated as the root page table.
3. **Identity map (low 1 GiB)** – `identity_map_first_1gb_2mb_alloc()` installs
   2 MiB entries covering the first gigabyte of physical memory. This keeps
   early code, MMIO, and bootloader data reachable while the kernel moves into
   the high half.
4. **High-half kernel mapping** – `map_kernel_high_half_4k_alloc()` maps the
   kernel image at `KERNEL_VIRTUAL_BASE` (`0xFFFFFFFF80000000`) using 4 KiB pages
   and an 8 MiB safety pad to cover .bss growth.
5. **Framebuffer and temporary heap** – If the handoff provided them,
   `map_framebuffer_alloc()` maps the GOP framebuffer at
   `0xFFFFFFFF90000000`, and `map_temporary_heap_alloc()` maps the boot
   loader’s staging heap at `TEMP_HEAP_VIRTUAL_BASE`
   (`0xFFFFFFFFA0000000`).
6. **PHYS_OFFSET linear window** – Depending on
   `config::MAP_LEGACY_PHYS_OFFSET_1GIB`, either
   `map_phys_offset_1gb_2mb_alloc()` maps the first GiB of DRAM at
   `PHYS_OFFSET` (`0xFFFF800000000000`) using 2 MiB large pages, or
   `map_phys_offset_range_2mb_alloc()` maps a smaller region and relocates the
   handoff memory map into a high virtual address window.
7. **MMIO regions** – LAPIC (`0xFEE00000`) and IOAPIC (`0xFEC00000`) ranges are
   mapped one page at a time with cache-disable (`PTE_PCD`) and write-through
   (`PTE_PWT`) bits set so interrupt controllers remain accessible after paging
   turns on.

When the setup finishes, `MemoryManager` returns the PML4 pointer, its physical
address, and the still-usable `BootFrameAllocator` for later mappings.

## High-Half Transition

* `activate_virtual_memory()` loads the new PML4 frame into CR3. The function
  logs the before/after CR3 value for debugging.
* Once CR3 is active, `set_phys_offset_active()` marks the PHYS_OFFSET linear
  mapping as usable. From this point the kernel can translate physical addresses
  with `phys_to_virt_pa()`.
* `MemoryManager::jump_to_high_half()` computes the high-half entry address for
  the provided kernel entry symbol, verifies that the translation works through
  an `OffsetPageTable`, and performs a non-returning jump into the high half.

After the jump succeeds the kernel operates entirely from the high-half
addresses defined in `memory.rs` (`KERNEL_VIRTUAL_BASE`, `KERNEL_HEAP_BASE`,
`TEMP_HEAP_VIRTUAL_BASE`, etc.).

## Mapping Helpers and Policies

Low-level mapping routines live in `kernel/src/memory/mapping.rs` and are shared
between the boot-time builder and later runtime code:

* `map_page_alloc()` and `map_2mb_page_alloc()` create final-level entries,
  allocating intermediate tables with `get_or_create_page_table_alloc()` as
  needed.
* `map_range_with_policy()` centralizes the “prefer 2 MiB huge pages when both
  VA and PA are aligned” rule and falls back to 4 KiB mappings for unaligned
  prefixes or tails.
* Specialized helpers such as `map_framebuffer_alloc()` and
  `map_lapic_mmio_alloc()` call into the policy routine with the correct access
  flags for their regions.

`PageTableBuilder` wraps these helpers to simplify mapping contiguous spans and
is used when mapping the kernel image and the boot structures.

## Physical Memory Management

Physical frame allocation is provided by `BootFrameAllocator`
(`kernel/src/memory/frame_allocator.rs`). Key behaviors:

* **Descriptor scan** – The allocator iterates the UEFI memory map, selecting
  regions with type `UEFI_CONVENTIONAL_MEMORY` (value `7`). It skips frame zero
  to avoid null-pointer aliasing and aligns region starts to 4 KiB.
* **Reserved pool** – `reserve_frames()` pulls up to 16 frames into a LIFO pool.
  Page-table allocation tries `allocate_reserved_frame()` first and uses the
  general pool only if the reserved stack is empty, guaranteeing progress for
  critical structures.
* **General allocation** – `allocate_frame()` returns monotonically increasing
  frames from the current descriptor and advances to the next descriptor on
  exhaustion.
* **Runtime reuse** – Later subsystems (e.g., `map_kernel_heap_x86()` in
  `environment.rs`) build new `BootFrameAllocator` instances from the handoff
  when they need additional frames, ensuring consistent bookkeeping.

### Zeroing and Temporary Access

Page tables must be zeroed before use. `get_or_create_page_table_alloc()` zeros
newly allocated frames by:

1. Preferring the reserved pool to obtain the frame.
2. If PHYS_OFFSET is active, acquiring a `TemporaryWindow` (see
   `kernel/src/memory/temporary_window.rs`) tied to the current PML4, mapping
   the frame at `TEMP_WINDOW_VA` (`0xFFFF_FFFE_0000_0000`), zeroing it, and then
   unmapping.
3. Falling back to identity writes when paging is not yet active.

`TemporaryWindow` also exposes `map_and_zero_frame()` and `with_mapped_frame()`
helpers for other ad-hoc physical accesses that must respect the current paging
state.

### PHYS_OFFSET Utilities

Once `PHYS_OFFSET` is live, helpers in `memory.rs` provide diagnostic access:

* `virt_addr_is_mapped()` and `virt_addr_has_flags()` let callers confirm that a
  virtual address (or range via `virt_range_has_flags()`) is mapped with the
  expected permission bits.
* `zero_phys_range()` and `zero_frame_safely()` clear physical memory segments
  through the linear mapping, falling back to identity writes when the mapping
  is not yet active.

## Permanent Heap Bring-Up

After the kernel migrates into the high half, `setup_kernel_environment()`
(`kernel/src/environment.rs`) finalizes heap management:

1. A temporary allocator is armed over the bootloader-provided heap that was
   mapped during boot.
2. A fresh `BootFrameAllocator` is created from the handoff and used with
   `map_kernel_heap_x86()` to back the permanent heap at `KERNEL_HEAP_BASE`
   (`0xFFFFFFFFB0000000`) for 1 MiB.
3. The global allocator switches to the permanent heap, the temporary heap is
   optionally unmapped via `unmap_temporary_heap_x86()`, and the low identity
   mapping of the kernel image is removed with `unmap_identity_kernel_x86()` to
   catch stray low-address references.

## Configuration Knobs

* `MAP_LEGACY_PHYS_OFFSET_1GIB` in `kernel/src/config.rs` controls whether the
  PHYS_OFFSET mapping spans a full GiB or only the minimal range needed for the
  handoff structures.
* The constants `KERNEL_HEAP_BASE` and `KERNEL_HEAP_SIZE` in
  `kernel/src/memory.rs` define the size and location of the first-stage
  permanent heap (currently 1 MiB starting at `0xFFFFFFFFB0000000`).

## Summary

TheseusOS now boots with a clean separation between boot-time identity access
and the high-half virtual layout. The combination of the reserved-frame pool,
the PHYS_OFFSET linear mapping, and the temporary window utility ensures that
page-table construction and later runtime mappings succeed even under memory
pressure, while the environment setup finishes the transition by switching the
global allocator to a dedicated high-half heap and unmapping obsolete regions.
