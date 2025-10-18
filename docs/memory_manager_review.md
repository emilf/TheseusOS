# Memory Manager Review

## High-Level Observations
- The early-page-table setup logic is split between `kernel/src/memory.rs` and helper modules (mapping, builder, temporary window). The split reduces file size but still leaves complex control flow inside `MemoryManager::new`.
- Both virtual and physical memory paths lean on ad-hoc helpers (`map_2mb_page_alloc`, `map_range_with_policy`, `TemporaryWindow`) that embed policy and safety requirements in code comments instead of reusable abstractions.
- Boot-time frame allocation mixes parsing of the UEFI memory map, tracking, and reserved-pool policy inside `BootFrameAllocator`, which makes the pointer arithmetic harder to audit.

## Suggested Improvements
- ✅ **Extract mapping phases**: `MemoryManager::new` now delegates to small helpers (`map_boot_identity_region`, `map_kernel_high_half_region`, etc.) so the boot sequence reads like a checklist.
- ✅ **Clarify high-half guard math**: Introduced `KernelMappingExtents` to compute guard-adjusted bases once, shared by both 4 KiB and 2 MiB mapping helpers.
- ✅ **Centralize mapping policy**: `map_range_with_policy` no longer fiddles with `PTE_PS`; huge-page flagging happens solely inside `map_2mb_page_alloc`.
- ✅ **Ensure TLB consistency**: `TemporaryWindow` flushes the `TEMP_WINDOW_VA` translation whenever it maps or unmaps a frame.
- ✅ **Encapsulate descriptor parsing**: Added `MemoryDescriptorView` to hide raw offset math when iterating the UEFI map.
- ✅ **Expose reserved-pool intent**: Wrapped the critical-frame stash in a `ReservedPool` helper with explicit `push`/`pop` semantics.
- ✅ **Validate handoff remapping**: Added assertions and logging before rewriting the handoff’s memory-map pointer to catch mismatches early.

## Simplifications & Future Work
- Introduce constants such as `const TWO_MB: u64` in a shared module to avoid repeated literal recomputation across helpers.
- Consider representing page-table indices with a dedicated type (or helper functions) to remove manual shifts scattered through the mapping code.
- Add smoke tests or debug asserts that walk the constructed PML4 after `MemoryManager::new`, verifying that the kernel image, phys-offset window, framebuffer, and heap regions are all present with expected flags.
- Expand Rustdoc examples to include end-to-end mapping snippets (e.g., using `PageTableBuilder::map_range` with a fake `FrameSource`) to serve as living documentation.
- Explore merging the boot-time `FrameSource` trait with the later `FrameAllocator` usage so that the same abstraction drives both early and steady-state mapping paths.

## Quick Wins
- ✅ Add inline assertions (e.g., `debug_assert!(PAGE_SIZE.is_power_of_two())`) at key mapping helpers to document assumptions.
- ✅ Replace manual loops that compute page counts with `size.div_ceil(PAGE_SIZE as u64)` once stabilized, or provide a small local helper to clarify intent.
- ✅ Expand the logging around MMIO mappings to include flag summaries, making it easier to audit cacheability choices at runtime.
