# Memory Manager Review

## High-Level Observations
- The early-page-table setup logic is split between `kernel/src/memory.rs` and helper modules (mapping, builder, temporary window). The split reduces file size but still leaves complex control flow inside `MemoryManager::new`.
- Both virtual and physical memory paths lean on ad-hoc helpers (`map_2mb_page_alloc`, `map_range_with_policy`, `TemporaryWindow`) that embed policy and safety requirements in code comments instead of reusable abstractions.
- Boot-time frame allocation mixes parsing of the UEFI memory map, tracking, and reserved-pool policy inside `BootFrameAllocator`, which makes the pointer arithmetic harder to audit.

## Suggested Improvements
- **Extract mapping phases**: Break `MemoryManager::new` into clearly named private methods (e.g., `identity_map_low_memory`, `map_kernel_image`, `map_phys_offset_regions`, `map_mmio_devices`). Each method can capture its logging/guard math, making the boot sequence easier to follow and test in isolation.
- **Clarify high-half guard math**: The guard-window computation in `map_kernel_high_half_4k_alloc` mixes padding and guard handling. Consider introducing a helper that returns `(va_start, pa_start, length)` and documents why a page is subtracted when `total_bytes >= PAGE_SIZE`.
- **Centralize mapping policy**: `map_2mb_page_alloc` always ORs `PTE_PS`, yet `map_range_with_policy` also sets the bit. Removing the duplication (and making the expected flag shape explicit in one place) reduces subtle flag mismatches.
- **Ensure TLB consistency**: `TemporaryWindow::map_phys_frame` and `unmap` write raw entries without issuing `invlpg`. After re-mapping the window to a new frame, the CPU may still hold the old translation. Add an `x86_64::instructions::tlb::flush()` or explicit `invlpg` on `TEMP_WINDOW_VA` to guarantee correctness.
- **Encapsulate descriptor parsing**: Move the hard-coded offsets in `BootFrameAllocator::advance_to_next_region` into a small `MemoryDescriptorView` helper. This would make the iterator logic clearer and safer against descriptor-layout changes.
- **Expose reserved-pool intent**: The reserved-frame pool currently lives in a fixed-size `[u64; 16]`. A lightweight wrapper (e.g., `ReservedPool`) with methods like `push`, `pop`, `is_full` would surface the policy better and allow unit tests.
- **Validate handoff remapping**: When remapping the UEFI memory map buffer, the code patches the handoff pointer in place. Add a small verification step (length sanity, alignment) or log assertion to catch truncated buffers.

## Simplifications & Future Work
- Introduce constants such as `const TWO_MB: u64` in a shared module to avoid repeated literal recomputation across helpers.
- Consider representing page-table indices with a dedicated type (or helper functions) to remove manual shifts scattered through the mapping code.
- Add smoke tests or debug asserts that walk the constructed PML4 after `MemoryManager::new`, verifying that the kernel image, phys-offset window, framebuffer, and heap regions are all present with expected flags.
- Expand Rustdoc examples to include end-to-end mapping snippets (e.g., using `PageTableBuilder::map_range` with a fake `FrameSource`) to serve as living documentation.
- Explore merging the boot-time `FrameSource` trait with the later `FrameAllocator` usage so that the same abstraction drives both early and steady-state mapping paths.

## Quick Wins
- Add inline assertions (e.g., `debug_assert!(PAGE_SIZE.is_power_of_two())`) at key mapping helpers to document assumptions.
- Replace manual loops that compute page counts with `size.div_ceil(PAGE_SIZE as u64)` once stabilized, or provide a small local helper to clarify intent.
- Expand the logging around MMIO mappings to include flag summaries, making it easier to audit cacheability choices at runtime.
