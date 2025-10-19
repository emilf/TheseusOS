# Physical Memory Manager Roadmap

## Context & Goals

- Summarize current boot-time flow in `kernel/src/memory.rs` and `kernel/src/memory/frame_allocator.rs` to confirm how the handoff map and temporary heap are used today.
- Define the long-term objective: a reclaimable, high-half friendly physical frame manager that survives past early boot and supports allocation/free APIs.

## Implementation Outline

1. Catalogue Existing Memory Sources

- Inspect `BootFrameAllocator::from_handoff()` and capture all regions (conventional, reserved, temp heap, framebuffer) into an intermediary description.
- Record requirements for preserving allocations already consumed by page tables, stacks, and heaps.

2. Design Persistent Frame Manager

- Introduce a new module (e.g. `kernel/src/memory/physical.rs`) that owns a bitmap or buddy allocator over the remaining free frames.
- Model zones or tags for special regions (MMIO, DMA limits) using data from the UEFI descriptors.

3. Bootstrap During Kernel Bring-Up

- Extend `MemoryManager::new()` to build the persistent manager after the high-half mappings are live, seeding it with unallocated frames and marking reserved ones.
- Ensure metadata (bitmaps, structures) lives in the kernel heap or a dedicated mapping that does not consume large identity ranges.

4. Provide Allocation APIs & Integration

- Expose safe wrappers (e.g. `alloc_frame()`, `alloc_contiguous(n)`, `free_frame()`) that other subsystems can call once boot-time code hands off.
- Update early clients (`page_table_builder`, future paging code) to migrate from `BootFrameAllocator` to the new manager after initialization.

5. Reclaim & Cleanup Boot Resources

- Release temporary boot structures (temporary heap frames, identity map tables) back into the persistent manager once they are unmapped.
- Document and, if feasible, add hooks so ACPI or driver init can return frames when no longer needed.

6. Diagnostics & Testing

- Add instrumentation (e.g. `kernel/src/monitor.rs`) to dump allocator state and counters.
- Extend `tests/kernel_tests.rs` or create new integration tests to exercise allocation/free paths with simulated memory maps.

## Deliverables

- Updated code under `kernel/src/memory/` implementing the persistent frame allocator.
- Adjusted boot sequence in `kernel/src/environment.rs` (or related entry) to initialize and publish the new allocator.
- Documentation refresh in `docs/MEMORY_LAYOUT.md` (and possibly a new design note) explaining the physical memory lifecycle.