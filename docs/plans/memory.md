# Memory Plan

## Scope

Track paging, heap bring-up, handoff memory-map access, and runtime physical-memory allocation.

This plan records what is already implemented, not aspirational design.

## Plan vs Repo

- Existing docs explained the layout fairly well, but they blended current truth with forward-looking cleanup work.
- The current implementation still carries transitional behavior such as temporary-heap rebasing and optional identity unmapping; that should be explicit.

## Implemented Invariants

- [x] IMPLEMENTED: `MemoryManager::new` builds a fresh PML4 and maps identity memory, the higher-half kernel, boot resources, PHYS_OFFSET, and APIC MMIO regions.
- [x] IMPLEMENTED: The higher-half kernel base is `KERNEL_VIRTUAL_BASE = 0xFFFF_FFFF_8000_0000`.
- [x] IMPLEMENTED: Physical memory becomes accessible through `PHYS_OFFSET = 0xFFFF_8000_0000_0000` after CR3 is switched and `set_phys_offset_active()` is called.
- [x] IMPLEMENTED: The bootloader allocates a temporary heap and records it in the handoff for early kernel allocations.
- [x] IMPLEMENTED: The bootloader-side non-overlapping allocation helper is a retry-based mitigation that tries to keep the temporary heap away from the conservatively estimated kernel image span.
- [x] IMPLEMENTED: The bootloader also carries firmware-side allocation/free helpers and an experimental runtime-services virtual-address-map helper, but those are not the kernel's primary higher-half paging architecture.
- [x] IMPLEMENTED: After the high-half transition, the kernel maps the temporary heap, then maps a permanent heap at `KERNEL_HEAP_BASE` and switches the shared allocator to it.
- [x] IMPLEMENTED: The persistent physical allocator is initialized from the UEFI memory map after the permanent heap is available.
- [x] IMPLEMENTED: The handoff memory-map buffer is remapped into a dedicated high virtual window during paging setup.

## TODO

- [ ] TODO: Document the exact conditions under which identity mappings should remain enabled versus being removed during bring-up.
- [ ] TODO: If exact kernel image extent accounting replaces the current conservative span, update the higher-half mapping documentation and related axioms.
- [ ] TODO: Add optional module overview docs under `docs/modules/` if memory code grows enough to benefit from per-subtree docs.

## Risks

- [!] RISK: The current boot flow still depends on transitional boot-time mapping behavior while stacks, heaps, and runtime services are being migrated fully into stable high-half operation.
- [!] RISK: Temporary heap teardown is optional and therefore stale low-assumption bugs may hide until the stricter configuration is enabled.
- [!] RISK: Bootloader-side heap placement still relies on retrying UEFI allocation to dodge a conservatively estimated image range, so stronger wording would overstate exactness.

## Related Axioms

- `../axioms/memory.md#A1:-The-kernel-executes-from-a-higher-half-virtual-base`
- `../axioms/memory.md#A2:-Physical-memory-is-accessed-through-a-fixed-PHYS_OFFSET-linear-mapping-after-paging-is-active`
- `../axioms/memory.md#A3:-The-boot-path-keeps-a-temporary-heap-before-switching-to-a-permanent-kernel-heap`
- `../axioms/memory.md#A4:-The-persistent-physical-allocator-is-initialized-from-the-UEFI-memory-map-after-the-permanent-heap-exists`

## Implementing Modules

- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `shared/src/handoff.rs`
- `kernel/src/handoff.rs`
- `kernel/src/memory.rs`
- `kernel/src/memory/mapping.rs`
- `kernel/src/memory/frame_allocator.rs`
- `kernel/src/memory/page_tables.rs`
- `kernel/src/memory/page_table_builder.rs`
- `kernel/src/memory/temporary_window.rs`
- `kernel/src/physical_memory.rs`
- `kernel/src/environment.rs`
