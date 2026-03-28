# Memory Axioms

Binding architectural truths for paging, heaps, and physical-memory management.

## A1: The kernel executes from a higher-half virtual base

**REQUIRED**

The kernel's linked and runtime higher-half base is `0xFFFF_FFFF_8000_0000`.

Implements / evidence:
- `kernel/src/memory.rs#KERNEL_VIRTUAL_BASE`
- `bootloader/src/boot_sequence.rs#set_kernel_image_from_loaded_image`

Related plans:
- `../plans/memory.md#implemented-invariants`

Affected modules:
- `bootloader/src/boot_sequence.rs`
- `kernel/src/memory.rs`
- `kernel/src/environment.rs`

## A2: Physical memory is accessed through a fixed PHYS_OFFSET linear mapping after paging is active

**REQUIRED**

Once the kernel loads its new `CR3` and calls `set_phys_offset_active`, physical addresses are translated through `PHYS_OFFSET = 0xFFFF_8000_0000_0000`.

Implements / evidence:
- `kernel/src/memory.rs#PHYS_OFFSET`
- `kernel/src/memory.rs#set_phys_offset_active`
- `kernel/src/memory.rs#phys_to_virt_pa`
- `kernel/src/environment.rs#setup_kernel_environment`

Related plans:
- `../plans/memory.md#implemented-invariants`

Affected modules:
- `kernel/src/memory.rs`
- `kernel/src/physical_memory.rs`
- `kernel/src/acpi/mod.rs`
- `kernel/src/interrupts/apic.rs`
- `kernel/src/environment.rs`

## A3: The boot path keeps a temporary heap before switching to a permanent kernel heap

**REQUIRED**

The bootloader allocates a temporary heap and records it in the handoff. After the high-half transition, the kernel maps that heap, then creates and switches to a permanent heap at `KERNEL_HEAP_BASE`.

Implements / evidence:
- `bootloader/src/main.rs` (temp heap allocation)
- `shared/src/handoff.rs` (`temp_heap_base`, `temp_heap_size`)
- `kernel/src/environment.rs#continue_after_stack_switch`
- `kernel/src/memory.rs#KERNEL_HEAP_BASE`

Related plans:
- `../plans/memory.md#implemented-invariants`

Affected modules:
- `bootloader/src/main.rs`
- `bootloader/src/memory.rs`
- `bootloader/src/boot_sequence.rs`
- `shared/src/handoff.rs`
- `kernel/src/handoff.rs`
- `kernel/src/environment.rs`
- `kernel/src/memory.rs`

## A3.5: The kernel VA allocator manages a dedicated dynamic region

**REQUIRED**

Runtime kernel subsystems that need virtual address space (runtime mappings,
kernel stacks, device MMIO windows) allocate from the VA allocator region
`0xFFFF_9000_0000_0000 .. 0xFFFF_B000_0000_0000`.

This region is well clear of all statically-assigned VA windows:

| Region | VA range |
|--------|----------|
| PHYS_OFFSET | `0xFFFF_8000_0000_0000` |
| **VA allocator** | `0xFFFF_9000_0000_0000 .. 0xFFFF_B000_0000_0000` |
| ACPI window | `0xFFFF_FF80_0000_0000` |
| TemporaryWindow | `0xFFFF_FFFE_0000_0000` |
| KERNEL_VIRTUAL_BASE | `0xFFFF_FFFF_8000_0000` |
| Framebuffer | `0xFFFF_FFFF_9000_0000` |
| TEMP_HEAP | `0xFFFF_FFFF_A000_0000` |
| KERNEL_HEAP | `0xFFFF_FFFF_B000_0000` |

Implements / evidence:
- `kernel/src/memory/va_alloc.rs`

Related plans:
- `../plans/memory.md`

Affected modules:
- `kernel/src/memory/va_alloc.rs`
- `kernel/src/memory/runtime_mapper.rs` (future consumer)
- `kernel/src/memory/stack_alloc.rs` (future consumer)

## A4: The persistent physical allocator is initialized from the UEFI memory map after the permanent heap exists

**REQUIRED**

The runtime physical-memory allocator is a bitmap allocator initialized from the handoff memory map only after the permanent heap is available.

Implements / evidence:
- `kernel/src/physical_memory.rs#init_from_handoff`
- `kernel/src/environment.rs#continue_after_stack_switch`

Related plans:
- `../plans/memory.md#implemented-invariants`

Affected modules:
- `shared/src/handoff.rs`
- `kernel/src/physical_memory.rs`
- `kernel/src/environment.rs`
- `kernel/src/memory/frame_allocator.rs`
- `kernel/src/handoff.rs`

## A5: Allocator Handoff

**REQUIRED**

The transition from early-boot memory consumption to the runtime physical allocator follows a strict timeline:

1. **Early boot**: Before `init_from_handoff` is called, any physical memory consumption (page tables, stacks, temporary structures) must be recorded via `record_boot_consumed_region`.

2. **Handoff initialization**: `init_from_handoff` is called with:
   - The UEFI memory map (conventional memory marked as free, all other types as reserved)
   - A list of additional consumed regions (e.g., framebuffer, ACPI tables)
   - A callback to allocate bitmap storage from the permanent heap

3. **Region reservation**: During initialization:
   - All UEFI-reserved regions are marked as reserved in the bitmap
   - Boot-consumed regions (from `BOOT_CONSUMED` log) are drained and reserved
   - Explicitly provided consumed regions are reserved
   - Debug builds validate no overlap between consumed regions and free frames

4. **Post-initialization**: After `init_from_handoff` succeeds:
   - `record_boot_consumed_region` directly reserves regions in the live bitmap
   - The `BOOT_CONSUMED` log is empty and unused

**Critical invariant**: If a region is missed during handoff (not recorded via `record_boot_consumed_region` and not in the consumed list), it may be allocated later, causing memory corruption or aliasing.

Implements / evidence:
- `kernel/src/physical_memory.rs#record_boot_consumed_region`
- `kernel/src/physical_memory.rs#init_from_handoff`
- `kernel/src/physical_memory.rs#reserve_boot_consumed`
- `kernel/src/physical_memory.rs#validate_no_overlap_with_free_frames` (debug)

Related plans:
- `../plans/memory.md#physical-memory-allocator`

Affected modules:
- `kernel/src/physical_memory.rs`
- `kernel/src/environment.rs`
- All early-boot code that consumes physical memory before the allocator is ready
