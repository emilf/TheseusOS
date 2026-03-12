# Memory Axioms

Binding architectural truths for paging, heaps, and physical-memory management.

## A1: The kernel executes from a higher-half virtual base

**REQUIRED**

The kernel’s linked and runtime higher-half base is `0xFFFF_FFFF_8000_0000`.

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
