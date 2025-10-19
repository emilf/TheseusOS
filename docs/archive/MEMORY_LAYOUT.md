# TheseusOS Memory Layout Documentation

This document provides a comprehensive overview of how TheseusOS organizes and manages memory, including both physical and virtual memory layouts, allocation strategies, and the location of critical kernel structures.

## Table of Contents

1. [Overview](#overview)
2. [Physical Memory Layout](#physical-memory-layout)
3. [Virtual Memory Layout](#virtual-memory-layout)
4. [Memory Allocation Strategy](#memory-allocation-strategy)
5. [Critical Kernel Structures](#critical-kernel-structures)
6. [Memory Mapping Details](#memory-mapping-details)
7. [Frame Allocation](#frame-allocation)
8. [Memory Safety Features](#memory-safety-features)

## Overview

TheseusOS uses a sophisticated memory management system that combines:
- **High-half kernel**: Kernel runs in high virtual addresses (0xFFFFFFFF80000000+)
- **Identity mapping**: Early boot uses 1:1 physical-to-virtual mapping
- **PHYS_OFFSET mapping**: Linear mapping of physical memory at 0xFFFF800000000000
- **Reserved frame pool**: Critical structures get guaranteed frame allocation
- **Temporary mapping window**: Safe access to arbitrary physical frames

## Physical Memory Layout

### Memory Regions (Typical QEMU Setup)

```
Physical Address Range    | Size      | Purpose
--------------------------|-----------|----------------------------------
0x0000000000000000        | 4KB       | Reserved (avoided by allocator)
0x0000000000001000        | 255KB     | Low memory (identity mapped)
0x0000000000040000        | 60MB      | Kernel image, bootloader data
0x00000000003E129000      | 348KB     | Kernel code/data segments
0x00000000003E180000      | 1MB       | Temporary heap (bootloader)
0x00000000003F000000      | 16MB      | UEFI runtime, ACPI tables
0x00000000080000000       | 64MB      | Framebuffer (GOP)
0x000000000FEE00000       | 4KB       | LAPIC MMIO registers
0x0000000010000000        | 3GB       | Available system RAM
```

### UEFI Memory Map Integration

The kernel builds its frame allocator from the UEFI memory map, which provides:
- **Conventional Memory**: Available for allocation (type 7)
- **Reserved Memory**: BIOS, ACPI, MMIO regions (other types)
- **Memory Holes**: Non-contiguous memory regions

**Example Memory Map Statistics**:
```
Total pages in memory map:     0x000503A4 (327,588 pages = ~1.25GB)
Conventional pages available: 0x0003D31E (250,686 pages = ~979MB)
```

## Virtual Memory Layout

### High-Half Kernel Design

TheseusOS uses a high-half kernel design where the kernel operates in the upper virtual address space:

```
Virtual Address Range      | Physical Mapping      | Purpose
---------------------------|----------------------|----------------------------------
0xFFFFFFFF80000000        | 0x000000003E129000   | Kernel code/data segments
0xFFFFFFFF80010000        | 0x000000003E180000   | Temporary heap
0xFFFFFFFFB0000000        | 0x000000003E190000   | Kernel heap (permanent)
0xFFFFFFFF80030000        | 0x000000003E1A0000   | IST stacks (DF, NMI, MC, PF)
0xFFFFFFFF80040000        | 0x000000003E1B0000   | Kernel stack (64KB)
0xFFFFFFFFA0000000        | 0x000000003E180000   | Temporary heap (alternative mapping)
0xFFFF800000000000        | 0x0000000000000000   | PHYS_OFFSET linear mapping
0xFFFF800000000000        | 0x0000000000001000   | PHYS_OFFSET (first 1GB mapped)
0xFFFF800000000000        | 0x0000000004000000   | PHYS_OFFSET (up to 1GB)
0xFFFF8000FEE00000        | 0x000000000FEE00000  | LAPIC MMIO mapping
```

### Identity Mapping (Early Boot)

During early boot, the first 1GB of physical memory is identity-mapped:

```
Virtual Address Range      | Physical Address Range | Purpose
---------------------------|----------------------|----------------------------------
0x0000000000000000        | 0x0000000000000000   | Identity mapping (1GB)
0x0000000000001000        | 0x0000000000001000   | Available for early allocation
0x0000000000200000        | 0x0000000000200000   | 2MB page mappings
0x0000000000400000        | 0x0000000000400000   | 2MB page mappings
0x000000003FC00000        | 0x000000003FC00000   | UEFI boot services
```

## Memory Allocation Strategy

### Frame Allocation Hierarchy

1. **Reserved Frame Pool** (16 frames)
   - **Purpose**: Critical kernel structures (page tables, IST stacks)
   - **Allocation**: LIFO (Last In, First Out)
   - **Guarantee**: Always available, never starved

2. **General Frame Pool**
   - **Purpose**: Non-critical allocations
   - **Source**: UEFI conventional memory regions
   - **Behavior**: Can be exhausted by ephemeral allocations

### Allocation Priority

```rust
// Page table allocation prefers reserved frames
let frame = allocator.allocate_reserved_frame()  // Try reserved first
    .or_else(|| allocator.allocate_frame())      // Fall back to general
    .expect("No frames available");
```

### Memory Map Processing

The frame allocator processes UEFI memory map descriptors:

```rust
for descriptor in memory_map {
    if descriptor.type == UEFI_CONVENTIONAL_MEMORY {
        // Skip physical frame 0
        let start = max(descriptor.physical_start, 0x1000);
        // Align to page boundary
        let aligned_start = start & !0xFFF;
        // Add to available frames
        available_frames.push_range(aligned_start, descriptor.num_pages);
    }
}
```

## Critical Kernel Structures

### Page Tables

**PML4 (Page Map Level 4)**:
- **Location**: Dynamically allocated frame
- **Purpose**: Root of page table hierarchy
- **Mapping**: Index 0 = Identity, Index 510 = High-half kernel

**PDPT/PD/PT (Lower Level Tables)**:
- **Allocation**: Reserved frame pool (preferred) or general pool
- **Zeroing**: Via `TemporaryWindow` when PHYS_OFFSET active, identity when not

### Interrupt Stack Table (IST)

**Stack Locations** (High-half virtual addresses):
```
IST Index | Virtual Address        | Purpose
----------|----------------------|----------------------------------
0 (DF)    | 0xFFFFFFFF80030000   | Double Fault handler stack
1 (NMI)   | 0xFFFFFFFF80031000   | Non-Maskable Interrupt stack  
2 (MC)    | 0xFFFFFFFF80032000   | Machine Check handler stack
3 (PF)    | 0xFFFFFFFF80033000   | Page Fault handler stack
```

**Physical Mapping**: Each IST stack is 4KB and mapped to physical frames via the reserved pool.

### Kernel Stack

**Virtual Address**: `0xFFFFFFFF80040000`
**Size**: 64KB (16 pages)
**Physical Mapping**: Reserved frame pool
**Alignment**: 16-byte aligned for x86-64 ABI compliance

### Global Descriptor Table (GDT)

**Location**: Statically allocated in kernel data segment
**Contents**:
- Null descriptor (index 0)
- Kernel code segment (index 1)
- Kernel data segment (index 2)
- Task State Segment (index 3)

### Task State Segment (TSS)

**Location**: Static allocation in kernel
**Purpose**: Contains IST stack pointers and CPU state
**Size**: 104 bytes (standard x86-64 TSS)

## Memory Mapping Details

### High-Half Kernel Mapping

The kernel image is mapped into high-half using 4KB pages:

```rust
// Kernel virtual base: 0xFFFFFFFF80000000
// Kernel physical base: 0x000000003E129000
let kernel_size = handoff.kernel_image_size;  // ~348KB
let num_pages = (kernel_size + 0xFFF) >> 12;  // Round up to pages

for page in 0..num_pages {
    let virt_addr = KERNEL_VIRTUAL_BASE + (page << 12);
    let phys_addr = kernel_phys_base + (page << 12);
    map_page_4k(pml4, virt_addr, phys_addr, PTE_PRESENT | PTE_WRITABLE);
}
```

### PHYS_OFFSET Linear Mapping

The first 1GB of physical memory is linearly mapped at `PHYS_OFFSET`:

```rust
// PHYS_OFFSET: 0xFFFF800000000000
// Maps: [0x0000000000000000, 0x0000000040000000) -> [0xFFFF800000000000, 0xFFFF800040000000)
for page in 0..(1 << 20) {  // 1GB = 2^20 pages
    let virt_addr = PHYS_OFFSET + (page << 12);
    let phys_addr = page << 12;
    map_page_2mb(pml4, virt_addr, phys_addr, PTE_PRESENT | PTE_WRITABLE);
}
```

### Temporary Mapping Window

A fixed virtual address is used for temporary physical frame access:

```rust
const TEMP_WINDOW_VA: u64 = 0xFFFF_FFFE_0000_0000;

// Map arbitrary physical frame to temporary window
temporary_window.map_phys_frame(physical_frame_addr, &mut frame_allocator);
// Access via virtual address
let data = unsafe { *(TEMP_WINDOW_VA as *const u64) };
// Unmap when done
temporary_window.unmap();
```

## Frame Allocation

### BootFrameAllocator Structure

```rust
pub struct BootFrameAllocator {
    // UEFI memory map access
    base_ptr: *const u8,           // Pointer to memory map buffer
    desc_size: usize,              // Size of each descriptor
    count: usize,                  // Number of descriptors
    
    // Current region tracking
    cur_index: usize,              // Current descriptor index
    cur_next_addr: u64,            // Next available address
    cur_remaining_pages: u64,      // Pages remaining in current region
    
    // Reserved pool for critical structures
    reserved: [u64; 16],           // Physical addresses of reserved frames
    reserved_count: usize,         // Number of frames in reserved pool
}
```

### Frame Allocation Process

1. **Reservation Phase** (Early in `MemoryManager::new()`):
   ```rust
   let mut allocator = BootFrameAllocator::from_handoff(handoff);
   let reserved_count = allocator.reserve_frames(16);  // Reserve 16 frames
   ```

2. **Critical Allocation** (Page tables, IST stacks):
   ```rust
   let frame = allocator.allocate_reserved_frame()  // Try reserved first
       .or_else(|| allocator.allocate_frame());     // Fall back to general
   ```

3. **General Allocation** (Non-critical structures):
   ```rust
   let frame = allocator.allocate_frame();  // From general pool
   ```

### Persistent Physical Memory Manager

After the permanent heap is mapped and the global allocator switches away from UEFI services, Theseus hands frame ownership to a bitmap-backed manager that survives for the life of the kernel.

#### Bootstrap Sequence

1. **Collect boot-time usage** – every frame consumed while the boot allocator was live is recorded via `record_boot_consumed_region`. When the persistent manager is about to come online we drain that log so those PFNs are immediately marked in-use.
2. **Allocate bitmap backing** – a closure passed to `init_from_handoff` grabs a contiguous run of frames (with the still-active `BootFrameAllocator`), zeros them through `PHYS_OFFSET`, and records them as consumed before the boot allocator disappears.
3. **Build the bitmap allocator** – `PhysicalMemoryManager::new` computes the lowest usable PFN (dropping a 64 KiB guard under the earliest runtime symbol), seeds the bitmap, and reserves every region the UEFI map marked non-conventional alongside the boot-consumed ranges.

#### API Surface

```rust
// After permanent heap is ready:
let consumed = physical_memory::drain_boot_consumed();
physical_memory::init_from_handoff(handoff, &consumed, bitmap_provider)?;

// Runtime allocations:
let pa = physical_memory::alloc_frame()?;
physical_memory::free_frame(pa)?;

// Paging helpers can borrow a FrameAllocator implementation:
let mut persistent = physical_memory::PersistentFrameAllocator;
mapper.map_to(page, frame, flags, &mut persistent)?;
```

`physical_memory::dump_state()` produces a quick summary (base PFN, total and free frame counts) that can be wired into the monitor.

#### Guarded Lower PFN

The runtime base finder measures the physical addresses of `kernel_entry`, the high-half trampoline, stack-switch helpers, and other early routines, then subtracts a 64 KiB guard window before initialising the bitmap. This keeps helper code that was linked just below the 2 MiB-aligned handoff base mapped and prevents it from being recycled accidentally.

### Frame Zeroing Strategy

Frames are zeroed using the appropriate method based on paging state:

```rust
unsafe fn zero_frame_safely(pa: u64) {
    if phys_offset_is_active() {
        // Use PHYS_OFFSET mapping for virtual access
        zero_phys_range(pa, PAGE_SIZE);
    } else {
        // Early boot: direct physical write (identity mapped)
        core::ptr::write_bytes(pa as *mut u8, 0, PAGE_SIZE);
    }
}

## Physical Memory Manager Summary

- Boot allocations are tracked via the consumed-region log; draining that list before initialisation ensures the persistent bitmap never hands out frames that were used to reach the high half.
- A dedicated bitmap backing store is allocated with the boot allocator, zeroed, and immediately added to the consumed set so the persistent manager owns it without touching the new heap.
- `PersistentFrameAllocator` exposes the `FrameAllocator<Size4KiB>` trait, letting existing paging helpers request frames from the long-lived pool with no code changes.
- Direct helpers (`alloc_frame`, `free_frame`, `dump_state`) provide PFN-level access for subsystems that want to work with raw physical addresses.
- The 64 KiB lower guard keeps the helper routines that sit just below the handoff-aligned kernel base mapped and out of the free list.
```

## Memory Safety Features

### Reserved Frame Pool

**Purpose**: Prevent critical structure starvation
**Implementation**: 16-frame LIFO pool
**Usage**: Page tables, IST stacks, kernel stack
**Fallback**: General pool if reserved exhausted

### Temporary Mapping Window

**Purpose**: Safe access to arbitrary physical frames
**Location**: Fixed virtual address (0xFFFF_FFFE_0000_0000)
**Safety**: Automatic unmapping, no identity dependency
**Usage**: Zeroing page tables, accessing MMIO

### PHYS_OFFSET Integration

**Purpose**: Linear physical memory access
**Location**: 0xFFFF800000000000
**Coverage**: First 1GB of physical memory
**Safety**: Volatile writes, proper alignment

### Memory Map Validation

**Diagnostics**: Total and conventional page counts
**Verification**: Confirms allocator sees full UEFI memory map
**Debugging**: Helps identify memory allocation issues

## Memory Layout Summary

### Virtual Address Space Organization

```
0x0000000000000000 - 0x000000003FFFFFFF  Identity mapping (1GB)
0xFFFFFFFF80000000 - 0xFFFFFFFF8003FFFF  Kernel image (256KB)
0xFFFFFFFF80010000 - 0xFFFFFFFF8001FFFF  Temporary heap (64KB)
0xFFFFFFFFB0000000 - 0xFFFFFFFFB00FFFFF  Permanent heap (1MB)
0xFFFFFFFF80030000 - 0xFFFFFFFF80033FFF  IST stacks (16KB)
0xFFFFFFFF80040000 - 0xFFFFFFFF8004FFFF  Kernel stack (64KB)
0xFFFFFFFFA0000000 - 0xFFFFFFFFA00FFFFF  Temp heap alt mapping (1MB)
0xFFFF800000000000 - 0xFFFF80003FFFFFFF  PHYS_OFFSET (1GB)
0xFFFF8000FEE00000 - 0xFFFF8000FEE00FFF  LAPIC MMIO (4KB)
0xFFFF_FFFE_0000_0000                     Temporary window (4KB)
```

### Physical Memory Organization

```
0x0000000000000000 - 0x0000000000000FFF  Reserved (frame 0)
0x0000000000001000 - 0x000000003FFFFFFF  Available system RAM
0x000000003E129000 - 0x000000003E1AFFFF  Kernel image + structures
0x000000003E180000 - 0x000000003E27FFFF  Temporary heap
0x000000003F000000 - 0x000000003FFFFFFF  UEFI runtime
0x0000000080000000 - 0x0000000083E7FFFF  Framebuffer
0x00000000FEE00000 - 0x00000000FEE00FFF  LAPIC MMIO
0x0000000100000000 - 0x0000000FFFFFFFFF  Extended system RAM
```

## Physical Memory Manager

The kernel now owns a persistent physical memory manager that tracks all frames
after boot. Key points:

- Built from the UEFI memory map embedded in the handoff structure.
- Maintains a bitmap over 4 KiB frames stored in the permanent kernel heap.
- Marks frames consumed during boot (page tables, heap backing, stacks) so they
  are not reused.
- Exposes `alloc_frame()` / `free_frame()` for subsystems and offers a
  `dump_state()` helper visible via the kernel monitor for diagnostics.

Future extensions can add contiguous allocation support, zone tagging, and
NUMA-awareness, but the current implementation already provides safe frame reuse
post-boot.
