# Temporary Heap System for Single-Binary Boot

## What is a "Heap"?

In programming, a **heap** is a region of memory where programs can dynamically allocate and free memory as needed. Think of it like a storage room where you can put things and take them out as needed.

## Why Do We Need a Temporary Heap?

In the single-binary architecture, we use UEFI's allocator during the bootloader phase. However, once we establish higher-half virtual memory mapping, we need a kernel-controlled heap region. The temporary heap bridges this transition:

- Gives the kernel a mapped memory region to use early on
- Avoids complex heap setup during initial higher-half transition
- Provides a safe, pre-allocated memory area that survives ExitBootServices
- Allows testing heap operations before permanent heap is ready

## How It Works in Single-Binary Boot

### Bootloader Phase (Pre-ExitBootServices)
- **Global Allocator**: Uses UEFI's `global_allocator` for Vec, String, etc.
- **Temp Heap Allocation**: Allocates 1MB using `allocate_memory_non_overlapping`
- **Overlap Avoidance**: Ensures temp heap doesn't overlap kernel image region
- **Retry Strategy**: Increments allocation size to perturb UEFI allocator if needed
- **Handoff Storage**: Stores physical base and size in handoff structure
- **Failure Handling**: If allocation fails, sets fields to 0 (kernel uses fallback)

### Kernel Phase (Post-ExitBootServices)
- **Initial State**: UEFI allocator still active (persists after ExitBootServices)
- **Higher-Half Mapping**: Temp heap is mapped to `TEMP_HEAP_VIRTUAL_BASE` (0xFFFFFFFFA0000000)
- **Early Use**: Can be used for allocations during higher-half transition
- **Permanent Heap**: After mapping KERNEL_HEAP_BASE, switches to permanent heap
- **Cleanup**: Temporary heap mapping is unmapped after permanent heap is active
- **Fallback**: If no temp heap provided, initializes permanent heap directly at KERNEL_HEAP_BASE

## Implementation Details

### Handoff Structure Fields

```rust
// Temporary Heap Information
pub temp_heap_base: u64,   // Physical address of pre-allocated heap
pub temp_heap_size: u64,   // Size of heap in bytes
```

### Bump Allocator

- **Strategy**: Simple bump allocation with proper alignment
- **Features**: No deallocation support (suitable for setup phase)
- **Alignment**: Handles arbitrary alignment requirements
- **Bounds Checking**: Prevents allocation beyond heap boundaries

### Memory Flow

1. **Bootloader**: Allocates 1MB using UEFI `allocate_pages()`
2. **Handoff**: Stores address and size in handoff structure
3. **Kernel**: Reads handoff structure on entry
4. **Initialization**: Sets up bump allocator with pre-allocated memory
5. **Usage**: Kernel can allocate memory immediately upon entry

## Why This Design is Good

### For the Kernel
- **Ready to Use**: As soon as the kernel starts, it has memory available
- **Simple**: No need to figure out complex memory management right away
- **Safe**: The memory was allocated by the bootloader, so it's guaranteed to be safe
- **Focused**: Designed just for the kernel's startup phase

### For the Overall System
- **Clear Responsibilities**: Bootloader handles memory setup, kernel uses it
- **Reliable**: Even if something goes wrong, the system has a backup plan
- **Easy to Understand**: Simple address and size fields in the handoff structure
- **Flexible**: Easy to change the heap size or add more features later

## Usage Example (Single-Binary)

```rust
// Bootloader phase: allocate non-overlapping temp heap
let img_start = HANDOFF.kernel_physical_base;
let img_end = img_start + HANDOFF.kernel_image_size;
match memory::allocate_memory_non_overlapping(
    1024 * 1024,
    MemoryType::LOADER_DATA,
    img_start,
    img_end,
) {
    Ok(region) => {
        HANDOFF.temp_heap_base = region.physical_address;
        HANDOFF.temp_heap_size = region.size;
    }
    Err(_) => {
        HANDOFF.temp_heap_base = 0;
        HANDOFF.temp_heap_size = 0;
    }
}

// Kernel phase: use temp heap or fallback to permanent heap
if handoff.temp_heap_size != 0 {
    // Map temp heap to high-half and use it
    ALLOCATOR_LINKED.lock().init(
        TEMP_HEAP_VIRTUAL_BASE as *mut u8,
        handoff.temp_heap_size as usize
    );
} else {
    // Initialize permanent heap directly
    ALLOCATOR_LINKED.lock().init(
        KERNEL_HEAP_BASE as *mut u8,
        KERNEL_HEAP_SIZE
    );
}
```

## Testing

The system includes comprehensive testing:
- **Basic Allocation**: String allocation using `alloc::format!`
- **Vector Allocation**: Dynamic array allocation using `alloc::vec!`
- **Integration Test**: Full bootloader-to-kernel handoff verification

## Current Status

✅ **Working**: Single-binary architecture with:
- UEFI allocator active during bootloader phase
- Non-overlapping temp heap allocation (when successful)
- Fallback to direct permanent heap init (when temp heap fails)
- Permanent heap at KERNEL_HEAP_BASE after higher-half transition
- Clean transition from UEFI allocator to kernel-managed heap

⚠️ **Known Limitation**: Temp heap allocation sometimes fails to find non-overlapping region due to conservative 16 MiB kernel image size estimate. Fallback to permanent heap works correctly.

## Future Improvements

- **Better Image Size Detection**: Use more accurate kernel image size instead of 16 MiB estimate
- **Allocator Statistics**: Track allocation patterns and usage
- **Multiple Heap Regions**: Support zone-based allocation strategies
