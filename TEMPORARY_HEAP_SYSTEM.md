# Temporary Heap System for Kernel Setup

## What is a "Heap"?

In programming, a **heap** is a region of memory where programs can dynamically allocate and free memory as needed. Think of it like a storage room where you can put things and take them out as needed.

## Why Do We Need a Temporary Heap?

When the kernel starts up, it needs memory to work with, but it hasn't set up its own memory management system yet. It's like trying to organize a room when you don't have any shelves or storage containers - you need some basic storage first!

The temporary heap system solves this by:
- Giving the kernel a "starter" memory region to use
- Avoiding complex memory setup during early kernel initialization
- Providing a safe, pre-allocated memory area

## How It Works

### Bootloader Side (The Memory Allocator)
- **What it does**: Asks UEFI for 1MB of safe memory to give to the kernel
- **How it works**: Uses UEFI's memory allocation services (like asking the system for memory)
- **What it stores**: Saves the memory address and size in the handoff structure
- **Safety**: If it can't get memory, it sets the fields to 0 (no memory available)

### Kernel Side (The Memory User)
- **What it does**: Checks if the bootloader left it any memory to use
- **How it works**: Sets up a simple "bump allocator" (just moves a pointer forward as it allocates)
- **Fallback**: If no memory was provided, uses a fixed 1MB region at a known address
- **Result**: The kernel can now allocate memory during its startup phase

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

## Usage Example

```rust
// Bootloader allocates heap
match memory::allocate_memory(1024 * 1024, MemoryType::LOADER_DATA) {
    Ok(region) => {
        handoff.temp_heap_base = region.physical_address;
        handoff.temp_heap_size = region.size;
    }
    Err(_) => {
        handoff.temp_heap_base = 0;
        handoff.temp_heap_size = 0;
    }
}

// Kernel uses heap
if handoff.temp_heap_base != 0 && handoff.temp_heap_size != 0 {
    // Use pre-allocated heap
    HEAP_START = handoff.temp_heap_base as *mut u8;
    HEAP_END = (handoff.temp_heap_base + handoff.temp_heap_size) as *mut u8;
    HEAP_NEXT = handoff.temp_heap_base as *mut u8;
} else {
    // Use fallback heap
    init_heap_fallback();
}
```

## Testing

The system includes comprehensive testing:
- **Basic Allocation**: String allocation using `alloc::format!`
- **Vector Allocation**: Dynamic array allocation using `alloc::vec!`
- **Integration Test**: Full bootloader-to-kernel handoff verification

## Current Status

✅ **Working Perfectly**: The system is fully functional with:
- Bootloader successfully allocates 1MB of memory for the kernel
- Kernel can use this memory immediately when it starts
- Simple but effective memory allocation system
- Clean startup and shutdown process

## What's Next

- **Configurable Size**: Allow the heap size to be adjusted based on system needs
- **Multiple Heaps**: Support for having several memory regions if needed
- **Smart Memory Selection**: Automatically choose the best type of memory for the heap
- **Usage Tracking**: Monitor how much memory is being used during startup
