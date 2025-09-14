# Temporary Heap System for Kernel Setup

## Overview

The temporary heap system provides the kernel with a pre-allocated memory region for use during its initialization phase, before proper page tables and memory management are established. This system eliminates the need for complex memory map parsing in the kernel while ensuring safe memory allocation.

## Architecture

### Bootloader Side
- **Allocation**: Bootloader allocates 1MB of safe memory using UEFI Boot Services
- **Storage**: Memory address and size stored in handoff structure
- **Type**: Uses `LOADER_DATA` memory type for safe allocation
- **Fallback**: If allocation fails, handoff fields are set to 0

### Kernel Side
- **Detection**: Kernel checks handoff structure for temporary heap information
- **Initialization**: Sets up bump allocator using pre-allocated memory
- **Fallback**: Uses fixed 1MB region at 0x100000 if no temporary heap available
- **Usage**: Provides working heap for kernel initialization without complex setup

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

## Benefits

### For Kernel
- **Immediate Availability**: Heap ready immediately upon kernel entry
- **No Complexity**: No need to parse memory maps or set up complex memory management
- **Safe Memory**: Guaranteed to use bootloader-allocated safe memory
- **Setup Focused**: Designed specifically for kernel initialization phase

### For System
- **Clean Separation**: Bootloader handles memory allocation, kernel consumes
- **Robust Fallback**: System works even if temporary heap allocation fails
- **Simple Interface**: Clean address/size fields in handoff structure
- **Future Proof**: Easy to extend or modify heap size as needed

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

## Status

✅ **Fully Functional**: System is working correctly with:
- Successful bootloader heap allocation (1MB at 0xE184000)
- Kernel heap initialization using pre-allocated memory
- Working bump allocator with proper alignment
- Clean kernel initialization and exit

## Future Enhancements

- **Dynamic Sizing**: Allow configurable heap size based on system needs
- **Multiple Regions**: Support for multiple heap regions if needed
- **Memory Type Detection**: Automatic selection of best memory type for heap
- **Statistics**: Track heap usage during kernel initialization
