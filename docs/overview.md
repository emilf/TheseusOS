# Overview

This project provides a comprehensive Rust UEFI bootloader and kernel system that:

## Bootloader Features
1. Initializes UEFI services and logging
2. Prints to screen and serial with beautiful formatting
3. Collects comprehensive environment information (GOP, memory map, ACPI, firmware, boot info, CPU)
4. Retrieves and preserves the UEFI memory map with proper key management
5. Loads and parses kernel ELF binaries from the EFI file system
6. Prepares for kernel handoff with complete system information
7. Passes a comprehensive `Handoff` struct pointer in RDI for the kernel

## Kernel Features
1. Initializes heap from bootloader-provided memory
2. Sets up virtual memory with identity and high-half mappings
3. Configures CPU features and control registers
4. Sets up interrupt handling and exception management
5. Provides comprehensive system information access
6. Implements proper kernel environment setup

## Documentation
The entire codebase features comprehensive documentation following Rust standards:
- All public functions include detailed parameter and return value documentation
- Module-level documentation explains purpose and functionality
- Safety requirements are clearly documented for unsafe functions
- Examples and usage patterns are provided where appropriate

## How TheseusOS Starts Up

Here's what happens when you run TheseusOS, step by step:

### 1. **UEFI Phase** (The Motherboard Helps Out)
- The computer's UEFI firmware (like a mini-operating system) loads our bootloader
- Our bootloader starts running and can use UEFI's services
- We set up communication so we can display information

### 2. **Information Gathering** (Learning About the Computer)
- **Graphics**: Find out what kind of display is available
- **Memory**: Map out all the memory in the computer
- **Hardware**: Discover what devices are connected
- **CPU**: Learn about the processor's capabilities
- **ACPI**: Get power management and hardware configuration info

### 3. **Kernel Loading** (Getting Ready to Take Over)
- Load the actual operating system (kernel) from disk
- Set up memory for the kernel to use
- Prepare all the information the kernel will need

### 4. **Handoff** (Passing Control)
- Exit UEFI mode (we no longer need the motherboard's help)
- Jump to the kernel and give it all the information we collected
- The kernel takes over and starts running the operating system

### 5. **Kernel Phase** (The Real Operating System)
- Set up memory management
- Configure the CPU for optimal performance
- Set up interrupt handling
- Initialize the system for normal operation

## The Handoff Structure (The Message Between Bootloader and Kernel)

The "handoff structure" is like a detailed report that the bootloader writes for the kernel. It contains all the information the kernel needs to know about the computer.

### What's in the Handoff Structure?

Think of it like a comprehensive system report with these sections:

- **System Info**: How big the structure is, what version it is
- **Graphics**: Information about the display (resolution, color format, etc.)
- **Memory**: Complete map of all the memory in the computer
- **Hardware**: List of all devices connected to the computer
- **CPU**: Information about the processor and its capabilities
- **ACPI**: Power management and advanced hardware configuration
- **Boot Info**: When the system started, what device it booted from
- **Kernel Info**: Where the kernel is loaded in memory

### Why This Design?

- **Complete Information**: The kernel gets everything it needs in one place
- **Efficient**: No need to re-discover hardware after bootloader finishes
- **Reliable**: All information is collected while UEFI services are available
- **Educational**: You can see exactly what information an OS needs to start up

### Handoff Structure Details:
- **Register**: `RDI` = pointer to `Handoff`
- **Size**: `Handoff.size` in bytes (168)
- **Graphics**: `gop_*` describe the framebuffer configuration
- **Memory Map**: `memory_map_*` provides complete memory layout information
- **ACPI**: `acpi_rsdp` provides ACPI table access (when available)
- **Firmware**: `firmware_*` provides UEFI firmware information
- **Boot Info**: `boot_time_*` and `boot_device_path_*` provide boot context
- **CPU**: `cpu_*` provides processor information and capabilities
- **Hardware Inventory**: `hardware_*` provides comprehensive device enumeration

## Hardware Inventory
The bootloader provides comprehensive hardware device enumeration:

- **Device Discovery**: Uses `uefi::boot::locate_handle_buffer()` to find all UEFI handles
- **Protocol Enumeration**: Enumerates devices by protocol type (LoadedImage, DevicePath, etc.)
- **Device Classification**: Categorizes devices by type (PCI, USB, Storage, Network, etc.)
- **Handle Tracking**: Stores device handles and metadata for kernel access
- **Memory Layout**: Hardware inventory data is stored in the `Handoff` structure
- **Beautiful Display**: Provides formatted output showing device counts and types

## Memory Management
The bootloader implements proper UEFI memory management:

- **Memory Map Collection**: Uses `uefi::boot::memory_map()` to retrieve the complete system memory layout
- **Memory Map Key**: Preserves the memory map key (`MemoryMapKey`) for proper kernel handoff
- **Buffer Management**: The `MemoryMapOwned` structure manages the memory map buffer internally
- **Kernel Access**: Memory map metadata is stored in the `Handoff` structure for kernel initialization
- **Boot Services**: Prepares for `exit_boot_services` with proper memory map handling

## Debugging
- `startQemu.sh` runs QEMU with `-s -S` and a monitor (`telnet:127.0.0.1:55555`).
- Use `gdb -ex 'target remote :1234'` to connect and breakpoints as needed.
- Inspect `RDI` after exit to read the comprehensive `Handoff` struct.
- Serial output provides detailed system information collection status.

## Extending to a Kernel
- **Kernel Loading**: Add kernel loading from ESP and jump to entry; preserve RDI.
- **Memory Management**: Initialize paging and memory manager using the complete memory map information.
- **System Information**: Use the comprehensive `Handoff` structure for complete system initialization.
- **Boot Services**: Implement proper `exit_boot_services` call with memory map key.
- **Runtime Transition**: Hand off control quickly; minimize time spent in EFI runtime.
