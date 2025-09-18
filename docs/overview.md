# Overview

This project provides a comprehensive Rust UEFI bootloader that:
1. Initializes UEFI services and logging
2. Prints to screen and serial with beautiful formatting
3. Collects comprehensive environment information (GOP, memory map, ACPI, firmware, boot info, CPU)
4. Retrieves and preserves the UEFI memory map with proper key management
5. Prepares for kernel handoff with complete system information
6. Passes a comprehensive `Handoff` struct pointer in RDI for the kernel
7. Halts (placeholder until chaining to a kernel)

## Boot Flow
- UEFI loads `EFI/BOOT/BOOTX64.EFI` from the ESP.
- Entry `efi_main` runs with `SystemTable<Boot>`.
- We initialize serial communication and logging.
- We collect Graphics Output Protocol (GOP) framebuffer information.
- We retrieve the complete UEFI memory map and preserve it for kernel access.
- We attempt to locate ACPI RSDP table (placeholder for UEFI-RS 0.35 API).
- We collect firmware, boot time, device path, and CPU information.
- We finalize the comprehensive `Handoff` structure with all collected data.
- We prepare the memory map for kernel handoff (with memory map key).
- We halt the CPU (ready for kernel chaining).

## Handoff ABI
C-compatible layout (`#[repr(C)]`) - **168 bytes total**:

```
struct Handoff {
  // Header
  size: u32,                    // Total size of this struct (168 bytes)
  handoff_version: u32,         // Handoff format version (1)
  
  // Graphics Output Protocol (GOP) Information
  gop_fb_base: u64,            // Framebuffer base address
  gop_fb_size: u64,            // Framebuffer size in bytes
  gop_width: u32,              // Framebuffer width in pixels
  gop_height: u32,             // Framebuffer height in pixels
  gop_stride: u32,             // Framebuffer stride
  gop_pixel_format: u32,       // Pixel format enum (0=Rgb, 1=Bgr, 2=Bitmask, 3=BltOnly)
  
  // Memory Map Information
  memory_map_buffer_ptr: u64,   // Memory map buffer pointer (managed by UEFI-RS)
  memory_map_descriptor_size: u32,    // Descriptor size in bytes (48)
  memory_map_descriptor_version: u32, // Descriptor version (1)
  memory_map_entries: u32,      // Number of memory map entries
  memory_map_size: u32,         // Total memory map buffer size
  
  // ACPI Information
  acpi_rsdp: u64,              // ACPI RSDP table address (0 if not found)
  
  
  // Firmware Information
  firmware_vendor_ptr: u64,    // Firmware vendor string pointer (0 if not available)
  firmware_vendor_len: u32,    // Firmware vendor string length
  firmware_revision: u32,      // Firmware revision
  
  // Boot Time Information
  boot_time_seconds: u64,      // Boot time (seconds since epoch)
  boot_time_nanoseconds: u32,  // Boot time (nanoseconds)
  boot_device_path_ptr: u64,   // Boot device path pointer (0 if not available)
  boot_device_path_size: u32,  // Boot device path size in bytes
  
  // CPU Information
  cpu_count: u32,              // CPU count
  cpu_features: u64,           // CPU features flags
  microcode_revision: u32,     // Microcode revision
  
  // Hardware Inventory Information
  hardware_device_count: u32,  // Number of hardware devices found
  hardware_inventory_ptr: u64, // Hardware inventory data pointer
  hardware_inventory_size: u64, // Hardware inventory data size
}
```

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
