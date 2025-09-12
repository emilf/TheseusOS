# TheseusOS UEFI Bootloader (Rust)

A comprehensive UEFI application in Rust that provides a complete system information collection and kernel loading infrastructure. Features beautiful serial output formatting, comprehensive system information gathering, proper memory management, real kernel loading from EFI file system, and a detailed C-compatible Handoff struct for kernel consumption.

## Prerequisites
- Rust (stable) with target `x86_64-unknown-uefi`
- QEMU and OVMF firmware
- Linux host

Install target:
```bash
rustup target add x86_64-unknown-uefi
```

Populate firmware in the project (used by `startQemu.sh`):
```bash
mkdir -p OVMF
cp /usr/share/edk2-ovmf/x64/OVMF_CODE.fd OVMF/
cp /usr/share/edk2-ovmf/x64/OVMF_VARS.fd OVMF/
```

## Build and Run
```bash
make all
./startQemu.sh
```

### Build System Features:
- **Multi-crate workspace**: Bootloader, kernel, and shared libraries
- **Automated BIOS setup**: Automatically copies and configures OVMF firmware
- **GPT disk image creation**: Creates proper EFI System Partition with GPT partition table
- **Kernel integration**: Builds both bootloader and kernel binaries

### ESP Layout:
- `build/EFI/BOOT/BOOTX64.EFI` - Main bootloader
- `build/kernel.efi` - Kernel binary
- `build/disk.img` - Complete GPT disk image for QEMU

### Runtime Behavior:
- Serial output is redirected to the terminal; video output appears in the VM window
- Collects comprehensive system information (memory map, ACPI, hardware inventory)
- Loads kernel binary from EFI file system using proper UEFI FileInfo API
- Analyzes kernel ELF structure and allocates memory appropriately
- Exits boot services and jumps to kernel entry point
- Places `&Handoff` structure in RDI register for kernel access

## Debugging
`startQemu.sh` enables:
- GDB stub on `localhost:1234` (`-s -S`)
- QEMU monitor on `telnet:127.0.0.1:55555`

To debug with GDB:
```bash
gdb -ex 'target remote :1234'
```

To open QEMU monitor:
```bash
telnet 127.0.0.1 55555
```

## Kernel Loading Features

### UEFI File System Integration:
- **Real file system access**: Uses UEFI SimpleFileSystem protocol to read kernel binary
- **Proper file size detection**: Implements UEFI FileInfo API for accurate file size retrieval
- **ELF binary analysis**: Parses kernel ELF structure to extract section information
- **Memory allocation**: Finds suitable free memory regions using UEFI memory map
- **Section loading**: Loads kernel sections (.text, .rodata, .data, .bss) into allocated memory

### Boot Sequence:
1. **System Information Collection**: Memory map, ACPI, hardware inventory
2. **Kernel Discovery**: Locates `kernel.efi` in EFI System Partition
3. **File Analysis**: Reads kernel binary and analyzes ELF structure
4. **Memory Allocation**: Finds free memory region for kernel loading
5. **Kernel Loading**: Copies kernel sections to allocated memory
6. **Boot Services Exit**: Exits UEFI boot services with proper memory map
7. **Kernel Jump**: Transfers control to kernel entry point

## Handoff ABI (to your kernel)
- **Location**: Static storage inside the EFI image
- **Register**: RDI = pointer to `Handoff`
- **Size**: `Handoff.size` bytes (208 bytes total)

### Comprehensive System Information:
- **Graphics**: GOP framebuffer base/size, resolution, stride, pixel format
- **Memory Map**: Complete memory layout with descriptor metadata and key
- **ACPI**: RSDP physical address (when available)
- **Device Tree**: DTB pointer and size (ARM systems)
- **Firmware**: UEFI vendor info and revision
- **Boot Context**: Boot time and device path information
- **CPU**: Processor count, features, and microcode revision
- **Hardware Inventory**: Comprehensive device enumeration with handles and metadata
- **Kernel Information**: Physical and virtual addresses, entry points, memory layout

## Technical Implementation

### Key Components:
- **`kernel_loader.rs`**: Complete kernel loading implementation with UEFI FileInfo API
- **`boot_sequence.rs`**: Orchestrates system information collection and kernel handoff
- **`shared/`**: Common data structures and constants shared between bootloader and kernel
- **Multi-crate workspace**: Organized codebase with proper separation of concerns

### UEFI-RS 0.35.0 Compliance:
- Uses latest uefi-rs API with proper error handling
- Implements correct FileInfo buffer management
- Follows UEFI specification for file system access
- Proper memory map handling and boot services exit

## Next Steps
- **Kernel Development**: Implement actual kernel functionality (memory management, process scheduling)
- **Virtual Memory**: Set up page tables and enable virtual memory addressing
- **Device Drivers**: Implement drivers for hardware discovered during boot
- **System Services**: Build on the comprehensive Handoff structure for full OS functionality
