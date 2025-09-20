# TheseusOS

A bare-metal operating system for x86-64 systems, consisting of a UEFI bootloader and kernel written in Rust. 
This is an experiment in "vibe coding", as the creator is not well versed in rust, but is using this as a learning experience.

TheseusOS demonstrates modern OS development practices with comprehensive system information collection, proper UEFI boot services management, and kernel environment setup.

## Documentation

This project features comprehensive documentation throughout the codebase:

- **Function Documentation**: All public functions include detailed documentation with parameter descriptions, return values, and safety requirements
- **Module Documentation**: Each module has a comprehensive overview explaining its purpose and functionality
- **Code Comments**: Inline comments explain complex logic and implementation details
- **API Documentation**: Generated documentation available via `cargo doc --open`

The codebase follows Rust documentation standards and includes examples where appropriate.

## Architecture

TheseusOS is composed of three main components:

1. **UEFI Bootloader**: Comprehensive system information collection and kernel loading
2. **Kernel**: Bare-metal kernel with environment setup and memory management foundations
3. **Shared Library**: Common data structures and constants

## Current Status

✅ **Working Features:**
- Complete UEFI bootloader with system information collection
- Real kernel loading from EFI filesystem with ELF parsing
- Proper UEFI boot services exit using uefi-rs 0.35.0
- Kernel environment setup (interrupts, GDT, CPU features)
- Temporary heap allocator for kernel initialization
- Comprehensive handoff structure for kernel-bootloader communication

🚧 **In Development:**
- Virtual memory management and page tables
- Device driver framework
- Process management and scheduling

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
1. **System Information Collection**: Memory map, ACPI, hardware inventory, graphics info
2. **Temporary Heap Allocation**: Allocates 1MB heap for kernel use during initialization
3. **Kernel Discovery**: Locates `kernel.efi` in EFI System Partition
4. **File Analysis**: Reads kernel binary and analyzes ELF structure
5. **Memory Allocation**: Finds free memory region for kernel loading
6. **Kernel Loading**: Copies kernel sections to allocated memory
7. **Boot Services Exit**: Calls `uefi::boot::exit_boot_services()` to exit UEFI environment
8. **Kernel Jump**: Transfers control to kernel entry point with handoff structure
9. **Kernel Environment Setup**: Kernel initializes heap, disables interrupts, sets up GDT and CPU features

## Handoff ABI (to your kernel)
- **Location**: Static storage inside the EFI image
- **Register**: RDI = pointer to `Handoff`
- **Size**: `Handoff.size` bytes (248 bytes total)
- **Boot Services Status**: `Handoff.boot_services_exited` indicates if boot services have been exited

### Comprehensive System Information:
- **Graphics**: GOP framebuffer base/size, resolution, stride, pixel format
- **Memory Map**: Complete memory layout with descriptor metadata and key
- **ACPI**: RSDP physical address (when available)
- **Firmware**: UEFI vendor info and revision
- **Boot Context**: Boot time and device path information
- **CPU**: Processor count, features, and microcode revision
- **Hardware Inventory**: Comprehensive device enumeration with handles and metadata
- **Kernel Information**: Physical and virtual addresses, entry points, memory layout
- **Temporary Heap**: Pre-allocated 1MB heap region for kernel initialization
- **UEFI Context**: System table and image handle for boot services management
- **Boot Services Status**: Indicates whether UEFI boot services have been exited

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

## Documentation Standards

This project follows comprehensive documentation standards:

### Function Documentation
Every public function includes:
- Brief description of what the function does
- `# Parameters` section listing all parameters with types and descriptions
- `# Returns` section describing the return value and possible error conditions
- `# Safety` section for unsafe functions explaining safety requirements
- `# Examples` section where appropriate

### Module Documentation
Each module includes:
- Module-level documentation explaining the module's purpose
- Overview of the module's functionality and responsibilities
- List of key types and functions provided
- Usage examples and integration notes

### Code Comments
- Inline comments explain complex logic and implementation details
- TODO comments mark areas for future improvement
- Safety comments explain why certain operations are safe
- Architecture comments explain design decisions

### Generated Documentation
Run `cargo doc --open` to generate and view the complete API documentation.

## Next Steps
- **Kernel Development**: Implement actual kernel functionality (memory management, process scheduling)
- **Virtual Memory**: Set up page tables and enable virtual memory addressing
- **Device Drivers**: Implement drivers for hardware discovered during boot
- **System Services**: Build on the comprehensive Handoff structure for full OS functionality
