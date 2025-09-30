# TheseusOS Single-Binary Workspace Setup

## What is This?

This document explains how TheseusOS is organized as a **single-binary workspace** using Rust's workspace and crate system. While we have multiple crates (bootloader, kernel library, shared library), they all link into a single executable binary.

## Why Single-Binary?

TheseusOS uses a unified architecture where the bootloader and kernel are part of the same binary:

1. **Bootloader Crate**: UEFI application that collects system info and exits boot services
2. **Kernel Library**: Provides `kernel_entry` function called by bootloader
3. **Shared Library**: Contains common data structures used by both

This setup provides:
- Simpler deployment (one EFI file instead of two)
- No ELF loading complexity
- Direct function calls instead of assembly jumps
- Unified debugging with all symbols in one binary
- UEFI allocator available throughout boot
- Eliminates relocation and optimization issues from separate binaries

## Project Structure

```
TheseusOS/
├── Cargo.toml                 # Workspace root with shared dependencies
├── .cargo/
│   └── config.toml           # Kernel target configuration
├── linker.ld                  # Kernel linker script (for tests)
├── Makefile                  # Build system 
├── startQemu.sh              # QEMU execution script
├── bootloader/
│   ├── Cargo.toml            # Depends on kernel library
│   └── src/
│       ├── main.rs           # UEFI entry (efi_main), calls kernel_entry
│       ├── boot_sequence.rs  # System info collection, kernel handoff
│       ├── memory.rs         # UEFI memory allocation helpers
│       ├── acpi.rs           # ACPI table discovery
│       ├── hardware.rs       # Hardware inventory collection
│       ├── system_info.rs    # Firmware, CPU, boot time collection
│       └── drivers/          # Output drivers (UEFI serial, QEMU debug)
├── kernel/
│   ├── Cargo.toml            # Library crate (no bin target)
│   └── src/
│       ├── lib.rs            # Exports kernel_entry function
│       ├── environment.rs    # Complete kernel initialization
│       ├── memory.rs         # Virtual memory and page tables
│       ├── allocator.rs      # Heap management
│       ├── interrupts.rs     # IDT, exceptions, LAPIC timer
│       ├── gdt.rs            # GDT, TSS, segment setup
│       └── ...               # Other kernel modules
├── shared/
│   ├── Cargo.toml            # Shared library dependencies
│   └── src/
│       ├── lib.rs            # Library root (no_std compatible)
│       ├── handoff.rs        # Handoff structure definition
│       └── constants.rs      # Shared constants
├── tests/                    # Test binaries
│   ├── bare_metal_tests.rs
│   ├── kernel_tests.rs
│   └── should_panic.rs
├── build/                    # Build output directory
│   ├── EFI/BOOT/BOOTX64.EFI # Single unified binary
│   └── disk.img             # GPT disk image
├── OVMF/                    # OVMF firmware files
└── target/                  # Cargo build cache
```

**Note**: The kernel no longer builds as a separate binary. It's a library linked into the bootloader.

## How It Works

### 1. **Workspace Configuration** (The Master Plan)
- **Root `Cargo.toml`**: This is like the "master configuration" that tells Rust about all the different parts of our project
- **Profile Settings**: Special settings that make our code work without the standard library (needed for operating systems)
- **Dependency Management**: All parts of the project use the same versions of external libraries

### 2. **Shared Library** (`theseus_shared`) - The Common Ground
- **What it does**: Contains data structures that both the bootloader and kernel need to share
- **Why it's special**: It works in both UEFI and kernel environments (no standard library)
- **Key component**: The "handoff structure" - a way for the bootloader to pass information to the kernel
- **Think of it as**: A shared vocabulary that both parts of the system understand

### 3. **Bootloader Crate** (`bootloader`) - The UEFI Phase
- **What it does**: UEFI application entry point; collects system info; calls kernel
- **Environment**: Runs in UEFI with boot services active
- **Key tasks**: 
  - Collect hardware and system information
  - Compute kernel image base/size from loaded binary
  - Allocate non-overlapping temporary heap
  - Exit UEFI boot services
  - Call `kernel_entry` directly
- **Output**: Uses QEMU debug port and UEFI console
- **Dependencies**: Depends on `theseus-kernel` (library), links everything together

### 4. **Kernel Library** (`kernel`) - The OS Core
- **What it does**: Provides `kernel_entry` function and all kernel subsystems
- **Environment**: Called after ExitBootServices; runs on bare metal
- **Key tasks**:
  - Establish higher-half virtual memory mapping
  - Initialize permanent heap allocator
  - Set up interrupts, GDT, IDT, LAPIC
  - Configure CPU features
  - Manage framebuffer and timer
- **Current status**: Full initialization working, enters idle loop with heart animation

### 5. **Build System Integration**
- **Makefile Updates**: Handles multiple targets with correct dependencies
- **Cargo Configuration**: Proper target specifications and linker settings
- **Release Profiles**: Optimized builds for both bootloader and kernel

## Build Commands

### Cargo Commands
```bash
# Build bootloader (automatically builds kernel library as dependency)
cargo build --package theseus-bootloader --target x86_64-unknown-uefi

# Build kernel library only
cargo build --lib --package theseus-kernel --target x86_64-unknown-none

# Build shared library only
cargo build --package theseus-shared
```

### Makefile Targets (Recommended)
```bash
# Build unified binary
make build

# Build and create ESP
make esp

# Full build with ESP and BIOS setup
make all

# Build and run
make run
```

## How the Bootloader and Kernel Communicate

The "handoff structure" is like a message that the bootloader writes to the kernel. Here's how it works:

### 1. **Bootloader Phase** (System Startup)
- **Step 1**: Collect information about the computer (how much memory, what hardware, etc.)
- **Step 2**: Write this information into a special data structure called `HANDOFF`
- **Step 3**: Exit the UEFI environment (we no longer need the motherboard's help)
- **Step 4**: Jump to the kernel and pass it the address of the `HANDOFF` structure

### 2. **Kernel Phase** (Operating System Takes Over)
- **Step 1**: `kernel_entry(handoff_addr)` is called directly by bootloader
- **Step 2**: Validate and read the `Handoff` structure
- **Step 3**: Set up higher-half virtual memory mapping
- **Step 4**: Initialize heap, interrupts, and all kernel subsystems
- **Step 5**: Begin normal operating system operation

**Think of it like**: The bootloader is like a real estate agent who shows you around a house (the computer) and gives you a detailed report about what's available. The kernel is like the new owner who uses that report to set up the house properly.

## Technical Implementation Details

### **No-Std Compatibility**
- Shared library uses `#![no_std]` for kernel compatibility
- Removed `std` dependencies and derive macros
- Custom panic handlers for both bootloader and kernel

### **Target Configuration**
- **Bootloader**: `x86_64-unknown-uefi` with UEFI runtime
- **Kernel**: `x86_64-unknown-none` with custom linker script
- **Linker Script**: Defines kernel memory layout starting at 0x100000

### **Memory Safety**
- Handoff structure uses `#[repr(C)]` for stable layout
- Static mutable references documented with safety assumptions
- Bounds checking implemented where needed

### **Output System**
- **Bootloader**: Flexible driver system with automatic selection
- **Kernel**: Direct QEMU debug port (0xe9) communication
- **Panic Handling**: Both environments provide debug output on panic

## Current Status (What's Working Now)

### ✅ **Completed Features**
- **Single-Binary Architecture**: Bootloader and kernel in one unified `BOOTX64.EFI`
- **System Information Collection**: Complete hardware, memory, ACPI detection
- **Direct Kernel Entry**: Function call from bootloader to kernel (same binary)
- **Higher-Half Mapping**: Full virtual memory with 0xFFFFFFFF80000000 kernel base
- **Heap Management**: UEFI allocator during boot, permanent kernel heap after transition
- **Interrupt Handling**: IDT, LAPIC timer, exception handlers all working
- **Framebuffer**: GOP-based framebuffer with drawing and animation
- **Documentation**: Complete rustdoc documentation throughout codebase

### 🚧 **In Development**
- **Device Drivers**: Extended driver framework
- **ACPI Parsing**: Full ACPI table processing
- **SMP Support**: Multi-processor initialization

### 🔮 **Future Goals**
- **Process Management**: Task scheduling and context switching
- **User Programs**: Ability to run user applications
- **File System**: VFS and filesystem support
- **Real Hardware**: Testing on physical machines

## Benefits of Single-Binary Setup

1. **Simplified Deployment**: One file instead of two
2. **No ELF Loading**: Eliminates complex file I/O and parsing
3. **Direct Function Call**: Clean, debuggable transition from bootloader to kernel
4. **Unified Debugging**: All symbols in one binary for GDB
5. **UEFI Allocator Throughout**: Can use allocations during entire boot phase
6. **No Relocation Issues**: Avoids problems with separate binaries and optimizations
7. **Faster Boot**: No disk I/O for kernel loading
8. **Shared Code**: Common structures via workspace dependencies
9. **Rust Best Practices**: Follows standard workspace and library patterns
10. **Easy Testing**: Tests can include kernel library directly

This architecture provides a robust foundation for OS development while simplifying the build and boot process.
