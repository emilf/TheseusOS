# TheseusOS Multi-Binary Workspace Setup

## What is This?

This document explains how TheseusOS is organized as a **multi-binary workspace** - think of it like having multiple separate programs that work together to create an operating system. This is a common pattern in Rust projects where you need different parts of your system to run in different environments.

## Why Multi-Binary?

Operating systems need different components that run in different contexts:

1. **Bootloader**: Runs in the UEFI environment (like a mini-operating system)
2. **Kernel**: Runs in "bare metal" mode (directly on the hardware)
3. **Shared Library**: Contains common data structures both need

This setup allows us to:
- Keep related code together
- Share common data structures between components
- Build each part independently
- Follow Rust best practices for large projects

## Project Structure

```
TheseusOS/
├── Cargo.toml                 # Workspace root with shared dependencies
├── .cargo/
│   └── config.toml           # Kernel target configuration
├── linker.ld                  # Kernel linker script
├── Makefile                  # Build system with multi-target support
├── startQemu.sh              # QEMU execution script
├── bootloader/
│   ├── Cargo.toml            # Bootloader-specific dependencies
│   └── src/
│       ├── main.rs           # UEFI bootloader entry point
│       ├── boot_sequence.rs  # Boot sequence orchestration
│       ├── qemu_exit.rs      # QEMU exit utilities
│       ├── serial.rs         # Serial communication helpers
│       ├── display.rs        # System information display
│       ├── hardware.rs       # Hardware inventory collection
│       ├── acpi.rs           # ACPI table discovery and parsing
│       ├── system_info.rs    # System information collection
│       └── drivers/          # UEFI-specific output drivers
│           ├── mod.rs
│           ├── manager.rs    # Driver selection and management
│           ├── uefi_serial.rs
│           ├── raw_serial.rs
│           └── qemu_debug.rs
├── kernel/
│   ├── Cargo.toml            # Kernel-specific dependencies
│   └── src/
│       └── main.rs           # Kernel entry point
├── shared/
│   ├── Cargo.toml            # Shared library dependencies
│   └── src/
│       ├── lib.rs            # Library root (no_std compatible)
│       ├── handoff.rs        # Handoff structure definition
│       └── constants.rs      # Shared constants and magic numbers
├── build/                    # Build output directory
│   ├── EFI/BOOT/BOOTX64.EFI # Built bootloader binary
│   └── OVMF_VARS.fd         # QEMU firmware variables
├── OVMF/                    # Original OVMF firmware files
└── target/                  # Cargo build cache
```

**Note**: The old monolithic `src/` directory has been removed and replaced with the new workspace structure.

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

### 3. **Bootloader** (`bootloader`) - The System Starter
- **What it does**: Runs first when the computer starts up, collects system information, then starts the kernel
- **Environment**: Runs in UEFI (like a mini-operating system provided by the motherboard)
- **Key tasks**: 
  - Find out what hardware is available
  - Set up memory for the kernel
  - Load the kernel from disk
  - Pass control to the kernel
- **Output**: Can display information during boot process

### 4. **Kernel** (`kernel`) - The Real Operating System
- **What it does**: The actual operating system that manages the computer
- **Environment**: Runs directly on the hardware (no other operating system)
- **Key tasks**:
  - Set up memory management
  - Handle interrupts and exceptions
  - Provide basic system services
- **Current status**: Basic framework is working, ready for more features

### 5. **Build System Integration**
- **Makefile Updates**: Handles multiple targets with correct dependencies
- **Cargo Configuration**: Proper target specifications and linker settings
- **Release Profiles**: Optimized builds for both bootloader and kernel

## Build Commands

### Individual Builds
```bash
# Build shared library
cargo build --package TheseusOS-shared

# Build bootloader (UEFI)
cargo build --package TheseusOS-bootloader --target x86_64-unknown-uefi

# Build kernel (bare metal)
cargo build --package TheseusOS-kernel --target x86_64-unknown-none

# Build everything
cargo build
```

### Makefile Targets
```bash
# Build both bootloader and kernel
make build

# Build bootloader only
make build-bootloader

# Build kernel only
make build-kernel

# Full build with ESP setup
make all
```

## How the Bootloader and Kernel Communicate

The "handoff structure" is like a message that the bootloader writes to the kernel. Here's how it works:

### 1. **Bootloader Phase** (System Startup)
- **Step 1**: Collect information about the computer (how much memory, what hardware, etc.)
- **Step 2**: Write this information into a special data structure called `HANDOFF`
- **Step 3**: Exit the UEFI environment (we no longer need the motherboard's help)
- **Step 4**: Jump to the kernel and pass it the address of the `HANDOFF` structure

### 2. **Kernel Phase** (Operating System Takes Over)
- **Step 1**: Start running at the `kernel_main()` function
- **Step 2**: Read the `HANDOFF` structure to learn about the system
- **Step 3**: Use this information to set up the operating system properly
- **Step 4**: Begin normal operating system operation

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
- **Bootloader**: Successfully collects system information and loads the kernel
- **Kernel Loading**: Can load and execute kernel binaries from the EFI file system
- **Memory Management**: Basic virtual memory setup with identity and high-half mappings
- **System Information**: Comprehensive hardware detection and information collection
- **Documentation**: Complete documentation throughout the codebase

### 🚧 **In Development**
- **Kernel Features**: More advanced memory management and system services
- **Device Drivers**: Support for various hardware devices
- **Process Management**: Basic process and thread management

### 🔮 **Future Goals**
- **User Programs**: Ability to run user applications
- **File System**: Support for reading and writing files
- **Graphics**: Use the framebuffer for graphical output
- **Real Hardware**: Testing on actual computers (not just QEMU)

## Benefits of This Setup

1. **Clean Architecture**: Clear separation between bootloader and kernel
2. **Shared Code**: Common structures and constants without duplication
3. **Independent Development**: Bootloader and kernel can be developed separately
4. **Scalability**: Easy to add more binaries (utilities, tests, etc.)
5. **Rust Best Practices**: Follows standard Rust workspace organization
6. **Build System Integration**: Seamless integration with existing Makefile

This setup provides a solid foundation for developing a complete operating system with proper separation of concerns and shared infrastructure.
