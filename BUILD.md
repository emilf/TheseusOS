# TheseusOS Build System

## Getting Started (For Beginners)

TheseusOS is designed to be easy to build and run, even if you're new to operating system development.

### The Simplest Way to Run TheseusOS

```bash
# Build and run TheseusOS (no graphics window - just text output)
make run

# Build and run with a graphics window (shows QEMU window)
make run-headed
```

**That's it!** TheseusOS will build itself and start running. You'll see output in your terminal showing what's happening during boot.

### Other Useful Commands

```bash
# Just build TheseusOS without running it
make build

# Clean up all build files (if something goes wrong)
make clean

# Run with a timeout (useful for testing)
make run-test

# Debug mode (pauses and waits for GDB debugger)
make debug
```

## Manual QEMU Control

You can also use the `startQemu.sh` script directly:

```bash
# Headless mode (exits automatically when complete)
./startQemu.sh headless

# Headed mode (exits automatically when complete)
./startQemu.sh headed

# With timeout (in seconds) - still useful for safety
./startQemu.sh headless 30
./startQemu.sh headed 15

# Debug mode
QEMU_OPTS="-S -s" ./startQemu.sh headless
```

## What the Build System Does for You

The build system handles all the complex parts automatically:

- **Finds BIOS Files**: Automatically locates and copies the UEFI firmware files your system needs
- **Creates Boot Disk**: Sets up a virtual hard drive with the correct structure for UEFI booting
- **Builds Everything**: Compiles the bootloader, kernel, and shared library in the right order
- **Runs QEMU**: Starts the computer simulator with all the right settings
- **Shows Output**: Displays all the boot messages and system information
- **Handles Errors**: Gives you clear error messages if something goes wrong

**You don't need to worry about any of this** - just run `make run` and everything happens automatically!

## What You'll See When Running TheseusOS

When you run TheseusOS, you'll see lots of interesting output showing what's happening:

### Bootloader Phase
- System information collection (memory, CPU, hardware)
- Graphics setup and framebuffer information
- ACPI table discovery
- Hardware inventory
- Kernel loading process

### Kernel Phase
- Memory management setup
- CPU feature detection
- Interrupt handling setup
- Virtual memory configuration
- System initialization

### How to See All Output
```bash
# Run TheseusOS and see everything in your terminal
make run

# If you want to save output to a file for later reading
make run > output.log 2>&1
```

**Note**: The output might look technical, but it's actually showing you exactly what a real operating system does when it starts up!

## What You Need to Install

Before running TheseusOS, you need to install a few things:

### Required Software
- **QEMU**: A computer simulator (like a virtual computer)
  - Arch Linux: `sudo pacman -S qemu-system-x86`
  - Ubuntu: `sudo apt install qemu-system-x86`
  - macOS: `brew install qemu`

- **UEFI Firmware**: The "BIOS" that TheseusOS needs to boot
  - Arch Linux: `sudo pacman -S edk2-ovmf`
  - Ubuntu: `sudo apt install ovmf`

- **Rust**: The programming language TheseusOS is written in
  - Install from [rustup.rs](https://rustup.rs/)
  - Add UEFI target: `rustup target add x86_64-unknown-uefi`

### If Something Goes Wrong

**"Missing OVMF files" error:**
- Make sure you installed the `edk2-ovmf` package
- The build system will tell you exactly what to install

**"Build failed" error:**
- Run `make clean` to start fresh
- Make sure you have the latest Rust toolchain

**"QEMU not found" error:**
- Install QEMU using your package manager
- Make sure it's in your PATH

**Need help?** The error messages are designed to be helpful and tell you exactly what's missing!
