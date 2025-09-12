# HobbyOS Build System

## Quick Start

```bash
# Build and run in headless mode (exits automatically when complete)
make run

# Build and run with GUI (exits automatically when complete)
make run-headed

# Build and run with 20s timeout for testing (faster with auto-exit)
make run-test

# Build only
make build

# Clean build artifacts
make clean

# Debug mode (paused with GDB)
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

## What's Automated

- **BIOS Files**: Automatically finds and copies `OVMF_CODE.fd` and `OVMF_VARS.fd` from system locations
- **ESP Creation**: Automatically creates the EFI System Partition with the correct structure
- **Dependencies**: Builds in the correct order (cargo → ESP → BIOS files)
- **Auto-Exit**: UEFI application exits QEMU gracefully when complete (much faster testing!)
- **Error Handling**: Provides clear error messages if BIOS files are missing

## System Requirements

- **edk2-ovmf**: Install with your package manager
  - Arch: `sudo pacman -S edk2-ovmf`
  - Ubuntu: `sudo apt install ovmf`
- **QEMU**: Install with your package manager
  - Arch: `sudo pacman -S qemu-system-x86`
  - Ubuntu: `sudo apt install qemu-system-x86`

## Troubleshooting

### Missing BIOS Files
If you get errors about missing OVMF files:
1. Install the `edk2-ovmf` package
2. Check that files exist in `/usr/share/edk2-ovmf/x64/` or `/usr/share/edk2/x64/`

### Serial Output in Headed Mode
In headed mode, serial output goes to `serial.log` file to avoid conflicts with the monitor.

### Build Issues
- Run `make clean` to remove all build artifacts
- Ensure you have the latest Rust toolchain with UEFI target: `rustup target add x86_64-unknown-uefi`
