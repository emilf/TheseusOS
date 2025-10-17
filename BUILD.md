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
- **GNU objcopy (binutils)**: Used to convert the EFI PE/COFF binary into an ELF symbol file for GDB. Install via your binutils package (`sudo pacman -S binutils` / `sudo apt install binutils` / `brew install binutils`).

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

## Detailed Build Instructions

This section expands on the quick commands above and documents how the Makefile, cargo, and QEMU work together.

### Overview

- The repository uses a `Makefile` wrapper to orchestrate building the UEFI bootloader, kernel, and shared crates, create an EFI System Partition (ESP) disk image, copy OVMF firmware when needed, and launch QEMU with sensible defaults for testing.
- The Makefile exposes a small set of targets (`build`, `esp`, `run`, `run-headed`, `debug`, `test-*`, `clean`) and a couple of environment knobs to customize behavior.

### Environment variables and options

- `PROFILE` (default: `release`) — controls whether cargo builds release or debug artifacts. Set to `debug` to build debug artifacts and omit the `--release` flag when invoking `cargo`.
  - Example: `make PROFILE=debug build esp`
- `FEATURES` — optional cargo features to pass to `cargo build`, for example `make FEATURES=foo build`.
- `TIMEOUT` — used by test run targets to limit QEMU runtime (seconds).

Internally the Makefile maps `PROFILE` to the correct cargo invocation (release builds use `--release`, debug builds pass no flag) so artifacts end up under `target/<target>/(release|debug)` as expected.

### Common build examples

- Build release (default) and create ESP image:

```bash
make build esp
```

- Build debug and create ESP image (kernel and UEFI bootloader from debug folders will be used in the disk image):

```bash
make PROFILE=debug build esp
```

- Build and run headless in QEMU (release):

```bash
make run
```

- Build and pause QEMU for GDB (debug):

```bash
make PROFILE=debug debug
# then connect with: gdb -ex 'target remote :1234'
```

### Where artifacts are placed

- Unified binary: `target/x86_64-unknown-uefi/(release|debug)/theseus_efi.efi`
- Kernel library: `target/x86_64-unknown-none/(release|debug)/libtheseus_kernel.rlib` (linked into bootloader)
- Test binaries: `target/x86_64-unknown-none/(release|debug)/deps/*` (copied to ESP by test targets)

### Disk image / ESP creation

- The `esp` target creates a FAT32-formatted disk image with proper GPT table and EFI System Partition
- The Makefile copies the unified `BOOTX64.EFI` binary to `EFI/BOOT/BOOTX64.EFI`
- When you build with `PROFILE=debug`, the debug artifact from `target/x86_64-unknown-uefi/debug` is copied
- No separate kernel file needed; everything is in `BOOTX64.EFI`
- Bootloader builds use the workspace target spec `x86_64-unknown-uefi-dwarf.json`, which is a copy of the upstream UEFI target tweaked to emit DWARF debug info instead of PDB. Cargo is invoked with `-Z build-std=core,alloc` (nightly toolchain and `rust-src` component) so no manual setup is required beyond `rustup toolchain install` handled in `rust-toolchain.toml`.

### OVMF / Firmware handling

- If you do not have local OVMF firmware files in the repository's `OVMF/` directory, the Makefile attempts to find them in common system locations and copy them into `OVMF/` for use by QEMU. Install `edk2-ovmf` (Arch) or `ovmf` (Ubuntu) if you get an OVMF-related error.

### Test targets

- The Makefile provides `test-bare-metal`, `test-kernel`, and `test-panic` targets which build the appropriate test binary, create a test disk image containing the test binary and the bootloader, and run it in QEMU with `isa-debug-exit` for automatic pass/fail detection. Use `TIMEOUT` to change the test timeout, e.g. `make TIMEOUT=60 test-bare-metal`.

### Debugging and GDB

- The `debug` Makefile target starts QEMU with the `-S -s` options (GDB stub listening on `:1234` and paused). Connect with your local `gdb` instance:

```bash
gdb -ex 'target remote :1234'
```

You can also start QEMU manually with the `startQemu.sh` helper and pass `QEMU_OPTS="-S -s"` if you prefer.

- EFI builds now emit a companion ELF symbol file at `build/BOOTX64.SYM` using GNU `objcopy`. This file carries DWARF debug info in a format that GDB understands while the firmware-friendly `BOOTX64.EFI` stays unchanged.
- The Makefile exports `RUSTFLAGS += -C debuginfo=2` so release and debug builds carry full DWARF info for symbol generation. You can append more flags by setting `RUSTFLAGS` in the environment; they will be merged.
- When you launch `gdb` with the provided `debug.gdb` script, it automatically loads `build/BOOTX64.SYM` before attaching to QEMU, so symbol lookup and breakpoints work without manual `add-symbol-file` commands.
- Rust symbols keep their crate/module path in DWARF. Break on `'theseus_efi::efi_main'` (note the single quotes) if you set breakpoints manually. Alternatively, source `debug.gdb` and use `theseus-load <efi_main_runtime_addr>` (grab the runtime address from the boot log) to reload symbols at the relocated base and arm software breakpoints on `efi_main` (entry, first line, +0x200, +0x300); reset or restart QEMU afterwards so the next boot re-enters `efi_main` and trips the breakpoints.
- If you previously built with the stock `x86_64-unknown-uefi` target, run `make clean` once so new DWARF-enabled artifacts replace the old PDB-based ones; otherwise GDB may complain about missing `.dwo` files.
- If you regenerate the bootloader with a different profile or feature set, re-run `make esp` (or any target that depends on it) to refresh both `BOOTX64.EFI` and the matching `BOOTX64.SYM`.

### Troubleshooting / common issues

- "Missing OVMF files": install `edk2-ovmf` / `ovmf` or copy your system's OVMF files into the repository `OVMF/` directory.
- "Build failures": Run `make clean` then rebuild; ensure rust toolchain is up to date
- "QEMU can't find KVM": on Linux ensure you have KVM enabled and the user has permission; the Makefile falls back to TCG if KVM isn't available.

### Where to look next

- `Makefile`: the canonical place to see exactly which `cargo` commands, flags, and QEMU options are used.
- `startQemu.sh`: wrapper that sets up environment and runs `qemu-system-x86_64` with options used by the project.

The Makefile includes a `make help` target that prints quick usage and examples, including `PROFILE`, `FEATURES`, and `TIMEOUT`. Use `make help` for a concise summary of common commands.
