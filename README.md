# hobbyos UEFI Bootloader (Rust)

A comprehensive UEFI application in Rust that provides a complete system information collection and kernel handoff infrastructure. Features beautiful serial output formatting, comprehensive system information gathering, proper memory management, and a detailed C-compatible Handoff struct for kernel consumption.

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
cargo build --release
make esp
./startQemu.sh
```
- ESP layout: `build/EFI/BOOT/BOOTX64.EFI`
- Serial output is redirected to the terminal; video output appears in the VM window.
- The app collects comprehensive system information, prepares memory map for kernel handoff, and places `&Handoff` in RDI, then halts.

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

## Handoff ABI (to your kernel)
- **Location**: Static storage inside the EFI image
- **Register**: RDI = pointer to `Handoff`
- **Size**: `Handoff.size` bytes (168 bytes total)

### Comprehensive System Information:
- **Graphics**: GOP framebuffer base/size, resolution, stride, pixel format
- **Memory Map**: Complete memory layout with descriptor metadata and key
- **ACPI**: RSDP physical address (when available)
- **Device Tree**: DTB pointer and size (ARM systems)
- **Firmware**: UEFI vendor info and revision
- **Boot Context**: Boot time and device path information
- **CPU**: Processor count, features, and microcode revision
- **Hardware Inventory**: Comprehensive device enumeration with handles and metadata

See `docs/overview.md` for complete Handoff structure details and boot flow.

## Next Steps
- **Kernel Loading**: Chain-load your kernel: jump to kernel entry preserving RDI
- **Memory Management**: Initialize paging and memory manager using the comprehensive memory map information
- **System Initialization**: Use the complete Handoff structure for full system setup
- **Boot Services**: Implement proper `exit_boot_services` call with memory map key
- **Runtime Transition**: Complete the transition from boot services to kernel runtime
