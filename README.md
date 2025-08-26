# hobbyos UEFI Bootloader (Rust)

Minimal UEFI application in Rust that prints to screen and serial, gathers execution environment data, exits boot services, and halts. It builds a C-compatible Handoff struct and passes its address in RDI for the kernel to consume.

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
- The app prints environment info, exits boot services, places `&Handoff` in RDI, then halts.

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
- Location: Static storage inside the EFI image
- Register: RDI = pointer to `Handoff`
- Size: `Handoff.size` bytes (kernel can reserve this much for copying)

Fields (high-level):
- Firmware: UEFI revision
- ACPI: RSDP physical address
- Graphics: GOP framebuffer base/size, resolution, stride, pixel format
- Memory: Raw memory map buffer pointer/length, descriptor size/version

See `docs/overview.md` for more details on the boot flow and ABI.

## Next Steps
- Chain-load your kernel: jump to kernel entry preserving RDI
- Switch to manual ExitBootServices with `map_key` if you want a tighter contract
- Initialize paging and memory manager using the memory map and `Handoff.size`
