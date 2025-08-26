# Overview

This project provides a minimal Rust UEFI bootloader that:
1. Initializes UEFI services and logging
2. Prints to screen and serial
3. Collects environment information (firmware, GOP, ACPI)
4. Retrieves the UEFI memory map
5. Exits Boot Services
6. Passes a `Handoff` struct pointer in RDI for the kernel
7. Halts (placeholder until chaining to a kernel)

## Boot Flow
- UEFI loads `EFI/BOOT/BOOTX64.EFI` from the ESP.
- Entry `efi_main` runs with `SystemTable<Boot>`.
- We log to the screen and write to the serial port.
- We query GOP for framebuffer info and ACPI RSDP from the config table.
- We retrieve the memory map and summarize it.
- We call `ExitBootServices` (convenience API for this crate version).
- We store the address of the `Handoff` struct into `RDI` for the kernel.
- We halt the CPU.

## Handoff ABI
C-compatible layout (`#[repr(C)]`):

```
struct Handoff {
  size: u32,
  firmware_revision: u32,
  rsdp_address: u64,
  gop_fb_base: u64,
  gop_fb_size: u64,
  gop_width: u32,
  gop_height: u32,
  gop_stride: u32,
  gop_pixel_format: u32,
  mmap_ptr: u64,
  mmap_len: u64,
  mmap_desc_size: u32,
  mmap_desc_version: u32,
}
```

- Register: `RDI` = pointer to `Handoff`
- Size: `Handoff.size` in bytes
- ACPI: `rsdp_address` is valid after exit
- Graphics: `gop_*` describe the framebuffer
- Memory Map: `mmap_*` points to the raw buffer we retrieved prior to exit

## Debugging
- `startQemu.sh` runs QEMU with `-s -S` and a monitor (`telnet:127.0.0.1:55555`).
- Use `gdb -ex 'target remote :1234'` to connect and breakpoints as needed.
- Inspect `RDI` after exit to read the `Handoff` struct.

## Extending to a Kernel
- Add kernel loading and jump to entry; preserve RDI.
- Initialize paging and memory manager using the memory map and `Handoff.size`.
- Hand off control quickly; minimize time spent in EFI runtime.
