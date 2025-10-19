# Development & Debugging

This guide walks through the tooling around TheseusOS so you can build, run, and inspect the system quickly. Everything below assumes you are working from the project root.

## Toolchain & Workspace
- Rust nightly is pinned via `rust-toolchain.toml`.
- Cargo workspaces: `bootloader`, `kernel`, and shared crates under `shared/`.
- The top-level `Makefile` orchestrates building the unified UEFI binary, copying OVMF firmware blobs, and creating a bootable disk image in `build/disk.img`.

## Building
```bash
# Build bootloader + kernel + shared crates, fetch OVMF firmware
make all

# (Optional) Clean artifacts
make clean
```

## Running Under QEMU
Use the convenience script `startQemu.sh`:

```bash
# Headless run with serial/monitor exposed via named pipes
./startQemu.sh true

# Headed run with QEMU window and debug.log capturing port 0xE9 output
./startQemu.sh false

# Add extra QEMU flags (example: wait for GDB)
QEMU_OPTS="-S -s" ./startQemu.sh true
```

Script highlights:
- Removes stale `debug.log`/`qemu.log`.
- Builds automatically (`make all`) before launching QEMU.
- Configures the ISA debug exit device so the kernel can terminate QEMU with success/failure codes (`theseus_shared::qemu_exit_{ok,error}!`).
- Routes COM1 over a named pipe (`/tmp/qemu-serial`) in headless mode. Attach with `screen /tmp/qemu-serial` or `socat - UNIX-CONNECT:/tmp/qemu-serial`.

## Debugging with GDB
- Launch QEMU with `QEMU_OPTS="-S -s"` to pause CPU 0 and listen on TCP 1234.
- Use the provided script `debug.gdb` as a starting point:
  ```bash
  gdb -x debug.gdb
  ```
- Useful breakpoints: `kernel_entry`, `environment::continue_after_stack_switch`, `interrupts::handler_timer`.
- Inspect the bootloader-to-kernel handoff by examining the pointer in `RDI` right before `kernel_entry` runs.

## Logging
- Macros (`log_error!`, `log_warn!`, `log_info!`, `log_debug!`, `log_trace!`) live in `kernel/src/logging`.
- Default routing (configurable in `kernel/src/config.rs`):
  - ERROR/WARN/INFO → QEMU port 0xE9 and serial.
  - DEBUG/TRACE → QEMU port 0xE9 only (to reduce serial noise).
- Adjust at runtime:
  ```
  > loglevel kernel::memory DEBUG
  > logoutput INFO serial
  ```

## Serial Monitor
- Enabled by default via `config::ENABLE_KERNEL_MONITOR`.
- Access through the COM1 serial line (see QEMU pipe above).
- Type `help` for an interactive command list. Favorites:
  - `status` — quick system summary (ticks, heap usage, active drivers).
  - `mem <addr>` — inspect memory.
  - `idt`, `gdt`, `lapic` — inspect descriptor tables and APIC state.
  - `devices`, `drivers` — inspect driver manager registries.
- Monitor processing is interrupt-driven; the CPU can `hlt` between keystrokes.

## Automated Tests
- `run_tests.sh` executes host-side Rust tests and any available guest smoke tests. The script ensures the right targets are built first.
- Integration tests inside `kernel/tests` call directly into kernel functions; the `custom_test_frameworks` feature is stubbed out because the bare-metal harness cannot rely on trait-object-based runners.

## Troubleshooting Tips
- Kernel exits immediately? Check `config::KERNEL_SHOULD_IDLE` (default `false`). Set to `true` when you want to linger in the monitor.
- Heart animation not updating? Confirm the LAPIC timer IRQ counter is increasing (`timer` command in the monitor) and that `interrupts::lapic_timer_configure` succeeded.
- UEFI runtime service failures? See logs around `SetVirtualAddressMap` in `environment::continue_after_stack_switch`; the kernel aborts with context if the call fails.

## Continue Exploring
- For the control-flow walkthrough, visit [Kernel Architecture](kernel-architecture.md).
- Dive into paging details through [Memory Management](memory-management.md).
- Review device and monitor capabilities in [Hardware & Drivers](hardware-and-drivers.md).
