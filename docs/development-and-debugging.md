# Development & Debugging

This guide walks through the tooling around TheseusOS so you can build, run, and inspect the system quickly. Everything below assumes you are working from the project root.

This is a workflow guide, not a binding architecture document; when it conflicts with `docs/axioms/` or `docs/plans/`, the axioms/plans win.

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
Preferred current path: use the Rust runner `tools/theseus-qemu` for reproducible argv generation and explicit relay/QMP/serial toggles.

```bash
# Headless run
cargo run -p theseus-qemu -- --headless

# Print the resolved QEMU command without running it
cargo run -p theseus-qemu -- print --headless

# Use the standard relay endpoints
cargo run -p theseus-qemu -- --relays --headless
```

The older `startQemu.sh` script still exists as historical/convenience glue, but the Rust runner is the preferred current run path.

For a self-contained interactive debugging loop that also creates tmux-managed relay endpoints, use:

```bash
./scripts/tmux-qemu-live.sh
```

That helper script:
- creates a `theseus-live` tmux session
- starts serial / monitor / debugcon / QMP relays with `socat`
- launches headless QEMU via `theseus-qemu --relays`
- leaves stable host-side endpoints under `/tmp/qemu-*-host`

For one-shot QMP control against the host-side relay socket, use:

```bash
./scripts/qmp-command.py status
./scripts/qmp-command.py hmp 'info pci'
./scripts/qmp-command.py reset
```

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
  - ERROR/WARN → both QEMU debug and serial by default.
  - INFO/DEBUG/TRACE → QEMU debug by default unless config/runtime routing is changed.
- Adjust at runtime:
  ```
  > loglevel kernel::memory DEBUG
  > logoutput INFO serial
  ```

## Serial Monitor
- Enabled by default via `config::ENABLE_KERNEL_MONITOR`.
- Access through the COM1 serial line (see QEMU pipe above).
- Type `help` for an interactive command list. Favorites:
  - `status` — quick system summary (timer ticks, monitor/idle state, physical-memory summary, platform info, registered devices).
  - `mem <addr>` — inspect memory.
  - `idt`, `gdt`, `lapic` — inspect descriptor tables and APIC state.
  - `devices`, `drivers` — inspect driver manager registries.
- Monitor processing is interrupt-driven; the CPU can `hlt` between keystrokes.

### Practical note: talking to the serial monitor from automation
- Writing commands into the serial PTY is easy: `printf 'status\r' > /tmp/qemu-serial-host`
- Reading responses back from that same PTY can be awkward/non-deterministic in automation because PTY semantics, relay timing, and already-buffered boot output interact in annoying ways.
- For ad-hoc human use, the tmux relay workflow is fine.
- For deterministic command/response capture, prefer a one-off QEMU run with serial on stdio and delay the command until the monitor prompt is ready, e.g.:
  ```bash
  (sleep 4; printf 'status\r') | cargo run -p theseus-qemu -- \
    --headless --profile min --serial stdio \
    --debugcon-pty /tmp/qemu-debugcon \
    --no-build --no-qemu-debug --timeout-secs 10
  ```
- If another QEMU instance already has `build/disk.img` open, stop that instance first or use a separate disk image/copy.

## Automated Tests
- `run_tests.sh` executes host-side Rust tests and any available guest smoke tests. The script ensures the right targets are built first.
- The crate still carries the `custom_test_frameworks` hook for compatibility, but the current bare-metal test workflow does not rely on trait-object-based runners.

## Troubleshooting Tips
- Kernel exits immediately? Check `config::KERNEL_SHOULD_IDLE` (default `true`). Set it to `false` when you want the kernel to exit QEMU instead of lingering in the monitor/idle path.
- Heart animation not updating? Confirm the LAPIC timer IRQ counter is increasing (`timer` command in the monitor) and that `interrupts::lapic_timer_configure` succeeded.
- UEFI runtime service failures? See logs around `SetVirtualAddressMap` in `environment::continue_after_stack_switch`; the kernel aborts with context if the call fails.

## Continue Exploring
- For the control-flow walkthrough, visit [Kernel Architecture](kernel-architecture.md).
- Dive into paging details through [Memory Management](memory-management.md).
- Review device and monitor capabilities in [Hardware & Drivers](hardware-and-drivers.md).
