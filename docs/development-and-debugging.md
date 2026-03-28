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

For a full explanation of the relay architecture, how to connect tools, and the
difference between systemd relay services and tmux-managed relays, see
[`docs/qemu-io-relays.md`](qemu-io-relays.md).

For one-shot QMP control against the host-side relay socket, use:

```bash
./scripts/qmp-command.py status
./scripts/qmp-command.py hmp 'info pci'
./scripts/qmp-command.py reset
```

## Debugging with GDB

### Automated session (recommended)

```bash
make debug-auto
```

That's it. The script (`scripts/gdb-auto.py`) will:

1. Start QEMU with a GDB stub on TCP :1251, kept alive in a tmux pane.
2. Spawn GDB via pexpect (drives it as a real interactive TTY — no batch-mode races).
3. Run `theseus-auto` — a GDB Python command that sets a hardware watchpoint on
   the **debug mailbox** at physical `0x7008`. When `efi_main` starts it writes
   its own runtime address to `0x7000` then the magic sentinel to `0x7008`;
   the watchpoint fires and `theseus-auto` loads DWARF symbols automatically.
4. Drop you into interactive GDB, stopped inside `efi_main` with full Rust
   source-level symbols. No address copying, no probe run, works every boot.

Requires `pexpect` and a `tmux` session named `theseus` (created automatically):

```bash
pip install --break-system-packages pexpect
```

Non-interactive CI mode (exits after verifying breakpoint + printing backtrace):

```bash
make debug-auto-ci
```

### Manual session

If you want direct GDB control, or are debugging something before `efi_main`:

```bash
make debug          # QEMU paused on :1234 with GDB stub
gdb -x debug.gdb    # in a separate terminal
```

Then at the GDB prompt:

```
(gdb) target remote localhost:1234
(gdb) theseus-auto          # watchpoint → symbols → stop at efi_main automatically
```

Or, if you need to load symbols at a specific address manually:

```
(gdb) continue              # let UEFI run; read "efi_main @ 0x..." from debugcon
(gdb) theseus-load 0x<addr> # load symbols at runtime address
```

`debug.gdb` provides three commands:

| Command | What it does |
|---------|-------------|
| `theseus-auto` | Fully automated: watchpoint on mailbox → capture address → load symbols. No argument. |
| `theseus-load <addr>` | Load DWARF at given runtime `efi_main` address; arms breakpoints at entry, +0x200, +0x300. |
| `theseus-go <addr>` | Like `theseus-load` but also issues `continue`. |

Section deltas are computed dynamically from `build/BOOTX64.SYM` on every GDB
startup — no hardcoded offsets that go stale after rebuilds.

### Useful breakpoints

- `kernel_entry` — first kernel code after ExitBootServices
- `environment::continue_after_stack_switch` — post-stack-switch environment init
- `interrupts::handler_timer` — LAPIC timer interrupt path

Inspect the bootloader-to-kernel handoff by examining `RDI` just before `kernel_entry` runs (it holds the `*const Handoff` pointer).

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
