# QEMU Runner (`theseus-qemu`)

TheseusOS includes a small Rust CLI tool to **generate** and **run** the QEMU command-line in a reproducible way.

It is intended to replace (or eventually supersede) `startQemu.sh` once we’ve reached feature parity and added profiles.

## Why a Rust runner?

- **Profiles**: named, composable configurations (minimal, usb-kbd, net, storage, etc.).
- **Opt-in sockets**: when working in restricted environments (e.g. editor sandboxes), you may not be allowed to bind `/tmp` sockets. The runner makes serial/QMP/HMP endpoints explicit flags.
- **Reproducibility**: `print` and `artifact` make it easy to copy/paste or store the exact argv.

## Location

- Source: `tools/theseus-qemu/`
- Run via Cargo workspace:

```bash
cargo run -p theseus-qemu -- --help
```

## Basic usage

### Default behavior: run

Running **without** a subcommand executes QEMU:

```bash
# headed by default (QEMU window)
cargo run -p theseus-qemu --

# headless
cargo run -p theseus-qemu -- --headless
```

### Print the command instead of running

```bash
cargo run -p theseus-qemu -- print --headless
```

### Dry mode (print without requiring build artifacts)

`--dry` disables the runner’s preflight checks for required files (OVMF vars, disk image, etc.).
This is useful when you want to **see** the command, even if you can’t build/run right now.

```bash
cargo run -p theseus-qemu -- print --dry --profile min --headless
```

Notes:
- `print` and `artifact` imply `--dry` automatically.
- `--dry` does **not** make QEMU succeed if the files truly don’t exist; it only skips the runner checks.

## Profiles

Current profiles (early days):
- `default`: mirrors `startQemu.sh` device defaults (q35, nvme, root ports, xhci, virtio-gpu, virtio-net)
- `min`: minimal bring-up (no GPU/USB/NIC)
- `usb-kbd`: xHCI + usb keyboard

Example:

```bash
cargo run -p theseus-qemu -- --profile usb-kbd --headless
```

## Opt-in automation endpoints (serial/QMP/HMP)

These flags are intentionally explicit so humans can run without sockets by default, while automation can enable them.

### Serial

```bash
# serial over stdio
cargo run -p theseus-qemu -- --serial stdio

# serial over unix socket (for automation)
cargo run -p theseus-qemu -- --serial unix --serial-path /tmp/theseus-serial.sock
```

### Monitor + serial relays (recommended for interactive debugging)

TheseusOS ships systemd user units (see `scripts/`) that create stable PTY endpoints under `/tmp`:

- monitor: `/tmp/qemu-monitor` (QEMU side) and `/tmp/qemu-monitor-host` (your terminal/minicom)
- serial: `/tmp/qemu-serial` (QEMU side) and `/tmp/qemu-serial-host` (your terminal/minicom)
- debugcon: `/tmp/qemu-debugcon` (QEMU side) and `/tmp/qemu-debugcon-host` (PTY stream; use `cat`, not `tail -f`)
  - optional log file (enable `qemu_debugcon_logger.service`): `/tmp/qemu-debugcon.log` (then you can `tail -f`)

Enable them:

```bash
./scripts/install-qemu-relays.sh
```

Use them with the runner:

```bash
# single convenience flag
cargo run -p theseus-qemu -- --relays

Note: `--relays` assumes you have the relay PTYs running (see `./scripts/install-qemu-relays.sh`). It routes QEMU monitor/serial/debugcon into the QEMU-side PTYs under `/tmp/qemu-*`.

# or explicit flags
cargo run -p theseus-qemu -- --serial unix --serial-path /tmp/qemu-serial
cargo run -p theseus-qemu -- --monitor-pty
cargo run -p theseus-qemu -- --debugcon-pty
```

### QMP (machine control)

```bash
# default path
cargo run -p theseus-qemu -- --qmp

# custom path
cargo run -p theseus-qemu -- --qmp /tmp/qemu-qmp.sock
```

If you also enable the QMP relay unit, it exposes a stable host socket:
- `/tmp/qemu-qmp-host.sock` → forwards to `/tmp/qemu-qmp.sock`

### HMP unix socket (optional)

```bash
# default path
cargo run -p theseus-qemu -- --hmp

# custom path
cargo run -p theseus-qemu -- --hmp /tmp/theseus-hmp.sock
```

## Artifacts

You can emit a JSON file with the resolved argv:

```bash
cargo run -p theseus-qemu -- artifact --out build/qemu-argv.json
```

## Relationship with `startQemu.sh`

Right now `startQemu.sh` remains the historical reference implementation.

As of the `feat/theseus-qemu-parity` work, the Rust runner supports several of the practical conveniences from `startQemu.sh`:
- **Build-before-run** (default): runs `make all` before launching QEMU. Disable with `--no-build`.
- **Timeout**: `--timeout-secs N` runs QEMU under `timeout --foreground`.
- **Success marker**: if QEMU output contains the marker string (default: `Kernel environment test completed successfully`), the runner forces exit code 0. Override via `--success-marker`.

The runner still focuses on:
- stable argv generation
- profile selection
- explicit socket toggles

Next steps for parity:
- richer profiles (net/storage variants)
- optional log cleanup and artifact capture
- first-class QMP/HMP helpers (later skills will consume these)
