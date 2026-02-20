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

### QMP (recommended for automation)

```bash
cargo run -p theseus-qemu -- --qmp /tmp/theseus-qmp.sock
```

### HMP monitor socket (optional)

```bash
cargo run -p theseus-qemu -- --hmp /tmp/theseus-hmp.sock
```

## Artifacts

You can emit a JSON file with the resolved argv:

```bash
cargo run -p theseus-qemu -- artifact --out build/qemu-argv.json
```

## Relationship with `startQemu.sh`

Right now `startQemu.sh` remains the reference implementation and includes build + timeout + success-marker parsing.

The Rust runner currently focuses on:
- stable argv generation
- profile selection
- explicit socket toggles

We’ll incrementally migrate the remaining features (build orchestration, success-marker exit normalization, richer device profiles) into the Rust runner.
