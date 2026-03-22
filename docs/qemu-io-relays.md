# QEMU I/O Relay Architecture

This is a workflow guide, not a binding architecture document. When it conflicts
with `docs/axioms/` or `docs/plans/`, the axioms/plans win.

## Why relays exist

QEMU exposes serial, monitor, debugcon, and QMP endpoints for the duration of
its process lifetime. Without relays, any tool (minicom, a log tailer, a script)
that connects to those endpoints must restart every time QEMU restarts — losing
early boot output and requiring manual reconnect.

The relay architecture solves this by inserting **persistent intermediary
sockets** between QEMU and your tools:

```
  minicom  ──────────────────────────────────┐
  cat      ────── /tmp/qemu-serial-host      │
  scripts  ─── (host-side, always present)   │
                       │                     │
                  socat relay                │
                   (systemd service)          │
                       │                     │
              /tmp/qemu-serial ──────── QEMU │
              (QEMU-side, present only       │
               while QEMU is running)        │
                                             ▼
                                      (other channels similar)
```

The relay services stay alive independently of QEMU. Tools connect once to the
host-side endpoint and remain connected across QEMU restarts. QEMU connects to
the QEMU-side endpoint when it starts, and disconnects when it exits — the relay
bridges the gap.

**Important:** the presence of host-side socket files under `/tmp/qemu-*-host`
does **not** imply QEMU is currently running. Those sockets belong to the relay
services, not to QEMU.

## Socket endpoints

| Channel | Host-side (connect here) | QEMU-side (QEMU connects here) |
|---------|--------------------------|-------------------------------|
| Serial console | `/tmp/qemu-serial-host` | `/tmp/qemu-serial` |
| QEMU monitor (HMP) | `/tmp/qemu-monitor-host` | `/tmp/qemu-monitor` |
| Debugcon | `/tmp/qemu-debugcon-host` | `/tmp/qemu-debugcon` |
| QMP | `/tmp/qemu-qmp-host.sock` | `/tmp/qemu-qmp.sock` |
| Debugcon log | `/tmp/qemu-debugcon.log` | (optional; see below) |

All relay sockets are Unix PTYs (serial, monitor, debugcon) or Unix domain
sockets (QMP), created by `socat`.

The debugcon log (`/tmp/qemu-debugcon.log`) is written by a separate optional
logger service (`qemu_debugcon_logger.service`). If that service is running you
can `tail -f /tmp/qemu-debugcon.log`. If it is not running, use
`stdbuf -o0 cat /tmp/qemu-debugcon-host` instead (do not use `tail -f` on the
PTY directly).

## Relay services

The relays are **systemd user services**. They are not part of QEMU itself.

### Install

```bash
./scripts/install-qemu-relays.sh
```

This installs and enables the relay units under `~/.config/systemd/user/`.

### Start / status

```bash
# Check status of all relay services
systemctl --user status qemu-serial-relay.service \
                        qemu-monitor-relay.service \
                        qemu-debugcon-relay.service \
                        qemu-qmp-relay.service

# Start all manually if not enabled
systemctl --user start qemu-serial-relay.service \
                       qemu-monitor-relay.service \
                       qemu-debugcon-relay.service \
                       qemu-qmp-relay.service
```

### Verify endpoints are present

```bash
ls -l /tmp/qemu-*-host /tmp/qemu-qmp-host.sock 2>/dev/null
```

If the files are missing, the relay services are not running.

## Connecting tools

### Serial console

```bash
# Read output (exits after 2 s of silence — adjust as needed)
stdbuf -o0 cat /tmp/qemu-serial-host

# Write to serial
printf 'your-input\r' > /tmp/qemu-serial-host

# Interactive via minicom (connect, then detach with Ctrl-A X)
minicom -D /tmp/qemu-serial-host
```

### QEMU monitor (HMP)

```bash
# Send a one-shot HMP command
printf 'info registers\r' > /tmp/qemu-monitor-host
stdbuf -o0 cat /tmp/qemu-monitor-host | head -40

# Interactive
minicom -D /tmp/qemu-monitor-host
```

### Debugcon

```bash
# Stream debugcon output (preferred — avoids tail -f on PTY)
stdbuf -o0 cat /tmp/qemu-debugcon-host

# If the logger service is running
tail -f /tmp/qemu-debugcon.log
```

### QMP (machine protocol)

Use the bundled helper for one-shot QMP commands:

```bash
./scripts/qmp-command.py status
./scripts/qmp-command.py reset
./scripts/qmp-command.py quit
./scripts/qmp-command.py hmp 'info pci'
./scripts/qmp-command.py cmd query-pci
```

The helper connects to `/tmp/qemu-qmp-host.sock` by default, negotiates QMP
capabilities, and prints the JSON reply.

For raw QMP access:

```bash
socat - UNIX-CONNECT:/tmp/qemu-qmp-host.sock
```

## Running QEMU with relays

### Preferred: Rust runner with `--relays`

```bash
cargo run -p theseus-qemu -- --relays --headless
```

`--relays` routes QEMU serial/monitor/debugcon into the QEMU-side PTY endpoints
under `/tmp/qemu-*`. The relay services must already be running.

### All-in-one: tmux helper

```bash
./scripts/tmux-qemu-live.sh
```

This script:
- Creates a `theseus-live` tmux session (or reuses an existing one)
- Starts `socat` relay processes in dedicated tmux windows for all four channels
- Launches headless QEMU via `theseus-qemu --relays`
- Leaves stable host-side endpoints under `/tmp/qemu-*-host`

Use this when you want a self-contained environment without the systemd relay
services, or for a quick interactive session. The tmux relays serve the same
role as the systemd services but are scoped to the tmux session lifetime.

```bash
# Attach to the running session
tmux attach -t theseus-live

# Switch between windows
# serial output:  window 'serial-relay'
# monitor:        window 'monitor-relay'
# debugcon:       window 'debugcon-relay'
# QMP:            window 'qmp-relay'
# QEMU runner:    window 'qemu'
```

## Relationship between systemd relays and tmux relays

Both approaches expose the same host-side socket paths. They are alternatives,
not complements — do not run both simultaneously or they will conflict on the
same socket paths.

| | systemd relay services | tmux-qemu-live.sh |
|-|------------------------|-------------------|
| Lifetime | Persistent across reboots | Scoped to tmux session |
| Setup | `install-qemu-relays.sh` once | No setup required |
| Best for | Day-to-day development | Quick/ad-hoc sessions |

## See also

- [`docs/qemu-runner.md`](qemu-runner.md) — Rust runner flags, profiles, and argv generation
- [`docs/development-and-debugging.md`](development-and-debugging.md) — full build and debug workflow
- [`docs/archive/qemu_io_documentation.md`](archive/qemu_io_documentation.md) — original named-pipe design (historical reference)
