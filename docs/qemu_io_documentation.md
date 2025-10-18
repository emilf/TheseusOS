# QEMU Serial / Monitor / Debugcon Integration

This project uses **persistent named pipes** for all host I/O channels so that tools like
`minicom`, `tee`, or custom loggers can attach *before or after QEMU runs* without losing
early boot output.

## Architecture

| Device | QEMU Arg | Pipe Files | Typical Use |
|---------|-----------|-------------|--------------|
| Serial console | `-serial pipe:/tmp/qemu-serial` | `/tmp/qemu-serial.in` / `.out` | Kernel printf / console |
| QEMU monitor | `-monitor pipe:/tmp/qemu-monitor` | `/tmp/qemu-monitor.in` / `.out` | Interactive QEMU commands |
| Debugcon | `-debugcon pipe:/tmp/qemu-debugcon` | `/tmp/qemu-debugcon.in` / `.out` | Low-level debug I/O |

Each pair consists of an input and output FIFO.  
- `.in` → host writes → guest/QEMU reads  
- `.out` → guest/QEMU writes → host reads  

All three endpoints are created and kept alive by `systemd --user` services running
`mkfifo` + `socat` to loop the pair so multiple readers/writers can attach.

## Services

| Service | Purpose | Files |
|----------|----------|-------|
| `qemu-serial-relay.service` | Serial I/O | `/tmp/qemu-serial.*` |
| `qemu-monitor-relay.service` | QEMU monitor | `/tmp/qemu-monitor.*` |
| `qemu-debugcon-relay.service` | QEMU debug console | `/tmp/qemu-debugcon.*` |

### Start or enable all

```bash
systemctl --user daemon-reload
systemctl --user enable --now qemu-serial-relay.service qemu-monitor-relay.service qemu-debugcon-relay.service
```

### Check status

```bash
systemctl --user status qemu-serial-relay.service
ls -l /tmp/qemu-*.in /tmp/qemu-*.out
```

## Connecting Tools

- **Serial console**
  ```bash
  minicom -D unix#/tmp/qemu-serial.out
  cat /tmp/qemu-serial.out
  ```

- **QEMU monitor**
  ```bash
  minicom -D unix#/tmp/qemu-monitor.out
  ```

- **Debugcon output**
  ```bash
  tail -f /tmp/qemu-debugcon.out
  ```

## Launching QEMU

Use the provided script:

```bash
./scripts/run-qemu.sh [extra QEMU args...]
```

It will:
- Verify or create pipe pairs.
- Auto-spawn `socat` relays if systemd units aren’t running.
- Start QEMU with all three channels connected.

Example:

```bash
./scripts/run-qemu.sh -smp 4 -enable-kvm -cpu host
```

## Logging

You can easily tee serial output to a file:

```bash
socat -u /tmp/qemu-serial.out - | tee -a serial.log
```

## Advantages

- Never lose early boot output
- Always-on endpoints for external tools
- Works across QEMU restarts
- Clean integration with systemd and shell tools

---

