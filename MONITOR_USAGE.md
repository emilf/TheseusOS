# Kernel Monitor Quick Start

## Running the Monitor

### 1. Enable the Monitor

Edit `kernel/src/config.rs`:
```rust
pub const ENABLE_KERNEL_MONITOR: bool = true;
```

### 2. Build and Run

```bash
make build
./startQemu.sh headless
```

The monitor prompt will appear:
```
=================================================
  TheseusOS Kernel Monitor v0.1
  Inspired by Wozmon - Extended for modern debug
=================================================

Type 'help' for available commands

> 
```

### 3. Try Commands

```
> help              # Show all commands
> regs              # Display registers
> devices           # List devices
> mem 0xFFFFFFFF80000000   # Examine kernel memory
> cpuid             # CPU information
> stack             # Stack trace
> halt              # Halt system
```

## Quick Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `help` | Show help | `help` |
| `regs` | CPU registers | `regs` |
| `mem ADDR` | Examine memory | `mem 0x1000` |
| `dump ADDR [LEN]` | Hex dump | `dump 0x1000 256` |
| `write ADDR VAL` | Write byte | `write 0x1000 0xFF` |
| `devices` | List devices | `devices` |
| `cpuid` | CPU info | `cpuid` |
| `stack` | Backtrace | `stack` |
| `idt` | Show IDT | `idt` |
| `io r PORT` | Read I/O port | `io r 0x3F8` |
| `io w PORT VAL` | Write I/O port | `io w 0x3F8 0x41` |
| `msr ADDR` | Read MSR | `msr 0x1B` |
| `halt` | Halt CPU | `halt` |
| `reset` | Reset system | `reset` |

## Tips

- **Continuing memory examination**: Type `mem` without an address to continue from the last address
- **Number formats**: Use `0x` prefix for hex, or plain decimal: `mem 0x1000` or `mem 4096`
- **Backspace works**: Use backspace to edit your command
- **Ctrl+C**: Cancel current line
- **Ctrl+L**: Clear screen

## Disable Monitor

To return to normal kernel operation:

Edit `kernel/src/config.rs`:
```rust
pub const ENABLE_KERNEL_MONITOR: bool = false;
```

Then rebuild:
```bash
make build
```

## Debugging the Monitor Itself

Check `qemu-debug.log` for kernel debug output if the monitor doesn't start properly:

```bash
tail -f qemu-debug.log
```

Look for:
```
[serial] COM1 initialized successfully
[monitor] ENABLE_KERNEL_MONITOR=true, entering monitor...
[monitor] starting kernel monitor
```

## See Also

- `docs/kernel-monitor.md` - Comprehensive monitor documentation
- `docs/serial-driver.md` - Serial driver implementation
- `docs/hardware-inventory.md` - Hardware device discovery

