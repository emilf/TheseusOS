# Complete Guide to Serial Communication and WozMon Monitor

## Quick Reference

This document provides a high-level overview of the serial communication system and interactive monitor. For detailed information, see:

- **[WOZMON_SERIAL_MONITOR.md](WOZMON_SERIAL_MONITOR.md)** - Complete monitor documentation with command reference
- **Source Code**:
  - `kernel/src/monitor.rs` - Monitor implementation
  - `kernel/src/drivers/serial.rs` - Serial driver (16550 UART)
  - `kernel/src/serial_debug.rs` - Debug utilities
  - `kernel/src/drivers/traits.rs` - Driver framework
  - `kernel/src/drivers/manager.rs` - Driver manager
  - `bootloader/src/serial.rs` - UEFI bootloader serial

## System Overview

```text
┌─────────────────────────────────────────────────────────────────┐
│                     User Terminal                                │
│            (minicom, screen, QEMU stdio)                         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            │ RS-232 / Serial Cable
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                    COM1 Serial Port                              │
│                    (I/O Port 0x3F8)                              │
│                    16550 UART Hardware                           │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            │ IRQ 4 (on data received)
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                  Interrupt Controller                            │
│              (IOAPIC → LAPIC → CPU)                              │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            │ Vector 0x41 (SERIAL_RX_VECTOR)
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│               Serial Driver IRQ Handler                          │
│         (kernel/src/drivers/serial.rs)                           │
│  - Reads byte from UART                                          │
│  - Enqueues into circular buffer                                 │
│  - Notifies monitor                                              │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            │ push_serial_byte()
                            │
┌───────────────────────────┴─────────────────────────────────────┐
│                  Kernel Monitor                                  │
│           (kernel/src/monitor.rs)                                │
│  - Accumulates input into line buffer                            │
│  - Parses and executes commands                                  │
│  - Outputs results via serial driver                             │
└──────────────────────────────────────────────────────────────────┘
```

## Key Concepts

### 1. Two-Phase Serial Support

#### Phase 1: UEFI Bootloader (Before Kernel)
- Uses UEFI Serial I/O Protocol
- Simple output-only logging
- Platform-abstracted (works everywhere UEFI does)
- Disappears after ExitBootServices()

#### Phase 2: Kernel (After Boot)
- Custom 16550 UART driver
- Full bidirectional communication
- Interrupt-driven reception
- Direct hardware control

### 2. Interrupt-Driven Reception

The serial driver doesn't poll for input. Instead:

1. UART receives byte from wire
2. Hardware FIFO stores byte
3. UART asserts IRQ 4 signal
4. IOAPIC routes to CPU
5. CPU jumps to interrupt handler
6. Handler reads all available bytes
7. Bytes stored in software circular buffer
8. Monitor notified of new input

This approach is efficient and ensures no data loss.

### 3. Circular Buffer

```text
Initial state (empty):
  Head=0, Tail=0
  [.........................]
   ↑
   Both pointers here

After receiving "ABC":
  Head=3, Tail=0
  [ABC......................]
   ↑  ↑
   T  H

After application reads "A":
  Head=3, Tail=1
  [ABC......................]
      ↑  ↑
      T  H

After receiving more data (wraps around):
  Head=2, Tail=5
  [XY..........CDEFGHIJKLMN..]
     ↑        ↑
     H        T

Buffer full (Head+1 == Tail):
  Head=4, Tail=5
  [XYZW.................ABC.]
        ↑↑
        TH
```

### 4. Monitor Command Flow

```text
User types: mem 0x1000 <Enter>
     ↓
Serial hardware receives bytes: 'm', 'e', 'm', ' ', '0', 'x', '1', '0', '0', '0', '\r'
     ↓
Each byte triggers IRQ → handler enqueues it
     ↓
Monitor's handle_char() called for each byte:
  - 'm' → add to line_buffer, echo back
  - 'e' → add to line_buffer, echo back
  - 'm' → add to line_buffer, echo back
  - ' ' → add to line_buffer, echo back
  - ... (rest of characters)
  - '\r' → line complete!
     ↓
Monitor calls process_command():
  - Splits line into words: ["mem", "0x1000"]
  - Matches "mem" command
  - Calls cmd_memory(["0x1000"])
     ↓
cmd_memory() executes:
  - Parses address: 0x1000
  - Reads 16 bytes from memory
  - Formats output with hex and ASCII
  - Writes to serial port
     ↓
Serial output:
  "0000000000001000: 48 89 5C 24 08 ... |H.\$.H.l$.H.t$.W|\r\n"
     ↓
User's terminal displays the output
```

## Architecture Decisions

### Why Wozmon Style?

Wozmon (1976) was elegant in its simplicity:
- Direct memory access
- Minimal command set
- Immediate feedback
- Continuation patterns (examine next 16 bytes with just Enter)

We preserve this philosophy while adding modern necessities:
- 64-bit addressing
- x86-64 system registers
- ACPI/UEFI integration
- Interrupt-driven I/O

### Why 16550 UART?

The 16550 UART is:
- Universally supported (real hardware and emulators)
- Well-documented
- Simple to program
- Provides hardware FIFOs (reduces interrupt frequency)
- Industry standard since 1990s

### Why Circular Buffer?

A circular buffer is ideal for serial I/O because:
- Fixed size (no dynamic allocation in IRQ context)
- Mutex-protected storage with atomic head/tail indices (safe in IRQ + task contexts)
- Efficient wraparound (modulo arithmetic)
- Natural FIFO behavior
- Handles bursty traffic well

### Why Interrupt-Driven RX, Polled TX?

**Reception (Interrupt-Driven)**:
- Can't predict when data arrives
- Must not miss bytes
- CPU can do other work between bytes
- Latency-sensitive (user expects responsive input)

**Transmission (Polled)**:
- We control when to send
- Output is typically burst (whole line at once)
- Polling simpler than interrupt buffering
- UART is fast enough (115200 bps = ~11520 bytes/sec)

## Common Patterns

### Adding a New Monitor Command

1. Add match arm in `process_command()`:
```rust
match parts[0] {
    // ... existing commands ...
    "mynewcmd" => self.cmd_mynewcmd(&parts[1..]),
}
```

2. Implement command handler:
```rust
fn cmd_mynewcmd(&self, args: &[&str]) {
    if args.is_empty() {
        self.writeln("Usage: mynewcmd ARG");
        return;
    }
    // Do something with args
    self.writeln("Command executed!");
}
```

3. Update help text in `cmd_help()`.

### Reading from Serial in Your Code

```rust
let mut buf = [0u8; 64];
let n = driver_manager()
    .lock()
    .read_class(DeviceClass::Serial, &mut buf)?;

// buf[0..n] contains received data
```

### Writing to Serial in Your Code

```rust
driver_manager()
    .lock()
    .write_class(DeviceClass::Serial, b"Hello, world!\n")?;
```

### Debugging Serial Issues

1. **No output at all**:
   - Check `config::ENABLE_SERIAL_OUTPUT` is true
   - Verify QEMU has `-serial stdio` or equivalent
   - Check baud rate matches (115200)

2. **Garbled output**:
   - Verify baud rate (115200)
   - Check line settings (8N1: 8 data bits, no parity, 1 stop bit)
   - Ensure terminal has flow control disabled

3. **No input (monitor doesn't respond)**:
   - Check `config::ENABLE_KERNEL_MONITOR` is true
   - Verify IRQ 4 is being delivered (check IOAPIC config)
   - Ensure OUT2 bit is set in modem control register
   - Check that interrupts are globally enabled (RFLAGS.IF)

4. **Monitor hangs after command**:
   - Command may have triggered a fault
   - Check kernel logs for panic/exception messages
   - Use QEMU monitor (`Ctrl+A c` in stdio mode) to inspect state

## Performance Considerations

### Serial Bottlenecks

At 115200 bps:
- Maximum throughput: ~11.5 KB/s
- Typical command: <100 bytes
- Response time: <10ms for short commands

Serial I/O is inherently slow, so:
- Don't dump large memory regions continuously
- Use `hlt` when waiting for input (saves power)
- Batch output when possible (write whole lines, not single chars)

### IRQ Overhead

Each serial IRQ:
- Saves CPU state (~20 instructions)
- Jumps to handler (~5 instructions)
- Reads byte(s) from UART (~10 instructions)
- Enqueues to buffer (~20 instructions)
- Notifies monitor (~10 instructions)
- Restores CPU state (~20 instructions)

Total: ~85 instructions per IRQ ≈ 100 CPU cycles

At typical typing speed (5 chars/sec), this is negligible overhead.

### Buffer Sizing

- RX buffer: 1024 bytes
  - At 115200 bps, fills in ~88ms
  - Typical typing is much slower
  - Provides good burst tolerance

- Line buffer: 128 characters
  - Reasonable limit for command length
  - Prevents malicious long-line attacks
  - Fits comfortably on screen

## Security Considerations

### Monitor Access = Root Access

The monitor operates with full kernel privileges. Anyone with serial access can:
- Read arbitrary memory (including passwords, keys)
- Write arbitrary memory (including code, page tables)
- Execute arbitrary code (via `call` command)
- Crash the system (via `reset`, `halt`, or malicious commands)

**Production systems should disable the monitor** (`config::ENABLE_KERNEL_MONITOR = false`)
or secure serial port access.

### Command Safety Levels

✅ **Safe**: help, regs, devices, acpi, cpuid, idt, gdt, mmap, stack, clear
- Read-only operations
- Cannot corrupt system state

⚠️ **Potentially Dangerous**: mem, dump, msr, io r
- Read arbitrary locations
- Could read sensitive data
- Could trigger faults on invalid addresses

🔴 **Very Dangerous**: write, fill, io w, call, int, reset, halt
- Modify system state
- Execute arbitrary code
- Force reboot/shutdown

### Buffer Overflow Protection

- Line buffer has fixed MAX_LINE (128) limit
- Input beyond limit is silently dropped
- No dynamic allocation in IRQ context
- No null-terminated strings (Rust String type)

## Testing

### Manual Testing

1. Boot system with serial console
2. Type `help` to verify monitor is active
3. Test each command category:
   - Memory: `mem 0xFFFF800000100000`
   - Registers: `regs`
   - System: `devices`, `acpi`, `cpuid`
   - I/O: `io r 0x3F8` (read serial data register)

### Automated Testing

The reverse echo session (`serial_debug.rs`) can be automated:

```bash
# Start QEMU with serial on TCP socket
qemu-system-x86_64 ... -serial tcp::4444,server,nowait &

# Connect and test
(sleep 2; echo "hello"; sleep 1) | nc localhost 4444 | grep "olleh"
```

### Stress Testing

```bash
# Send rapid input
yes "mem 0x1000" | head -n 100 | nc localhost 4444

# Send long lines (should truncate)
python3 -c "print('A' * 1000)" | nc localhost 4444
```

## Future Enhancements

### Potential Features

1. **Command History**
   - Up/down arrows to recall previous commands
   - Stored in small circular buffer (10-20 commands)

2. **Tab Completion**
   - ESC sequences for cursor positioning
   - Complete command names
   - Complete hex addresses from symbol table

3. **Breakpoints**
   - Software breakpoints (INT3 instruction)
   - Hardware breakpoints (debug registers)
   - Conditional breakpoints

4. **Memory Watchpoints**
   - Debug registers DR0-DR3
   - Break on read/write/execute

5. **Multi-core Support**
   - View all CPUs' registers
   - Run commands on specific cores
   - Track per-core state

6. **Symbol Resolution**
   - Load kernel symbol table
   - Display function names in stack traces
   - Resolve addresses in commands

7. **Scripting**
   - Execute sequence of commands from file
   - Conditional execution
   - Loops and variables

8. **Enhanced ACPI**
   - Parse and display all ACPI tables
   - MADT, FADT, DSDT, SSDT
   - Device enumeration

## References

### Primary Sources
- [Wozmon Original Source](https://www.sbprojects.net/projects/apple1/wozmon.php)
- [16550 UART Datasheet](http://caro.su/msx/ocm_de1/16550.pdf)
- [Intel 64 SDM](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html)

### Related Documentation
- [OSDev Wiki - Serial Ports](https://wiki.osdev.org/Serial_Ports)
- [OSDev Wiki - 16550 UART](https://wiki.osdev.org/16550_UART)
- [ACPI Specification](https://uefi.org/specifications)
- [UEFI Specification](https://uefi.org/specifications)

### Code Documentation
All source files are heavily commented. Start with:
1. `docs/WOZMON_SERIAL_MONITOR.md` (this overview)
2. `kernel/src/monitor.rs` (monitor implementation)
3. `kernel/src/drivers/serial.rs` (serial driver)

## Conclusion

The TheseusOS serial monitor provides a powerful debugging interface inspired by
classic computing while embracing modern architectures. The combination of:

- Simple, direct hardware control (16550 UART)
- Efficient interrupt-driven I/O (IRQ-based reception)
- Clean abstraction layers (driver framework)
- Comprehensive command set (memory, registers, system info)
- Well-documented codebase (inline comments and guides)

...makes it both accessible to newcomers and useful for experienced developers.

The monitor embodies the Unix philosophy: do one thing (provide debug access) and
do it well. It's not trying to be a full IDE or debugger; it's a lightweight tool
for inspecting and manipulating system state when more sophisticated tools aren't
available or practical.

Happy debugging! 🔧
