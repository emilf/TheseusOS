# Kernel Monitor

The TheseusOS Kernel Monitor is an interactive debugging console inspired by Steve Wozn iak's Wozmon but extended with modern debugging features. It provides a serial-based command-line interface for examining and modifying system state at runtime.

## Features

### Memory Operations
- **`mem ADDR`** - Examine 16 bytes at address, continue with repeated calls
- **`dump ADDR [LEN]`** - Hex dump of memory region (default 256 bytes)
- **`write ADDR VAL`** - Write a byte to memory
- **`fill ADDR LEN VAL`** - Fill memory region with a value

### System Information
- **`regs`** / **`r`** - Display CPU registers (general purpose, control, segment)
- **`devices`** / **`dev`** - List all registered hardware devices
- **`acpi`** - Display ACPI platform information
- **`mmap`** - Show UEFI memory map
- **`cpuid`** - CPU identification and features

### System Tables
- **`idt`** - Display Interrupt Descriptor Table information
- **`gdt`** - Display Global Descriptor Table information

### Debugging
- **`stack`** / **`bt`** - Stack backtrace (walks RBP chain)
- **`msr ADDR`** - Read model-specific register
- **`io r PORT`** - Read from I/O port
- **`io w PORT VAL`** - Write to I/O port
- **`int NUM`** - Trigger software interrupt (limited to INT3 for safety)
- **`call ADDR`** - Call function at address (dangerous!)

### System Control
- **`reset`** - Reset the system
- **`halt`** - Halt the CPU
- **`clear`** / **`cls`** - Clear screen (ANSI)

### Special Keys
- **Enter** - Execute command
- **Backspace** / **DEL** - Delete last character
- **Ctrl+C** - Cancel current line
- **Ctrl+L** - Clear screen

## Usage

### Enabling the Monitor

Set the configuration flag in `kernel/src/config.rs`:

```rust
pub const ENABLE_KERNEL_MONITOR: bool = true;
```

Rebuild and run:

```bash
make build
./startQemu.sh headless
```

The monitor will start automatically after kernel initialization. Debug output goes to `qemu-debug.log`, and the interactive monitor appears on stdio.

### Example Session

```
=================================================
  TheseusOS Kernel Monitor v0.1
  Inspired by Wozmon - Extended for modern debug
=================================================

Type 'help' for available commands

> help
Available commands:

Memory Operations:
  mem ADDR           - Examine 16 bytes at ADDR
  dump ADDR [LEN]    - Hex dump memory region
  write ADDR VAL     - Write byte to memory
  fill ADDR LEN VAL  - Fill memory region

System Information:
  regs, r            - Display CPU registers
  devices, dev       - List hardware devices
  acpi               - ACPI platform info
  mmap               - Memory map from UEFI
  cpuid              - CPU identification

...

> regs
CPU Registers:
  RAX: 0x0000000000000000  RBX: 0x0000000000000000
  RCX: 0x0000000000000000  RDX: 0x0000000000000000
  RSI: 0x0000000000000000  RDI: 0x0000000000000000
  RSP: 0xFFFF800000123456  RBP: 0xFFFF800000123478

Control Registers:
  CR0: 0x0000000080050033  CR2: 0x0000000000000000
  CR3: 0x0000000000011000  CR4: 0x00000000000006A0

Segment Registers:
  CS: 0x0008  DS: 0x0010  SS: 0x0010  ES: 0x0010

> mem 0xFFFFFFFF80000000
FFFF FFFF80000000: 4D 5A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 |MZ..............|

> mem
FFFF FFFF80000010: 00 00 00 00 00 00 00 00 40 00 00 00 00 00 00 00 |........@.......|

> dump 0xFFFF800000000000 64
Memory dump at 0xFFFF800000000000, 64 bytes:
FFFF800000000000: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 |................|
FFFF800000000010: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 |................|
FFFF800000000020: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 |................|
FFFF800000000030: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 |................|

> devices
Registered devices:
  [ 0] RAW:com1 [BOUND] addr:0x3F8 irq:4
  [ 1] RAW:unknown addr:0x3F8E3518 irq:none
  [ 2] RAW:acpi addr:0x3F048898 irq:none
  [ 3] RAW:vendor addr:0x3ED30598 irq:none
  ...

> cpuid
CPUID Information:
  Vendor: GenuineIntel
  Family: 0x6
  Model: 0x6
  Stepping: 0x3

Features:
  - FPU (Floating Point Unit)
  - PAE (Physical Address Extension)
  - APIC (Advanced PIC)
  - MSR (Model Specific Registers)
  - TSC (Time Stamp Counter)
  - SSE
  - SSE2

> stack
Stack trace:
  Frame # 0: RIP=0xFFFFFFFF801E5ABC RBP=0xFFFF800000237FE0
  Frame # 1: RIP=0xFFFFFFFF801E6123 RBP=0xFFFF800000238000
  ...

> io r 0x3FD
IN 0x3FD = 0x60

> msr 0x1B
MSR 0x1B = 0x00000000FEE00D00

> idt
Interrupt Descriptor Table:
  IDTR Base:  0xFFFF800000240000
  IDTR Limit: 0x0FFF (256 entries)

Key interrupt vectors:
  0x00: Divide Error
  0x03: Breakpoint
  0x06: Invalid Opcode
  0x0D: General Protection Fault
  0x0E: Page Fault
  0x24: COM1 Serial (IRQ 4)
  0x40: APIC Timer
  0xFE: APIC Error
  0xFF: Spurious
```

## Implementation Details

### Interrupt-Driven I/O

The monitor uses interrupt-driven serial I/O for responsive input:

1. **Ring Buffer**: 1024-byte `VecDeque` stores received characters
2. **IRQ 4 Handler**: Reads data from UART and buffers it
3. **Non-blocking Reads**: Monitor polls buffer without busy-waiting

### Architecture

```
User Input (Serial)
    ↓
IRQ 4 Triggered
    ↓  
Serial Driver Interrupt Handler
    ↓
Read byte from UART → Ring Buffer
    ↓
Monitor Poll Loop
    ↓
Read from Ring Buffer
    ↓
Echo & Process Character
    ↓
Execute Command
```

### Command Processing

Commands are line-buffered with basic editing:
- Characters accumulate in `line_buffer`
- Backspace removes characters
- Enter executes the command
- Ctrl+C cancels current line

### Number Parsing

The monitor accepts addresses and values in two formats:
- **Hexadecimal**: `0x1234` or `0X1234`
- **Decimal**: `1234`

### Safety

Most commands require `unsafe` blocks to access raw memory or hardware:
- Memory read/write operations
- Register access via inline assembly
- I/O port operations
- MSR reads

The monitor is intended for debugging and assumes the user knows what they're doing.

## Configuration

### Kernel Config

```rust
// kernel/src/config.rs
pub const ENABLE_KERNEL_MONITOR: bool = true;  // Enable monitor
```

### QEMU Setup

The `startQemu.sh` script automatically configures:
- COM1 Serial → stdio (interactive)
- Debug port (0xe9) → qemu-debug.log (kernel logs)

### Alternative Configurations

```bash
# COM1 to TCP socket (remote access)
-serial tcp:127.0.0.1:4444,server,nowait

# COM1 to telnet (easier interaction)
-serial telnet:127.0.0.1:4444,server,nowait

# COM1 to file (log only, no interaction)
-serial file:monitor.log
```

## Extending the Monitor

### Adding New Commands

1. Add the command to the `match` statement in `process_command()`:

```rust
match parts[0] {
    // ... existing commands ...
    "mycommand" => self.cmd_mycommand(&parts[1..]),
    _ => { /* unknown command */ }
}
```

2. Implement the command handler:

```rust
fn cmd_mycommand(&self, args: &[&str]) {
    self.writeln("Executing my command...");
    // Command implementation
}
```

### Example: PCI Configuration Space Reader

```rust
fn cmd_pci(&self, args: &[&str]) {
    if args.len() < 3 {
        self.writeln("Usage: pci BUS DEV FUNC");
        return;
    }
    
    let bus = parse_number(args[0]).unwrap_or(0) as u8;
    let dev = parse_number(args[1]).unwrap_or(0) as u8;
    let func = parse_number(args[2]).unwrap_or(0) as u8;
    
    // Read PCI config space via CF8/CFC ports
    let addr = 0x80000000u32 
        | ((bus as u32) << 16)
        | ((dev as u32) << 11)
        | ((func as u32) << 8);
    
    unsafe {
        let mut cf8: Port<u32> = Port::new(0xCF8);
        let mut cfc: Port<u32> = Port::new(0xCFC);
        
        cf8.write(addr);
        let vendor_device = cfc.read();
        
        self.writeln(&format!("PCI {:02X}:{:02X}.{:X}", bus, dev, func));
        self.writeln(&format!("  Vendor:Device = 0x{:08X}", vendor_device));
    }
}
```

## Troubleshooting

### Monitor Doesn't Start

**Symptom**: No monitor prompt appears

**Checks**:
1. Verify `ENABLE_KERNEL_MONITOR = true` in config.rs
2. Check that COM1 driver initialized successfully (check `qemu-debug.log`)
3. Ensure QEMU is configured with `-serial stdio`

### No Character Echo

**Symptom**: Typing doesn't show characters

**Solution**: The monitor echoes manually. Check that:
- Serial driver's `write` works
- Driver is bound to COM1 device

### Interrupts Not Working

**Symptom**: Must press Enter multiple times for command to register

**Checks**:
1. Verify IRQ 4 is unmasked (`unmask_pic_irq(4)` was called)
2. Check IDT has handler at vector 0x24
3. Verify serial interrupts are enabled in hardware (IER register bit 0)

### Commands Hang or Crash

**Symptom**: Command causes system to freeze

**Cause**: Invalid memory access or unsafe operation

**Recovery**: Use QEMU monitor or reset

## Future Enhancements

### Planned Features

1. **Expression Evaluator**: Support arithmetic in commands (e.g., `mem 0x1000+0x50`)
2. **Scripting**: Load and execute command scripts
3. **Breakpoints**: Software breakpoints for debugging
4. **Watchpoints**: Break on memory access
5. **Disassembly**: Inline x86-64 disassembler
6. **Symbol Lookup**: Map addresses to function names
7. **Thread List**: Show kernel threads when multithreading is implemented
8. **Memory Allocation Stats**: Heap usage and fragmentation
9. **Performance Counters**: CPU performance monitoring
10. **Network Commands**: When networking is available

### Advanced Use Cases

**Kernel Debugging**:
```
> stack           # See where you are
> mem 0xRIP       # Examine code at crash point  
> dump 0xRSP 128  # See stack contents
```

**Hardware Exploration**:
```
> devices         # See what's present
> io r 0x3F8      # Read COM1 data register
> msr 0x1B        # Check APIC base
```

**Memory Investigation**:
```
> dump 0xFFFFFFFF80000000 4096  # Dump kernel image
> mem 0xFFFF800000000000         # Walk page tables
```

## Integration with Debugging Tools

### QEMU Monitor

Run QEMU with monitor access:

```bash
# Terminal 1: QEMU with serial on stdio
./startQemu.sh headless

# Terminal 2: QEMU monitor via telnet
telnet 127.0.0.1 1234  # If configured with -monitor telnet:...
```

### GDB Integration

The monitor complements GDB:
- GDB: Source-level debugging, breakpoints
- Monitor: Runtime state inspection, hardware access

### addr2line

Map addresses from stack traces:

```bash
# Get address from 'stack' command
> stack
Frame # 0: RIP=0xFFFFFFFF801E5ABC

# Look up symbol
addr2line -e target/x86_64-unknown-uefi/release/theseus_efi.efi 0xFFFFFFFF801E5ABC
```

## Security Considerations

### Dangerous Commands

Some commands can corrupt or crash the system:

- **`write`** - Can overwrite kernel code/data
- **`fill`** - Can destroy large memory regions
- **`call`** - Can jump to invalid code
- **`int`** - Can trigger exceptions
- **`io w`** - Can misconfigure hardware
- **`reset`** - Immediately reboots

### Production Use

**WARNING**: The monitor is a debugging tool. Do NOT enable in production:
- It exposes full system access
- No authentication or authorization
- Can bypass all security boundaries
- Direct hardware manipulation allowed

Always set `ENABLE_KERNEL_MONITOR = false` for production builds.

## Technical Notes

### Interrupt Handling

The serial driver uses interrupt-driven I/O for efficiency:

```rust
// IRQ 4 (vector 0x24) triggers on:
// - Received data available
// - Transmitter holding register empty (if enabled)
// - Receiver line status change
// - Modem status change (if enabled)

// Handler reads IIR to determine cause and buffers data:
fn handle_interrupt(&mut self) {
    let iir = read_iir();
    match (iir >> 1) & 0x07 {
        0b010 => { /* Data available - buffer it */ }
        0b110 => { /* Line status error - clear it */ }
        0b100 => { /* Character timeout - read data */ }
        _ => {}
    }
}
```

### PIC Configuration

IRQ 4 must be unmasked in the PIC:

```rust
// In environment.rs when starting monitor:
unsafe {
    crate::interrupts::unmask_pic_irq(4);
}
```

This clears bit 4 in the PIC mask register (port 0x21).

### Command Parser

The monitor uses a simple whitespace-based parser:

```rust
let line = "dump 0x1000 256";
let parts: Vec<&str> = line.trim().split_whitespace().collect();
// parts[0] = "dump"
// parts[1] = "0x1000"
// parts[2] = "256"
```

Numbers are parsed with `parse_number()` which handles both hex (0x...) and decimal.

## References

- [Wozmon](https://www.sbprojects.net/projects/apple1/wozmon.php) - Original inspiration
- [OSDev Serial Ports](https://wiki.osdev.org/Serial_Ports)
- [16550 UART Datasheet](https://www.ti.com/lit/ds/symlink/pc16550d.pdf)
- [Intel SDM Volume 3](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html) - System programming guide

