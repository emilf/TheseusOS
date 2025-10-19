# WozMon Serial Monitor & Serial Driver Documentation

## Overview

The TheseusOS kernel includes an interactive debugging console inspired by the legendary **Wozmon** (the Apple I monitor written by Steve Wozniak). This modern implementation provides powerful debugging capabilities through the COM1 serial port, enabling developers to inspect memory, examine registers, manipulate system state, and control kernel execution from a remote terminal.

## Architecture

### Component Overview

The serial monitor system consists of four main components:

1. **Kernel Monitor** (`kernel/src/monitor.rs`) - Interactive command processor
2. **Serial Driver** (`kernel/src/drivers/serial.rs`) - Hardware-level UART driver
3. **Serial Debug Utilities** (`kernel/src/serial_debug.rs`) - Testing utilities
4. **Bootloader Serial** (`bootloader/src/serial.rs`) - UEFI-level serial support

### Data Flow

```
User Terminal (e.g., minicom, screen)
         ↕
    COM1 Serial Port (Hardware)
         ↕
    Serial Driver (IRQ-based reception)
         ↕
    RX Buffer (1024 bytes circular)
         ↕
    Monitor Input Handler
         ↕
    Command Processor
         ↕
    Command Execution
         ↕
    Serial Output (via driver)
```

## Serial Driver (`kernel/src/drivers/serial.rs`)

### Hardware Initialization

The serial driver configures the 16550 UART on COM1 (I/O base 0x3F8) with:

- **Baud Rate**: 115200 (divisor = 1)
- **Data Format**: 8 data bits, no parity, 1 stop bit (8N1)
- **FIFO**: Enabled with 14-byte trigger threshold
- **Interrupts**: Receive Data Available (RDA) enabled

### Register Map

| Port Offset | Register | Purpose |
|-------------|----------|---------|
| +0 | DATA (DLAB=0) | Transmit/Receive buffer |
| +0 | DIVISOR_LO (DLAB=1) | Baud rate divisor low byte |
| +1 | INT_ENABLE (DLAB=0) | Interrupt enable register |
| +1 | DIVISOR_HI (DLAB=1) | Baud rate divisor high byte |
| +2 | FIFO_CTRL | FIFO control register |
| +3 | LINE_CTRL | Line control (DLAB bit) |
| +4 | MODEM_CTRL | Modem control (OUT2 required for IRQs) |
| +5 | LINE_STATUS | Line status register |

### Interrupt Handling

The serial driver uses IRQ 4 (GSI 4 on legacy systems) to receive data asynchronously:

1. Hardware asserts IRQ when data arrives
2. IOAPIC routes IRQ 4 to vector 0x41 (SERIAL_RX_VECTOR)
3. Interrupt handler reads all available bytes from UART
4. Bytes are enqueued into circular buffer
5. Monitor is notified of new data
6. EOI is sent to LAPIC

### Circular Buffer Management

The driver maintains a 1024-byte circular buffer for received data:

- **Head**: Write position (atomic, modified by IRQ handler)
- **Tail**: Read position (atomic, modified by application)
- **Overflow Behavior**: When full, oldest byte is overwritten
- **Thread Safety**: `spin::Mutex` protects buffer storage; atomic head/tail indices coordinate producer/consumer

### Key Functions

#### `register_serial_driver()`
Registers the serial driver with the kernel driver manager.

#### `init_serial()`
Initializes the serial subsystem:
- Checks configuration flags
- Registers driver with the manager
- Relies on driver system enumeration to discover and bind COM1 devices

#### `write_bytes_direct(buf: &[u8])`
Direct write path bypassing driver manager (useful in IRQ context):
- Polls Line Status Register (LSR) bit 5 (Transmit Holding Register Empty)
- Converts LF to CRLF for terminal compatibility
- Returns number of bytes written

#### `irq_handler()`
Called when serial IRQ fires:
- Reads all available bytes from UART
- Enqueues bytes into circular buffer
- Notifies monitor of new input
- Returns true if IRQ was handled

## Monitor (`kernel/src/monitor.rs`)

### Design Philosophy

The monitor is inspired by Wozmon's simplicity but extended for modern x86-64 debugging:

- **Interactive**: Responds to user commands over serial
- **Non-Intrusive**: Can be enabled/disabled at compile time
- **Powerful**: Direct memory access, register inspection, system control
- **Safe-ish**: Some commands (like `call`) are dangerous by design

### State Management

The monitor maintains:
- **Line Buffer**: Current input line (max 128 chars)
- **Last Address**: For memory examination continuation
- **Active Flag**: Tracks whether banner has been shown
- **Serial Class**: DeviceClass::Serial for driver communication

### Command Parser

The command parser splits input on whitespace and dispatches to handlers:

```rust
match parts[0] {
    "help" | "?" => self.cmd_help(),
    "regs" | "r" => self.cmd_registers(),
    "mem" | "m" => self.cmd_memory(&parts[1..]),
    // ... more commands
}
```

### Input Handling

Special key sequences:
- **Enter** (CR/LF): Process command
- **Backspace** (0x08/0x7F): Delete character (echoes "\b \b")
- **Ctrl+C** (0x03): Cancel current line
- **Ctrl+L** (0x0C): Clear screen (ANSI escape sequence)

### Available Commands

#### Memory Operations

**`mem ADDR`**  
Examine 16 bytes at address. Subsequent calls without address continue from last position.

```
> mem 0xFFFF800000100000
FFFF800000100000: 48 89 5C 24 08 48 89 6C 24 10 48 89 74 24 18 57  |H.\$.H.l$.H.t$.W|
```

**`dump ADDR [LENGTH]`**  
Hex dump a memory region (default 256 bytes).

**`write ADDR VALUE`**  
Write a single byte to memory.

**`fill ADDR LENGTH VALUE`**  
Fill a memory region with a byte value.

#### System Information

**`regs` / `r`**  
Display CPU registers:
- General purpose: RAX–R15, RSP, RBP
- Instruction pointer and flags: RIP, RFLAGS
- Control: CR0, CR2, CR3, CR4, CR8
- Segments: CS, DS, SS, ES, FS, GS

**`devices` / `dev`**  
List all registered devices with their class, binding status, MMIO address, and IRQ.

**`acpi`**  
Display ACPI information pulled from the handoff and cached tables:
- RSDP signature, OEM ID, checksum status, RSDT/XSDT pointers
- Platform summary (CPU count, IO APICs, local APIC base, legacy PIC)
- MADT details (APIC IDs, IO APIC entries) when available

**`mmap [summary|entries [N]|entry INDEX]`**  
Show UEFI memory map summary and descriptor details. By default prints a summary plus the first few entries. Use `mmap entries` to list all descriptors or `mmap entry 5` for a specific one.

**`cpuid`**  
Display CPU identification and features:
- Vendor/brand strings, family/model/stepping, APIC information
- Standard feature flags (FPU, PAE, SSE family, AVX, x2APIC, hypervisor, etc.)
- Extended flags (FSGSBASE, BMI, AVX2, SMEP/SMAP, RDSEED, RTM, NX, LM, RDTSCP, 1GiB pages)

#### System Tables

**`idt [N]`**  
Show IDT base/limit and dump the first *N* descriptors (default 16) including selector, target address, gate type, DPL, and IST slot.

**`gdt [N]`**  
Show GDT base/limit and dump segment descriptors (default all). Includes base, limit, descriptor type, privilege level, presence, and flags. Long system descriptors automatically consume both entries.

#### Debugging

**`stack` / `bt`**  
Walk the stack frame chain and display return addresses:
```
Frame # 0: RIP=0xFFFF800000102ABC RBP=0xFFFF800000205E40
Frame # 1: RIP=0xFFFF800000101234 RBP=0xFFFF800000205E80
...
```

**`msr [r|w] ADDR [VALUE]`**  
Read or write Model-Specific Registers. `msr 0x1B` reads IA32_APIC_BASE, `msr w 0x1B 0x...` writes a new value (dangerous).

Common MSRs:
- 0x1B: IA32_APIC_BASE
- 0x10: IA32_TIME_STAMP_COUNTER
- 0xC0000080: IA32_EFER

**`io (r|w)[8|16|32] PORT [VALUE]`**  
Read/write I/O ports at byte, word, or dword granularity, e.g. `io r16 0x64`, `io w32 0xCF8 0x80000010`.

**`int NUM`**  
Trigger software interrupt (INT3 breakpoint or INT 0x80 syscall stub allowed for safety).

**`call ADDR`**  
Call a function at the specified address (⚠️ DANGEROUS - can crash system).

#### System Control

**`reset`**  
Reset the system via keyboard controller or triple fault.

**`halt`**  
Halt the CPU (requires reset to recover).

**`clear` / `cls`**  
Clear the screen using ANSI escape sequences.

### Number Parsing

The monitor supports both hex and decimal input:
- Hex: `0x1234ABCD` or `0X1234abcd`
- Decimal: `1234567890`

### Safety Considerations

The monitor operates in kernel mode with full privileges:

✅ **Safe Commands**: `help`, `regs`, `mem`, `dump`, `devices`, `acpi`, `cpuid`, `idt`, `gdt`, `mmap`, `stack`, `msr`, `io r`

⚠️ **Potentially Dangerous**: `write`, `fill`, `io w` - can corrupt system state

🔴 **Very Dangerous**: `call` - arbitrary code execution, `int` - can trigger faults, `reset` - forces reboot

## Serial Debug Utilities (`kernel/src/serial_debug.rs`)

### Reverse Echo Session

A simple test mode that echoes each line back in reverse order:

```
Input:  hello world
Output: dlrow olleh
```

This is useful for:
- Testing serial IRQ functionality
- Verifying bidirectional communication
- Debugging early serial initialization

Enable via `config::RUN_POST_BOOT_SERIAL_REVERSE_ECHO = true` (blocks kernel progress).

## Bootloader Serial (`bootloader/src/serial.rs`)

### UEFI Serial Protocol

The bootloader uses UEFI's Serial protocol for early logging:

- **`serial_write(handle, data)`**: Write raw bytes
- **`serial_write_line(handle, line)`**: Write line with CRLF

This is separate from the kernel driver since UEFI services are not available after ExitBootServices().

## Configuration

All serial and monitor features can be toggled in `kernel/src/config.rs`:

```rust
/// Enable serial output to COM1
pub const ENABLE_SERIAL_OUTPUT: bool = true;

/// Activate the interactive kernel monitor on COM1
pub const ENABLE_KERNEL_MONITOR: bool = true;

/// Run the reverse-echo test loop
pub const RUN_POST_BOOT_SERIAL_REVERSE_ECHO: bool = false;
```

## Driver Framework Integration

The serial driver integrates with the kernel's generic driver framework:

### Device Registration

```rust
Device {
    id: DeviceId::Class(DeviceClass::Serial),
    class: DeviceClass::Serial,
    phys_addr: Some(0x3F8),  // COM1 I/O base
    irq: Some(4),             // Legacy IRQ 4
    driver_data: Some(ptr),   // SerialDriverState pointer
}
```

### Driver Lifecycle

1. **Registration**: `register_driver()` adds driver to manager
2. **Probe**: Manager calls `probe()` to check device compatibility
3. **Init**: `init()` configures hardware and enables interrupts
4. **IRQ**: `irq_handler()` called when interrupt fires
5. **I/O**: `read()`/`write()` provide character device interface

### Class-Based I/O

The driver manager provides class-based I/O operations:

```rust
// Write to any Serial class device
driver_manager().lock().write_class(DeviceClass::Serial, b"Hello\n");

// Read from any Serial class device
let mut buf = [0u8; 64];
let n = driver_manager().lock().read_class(DeviceClass::Serial, &mut buf)?;
```

## Usage Examples

### Basic Memory Examination

```
> mem 0xFFFF800000100000
FFFF800000100000: 48 89 5C 24 08 48 89 6C 24 10 48 89 74 24 18 57  |H.\$.H.l$.H.t$.W|
> mem
FFFF800000100010: 48 83 EC 20 48 8B E9 48 8D 4C 24 30 E8 A4 FE FF  |H.. H..H.L$0....|
```

### Register Inspection

```
> regs
CPU Registers:
  RAX: 0x0000000000000001  RBX: 0xFFFF800000205E40
  RCX: 0x0000000000000000  RDX: 0x00000000000003F8
  RSI: 0x0000000000000000  RDI: 0xFFFF800000300000
  RSP: 0xFFFF800000205DE8  RBP: 0xFFFF800000205E10

Control Registers:
  CR0: 0x0000000080050033  CR2: 0x0000000000000000
  CR3: 0x000000000010A000  CR4: 0x00000000000006A0

Segment Registers:
  CS: 0x0008  DS: 0x0010  SS: 0x0010  ES: 0x0010
```

### Device Enumeration

```
> devices
Registered devices:
  [ 0] CLASS:Serial [BOUND] addr:0x3F8 irq:4
```

### Stack Trace

```
> stack
Stack trace:
  Frame # 0: RIP=0xFFFF800000102ABC RBP=0xFFFF800000205E40
  Frame # 1: RIP=0xFFFF800000101234 RBP=0xFFFF800000205E80
  Frame # 2: RIP=0xFFFF800000100987 RBP=0xFFFF800000205EC0
```

## Accessing the Monitor

### QEMU with stdio

```bash
qemu-system-x86_64 \
    -serial stdio \
    -drive if=pflash,format=raw,file=OVMF_CODE.fd,readonly=on \
    -drive if=pflash,format=raw,file=OVMF_VARS.fd \
    -drive format=raw,file=build/disk.img
```

### QEMU with Unix Socket

```bash
qemu-system-x86_64 \
    -serial unix:/tmp/qemu-serial.sock,server,nowait \
    ...

# In another terminal
socat UNIX-CONNECT:/tmp/qemu-serial.sock -,raw,echo=0
```

### Physical Hardware

Use a null modem cable or USB-to-serial adapter:

```bash
minicom -D /dev/ttyS0 -b 115200
# or
screen /dev/ttyS0 115200
```

## Implementation Notes

### Why Wozmon?

The Wozmon monitor (1976) was revolutionary in its simplicity:
- Only ~256 bytes of code
- Simple command syntax
- Direct memory access
- Efficient use of limited resources

Our implementation honors this philosophy while adding modern necessities:
- 64-bit addressing
- x86-64 system registers
- ACPI/UEFI integration
- Interrupt-driven I/O

### Thread Safety

The monitor uses several synchronization mechanisms:

1. **MONITOR** static: Spin mutex protecting monitor state
2. **SERIAL_STATE**: Spin mutex protecting driver state
3. **Circular Buffer**: `spin::Mutex` guards data; atomic head/tail indices track positions
4. **IRQ Context**: Uses `write_bytes_direct()` to avoid reentrancy

### Performance Considerations

- **Polling vs Interrupts**: The driver uses IRQ-based reception for efficiency
- **Buffering**: 1024-byte buffer smooths out burst traffic
- **Direct Write**: Bypasses buffer for immediate output
- **FIFO**: Hardware FIFO reduces interrupt frequency

### Future Enhancements

Potential improvements:
- Command history (up/down arrows)
- Tab completion
- Scripting support
- Breakpoint support
- Memory watchpoints
- Multi-core inspection
- ACPI table parsing and display
- PCI configuration space access
- Disk sector viewer

## Troubleshooting

### Monitor Not Responding

1. Check `config::ENABLE_KERNEL_MONITOR` is `true`
2. Verify serial output is enabled (`config::ENABLE_SERIAL_OUTPUT`)
3. Ensure QEMU serial is configured correctly
4. Try sending Ctrl+L to refresh screen

### Garbled Output

1. Verify baud rate matches (115200)
2. Check terminal settings (8N1, no flow control)
3. Ensure CRLF handling is correct

### IRQ Not Firing

1. Check IOAPIC configuration in kernel logs
2. Verify OUT2 bit is set in modem control register
3. Ensure interrupts are enabled globally (IF flag)
4. Check if IRQ 4 is masked in IOAPIC

### System Hangs After Command

Some commands (especially `halt`, `reset`, `call`) are designed to be destructive. Use with caution!

## References

- [Wozmon Original Source](https://www.sbprojects.net/projects/apple1/wozmon.php)
- [16550 UART Datasheet](http://caro.su/msx/ocm_de1/16550.pdf)
- [Intel 64 and IA-32 Architectures Software Developer's Manual](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html)
- [OSDev Wiki - Serial Ports](https://wiki.osdev.org/Serial_Ports)

## License

This documentation and the TheseusOS kernel monitor are released under the same license as the TheseusOS project.
