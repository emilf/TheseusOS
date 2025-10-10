# COM1 Serial Terminal Driver

This document describes the COM1 serial terminal driver implementation in TheseusOS, which provides character-device access to the COM1 serial port (UART 16550).

## Overview

The serial driver is a fully-featured character device driver that integrates with the kernel's driver framework. It provides read and write operations for serial communication and serves as an example of proper driver implementation.

## Architecture

### Components

- **`kernel/src/drivers/serial.rs`**: Main driver implementation
- **`kernel/src/drivers/manager.rs`**: Enhanced with device lookup and I/O methods
- **`kernel/src/drivers/traits.rs`**: Driver trait definitions
- **`shared/src/constants.rs`**: COM1 hardware constants

### Hardware

- **Base I/O Port**: `0x3F8` (COM1)
- **IRQ**: 4
- **Configuration**: 8-N-1 (8 data bits, no parity, 1 stop bit)
- **Baud Rate**: 115200 (standard for QEMU and modern systems)
- **FIFO**: Enabled with 14-byte threshold

### Registers

```rust
pub mod com1 {
    pub const DATA: u16 = 0x3f8;           // Data register (R/W)
    pub const INT_ENABLE: u16 = 0x3f9;     // Interrupt enable (R/W)
    pub const LINE_CTRL: u16 = 0x3fb;      // Line control (R/W)
    pub const MODEM_CTRL: u16 = 0x3fc;     // Modem control (R/W)
    pub const LINE_STATUS: u16 = 0x3fd;    // Line status (RO)
}
```

## Driver Lifecycle

### 1. Registration

```rust
// During kernel initialization
drivers::serial::register_serial_driver();
drivers::serial::register_com1_device();
```

The driver registers itself with the driver manager and creates a COM1 device descriptor.

### 2. Probing

When the device is added, the driver manager calls `probe()` on all registered drivers:

```rust
fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str> {
    match &dev.id {
        DeviceId::Raw(name) if *name == "com1" => {
            // Initialize hardware
            let mut state = self.state.lock();
            if !state.init() {
                return Err("Failed to initialize COM1");
            }
            
            // Mark device as bound
            dev.driver_data = Some(0xC041); // Magic value "COM1"
            Ok(())
        }
        _ => Err("Not a COM1 device"),
    }
}
```

### 3. Initialization Sequence

The hardware initialization follows the standard 16550 UART procedure:

1. **Disable Interrupts**: `int_enable.write(0x00)`
2. **Enable DLAB**: `line_ctrl.write(0x80)` (to set baud rate)
3. **Set Baud Rate**: Divisor = 1 for 115200 baud
   - `data.write(0x01)` - Low byte
   - `int_enable.write(0x00)` - High byte
4. **Configure Line**: `line_ctrl.write(0x03)` (8-N-1)
5. **Enable FIFO**: `int_enable.write(0xC7)` (14-byte threshold)
6. **Set Modem Control**: `modem_ctrl.write(0x0B)` (RTS/DSR)
7. **Loopback Test**: Verify hardware responds correctly
8. **Normal Mode**: `modem_ctrl.write(0x0F)` (IRQs enabled)

### 4. Operation

Once initialized, the driver provides:

#### Write Operations

```rust
// Through driver manager
let com1_id = DeviceId::Raw("com1");
driver_manager()
    .lock()
    .write_to_device(&com1_id, b"Hello, World!\r\n")?;

// Direct access (if needed)
SERIAL_DRIVER.write_str("Debug message\r\n");
```

#### Read Operations

```rust
let mut buffer = [0u8; 256];
let bytes_read = driver_manager()
    .lock()
    .read_from_device(&com1_id, &mut buffer)?;
```

## Driver Framework Improvements

The serial driver implementation led to several enhancements to the driver framework:

### Device Lookup

```rust
// Find a device by ID
pub fn find_device(&self, id: &DeviceId) -> Option<&Device>;
pub fn find_device_mut(&mut self, id: &DeviceId) -> Option<&mut Device>;
```

### High-Level I/O Operations

```rust
// Write to a device without directly calling driver methods
pub fn write_to_device(&mut self, id: &DeviceId, buf: &[u8]) 
    -> Result<usize, &'static str>;

// Read from a device without directly calling driver methods
pub fn read_from_device(&mut self, id: &DeviceId, buf: &mut [u8]) 
    -> Result<usize, &'static str>;
```

### Driver Discovery

```rust
// Get the driver bound to a specific device
pub fn get_driver_for_device(&self, id: &DeviceId) 
    -> Option<&'static dyn Driver>;
```

## Testing

### Test Output

The driver is tested during kernel initialization:

```
[serial] registering COM1 driver
[serial] COM1 driver registering
[driver] registering driver
[serial] registering COM1 device
[driver] discovered device
[serial] probing COM1 device
[serial] COM1 initialized successfully
[serial] COM1 ready for I/O
[driver] device bound successfully
[driver] testing serial driver
[driver] serial test: wrote 0x1C bytes to COM1
[driver] serial test: second write successful
```

### COM1 Output

The actual serial output can be captured in QEMU:

```bash
# Run with COM1 output to file
./startQemu.sh headless

# Check the output
cat com1.log
```

Output:
```
COM1 Serial Driver Initialized
Serial driver test message
Second test message: Hello from TheseusOS!
```

## QEMU Configuration

### Serial Port Setup

The `startQemu.sh` script configures QEMU to capture COM1 output:

```bash
# Headless mode: COM1 → com1.log
QEMU_DISPLAY_ARGS=( -display none -serial file:com1.log )

# Alternative: COM1 → stdio
QEMU_DISPLAY_ARGS=( -display none -serial stdio )

# Alternative: COM1 → TCP socket (for remote access)
QEMU_DISPLAY_ARGS=( -display none -serial tcp:127.0.0.1:4444,server,nowait )
```

### Dual Serial Configuration

QEMU supports multiple serial ports:

```bash
# Port 0 (UEFI/bootloader) → stdio
# Port 1 (COM1/kernel) → com1.log
-serial stdio -serial file:com1.log
```

## Usage Examples

### Basic Write

```rust
use crate::drivers::manager::driver_manager;
use crate::drivers::traits::DeviceId;

let com1 = DeviceId::Raw("com1");
driver_manager()
    .lock()
    .write_to_device(&com1, b"Message\r\n")?;
```

### Formatted Output

```rust
use alloc::format;

let message = format!("Counter: {}\r\n", 42);
driver_manager()
    .lock()
    .write_to_device(&com1, message.as_bytes())?;
```

### Read with Timeout

```rust
let mut buffer = [0u8; 128];
let mut total = 0;
let mut attempts = 0;

while total < buffer.len() && attempts < 100 {
    match driver_manager().lock().read_from_device(&com1, &mut buffer[total..]) {
        Ok(n) if n > 0 => {
            total += n;
            attempts = 0;
        }
        Ok(_) => {
            attempts += 1;
            // Small delay
            for _ in 0..1000 { core::hint::spin_loop(); }
        }
        Err(_) => break,
    }
}
```

## Implementation Details

### Thread Safety

The driver uses a `Mutex<SerialState>` to protect concurrent access:

```rust
pub struct SerialDriver {
    state: Mutex<SerialState>,
}
```

This ensures that multiple kernel threads can safely use the serial port without data corruption.

### Transmit Buffering

Before each write, the driver waits for the transmit buffer to be empty:

```rust
fn write_byte(&mut self, byte: u8) {
    unsafe {
        // Wait for transmit buffer to be empty (bit 5 of LSR)
        while !self.is_transmit_empty() {
            core::hint::spin_loop();
        }
        self.data.write(byte);
    }
}
```

### Line Endings

The driver automatically converts `\n` to `\r\n` for proper terminal display:

```rust
pub fn write_str(&'static self, s: &str) {
    for byte in s.as_bytes() {
        if *byte == b'\n' {
            state.write_byte(b'\r');
        }
        state.write_byte(*byte);
    }
}
```

### Non-Blocking Reads

The read operation returns immediately if no data is available:

```rust
fn read_byte(&mut self) -> Option<u8> {
    if self.data_available() {
        unsafe { Some(self.data.read()) }
    } else {
        None
    }
}
```

## Interrupt Support

The driver includes basic IRQ support:

```rust
fn irq_handler(&'static self, dev: &mut Device, irq: u32) -> bool {
    if dev.driver_data == Some(0xC041) && irq == 4 {
        // Handle serial interrupt
        true
    } else {
        false
    }
}
```

Currently, interrupts are disabled during initialization. Future enhancements could:
- Enable receive interrupts for asynchronous I/O
- Implement interrupt-driven transmit
- Add DMA support for bulk transfers

## Future Enhancements

### Planned Features

1. **Buffered I/O**: Add ring buffers for transmit and receive
2. **Async Operations**: Integrate with async runtime when available
3. **Flow Control**: Implement hardware (RTS/CTS) and software (XON/XOFF) flow control
4. **Multiple Ports**: Support COM2-COM4
5. **Modem Control**: Full modem signal support (DTR, DSR, RI, DCD)
6. **Terminal Emulation**: VT100/ANSI escape sequence support

### Performance Optimizations

- DMA transfers for bulk data
- Interrupt-driven I/O
- Transmit FIFO optimization
- Receive buffering

## Troubleshooting

### Common Issues

**Problem**: No output in `com1.log`

**Solution**: Check that:
- The driver initializes successfully (check kernel logs)
- QEMU is configured with `-serial file:com1.log`
- The device is bound (`driver_data != None`)

**Problem**: Garbled output

**Solution**: Verify:
- Baud rate matches (115200)
- Line configuration is 8-N-1
- No interrupt conflicts

**Problem**: Driver fails to initialize

**Solution**: Check:
- COM1 hardware is present in QEMU
- Port addresses are correct (0x3F8)
- No other driver is using COM1

## References

- [16550 UART Datasheet](https://www.ti.com/lit/ds/symlink/pc16550d.pdf)
- [OSDev Serial Ports](https://wiki.osdev.org/Serial_Ports)
- [QEMU Serial Documentation](https://www.qemu.org/docs/master/system/device-emulation.html#serial-ports)
- TheseusOS Driver Framework Documentation

## Example: Simple Console

Here's how to build a simple serial console:

```rust
pub fn serial_console_loop() {
    let com1 = DeviceId::Raw("com1");
    let mut buffer = [0u8; 1];
    
    loop {
        // Read one character
        if let Ok(1) = driver_manager().lock().read_from_device(&com1, &mut buffer) {
            match buffer[0] {
                b'\r' | b'\n' => {
                    // Echo newline
                    driver_manager().lock().write_to_device(&com1, b"\r\n").ok();
                }
                0x03 => {
                    // Ctrl+C - exit
                    break;
                }
                byte if byte >= 0x20 && byte < 0x7F => {
                    // Printable character - echo it
                    driver_manager().lock().write_to_device(&com1, &[byte]).ok();
                }
                _ => {
                    // Non-printable - ignore
                }
            }
        }
    }
}
```

This creates a simple echo console that reads from COM1 and echoes back characters, handling special keys like Enter and Ctrl+C.

