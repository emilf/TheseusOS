# Driver Output Configuration Guide

This document explains how TheseusOS displays information during boot and how to see that output.

## What are "Drivers"?

In operating systems, a **driver** is a piece of code that knows how to talk to a specific piece of hardware. For displaying text, we need drivers that know how to send text to different output devices.

## Why Do We Need Different Drivers?

Different environments need different ways to display text:

1. **During Development**: We use QEMU (a computer simulator), so we need a driver that works with QEMU
2. **On Real Hardware**: We need drivers that work with actual computer hardware
3. **Different Phases**: Some drivers work during UEFI boot, others work after the kernel takes over

## Driver Types and Where They Send Output

### 1. QEMU Debug Driver (Default - What We Use Most)
- **What it does**: Sends text directly to QEMU's debug port
- **When it works**: Always (this is our main driver for development)
- **Where you see output**: In your terminal when running QEMU
- **Why it's good**: Simple and works great for learning and development

### 2. UEFI Serial Driver (For Real Hardware)
- **What it does**: Uses UEFI's built-in text output system
- **When it works**: During the bootloader phase on real hardware
- **Where you see output**: On the computer's serial port (like an old modem port)
- **Why it's useful**: Works on real computers, not just simulators

### 3. Raw Serial Driver (Backup for Real Hardware)
- **What it does**: Talks directly to the computer's serial port hardware
- **When it works**: After the kernel takes over on real hardware
- **Where you see output**: On the computer's serial port
- **Why it's needed**: Some systems don't have UEFI serial support

## How to See the Output

### Simple Way (Recommended for Learning)
Just run TheseusOS and you'll see all the output in your terminal:

```bash
# Run in headless mode (no graphics window)
make run

# Run with graphics window
make run-headed
```

### What You'll See
- **Bootloader output**: Information about system hardware, memory, etc.
- **Kernel output**: Messages from the operating system as it starts up
- **Debug information**: Technical details about what's happening

### Advanced: Multiple Output Streams
If you want to see output from different drivers separately:

```bash
# Terminal 1: Start TheseusOS
make run

# Terminal 2: Watch debug output (if using headed mode)
tail -f debug.log
```

## Current Status

✅ **Everything Working**: All output drivers are properly configured:
1. **QEMU Debug Driver**: Main driver for development (shows in terminal)
2. **UEFI Serial Driver**: Works during bootloader phase
3. **Raw Serial Driver**: Available as backup for real hardware

## Solutions

### Option 1: Configure COM1 Serial Port (Recommended)

Add a second serial port to capture COM1 output:

```bash
# Add this to QEMU command
-serial tcp:127.0.0.1:4444,server,nowait
```

**Usage:**
```bash
# Terminal 1: Start QEMU with COM1 capture
QEMU_OPTS="-serial tcp:127.0.0.1:4444,server,nowait" ./startQemu.sh headless

# Terminal 2: Connect to COM1 output
telnet 127.0.0.1 4444
```

### Option 2: Use QEMU Monitor for COM1

```bash
# Add this to QEMU command  
-serial mon:stdio
```

This sends COM1 output to the QEMU monitor (useful for debugging).

### Option 3: File Output for COM1

```bash
# Add this to QEMU command
-serial file:com1.log
```

This writes COM1 output to a file.

### Option 4: Multiple Serial Ports

```bash
# UEFI Serial (current)
-serial stdio

# COM1 for I/O Port drivers
-serial tcp:127.0.0.1:4444,server,nowait
```

## Updated QEMU Configuration

Here's how to modify `startQemu.sh` to capture all driver outputs:

### For Headless Mode:
```bash
# Current UEFI serial output
QEMU_SERIAL="-serial stdio"

# Add COM1 output for I/O Port drivers
QEMU_COM1="-serial tcp:127.0.0.1:4444,server,nowait"

# Combine both
QEMU_SERIAL_OPTS="$QEMU_SERIAL $QEMU_COM1"
```

### For Headed Mode:
```bash
# Current UEFI serial output
QEMU_SERIAL="-serial file:serial.log"

# Add COM1 output for I/O Port drivers  
QEMU_COM1="-serial file:com1.log"

# Combine both
QEMU_SERIAL_OPTS="$QEMU_SERIAL $QEMU_COM1"
```

## Testing Different Drivers

### Force I/O Port Driver (for testing)
```rust
// In main.rs, temporarily add:
output_driver.force_driver(DriverType::IoPortSerial);
```

### Monitor Both Outputs
```bash
# Terminal 1: Start QEMU
QEMU_OPTS="-serial tcp:127.0.0.1:4444,server,nowait" ./startQemu.sh headless

# Terminal 2: Monitor COM1 output
telnet 127.0.0.1 4444

# Terminal 3: Monitor UEFI serial output (if needed)
# This will show in Terminal 1's stdout
```

## Driver Selection Logic

The system automatically selects drivers based on boot services status:

1. **During Boot Services**: Uses UEFI Serial Driver
2. **After Boot Services Exit**: Switches to I/O Port Serial Driver
3. **Fallback**: Uses Raw Serial Driver if others fail

## Current Status

- ✅ **UEFI Serial Driver**: Properly configured and working
- ❌ **I/O Port Serial Driver**: Configured but output goes nowhere
- ❌ **Raw Serial Driver**: Configured but output goes nowhere

## Recommendation

Add COM1 serial port configuration to capture I/O Port and Raw Serial driver output:

```bash
# In startQemu.sh, add:
QEMU_COM1="-serial tcp:127.0.0.1:4444,server,nowait"
```

This will allow you to see output from all drivers during different phases of execution.
