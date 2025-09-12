# Driver Output Configuration Guide

This document explains how to configure QEMU to capture output from the different drivers in the HobbyOS UEFI loader.

## Driver Types and Output Destinations

### 1. QEMU Debug Driver (Default)
- **What it does**: Writes directly to QEMU debug port (0xe9) using I/O ports
- **When it's active**: Always (default driver for QEMU targets)
- **Output destination**: QEMU debug console (stdout in headless, `debug.log` in headed)
- **Use case**: QEMU development and testing (simplest option)

### 2. UEFI Serial Driver
- **What it does**: Uses UEFI Serial I/O protocol
- **When it's active**: Available during boot services (not used by default)
- **Output destination**: UEFI serial console (QEMU's `-serial` option)
- **Use case**: Real hardware UEFI systems

### 3. Raw Serial Driver
- **What it does**: Direct hardware access to COM1 serial port (0x3f8)
- **When it's active**: If not on QEMU and after exiting boot services
- **Output destination**: COM1 serial port (real hardware)
- **Use case**: Real hardware after boot services exit

## Current QEMU Configuration

The current `startQemu.sh` script configures serial output as follows:

### Headless Mode (Default)
```bash
QEMU_SERIAL="-serial stdio"
QEMU_DEBUG="-device isa-debugcon,chardev=debugcon"
QEMU_DEBUG_CHAR="-chardev file,id=debugcon,path=debug.log"
```
- **UEFI Serial Driver**: ✅ Output goes to stdout
- **QEMU Debug Driver**: ✅ Output goes to `debug.log`
- **Raw Serial Driver**: ✅ Available as fallback (COM1)

### Headed Mode
```bash
QEMU_SERIAL="-serial file:serial.log"
QEMU_DEBUG="-device isa-debugcon,chardev=debugcon"
QEMU_DEBUG_CHAR="-chardev file,id=debugcon,path=debug.log"
```
- **UEFI Serial Driver**: ✅ Output goes to `serial.log`
- **QEMU Debug Driver**: ✅ Output goes to `debug.log`
- **Raw Serial Driver**: ✅ Available as fallback (COM1)

## ✅ All Drivers Now Working!

All three drivers are now properly configured and working:

1. **UEFI Serial Driver**: Works during boot services via UEFI Serial I/O protocol
2. **QEMU Debug Driver**: Works after boot services via port 0xe9 (QEMU-specific)
3. **Raw Serial Driver**: Available as fallback via COM1 (real hardware)

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
