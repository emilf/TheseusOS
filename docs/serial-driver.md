# Serial Driver Integration

This document summarizes the initial serial terminal driver built on the kernel driver framework.

## Overview
- Implements a lightweight UART driver (`kernel/src/drivers/serial.rs`) bound to devices classified as `DeviceClass::Serial`.
- Uses the driver manager to probe, initialize, and expose character output via `Driver::write`.
- Adds a shared `DeviceClass` enum and class-based matching in the driver framework allowing drivers to express supported device categories.

## Driver Registration Flow
1. `drivers::system::init()` registers the serial driver before enumerating hardware inventory.
2. Bootloader hardware inventory marks UART devices with `DEVICE_TYPE_SERIAL`, which maps to `DeviceId::Class(DeviceClass::Serial)` in the kernel.
3. Driver manager probes the serial driver for serial-class devices and calls the driver's `init` method, which programs the COM1 UART.

## Driver Capabilities
- Configurable via `config::ENABLE_SERIAL_OUTPUT` flag.
- Initializes port 0x3F8 (COM1) with 115200 baud, 8-N-1, FIFO enabled.
- Provides buffered write support that converts `\n` into CRLF for terminal compatibility.

## Driver Framework Enhancements
- Introduced `DeviceClass` enum and `DeviceId::Class` variant.
- Driver trait now declares `supported_classes()` and optional `init()` hook.
- Driver manager filters probes by class, calls `init` on the bound driver, and exposes `write_class()` for subsystems (e.g., display) to deliver output to class-matched devices.

## Using the Serial Driver
- Call `drivers::serial::init_serial()` (handled automatically during driver system initialization).
- Use helper `display::kernel_write_serial("message")` to send data through the framework to the serial terminal.

## Future Work
- Add interrupt-driven RX/TX support.
- Expose a higher-level console abstraction with buffering.
- Extend bootloader classification to include additional UART variants.
