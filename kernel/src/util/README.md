# Hexdump Utility for TheseusOS

This module provides a hexdump utility for debugging memory regions in the TheseusOS kernel.

## Features

- **Allocation-free**: Uses stack buffers only, safe for panic/interrupt contexts
- **Hex + ASCII display**: Shows both hexadecimal bytes and ASCII representation
- **Safe wrappers**: Validation functions to check pointers before dumping
- **Flexible addressing**: Support for custom base addresses in output
- **Labeled dumps**: Add descriptive labels to hexdump output

## Usage Examples

### Basic hexdump
```rust
use crate::util;

let data = b\"Hello, TheseusOS!\";
unsafe {
    util::hexdump(data.as_ptr(), data.len(), None);
}
```

### Safe wrapper with validation
```rust
use crate::util;

let data = [0xDEu8, 0xAD, 0xBE, 0xEF];
match util::hexdump_safe(data.as_ptr(), data.len(), None) {
    Ok(()) => log_info!(\"Dump successful\"),
    Err(e) => log_error!(\"Dump failed: {}\", e),
}
```

### Labeled dump with custom base address
```rust
use crate::util;

let buffer = [0u8; 32];
unsafe {
    util::hexdump_labeled(\"Zero-initialized buffer\", 
                         buffer.as_ptr(), 
                         buffer.len(), 
                         Some(0x1000));
}
```

### Pointer validation
```rust
use crate::util;

if util::ptr_likely_valid(ptr, len) {
    // Safe to use the pointer
} else {
    log_warn!(\"Pointer validation failed\");
}
```

## Safety Considerations

1. **Memory validity**: The caller must ensure the memory region is valid and readable
2. **No allocation**: Functions use stack buffers only (max 512 bytes)
3. **Interrupt safety**: Safe to use in interrupt handlers (no locks, no heap)
4. **Null pointers**: Safe wrappers validate for null pointers

## Output Format

The hexdump output follows this format:
```
00001000: 48 65 6C 6C 6F 2C 20 54  68 65 73 65 75 73 4F 53  |Hello, TheseusOS|
00001010: 21 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |!...............|
```

- **Offset**: 8-digit hexadecimal address (customizable with `base_addr`)
- **Hex bytes**: 16 bytes per line, space between groups of 8
- **ASCII**: Printable characters shown as-is, others as '.'

## Testing

Run the test suite with:
```bash
cargo test --package theseus-kernel --lib util
```

The tests verify:
- Basic hexdump functionality
- Safe wrapper validation
- Pointer validation logic
- Edge cases (zero length, null pointers)