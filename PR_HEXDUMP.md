# Add Hexdump Utility for Kernel Debugging

## Summary
This PR adds a hexdump utility function to the kernel's `src/util/mod.rs` for debugging memory regions. The utility provides allocation-free hex+ASCII memory dumps that integrate with the existing logging system.

## Changes

### Added
- `kernel/src/util/mod.rs`: Main hexdump implementation
- `kernel/src/util/test_hexdump.rs`: Test suite for hexdump functionality  
- `kernel/src/util/example.rs`: Usage examples
- `kernel/src/util/README.md`: Documentation
- Updated `kernel/src/lib.rs` to include the new util module

### Features
1. **Allocation-free implementation**: Uses stack buffers only (safe for panic/interrupt contexts)
2. **Hex + ASCII display**: Shows both hexadecimal bytes and ASCII representation
3. **Safe wrappers**: `hexdump_safe()` and `hexdump_labeled_safe()` with pointer validation
4. **Flexible addressing**: Support for custom base addresses in output
5. **Labeled dumps**: Add descriptive labels to hexdump output
6. **Pointer validation**: `ptr_likely_valid()` helper function

## Usage Examples

### Basic hexdump
```rust
use crate::util;

let data = b"Hello, TheseusOS!";
unsafe {
    util::hexdump(data.as_ptr(), data.len(), None);
}
```

### Safe wrapper with validation
```rust
use crate::util;

let data = [0xDEu8, 0xAD, 0xBE, 0xEF];
match util::hexdump_safe(data.as_ptr(), data.len(), None) {
    Ok(()) => log_info!("Dump successful"),
    Err(e) => log_error!("Dump failed: {}", e),
}
```

### Labeled dump
```rust
use crate::util;

let buffer = [0u8; 32];
unsafe {
    util::hexdump_labeled("Zero buffer", buffer.as_ptr(), buffer.len(), Some(0x1000));
}
```

## Safety
- **Memory validity**: Caller must ensure memory region is valid and readable
- **No allocation**: Uses stack buffers only (max 512 bytes)
- **Interrupt safety**: Safe for interrupt handlers (no locks, no heap)
- **Null pointers**: Safe wrappers validate for null pointers

## Testing
The implementation includes a comprehensive test suite that verifies:
- Basic hexdump functionality
- Safe wrapper validation
- Pointer validation logic  
- Edge cases (zero length, null pointers, overflow)

## Output Format
```
00001000: 48 65 6C 6C 6F 2C 20 54  68 65 73 65 75 73 4F 53  |Hello, TheseusOS|
00001010: 21 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |!...............|
```

## Code Standards
- Follows TheseusOS Rust 2021, no_std conventions
- Integrates with existing `log_trace!` macros
- Proper error handling with Result types
- Comprehensive documentation and examples

This utility will be invaluable for kernel debugging, memory inspection, and driver development.