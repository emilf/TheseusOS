# TheseusOS Testing Framework

This document describes the testing framework for TheseusOS, which supports running tests in different environments from bare-metal to fully initialized kernel.

## Overview

The testing framework is designed to test different parts of the OS at various initialization stages:

- **Bare-metal tests**: Run immediately after bootloader handoff (no kernel services)
- **Kernel tests**: Run after full kernel initialization (with heap, memory mapping, interrupts)
- **Panic tests**: Verify that panic handling works correctly

## Quick Start

```bash
# Run all tests
make test-all

# Run specific test environments
make test-bare-metal    # Tests with no kernel services
make test-kernel        # Tests with full kernel initialization
make test-panic         # Tests that verify panic handling

# Show available test targets
make test-help
```

## Test Environments

### Bare-Metal Tests (`test-bare-metal`)

**Environment**: Runs immediately after the bootloader hands off control to the kernel.

**Capabilities**:
- ✅ Basic CPU operations (arithmetic, bitwise, control flow)
- ✅ Stack operations and local variables
- ✅ Function calls and returns
- ❌ No heap allocation
- ❌ No kernel services
- ❌ No memory mapping
- ❌ No interrupt handling

**Use Cases**:
- Testing core CPU functionality
- Validating basic control flow
- Testing stack operations
- Verifying function call mechanisms

**Example Test**:
```rust
fn test_basic_cpu_ops() {
    // Test basic arithmetic
    assert_eq!(1 + 1, 2);
    assert_eq!(5 * 3, 15);
    
    // Test bitwise operations
    assert_eq!(0b1010 & 0b1100, 0b1000);
    assert_eq!(0b1010 | 0b1100, 0b1110);
}
```

### Kernel Tests (`test-kernel`)

**Environment**: Runs after the kernel has been fully initialized.

**Capabilities**:
- ✅ All bare-metal capabilities
- ✅ Heap allocation (`Box`, `Vec`, `String`)
- ✅ Memory mapping and page tables
- ✅ Interrupt handling
- ✅ Kernel services
- ✅ Full kernel API access

**Use Cases**:
- Testing memory management
- Testing heap allocation
- Testing kernel services
- Testing interrupt handling
- Integration testing with full kernel

**Example Test**:
```rust
fn test_heap_allocation() {
    extern crate alloc;
    use alloc::vec::Vec;
    use alloc::boxed::Box;
    
    // Test Vec allocation
    let mut vec = Vec::new();
    vec.push(1);
    vec.push(2);
    assert_eq!(vec.len(), 2);
    
    // Test Box allocation
    let boxed = Box::new(42);
    assert_eq!(*boxed, 42);
}
```

### Panic Tests (`test-panic`)

**Environment**: Minimal environment to test panic handling.

**Capabilities**:
- ✅ Panic handling verification
- ✅ Assertion testing
- ❌ Limited other functionality

**Use Cases**:
- Verifying that `assert_eq!` works correctly
- Testing panic handlers
- Ensuring proper error reporting

## Implementation Guide

### Creating New Tests

#### 1. Bare-Metal Tests

Create a new test function in `tests/bare_metal_tests.rs`:

```rust
/// Test description here
fn test_my_functionality() {
    // Print test identifier for debugging
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9",
            "mov al, 'X'",  // Use unique character
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }
    
    // Your test code here
    assert_eq!(my_function(), expected_result);
    
    // Print completion marker
    unsafe {
        core::arch::asm!(
            "mov dx, 0xe9",
            "mov al, 'Y'",  // Use unique character
            "out dx, al",
            options(nomem, nostack, preserves_flags)
        );
    }
}
```

Then add it to the test list in `kernel_main`:

```rust
#[no_mangle]
pub extern "C" fn kernel_main(_handoff_addr: u64) -> ! {
    // ... existing code ...
    
    test_my_functionality();  // Add your test here
    
    // ... existing code ...
}
```

#### 2. Kernel Tests

Create a new test function in `tests/kernel_tests.rs`:

```rust
/// Test description here
fn test_my_kernel_functionality() {
    // Use kernel logging instead of direct I/O
    theseus_kernel::display::kernel_write_line("Testing my functionality...");
    
    // Your test code here
    extern crate alloc;
    use alloc::vec::Vec;
    
    let mut vec = Vec::new();
    vec.push(42);
    assert_eq!(vec[0], 42);
    
    theseus_kernel::display::kernel_write_line("My test completed successfully");
}
```

Then add it to the test list in `kernel_main`:

```rust
#[no_mangle]
pub extern "C" fn kernel_main(handoff_addr: u64) -> ! {
    // ... kernel initialization ...
    
    test_my_kernel_functionality();  // Add your test here
    
    // ... existing code ...
}
```

### Test Output and Debugging

#### QEMU Debug Port Output

Tests use the QEMU debug port (0xE9) for output. You can see this output when running tests:

- `M` = kernel_main started
- `1`, `2`, `3`, etc. = individual test numbers
- `A`, `B`, `C`, etc. = test completion markers
- `D` = overall completion
- `%` = QEMU exit (success)
- `P` = panic occurred

#### Kernel Logging

Kernel tests can use `theseus_kernel::display::kernel_write_line()` for more detailed output.

### Makefile Integration

The Makefile provides several targets:

- `make test` - Run bare-metal tests (default)
- `make test-bare-metal` - Run bare-metal tests explicitly
- `make test-kernel` - Run kernel tests
- `make test-panic` - Run panic tests
- `make test-all` - Run all test suites
- `make test-help` - Show available targets

Each target:
1. Builds the bootloader
2. Builds the specific test
3. Creates a test disk image
4. Runs the test in QEMU
5. Reports success/failure

### Test Framework Architecture

#### Custom Test Framework

The tests use Rust's `custom_test_frameworks` feature but bypass the standard collection mechanism due to issues with function pointers in bare-metal environments.

Instead, tests use **direct function calls**:

```rust
// Instead of relying on the framework to collect and call tests:
// test_main();  // This would use the broken framework

// We call tests directly:
test_basic_cpu_ops();
test_memory_operations();
test_control_flow();
```

#### Entry Points

Each test binary has a `kernel_main` entry point that the bootloader calls:

```rust
#[no_mangle]
pub extern "C" fn kernel_main(handoff_addr: u64) -> ! {
    // Test-specific initialization
    // Direct test function calls
    // Clean exit
}
```

#### Exit Mechanisms

Tests exit using QEMU's `isa-debug-exit` device:

```rust
// Success exit (exit code 0)
unsafe {
    core::arch::asm!(
        "mov dx, 0xf4",
        "mov al, 0",
        "out dx, al",
        options(nomem, nostack, preserves_flags)
    );
}

// Error exit (exit code 1)
unsafe {
    core::arch::asm!(
        "mov dx, 0xf4",
        "mov al, 1",
        "out dx, al",
        options(nomem, nostack, preserves_flags)
    );
}
```

## Troubleshooting

### Tests Hang or Timeout

If tests hang, check:

1. **QEMU debug output**: Look for the last character printed to identify where it hangs
2. **Test function calls**: Ensure you're calling functions directly, not through the framework
3. **Panic handling**: Make sure panic handlers are properly set up

### Compilation Errors

Common issues:

1. **Missing `extern crate alloc`**: Required for heap allocation tests
2. **Duplicate panic handlers**: Only define panic handlers in test files, not in library
3. **Missing test attributes**: Include `#![feature(custom_test_frameworks)]` even if not using the framework

### Test Failures

If tests fail:

1. **Check QEMU output**: Look for panic markers (`P`) or unexpected characters
2. **Verify assertions**: Ensure your test logic is correct
3. **Check environment**: Make sure you're testing in the right environment (bare-metal vs kernel)

## Best Practices

### Test Design

1. **Keep tests focused**: Each test should verify one specific behavior
2. **Use descriptive names**: Test function names should clearly indicate what they test
3. **Add output markers**: Use QEMU debug port output for debugging
4. **Handle failures gracefully**: Tests should fail fast with clear error messages

### Environment Selection

1. **Use bare-metal tests** for:
   - CPU operations
   - Basic control flow
   - Stack operations
   - Function call mechanisms

2. **Use kernel tests** for:
   - Memory management
   - Heap allocation
   - Kernel services
   - Integration testing

3. **Use panic tests** for:
   - Error handling
   - Assertion verification
   - Panic handler testing

### Code Organization

1. **Group related tests**: Keep related functionality together
2. **Document test purposes**: Add clear doc comments to test functions
3. **Use consistent naming**: Follow the established naming conventions
4. **Keep tests independent**: Tests should not depend on each other

## File Structure

```
tests/
├── bare_metal_tests.rs    # Bare-metal environment tests
├── kernel_tests.rs        # Kernel-initialized environment tests
└── should_panic.rs        # Panic handling tests

docs/
└── TESTING.md             # This documentation

Makefile                   # Test targets and automation
```

## Advanced Usage

### Custom Test Environments

You can create additional test environments by:

1. Creating a new test file (e.g., `tests/user_space_tests.rs`)
2. Adding it to `kernel/Cargo.toml`
3. Creating a new Makefile target
4. Following the established patterns

### Integration with CI/CD

The test framework is designed to work with automated systems:

```bash
# Run tests with specific timeout
TIMEOUT=30 make test-all

# Check exit codes
if make test-all; then
    echo "All tests passed"
else
    echo "Some tests failed"
    exit 1
fi
```

### Debugging Failed Tests

1. **Check test output**: Look at `/tmp/{test_name}_test_output.log`
2. **Run individual tests**: Use specific make targets to isolate issues
3. **Add debug output**: Use QEMU debug port or kernel logging for debugging
4. **Use GDB**: The debug target provides GDB integration for complex debugging

## Contributing

When adding new tests:

1. Follow the established patterns
2. Add appropriate documentation
3. Test in the correct environment
4. Update this documentation if needed
5. Ensure tests pass before submitting

For questions or issues, refer to the existing test implementations or the TheseusOS development documentation.
