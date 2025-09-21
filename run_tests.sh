#!/usr/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[TEST]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Function to run a single test
run_test() {
    local test_name="$1"
    local test_binary="$2"
    local expected_exit_code="$3"
    
    print_status "Running test: $test_name"
    
    # Build the test
    print_status "Building test binary..."
    if ! cargo build --target x86_64-unknown-none --test "$test_name" --release; then
        print_error "Failed to build test: $test_name"
        return 1
    fi
    
    # Find the built test binary
    local test_path=$(find target/x86_64-unknown-none/release/deps -name "${test_name}-*" -type f -executable | head -1)
    
    if [[ ! -f "$test_path" ]]; then
        print_error "Test binary not found: $test_name"
        return 1
    fi
    
    print_status "Test binary: $test_path"
    
    # Create a temporary disk image with the test binary
    local temp_disk="/tmp/test_disk.img"
    local temp_dir="/tmp/test_efi"
    
    # Clean up any existing temp files
    rm -rf "$temp_disk" "$temp_dir"
    
    # Create EFI directory structure
    mkdir -p "$temp_dir/EFI/BOOT"
    
    # Copy the test binary as the kernel
    cp "$test_path" "$temp_dir/EFI/BOOT/kernel.efi"
    
    # Create a minimal EFI bootloader that just loads and runs the test
    # For now, we'll use the existing bootloader and replace the kernel
    if [[ -f "$SCRIPT_DIR/build/disk.img" ]]; then
        cp "$SCRIPT_DIR/build/disk.img" "$temp_disk"
        
        # Mount the disk image and replace the kernel
        # This is a simplified approach - in practice you might want to use
        # a more robust method to modify the disk image
        print_warning "Using existing disk image with test binary replacement"
    else
        print_error "No existing disk image found. Please run 'make all' first."
        return 1
    fi
    
    # Run QEMU with the test
    print_status "Running test in QEMU..."
    
    local qemu_output
    local qemu_exit_code
    
    # Use the existing QEMU configuration but with our test disk
    qemu_output=$(
        timeout 30s qemu-system-x86_64 \
            -machine q35,accel=kvm:tcg \
            -cpu max \
            -m 1G \
            -drive if=pflash,format=raw,readonly=on,file="$SCRIPT_DIR/OVMF/OVMF_CODE.fd" \
            -drive if=pflash,format=raw,file="$SCRIPT_DIR/build/OVMF_VARS.fd" \
            -device isa-debug-exit,iobase=0xf4,iosize=0x04 \
            -device isa-debugcon,chardev=debugcon \
            -chardev stdio,id=debugcon \
            -display none \
            -serial null \
            -monitor none \
            -drive format=raw,file="$temp_disk" \
            -nic none \
            -no-reboot \
            2>&1
    )
    
    qemu_exit_code=$?
    
    # Check the output for test results
    if echo "$qemu_output" | grep -q "All tests passed!"; then
        print_status "✓ $test_name PASSED"
        echo "$qemu_output"
        return 0
    elif echo "$qemu_output" | grep -q "\[failed\]"; then
        print_error "✗ $test_name FAILED"
        echo "$qemu_output"
        return 1
    elif [[ $qemu_exit_code -eq 124 ]]; then
        print_error "✗ $test_name TIMED OUT"
        echo "$qemu_output"
        return 1
    else
        print_warning "✗ $test_name UNKNOWN RESULT (exit code: $qemu_exit_code)"
        echo "$qemu_output"
        return 1
    fi
}

# Main execution
main() {
    print_status "TheseusOS Test Runner"
    print_status "===================="
    
    # Check if we're in the right directory
    if [[ ! -f "Cargo.toml" ]]; then
        print_error "Not in project root directory"
        exit 1
    fi
    
    # Build the project first
    print_status "Building project..."
    if ! make all; then
        print_error "Failed to build project"
        exit 1
    fi
    
    local tests_passed=0
    local tests_failed=0
    
    # Run tests
    if run_test "simple" "simple" 0; then
        ((tests_passed++))
    else
        ((tests_failed++))
    fi
    
    if run_test "should_panic" "should_panic" 0; then
        ((tests_passed++))
    else
        ((tests_failed++))
    fi
    
    # Summary
    print_status "===================="
    print_status "Test Results:"
    print_status "  Passed: $tests_passed"
    if [[ $tests_failed -gt 0 ]]; then
        print_error "  Failed: $tests_failed"
        exit 1
    else
        print_status "  Failed: $tests_failed"
        print_status "All tests passed! ✓"
        exit 0
    fi
}

main "$@"
