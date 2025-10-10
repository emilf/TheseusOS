#!/usr/bin/env bash
# Test script for kernel monitor
# This script sends commands to the monitor and captures output

set -e

echo "=== TheseusOS Kernel Monitor Test ==="
echo ""
echo "This script will:"
echo "1. Start QEMU with the kernel monitor"
echo "2. Send test commands automatically"
echo "3. Display the results"
echo ""
echo "Press Ctrl+C to stop the test"
echo ""

# Clean up old logs
rm -f qemu-debug.log monitor-test.log

# Start QEMU in the background with monitor enabled
# Send serial output to a file for automated testing
timeout 30 qemu-system-x86_64 \
  -machine q35 \
  -cpu qemu64 \
  -m 256M \
  -drive if=pflash,format=raw,readonly=on,file=OVMF/OVMF_CODE.4m.fd \
  -drive if=pflash,format=raw,file=OVMF/OVMF_VARS.4m.fd \
  -drive format=raw,file=build/theseus_efi.img \
  -display none \
  -serial file:monitor-test.log \
  -debugcon file:qemu-debug.log \
  -device isa-debugcon,chardev=debugcon \
  -chardev file,path=qemu-debug.log,id=debugcon \
  -monitor none \
  2>&1 &

QEMU_PID=$!

echo "QEMU started (PID: $QEMU_PID)"
echo "Waiting for monitor to be ready..."
sleep 3

# Function to send command to monitor
send_command() {
    echo "$1" >> monitor-test.log
    sleep 0.5
}

echo "Sending test commands..."

# Send test commands
send_command "help"
send_command "regs"  
send_command "devices"
send_command "cpuid"
send_command "idt"
send_command "gdt"
send_command "mem 0xFFFFFFFF80000000"
send_command "mem"
send_command "dump 0xFFFF800000000000 64"
send_command "io r 0x3FD"
send_command "halt"

# Wait a bit for commands to process
sleep 2

# Kill QEMU if still running
kill $QEMU_PID 2>/dev/null || true
wait $QEMU_PID 2>/dev/null || true

echo ""
echo "=== Test Complete ==="
echo ""
echo "Monitor output saved to: monitor-test.log"
echo "Debug output saved to: qemu-debug.log"
echo ""
echo "To view monitor output:"
echo "  cat monitor-test.log"
echo ""
echo "To view debug output:"
echo "  cat qemu-debug.log"

