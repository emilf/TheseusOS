#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# Default to headless mode
HEADLESS=${1:-true}
TIMEOUT=${2:-0}

pushd "$SCRIPT_DIR" >/dev/null

# Build everything including BIOS files
echo "Building project..."
make all

# Use BIOS files set up by Makefile
OVMF_CODE="$SCRIPT_DIR/OVMF/OVMF_CODE.fd"
OVMF_VARS_RW="$SCRIPT_DIR/build/OVMF_VARS.fd"

# Verify BIOS files exist (they should be set up by make all)
if [[ ! -f "$OVMF_CODE" ]]; then
  echo "Missing OVMF_CODE.fd - run 'make all' first" >&2
  exit 1
fi

if [[ ! -f "$OVMF_VARS_RW" ]]; then
  echo "Missing OVMF_VARS.fd - run 'make all' first" >&2
  exit 1
fi

# Configure QEMU options based on mode
if [[ "$HEADLESS" == "true" || "$HEADLESS" == "headless" ]]; then
  echo "Starting QEMU in headless mode..."
  echo "  QEMU Debug Driver output: stdout (port 0xe9)"
  QEMU_DISPLAY="-nographic -serial null"
  QEMU_MONITOR="-monitor null"
  QEMU_DEBUG="-device isa-debugcon,chardev=debugcon"
  QEMU_DEBUG_CHAR="-chardev stdio,id=debugcon"
else
  echo "Starting QEMU in headed mode..."
  echo "  QEMU Debug Driver output: debug.log"
  QEMU_DISPLAY=""
  QEMU_MONITOR="-monitor stdio"
  QEMU_DEBUG="-device isa-debugcon,chardev=debugcon"
  QEMU_DEBUG_CHAR="-chardev file,id=debugcon,path=debug.log"
fi

# Optional extra QEMU options (e.g., -S -s) via QEMU_OPTS env
QEMU_OPTS=${QEMU_OPTS:-}

# Build QEMU command
QEMU_CMD="qemu-system-x86_64 \
  -machine q35,accel=kvm:tcg \
  -cpu max \
  -m 256M \
  -drive if=pflash,format=raw,readonly=on,file=\"$OVMF_CODE\" \
  -drive if=pflash,format=raw,file=\"$OVMF_VARS_RW\" \
  -device isa-debug-exit,iobase=0xf4,iosize=0x04 \
  $QEMU_DEBUG \
  $QEMU_DEBUG_CHAR \
  $QEMU_DISPLAY \
  $QEMU_MONITOR \
  -drive format=raw,file=fat:rw:build \
  -nic none \
  -no-reboot \
  ${QEMU_OPTS}"

# Run QEMU with optional timeout
if [[ "$TIMEOUT" -gt 0 ]]; then
  echo "Running QEMU with ${TIMEOUT}s timeout..."
  timeout "$TIMEOUT"s bash -c "eval '$QEMU_CMD'"
  EXIT_CODE=$?
  if [[ $EXIT_CODE -eq 1 ]]; then
    echo "✓ QEMU exited gracefully from guest"
  elif [[ $EXIT_CODE -eq 124 ]]; then
    echo "⚠ QEMU timed out after ${TIMEOUT}s"
  else
    echo "QEMU exited with code $EXIT_CODE"
  fi
else
  echo "Running QEMU (no timeout)..."
  eval "$QEMU_CMD"
  EXIT_CODE=$?
  if [[ $EXIT_CODE -eq 1 ]]; then
    echo "✓ QEMU exited gracefully from guest"
  fi
fi

popd >/dev/null
