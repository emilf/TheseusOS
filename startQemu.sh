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

# Use project-local OVMF firmware
OVMF_DIR=${OVMF_DIR:-"$SCRIPT_DIR/OVMF"}
OVMF_CODE=${OVMF_CODE:-"$OVMF_DIR/OVMF_CODE.fd"}

# Set up OVMF_CODE file
mkdir -p "$OVMF_DIR"
if [[ ! -f "$OVMF_CODE" ]]; then
  echo "Setting up OVMF_CODE..."
  for base in \
    /usr/share/edk2-ovmf/x64 \
    /usr/share/edk2/x64 \
    /usr/share/OVMF; do
    if [[ -f "$base/OVMF_CODE.fd" ]]; then
      echo "Found OVMF_CODE.fd in $base"
      cp -f "$base/OVMF_CODE.fd" "$OVMF_CODE"
      break
    fi
  done
fi

if [[ ! -f "$OVMF_CODE" ]]; then
  echo "Missing OVMF_CODE.fd" >&2
  echo "Please install edk2-ovmf package" >&2
  exit 1
fi

# OVMF_VARS_RW is already set up by make bios
OVMF_VARS_RW="$SCRIPT_DIR/build/OVMF_VARS.fd"

# Configure QEMU options based on mode
if [[ "$HEADLESS" == "true" || "$HEADLESS" == "headless" ]]; then
  echo "Starting QEMU in headless mode..."
  QEMU_DISPLAY="-nographic"
  QEMU_MONITOR="-monitor none"
  QEMU_SERIAL="-serial stdio"
else
  echo "Starting QEMU in headed mode..."
  QEMU_DISPLAY=""
  QEMU_MONITOR="-monitor stdio"
  QEMU_SERIAL="-serial file:serial.log"
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
  $QEMU_SERIAL \
  $QEMU_DISPLAY \
  $QEMU_MONITOR \
  -drive format=raw,file=fat:rw:build \
  -nic none \
  -no-reboot \
  ${QEMU_OPTS}"

# Run QEMU with optional timeout
if [[ "$TIMEOUT" -gt 0 ]]; then
  echo "Running QEMU with ${TIMEOUT}s timeout..."
  timeout "$TIMEOUT"s bash -c "$QEMU_CMD"
  EXIT_CODE=$?
  if [[ $EXIT_CODE -eq 3 ]]; then
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
  if [[ $EXIT_CODE -eq 3 ]]; then
    echo "✓ QEMU exited gracefully from guest"
  fi
fi

popd >/dev/null


