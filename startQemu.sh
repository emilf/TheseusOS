#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# Default to headless mode
HEADLESS=${1:-true}
TIMEOUT=${2:-0}

pushd "$SCRIPT_DIR" >/dev/null

# Remove debug logs
rm -f debug.log qemu.log

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

## Configure QEMU options based on mode
if [[ "$HEADLESS" == "true" || "$HEADLESS" == "headless" ]]; then
  echo "Starting QEMU in headless mode..."
  echo "  QEMU Debug Driver output: qemu-debug.log (port 0xe9)"
  echo "  COM1 Serial: stdio (for interactive monitor)"
  QEMU_DISPLAY_ARGS=( -display none -serial stdio )
  # Do not wait for a monitor connection or pause by default
  QEMU_MONITOR_ARGS=( -monitor none )
  QEMU_PAUSE_ARGS=()
  QEMU_DEBUG_ARGS=( -device isa-debugcon,chardev=debugcon )
  QEMU_DEBUG_CHAR_ARGS=( -chardev file,path=qemu-debug.log,id=debugcon )
else
  echo "Starting QEMU in headed mode..."
  echo "  QEMU Debug Driver output: debug.log"
  QEMU_DISPLAY_ARGS=()
  QEMU_MONITOR_ARGS=( -monitor stdio )
  QEMU_PAUSE_ARGS=()
  QEMU_DEBUG_ARGS=( -device isa-debugcon,chardev=debugcon )
  QEMU_DEBUG_CHAR_ARGS=( -chardev file,id=debugcon,path=debug.log )
fi

# Optional extra QEMU options (e.g., -S -s) via QEMU_OPTS env
# For GDB debugging, use: QEMU_OPTS="-S -s" make run-test
QEMU_OPTS=${QEMU_OPTS:-}

# If caller asked for CPU debug but no output file, direct to stdout
if [[ "$QEMU_OPTS" == *"-d"* && "$QEMU_OPTS" != *"-D"* ]]; then
  # Route QEMU's debug output to stdio by using chardev stdio for serial
  # and avoid specifying -D so debug goes to stderr; we keep it merged in the console
  :
fi

## Build QEMU command as an array (robust quoting, no eval needed)
QEMU_CMD=(
  qemu-system-x86_64
  -machine q35,accel=kvm:tcg
  -cpu max
  -m 1G
  -drive if=pflash,format=raw,readonly=on,file="$OVMF_CODE"
  -drive if=pflash,format=raw,file="$OVMF_VARS_RW"
  -device isa-debug-exit,iobase=0xf4,iosize=0x04
  ${QEMU_DEBUG_ARGS[@]}
  ${QEMU_DEBUG_CHAR_ARGS[@]}
  ${QEMU_DISPLAY_ARGS[@]}
  ${QEMU_MONITOR_ARGS[@]}
  ${QEMU_PAUSE_ARGS[@]}
  -drive format=raw,file="$SCRIPT_DIR/build/disk.img"
  -nic none
  -no-reboot
)

# Append extra options from QEMU_OPTS (split on spaces safely)
if [[ -n "${QEMU_OPTS}" ]]; then
  # shellcheck disable=SC2206
  EXTRA_OPTS=( ${QEMU_OPTS} )
  QEMU_CMD+=( "${EXTRA_OPTS[@]}" )
fi

## Run QEMU with optional timeout
if [[ "$TIMEOUT" -gt 0 ]]; then
  echo "Running QEMU with ${TIMEOUT}s timeout..."
  TMP_OUT=$(mktemp)
  timeout --foreground "${TIMEOUT}s" "${QEMU_CMD[@]}" 2>&1 | tee "$TMP_OUT"
  EXIT_CODE=$?
  if grep -q "Kernel environment test completed successfully" "$TMP_OUT"; then
    echo "✓ Detected success marker from kernel"
    EXIT_CODE=0
  fi
  if [[ $EXIT_CODE -eq 1 ]]; then
    echo "✓ QEMU exited gracefully from guest"
  elif [[ $EXIT_CODE -eq 124 ]]; then
    echo "⚠ QEMU timed out after ${TIMEOUT}s"
  else
    echo "QEMU exited with code $EXIT_CODE"
  fi
  exit $EXIT_CODE
else
  echo "Running QEMU (no timeout)..."
  TMP_OUT=$(mktemp)
  "${QEMU_CMD[@]}" 2>&1 | tee "$TMP_OUT"
  EXIT_CODE=$?
  if grep -q "Kernel environment test completed successfully" "$TMP_OUT"; then
    echo "✓ Detected success marker from kernel"
    EXIT_CODE=0
  fi
  if [[ $EXIT_CODE -eq 1 ]]; then
    echo "✓ QEMU exited gracefully from guest"
  fi
  exit $EXIT_CODE
fi

popd >/dev/null
