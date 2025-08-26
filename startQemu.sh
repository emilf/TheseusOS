#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

pushd "$SCRIPT_DIR" >/dev/null
make esp

# Use project-local OVMF firmware; copy VARS to a writable location
OVMF_DIR=${OVMF_DIR:-"$SCRIPT_DIR/OVMF"}
OVMF_CODE=${OVMF_CODE:-"$OVMF_DIR/OVMF_CODE.fd"}
OVMF_VARS=${OVMF_VARS:-"$OVMF_DIR/OVMF_VARS.fd"}

mkdir -p "$OVMF_DIR"
if [[ ! -f "$OVMF_CODE" || ! -f "$OVMF_VARS" ]]; then
  for base in \
    /usr/share/edk2-ovmf/x64 \
    /usr/share/edk2/x64 \
    /usr/share/OVMF; do
    if [[ -f "$base/OVMF_CODE.fd" && -f "$base/OVMF_VARS.fd" ]]; then
      cp -f "$base/OVMF_CODE.fd" "$OVMF_CODE"
      cp -f "$base/OVMF_VARS.fd" "$OVMF_VARS"
      break
    fi
  done
fi

if [[ ! -f "$OVMF_CODE" || ! -f "$OVMF_VARS" ]]; then
  echo "Missing OVMF firmware in $OVMF_DIR" >&2
  echo "Expected files: OVMF_CODE.fd and OVMF_VARS.fd" >&2
  echo "Set OVMF_CODE/OVMF_VARS env vars to override, or place files in $OVMF_DIR." >&2
  exit 1
fi

mkdir -p build
OVMF_VARS_RW="$SCRIPT_DIR/build/OVMF_VARS.fd"
cp -f "$OVMF_VARS" "$OVMF_VARS_RW"

# Optional extra QEMU options (e.g., -S -s) via QEMU_OPTS env
QEMU_OPTS=${QEMU_OPTS:-}

qemu-system-x86_64 \
  -machine q35,accel=kvm:tcg \
  -cpu max \
  -m 256M \
  -drive if=pflash,format=raw,readonly=on,file="$OVMF_CODE" \
  -drive if=pflash,format=raw,file="$OVMF_VARS_RW" \
  -serial mon:stdio \
  -drive format=raw,file=fat:rw:build \
  -nic none \
  ${QEMU_OPTS}
popd >/dev/null


