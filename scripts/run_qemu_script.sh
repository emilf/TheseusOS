#!/usr/bin/env bash
set -euo pipefail

# === CONFIG ===
SERIAL_BASE="/tmp/qemu-serial"
MONITOR_BASE="/tmp/qemu-monitor"
DEBUGCON_BASE="/tmp/qemu-debugcon"
KERNEL_PATH="path/to/your/kernel.elf"
MEMORY="512M"

# === FUNCTIONS ===
relay_active() {
    systemctl --user is-active --quiet "$1" 2>/dev/null
}

ensure_pipes() {
    local base="$1"
    if [ ! -p "${base}.in" ] || [ ! -p "${base}.out" ]; then
        echo "Creating temporary pipe pair for ${base}"
        rm -f "${base}.in" "${base}.out"
        mkfifo "${base}.in" "${base}.out"
        socat -u PIPE:"${base}.in" PIPE:"${base}.out" >/dev/null 2>&1 &
        sleep 0.2
    fi
}

# === RELAY CHECKS ===
if ! relay_active qemu-serial-relay.service; then
    ensure_pipes "$SERIAL_BASE"
fi

if ! relay_active qemu-monitor-relay.service; then
    ensure_pipes "$MONITOR_BASE"
fi

if ! relay_active qemu-debugcon-relay.service; then
    ensure_pipes "$DEBUGCON_BASE"
fi

# === QEMU ARGS ===
QEMU_ARGS=(
    -m "$MEMORY"
    -kernel "$KERNEL_PATH"
    -no-reboot
    -d guest_errors
    -serial pipe:"$SERIAL_BASE"
    -monitor pipe:"$MONITOR_BASE"
    -debugcon pipe:"$DEBUGCON_BASE"
)

exec qemu-system-x86_64 "${QEMU_ARGS[@]}" "$@"
