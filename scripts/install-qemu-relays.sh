#!/usr/bin/env bash
set -euo pipefail

# Install TheseusOS QEMU relay units (user systemd)
#
# Creates persistent /tmp endpoints used for interactive debugging:
# - /tmp/qemu-serial-host   (PTY)  -> QEMU uses /tmp/qemu-serial
# - /tmp/qemu-monitor-host  (PTY)  -> QEMU uses /tmp/qemu-monitor
# - /tmp/qemu-debugcon-host (PTY)  -> QEMU uses /tmp/qemu-debugcon
# - /tmp/qemu-qmp-host.sock (unix) -> QEMU uses /tmp/qemu-qmp.sock
#
# Requirements: systemd user session + socat.

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

UNITS=(
  qemu_serial_relay.service
  qemu_monitor_relay.service
  qemu_debugcon_relay.service
  qemu_qmp_relay.service
  qemu_debugcon_logger.service
)

mkdir -p "${HOME}/.config/systemd/user"

for u in "${UNITS[@]}"; do
  src="${SCRIPT_DIR}/${u}"
  dst="${HOME}/.config/systemd/user/${u}"
  if [[ ! -f "$src" ]]; then
    echo "Missing unit file: $src" >&2
    exit 1
  fi
  cp -f "$src" "$dst"
  echo "Installed $u"
done

systemctl --user daemon-reload

for u in "${UNITS[@]}"; do
  systemctl --user enable --now "$u"
  systemctl --user status "$u" --no-pager --full | sed -n '1,8p'
  echo "---"
done

echo "✓ QEMU relay units installed and started."
