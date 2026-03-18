#!/usr/bin/env bash
set -euo pipefail

# Bring up a reusable tmux-based interactive TheseusOS/QEMU loop.
#
# This complements the long-running systemd user relay units with a self-contained
# workflow that is easy for humans and AI tools to spawn on demand:
# - creates PTY/socket relays via socat in tmux windows
# - launches headless QEMU through the Rust runner
# - leaves stable host endpoints under /tmp for serial/monitor/debugcon/QMP access
#
# Endpoints created:
# - /tmp/qemu-serial-host
# - /tmp/qemu-monitor-host
# - /tmp/qemu-debugcon-host
# - /tmp/qemu-qmp-host.sock
#
# QEMU consumes the matching QEMU-side endpoints:
# - /tmp/qemu-serial
# - /tmp/qemu-monitor
# - /tmp/qemu-debugcon
# - /tmp/qemu-qmp.sock

SESSION_NAME=${SESSION_NAME:-theseus-live}
PROFILE=${PROFILE:-min}
HEADLESS=${HEADLESS:-1}
NO_BUILD=${NO_BUILD:-1}
REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

require() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "Missing required command: $1" >&2
        exit 1
    }
}

require tmux
require socat
require cargo

cleanup_paths() {
    rm -f \
        /tmp/qemu-serial /tmp/qemu-serial-host \
        /tmp/qemu-monitor /tmp/qemu-monitor-host \
        /tmp/qemu-debugcon /tmp/qemu-debugcon-host \
        /tmp/qemu-qmp.sock /tmp/qemu-qmp-host.sock
}

start_relays() {
    tmux new-session -d -s "$SESSION_NAME" -n qemu "cd '$REPO_ROOT' && exec bash"
    tmux new-window -t "$SESSION_NAME" -n serial-relay \
        "exec socat PTY,link=/tmp/qemu-serial,raw,echo=0 PTY,link=/tmp/qemu-serial-host,raw,echo=0"
    tmux new-window -t "$SESSION_NAME" -n monitor-relay \
        "exec socat PTY,link=/tmp/qemu-monitor,raw,echo=0 PTY,link=/tmp/qemu-monitor-host,raw,echo=0"
    tmux new-window -t "$SESSION_NAME" -n debugcon-relay \
        "exec socat PTY,link=/tmp/qemu-debugcon,raw,echo=0 PTY,link=/tmp/qemu-debugcon-host,raw,echo=0"
    tmux new-window -t "$SESSION_NAME" -n qmp-relay \
        "exec socat UNIX-LISTEN:/tmp/qemu-qmp-host.sock,fork UNIX-CONNECT:/tmp/qemu-qmp.sock"
}

launch_qemu() {
    local cmd=(cargo run -p theseus-qemu -- --profile "$PROFILE" --relays)
    if [[ "$HEADLESS" == "1" ]]; then
        cmd+=(--headless)
    fi
    if [[ "$NO_BUILD" == "1" ]]; then
        cmd+=(--no-build)
    fi

    tmux send-keys -t "$SESSION_NAME":qemu C-c
    tmux send-keys -t "$SESSION_NAME":qemu "cd '$REPO_ROOT' && ${cmd[*]}" Enter
}

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "tmux session '$SESSION_NAME' already exists; reusing it" >&2
else
    cleanup_paths
    start_relays
    sleep 1
fi

launch_qemu
sleep 2

echo "tmux session: $SESSION_NAME"
echo "windows:"
tmux list-windows -t "$SESSION_NAME"
echo
echo "host endpoints:"
for p in \
    /tmp/qemu-serial-host \
    /tmp/qemu-monitor-host \
    /tmp/qemu-debugcon-host \
    /tmp/qemu-qmp-host.sock; do
    if [[ -e "$p" ]]; then
        ls -l "$p"
    else
        echo "missing: $p"
    fi
done

echo
echo "examples:"
echo "  timeout 2 cat /tmp/qemu-debugcon-host | tail -20"
echo "  printf 'help\\r' > /tmp/qemu-serial-host"
echo "  timeout 2 cat /tmp/qemu-serial-host | tail -40"
echo "  tmux attach -t $SESSION_NAME"
