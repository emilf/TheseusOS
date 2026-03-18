#!/usr/bin/env python3
"""Small QMP helper for TheseusOS/QEMU debugging.

Connects to the stable host-side QMP relay socket (default: /tmp/qemu-qmp-host.sock),
negotiates capabilities, sends one command, and prints the JSON reply.

Examples:
  ./scripts/qmp-command.py status
  ./scripts/qmp-command.py reset
  ./scripts/qmp-command.py quit
  ./scripts/qmp-command.py cmd query-pci
  ./scripts/qmp-command.py hmp 'info pci'
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Any

DEFAULT_SOCKET = "/tmp/qemu-qmp-host.sock"


def recv_message(sock_file) -> dict[str, Any]:
    line = sock_file.readline()
    if not line:
        raise RuntimeError("QMP socket closed before a complete JSON message was received")
    return json.loads(line)


def send_message(sock_file, msg: dict[str, Any]) -> None:
    sock_file.write(json.dumps(msg) + "\r\n")
    sock_file.flush()


def negotiate(sock_file) -> dict[str, Any]:
    greeting = recv_message(sock_file)
    send_message(sock_file, {"execute": "qmp_capabilities"})
    response = recv_message(sock_file)
    if "return" not in response:
        raise RuntimeError(f"QMP capabilities negotiation failed: {response}")
    return greeting


def build_command(args: argparse.Namespace) -> dict[str, Any]:
    if args.action == "status":
        return {"execute": "query-status"}
    if args.action == "reset":
        return {"execute": "system_reset"}
    if args.action == "quit":
        return {"execute": "quit"}
    if args.action == "cmd":
        return {"execute": args.name}
    if args.action == "hmp":
        return {
            "execute": "human-monitor-command",
            "arguments": {"command-line": args.command},
        }
    raise RuntimeError(f"unknown action {args.action}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Send one QMP command and print the JSON reply")
    parser.add_argument("--socket", default=DEFAULT_SOCKET, help=f"QMP socket path (default: {DEFAULT_SOCKET})")

    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("status", help="query VM run state")
    sub.add_parser("reset", help="reset the VM")
    sub.add_parser("quit", help="quit the VM")

    cmd = sub.add_parser("cmd", help="send a raw QMP execute command with no arguments")
    cmd.add_argument("name", help="QMP command name, e.g. query-pci")

    hmp = sub.add_parser("hmp", help="send an HMP command via QMP human-monitor-command")
    hmp.add_argument("command", help="HMP command line, e.g. 'info pci'")

    args = parser.parse_args()

    command = build_command(args)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(args.socket)
        sock_file = sock.makefile("rw", encoding="utf-8", newline="\n")
        greeting = negotiate(sock_file)
        send_message(sock_file, command)
        response = recv_message(sock_file)
        print(json.dumps({"greeting": greeting, "response": response}, indent=2, sort_keys=True))
        return 0 if "error" not in response else 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
