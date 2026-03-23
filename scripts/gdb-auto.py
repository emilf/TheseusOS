#!/usr/bin/env python3
"""
gdb-auto.py — One-command GDB debug session for TheseusOS.

Usage:
    python3 scripts/gdb-auto.py [--tmux SESSION] [--timeout-boot SECS]
                                 [--qemu-pane 0] [--gdb-pane 1]
                                 [--no-restart] [--addr 0x...]

Workflow:
    1. Start QEMU (or reuse existing pane) without -S so UEFI boots freely
       and writes "efi_main @ 0x<addr>" to the debugcon log.
    2. Wait for that line (with timeout), extract the runtime address.
    3. Kill QEMU and restart it paused (-S) with a GDB unix socket.
    4. Spawn GDB via pexpect, source debug.gdb, connect, call theseus-load
       with the captured address, and drop into interactive mode.

Every wait has a hard timeout — the script never hangs silently.

Requirements:
    pip install pexpect       (or: pip install --break-system-packages pexpect)

Environment:
    Designed to run inside the bwrap sandbox that OpenClaw uses.
    QEMU is kept alive in a tmux pane (survives sandbox exec sessions).
    GDB is driven via pexpect so Ctrl-C reliably reaches the remote target.
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

try:
    import pexpect
except ImportError:
    sys.exit(
        "ERROR: pexpect not installed.\n"
        "Run: pip install --break-system-packages pexpect"
    )

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
WORKSPACE       = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEBUGCON_LOG    = "/tmp/theseus-gdb-auto-debugcon.log"
GDB_SOCKET      = "/tmp/theseus-gdb-auto.sock"
GDB_SCRIPT      = os.path.join(WORKSPACE, "debug.gdb")
SYMBOL_FILE     = os.path.join(WORKSPACE, "build", "BOOTX64.SYM")
OVMF_CODE       = os.path.join(WORKSPACE, "OVMF", "OVMF_CODE.fd")
OVMF_VARS       = os.path.join(WORKSPACE, "build", "OVMF_VARS.fd")
DISK_IMG        = os.path.join(WORKSPACE, "build", "disk.img")

GDB_PROMPT_RE   = r"\(gdb\)"
ADDR_RE         = re.compile(r"efi_main @ (0x[0-9a-fA-F]+)")

DEFAULT_TMUX    = "theseus"
DEFAULT_TIMEOUT_ADDR  = 60    # seconds to wait for efi_main address
DEFAULT_TIMEOUT_BOOT  = 120   # seconds to wait for breakpoint after continue
DEFAULT_QEMU_PANE     = 0
DEFAULT_GDB_PANE      = 1


# ---------------------------------------------------------------------------
# QEMU command builder
# ---------------------------------------------------------------------------
def qemu_cmd(paused: bool, gdb_socket: str, debugcon_log: str) -> list[str]:
    cmd = [
        "qemu-system-x86_64",
        "-machine", "q35,accel=kvm:tcg,kernel-irqchip=split",
        "-cpu", "max",
        "-smp", "4",
        "-m", "2G",
        "-drive", f"if=pflash,format=raw,readonly=on,file={OVMF_CODE}",
        "-drive", f"if=pflash,format=raw,file={OVMF_VARS}",
        "-device", "isa-debug-exit,iobase=0xf4,iosize=0x04",
        "-device", "isa-debugcon,chardev=debugcon",
        "-chardev", f"file,id=debugcon,path={debugcon_log}",
        "-display", "none",
        "-drive", f"if=none,id=nvme0,file={DISK_IMG},format=raw",
        "-device", "nvme,drive=nvme0,serial=deadbeef",
        "-device", "pcie-root-port,id=rp0,slot=0,chassis=1",
        "-device", "pcie-root-port,id=rp1,slot=1,chassis=2",
        "-device", "pcie-root-port,id=rp2,slot=2,chassis=3",
        "-device", "virtio-gpu-pci,bus=rp0",
        "-device", "qemu-xhci,id=xhci0",
        "-device", "usb-kbd,bus=xhci0.0",
        "-device", "usb-mouse,bus=xhci0.0",
        "-device", "virtio-net-pci,id=nic0,bus=rp2",
        "-nic", "none",
        # GDB stub via unix socket (no TCP port conflicts)
        "-chardev", f"socket,path={gdb_socket},server=on,wait=off,id=gdb0",
        "-gdb", "chardev:gdb0",
    ]
    if paused:
        cmd.append("-S")
    else:
        cmd.append("-no-reboot")
    return cmd


# ---------------------------------------------------------------------------
# tmux helpers
# ---------------------------------------------------------------------------
def tmux_send(session: str, pane: int, text: str):
    subprocess.run(
        ["tmux", "send-keys", "-t", f"{session}:0.{pane}", text, "Enter"],
        check=True,
    )


def tmux_capture(session: str, pane: int) -> str:
    r = subprocess.run(
        ["tmux", "capture-pane", "-t", f"{session}:0.{pane}", "-p"],
        capture_output=True, text=True,
    )
    return r.stdout


def tmux_kill_pane_process(session: str, pane: int):
    """Send Ctrl-C to whatever is running in the pane."""
    subprocess.run(
        ["tmux", "send-keys", "-t", f"{session}:0.{pane}", "C-c"],
        check=False,
    )
    time.sleep(0.5)


def ensure_tmux_session(session: str):
    r = subprocess.run(["tmux", "has-session", "-t", session],
                       capture_output=True)
    if r.returncode != 0:
        subprocess.run(
            ["tmux", "new-session", "-d", "-s", session, "-x", "220", "-y", "50"],
            check=True,
        )
        # Create second pane
        subprocess.run(
            ["tmux", "split-window", "-h", "-t", session],
            check=True,
        )
        print(f"[gdb-auto] Created tmux session '{session}' with 2 panes")
    else:
        # Ensure at least 2 panes exist
        r2 = subprocess.run(
            ["tmux", "list-panes", "-t", session],
            capture_output=True, text=True,
        )
        if r2.stdout.count("\n") < 2:
            subprocess.run(
                ["tmux", "split-window", "-h", "-t", session],
                check=False,
            )


# ---------------------------------------------------------------------------
# Address probe: run QEMU freely, wait for efi_main line in debugcon log
# ---------------------------------------------------------------------------
def probe_efi_main_address(
    session: str,
    qemu_pane: int,
    timeout: int,
) -> str | None:
    # Clear old log
    try:
        os.unlink(DEBUGCON_LOG)
    except FileNotFoundError:
        pass
    try:
        os.unlink(GDB_SOCKET)
    except FileNotFoundError:
        pass

    cmd = " ".join(qemu_cmd(paused=False, gdb_socket=GDB_SOCKET,
                             debugcon_log=DEBUGCON_LOG))
    print(f"[gdb-auto] Starting QEMU (probe run, no -S)...")
    tmux_kill_pane_process(session, qemu_pane)
    tmux_send(session, qemu_pane, f"cd {WORKSPACE} && {cmd}")

    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(1)
        try:
            with open(DEBUGCON_LOG) as f:
                for line in f:
                    m = ADDR_RE.search(line)
                    if m:
                        addr = m.group(1)
                        print(f"[gdb-auto] ✅ Got efi_main address: {addr}")
                        return addr
        except FileNotFoundError:
            pass

    print(f"[gdb-auto] ⏰ Timed out waiting for efi_main address ({timeout}s)")
    return None


# ---------------------------------------------------------------------------
# Main debug session: restart QEMU paused, drive GDB via pexpect
# ---------------------------------------------------------------------------
def run_debug_session(
    session: str,
    qemu_pane: int,
    gdb_pane: int,
    efi_main_addr: str,
    timeout_boot: int,
    interactive: bool,
):
    # Kill probe QEMU, start paused QEMU
    try:
        os.unlink(GDB_SOCKET)
    except FileNotFoundError:
        pass

    cmd = " ".join(qemu_cmd(paused=True, gdb_socket=GDB_SOCKET,
                             debugcon_log=DEBUGCON_LOG))
    print(f"[gdb-auto] Restarting QEMU paused (-S)...")
    tmux_kill_pane_process(session, qemu_pane)
    time.sleep(1)
    tmux_send(session, qemu_pane, f"cd {WORKSPACE} && {cmd}")

    # Wait for GDB socket to appear
    deadline = time.time() + 15
    while time.time() < deadline:
        if os.path.exists(GDB_SOCKET):
            break
        time.sleep(0.3)
    else:
        sys.exit("ERROR: QEMU GDB socket never appeared. Check QEMU startup.")

    print(f"[gdb-auto] GDB socket ready: {GDB_SOCKET}")
    print(f"[gdb-auto] Spawning GDB (pexpect)...")

    child = pexpect.spawn(
        "gdb",
        cwd=WORKSPACE,
        encoding=None,
        timeout=30,
        logfile=open("/tmp/gdb-auto-raw.log", "wb"),
    )

    def gdb(cmd: str, timeout: int = 15, prompt_timeout: int = 15) -> str:
        child.sendline(cmd.encode())
        child.expect(GDB_PROMPT_RE.encode(), timeout=prompt_timeout)
        out = child.before.decode(errors="replace").strip()
        return out

    def gdb_print(cmd: str, timeout: int = 15):
        out = gdb(cmd, timeout=timeout, prompt_timeout=timeout)
        if out:
            # Strip the echoed command from output
            lines = [l for l in out.splitlines() if l.strip() and l.strip() != cmd.strip()]
            for l in lines:
                print(f"  {l}")
        return out

    try:
        child.expect(GDB_PROMPT_RE.encode(), timeout=15)
        print("[gdb-auto] GDB started")

        gdb("set pagination off")
        gdb("set confirm off")
        gdb("set architecture i386:x86-64")
        gdb("set demangle-style rust")
        gdb(f"symbol-file {SYMBOL_FILE}")

        out = gdb(f"source {GDB_SCRIPT}", timeout=20)
        # Print section deltas from the source output
        for line in out.splitlines():
            if "Δ" in line or "image_base" in line or "efi_main link" in line:
                print(f"  {line}")

        print(f"[gdb-auto] Connecting to QEMU ({GDB_SOCKET})...")
        out = gdb(f"target remote {GDB_SOCKET}", timeout=15)
        rip_check = gdb("info registers rip")
        rip = next((l for l in rip_check.splitlines() if "rip" in l), "")
        if "0xfff0" in rip:
            print(f"[gdb-auto] ✅ Confirmed halted at reset vector (rip=0xfff0)")
        else:
            print(f"[gdb-auto] ⚠️  Unexpected RIP: {rip} (expected 0xfff0)")

        print(f"[gdb-auto] Loading symbols for efi_main @ {efi_main_addr}...")
        out = gdb_print(f"theseus-load {efi_main_addr}", timeout=20)

        if interactive:
            # Hand off to fully interactive GDB in the GDB pane
            # We do this by writing the GDB PID and connecting tmux pane to it
            print()
            print("[gdb-auto] ─────────────────────────────────────────────────")
            print("[gdb-auto]  Symbols loaded. Dropping into interactive GDB.")
            print("[gdb-auto]  Breakpoints armed. Type 'c' to run.")
            print(f"[gdb-auto]  tmux pane: {session}:0.{gdb_pane}")
            print("[gdb-auto] ─────────────────────────────────────────────────")
            # pexpect.interact() hands the TTY directly to the user
            child.interact()
        else:
            # Non-interactive: continue and wait for breakpoint
            print(f"[gdb-auto] Issuing continue, waiting up to {timeout_boot}s...")
            child.sendline(b"continue")
            idx = child.expect(
                [GDB_PROMPT_RE.encode(), pexpect.TIMEOUT, pexpect.EOF],
                timeout=timeout_boot,
            )
            output = child.before.decode(errors="replace")

            if idx == 0:
                print(f"[gdb-auto] GDB stopped. Output:")
                for line in output.splitlines():
                    print(f"  {line}")
                if "Breakpoint" in output and (
                    "efi_main" in output or efi_main_addr in output
                ):
                    print(f"[gdb-auto] ✅ BREAKPOINT HIT at efi_main!")
                else:
                    print(f"[gdb-auto] ⚠️  GDB stopped but not at expected breakpoint")
                gdb_print("info registers rip")
                gdb_print("backtrace 5")
            elif idx == 1:
                print(f"[gdb-auto] ⏰ Timeout ({timeout_boot}s) — breakpoint not hit")
                print(f"[gdb-auto]    This usually means the efi_main address changed.")
                print(f"[gdb-auto]    Re-run without --addr to probe again.")
                child.sendcontrol("c")
                child.expect(GDB_PROMPT_RE.encode(), timeout=10)
                gdb_print("info registers rip")
            else:
                print("[gdb-auto] ❌ GDB exited unexpectedly (EOF)")

            gdb("quit")

    except pexpect.exceptions.TIMEOUT as e:
        print(f"[gdb-auto] ❌ pexpect timeout: {e}")
        sys.exit(1)
    except pexpect.exceptions.EOF:
        # Clean exit from quit
        pass
    except KeyboardInterrupt:
        print("\n[gdb-auto] Interrupted.")
        child.sendcontrol("c")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--tmux", default=DEFAULT_TMUX, metavar="SESSION",
        help=f"tmux session name (default: {DEFAULT_TMUX})",
    )
    parser.add_argument(
        "--qemu-pane", type=int, default=DEFAULT_QEMU_PANE, metavar="N",
        help=f"tmux pane index for QEMU (default: {DEFAULT_QEMU_PANE})",
    )
    parser.add_argument(
        "--gdb-pane", type=int, default=DEFAULT_GDB_PANE, metavar="N",
        help=f"tmux pane index for GDB output (default: {DEFAULT_GDB_PANE})",
    )
    parser.add_argument(
        "--timeout-addr", type=int, default=DEFAULT_TIMEOUT_ADDR, metavar="SECS",
        help=f"Max seconds to wait for efi_main address from debugcon "
             f"(default: {DEFAULT_TIMEOUT_ADDR})",
    )
    parser.add_argument(
        "--timeout-boot", type=int, default=DEFAULT_TIMEOUT_BOOT, metavar="SECS",
        help=f"Max seconds to wait for breakpoint hit after continue "
             f"(default: {DEFAULT_TIMEOUT_BOOT})",
    )
    parser.add_argument(
        "--addr", metavar="0x...",
        help="Skip probe run and use this efi_main address directly",
    )
    parser.add_argument(
        "--no-interactive", action="store_true",
        help="Run non-interactively: continue, check breakpoint, quit",
    )
    args = parser.parse_args()

    # Validate workspace
    for path in (SYMBOL_FILE, GDB_SCRIPT, OVMF_CODE, OVMF_VARS, DISK_IMG):
        if not os.path.exists(path):
            sys.exit(f"ERROR: required file not found: {path}\n"
                     f"Run 'make all' first.")

    ensure_tmux_session(args.tmux)

    # Step 1: get efi_main runtime address
    if args.addr:
        addr = args.addr
        print(f"[gdb-auto] Using provided address: {addr}")
    else:
        addr = probe_efi_main_address(
            session=args.tmux,
            qemu_pane=args.qemu_pane,
            timeout=args.timeout_addr,
        )
        if addr is None:
            sys.exit(1)

    # Step 2: debug session
    run_debug_session(
        session=args.tmux,
        qemu_pane=args.qemu_pane,
        gdb_pane=args.gdb_pane,
        efi_main_addr=addr,
        timeout_boot=args.timeout_boot,
        interactive=not args.no_interactive,
    )


if __name__ == "__main__":
    main()
