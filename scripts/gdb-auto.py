#!/usr/bin/env python3
"""
gdb-auto.py — One-command GDB debug session for TheseusOS.

Usage:
    python3 scripts/gdb-auto.py [--tmux SESSION] [--timeout-boot SECS]
                                 [--qemu-pane 0] [--gdb-pane 1]

Workflow:
    1. Start QEMU paused (-S) with a unix-socket GDB stub in a tmux pane.
    2. Spawn GDB via pexpect, source debug.gdb, connect.
    3. Run 'theseus-auto' — a GDB Python command that:
         a. Sets a hardware watchpoint on the debug mailbox sentinel
            (physical address 0x7008, written by efi_main on entry).
         b. Continues — UEFI boots and efi_main writes its runtime address
            to 0x7000, then writes the magic sentinel to 0x7008.
         c. Watchpoint fires; theseus-auto reads the address from 0x7000,
            calls theseus-load with it, continues to the efi_main breakpoint.
    4. Drops into interactive GDB (or exits cleanly for --no-interactive CI).

Key property: single QEMU session, no probe-then-restart, address is always
correct because it comes from the running binary itself.

Every wait has a hard timeout — the script never hangs silently.

Requirements:
    pip install pexpect       (or: pip install --break-system-packages pexpect)
    Kernel built with debug mailbox support (shared/src/constants.rs::debug_mailbox)

Environment:
    Designed to run inside the bwrap sandbox that OpenClaw uses.
    QEMU is kept alive in a tmux pane (survives sandbox exec sessions).
    GDB is driven via pexpect so Ctrl-C reliably reaches the remote target.
"""

import argparse
import os
import subprocess
import sys
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
GDB_SOCKET      = "/tmp/theseus-gdb-auto.sock"  # unused: kept for reference
GDB_TARGET      = "localhost:1251"               # TCP port — survives system_reset
GDB_SCRIPT      = os.path.join(WORKSPACE, "debug.gdb")
SYMBOL_FILE     = os.path.join(WORKSPACE, "build", "BOOTX64.SYM")
OVMF_CODE       = os.path.join(WORKSPACE, "OVMF", "OVMF_CODE.fd")
OVMF_VARS       = os.path.join(WORKSPACE, "build", "OVMF_VARS.fd")
DISK_IMG        = os.path.join(WORKSPACE, "build", "disk.img")

GDB_PROMPT_RE   = r"\(gdb\)"

DEFAULT_TMUX    = "theseus"
DEFAULT_TIMEOUT_BOOT  = 120   # seconds to wait for mailbox + breakpoint
DEFAULT_QEMU_PANE     = 0
DEFAULT_GDB_PANE      = 1


# ---------------------------------------------------------------------------
# QEMU command builder
# ---------------------------------------------------------------------------
def qemu_cmd(paused: bool, gdb_target: str, debugcon_log: str) -> list[str]:
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
        # GDB stub via TCP — survives system_reset (unlike unix sockets)
        "-gdb", f"tcp::{gdb_target.split(':')[-1]}",
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
# Main debug session — single QEMU run, mailbox watchpoint approach
# ---------------------------------------------------------------------------
def run_debug_session(
    session: str,
    qemu_pane: int,
    gdb_pane: int,
    timeout_boot: int,
    interactive: bool,
):
    # Clean up stale socket
    try:
        os.unlink(GDB_SOCKET)
    except FileNotFoundError:
        pass
    try:
        os.unlink(DEBUGCON_LOG)
    except FileNotFoundError:
        pass

    # Start QEMU running (not paused) — the mailbox watchpoint will halt it
    # automatically when efi_main writes the sentinel. No -S needed.
    cmd = " ".join(qemu_cmd(paused=False, gdb_target=GDB_TARGET,
                             debugcon_log=DEBUGCON_LOG))
    print(f"[gdb-auto] Starting QEMU (running, watchpoint will halt at efi_main)...")
    tmux_kill_pane_process(session, qemu_pane)
    time.sleep(1)
    tmux_send(session, qemu_pane, f"cd {WORKSPACE} && {cmd}")

    # Wait for GDB TCP port to be ready
    import socket as _socket
    host, port = GDB_TARGET.rsplit(":", 1)
    deadline = time.time() + 15
    while time.time() < deadline:
        try:
            s = _socket.create_connection((host, int(port)), timeout=1)
            s.close()
            break
        except (ConnectionRefusedError, OSError):
            time.sleep(0.5)
    else:
        sys.exit(f"ERROR: QEMU GDB port {GDB_TARGET} never opened — is QEMU starting correctly?")

    print(f"[gdb-auto] GDB socket ready. Spawning GDB (pexpect)...")

    child = pexpect.spawn(
        "gdb",
        cwd=WORKSPACE,
        encoding=None,
        timeout=30,
        logfile=open("/tmp/gdb-auto-raw.log", "wb"),
    )

    def gdb_cmd(cmd: str, timeout: int = 15) -> str:
        child.sendline(cmd.encode())
        child.expect(GDB_PROMPT_RE.encode(), timeout=timeout)
        out = child.before.decode(errors="replace").strip()
        return out

    def gdb_print(cmd: str, timeout: int = 15):
        out = gdb_cmd(cmd, timeout=timeout)
        if out:
            lines = [l for l in out.splitlines()
                     if l.strip() and l.strip() != cmd.strip()]
            for l in lines:
                print(f"  {l}")
        return out

    try:
        child.expect(GDB_PROMPT_RE.encode(), timeout=15)
        print("[gdb-auto] GDB started")

        gdb_cmd("set pagination off")
        gdb_cmd("set confirm off")
        gdb_cmd("set architecture i386:x86-64")
        gdb_cmd("set demangle-style rust")
        gdb_cmd(f"symbol-file {SYMBOL_FILE}")

        out = gdb_cmd(f"source {GDB_SCRIPT}", timeout=20)
        for line in out.splitlines():
            if any(tok in line for tok in ("Δ", "image_base", "efi_main link")):
                print(f"  {line}")

        print(f"[gdb-auto] Connecting to QEMU ({GDB_TARGET})...")
        # Tell theseus-auto which target to use for post-reset reconnect
        gdb_cmd(f'python gdb.set_convenience_variable("_gdb_target", "{GDB_TARGET}")')
        gdb_cmd(f"target remote {GDB_TARGET}", timeout=15)

        rip_check = gdb_cmd("info registers rip")
        rip = next((l for l in rip_check.splitlines() if "rip" in l), "")
        if "0xfff0" in rip:
            print(f"[gdb-auto] ✅ Confirmed halted at reset vector (rip=0xfff0)")
        else:
            print(f"[gdb-auto] ⚠️  Unexpected RIP after connect: {rip}")

        # Run theseus-auto — fully automated sequence:
        #   1. Sets hw watchpoint on mailbox sentinel (0x7008)
        #   2. Continues → UEFI boots → efi_main writes mailbox → watchpoint fires
        #   3. Reads runtime address from 0x7000, calls theseus-load
        #   4. Issues monitor system_reset, reconnects, continues
        #   5. UEFI reboots → efi_main runs again → hits the sw breakpoint
        print(f"[gdb-auto] Running theseus-auto (timeout {timeout_boot}s)...")
        print(f"[gdb-auto]   Watching mailbox sentinel at 0x7008 for "
              f"magic 0xDEADBEEFCAFEF00D...")

        child.sendline(b"theseus-auto")

        # theseus-auto internally calls gdb.execute("continue") twice, each
        # of which blocks until GDB stops. The pexpect expect() here waits for
        # the final (gdb) prompt that appears after the efi_main breakpoint hit.
        # The full timeout covers both the first boot (mailbox write) and the
        # second boot (breakpoint hit) so multiply by 2 for safety.
        idx = child.expect(
            [GDB_PROMPT_RE.encode(), pexpect.TIMEOUT, pexpect.EOF],
            timeout=timeout_boot * 2,
        )
        output = child.before.decode(errors="replace")

        if idx == 1:
            print(f"[gdb-auto] ⏰ Timeout ({timeout_boot*2}s) waiting for theseus-auto")
            print(f"[gdb-auto]    Check: was the kernel built with debug mailbox support?")
            print(f"[gdb-auto]    Check: does UEFI reach efi_main within the timeout?")
            child.sendcontrol("c")
            try:
                child.expect(GDB_PROMPT_RE.encode(), timeout=10)
            except Exception:
                pass
            gdb_print("info registers rip")
            gdb_cmd("quit")
            return
        elif idx == 2:
            print("[gdb-auto] ❌ GDB exited unexpectedly (EOF)")
            return

        # idx == 0: theseus-auto completed and returned a prompt
        print(f"[gdb-auto] theseus-auto output:")
        for line in output.splitlines():
            if line.strip():
                print(f"  {line}")

        if "Breakpoint" in output and "efi_main" in output and "failed to reconnect" not in output:
            print(f"[gdb-auto] ✅ BREAKPOINT HIT at efi_main!")
        elif "failed to reconnect" in output:
            print(f"[gdb-auto] ⚠️  Reconnect after reset failed — see output above")
        elif "mailbox fired" in output:
            print(f"[gdb-auto] ✅ Mailbox fired — address captured")
        else:
            print(f"[gdb-auto] ⚠️  theseus-auto completed but breakpoint status unclear")

        if interactive:
            print()
            print("[gdb-auto] ─────────────────────────────────────────────────")
            print("[gdb-auto]  Dropping into interactive GDB.")
            print("[gdb-auto]  Symbols loaded, stopped at efi_main.")
            print(f"[gdb-auto]  Ctrl-C to interrupt, 'q' to quit.")
            print("[gdb-auto] ─────────────────────────────────────────────────")
            child.interact()
        else:
            gdb_print("info registers rip")
            gdb_print("backtrace 5")
            gdb_cmd("quit")

    except pexpect.exceptions.TIMEOUT as e:
        print(f"[gdb-auto] ❌ Unexpected pexpect timeout: {e}")
        sys.exit(1)
    except pexpect.exceptions.EOF:
        pass  # clean GDB exit
    except KeyboardInterrupt:
        print("\n[gdb-auto] Interrupted.")
        try:
            child.sendcontrol("c")
        except Exception:
            pass


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
        "--timeout-boot", type=int, default=DEFAULT_TIMEOUT_BOOT, metavar="SECS",
        help=f"Max seconds to wait for mailbox watchpoint + breakpoint "
             f"(default: {DEFAULT_TIMEOUT_BOOT})",
    )
    parser.add_argument(
        "--no-interactive", action="store_true",
        help="Run non-interactively: check breakpoint then quit (CI mode)",
    )
    args = parser.parse_args()

    # Validate workspace artifacts exist
    for path in (SYMBOL_FILE, GDB_SCRIPT, OVMF_CODE, OVMF_VARS, DISK_IMG):
        if not os.path.exists(path):
            sys.exit(f"ERROR: required file not found: {path}\n"
                     f"Run 'make all' first.")

    ensure_tmux_session(args.tmux)

    run_debug_session(
        session=args.tmux,
        qemu_pane=args.qemu_pane,
        gdb_pane=args.gdb_pane,
        timeout_boot=args.timeout_boot,
        interactive=not args.no_interactive,
    )


if __name__ == "__main__":
    main()
