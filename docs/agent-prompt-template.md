# Agent Prompt Template — TheseusOS

Use this as the base when spawning a coding agent on TheseusOS.
Fill in `[TASK]` and adjust the verification checklist items for the specific change.

---

## Template

```
You are working on TheseusOS, a Rust x86_64 hobby OS kernel.
Repo: /home/emil/.openclaw/workspace/TheseusOS

BEFORE WRITING ANY CODE:
1. Read AGENTS.md in the repo root — it contains binding rules for all agents working here
2. Read docs/axioms/ files relevant to your task area (memory, boot, arch-x86_64, debug)
3. Read docs/plans/ files relevant to your task area to understand current status and risks
4. Read the source files you'll be changing — understand what's actually there before touching it

YOUR TASK:
[TASK]

CONSTRAINTS:
- Work on a feature branch, not main
- Keep diffs small and focused — one concern per commit
- Do not add log spam (read docs/logging.md for policy)
- Do not introduce unsafe code without documenting the invariant it relies on
- Do not call mapping.rs helpers post-boot — they are boot-time only
- Do not add driver_data raw pointer casts — they are being replaced (see AGENTS.md §14)
- All VA regions are hardcoded constants — do not invent dynamic mappings

VERIFICATION — do all of these before declaring done:
1. `cargo run -p theseus-qemu -- --headless` boots cleanly with no new ERRORs or WARNs
2. [task-specific check, e.g. "APIC timer still fires", "keyboard events still arrive", "memory monitor shows sane state"]
3. Updated the relevant docs/plans/ checklist item
4. Opened a PR with a clear description of what changed and why

Prove to me this works — show the boot log output, not just that it compiled.
```

---

## Notes on filling it in

**For memory/paging changes:**
Add to verification: "No page faults at boot. `memory` monitor command shows expected state."

**For interrupt/APIC changes:**
Add to verification: "APIC timer ISR still fires (heart blinks in headed mode or tick counter increments via monitor)."

**For driver/PCI changes:**
Add to verification: "USB keyboard still works end-to-end. `devices list` monitor output is correct."

**For doc-only changes:**
Remove the cargo run verification requirement, but still ask for a review of the doc for accuracy against current code.

---

## QEMU quick reference (include in prompt if agent needs to run QEMU)

```
Run: cargo run -p theseus-qemu -- --headless
With relays: cargo run -p theseus-qemu -- --relays --headless
With display: cargo run -p theseus-qemu -- --headed

Relay sockets (PTY files, use `cat` not `tail -f`):
  Serial:   /tmp/qemu-serial-host
  Monitor:  /tmp/qemu-monitor-host
  Debugcon: /tmp/qemu-debugcon-host
  QMP:      /tmp/qemu-qmp-host.sock

GDB: make debug-auto (handles higher-half symbol offsets automatically)
```
