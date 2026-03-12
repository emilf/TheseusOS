# TheseusOS

TheseusOS is a teaching-focused x86_64 operating system written in Rust.

The current repository boots as a **single UEFI executable**: the bootloader gathers firmware and platform state, copies a handoff structure into persistent memory, exits boot services, and then directly calls the kernel entry point inside the same unified binary.

This repo is also explicitly a learning project, so some docs and subsystems are in motion. For binding architectural truth, prefer:

- `docs/index.md`
- `docs/axioms/`
- `docs/plans/`
- `docs/map.md`

## Current Architecture Snapshot

TheseusOS currently consists of three workspace crates:

1. **bootloader** — UEFI entrypoint and boot-time discovery/transition logic
2. **kernel** — higher-half kernel bring-up, runtime subsystems, and device support
3. **shared** — handoff ABI, shared constants, and utility code used across boot phases

## Current Capabilities

- Single-binary UEFI boot flow with direct `kernel_entry` handoff
- Boot-time collection of graphics, ACPI, firmware, memory-map, CPU, and hardware-inventory data
- Higher-half kernel transition with explicit paging setup and `PHYS_OFFSET`
- Temporary-heap to permanent-heap allocator transition
- APIC-based interrupt bring-up with GDT/TSS/IDT installation
- Driver framework with PCI enumeration, USB/xHCI bring-up, and serial/monitor tooling
- QEMU workflows via the in-repo `theseus-qemu` runner plus older helper scripts still present for compatibility

## Build and Run

Preferred current path:

```bash
# Headless run via the Rust QEMU runner
cargo run -p theseus-qemu -- --headless

# Print the resolved QEMU command
cargo run -p theseus-qemu -- print --headless
```

Older compatibility/convenience paths still exist:

```bash
make run
make run-headed
./startQemu.sh headless
```

See `BUILD.md` and `docs/development-and-debugging.md` for workflow details.

## Documentation Guide

- Start at `docs/index.md`
- Use `docs/_inventory.md` for evidence-only implementation facts
- Use `docs/axioms/` for binding truths
- Use `docs/plans/` for evolving work, mismatch notes, and risks
- Use `docs/archive/` for historical material that should not be treated as current authority

## Notes on Older Docs

Some older markdown files and historical writeups still exist in the repository. They may be useful reference material, but they are not automatically current just because they exist. The docs architecture under `docs/index.md` is the intended front door.
