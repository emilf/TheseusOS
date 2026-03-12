# TheseusOS Documentation Wiki

This is the architectural front page for the current TheseusOS repository.

Use it in this order:

- Start with [`docs/_inventory.md`](_inventory.md) if you need evidence-only implementation facts.
- Read [`docs/axioms/`](axioms/) for binding truths the current code relies on.
- Read [`docs/plans/`](plans/) for evolving work, mismatch notes, and risks.
- Use [`docs/map.md`](map.md) to jump between plans, axioms, and implementing modules.
- Read the repo rules in [`../AGENTS.md`](../AGENTS.md) before making architectural changes.
- Use [`development-and-debugging.md`](development-and-debugging.md) for build/debug workflow details.

## High-Level Overview

TheseusOS currently boots as a single UEFI executable. The bootloader gathers firmware and platform state, copies a handoff structure into persistent memory, exits boot services, and directly calls `kernel_entry`. The kernel then establishes a new paging environment, switches into the higher half, initializes heaps and the persistent physical allocator, brings up interrupt/platform services, and exposes runtime inspection through logging and the monitor.

## Core Navigation

### Evidence

- [`_inventory.md`](_inventory.md) — evidence-only repository inventory built from current code.

### Axioms

- [`axioms/boot.md`](axioms/boot.md) — boot-flow truths.
- [`axioms/memory.md`](axioms/memory.md) — paging, heap, and physical-memory truths.
- [`axioms/arch-x86_64.md`](axioms/arch-x86_64.md) — CPU, descriptor-table, and interrupt-controller truths.
- [`axioms/debug.md`](axioms/debug.md) — logging, panic, and runtime-inspection truths.

### Plans

- [`plans/boot-flow.md`](plans/boot-flow.md)
- [`plans/memory.md`](plans/memory.md)
- [`plans/interrupts-and-platform.md`](plans/interrupts-and-platform.md)
- [`plans/observability.md`](plans/observability.md)
- [`plans/drivers-and-io.md`](plans/drivers-and-io.md)

### Existing Narrative Docs

These remain useful as human-oriented guides, but they are subordinate to the axioms/plans split above:

- [`overview.md`](overview.md) — project goals, architecture snapshot, and suggested learning paths.
- [`boot-sequence.md`](boot-sequence.md) — how the UEFI loader prepares the machine and hands control to the kernel.
- [`kernel-architecture.md`](kernel-architecture.md) — tour of the major subsystems that come online during `kernel_entry`.
- [`memory-management.md`](memory-management.md) — virtual layout, frame allocation, and paging helpers.
- [`hardware-and-drivers.md`](hardware-and-drivers.md) — device inventory, driver manager, serial stack, interrupts, and newer driver subsystems.
- [`development-and-debugging.md`](development-and-debugging.md) — building, running under QEMU, logging, and the serial monitor.
- [`qemu-runner.md`](qemu-runner.md) — the `theseus-qemu` Rust runner and its reproducible QEMU profiles.
- [`logging.md`](logging.md) — current logging behavior and routing notes.

Bootloader-side observability and output plumbing now also lives in the new architecture split (see `plans/boot-flow.md`, `plans/observability.md`, and `axioms/debug.md`), rather than only in the older narrative guides. The same is now true for the bootloader-side memory/handoff story via `plans/memory.md` and `axioms/memory.md`.

### Historical Material

- [`archive/`](archive/) — older notes and superseded design/docs. Useful context, not current authority.
- `archive/driver-systems-deep-dive.md` — detailed reference dump moved out of the front-door docs surface.
- `archive/openclaw-heartbeat-and-tests.md` — archived workflow memo, not current repo authority.
- `archive/CODE_IMPROVEMENT_RECOMMENDATIONS.md` — stale progress/recommendation log archived out of the repo root.
- `archive/DRIVER_OUTPUT.md` — older output-driver/QEMU-path guide archived out of the repo root.
- `archive/WORKSPACE_SETUP.md` — older single-binary workspace explainer archived out of the repo root.
- `archive/TEMPORARY_HEAP_SYSTEM.md` — temporary-heap explainer archived out of the repo root; current truth lives in the memory plans/axioms.
