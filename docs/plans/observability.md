# Observability Plan

## Scope

Track logging, panic reporting, runtime inspection, and debugging surfaces.

## Plan vs Repo

- Current docs already explain logging and monitor usage fairly well, but they were still mostly narrative docs rather than architecture-tracking docs.
- After the bootloader driver/output sweep, observability in this repo is clearly not just a kernel concern: firmware-side output selection, serial helpers, and QEMU-exit behavior are also part of the real debug surface.

## Implemented Invariants

- [x] IMPLEMENTED: Kernel logging is initialized at the start of `kernel_entry`.
- [x] IMPLEMENTED: Logging output is designed to work without heap allocation and remain usable in interrupt and panic contexts.
- [x] IMPLEMENTED: Panic handling logs failure details and exits QEMU with an error status.
- [x] IMPLEMENTED: The kernel monitor exposes live inspection for tables, memory, ACPI, CPU state, devices, APIC mode/base state, and log-output routing.
- [x] IMPLEMENTED: Monitor input handling stays split between cheap IRQ-fed serial byte queuing and later command processing in monitor context.
- [x] IMPLEMENTED: The development workflow uses QEMU debug output and serial as first-class observability channels.
- [x] IMPLEMENTED: Bootloader-side output selection currently prefers the QEMU debug-port path for normal development/test runs, with firmware serial and raw UART support treated as fallback observability helpers.
- [x] IMPLEMENTED: Bootloader-side output selection, serial helpers, and QEMU-exit helpers are also part of the current observability surface before `ExitBootServices`.
- [x] IMPLEMENTED: The repo now includes a dedicated `theseus-qemu` runner that can print, artifact, and execute reproducible QEMU argv with explicit debug/relay endpoint controls.

## TODO

- [ ] TODO: Fold any durable pieces of the existing narrative debugging docs into module headers or module overview pages where they belong.
- [x] IMPLEMENTED: `BUILD.md` has been kept in the repo root but updated to describe the Rust QEMU runner as the preferred current workflow and to de-authorize itself as an architecture source.
- [x] IMPLEMENTED: `TEMPORARY_HEAP_SYSTEM.md` has been moved out of the repo root into `docs/archive/` because it was explanatory historical material, not current authority.
- [x] IMPLEMENTED: `README.md` has been rewritten to match the current single-binary architecture and docs front door.
- [x] IMPLEMENTED: `CODE_IMPROVEMENT_RECOMMENDATIONS.md` has been moved out of the repo root into `docs/archive/` because it was a stale progress/recommendation log, not current authority.
- [x] IMPLEMENTED: `DRIVER_OUTPUT.md` and `WORKSPACE_SETUP.md` have been moved out of the repo root into `docs/archive/` because they were explanatory historical docs, not current authority.
- [ ] TODO: Add scoped module-overview docs if monitor/debugging code grows more fragmented.

## Risks

- [!] RISK: Historical docs in `docs/archive/` contain useful context but are not authoritative and may contradict current behavior if read as current truth.
- [!] RISK: If docs describe logging routing more strongly than the code/config currently enforces, they will drift into fiction quickly.
- [!] RISK: The bootloader driver-selection policy is intentionally simple and QEMU-biased today; wording that implies adaptive hardware detection would overstate the implementation.

## Related Axioms

- `../axioms/debug.md#A1:-Kernel-logging-is-initialized-at-kernel-entry-and-is-designed-to-work-without-heap-allocation`
- `../axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status`
- `../axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface`

## Implementing Modules

- `bootloader/src/display.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `bootloader/src/drivers/mod.rs`
- `bootloader/src/drivers/manager.rs`
- `bootloader/src/drivers/qemu_debug.rs`
- `bootloader/src/drivers/raw_serial.rs`
- `bootloader/src/drivers/uefi_serial.rs`
- `kernel/src/lib.rs`
- `kernel/src/logging/mod.rs`
- `kernel/src/logging/output.rs`
- `kernel/src/logging/filter.rs`
- `kernel/src/logging/macros.rs`
- `kernel/src/panic.rs`
- `kernel/src/display.rs`
- `kernel/src/framebuffer.rs`
- `kernel/src/bootlogo.rs`
- `kernel/src/monitor/mod.rs`
- `kernel/src/monitor/parsing.rs`
- `kernel/src/monitor/commands/*`
- `docs/development-and-debugging.md`
- `docs/logging.md`
- `docs/qemu-runner.md`
- `tools/theseus-qemu/src/main.rs`
