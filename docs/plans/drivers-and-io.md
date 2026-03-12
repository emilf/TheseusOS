# Drivers and I/O Plan

## Scope

Track the current driver framework, PCI enumeration, USB controller handoff/bring-up, and input-facing runtime inspection surfaces.

## Plan vs Repo

- The merged repo now contains materially more driver-system reality than the older docs reflected: PCI bridge refinement, USB/xHCI code, keyboard input modules, and monitor commands for PCI/USB inspection.
- This plan records what is already true without pretending the full driver stack is feature-complete.
- The current repo has enough real PCI/USB/input/runtime-inspection behavior that the driver layer must be documented as architecture, not just experimental scaffolding.

## Implemented Invariants

- [x] IMPLEMENTED: The kernel has a central `DriverManager` that owns driver registration, discovered-device tracking, probe/bind flow, IRQ dispatch, and class-based I/O helpers.
- [x] IMPLEMENTED: PCI device discovery is ECAM-based from ACPI MCFG regions and includes bridge-aware bus scanning plus BAR/capability parsing.
- [x] IMPLEMENTED: The PCI path records MSI/MSI-X capability information and exposes helper paths for configuring message-signaled interrupts.
- [x] IMPLEMENTED: The USB subsystem currently exposes firmware handoff helpers and xHCI driver registration/runtime service hooks.
- [x] IMPLEMENTED: The kernel monitor now includes PCI/USB-oriented inspection commands in addition to the older system/memory/table views.
- [x] IMPLEMENTED: The repo includes a Rust QEMU runner at `tools/theseus-qemu` with named profiles and explicit relay/QMP/serial endpoint toggles.

## TODO

- [ ] TODO: Promote any stable cross-subsystem driver invariants into axioms once the driver stack depends on them broadly enough to be architectural truth instead of implementation shape.
- [ ] TODO: Document which USB/xHCI runtime paths are considered production bring-up versus active iteration/debugging helpers.
- [ ] TODO: Decide whether `hardware-and-drivers.md`, `qemu-runner.md`, and `logging.md` remain current front-door docs or should be further folded into the axioms/plans/wiki structure.
- [x] IMPLEMENTED: `driver-systems-deep-dive.md` has been moved out of the front-door docs surface into `docs/archive/` because it is useful reference material but not current repository authority.

## Risks

- [!] RISK: Driver and USB code is moving faster than the new axioms/plans scaffold, so docs can drift unless each merge updates this plan.
- [!] RISK: Some current module-level rustdoc is detailed but aspirational in tone; if code behavior changes, it can silently stop matching reality.

## Related Plans

- `boot-flow.md`
- `interrupts-and-platform.md`
- `observability.md`

## Implementing Modules

- `kernel/src/drivers/mod.rs`
- `kernel/src/drivers/manager.rs`
- `kernel/src/drivers/pci.rs`
- `kernel/src/drivers/system.rs`
- `kernel/src/drivers/traits.rs`
- `kernel/src/drivers/usb/mod.rs`
- `kernel/src/drivers/usb/handoff.rs`
- `kernel/src/drivers/usb/xhci/*`
- `kernel/src/input/*`
- `kernel/src/monitor/mod.rs`
- `kernel/src/monitor/commands/devices.rs`
- `kernel/src/monitor/commands/pci.rs`
- `kernel/src/monitor/commands/usb.rs`
- `tools/theseus-qemu/src/main.rs`
- `docs/hardware-and-drivers.md`
- `docs/qemu-runner.md`
- `docs/logging.md`
- `docs/archive/driver-systems-deep-dive.md`
