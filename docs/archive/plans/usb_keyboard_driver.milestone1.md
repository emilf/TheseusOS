# USB Keyboard Driver – Milestone 1 (PCI Discovery)

Milestone 1 establishes a clean PCI discovery layer that future USB work can build on. The focus was to rely entirely on ACPI-provided ECAM windows, walk the bus hierarchy accurately, and expose only meaningful devices (e.g. xHCI controllers) to the driver subsystem.

## Completed Deliverables
- **Pure ECAM access**: `kernel/src/acpi/mod.rs:314` parses the MCFG table, maps each ECAM window uncached, and exposes the regions through `PlatformInfo::pci_config_regions`, eliminating the legacy 0xCF8/0xCFC path.
- **Bridge-aware enumeration**: `kernel/src/drivers/pci.rs:110` performs a depth-first walk of the PCI hierarchy via secondary/subordinate bus numbers, logging bridges at trace level and emitting every reachable function exactly once. Bridge metadata is cached on `PlatformInfo::pci_bridges` for later diagnostics.
- **Bridge bring-up**: `kernel/src/drivers/pci.rs:270` enables I/O/MEM/bus-mastering on each bridge, assigns fresh secondary bus numbers, pulses secondary reset, and now programs I/O/memory/prefetch windows to match the exact range consumed by the downstream devices.
- **Driver-manager integration**: `kernel/src/drivers/system.rs:96` consumes the enumerator, classifies functions (USB/storage/network), skips bridges, and registers actionable devices with the driver manager so upcoming USB code can bind cleanly.
- **BAR/IRQ metadata + capabilities**: BAR decoding captures the first MMIO BAR and any assigned interrupt line, and `PciDeviceInfo` now records MSI/MSI-X capability bits for future interrupt routing work.
- **BAR relocation + bridge windows**: During enumeration every non-bridge function now receives a freshly allocated BAR aperture (IO, MEM, prefetchable MEM) with decode temporarily disabled to avoid device side effects. Bridge windows (I/O, MEM, prefetch) are then programmed to cover the exact span of their children, so we no longer rely on fixed 16 MiB placeholders.
- **Driver resource handles**: Each registered PCI device now carries a list of I/O and memory apertures (base + size + prefetch flag) so future drivers can directly map the hardware resources they need without re-decoding BARs.
- **Monitor visibility**: The `devices` monitor command prints out these resource windows per device, along with the programmed bridge windows, making it easy to audit allocations at runtime.
- **Driver resource helpers**: `Device` now exposes convenience helpers for querying memory and I/O apertures so upcoming drivers can claim the regions they need without re-decoding BARs.
- **PCI monitor command**: Added a `pci` monitor command that re-enumerates functions/bridges on demand and prints BAR resources for quick topology inspection.
- **Legacy ownership diagnostics**: The `pci` monitor command now reports FADT legacy handoff details and any ACPI `XHCI` descriptors so firmware state is visible in the console (`kernel/src/monitor/commands/pci.rs:12`).
- **QEMU topology**: `startQemu.sh` now binds `qemu-xhci` on the root bus so the virtual controller shows up during development and lighting tests (`startQemu.sh:92`).
- **xHCI reset coverage**: The driver performs the recommended halt/reset sequence and logs `controller reset complete`, confirming the modern initialization path is active during boot (`kernel/src/drivers/usb/xhci/mod.rs:250`).
- **MSI helper**: Introduced `pci::enable_msi` so future drivers can flip devices over to message-signalled interrupts without duplicating capability plumbing.
- **DMA buffer helper**: `kernel/src/memory/dma.rs:1` wraps the contiguous allocator to provide aligned, zeroed, kernel-mapped buffers suitable for xHCI command/event rings or other DMA-heavy peripherals.
- **Firmware handoff hooks**: `kernel/src/acpi/mod.rs:205` now captures FADT legacy ownership knobs and any ACPI `XHCI` descriptors, while `kernel/src/drivers/system.rs:155` invokes `usb::ensure_legacy_usb_handoff` so BIOS-owned controllers are released before drivers bind.
- **Ring staging**: The nascent xHCI driver allocates DMA-backed command/event rings and logs their physical addresses so later milestones can plug in TRB programming quickly (`kernel/src/drivers/usb/xhci/mod.rs:300`).

## Outstanding Tasks
1. **Interrupt model upgrade**: Integrate the MSI helper with the interrupt allocator (vector reservation, masks, teardown) and provide IO-APIC fallback policies.
2. **Diagnostics tooling**: Round out the `pci` monitor output with capability summaries and surface the new FADT/legacy handoff state so bring-up issues are easier to spot.

## Testing & Verification
- Sandbox runs reach the `=== Kernel environment setup complete ===` marker; serial/monitor pipes are intentionally disabled, so the harness exits immediately afterwards.
- On a developer workstation, run `./startQemu.sh headless 15`. You should now see the `qemu-xhci` controller (class `0c0330`) bind and emit capability logs such as `xHCI 0000:03:00.0 HCI v0.0 slots=65 ports=8 context_size=32`.
- With the new legacy handoff path, expect logs confirming OS ownership (or warnings if BIOS refuses to release); these appear before the xHCI capability dump.

## Next Milestone Preview
- Feed the refined PCI class information into driver binding so xHCI and storage drivers can auto-attach.
- Prototype an xHCI init stub that touches the new DMA helper and verifies ECAM reads/writes succeed.
- Implement MSI/MSI-X setup and fallback logic so USB interrupts no longer depend on legacy IO-APIC routing.
