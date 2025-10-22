# USB Keyboard Driver – Milestone 1 (PCI Discovery)

Milestone 1 establishes a clean PCI discovery layer that future USB work can build on. The focus was to rely entirely on ACPI-provided ECAM windows, walk the bus hierarchy accurately, and expose only meaningful devices (e.g. xHCI controllers) to the driver subsystem.

## Completed Deliverables
- **Pure ECAM access**: `kernel/src/acpi/mod.rs:314` parses the MCFG table, maps each ECAM window uncached, and exposes the regions through `PlatformInfo::pci_config_regions`, eliminating the legacy 0xCF8/0xCFC path.
- **Bridge-aware enumeration**: `kernel/src/drivers/pci.rs:110` performs a depth-first walk of the PCI hierarchy via secondary/subordinate bus numbers, logging bridges at trace level and emitting every reachable function exactly once. Bridge metadata is cached on `PlatformInfo::pci_bridges` for later diagnostics.
- **Driver-manager integration**: `kernel/src/drivers/system.rs:96` consumes the enumerator, classifies functions (USB/storage/network), skips bridges, and registers actionable devices with the driver manager so upcoming USB code can bind cleanly.
- **BAR/IRQ metadata + capabilities**: BAR decoding captures the first MMIO BAR and any assigned interrupt line, and `PciDeviceInfo` now records MSI/MSI-X capability bits for future interrupt routing work.
- **DMA buffer helper**: `kernel/src/memory/dma.rs:1` wraps the contiguous allocator to provide aligned, zeroed, kernel-mapped buffers suitable for xHCI command/event rings or other DMA-heavy peripherals.

## Outstanding Tasks
1. **Bridge bus programming**: Firmware leaves several root ports with identical secondary/subordinate numbers; we need to assign fresh bus numbers so downstream controllers (e.g. `qemu-xhci`) become visible during enumeration.
2. **Class-to-driver mapping**: Extend the classifier to cover more subclasses (AHCI, NVMe, USB companion controllers) and wire those classes into future drivers.
3. **Interrupt model upgrade**: Add MSI/MSI-X capability enablement; the current IO-APIC routing works but is not ideal for a high-throughput xHCI driver.
4. **Diagnostics tooling**: Expand the new `devices` monitor output (already shows bridges) into a full PCI topology dump, including BAR sizes and capability lists.

## Testing & Verification
- Sandbox runs reach the `=== Kernel environment setup complete ===` marker; serial/monitor pipes are intentionally disabled, so the harness exits immediately afterwards.
- On a developer workstation, run `./startQemu.sh headless 15` and confirm the logs contain `PCI scan:` and the bead of `Registering USB controller:` once bridge bus numbers are configured.

## Next Milestone Preview
- Feed the refined PCI class information into driver binding so xHCI and storage drivers can auto-attach.
- Prototype an xHCI init stub that touches the new DMA helper and verifies ECAM reads/writes succeed.
- Implement MSI/MSI-X setup and fallback logic so USB interrupts no longer depend on legacy IO-APIC routing.
