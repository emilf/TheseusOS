# USB Keyboard Driver – Milestone 1 (PCI Discovery)

Milestone 1 called for a baseline PCI discovery layer that future USB work can rely on. The kernel now owns the legacy configuration space path and feeds discovered devices into the driver manager, so later stages can match on class/subclass instead of relying solely on UEFI inventory dumps.

## New Capabilities
- Added `kernel/src/drivers/pci.rs` with an ECAM-aware config-space reader, BAR decoder, and a full bus/device/function enumerator (no legacy 0xCF8/0xCFC path).
- `kernel/src/acpi/mod.rs:314` now parses MCFG entries, maps the ECAM windows once, and publishes them through `PlatformInfo::pci_config_regions`, so other subsystems can reach configuration space safely.
- The driver system wires the enumerator into initialization (`kernel/src/drivers/system.rs:96`), logging each function and registering it as a `DeviceId::Pci` so the driver manager can route future PCI-aware drivers.
- Basic BAR analysis identifies the first memory-mapped region for each function and records interrupt line wiring when present, giving upcoming xHCI work the MMIO base and IRQ number without additional plumbing.
- The persistent physical allocator gained `alloc_contiguous` / `free_contiguous` (`kernel/src/physical_memory.rs:708`) to supply aligned DMA buffers for things like xHCI rings and scratchpads.

## Outstanding Work
- Bridge/CardBus headers still skip BAR decoding; we should extend the parser to expose secondary bus numbers and windows.
- We need a policy for mapping PCI class codes to `DeviceClass` variants so USB/xHCI functions can advertise themselves to the driver manager without bespoke probe code.
- Interrupt routing still relies on IO APIC programming; MSI/MSI-X support will be required before the xHCI driver can rely on edge-triggered events.

## Testing Notes
- QEMU headless runs continue to fail inside the sandbox because `startQemu.sh` cannot create its `/tmp/qemu-*` pipes; rerun locally to confirm the new PCI discovery logs list the expected `qemu-xhci` controller and its BAR/IRQ metadata.
