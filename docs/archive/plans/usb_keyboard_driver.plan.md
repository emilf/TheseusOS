# USB Keyboard Driver Roadmap

## Context & Goals
- Bring up end-to-end support for a USB HID keyboard under the hobby OS kernel, targeting QEMU's `qemu-xhci` controller first and leaving hooks for real hardware.
- Fill current gaps: no PCI bus layer, no USB stack, no input event pipeline, and limited DMA-friendly memory allocation.
- Deliver a staged plan that sequences foundational infrastructure (PCI, ACPI handoff, DMA buffers), controller bring-up, USB protocol layers, and keyboard input integration.

## Status Snapshot
- [x] **Milestone 0 – Prerequisite Audit**: Baseline inventory captured; FADT-derived legacy USB knobs and XHCI descriptors are now mirrored in `PlatformInfo`.
- [x] **Milestone 1 – PCI Discovery**: ECAM enumeration integrated with the driver system, and USB controllers are claimed via the new legacy handoff path.
- [ ] **Milestone 2 – ACPI & Controller Handoff**: Firmware ownership negotiation and monitor diagnostics are in place; documentation and policy wiring remain.

## Milestone 0: Prerequisite Audit
- Confirm interrupt delivery story: document current APIC/IOAPIC usage, available vectors, and whether MSI/MSI-X is viable for early use; define the fallback legacy IRQ path.
- Inventory memory allocators: identify what is needed to hand out physically contiguous, cache-coherent buffers for xHCI rings (tie-in with the persistent frame allocator roadmap).
- Review bootloader hardware inventory (`HardwareDevice` records) to ensure USB/XHCI handles are discoverable and feed that into kernel-side logging for debugging.

## Milestone 1: PCI Discovery Baseline
- Implement a minimal PCI config accessor (ports `0xCF8/0xCFC` and later MMCONFIG via ACPI MCFG) under `kernel/src/drivers/system` with safe wrappers.
- Build a PCI enumerator that walks buses/devices/functions, records class/subclass, and registers devices with the driver manager using existing traits.
- Enable bus mastering and MMIO for class `0x0C/0x03` (Serial Bus / USB controller) devices, and expose BAR information plus interrupt capabilities to later stages.

## Milestone 2: ACPI & Controller Handoff Support
- ✅ Extend ACPI parsing to read XSDT/RSDT beyond MADT, focusing on finding the FADT (for legacy USB handoff) and the XHCI-specific Extended Capabilities Descriptor.
- ✅ Implement BIOS ownership/OS ownership handoff (EHCI legacy handoff and xHCI Extended Capabilities) to disable firmware emulation that might steal the controller.
- ✅ Surface MCFG ranges (if present) to let the PCI layer switch to memory-mapped config space for performance and to access devices above bus 255 if needed.
- ✅ Expose diagnostics/monitor hooks for the newfound legacy handoff state so firmware ownership can be audited quickly.
- ⬜ Document the firmware release procedure and codify policy wiring for MSI/IOAPIC fallback.

## Milestone 3: xHCI Host Controller Bring-Up
- ☐ Create an xHCI driver module that maps the controller MMIO region, parses capability/operational registers, and performs controller reset to a known state.
  - Stub driver now maps MMIO, performs the halt/reset handshake, reports capabilities, allocates command/event rings, programs CRCR/ERST for interrupter 0, stands up the DCBAA, enables the advertised slot count, and transitions the controller to RUN (`kernel/src/drivers/usb/xhci/mod.rs:1`); runtime bring-up remains (no TRBs yet).
  - QEMU launch script pins `qemu-xhci` to the root bus so the virtual controller reliably enumerates during bring-up (`startQemu.sh:92`).
- Allocate and initialize the command ring, event ring, and scratchpad buffers using DMA-capable physical memory; ensure virtual mappings stay uncached or write-back per spec.
- Program slot contexts and endpoint contexts for the default control endpoint, handling doorbell and interrupter configuration; route interrupts via MSI/MSI-X if possible, falling back to IOAPIC.

## Milestone 4: USB Device Enumeration Stack
- Implement basic USB request/response infrastructure (setup packets, control transfers, TRBs) sufficient to enumerate attached devices over the default control endpoint.
- Parse descriptors (device, configuration, interface, HID, endpoint) and select the correct configuration for HID keyboards; handle both full- and high-speed devices via the root hub.
- Build a lightweight hub management layer to service port status change events and create device slots for each attached keyboard.

## Milestone 5: HID Keyboard Integration
- Implement HID report parsing for boot protocol keyboards and translate reports into key press/release events, including modifier handling.
- Define a kernel-side input event channel (ring buffer or callback) and integrate with the driver manager so consumers (shell/monitor) can subscribe.
- Provide a fallback polling path for early boot debugging when interrupts are not yet reliable, with a compile-time or runtime switch.

## Diagnostics & Testing
- Add verbose logging hooks in the xHCI driver tied into the serial console to trace controller state transitions and USB requests.
- Create a QEMU-based test recipe (`scripts/` or `docs/`) documenting how to launch with `-device qemu-xhci -device usb-kbd` and verify enumeration.
- Plan unit-style tests for descriptor parsing and HID report decoding, plus a kernel monitor command to dump active USB devices and their endpoints.

## Deliverables
- New PCI, ACPI, and xHCI modules under `kernel/src/` with integration into the driver manager and initialization path.
- Documentation updates: `docs/hardware-inventory.md` (PCI snapshot), a new USB design note, and troubleshooting steps for the keyboard driver.
- Scripts or instructions for reproducible QEMU runs and guidelines for extending support to other USB HID devices once the keyboard path is solid.
