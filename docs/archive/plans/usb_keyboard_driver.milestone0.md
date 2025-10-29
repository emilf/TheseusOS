# USB Keyboard Driver – Milestone 0 Findings

Milestone 0 was about auditing prerequisites before writing any USB code. This note captures what we confirmed, the new visibility we added, and the gaps we still need to close before moving on to PCI/xHCI bring-up.

## Interrupt Delivery Baseline
- `kernel/src/interrupts/mod.rs:23` shows the IDT only routes the LAPIC timer (0x40) and COM1 RX (0x41); no shared interrupt vectors exist yet for future devices.
- Early boot masks the legacy PIC and NMI lines (`kernel/src/interrupts/apic.rs:63`) and assumes a LAPIC at the legacy base (`kernel/src/interrupts/apic.rs:136`), so we currently lack a tested fallback if MSI/MSI-X is unavailable.
- ACPI discovery already records IO APIC descriptors (`kernel/src/acpi/madt.rs:81`) and we map both LAPIC and IOAPIC MMIO windows as uncached (`kernel/src/memory/mapping.rs:258`, `kernel/src/memory/mapping.rs:283`).
- The serial driver is the lone user of IO APIC routing today, directly programming a redirection entry for COM1 (`kernel/src/drivers/serial.rs:376`). We now detect MSI/MSI-X capabilities during PCI enumeration (`kernel/src/drivers/pci.rs:287`), but enabling those interrupt modes is still future work.

## DMA-Capable Memory Inventory
- The persistent allocator now exposes contiguous allocation/free helpers for DMA workloads (`kernel/src/physical_memory.rs:755`), and `kernel/src/memory/dma.rs:1` wraps them in a zeroed, kernel-mapped `DmaBuffer`.
- Buffer helpers now expose cache-policy flags (write-back, write-combining, uncached) so drivers can request write-combining descriptors or uncached bounce buffers explicitly.
- Added a simple fixed-size DMA pool helper (`kernel/src/memory/dma_pool.rs`) to recycle descriptor/ring allocations instead of constantly hitting the global contiguous allocator.
- MMIO regions already use PCD/PWT flags when mapped (`kernel/src/memory/mapping.rs:258`), which is good for controllers, but we still need explicit APIs to map DMA buffers with the write-back policy xHCI expects.

## Hardware Inventory Logging Improvements
- The bootloader now tags PCI and USB device-path nodes (`bootloader/src/hardware.rs:360`, `bootloader/src/hardware.rs:413`) so xHCI controllers and USB keyboards show up in the handoff metadata.
- Kernel logging of the firmware-provided inventory is enabled by default (`kernel/src/config.rs:15`), which means every boot will emit the device list needed to correlate PCI bus IDs with USB endpoints during early debugging.
- With the new summaries (USB port/interface, PCI device/function numbering), we can sanity-check the QEMU topology before writing any enumeration code and confirm the keyboard is actually visible at boot.

## ACPI Legacy Ownership Inputs
- `PlatformInfo::legacy_usb` now captures the FADT-provided SMI command port, SCI line, and ownership bytes so firmware handoff sequencing is documented alongside other prerequisites.
- Any ACPI `XHCI` tables are mirrored into `PlatformInfo::xhci_descriptors`, preserving controller-specific hints for later milestones.

## Follow-Up Actions
- ✅ Integrated the `pci::enable_msi` helper with the xHCI driver. A dedicated MSI vector (0x50) is now wired into the IDT, a lightweight handler drains controller events, and the driver programs the capability whenever firmware exposes it—falling back to polling only when MSI is absent.
- ✅ Added a ring recycler atop the DMA helper so command and event rings reuse zeroed, aligned buffers across controller resets instead of re-allocating from the global contiguous allocator.
