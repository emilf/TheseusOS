# USB Keyboard Driver Roadmap

## Context & Goals
- Bring up end-to-end support for a USB HID keyboard under the hobby OS kernel, targeting QEMU's `qemu-xhci` controller first and leaving hooks for real hardware.
- Fill current gaps: no PCI bus layer, no USB stack, no input event pipeline, and limited DMA-friendly memory allocation.
- Deliver a staged plan that sequences foundational infrastructure (PCI, ACPI handoff, DMA buffers), controller bring-up, USB protocol layers, and keyboard input integration.

## Status Snapshot
- [x] **Milestone 0 – Prerequisite Audit**: Baseline inventory captured; FADT-derived legacy USB knobs and XHCI descriptors are now mirrored in `PlatformInfo`.
- [x] **Milestone 1 – PCI Discovery**: ECAM enumeration integrated with the driver system, and USB controllers are claimed via the new legacy handoff path.
- [x] **Milestone 2 – ACPI & Controller Handoff**: Firmware ownership negotiation and monitor diagnostics are in place; documentation and policy wiring now cover the MSI/IOAPIC fallback story.
- [x] **Milestone 3 – xHCI Bring-Up**: QEMU HID keyboard enumerates via the interrupt pipeline; follow-ups remain for hotplug, IRQ allocator integration, and cleanup.

## Milestone 0: Prerequisite Audit
- Confirm interrupt delivery story: document current APIC/IOAPIC usage, available vectors, and whether MSI/MSI-X is viable for early use; define the fallback legacy IRQ path.
- Inventory memory allocators: identify what is needed to hand out physically contiguous, cache-coherent buffers for xHCI rings (tie-in with the persistent frame allocator roadmap).
- Review bootloader hardware inventory (`HardwareDevice` records) to ensure USB/XHCI handles are discoverable and feed that into kernel-side logging for debugging.
- Tie the MSI helper into xHCI so controllers that advertise MSI immediately switch away from legacy routing (vector 0x50 with a simple runtime handler) while leaving logging in place for firmware that only offers IOAPIC paths.
- Recycle DMA-backed command/event rings using the shared allocator so controller resets no longer churn the contiguous allocator.

## Milestone 1: PCI Discovery Baseline
- Implement a minimal PCI config accessor (ports `0xCF8/0xCFC` and later MMCONFIG via ACPI MCFG) under `kernel/src/drivers/system` with safe wrappers.
- Build a PCI enumerator that walks buses/devices/functions, records class/subclass, and registers devices with the driver manager using existing traits.
- Enable bus mastering and MMIO for class `0x0C/0x03` (Serial Bus / USB controller) devices, and expose BAR information plus interrupt capabilities to later stages.

## Milestone 2: ACPI & Controller Handoff Support
- ✅ Extend ACPI parsing to read XSDT/RSDT beyond MADT, focusing on finding the FADT (for legacy USB handoff) and the XHCI-specific Extended Capabilities Descriptor.
- ✅ Implement BIOS ownership/OS ownership handoff (EHCI legacy handoff and xHCI Extended Capabilities) to disable firmware emulation that might steal the controller.
- ✅ Surface MCFG ranges (if present) to let the PCI layer switch to memory-mapped config space for performance and to access devices above bus 255 if needed.
- ✅ Expose diagnostics/monitor hooks for the newfound legacy handoff state so firmware ownership can be audited quickly.
- ✅ Document the firmware release procedure and codify policy wiring for MSI/IOAPIC fallback.

### Firmware Release & Interrupt Policy (2025-10-29)
- **Ownership release**: During boot we clear the BIOS-owned bit in the xHCI Extended Capabilities and wait for firmware acknowledgement before touching runtime registers. If the BIOS refuses to yield within the 500 µs polling window we log the fault and continue with a degraded configuration, keeping the fallback polling path active.
- **Preferred routing**: When MSI or MSI-X capabilities are present we allocate vector `0x50`, program the message address with the BSP LAPIC ID, and mark the controller as interrupt driven. Each controller records the selected vector in `msi_vector` so diagnostics can confirm delivery.
- **Fallback rules**: If the platform lacks MSI/MSI-X (or the enable step fails) we leave `msi_enabled` false and rely on the cooperative polling loop. The idle path calls `poll_runtime_events_fallback()` once per `hlt` iteration so controllers without modern interrupt support still make progress.
- **Re-entrancy guard**: Interrupt-side servicing now routes through `service_runtime_interrupt()`, which uses a non-blocking `try_lock` around the controller list. When the lock is contended we set a deferred-service latch so the idle-time fallback poll forces a subsequent drain—eliminating the deadlock that starved HID reports after MSI-X was enabled.

## Milestone 3: xHCI Host Controller Bring-Up
- [x] Create an xHCI driver module that maps the controller MMIO region, parses capability/operational registers, and performs controller reset to a known state.
  - Stub driver now maps MMIO, performs the halt/reset handshake, reports capabilities, allocates command/event rings, programs CRCR/ERST for interrupter 0, stands up the DCBAA, enables the advertised slot count, transitions the controller to RUN, and verifies command submission/completion with a NOOP (`kernel/src/drivers/usb/xhci/mod.rs:1`); runtime bring-up remains (no device contexts yet).
  - Refactor note: the xHCI implementation is being modularized for merge-readiness. Ring plumbing is in `kernel/src/drivers/usb/xhci/rings.rs`, TRB constants/types in `kernel/src/drivers/usb/xhci/trb.rs`, and HID boot keyboard parsing in `kernel/src/drivers/usb/xhci/hid.rs`; the main `xhci/mod.rs` remains the integration point.
  - QEMU launch script pins `qemu-xhci` to the root bus so the virtual controller reliably enumerates during bring-up (`startQemu.sh:92`).
- Allocate and initialize the command ring, event ring, and scratchpad buffers using DMA-capable physical memory; ensure virtual mappings stay uncached or write-back per spec. (Command/event rings + DCBAA live, scratchpad pointer table and DMA allocation helpers now wired into the driver.)
  - Root port diagnostics are now decoded into structured states, including automatic identification of the first connected device port to drive forthcoming enumeration.
- Program slot contexts and endpoint contexts for the default control endpoint, handling doorbell and interrupter configuration; route interrupts via MSI/MSI-X if possible, falling back to IOAPIC. (Context sizing helpers, input-context priming, and on-boot enable-slot/address-device commands now capture the assigned slot ID for the first attached port.)
  - Detailed EP0 control-transfer design captured in `docs/archive/plans/usb_keyboard_driver.ep0_control_transfer.design.md`.
- Teach the shared event ring to separate command completions from EP0 transfer events, cache control-transfer completions, and follow the spec-mandated enumeration path: `ADDRESS_DEVICE` with `BSR=1`, partial 8-byte `GET_DESCRIPTOR`, `EVALUATE_CONTEXT` to apply the true Max Packet size, a second `ADDRESS_DEVICE` with `BSR=0`, and finally the full descriptor. (Route-string, TRB-type, and transfer-ring lifecycle fixes now deliver both the prefix and full 18-byte device descriptor under QEMU, with evaluate-context and configure-endpoint commands succeeding. The control path now also retrieves the configuration descriptor, walks the descriptor chain, and records the HID boot keyboard interrupt endpoint.)
- Ensure the slot context advertises both the slot and EP0 endpoint in the Context Entries field and seed the endpoint context with the correct TR dequeue pointer/cycle state so the controller has a valid view before the first doorbell. (The HID boot keyboard’s interrupt-IN endpoint now loops continuously: transfer events are logged, the capture buffer is recycled, and the ring is rearmed after every report; a class `SET_IDLE` request keeps QEMU quiet between real keystrokes. Boot reports are now decoded into press/release logs, annotated with ASCII where applicable, and fanned out through the shared `input::keyboard` hub so the kernel monitor’s `kbd` command and any future listeners can drain them.)
- Extend the diagnostics path so `usb summary` echoes MSI enablement, assigned vector, and the raw IMAN register (IE/IP bits plus hex value). A warn-level latch also fires if IMAN.IE clears unexpectedly, giving us concrete evidence when MSI delivery stalls even though the controller is posting events.
- Fix the MSI-X control write so we enable the table (bit 15) while clearing the function mask (bit 14); with the corrected sequence QEMU now delivers completions over MSI-X and the idle fallback can remain disabled without starving the keyboard.
- Add an interrupt-only MVP self-test: after MSI/MSI-X is enabled, enqueue a NOOP command and assert that the completion is observed by the interrupt-driven runtime event-ring path (no user input required). Keep idle-loop IMAN peeking disabled by default (`USB_IDLE_IMAN_DIAGNOSTICS=false`) to avoid QEMU `trace:usb_*` log storms.
- Do not treat `IMAN.IP` as authoritative for “event ring has entries.” Under QEMU, `IMAN.IP` may be cleared after a successful MSI/MSI-X post, so the correct drain predicate is the event ring TRB cycle bit and consumer index. Relying on `IMAN.IP` can wedge interrupt-driven drains while the ring still contains events.
- Ensure endpoint IDs match QEMU’s xHCI EPID encoding during bring-up (EP0=1, EPn IN=`n*2+1`, EPn OUT=`n*2`). If you ring the doorbell with `epid 4`, QEMU will issue tokens to USB endpoint 2 and the virtual keyboard will STALL (`CC_STALL_ERROR`).
- Ensure “deferred MSI servicing” makes progress even when polling fallback is disabled: if the MSI handler cannot take the controller-list lock, it should latch a request that the idle loop later drains once the lock becomes available.
- For the boot-time MSI/MSI-X NOOP self-test, perform a single forced drain pass after submission so the test cannot wedge forever (and so diagnostics remain stable even while interrupt delivery is still being tuned).

### Milestone 3 Cleanup & Quality
- Wire port-status-change events (PSCs) into the runtime path so new devices/hotplug enumerate instead of only the first connected port at init; use PSCs to trigger slot creation and port resets.
- Integrate MSI/MSI-X vector selection with the interrupt allocator and connect the driver’s `irq_handler` to the event-ring drain path instead of the current fixed `0x50` vector and stub.
- Add HID `SET_PROTOCOL` negotiation for boot keyboards that require 8-byte reports; confirm interrupt endpoint configuration still arms correctly after the protocol switch.
- Split `kernel/src/drivers/usb/xhci/mod.rs` into smaller domains (capabilities/handoff, control-transfer + enumeration, runtime interrupt/polling, diagnostics) and tighten inline docs around the polling/IMAN diagnostic flags and event-ring drain rules.
- Expand docs with a concise “how interrupts are serviced” note covering MSI enable, IMAN/IMOD interaction, and when to toggle `USB_ENABLE_POLLING_FALLBACK` during debugging.

## Milestone 4: USB Device Enumeration Stack
- Implement basic USB request/response infrastructure (setup packets, control transfers, TRBs) sufficient to enumerate attached devices over the default control endpoint.
- Parse descriptors (device, configuration, interface, HID, endpoint) and select the correct configuration for HID keyboards; handle both full- and high-speed devices via the root hub.
- Build a lightweight hub management layer to service port status change events and create device slots for each attached keyboard.

## Milestone 5: HID Keyboard Integration
- Implement HID report parsing for boot protocol keyboards and translate reports into key press/release events, including modifier handling.
- Define a kernel-side input event channel (ring buffer or callback) and integrate with the driver manager so consumers (shell/monitor) can subscribe.
- Provide a fallback polling path for early boot debugging when interrupts are not yet reliable, with a compile-time or runtime switch.

## Diagnostics & Testing
- Add verbose logging hooks in the xHCI driver tied into the serial console to trace controller state transitions and USB requests. (Port status snapshots and scratchpad bookkeeping now emit at INFO level during bring-up.)
- Create a QEMU-based test recipe (`scripts/` or `docs/`) documenting how to launch with `-device qemu-xhci -device usb-kbd` and verify enumeration.
- Plan unit-style tests for descriptor parsing and HID report decoding, plus a kernel monitor command to dump active USB devices and their endpoints.

## Deliverables
- New PCI, ACPI, and xHCI modules under `kernel/src/` with integration into the driver manager and initialization path.
- Documentation updates: `docs/hardware-inventory.md` (PCI snapshot), a new USB design note, and troubleshooting steps for the keyboard driver.
- Scripts or instructions for reproducible QEMU runs and guidelines for extending support to other USB HID devices once the keyboard path is solid.
