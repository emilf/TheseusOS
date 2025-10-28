# USB Keyboard Driver – Milestone 3 (xHCI Bring-Up)

Milestone 3 focuses on bringing the xHCI host controller online with modern practices so later stages can layer USB protocol support without revisiting firmware handoff or DMA plumbing.

## Completed Work This Iteration
- **Scratchpad infrastructure**: The driver now decodes `HCSParams2`, allocates the scratchpad pointer table, and provisions aligned DMA buffers when the controller advertises a non-zero count (`kernel/src/drivers/usb/xhci/mod.rs:1242`). DCBAA entry 0 is updated automatically so future slot/context setup can assume the scratchpad array is ready.
- **Capability telemetry**: Capability logging now reports the required scratchpad count alongside slots, ports, and context size, making controller expectations visible in the serial log (`kernel/src/drivers/usb/xhci/mod.rs:1491`).
- **Port diagnostics**: After RUN is asserted the driver walks each root port, emitting a structured summary (link state, speed, power, reset) and highlighting the first connected port to seed enumeration decisions (`kernel/src/drivers/usb/xhci/mod.rs:1569`).
- **Context scaffolding**: Device-context allocations now respect the controller's advertised context size, record the stride once per controller, expose slice-based helpers for slot/endpoint access, prime the input context for the default control endpoint using the first connected port, and immediately issue Enable Slot/Address Device commands to capture the assigned slot ID (`kernel/src/drivers/usb/xhci/mod.rs:1621`).
- **Documentation coverage**: Every helper added for scratchpads and port introspection carries Rustdoc comments so new contributors can follow the flow without cross-referencing the xHCI spec.
- **EP0 event handling**: Command-completion polling now delegates to a shared event-ring reader that recognises transfer events, caches EP0 completions, and attempts the synchronous GET_DESCRIPTOR(Device) transfer immediately after `ADDRESS_DEVICE`. Descriptor bytes are still pending—the QEMU run reports a cycle-bit mismatch before the transfer TRB is written—so the driver logs ring/USBSTS snapshots to guide the follow-up investigation.

## Observations from QEMU (`./startQemu.sh headless 10`)
- Controller capabilities report `scratchpads=0` on QEMU’s `qemu-xhci`, confirming the scratchpad path is dormant yet safe for hardware that requires it.
- Port summaries show ports 5 and 6 connected at high-speed in `Polling`, aligning with the virtual keyboard and mouse topology.
- The EP0 descriptor request currently triggers an event-ring cycle mismatch (cycle bit remains 0 while `USBSTS.TRANSFER_EVENT` is asserted). The driver now dumps the relevant TRBs to the log, and later milestones must resolve the desynchronisation so enumeration can continue.

## Next Steps
1. Investigate the EP0 transfer ring mismatch in QEMU (verify the deque pointer/DCS handshake and TRB layout) so the device descriptor transfer produces a valid event.
2. Wire the MSI helper into the controller once interrupt allocator plumbing lands; until then the legacy timer vector remains the only actively serviced interrupt.
3. Start decoding port status change interrupts to automatically kick off enumeration when devices arrive.
