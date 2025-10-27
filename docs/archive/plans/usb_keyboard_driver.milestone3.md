# USB Keyboard Driver – Milestone 3 (xHCI Bring-Up)

Milestone 3 focuses on bringing the xHCI host controller online with modern practices so later stages can layer USB protocol support without revisiting firmware handoff or DMA plumbing.

## Completed Work This Iteration
- **Scratchpad infrastructure**: The driver now decodes `HCSParams2`, allocates the scratchpad pointer table, and provisions aligned DMA buffers when the controller advertises a non-zero count (`kernel/src/drivers/usb/xhci/mod.rs:887`). DCBAA entry 0 is updated automatically so future slot/context setup can assume the scratchpad array is ready.
- **Capability telemetry**: Capability logging now reports the required scratchpad count alongside slots, ports, and context size, making controller expectations visible in the serial log (`kernel/src/drivers/usb/xhci/mod.rs:1232`).
- **Port diagnostics**: After RUN is asserted the driver walks each root port, printing connection status, power state, negotiated link state, and detected speed. This gives immediate feedback that QEMU is presenting both keyboard and mouse devices prior to enumeration (`kernel/src/drivers/usb/xhci/mod.rs:1109`).
- **Context scaffolding**: Device-context allocations now respect the controller's advertised context size, record the stride once per controller, and expose helpers for slot/endpoint context access so future milestones can program the default control endpoint cleanly (`kernel/src/drivers/usb/xhci/mod.rs:837`).
- **Documentation coverage**: Every helper added for scratchpads and port introspection carries Rustdoc comments so new contributors can follow the flow without cross-referencing the xHCI spec.

## Observations from QEMU (`./startQemu.sh headless 10`)
- Controller capabilities report `scratchpads=0` on QEMU’s `qemu-xhci`, confirming the scratchpad path is dormant yet safe for hardware that requires it.
- Port summaries show ports 5 and 6 connected at high-speed in `Polling`, aligning with the virtual keyboard and mouse topology.
- The run still exits via the environment guard after timer activity; no new regressions were observed relative to prior bring-up runs.

## Next Steps
1. Populate device and endpoint contexts for the default control endpoint so slot enabling can progress beyond the NOOP command.
2. Wire the MSI helper into the controller once interrupt allocator plumbing lands; until then the legacy timer vector remains the only actively serviced interrupt.
3. Start decoding port status change interrupts to automatically kick off enumeration when devices arrive.
