# USB Keyboard Driver – EP0 Control Transfer Design

## Why This Design Note Exists

The xHCI scaffolding can already:
- Discover the first connected port and prime slot/endpoint contexts.
- Allocate the EP0 transfer ring and a DMA buffer for descriptor reads.
- Issue `ENABLE_SLOT`/`ADDRESS_DEVICE`, albeit with the latter returning completion code `5` (`TRB_ERROR_SLOT_NOT_ENABLED` on QEMU).

The next milestone is to drive a *fully successful* GET_DESCRIPTOR(Device) request immediately after `ADDRESS_DEVICE`. This document captures the reasoning, detailed TRB layout, ordering, and verification strategy so subsequent code changes stay intentional and reviewable.

## Big Picture Alignment

| Layer | Current State | Planned Improvement | Contribution to HID keyboard goal |
|-------|---------------|---------------------|-----------------------------------|
| Controller bring-up (Milestone 3) | Slot contexts primed; `ADDRESS_DEVICE` still erroring | Correct endpoint context flags + transfer TRBs | Leaves controller in a state where enumeration can progress |
| Enumeration stack (Milestone 4) | Not implemented | First control transfer (Device Descriptor) | Supplies USB IDs, packet sizes, and ensures EP0 works before tackling hubs/keyboards |
| HID integration (Milestone 5) | Pending | N/A (dependent) | Requires descriptor + configuration parsing groundwork |

## Design Goals & Constraints

1. **Standards-compliant TRB programming.** Align with xHCI §4.11 so real hardware and QEMU behave identically.
2. **Deterministic sequencing.** Each step will return a well-defined completion code; in TCG we expect completion within the existing `RUN_STOP_TIMEOUT`.
3. **Minimal kernel footprint.** All temporary buffers allocated through `DmaBuffer`, preserving current allocator contracts.
4. **Observable progress.** Every state change should log enough information to reconstruct control transfers when debugging.
5. **Learning-first.** Keep the code readable; inline comments and docstrings should connect spec details to implementation decisions.

## Control Transfer Building Blocks

### 1. EP0 Context Requirements

| Field | Source | Desired Value | Rationale |
|-------|--------|---------------|-----------|
| DCS (bit 0 of TR Dequeue Pointer) | Transfer ring producer cycle | `1` on first arm | xHCI consumes TRBs whose cycle bit matches DCS; both must start `1`. |
| Dequeue Pointer bits 4..63 | `TransferRing::dequeue_pointer()` | 16-byte aligned physical address | Hardware ignores low nibble; software manages alignment. |
| Endpoint Type (Slot ctx word 1 bits 3..5) | Constant | `ENDPOINT_TYPE_CONTROL` | Mandatory for EP0. |
| Max Packet Size | Speed-dependent lookup | 8/64/512/1024 | Control endpoints must match USB spec for full/high/super speeds. |
| Error Count | Constant `3` | Standard recovery budget. |

### 2. TRB Field Map (xHCI Table 4-7 & 4-17)

| Stage | TRB Type | Parameter Field | Status Field | Control Field |
|-------|----------|-----------------|--------------|---------------|
| Setup | `TRB_TYPE_SETUP_STAGE` | Inline setup packet (IDT=1) | Transfer Length = 8 | Type, IDT, Chain set; no IOC (we rely on Status stage event). |
| Data | `TRB_TYPE_DATA_STAGE` | Physical buffer | Transfer Length = descriptor size | Type, Direction (`TRB_DIR_IN`), Chain, optional IOC if we want event after data. |
| Status | `TRB_TYPE_STATUS_STAGE` | Zero | Zero | Type, IOC=1, Direction opposite of data stage. |

Completion events (Type `TRB_TYPE_TRANSFER_EVENT`) report:
- Completion Code (`status[31:24]`)
- Remaining bytes (`status[23:0]`)
- Slot & Endpoint IDs (`control`)
- Pointer to the offending TRB (`parameter`)

### 3. Doorbell & Event Handling

1. After staging TRBs, ring Doorbell `slot_id` with value `endpoint_id` (1 for EP0).
2. Extend the event polling loop to:
   - Return command completions to the caller (existing behaviour).
   - **Queue transfer events for EP0** into a short FIFO (or handle inline) before returning to the caller.
3. Only one outstanding control transfer will be queued initially, so we can block waiting for the transfer event after `ADDRESS_DEVICE`. This keeps the state machine simple until we need asynchronous support.

## Implementation Plan & Verification

| Step | Change | Verification | Expected Logging |
|------|--------|-------------|------------------|
| 0 (baseline) | Ensure `startQemu.sh headless 10` passes | ✅ (already recorded) | Current logs ending with environment exit |
| 1 | Document existing code (DONE) & confirm baseline | `startQemu.sh headless 10` | No behaviour change |
| 2 | Adjust endpoint context writer to set DCS + masked dequeue pointer | Unit: `cargo check` + `startQemu` | `address-device` still code `5` (expected until TRBs queued) |
| 3 | Implement TRB enqueue helpers (setup/data/status) with strict asserts | Unit: `cargo check` | No runtime change yet |
| 4 | Queue GET_DESCRIPTOR after successful `ADDRESS_DEVICE`; do **not** yet wait for transfer event | `startQemu` | Should still log warning (code 5) until DCS fix verified |
| 5 | Teach event loop to handle transfer events, storing descriptor bytes; log descriptor summary | `startQemu` | Expect completion code `1`, descriptor bytes, slot remains enabled |
| 6 | Add regression guard: respond gracefully if completion code != `1` (log + leave buffer zeroed) | `startQemu` negative test optional |

### Detailed Reasoning Per Step

1. **Context Fix Before TRBs.** Completion code `5` indicates the slot transition failed—common causes are mismatched DCS or invalid dequeue pointer. Fixing context first ensures subsequent transfer work rests on a valid addressable slot.
2. **Reusable TRB Helpers.** Encapsulating TRB enqueue logic makes the learning experience clearer: one helper per stage will document the field layout inline versus scattering bit twiddling.
3. **Event Handling Split.** By reusing the existing poll loop and delegating transfer events to a helper, we keep command completion semantics intact while still being able to synchronously await descriptor data in the address-device path.
4. **Logging Strategy.** Every stage should announce the important parameters (descriptor length, vendor/product IDs, completion codes). This matches the philosophy of the rest of the project where logs are teaching material.

### Rollout & Feature Flags

No new feature flags required. The design assumes the control transfer runs unconditionally once the slot is enabled. If future testing shows flaky hardware, we can guard the descriptor fetch behind a configuration switch without refactoring the core logic.

## Open Questions / Future Work

1. **Timeout semantics.** The existing `RUN_STOP_TIMEOUT` is convenient but large; once control transfers are stable we can revisit timeouts to improve failure feedback.
2. **Asynchronous enumeration.** For now we perform a single blocking descriptor fetch. Milestone 4 will need an event-driven loop that can manage multiple outstanding transfers.
3. **Error recovery.** This design logs failures but does not retry. That is acceptable for the teaching kernel; production-quality code would handle stalls and requeue requests.
4. **Unit tests.** Descriptor parsing logic is straightforward and can be unit-tested in Rust once the synchronous fetch path delivers bytes. This note does not cover parsing yet.

## Lessons From Earlier Attempts

Implementing a proof-of-concept before writing this document surfaced several pitfalls that now shape the design:

1. **Endpoint context DCS matters.** Leaving the dequeue pointer’s cycle bit at `0` while the software producer started with cycle `1` caused `ADDRESS_DEVICE` to complete with code `5` (`TRB_ERROR_SLOT_NOT_ENABLED`). The controller simply ignored the transfer ring. The current plan explicitly sets both producer cycle and DCS to `1` when priming EP0.

2. **TRB field confusion is easy.** Our first script attempted to build setup/data/status TRBs but left large chunks implicit. As a result, the TRB type defaults were wrong, IOC wasn’t set, and the controller never generated the expected transfer event. Recording the exact field map (Table 4‑7, 4‑17) in this doc prevents guessing when we write the real helpers.

3. **Event ring handling must be shared.** Extending `poll_command_completion` to “just” look for transfer events muddied the logic and risked double-borrow panics. The revised design introduces separate helpers (`fetch_event`, `handle_transfer_event`, `wait_for_transfer_event`) so command completions and transfer events stay distinct while sharing ring iteration code.

4. **Logging gaps hinder debugging.** Earlier attempts only printed the failure code; debugging required manual inspection of raw buffers. The new plan mandates descriptive logs (vendor ID, product ID, completion codes) so regressions are self-explanatory.

5. **Small, reversible steps beat large jumps.** Rolling back the entire implementation after hitting code `5` wasted time. The verification plan now contains incremental `startQemu` checkpoints so we can stop at the last good state instead of unwinding everything.

These lessons are now baked into the design roadmap, reducing risk when we reattempt the implementation.

## Summary

This plan advances Milestone 3 to a verifiable end-state: EP0 can complete a standard USB control transfer and extract the device descriptor. The work is structured so every step is observable and reversible without large rewrites, priming the codebase for the broader enumeration and HID milestones that follow.
