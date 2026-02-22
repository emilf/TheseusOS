# Logging policy (TheseusOS)

This project relies heavily on logs during bring-up, but **default boot output must stay high-signal**.
Excessive logs waste developer time and (when using AI tools) consume precious context.

## Goals

- **Default config is quiet**: only high-signal INFO, plus WARN/ERROR.
- **Debugging is opt-in**: turn on DEBUG/TRACE intentionally, ideally scoped to a module.
- **Critical boot failures are unmissable**: emit to **serial** and **ISA debugcon (0xE9)**.

## Current knobs

### Kernel log level

- Default: `kernel/src/config.rs` → `DEFAULT_LOG_LEVEL` (should remain relatively quiet, e.g. `Info`).
- Per-module overrides: `MODULE_LOG_LEVELS` (prefer adding temporary overrides while debugging specific areas).

### Verbose output toggles

- `VERBOSE_KERNEL_OUTPUT`: enables extra debug traces.
- `PRINT_HARDWARE_INVENTORY`: UEFI inventory dump (very noisy; keep off by default).

### USB/xHCI diagnostics

- `USB_ENABLE_POLLING_FALLBACK`: keep `false` for normal runs.
- `USB_IDLE_IMAN_DIAGNOSTICS`: off by default (can spam QEMU host logs depending on tracing).
- `USB_XHCI_EVENT_RING_DIAGNOSTICS`: off by default (enables ring snapshots / extra warnings).
- `USB_RUN_MSIX_SELF_TEST`: on by default (useful, high-signal correctness check).

### QEMU debug flags (host-side)

In `startQemu.sh` headless mode:

- Default `-d` flags are intentionally minimal to avoid enormous output.
- Override with:

```bash
QEMU_DEBUG_FLAGS="int,guest_errors,cpu_reset" ./startQemu.sh headless 10
```

## Log level guidance

- **ERROR**: the system cannot proceed or will be incorrect; should be user-visible.
- **WARN**: unexpected but recoverable; should stand out.
- **INFO**: major milestones only ("kernel started", "ACPI init ok", "xHCI MSI enabled").
- **DEBUG**: detailed state dumps (descriptors, per-port state, per-device enumeration steps).
- **TRACE**: extremely chatty loops or per-register/per-TRB style logs; never default-on.

## Anti-logspam patterns

- Use **one-shot** logs ("first time we saw X")
- Use **rate limiting** for repeated warnings
- Use **module-scoped verbosity** rather than raising the global default
- Keep interrupt handlers quiet by default

## Notes

If you add new logging:
- ask: "Would I want to see this on every boot?" If not, it probably belongs in DEBUG/TRACE.
- prefer structured summaries over per-item dumps.
