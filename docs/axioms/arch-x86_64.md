# x86_64 Architecture Axioms

Binding truths for CPU mode, descriptor tables, and interrupt-controller assumptions.

## A1: The kernel is x86_64 `no_std` code using the x86-interrupt ABI

**REQUIRED**

The kernel is built as `no_std` x86_64 code and uses the nightly `abi_x86_interrupt` feature for IDT handlers.

Implements / evidence:
- `kernel/src/lib.rs`
- `kernel/src/interrupts/handlers.rs`

### A1.1: Calling conventions — Win64 at the firmware boundary, SysV internal

**REQUIRED**

The kernel is compiled under the UEFI target, where rustc forces `is-like-windows`
(because `os` is `uefi`). Consequently plain `extern "C"` means the **Microsoft
x64 ABI** (first argument in RCX, args in RCX/RDX/R8/R9).

That convention is correct *only* where firmware or bare assembly is the other
side. Internal boundaries — Rust-to-Rust, and assembly that calls into Rust —
declare `extern "sysv64"` explicitly, so they follow the **System V AMD64 ABI**
(first argument in RDI), which is what assembly references and external
documentation assume.

Rules:

- `extern "C"` / `efiapi` is permitted only at boundaries whose other side is
  firmware or raw assembly: `kernel_entry` (bootloader call, `entry-abi: win64`),
  the high-half entry symbols reached by a raw jump, and the IRQ stub symbol
  declarations / stub table (addresses only, never called through Rust).
- Everything else uses `extern "sysv64"`: e.g. `irq_dispatch_common`,
  `sycall_dispatch`, `noop_for_test`, and the monitor's `call` trampoline.
- An `asm!` or `global_asm!` block that calls into Rust must use the register the
  *callee's declared ABI* specifies. `irq_stub_common` pops the vector into
  **RDI** because `irq_dispatch_common` is `extern "sysv64"`.
- Do not "tidy" a retained `extern "C"` into `sysv64` without changing the other
  side in the same change. The strings are load-bearing, not stylistic.
- Red zone: SysV defines a 128-byte red zone that an interrupt can clobber. Both
  target specs keep `disable-redzone: true`; that must stay set.

Implements / evidence:
- `kernel/src/lib.rs#kernel_entry`
- `kernel/src/interrupts/handlers.rs#irq_dispatch_common`
- `kernel/src/interrupts/handlers.rs#irq_stub_common`
- `kernel/src/monitor/commands/control.rs`
- `x86_64-unknown-uefi-dwarf.json` (`entry-abi: win64`)

Related plans:
- `../plans/interrupts-and-platform.md#implemented-invariants`

Affected modules:
- `kernel/src/lib.rs`
- `kernel/src/interrupts/*`
- `kernel/src/environment.rs`
- `kernel/src/memory.rs`
- `kernel/src/monitor/commands/*`

## A2: GDT/TSS setup provides dedicated IST stacks for critical faults

**REQUIRED**

The runtime GDT/TSS setup includes dedicated IST entries for at least double fault, NMI, machine check, and page fault handling, and the bring-up path reloads the runtime GDT/TSS state before those fault handlers are relied upon.

Implements / evidence:
- `kernel/src/gdt.rs#build_gdt_state`
- `kernel/src/gdt.rs#refresh_tss_ist`

Related plans:
- `../plans/interrupts-and-platform.md#implemented-invariants`

Affected modules:
- `kernel/src/gdt.rs`
- `kernel/src/interrupts/*`
- `kernel/src/environment.rs`

## A3: Interrupt delivery is APIC-based during kernel bring-up, with legacy PIC masked

**REQUIRED**

The interrupt bring-up path masks the legacy 8259 PIC and uses LAPIC/IOAPIC infrastructure for runtime interrupt delivery.

Implements / evidence:
- `kernel/src/interrupts/apic.rs#disable_all_interrupts`
- `kernel/src/drivers/serial.rs#program_io_apic_entry`
- `kernel/src/environment.rs#continue_after_stack_switch`

Related plans:
- `../plans/interrupts-and-platform.md#implemented-invariants`

Affected modules:
- `kernel/src/interrupts/apic.rs`
- `kernel/src/drivers/serial.rs`
- `kernel/src/environment.rs`

## A3.1: All hardware IRQs 0x20–0xFD dispatch through uniform per-vector stubs

**REQUIRED**

Every hardware interrupt vector in the range 0x20–0xFD — including the APIC
timer (0x40), serial RX (0x41), and xHCI MSI (0x50) — is handled by a
per-vector assembly stub. No vector in this range has a special-cased
`extern "x86-interrupt"` IDT entry. Handlers are registered in the IRQ
registry and called uniformly through `irq_dispatch_common`.

The only deliberate exceptions are 0xFE (APIC error) and 0xFF (spurious),
which keep dedicated Rust handlers because the Intel SDM requires that
spurious interrupts (0xFF) must **not** receive an EOI.

Stub dispatch path:

```
CPU → IDT[N] → irq_stub_N (push N; jmp irq_stub_common)
             → irq_dispatch_common(vector: u8)
             → IRQ_REGISTRY[N] → fn() handler  (sends EOI, does work)
                               → EOI + warn    (no handler registered)
```

This design:
- Bakes the vector into the code — no APIC ISR scanning, no MMIO on the hot path.
- Is SMP-safe: stubs are pure read-only code shared across all CPUs; the registry is `Mutex`-guarded.
- Matches the canonical OS pattern (Linux `irq_entries_start`, FreeBSD `IDTVEC`).
- Allows any driver to claim a vector uniformly via `register_irq_handler`.

Implements / evidence:
- `kernel/src/interrupts/handlers.rs` — `global_asm!` stubs, `IRQ_STUB_TABLE`, `irq_dispatch_common`, `irq_timer`, `irq_serial_rx`, `irq_usb_xhci`
- `kernel/src/interrupts/mod.rs` — IDT setup loop (`set_handler_addr`)
- `kernel/src/interrupts/irq_registry.rs` — `dispatch_irq`, `register_irq_handler`, `list_irq_handlers`
- `kernel/src/interrupts/timer.rs` (`lapic_timer_configure`) — registers `irq_timer`
- `kernel/src/drivers/serial.rs` (`init_serial`) — registers `irq_serial_rx`
- `kernel/src/drivers/usb/mod.rs` (`init`) — registers `irq_usb_xhci`

Related plans:
- `../plans/interrupts-and-platform.md#irq-dispatch-architecture`

Affected modules:
- `kernel/src/interrupts/handlers.rs`
- `kernel/src/interrupts/mod.rs`
- `kernel/src/interrupts/irq_registry.rs`

## A4: SMP discovery exists, but AP startup is not yet a documented implemented invariant

**REQUIRED**

The current repo parses ACPI processor/APIC topology and records CPU counts, but it does not yet document an implemented AP startup path.

Implements / evidence:
- `kernel/src/acpi/mod.rs#initialize_acpi`
- `kernel/src/acpi/madt.rs#parse_madt`

Related plans:
- `../plans/interrupts-and-platform.md#todo`

Affected modules:
- `kernel/src/acpi/*`
- `kernel/src/drivers/system.rs`
