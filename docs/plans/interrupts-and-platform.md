# Interrupts and Platform Plan

## Scope

Track descriptor-table setup, APIC interrupt routing, timer bring-up, ACPI platform discovery, and SMP-adjacent platform status.

## Plan vs Repo

- The repo clearly implements GDT/TSS/IDT/APIC bring-up, but earlier docs did not make it obvious which interrupt truths were binding and which SMP pieces were still just discovery.

## Implemented Invariants

- [x] IMPLEMENTED: The kernel installs a GDT/TSS with dedicated IST stacks for double fault, NMI, machine check, and page fault handling.
- [x] IMPLEMENTED: The kernel installs an IDT before the high-half jump and reinstalls it after the stack switch in higher-half execution.
- [x] IMPLEMENTED: The kernel masks the legacy PIC during APIC-based interrupt bring-up.
- [x] IMPLEMENTED: The kernel configures and tests LAPIC timer delivery during bring-up before starting a periodic timer.
- [x] IMPLEMENTED: ACPI parsing records CPU/APIC topology, IO APIC data, and local APIC address into runtime platform structures.
- [x] IMPLEMENTED: The serial driver programs IOAPIC routing for interrupt-driven receive handling.
- [x] IMPLEMENTED: The current CPU setup path detects CPUID features once, applies a limited CR0/CR4 setup during bring-up, and enables SSE/AVX state only when the detected feature set and current setup path allow it.
- [x] IMPLEMENTED: APIC base/mode reporting now comes from `IA32_APIC_BASE`, so the monitor can distinguish x2APIC capability from the current runtime access mode even though mainline interrupt delivery still uses xAPIC-style MMIO.

## IRQ Dispatch Architecture

### Per-vector assembly stubs (implemented)

All hardware IRQ vectors 0x20–0xFD go through a uniform stub-based dispatch
path. No vector in this range has a dedicated `extern "x86-interrupt"` handler
anymore — including timer (0x40), serial RX (0x41), and xHCI MSI (0x50).

The only intentional exceptions are 0xFE (APIC error) and 0xFF (spurious):
the Intel SDM explicitly requires that spurious interrupts must **not** receive
an EOI, which makes them semantically different from regular hardware IRQs.

**Dispatch path:**

```
CPU takes interrupt on vector N
    → IDT[N]              → irq_stub_N        (push N; jmp irq_stub_common)
    → irq_stub_common     → irq_dispatch_common(vector: u8)   (pop rdi; call; iretq)
    → IRQ_REGISTRY[N]     → registered fn()   (handler sends EOI, does work)
                          → EOI + warn         (if no handler registered)
```

**Registering a handler:**

```rust
irq_registry::register_irq_handler(vector, "driver-name", handler_fn)
    .expect("vector already claimed");
```

Built-in registrations happen in `init_idt()`:
- `0x40` → `irq_timer` (APIC timer tick)
- `0x41` → `irq_serial_rx` (serial receive)
- `0x50` → `irq_usb_xhci` (xHCI MSI)

Any driver can claim any free vector the same way during its `init()`.

**Why not read APIC ISR?**
- ISR scanning is O(8 MMIO reads) on every interrupt — unacceptable on a hot path.
- Fragile under SMP: per-CPU LAPIC ISR gives the right answer per-core but adds
  confusing reasoning; stubs require no shared state at all.
- Assembly stubs are the canonical OS solution (Linux `irq_entries_start`, FreeBSD
  `IDTVEC` macros). The vector is baked into the code.

**Why convert timer/serial/xHCI to the same path?**
- Consistency: every hardware IRQ follows the same flow; none bypass the registry.
- Drivers register and deregister handlers uniformly; no special-casing in the IDT.
- Removed `install_timer_vector_runtime()` — manual IDT patching was a workaround
  that is no longer needed.

**SMP correctness:**
- Stubs are pure read-only code, safely shared across all CPUs.
- `IRQ_REGISTRY` is protected by a `Mutex` — SMP-safe for the single-core case;
  needs a per-CPU fast path (spinlock or lock-free table) when SMP is added.
- Per-CPU interrupt stacks (IST for NMI/DF/MCE) are a GDT/TSS concern, orthogonal.

**Implementation files:**
- `kernel/src/interrupts/handlers.rs` — `global_asm!` stubs, `IRQ_STUB_TABLE`, `irq_dispatch_common`, `irq_timer`, `irq_serial_rx`, `irq_usb_xhci`
- `kernel/src/interrupts/mod.rs` — IDT setup loop (`set_handler_addr`), built-in registrations
- `kernel/src/interrupts/irq_registry.rs` — `dispatch_irq`, `register_irq_handler`, `list_irq_handlers`

## TODO

- [ ] TODO: Document an implemented AP startup path if and when one exists in the repo.
- [ ] TODO: Promote any future x2APIC runtime bring-up to an axiom only after the main path depends on it.

## Risks

- [!] RISK: SMP discovery exists, but a documented AP-startup/runtime multi-core bring-up path is not yet an implemented invariant.
- [!] RISK: Descriptor-table and timer correctness currently rely on several staged transitions during high-half bring-up, which makes ordering mistakes expensive.

## Related Axioms

- `../axioms/arch-x86_64.md#A1:-The-kernel-is-x86_64-no_std-code-using-the-x86-interrupt-ABI`
- `../axioms/arch-x86_64.md#A2:-GDT/TSS-setup-provides-dedicated-IST-stacks-for-critical-faults`
- `../axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked`
- `../axioms/arch-x86_64.md#A4:-SMP-discovery-exists-but-AP-startup-is-not-yet-a-documented-implemented-invariant`

## Implementing Modules

- `kernel/src/gdt.rs`
- `kernel/src/interrupts/mod.rs`
- `kernel/src/interrupts/handlers.rs`
- `kernel/src/interrupts/apic.rs`
- `kernel/src/interrupts/timer.rs`
- `kernel/src/acpi/mod.rs`
- `kernel/src/acpi/madt.rs`
- `kernel/src/drivers/serial.rs`
- `kernel/src/environment.rs`
