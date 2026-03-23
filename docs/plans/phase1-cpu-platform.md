# Phase 1 — CPU & Platform Hardening

Detailed task breakdown for Phase 1 of the TheseusOS roadmap.
Last updated: 2026-03-23 based on full code audit of all relevant modules.

Tasks are marked:
- `[ ] TODO` — Not started
- `[~] IN PROGRESS` — Active work
- `[x] DONE` — Complete and binding

Leaf tasks include a **Spec** and **Completion Criteria**.

---

## 1.1 Memory Subsystem — Understand & Refactor

### What we actually have (audit findings)

- **`BootFrameAllocator`** — walks UEFI memory descriptors, linear scan, reserved pool of 16 frames. Boot-only, one-way.
- **`PhysicalMemoryManager`** — bitmap first-fit, global `Mutex<Option<...>>`. Initialized post-heap from handoff. Has `alloc_frame`, `free_frame`, `alloc_contiguous`. Solid.
- **`MemoryManager`** — boot-time PML4 builder. Transitional. After boot it's done, no runtime API.
- **`mapping.rs`** — a library of helpers that take `&mut impl FrameSource`. Boot-context only. No runtime mapper exists.
- **`TemporaryWindow`** — single scratch VA slot (`0xFFFF_FFFE_0000_0000`). One page at a time. Exists, works.
- **`PageTableBuilder`** — convenience wrapper over mapping helpers. Still boot-context.
- **`DMA allocator`** — contiguous physical allocation via `alloc_contiguous` + DMA pool on top. Works.
- **IST stacks** — 4× 16 KiB static arrays in `.bss.stack`, already mapped and wired into TSS. **Done.**

**VA layout (all hardcoded):**
```
0xFFFF800000000000  PHYS_OFFSET (linear physmap)
0xFFFF_FF80_0000_0000  ACPI window
0xFFFF_FFFE_0000_0000  TemporaryWindow
0xFFFFFFFF80000000  KERNEL_VIRTUAL_BASE
0xFFFFFFFF90000000  Framebuffer
0xFFFFFFFFA0000000  TEMP_HEAP
0xFFFFFFFFB0000000  KERNEL_HEAP
```

**Gaps:**
- No VA allocator — all virtual addresses are hardcoded constants
- No runtime page table manager — post-boot there is no map/unmap API
- The boot-to-runtime handoff (`record_boot_consumed_region` path) works but has no overlap validation
- No kernel stack allocator — only the boot stack and the 4 static IST stacks exist

---

### 1.1.1 [x] DONE — Physical frame allocator, DMA, TemporaryWindow

Solid. No action needed.

---

### 1.1.2 Virtual Address Space Allocator

`[ ] TODO`

**Prerequisites:** None — this is pure bookkeeping with no hardware dependency.

**Context:**
Every VA region is a hardcoded constant. To dynamically allocate kernel stacks, map MMIO for new drivers, or support per-process page tables, we need a VA allocator that knows the kernel's address space layout and can carve out new ranges.

**Spec:**
Implement a `KernelVaAllocator` in a new `kernel/src/memory/va_alloc.rs`:
- Manages a reserved window of kernel VA space, e.g. `0xFFFF900000000000..0xFFFFB00000000000` (2 TiB, well away from existing hardcoded regions)
- `alloc_va(size: u64, align: u64) -> Option<u64>` — returns aligned VA range of `size` bytes
- `free_va(base: u64, size: u64)` — returns range to pool
- Backed by a simple bump allocator to start; a free-list for reclamation in a follow-up
- Global singleton behind a `Mutex`; initialized during boot before any dynamic mappings are needed
- **Does NOT allocate physical frames** — that's the caller's job

**Completion Criteria:**
- `alloc_va` / `free_va` compile and pass unit tests
- Two calls to `alloc_va` return non-overlapping ranges
- Allocations respect requested alignment
- Used for at least one real allocation (kernel stack from 1.1.4 below)
- The reserved window is documented in `docs/axioms/memory.md` as part of VA layout

---

### 1.1.3 Runtime Page Table Manager

`[ ] TODO`

**Prerequisites:** 1.1.2 (VA allocator), `PhysicalMemoryManager` (done)

**Context:**
`mapping.rs` works only at boot time — callers pass in `&mut impl FrameSource` and the helpers build tables directly. After boot there is no API to map or unmap anything. The `PersistentFrameAllocator` exists but isn't wired into any mapper.

**What we can reuse:** `mapping.rs` helpers. The `PersistentFrameAllocator`. `PHYS_OFFSET` for translating page table frame addresses to virtual.

**Spec:**
Implement a `KernelMapper` type in `kernel/src/memory/runtime_mapper.rs`:
- Constructed from the current CR3 and `PHYS_OFFSET` (`x86_64::OffsetPageTable` under the hood)
- `map_page(va: u64, pa: u64, flags: PageTableFlags) -> Result<(), MapError>` — maps one 4K page; allocates page table frames from `PersistentFrameAllocator`
- `map_range(va: u64, pa: u64, size: u64, flags: PageTableFlags) -> Result<(), MapError>` — maps contiguous range
- `unmap_page(va: u64) -> Result<u64, MapError>` — unmaps, returns PA; does NOT free the physical frame (caller decides)
- `translate(va: u64) -> Option<u64>` — walks page tables, returns PA
- After each map/unmap: calls `invlpg` on the affected VA
- Global singleton behind `Mutex`; initialized after high-half transition completes
- **Out of scope:** TLB shootdowns across CPUs (SMP, Phase 1.7)

**Completion Criteria:**
- `map_page`, `unmap_page`, `translate` work for 4K pages in kernel VA
- Mapping an already-mapped VA returns `Err(MapError::AlreadyMapped)`, not a panic
- After `unmap_page`, `translate` returns `None` and `invlpg` was called
- Test: map a fresh frame, write to it, unmap, confirm translate returns None
- No use of `BootFrameAllocator` at runtime

---

### 1.1.4 Kernel Stack Allocator

`[ ] TODO`

**Prerequisites:** 1.1.2 (VA allocator), 1.1.3 (runtime mapper)

**Context:**
`stack.rs` today contains only one function: `switch_to_kernel_stack_and_jump`. The boot stack is a static 64 KiB array in `.bss.stack`, mapped during bring-up. The 4 IST stacks are also static arrays. There is no facility to allocate additional stacks at runtime — needed for kernel threads (Phase 2) and any future per-CPU stacks.

**Spec:**
Implement in `kernel/src/memory/stack_alloc.rs` (or extend `stack.rs`):

```rust
pub struct StackRegion {
    pub top: u64,    // Initial RSP value (high address, 16-byte aligned)
    pub bottom: u64, // Base of usable stack
    pub guard: u64,  // Guard page VA (= bottom - PAGE_SIZE)
    pub va_base: u64,// Start of VA reservation (= guard)
    pub size: u64,   // Usable stack size in bytes
}

pub fn alloc_kernel_stack(size: u64) -> Result<StackRegion, AllocError>
pub fn free_kernel_stack(region: StackRegion) -> Result<(), AllocError>
```

- `alloc_kernel_stack`:
  1. Uses `KernelVaAllocator` to reserve `size + PAGE_SIZE` bytes of VA
  2. Maps guard page as **not present** (i.e., just doesn't map it — any access faults)
  3. Allocates `size / PAGE_SIZE` frames from `PhysicalMemoryManager`
  4. Maps them via `KernelMapper`
  5. Returns `StackRegion` with `top = va_base + PAGE_SIZE + size` (aligned down to 16 bytes)

- `free_kernel_stack`:
  1. Unmaps all stack pages via `KernelMapper` and frees frames to `PhysicalMemoryManager`
  2. Returns VA region to `KernelVaAllocator`

**Completion Criteria:**
- `alloc_kernel_stack` returns a stack where writing to the guard page triggers `#PF`
- Allocating and freeing leaves allocators in consistent state
- Stack top is 16-byte aligned (required by ABI)
- The existing static IST stacks in `gdt.rs` do NOT need to be replaced by this — they stay as-is; this is for future dynamic stacks

---

### 1.1.5 Boot handoff documentation + validation

`[ ] TODO`

**Prerequisites:** None — this is documentation and a single assertion.

**Context:**
The boot-to-runtime allocator handoff (`BootFrameAllocator` → `PhysicalMemoryManager` via `record_boot_consumed_region` + `drain_boot_consumed`) works but has no overlap validation and no clear prose doc. Before SMP (which allocates more frames during bring-up), this needs to be solid and documented.

**Spec:**
- Add overlap check in `init_from_handoff`: after reserving all consumed regions, walk the bitmap and verify no free frame overlaps any consumed region (or simply assert in debug mode)
- Add a `dump_boot_consumed_log()` debug monitor command that shows what was logged (available even after init since init clears the log — maybe log a summary count before draining)
- Write a clear "Allocator Handoff" section in `docs/axioms/memory.md` explaining:
  - Timeline: when BootFrameAllocator is the only allocator, when PhysicalMemoryManager comes online
  - What `record_boot_consumed_region` records and why it's safe to log lazily
  - What happens if a region is missed

**Completion Criteria:**
- Debug assertion in `init_from_handoff` doesn't fire under normal boot
- `docs/axioms/memory.md` has the Allocator Handoff section
- Monitor shows a consumed-region count at boot (even if just a log line)

---

## 1.2 Driver Subsystem — Formalization

### What we actually have (audit findings)

- **`Driver` trait + `DriverManager`** — the framework is fully defined and compiles. `probe`, `init`, `irq_handler`, `read`, `write` are all there. First-success binding. Global `Mutex<DriverManager>`.
- **`driver_data: Option<usize>`** — defined with raw-pointer cast helpers (`set_driver_state`, `driver_state`, `driver_state_mut`). **The casts are defined but not actually used by any driver today.** PCI and xHCI both use their own globals/statics, not `driver_data`.
- **`pci.rs`** — full ECAM enumeration, BAR decoding (32-bit, 64-bit memory, I/O), capability parsing, MSI enable. **Does NOT call `DriverManager::add_device`.** xHCI is bound via hardcoded init call.
- **`handlers.rs`** — hardcoded vectors: 0x40 (APIC timer), 0x41 (serial RX), 0x50 (xHCI MSI), 0xFE (APIC error). xHCI vector calls `usb::handle_xhci_interrupt()` directly — **not through DriverManager**.
- **`framebuffer.rs`** — drawing utilities only (boot logo, heart animation). Mapped at fixed VA `0xFFFFFFFF90000000` from `map_framebuffer_alloc()` using `handoff.gop_fb_base`. Not a driver.

---

### 1.2.1 Fix driver state storage

`[ ] TODO`

**Prerequisites:** None — isolated change

**Context:**
`driver_data: Option<usize>` with raw-pointer casts is defined but not actually wired to any real driver state today. This is the right time to fix it before drivers start using it and the unsound pattern propagates.

**Spec:**
Replace `driver_data: Option<usize>` in `Device` with `driver_data: Option<alloc::boxed::Box<dyn core::any::Any + Send>>`:
- `set_driver_state<T: Any + Send>(state: T)` — boxes and stores
- `driver_state<T: Any + Send>() -> Option<&T>` — downcasts via `Any::downcast_ref`
- `driver_state_mut<T: Any + Send>() -> Option<&mut T>` — downcasts via `Any::downcast_mut`
- Remove all `unsafe` raw-pointer casts from the trait methods
- Since no drivers currently use `driver_data`, there are no callers to update — clean break

**Completion Criteria:**
- No `unsafe` raw casts in `traits.rs`
- Downcasting to the wrong type returns `None`, not UB
- Compiles with the existing (non-using) driver code unchanged
- Add a unit test: set state as `u32`, read back as `u32` (Ok), read back as `u64` (None)

---

### 1.2.2 PCI enumeration feeds DriverManager

`[ ] TODO`

**Prerequisites:** 1.2.1 (driver_data fix)

**Context:**
PCI enumeration works and BARs are decoded, but nothing flows to `DriverManager`. The xHCI driver is currently wired by a hardcoded call somewhere in the boot sequence, not via PCI probe. This needs to be closed before adding any new PCI driver.

**Spec:**
In `pci.rs`, after enumerating each PCI function:
- Construct a `Device` with `DeviceId::Pci { segment, bus, device, function }`
- Set `device.class` from PCI class code (map the relevant PCI class codes to `DeviceClass` variants; add any missing variants like `Display`, `Audio`)
- Decode BARs into `DeviceResource::Memory` / `DeviceResource::Io` entries on the device
- Call `driver_manager().lock().add_device(device)`

Then: register the xHCI driver with `DriverManager` during boot driver init, and remove the hardcoded xHCI init call. The xHCI driver's `probe()` method should match on `DeviceClass::UsbController`.

**Completion Criteria:**
- After boot, `devices list` monitor command shows all PCI functions that `pci list` shows
- xHCI is bound via this path (its `probe()` is called by DriverManager, not hardcoded)
- BARs appear as `DeviceResource` entries on PCI devices
- No regression: USB keyboard still works end-to-end

---

### 1.2.3 IRQ ownership model

`[ ] TODO`

**Prerequisites:** 1.2.2 (PCI → DriverManager)

**Context:**
IRQ vectors are hardcoded in `handlers.rs`. Vector 0x50 calls `usb::handle_xhci_interrupt()` directly. There's no mechanism for a driver to claim a vector or for DriverManager to dispatch it. For MSI, each device gets its own vector — so shared IRQs aren't the issue; the issue is that vectors are hardcoded and drivers can't claim them dynamically.

**Spec:**
- Add an IRQ vector registry: a simple array of 256 `Option<fn()>` (or `Option<&'static dyn Driver + device index>`) protected by a `Mutex`, initialized to `None`
- Add `fn register_irq_handler(vector: u8, handler: fn()) -> Result<(), &'static str>` — fails if already registered
- The general interrupt dispatch path in `handlers.rs` (for non-reserved vectors like timer/error): if no registered handler, log and EOI; otherwise call the handler
- xHCI driver registers its vector during `init()` via this API
- Serial driver registers its vector during `init()` via this API
- Timer and APIC error keep their dedicated handlers (reserved vectors, not routed through the registry)
- Export `irq list` monitor command that shows registered vectors

**What stays hardcoded for now:** Timer (0x40), APIC error (0xFE) — these are kernel internals, not driver IRQs.

**Completion Criteria:**
- xHCI and serial register their vectors during `init()`; not hardcoded in `handlers.rs`
- Registering the same vector twice returns an error
- `irq list` shows registered vectors and a string name for each
- USB and serial still work after the refactor

---

### 1.2.4 Framebuffer as a proper driver

`[ ] TODO`

**Prerequisites:** 1.1.3 (runtime mapper), 1.2.2 (PCI → DriverManager)

**Context:**
`framebuffer.rs` is drawing utilities over a raw fixed VA. It's not a driver, has no ownership model, and the mapping was done at boot-time by `map_framebuffer_alloc()`. For now we don't need mode-setting or multi-display, but wrapping it in a driver gives us a clean ownership boundary and lets us get rid of the raw global VA access.

**Note on priority:** This is lower priority than 1.2.2 and 1.2.3. If it's inconvenient, it can wait until after Phase 2 starts.

**Spec:**
- Add `DeviceClass::Framebuffer` enum variant
- Create `kernel/src/drivers/video/framebuffer.rs` implementing `Driver`:
  - `probe()`: matches `DeviceClass::Framebuffer`
  - `init()`: reads FB base, size, stride, pixel format from handoff (already accessible at this point); stores as `FramebufferState` via `set_driver_state`
  - Provides a `FramebufferHandle` with `write_pixel`, `fill_rect`, `blit`
- Register one synthetic `Device` with `DeviceClass::Framebuffer` during early boot (before PCI enumeration; it's a platform device not a PCI device)
- The existing drawing code in `framebuffer.rs` is refactored to go through `FramebufferHandle`
- The raw `0xFFFFFFFF90000000` VA access is encapsulated inside `drivers/video/framebuffer.rs` — still the same mapping, but no one else writes to it directly

**Completion Criteria:**
- Boot screen renders correctly after refactor
- `devices list` shows framebuffer device as bound
- No direct writes to `0xFFFFFFFF90000000` outside the framebuffer driver
- The `framebuffer.rs` drawing functions are wrappers around `FramebufferHandle` methods

---

## 1.3 TSS + IST Stacks

`[x] DONE`

**Audit finding:** `gdt.rs` already has a fully functional implementation:
- 4× 16 KiB static IST stacks (`IST_DF_STACK`, `IST_NMI_STACK`, `IST_MC_STACK`, `IST_PF_STACK`)
- TSS descriptor present in GDT; `load_tss()` is called (`ltr` executed)
- IST fields populated in `build_gdt_state()` with top-of-stack addresses, 16-byte aligned
- `refresh_tss_ist()` exists for runtime IST pointer updates
- IDT entries for `#NMI`, `#DF`, `#MC`, `#PF` use the IST indices

**Nothing to do here.** Per-CPU TSS (for SMP) deferred to Phase 1.7.

---

## 1.4 APIC Timer Calibration

### What we actually have (audit findings)

`timer.rs` configures the LAPIC timer with a `/16` divider and hardcoded initial counts (100,000 for one-shot tests, 50,000 for periodic). **No calibration is performed.** There is no reference to HPET, PIT, or TSC frequency. The values happen to work in QEMU but are meaningless on real hardware and don't give real-time semantics.

---

### 1.4.1 HPET or PIT calibration reference

`[ ] TODO`

**Prerequisites:** 1.5.2 (LAPIC abstraction, so calibration uses the clean API)

**Spec:**
Implement `calibrate_apic_timer() -> u64` returning `ticks_per_ms`:

**HPET path (preferred):**
- ACPI HPET table is already parsed in `acpi/mod.rs` — check if HPET base address is available
- Map HPET MMIO via `KernelMapper` (1.1.3) — one page at base address
- Read `GCAP_ID` register: `counter_clk_period` field (femtoseconds per tick, bits [63:32])
- Period in ns = `counter_clk_period / 1_000_000`
- Enable HPET main counter (`GEN_CONF` register, bit 0)
- Read `MAIN_COUNTER`, start LAPIC timer with a large initial count, wait until MAIN_COUNTER advances by 10ms worth of HPET ticks, read LAPIC current count, compute delta

**PIT fallback** (if HPET not available):
- Channel 2 + port 0x61 gate trick; 1.193182 MHz known frequency
- Set channel 2 to mode 0 (one-shot), count = 11932 (~10 ms)
- Start LAPIC timer, start PIT, wait for PIT OUT (poll port 0x61 bit 5)
- Read LAPIC remaining count, compute delta

Store result in `static APIC_TICKS_PER_MS: AtomicU64`.

**Completion Criteria:**
- Returns a non-zero value (for QEMU: typically 100–10000 ticks/ms depending on config)
- `APIC_TICKS_PER_MS` set before scheduler tick init
- Monitor command `cpu timer` shows calibrated ticks/ms value
- Graceful fallback: if HPET unavailable, uses PIT

---

### 1.4.2 Periodic scheduler tick

`[ ] TODO`

**Prerequisites:** 1.4.1

**Spec:**
After calibration, reconfigure timer in **periodic mode** at `SCHEDULER_TICK_HZ` (default 100 Hz = 10 ms/tick):
- `const SCHEDULER_TICK_HZ: u64 = 100` in `config.rs`
- `init_scheduler_tick()` computes `initial_count = APIC_TICKS_PER_MS * (1000 / SCHEDULER_TICK_HZ)`, sets APIC timer to periodic mode
- Timer ISR increments `static TICK_COUNT: AtomicU64`
- Provide `pub fn current_tick() -> u64` and `pub fn ticks_to_ms(ticks: u64) -> u64`
- TSC-Deadline mode: stretch goal; check `CpuFeatures::get().tsc_deadline` (from 1.6) and use it if available

**Completion Criteria:**
- `TICK_COUNT` increments at ~100 Hz (verified: read count, wait in QEMU, read again)
- `ticks_to_ms` gives values consistent with calibration
- Timer ISR doesn't break monitor or serial output

---

## 1.5 x2APIC Support

### What we actually have (audit findings)

`apic.rs` already reads `IA32_APIC_BASE` and has `ApicAccessMode { Disabled, XApic, X2Apic }` and `apic_base_info()` that returns mode. Mode detection is there. However, **all actual register access is MMIO-based** (`local_apic_read/write` use PHYS_OFFSET + base address). There are no x2APIC MSR accessors. So the kernel will GPF if firmware boots into x2APIC mode.

---

### 1.5.1 Detect and report APIC mode

`[x] DONE (partially)`

Detection works (`apic_base_info()`, `ApicAccessMode`). Boot log should already report this. The only gap: need to verify `cpu apic` monitor command surfaces the mode clearly.

**Remaining:** Add `ApicAccessMode` to the `cpu apic` monitor command output if not already there.

---

### 1.5.2 Abstract LAPIC access behind mode-agnostic interface

`[ ] TODO`

**Prerequisites:** None (self-contained change to `apic.rs`)

**Context:**
All LAPIC register reads/writes go through `local_apic_read(reg_offset)` / `local_apic_write(reg_offset, val)` which do MMIO. These two functions need to check detected mode and either use MMIO (xAPIC) or `rdmsr`/`wrmsr` at `0x800 + (reg_offset >> 4)` (x2APIC). The callers don't change.

**Spec:**
Modify `local_apic_read` / `local_apic_write` in `apic.rs`:
- Call `apic_base_info()` once at first use; cache the result in a `static OnceCell<ApicAccessMode>`
- If `XApic`: current MMIO path (unchanged)
- If `X2Apic`: use `RDMSR`/`WRMSR` with MSR = `0x800 + (offset / 16)`; note x2APIC register width is 32-bit for most, 64-bit for ICR — handle ICR as a special case
- If `Disabled`: panic with a clear message
- No changes to callers (timer, EOI, APIC ID, etc.)

**Completion Criteria:**
- Kernel boots without GPF in QEMU with `-cpu host` or `-cpu Skylake-Server,+x2apic`
- xAPIC path unchanged (QEMU default still works)
- APIC timer fires in both modes

---

## 1.6 CPUID Feature Abstraction

`[ ] TODO`

**Prerequisites:** None — isolated, no hardware side effects

### What we actually have (audit findings)

`cpu.rs` uses `raw_cpuid::CpuId` for feature detection, but **checks are scattered**:
- `cpu.rs`: `CpuId::new()` used to check `has_xsave`, control register setup
- `apic.rs`: inline CPUID checks for x2APIC detection
- `interrupts/mod.rs`: likely has inline CPUID for TSC/APIC features
- No central feature cache — each check re-executes `CPUID` instruction

**Spec:**
Implement `kernel/src/cpu_features.rs`:

```rust
pub struct CpuFeatures {
    pub x2apic: bool,
    pub tsc_deadline: bool,
    pub rdtscp: bool,
    pub fsgsbase: bool,   // needed for Phase 10 TLS
    pub smep: bool,
    pub smap: bool,       // enable both in 1.6 if present
    pub xsave: bool,
    pub avx: bool,
    pub avx2: bool,
}

impl CpuFeatures {
    pub fn detect() -> Self { ... }  // executes CPUID, populates all fields
    pub fn get() -> &'static Self    // panics if detect() not called first
}
```

- `detect()` called once in boot sequence (before `apic.rs` initializes APIC)
- All existing scattered `CpuId::new()` calls replaced by `CpuFeatures::get().<field>`
- Enable SMEP/SMAP in CR4 here if present (currently may be done in `cpu.rs` — consolidate)
- Monitor command `cpu features` prints all fields

**Completion Criteria:**
- `CpuFeatures::get()` panics with clear message if called before `detect()`
- All scattered CPUID checks replaced by `CpuFeatures::get()` field accesses
- `cpu features` monitor command output matches QEMU's CPU model
- SMEP/SMAP enabled if CPU supports them (verify with CR4 read in monitor)

---

## 1.7 SMP Bring-up

`[ ] TODO — deferred to after Phase 2 scheduler`

Not spec'd at leaf level here. Will become a sub-plan once the scheduler is stable on BSP.
High-level tasks noted for awareness:
- Parse AP LAPIC IDs from MADT (parsing exists, AP extraction needed)
- Per-CPU storage (GDT, IDT, TSS, stack, `gs`-relative)
- INIT-SIPI-SIPI sequence + 16→64-bit AP trampoline
- TLB shootdown IPI infrastructure
- AP joins scheduler runqueue

---

## Revised Dependency Graph

```
1.6 CPUID (no deps)
    └─► 1.5.2 LAPIC abstraction
            └─► 1.4.1 Calibration
                    └─► 1.4.2 Periodic tick

1.1.2 VA Allocator (no deps)
    └─► 1.1.3 Runtime Mapper
            └─► 1.1.4 Stack Allocator
            └─► 1.2.4 Framebuffer Driver (also needs 1.2.2)

1.2.1 Fix driver_data (no deps)
    └─► 1.2.2 PCI → DriverManager
            └─► 1.2.3 IRQ ownership

1.1.5 Boot handoff docs (no deps, do anytime)
1.3 TSS/IST (DONE)
1.5.1 APIC mode detection (DONE, minor monitor polish)
```

## Suggested Order of Work

1. **1.6 CPUID** — small, isolated, no hardware risk, unblocks x2APIC
2. **1.5.2 LAPIC abstraction** — unblocks calibration, zero regression risk (xAPIC path unchanged)
3. **1.4.1 + 1.4.2** — calibration + scheduler tick; unblocks Phase 2
4. **1.2.1 Fix driver_data** — isolated, clean break (nothing uses it yet)
5. **1.1.2 VA Allocator** — pure Rust bookkeeping, no hardware
6. **1.1.3 Runtime Mapper** — needs VA alloc + persistent frame alloc (both done)
7. **1.1.4 Stack Allocator** — needs 1.1.2 + 1.1.3
8. **1.2.2 PCI → DriverManager** — closes the PCI loop, removes hardcoded xHCI init
9. **1.2.3 IRQ ownership** — cleans up handlers.rs
10. **1.2.4 Framebuffer driver** — lowest urgency, can do last or skip to Phase 2
11. **1.1.5 Boot handoff docs** — anytime, no blockers

SMP (1.7) waits for Phase 2 scheduler.
