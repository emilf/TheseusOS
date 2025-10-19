# Kernel Architecture

Once `kernel_entry` is called, control never returns to firmware. The kernel brings the platform into a predictable state, maps its address space, lights up interrupts, and hands off to higher-level services. This page summarizes the major phases and where to look in the source.

## Entry: `kernel_entry`
- Location: `kernel/src/lib.rs`.
- Initializes the logging subsystem so that later steps can emit diagnostics.
- Validates the `handoff` structure (see [Boot Sequence & Handoff](boot-sequence.md)) and optionally dumps it when `config::VERBOSE_KERNEL_OUTPUT` is set.
- Installs the handoff pointer globally so other subsystems (framebuffer, interrupts) can reference it.
- Calls into `environment::setup_kernel_environment(handoff, kernel_physical_base)`.

## Phase 1: Low-Half Setup
Implemented in `kernel/src/environment.rs::setup_kernel_environment`.

1. **Interrupt lockdown** — `interrupts::disable_all_interrupts()` masks everything, including NMIs, so no firmware IRQ fires during the transition.
2. **Segments and TSS** — `gdt::setup_gdt()` loads a fresh GDT, configures the Task State Segment, and prepares Interrupt Stack Table (IST) entries for fault handlers.
3. **Control registers** — `cpu::setup_control_registers()` ensures CR0/CR4 bits for long mode, global pages, and SSE exceptions are enabled before paging changes.
4. **Page tables** — `MemoryManager::new(handoff)` constructs a fresh PML4:
   - Identity map for the low 1 GiB so the old addresses stay valid temporarily.
   - Higher-half mapping at `KERNEL_VIRTUAL_BASE`.
   - `PHYS_OFFSET` linear map so physical memory can be reached via `0xFFFF_8000_0000_0000 + pa`.
   - Framebuffer and LAPIC MMIO regions.
5. **CR3 switch** — `activate_virtual_memory()` loads the new root, and the kernel marks the `PHYS_OFFSET` mapping as active.
6. **Initial IDT** — An early IDT is installed so faults during the upcoming jump are handled gracefully.

## Phase 2: High-Half Transition
- `MemoryManager::jump_to_high_half(...)` uses `stack::switch_to_kernel_stack_and_jump` to land in `after_high_half_entry`.
- Before jumping, mappings for the dedicated kernel stack and IST stacks are verified (`memory::virt_range_has_flags`).
- If the jump fails (should not happen) the kernel aborts via `boot::abort_with_context`.

## Phase 3: `continue_after_stack_switch`
All subsequent work executes with higher-half addresses:

1. **Reinstall IDT** — `interrupts::setup_idt()` is called again so the handlers point at their high-half locations and IST stacks.
2. **Timer vector** — `interrupts::install_timer_vector_runtime()` patches vector 0x40 with the full 64-bit handler address.
3. **CPU features** — `cpu::setup_floating_point` enables SSE, and `cpu::setup_msrs` toggles MSRs such as EFER.SCE.
4. **LAPIC verification** — A one-shot timer is fired and checked (`lapic_timer_start_oneshot`) to confirm interrupts are working before continuing.
5. **Allocator bootstrap**:
   - The temporary heap from the handoff is remapped at `memory::TEMP_HEAP_VIRTUAL_BASE` and wired into the shared allocator shim.
   - `memory::map_kernel_heap_x86` provisions a permanent 1 MiB heap at `memory::KERNEL_HEAP_BASE`.
   - `physical_memory::init_from_handoff` consumes the firmware memory map and builds the persistent bitmap allocator.
   - Temporary heap and identity mappings are optionally torn down to catch stale pointers.
6. **UEFI runtime** — `set_virtual_address_map_runtime` calls `SetVirtualAddressMap` so runtime services may be invoked through their new virtual addresses; `log_uefi_firmware_time` demonstrates the mapping by querying the firmware clock.

## Phase 4: Platform Services
- `drivers::system::init()` inspects ACPI tables (if available), reports CPU/APIC counts, and seeds the driver manager (`drivers::manager`).
- Framebuffer animation is prepared via `framebuffer::init_framebuffer_drawing` and a heartbeat pattern is drawn using handoff data.
- `lapic_timer_start_periodic(50_000)` starts a 50 Hz timer to drive the animation and increment the global tick counter.
- Interrupts are re-enabled globally (`x86_64::instructions::interrupts::enable()`).
- Depending on `config`, the kernel:
  - Activates the interactive serial monitor (`monitor::init()`).
  - Runs a COM1 reverse echo loop (`serial_debug::run_reverse_echo_session()`).
  - Either idles with `hlt` in a loop (`config::KERNEL_SHOULD_IDLE`) or exits QEMU via the `isa-debug-exit` device.

## Configuration Overview
Centralized in `kernel/src/config.rs`:
- Logging verbosity (`VERBOSE_KERNEL_OUTPUT`) and output routing.
- Serial, monitor, and driver toggles.
- Whether to keep the legacy 1 GiB identity mapping (`MAP_LEGACY_PHYS_OFFSET_1GIB`).

## Where to Explore Next
- Dig into [Memory Management](memory-management.md) for details on `MemoryManager`, page-table helpers, and the frame allocator.
- Jump to [Hardware & Drivers](hardware-and-drivers.md) to see how interrupts, devices, and the monitor connect.
- Review [Development & Debugging](development-and-debugging.md) to learn how to rebuild and instrument the system under QEMU.
