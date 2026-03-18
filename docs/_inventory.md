# TheseusOS Repository Inventory

Evidence-only notes about the kernel as it exists on `main`. Every bullet below points to a file path and symbol or code location in the repo.

## Boot entrypoint, UEFI usage, and `ExitBootServices` timing

- The firmware entrypoint is `efi_main`, exposed with `#[entry]` in `bootloader/src/main.rs`. Source: `bootloader/src/main.rs` (`fn efi_main() -> Status`).
- The bootloader installs pre-exit allocators backed by UEFI Boot Services pool allocation before doing other work. Source: `bootloader/src/main.rs` (`theseus_shared::allocator::install_pre_exit_allocators`, `pre_exit_alloc`, `pre_exit_dealloc`).
- The bootloader collects graphics, memory-map, ACPI, system, hardware-inventory, and loaded-image-path data before handing control to the kernel. Source: `bootloader/src/main.rs` (`collect_graphics_info`, `collect_memory_map`, `collect_acpi_info`, `collect_system_info`, `collect_hardware_inventory_info`, `collect_loaded_image_path`).
- The kernel image metadata is derived from the in-process `theseus_kernel::kernel_entry` symbol rather than from loading a separate ELF image. Source: `bootloader/src/boot_sequence.rs` (`set_kernel_image_from_loaded_image`).
- The bootloader allocates a temporary heap as `LOADER_DATA`, explicitly avoiding overlap with the computed kernel image span. Source: `bootloader/src/main.rs` (`allocate_memory_non_overlapping` call storing `HANDOFF.temp_heap_base/temp_heap_size`); `bootloader/src/memory.rs` (`allocate_memory_non_overlapping`).
- The handoff structure is copied into a newly allocated persistent `LOADER_DATA` buffer immediately before leaving firmware. Source: `bootloader/src/boot_sequence.rs` (`jump_to_kernel_with_handoff`, allocation via `crate::memory::allocate_memory`, copy to `handoff_phys`).
- `ExitBootServices` is called inside `jump_to_kernel_with_handoff` after the handoff copy is made and before `kernel_entry` is called. Source: `bootloader/src/boot_sequence.rs` (`uefi::boot::exit_boot_services(None)`, then `boot_services_exited = 1`, then `entry(handoff_phys as u64)`).
- The kernel expects boot services to already be exited when `kernel_entry` runs. Source: `kernel/src/handoff.rs` (`validate_handoff`, check `boot_services_exited must be 1 at kernel entry`); `kernel/src/lib.rs` (`kernel_entry`).

## Memory map ingestion and representation

- The shared handoff ABI includes raw UEFI memory-map pointer, descriptor size/version, entry count, and total byte size. Source: `shared/src/handoff.rs` (`pub struct Handoff`, fields `memory_map_buffer_ptr`, `memory_map_descriptor_size`, `memory_map_descriptor_version`, `memory_map_entries`, `memory_map_size`).
- The bootloader stores the raw memory-map buffer pointer directly from `uefi::boot::memory_map(MemoryType::LOADER_DATA)`. Source: `bootloader/src/boot_sequence.rs` (`collect_memory_map`, storing `mmap.buffer().as_ptr() as u64` into `HANDOFF.memory_map_buffer_ptr`).
- Early boot frame allocation is driven directly from the handoff memory map. Source: `kernel/src/memory/frame_allocator.rs` (`BootFrameAllocator::from_handoff`, reads `memory_map_buffer_ptr`, `memory_map_descriptor_size`, `memory_map_entries`).
- The persistent physical allocator is also constructed from the handoff memory map. Source: `kernel/src/physical_memory.rs` (`init_from_handoff`, `memory_map_ptr`, `parse_memory_regions`).
- The kernel can access the memory map through three modes: already-virtual pointer, `PHYS_OFFSET` translation, or early identity mapping. Source: `kernel/src/physical_memory.rs` (`memory_map_ptr`).
- During page-table construction, the kernel remaps the handoff memory-map buffer into a dedicated high virtual window and rewrites `handoff.memory_map_buffer_ptr` to the remapped address. Source: `kernel/src/memory.rs` (`MemoryManager::map_phys_offset_windows`, constant `HANDOFF_MEMMAP_WINDOW_BASE`).
- The interactive monitor has a command to dump the UEFI memory map out of the live handoff structure. Source: `kernel/src/monitor/commands/tables.rs` (`cmd_memory_map`).

## Paging and mapping strategy

- The kernel’s linked higher-half base is `0xFFFF_FFFF_8000_0000`. Source: `kernel/src/memory.rs` (`pub const KERNEL_VIRTUAL_BASE`); also `bootloader/src/boot_sequence.rs` (`HANDOFF.kernel_virtual_base = 0xFFFFFFFF80000000`).
- The linear physical mapping base is `0xFFFF_8000_0000_0000`. Source: `kernel/src/memory.rs` (`pub const PHYS_OFFSET`).
- The temporary heap is mapped at `0xFFFF_FFFF_A000_0000`, and the permanent heap at `0xFFFF_FFFF_B000_0000`. Source: `kernel/src/memory.rs` (`TEMP_HEAP_VIRTUAL_BASE`, `KERNEL_HEAP_BASE`).
- `MemoryManager::new` builds a fresh PML4 and populates: low identity mappings, higher-half kernel mappings, framebuffer/temp-heap mappings, a `PHYS_OFFSET` window, and LAPIC/IOAPIC MMIO mappings. Source: `kernel/src/memory.rs` (`MemoryManager::new`, helper calls `map_boot_identity_region`, `map_kernel_high_half_region`, `map_boot_resources`, `map_phys_offset_windows`, `map_platform_mmio_regions`).
- Low physical memory is identity-mapped with 2 MiB pages up to a computed limit that covers conventional memory, kernel image, temp heap, and framebuffer. Source: `kernel/src/memory.rs` (`map_boot_identity_region`); `kernel/src/memory/mapping.rs` (`identity_map_range_2mb_alloc`).
- The kernel image is mapped into the high half with 4 KiB pages. Source: `kernel/src/memory.rs` (`map_kernel_high_half_region`); `kernel/src/memory/mapping.rs` (`map_kernel_high_half_4k_alloc`).
- The `PHYS_OFFSET` linear window is populated with 2 MiB pages over a computed physical range. Source: `kernel/src/memory.rs` (`map_phys_offset_windows`); `kernel/src/memory/mapping.rs` (`map_phys_offset_range_2mb_alloc`).
- LAPIC and IOAPIC MMIO regions are explicitly mapped during boot page-table construction. Source: `kernel/src/memory.rs` (`map_platform_mmio_regions`); `kernel/src/memory/mapping.rs` (`map_lapic_mmio_alloc`, `map_io_apic_mmio_alloc`).
- After constructing the new page tables, the kernel loads CR3 with `activate_virtual_memory` and marks `PHYS_OFFSET` as active. Source: `kernel/src/environment.rs` (`setup_kernel_environment`, calls `activate_virtual_memory(mm.page_table_root())` and `set_phys_offset_active()`); `kernel/src/memory.rs` (`activate_virtual_memory`).
- The high-half transition is a non-returning jump computed from the runtime physical base plus symbol offset. Source: `kernel/src/memory.rs` (`MemoryManager::jump_to_high_half`).
- After the allocator is live, the kernel optionally unmaps the temporary heap and the identity-mapped kernel image to catch stale low-address references. Source: `kernel/src/environment.rs` (`continue_after_stack_switch`, calls `unmap_temporary_heap_x86` and `unmap_identity_kernel_x86`).

## Allocator stages and when `alloc` becomes available

- Before `ExitBootServices`, the bootloader’s allocator backend uses UEFI `allocate_pool` / `free_pool`. Source: `bootloader/src/main.rs` (`pre_exit_alloc`, `pre_exit_dealloc`).
- The bootloader allocates a 1 MiB temporary heap for kernel bring-up, if available, and records it in the handoff. Source: `bootloader/src/main.rs` (`TEMP_HEAP_SIZE`, `allocate_memory_non_overlapping`).
- In the kernel, the shared allocator shim is present immediately, but the higher-half temp heap is only armed after the high-half transition. Source: `kernel/src/lib.rs` (`Allocator shim active (pre-exit backend)` log); `kernel/src/environment.rs` (`continue_after_stack_switch`, `theseus_shared::allocator::init_kernel_heap(base, size)` using `TEMP_HEAP_VIRTUAL_BASE`).
- The permanent heap is mapped at `KERNEL_HEAP_BASE` using fresh frames, then the shared allocator is switched to that heap. Source: `kernel/src/environment.rs` (`continue_after_stack_switch`, `map_kernel_heap_x86`, `theseus_shared::allocator::init_kernel_heap`, `switch_to_kernel_heap`).
- The early frame allocator is `BootFrameAllocator`, constructed from the UEFI memory map. Source: `kernel/src/memory/frame_allocator.rs` (`BootFrameAllocator::from_handoff`).
- The long-lived physical allocator is a bitmap allocator initialized after the permanent heap exists. Source: `kernel/src/physical_memory.rs` (`init_from_handoff`), invoked from `kernel/src/environment.rs` (`continue_after_stack_switch`).
- `PersistentFrameAllocator` is the runtime frame source after `physical_memory::init_from_handoff` completes. Source: `kernel/src/physical_memory.rs` (`pub struct PersistentFrameAllocator`); `kernel/src/environment.rs` (`continue_after_stack_switch`, `let mut frame_alloc = crate::physical_memory::PersistentFrameAllocator;`).

## Logging, debug channels, and panic behavior

- The kernel initializes its logging subsystem at the start of `kernel_entry`. Source: `kernel/src/lib.rs` (`crate::logging::init()`).
- Logging is designed to avoid heap allocation and to be usable in panic and interrupt contexts. Source: `kernel/src/logging/mod.rs` (`format_log_entry` stack buffer design); `kernel/src/logging/output.rs` (`write_bytes_any_context`).
- Logging output targets include QEMU debug port, serial, both, or none. Source: `kernel/src/logging/output.rs` (`enum OutputTarget` and related helpers); `kernel/src/logging/mod.rs` module docs.
- The logging fallback path writes directly to serial if the serial driver is unavailable. Source: `kernel/src/logging/output.rs` (`write_serial`, tries `crate::drivers::serial::write_bytes_direct` then falls back to direct port I/O).
- The panic handler logs panic information without using the allocator and then exits QEMU with an error status. Source: `kernel/src/panic.rs` (`#[panic_handler]`, comments and `theseus_shared::qemu_exit_error!()`).
- The development docs describe the current intended logging split: ERROR/WARN/INFO to QEMU port 0xE9 and serial, DEBUG/TRACE to QEMU only by default. Source: `docs/development-and-debugging.md` (`Logging and output routing` section around the `logoutput` command examples).

## CPU / x86_64 architectural assumptions

- The build is `no_std`, and the kernel enables the nightly `abi_x86_interrupt` feature. Source: `kernel/src/lib.rs` (`#![no_std]`, `#![feature(abi_x86_interrupt)]`).
- The UEFI target JSON disables the red zone, uses soft-float ABI, and names `efi_main` as the entry symbol. Source: `x86_64-unknown-uefi-dwarf.json` (`disable-redzone`, `rustc-abi`, `entry-name`).
- The kernel configures CR0/CR4 before loading its own page tables. Source: `kernel/src/environment.rs` (`setup_control_registers()` in `setup_kernel_environment`); `kernel/src/cpu.rs` (control-register setup logic).
- The kernel re-enables `CR4.OSFXSR`, `CR4.OSXMMEXCPT_ENABLE`, and `CR4.PAGE_GLOBAL` after the high-half switch. Source: `kernel/src/environment.rs` (`continue_after_stack_switch`, CR4 writes).
- SSE is explicitly enabled after the high-half transition. Source: `kernel/src/environment.rs` (`continue_after_stack_switch`, `setup_floating_point`).
- MSRs are configured after SSE setup. Source: `kernel/src/environment.rs` (`continue_after_stack_switch`, `setup_msrs`).
- The kernel relies on a GDT + TSS with dedicated IST stacks for double fault, NMI, machine check, and page fault. Source: `kernel/src/gdt.rs` (`TaskStateSegment`, `interrupt_stack_table[...]`, `setup_gdt`, `refresh_tss_ist`).
- The Local APIC base is read from `IA32_APIC_BASE` and the current runtime access mode (disabled/xAPIC/x2APIC) is decoded from the same MSR, but live register access still uses xAPIC-style MMIO through `PHYS_OFFSET`. Source: `kernel/src/interrupts/apic.rs` (`apic_base_info`, `get_apic_base`, `read_apic_register`, `write_apic_register`).
- The codebase now distinguishes x2APIC capability from x2APIC enablement in monitor tooling, but it still does not implement a main-path x2APIC bring-up switch. Source: `kernel/src/monitor/commands/cpu.rs` (CPUID feature reporting plus `apic_base_info` output); no corresponding main-path x2APIC init symbol found in `kernel/src/interrupts/*.rs`.

## Interrupts and exceptions status

- The kernel installs an IDT in low-half setup and reinstalls it after the high-half transition. Source: `kernel/src/environment.rs` (`setup_kernel_environment`, `continue_after_stack_switch`, both call `setup_idt()`).
- Interrupt handlers use the `extern "x86-interrupt"` ABI. Source: `kernel/src/interrupts/handlers.rs` (handler definitions such as `handler_spurious`); feature enabled in `kernel/src/lib.rs`.
- The GDT/TSS setup includes dedicated IST stacks for critical exceptions. Source: `kernel/src/gdt.rs` (`IST_INDEX_DF`, `IST_INDEX_NMI`, `IST_INDEX_MC`, `IST_INDEX_PF`; TSS setup).
- The kernel has LAPIC timer support and explicitly tests delivery during bring-up by starting a one-shot timer and checking whether the tick counter advanced. Source: `kernel/src/environment.rs` (`continue_after_stack_switch`, `lapic_timer_configure`, `lapic_timer_start_oneshot`, `timer_tick_count`).
- The kernel later starts a periodic LAPIC timer for framebuffer animation. Source: `kernel/src/environment.rs` (`continue_after_stack_switch`, `lapic_timer_start_periodic(50_000)`).
- The code explicitly masks legacy PIC interrupts during APIC-based bring-up. Source: `kernel/src/interrupts/apic.rs` (`disable_all_interrupts`, `mask_pic_interrupts`).
- The serial driver is designed for interrupt-driven RX routed through the IOAPIC. Source: `kernel/src/drivers/serial.rs` (module docs; `enable_interrupts`; `program_io_apic_entry`; `SERIAL_RX_VECTOR`).
- There is monitor support for inspecting IDT, GDT, APIC, timer, and page tables at runtime. Source: `kernel/src/monitor/commands/tables.rs`, `kernel/src/monitor/commands/system.rs`, `kernel/src/monitor/commands/memory.rs`.

## SMP status

- ACPI parsing extracts processor/APIC information, including boot processor and application processor APIC IDs. Source: `kernel/src/acpi/mod.rs` (`initialize_acpi`, `processor_info.boot_processor`, iterating `application_processors`); `kernel/src/acpi/madt.rs` (`parse_madt`, `cpu_apic_ids`).
- The platform summary records CPU count, IO APIC count, and local APIC address. Source: `kernel/src/acpi/mod.rs` (`PlatformInfo` population); `kernel/src/drivers/system.rs` (`system::init`).
- The current repo does not show an AP startup path or SMP scheduler bring-up in the kernel proper. Evidence: CPU enumeration exists in `kernel/src/acpi/*.rs`, but no `start_aps`, `smp`, or AP entry module exists under `kernel/src/` on this branch.

## Driver stack, USB, input, and runtime inspection

- The kernel driver subsystem exposes a central driver-manager layer plus PCI, serial, system, trait, and USB modules. Source: `kernel/src/drivers/mod.rs`.
- The driver manager maintains registered drivers and discovered devices, probes devices on registration/addition, and dispatches IRQs by device metadata. Source: `kernel/src/drivers/manager.rs` (`DriverManager`, `register_driver`, `add_device`, `probe_pending_devices`, `handle_irq`).
- PCI enumeration is ECAM-based from ACPI MCFG regions and includes recursive bridge-aware bus scanning, BAR decoding, and capability discovery for MSI/MSI-X. Source: `kernel/src/drivers/pci.rs` (`enumerate`, bridge/device structs and capability parsing helpers).
- The USB subsystem currently centers on firmware handoff and xHCI registration/runtime helpers rather than a generic multi-controller abstraction. Source: `kernel/src/drivers/usb/mod.rs` (`ensure_legacy_usb_handoff`, `register_xhci_driver`, runtime service exports).
- Input now has a dedicated keyboard module under `kernel/src/input/`, indicating input handling has been split out from ad-hoc device-only code paths. Source: repo file layout under `kernel/src/input/`.
- The runtime monitor includes explicit PCI and USB command modules in addition to the earlier system/memory/table views. Source: `kernel/src/monitor/commands/pci.rs`, `kernel/src/monitor/commands/usb.rs`, `kernel/src/monitor/commands/devices.rs`.

## QEMU runner and host-side execution tooling

- The repo now includes a Rust QEMU runner tool at `tools/theseus-qemu/` that can print argv, emit JSON artifacts, or execute QEMU directly. Source: `tools/theseus-qemu/src/main.rs` (`Cli`, `Cmd`, `main`).
- The runner supports named profiles (`default`, `min`, `usb-kbd`) and explicit relay/QMP/HMP/serial endpoint toggles. Source: `tools/theseus-qemu/src/main.rs` (`Profile`, `Args`).
- The runner can build before launch, apply timeout wrapping, and force success on a configured kernel output marker. Source: `tools/theseus-qemu/src/main.rs` (`Args.build`, `Args.timeout_secs`, `Args.success_marker`, `run_qemu`).
- The repo also ships relay-install and systemd-user unit scripts for serial, debugcon, monitor, and QMP forwarding. Source: `scripts/install-qemu-relays.sh`, `scripts/qemu_*_relay.service`, `scripts/qemu_debugcon_logger.service`.

## Build artifacts, targets, linker, and features

- The workspace consists of `bootloader`, `kernel`, and `shared`. Source: `Cargo.toml` (`[workspace] members = ["bootloader", "kernel", "shared"]`).
- The shared dependency set includes `uefi = 0.35` with `default-features = false` and `features = ["alloc"]`. Source: `Cargo.toml` (`[workspace.dependencies]`).
- Panic strategy is `abort` in both dev and release profiles. Source: `Cargo.toml` (`[profile.dev] panic = "abort"`, `[profile.release] panic = "abort"`).
- The UEFI DWARF target is COFF/PE-style, `os = "uefi"`, `entry-name = "efi_main"`, and emits `.efi` executables. Source: `x86_64-unknown-uefi-dwarf.json`.
- A separate legacy linker script exists that links a kernel at `VIRTUAL_BASE = 0xffffffff80000000` with `ENTRY(kernel_main)`, but the active single-binary boot flow derives the image from `efi_main` / `kernel_entry`. Source: `linker.ld`; compare with `bootloader/src/main.rs` and `bootloader/src/boot_sequence.rs`.
- The repo has both `x86_64-theseus.json` and `x86_64-unknown-uefi-dwarf.json` target files in the root, indicating at least two target configurations are tracked in-repo. Source: repo root file list.

## Notable current mismatches / reality flags to reconcile later

- Current top-level docs still describe some behavior at a high level without marking which parts are implementation facts versus intent. Source: existing docs under `docs/*.md` compared with the repo-wide documentation plan now recorded in workspace `AGENTS.md`.
- `bootloader/src/boot_sequence.rs::set_kernel_image_from_loaded_image` uses a conservative fixed 16 MiB kernel image span instead of deriving exact section extents. This is an implementation fact, but it should likely be documented as such rather than implied to be exact image accounting. Source: `bootloader/src/boot_sequence.rs` (`aligned_size = 16 * 1024 * 1024`).
- The mainline boot path uses `exit_boot_services(None)` directly rather than a visibly documented "refresh key and retry" flow. Source: `bootloader/src/boot_sequence.rs` (`uefi::boot::exit_boot_services(None)`).
- The repo includes historical docs in `docs/archive/` that describe older architecture and may conflict with the current single-binary implementation if read as authoritative. Source: `docs/archive/*` alongside current `docs/*.md`.
