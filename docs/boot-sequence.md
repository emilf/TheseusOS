# Boot Sequence & Handoff

TheseusOS boots as a single UEFI application. The executable starts in firmware mode, performs all discovery while boot services are live, and then jumps straight into the Rust kernel entry point. This page walks you through the major stages and the data that flows into the handoff structure shared with the kernel.

## 1. UEFI Entrypoint
- `bootloader/src/main.rs` exposes `efi_main`, the firmware entry symbol.
- The shared allocator shim is installed immediately so both bootloader and kernel can use `alloc`.
- `drivers::manager::OutputDriver` selects between console, serial, and QEMU debug-con outputs before logging begins.
- Configuration toggles such as `VERBOSE_OUTPUT` (see `bootloader/src/main.rs`) control how much diagnostic text is printed.

## 2. System Discovery While Services Are Live
The bootloader queries firmware APIs in a predictable order so that dependencies are ready when the kernel starts:

| Stage | Module | Purpose |
| --- | --- | --- |
| Graphics | `bootloader::display` | Collect GOP information (resolution, pixel format, framebuffer base). |
| Memory | `bootloader::memory` | Retrieve the UEFI memory map, reserve space for the handoff block, and allocate a temporary heap. |
| ACPI | `bootloader::acpi` | Locate the RSDP and record table addresses for later parsing. |
| Hardware inventory | `bootloader::hardware` | Enumerate device handles and classify them (PCI, USB, storage, etc.). |
| System info | `bootloader::system_info` | Capture firmware vendor strings, boot timestamps, and CPU feature flags. |

Each stage appends structured data into the global `HANDOFF` instance (`theseus_shared::handoff::Handoff`), which is later passed to the kernel verbatim.

## 3. Preparing the Handoff Structure
- **Image accounting** — The bootloader looks up the symbol for `kernel_entry` to compute the kernel image's physical base, virtual base, and size. This avoids parsing the on-disk ELF at runtime.
- **Temporary heap** — A non-overlapping heap region is reserved and stored in `temp_heap_base`/`temp_heap_size`. The kernel maps it into the higher-half at `TEMP_HEAP_VIRTUAL_BASE` for early allocations.
- **Memory map copy** — The raw descriptor list from firmware is copied into bootloader-owned memory so the kernel can iterate without calling UEFI.
- **Debug signatures** — `THESEUS_DEBUG_SIGNATURE` (see `bootloader/src/main.rs`) provides a recognizable marker for tooling and GDB sessions.

You can inspect the populated structure during debugging by pausing before `ExitBootServices` and dumping the pointer that gets passed to `kernel_entry`.

## 4. Leaving Firmware
- `ExitBootServices` is invoked once the memory map key is refreshed; failure to recapture the key results in an abort.
- Control transfers directly to the kernel via a function call (`kernel_entry(handoff_addr)`), no trampoline or boot protocol switching required.
- After this point all firmware services are gone—every piece of state the kernel needs must be inside the handoff.

## 5. Kernel Expectations
The kernel module `kernel/src/handoff.rs` validates the structure before continuing. Critical requirements:

- `handoff.size` must describe the full struct.
- Memory map descriptors must be aligned and cover every region the bootloader reported.
- Graphics and framebuffer pointers must remain accessible; the kernel maps them into its own address space during initialization.

If validation fails, the kernel aborts via `boot::abort_with_context`, printing diagnostics to both serial and the QEMU debug port.

## Next Steps
- Continue to [Kernel Architecture](kernel-architecture.md) for the control flow after `kernel_entry`.
- When you want to study how the handoff data gets mapped into page tables, jump to [Memory Management](memory-management.md).
