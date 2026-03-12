# Boot Flow Plan

## Scope

Track the documented boot flow from UEFI entry through `kernel_entry` and into the validated handoff boundary.

This plan records what is already implemented, not aspirational design.

## Plan vs Repo

- Older narrative docs described the flow correctly at a high level, but they did not distinguish binding truths from implementation risks.
- The current repo is single-binary boot; that must be stated as truth, not as a historical option.
- After the bootloader subtree sweep, it is clearer that the boot contract spans more than `main.rs` and `boot_sequence.rs`: boot-time memory, ACPI, inventory, output, and firmware-context helpers all participate in the real handoff boundary.

## Implemented Invariants

- [x] IMPLEMENTED: Boot begins at `bootloader/src/main.rs::efi_main` in a unified UEFI executable.
- [x] IMPLEMENTED: The bootloader gathers graphics, memory-map, ACPI, firmware, boot-time, CPU, hardware-inventory, and output-context data before kernel handoff.
- [x] IMPLEMENTED: The handoff is copied into persistent `LOADER_DATA` before firmware services are exited.
- [x] IMPLEMENTED: `ExitBootServices` runs before `kernel_entry`, and the kernel validates `boot_services_exited == 1` at entry.
- [x] IMPLEMENTED: The bootloader transfers control by directly calling `theseus_kernel::kernel_entry` with the physical address of the copied handoff.
- [x] IMPLEMENTED: The boot path records framebuffer, memory-map, ACPI, firmware, boot-time, boot-device, CPU, and hardware-inventory context in the handoff before leaving boot services.

## TODO

- [ ] TODO: Replace the conservative fixed image-span estimate with exact documented image extent accounting if the implementation changes.
- [ ] TODO: Add scoped docs/agent rules if the bootloader docs or source tree need stricter local rules.

## Risks

- [!] RISK: `set_kernel_image_from_loaded_image` currently uses a fixed 16 MiB kernel image span; this is conservative, not exact.
- [!] RISK: The mainline code path uses `exit_boot_services(None)` directly, so any stronger wording about memory-map-key refresh/retry logic would currently be fiction unless the code changes.

## Related Axioms

- `../axioms/boot.md#A1:-The-kernel-boots-as-a-single-UEFI-executable`
- `../axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry`
- `../axioms/boot.md#A3:-Kernel-image-metadata-is-derived-from-the-live-binary-not-a-separately-parsed-on-disk-image`

## Implementing Modules

- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `bootloader/src/acpi.rs`
- `bootloader/src/hardware.rs`
- `bootloader/src/display.rs`
- `bootloader/src/system_info.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `bootloader/src/drivers/mod.rs`
- `bootloader/src/drivers/manager.rs`
- `bootloader/src/drivers/qemu_debug.rs`
- `bootloader/src/drivers/raw_serial.rs`
- `bootloader/src/drivers/uefi_serial.rs`
- `shared/src/handoff.rs`
- `kernel/src/lib.rs`
- `kernel/src/handoff.rs`
