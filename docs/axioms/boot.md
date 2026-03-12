# Boot Axioms

Binding architectural truths for the current boot flow.

These are the invariants the boot path relies on today.

## A1: The kernel boots as a single UEFI executable

**REQUIRED**

The current TheseusOS boot path is a unified UEFI application, not a separate loader plus separately loaded kernel ELF.

That single-binary boot contract means the firmware-side helper modules that gather boot context and provide bootloader observability are part of the live boot boundary, not just incidental support code.

- Boot begins at `bootloader/src/main.rs::efi_main`.
- The bootloader transfers control by directly calling `theseus_kernel::kernel_entry`.

Implements / evidence:
- `bootloader/src/main.rs#efi_main`
- `bootloader/src/boot_sequence.rs#jump_to_kernel_with_handoff`
- `kernel/src/lib.rs#kernel_entry`

Related plans:
- `../plans/boot-flow.md#implemented-invariants`

Affected modules:
- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `bootloader/src/acpi.rs`
- `bootloader/src/hardware.rs`
- `bootloader/src/display.rs`
- `bootloader/src/system_info.rs`
- `bootloader/src/serial.rs`
- `bootloader/src/qemu_exit.rs`
- `bootloader/src/drivers/*`
- `shared/src/handoff.rs`
- `kernel/src/lib.rs`
- `kernel/src/handoff.rs`

## A2: Boot Services are exited before kernel entry

**REQUIRED**

`kernel_entry` is entered only after `ExitBootServices` completes and the copied handoff is marked with `boot_services_exited = 1`.

Implements / evidence:
- `bootloader/src/boot_sequence.rs#jump_to_kernel_with_handoff`
- `kernel/src/handoff.rs#validate_handoff`

Related plans:
- `../plans/boot-flow.md#implemented-invariants`

Affected modules:
- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `kernel/src/handoff.rs`
- `kernel/src/lib.rs`

## A3: Kernel image metadata is derived from the live binary, not a separately parsed on-disk image

**REQUIRED**

The bootloader computes kernel image metadata from the in-process `kernel_entry` symbol and records it in the handoff.

**RISK**

The current implementation uses a conservative fixed 16 MiB image span rather than exact section extents.

Implements / evidence:
- `bootloader/src/boot_sequence.rs#set_kernel_image_from_loaded_image`

Related plans:
- `../plans/boot-flow.md#risks`

Affected modules:
- `bootloader/src/main.rs`
- `bootloader/src/boot_sequence.rs`
- `bootloader/src/memory.rs`
- `bootloader/src/drivers/manager.rs`
- `shared/src/handoff.rs`
