# Theseus EFI GDB helpers ----------------------------------------------------
#
# Usage:
#   1. launch QEMU with -S -s so CPUs start halted
#   2. gdb -x debug.gdb
#   3. let the firmware run once (it will print `efi_main @ 0x...`)
#   4. run: theseus-load 0x<runtime_efi_main_address>
#      (the command computes the relocation delta, reloads DWARF at the correct
#       runtime base, and installs a hardware breakpoint at the entry)
#   5. reset the guest (monitor system_reset or restart QEMU) so efi_main fires
#
# NOTE: Section deltas are computed dynamically from BOOTX64.SYM at startup.
#       No hardcoded offsets — safe across rebuilds.
#

set pagination off
set architecture i386:x86-64
set demangle-style rust
set breakpoint pending on
set confirm off

# Make DWARF types and symbols available up-front.
symbol-file build/BOOTX64.SYM

python
import gdb
import os
import struct

SYMBOL_PATH = os.path.abspath("build/BOOTX64.SYM")
SIGNATURE = b"THESEUSDBGBASE!\x00"

def _read_elf(path):
    with open(path, "rb") as f:
        return f.read()

_elf_image = _read_elf(SYMBOL_PATH)

if _elf_image[:4] != b"\x7fELF":
    raise gdb.GdbError(f"{SYMBOL_PATH} is not an ELF file")

# --------------------------------------------------------------------------
# Parse ELF header
# --------------------------------------------------------------------------
_E_PHOFF     = struct.unpack_from("<Q", _elf_image, 0x20)[0]
_E_SHOFF     = struct.unpack_from("<Q", _elf_image, 0x28)[0]
_E_PHENTSIZE = struct.unpack_from("<H", _elf_image, 0x36)[0]
_E_PHNUM     = struct.unpack_from("<H", _elf_image, 0x38)[0]
_E_SHENTSIZE = struct.unpack_from("<H", _elf_image, 0x3a)[0]
_E_SHNUM     = struct.unpack_from("<H", _elf_image, 0x3c)[0]
_E_SHSTRNDX  = struct.unpack_from("<H", _elf_image, 0x3e)[0]

# --------------------------------------------------------------------------
# PT_LOAD segments → derive link-time image base
# --------------------------------------------------------------------------
def _iter_program_headers():
    for i in range(_E_PHNUM):
        off = _E_PHOFF + i * _E_PHENTSIZE
        p_type   = struct.unpack_from("<I", _elf_image, off)[0]
        p_offset = struct.unpack_from("<Q", _elf_image, off + 0x08)[0]
        p_vaddr  = struct.unpack_from("<Q", _elf_image, off + 0x10)[0]
        p_filesz = struct.unpack_from("<Q", _elf_image, off + 0x20)[0]
        yield p_type, p_offset, p_vaddr, p_filesz

# Use only PT_LOAD segments with p_offset != 0 to derive the base.
# Segments where vaddr - offset is inconsistent (e.g. .bss gap segments)
# are excluded by taking the minimum across segments whose file-offset is
# aligned with their vaddr (i.e. the non-gap ones at offset > 0).
_image_base_link = min(
    (vaddr - offset)
    for p_type, offset, vaddr, _ in _iter_program_headers()
    if p_type == 1 and offset != 0
)

# --------------------------------------------------------------------------
# Section headers → compute per-section deltas from image base
# (computed fresh from the actual ELF — no hardcoded constants)
# --------------------------------------------------------------------------
_shstr_entry_off = _E_SHOFF + _E_SHSTRNDX * _E_SHENTSIZE
_shstr_data_off  = struct.unpack_from("<Q", _elf_image, _shstr_entry_off + 0x18)[0]

_section_deltas = {}   # name -> delta from image base (link-time)

for i in range(_E_SHNUM):
    off        = _E_SHOFF + i * _E_SHENTSIZE
    sh_name_i  = struct.unpack_from("<I", _elf_image, off)[0]
    sh_addr    = struct.unpack_from("<Q", _elf_image, off + 0x10)[0]
    if sh_addr == 0:
        continue
    name_start = _shstr_data_off + sh_name_i
    name_end   = _elf_image.index(b"\x00", name_start)
    name       = _elf_image[name_start:name_end].decode()
    _section_deltas[name] = sh_addr - _image_base_link

# Dump computed deltas at startup so you can verify they match reality
gdb.write("Theseus: section deltas computed from BOOTX64.SYM\n")
gdb.write(f"  image_base_link = 0x{_image_base_link:x}\n")
for sname, delta in sorted(_section_deltas.items()):
    if not sname.startswith(".debug"):
        gdb.write(f"    {sname:20s} Δ = 0x{delta:x}\n")

# --------------------------------------------------------------------------
# Locate THESEUS_DEBUG_SIGNATURE in ELF to get its link-time address
# --------------------------------------------------------------------------
_signature_offset = _elf_image.find(SIGNATURE)
if _signature_offset == -1:
    raise gdb.GdbError(
        "THESEUS_DEBUG_SIGNATURE was not found in build/BOOTX64.SYM"
    )

_signature_link_addr = None
for p_type, offset, vaddr, filesz in _iter_program_headers():
    if p_type != 1:
        continue
    if offset <= _signature_offset < offset + filesz:
        _signature_link_addr = vaddr + (_signature_offset - offset)
        break

if _signature_link_addr is None:
    raise gdb.GdbError(
        "failed to map THESEUS_DEBUG_SIGNATURE to a PT_LOAD segment"
    )

# --------------------------------------------------------------------------
# Resolve efi_main link-time address
# --------------------------------------------------------------------------
_efi_main_symbol = gdb.lookup_global_symbol("theseus_efi::efi_main")
if _efi_main_symbol is None:
    raise gdb.GdbError("Unable to resolve theseus_efi::efi_main symbol")

_efi_main_link_addr = int(
    _efi_main_symbol.value().cast(gdb.lookup_type("long long"))
)
_efi_entry_offset = _efi_main_link_addr - _image_base_link

gdb.write(f"  efi_main link addr  = 0x{_efi_main_link_addr:x}  (offset 0x{_efi_entry_offset:x} from base)\n")

# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def _remove_existing_symbols():
    try:
        gdb.execute(f"remove-symbol-file {SYMBOL_PATH}", to_string=True)
    except gdb.error:
        pass

_AUTO_BREAK_LOCATIONS = set()

def _clear_auto_breakpoints():
    global _AUTO_BREAK_LOCATIONS
    if not _AUTO_BREAK_LOCATIONS:
        return
    for bp in gdb.breakpoints() or []:
        if bp.location in _AUTO_BREAK_LOCATIONS:
            bp.delete()
    _AUTO_BREAK_LOCATIONS.clear()

def _set_breakpoints(runtime_entry: int):
    _clear_auto_breakpoints()

    loc = f"*0x{runtime_entry:x}"
    gdb.execute(f"hbreak {loc}", to_string=True)
    _AUTO_BREAK_LOCATIONS.add(loc)
    gdb.write(f"   · Hardware breakpoint at efi_main entry ({loc}).\n")

    for delta, label in ((0x200, "efi_main+0x200"), (0x300, "efi_main+0x300")):
        addr = runtime_entry + delta
        aloc = f"*0x{addr:x}"
        gdb.execute(f"break {aloc}", to_string=True)
        _AUTO_BREAK_LOCATIONS.add(aloc)
        gdb.write(f"   · Software breakpoint at {label} ({aloc}).\n")

# --------------------------------------------------------------------------
# theseus-load command
# --------------------------------------------------------------------------
class TheseusLoadCommand(gdb.Command):
    """Load Theseus EFI symbols relocated to the runtime image base.

Usage: theseus-load <efi_main_runtime_address>
Example: theseus-load 0x3da60a10

The runtime address is printed by the bootloader on the debug port:
  efi_main @ 0x<addr>

This command:
  1. Computes the runtime image base from the runtime efi_main address.
  2. Applies per-section relocation (computed from BOOTX64.SYM, not hardcoded).
  3. Reloads DWARF symbols via add-symbol-file with all section addresses.
  4. Sets a hardware breakpoint at efi_main entry + two software sentinels.
"""

    def __init__(self):
        super().__init__("theseus-load", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)
        if len(argv) != 1:
            raise gdb.GdbError("usage: theseus-load <efi_main_runtime_address>")

        try:
            runtime_entry = int(argv[0], 0)
        except ValueError as exc:
            raise gdb.GdbError(f"invalid address '{argv[0]}': {exc}") from exc

        image_base_runtime = runtime_entry - _efi_entry_offset
        signature_runtime  = image_base_runtime + (_signature_link_addr - _image_base_link)

        gdb.write("Theseus symbol loader:\n")
        gdb.write(f"  · runtime efi_main:     0x{runtime_entry:x}\n")
        gdb.write(f"  · image base (runtime): 0x{image_base_runtime:x}\n")
        gdb.write(f"  · signature (runtime):  0x{signature_runtime:x}\n")

        # Build add-symbol-file command with all non-debug sections
        # that have a non-zero address (skip debug sections — GDB handles
        # those automatically from the ELF's DWARF).
        SKIP_PREFIXES = (".debug_", ".shstrtab")
        sections = {
            name: image_base_runtime + delta
            for name, delta in _section_deltas.items()
            if not any(name.startswith(p) for p in SKIP_PREFIXES)
        }

        if ".text" not in sections:
            raise gdb.GdbError("No .text section found in symbol file")

        text_addr = sections.pop(".text")

        gdb.write("  · Section runtime addresses:\n")
        gdb.write(f"      .text   = 0x{text_addr:x}\n")
        extra_args = []
        for sname, saddr in sorted(sections.items()):
            gdb.write(f"      {sname:20s} = 0x{saddr:x}\n")
            extra_args.append(f"-s {sname} 0x{saddr:x}")

        cmd = (
            f"add-symbol-file {SYMBOL_PATH} 0x{text_addr:x} "
            + " ".join(extra_args)
        )

        _remove_existing_symbols()
        gdb.execute(cmd, to_string=True)
        gdb.write("  · Symbols loaded.\n")

        _set_breakpoints(runtime_entry)
        gdb.write("⛳ Ready. Reset or rerun the guest so efi_main fires.\n")

TheseusLoadCommand()
end
