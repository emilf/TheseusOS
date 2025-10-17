# Theseus EFI GDB helpers ----------------------------------------------------
#
# Usage:
#   1. launch QEMU with -S -s so CPUs start halted
#   2. gdb -x debug.gdb
#   3. let the firmware run once (it will print `efi_main @ 0x...`)
#   4. run: theseus-load 0x3da60a10   # replace with your runtime address
#      (the command computes the relocation delta, reloads DWARF at the correct
#       runtime base, and installs a hardware breakpoint at the entry plus
#       software breakpoints at +0x200/+0x300)
#   5. reset the guest (e.g. `monitor system_reset` or restart QEMU) and rerun
#      so `efi_main` executes again and trips the breakpoints.
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

_E_PHOFF = struct.unpack_from("<Q", _elf_image, 0x20)[0]
_E_PHENTSIZE = struct.unpack_from("<H", _elf_image, 0x36)[0]
_E_PHNUM = struct.unpack_from("<H", _elf_image, 0x38)[0]

def _iter_program_headers():
    for i in range(_E_PHNUM):
        off = _E_PHOFF + i * _E_PHENTSIZE
        p_type = struct.unpack_from("<I", _elf_image, off)[0]
        p_offset = struct.unpack_from("<Q", _elf_image, off + 0x08)[0]
        p_vaddr = struct.unpack_from("<Q", _elf_image, off + 0x10)[0]
        p_filesz = struct.unpack_from("<Q", _elf_image, off + 0x20)[0]
        yield p_type, p_offset, p_vaddr, p_filesz

# Compute link-time image base.
_image_base_link = min(
    (vaddr - offset) for p_type, offset, vaddr, _ in _iter_program_headers()
    if p_type == 1  # PT_LOAD
)

# Locate the signature inside the ELF file to recover its link-time address.
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
        "failed to map THESES_DEBUG_SIGNATURE to a PT_LOAD segment"
    )

# Link-time address of efi_main (already loaded via symbol-file).
_efi_main_symbol = gdb.lookup_global_symbol('theseus_efi::efi_main')
if _efi_main_symbol is None:
    raise gdb.GdbError("Unable to resolve theseus_efi::efi_main symbol")

_efi_main_link_addr = int(_efi_main_symbol.value().cast(gdb.lookup_type("long long")))
_efi_entry_offset = _efi_main_link_addr - _image_base_link

TEXT_DELTA   = 0x1000
RDATA_DELTA  = 0x12000
DATA_DELTA   = 0x1a000
BSS_DELTA    = 0x1c000
EH_DELTA     = 0x6c000
RELOC_DELTA  = 0x6d000

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

_LL_TYPE = gdb.lookup_type("long long")

def _eval_address(expr: str):
    try:
        value = gdb.parse_and_eval(expr)
        if value.type.code == gdb.TYPE_CODE_PTR:
            value = value.cast(_LL_TYPE)
        else:
            value = value.cast(_LL_TYPE)
        return int(value)
    except gdb.error:
        return None

def _set_breakpoints(runtime_entry: int):
    _clear_auto_breakpoints()

    addresses = []
    labels = []

    gdb.execute(f"hbreak *0x{runtime_entry:x}", to_string=True)
    _AUTO_BREAK_LOCATIONS.add(f"*0x{runtime_entry:x}")
    gdb.write(f"   · Hardware breakpoint set at efi_main entry (*0x{runtime_entry:x}).\n")

    for delta, label in ((0x200, "efi_main+0x200"), (0x300, "efi_main+0x300")):
        addr = runtime_entry + delta
        loc = f"*0x{addr:x}"
        gdb.execute(f"break {loc}", to_string=True)
        _AUTO_BREAK_LOCATIONS.add(loc)
        gdb.write(f"   · Software breakpoint set at {label} ({loc}).\n")

class TheseusLoadCommand(gdb.Command):
    """Load Theseus EFI symbols at runtime with relocated base.

Usage: theseus-load <efi_main_runtime_address>
Example: theseus-load 0x3da60a10
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
        gdb.write(f"  · runtime efi_main:    0x{runtime_entry:x}\n")
        gdb.write(f"  · image base (runtime):0x{image_base_runtime:x}\n")
        gdb.write(f"  · signature (runtime): 0x{signature_runtime:x}\n")

        text_addr  = image_base_runtime + TEXT_DELTA
        rdata_addr = image_base_runtime + RDATA_DELTA
        data_addr  = image_base_runtime + DATA_DELTA
        bss_addr   = image_base_runtime + BSS_DELTA
        eh_addr    = image_base_runtime + EH_DELTA
        reloc_addr = image_base_runtime + RELOC_DELTA

        gdb.write("  · Section remap targets:\n")
        gdb.write(f"      .text   → 0x{text_addr:x}\n")
        gdb.write(f"      .rdata  → 0x{rdata_addr:x}\n")
        gdb.write(f"      .data   → 0x{data_addr:x}\n")
        gdb.write(f"      .bss    → 0x{bss_addr:x}\n")
        gdb.write(f"      .eh_fram→ 0x{eh_addr:x}\n")
        gdb.write(f"      .reloc  → 0x{reloc_addr:x}\n")

        _remove_existing_symbols()
        gdb.execute(
            f"add-symbol-file {SYMBOL_PATH} 0x{text_addr:x} "
            f"-s .rdata 0x{rdata_addr:x} "
            f"-s .data 0x{data_addr:x} "
            f"-s .bss 0x{bss_addr:x} "
            f"-s .eh_fram 0x{eh_addr:x} "
            f"-s .reloc 0x{reloc_addr:x}",
            to_string=True,
        )

        _set_breakpoints(runtime_entry)
        gdb.write("⛳ Breakpoints armed on theseus_efi::efi_main (entry [HW], +0x200, +0x300). Reset or rerun so they trigger.\n")

TheseusLoadCommand()
end
