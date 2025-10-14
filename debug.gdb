# GDB script for debugging theseus_efi with automatic image-base discovery
set architecture i386:x86-64
set demangle-style rust
set breakpoint pending on

target remote localhost:1234

# Allow firmware to run briefly so the EFI image is resident, then interrupt.
continue&
python
import time
time.sleep(0.5)
end
interrupt

# Locate the loaded PE/COFF image in guest memory and capture its base address.
python
import gdb

def locate_image_base(start=0x30000000, end=0x40000000, step=0x1000):
    inferior = gdb.selected_inferior()
    for addr in range(start, end, step):
        try:
            chunk = inferior.read_memory(addr, step)
        except gdb.error:
            continue
        data = bytes(chunk)
        idx = data.find(b'MZ')
        if idx == -1:
            continue
        candidate = addr + idx
        try:
            e_lfanew = int.from_bytes(bytes(inferior.read_memory(candidate + 0x3C, 4)), 'little')
            pe_sig = bytes(inferior.read_memory(candidate + e_lfanew, 4))
        except gdb.error:
            continue
        if pe_sig != b'PE\x00\x00':
            continue
        try:
            subsystem = int.from_bytes(bytes(inferior.read_memory(candidate + e_lfanew + 0x5C, 2)), 'little')
        except gdb.error:
            continue
        if subsystem != 0xA:  # EFI application
            continue
        gdb.write(f"Located Theseus EFI image at 0x{candidate:x}\n")
        gdb.execute(f"set $image_base = 0x{candidate:x}")
        return True
    return False

if not locate_image_base():
    gdb.write("Failed to locate Theseus EFI image; symbols not loaded.\n")
    gdb.execute("set $image_base = 0")
end

python
if int(gdb.parse_and_eval("$image_base")) == 0:
    raise gdb.GdbError("Cannot proceed without locating Theseus EFI image base")
end

add-symbol-file build/BOOTX64.SYM $image_base+0x1000 \
    -s .rdata $image_base+0x12000 \
    -s .data  $image_base+0x1a000 \
    -s .bss   $image_base+0x1c000 \
    -s .eh_fram $image_base+0x6c000 \
    -s .reloc $image_base+0x6d000

# Set breakpoints on key locations (resolved after relocation).
hbreak 'theseus_efi::efi_main'
hbreak *'theseus_efi::efi_main'+200
hbreak *'theseus_efi::efi_main'+300

# Print the Handoff struct when we hit the halt loop.
commands
  print/x *((struct Handoff*) $rdi)
  continue
end

continue
