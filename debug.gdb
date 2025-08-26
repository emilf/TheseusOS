# GDB script for debugging hobbyos_efi
set architecture i386:x86-64
target remote localhost:1234

# Set a breakpoint at the entry point
break efi_main

# Set a breakpoint before exiting boot services
break *efi_main+200

# Set a breakpoint at the halt loop
break *efi_main+300

# Print the Handoff struct when we hit the halt
commands
  print/x *((struct Handoff*) $rdi)
  continue
end

# Start debugging
continue
