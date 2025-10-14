## UEFI Runtime Services Integration

This kernel now keeps UEFI runtime services alive after `ExitBootServices()` by
transitioning the firmware into the higher-half virtual address space and
replaying the memory map with `SetVirtualAddressMap()`. The flow looks like this:

1. **Map runtime descriptors** – every memory descriptor marked with
   `EFI_MEMORY_RUNTIME` is remapped into the `PHYS_OFFSET` window so the firmware
   code/data remains accessible once paging is active.
2. **Switch the allocator** – the boot-time allocator shim is moved onto the
   permanent heap so we can safely create temporary buffers while finalising the
   runtime map.
3. **Call `SetVirtualAddressMap()`** – after the allocator switch we make the
   official runtime call, handing firmware the virtual addresses it should use.
4. **Smoke test the RTC** – with runtime services definitely working through
   virtual addresses, the kernel queries the firmware RTC via
   `uefi::runtime::get_time()` and prints the timestamp with `kernel_write_line`.
   A successful read confirms that the pageable runtime regions are reachable.

The timestamp shows up in the boot log as:

```
[rt] Firmware time 2024-07-30 13:05:47.123456789 UTC+02:00 daylight=0x00
```

If the call fails we emit the firmware status code instead, which makes it easy
to cross-reference with the UEFI specification or firmware documentation while
debugging.
