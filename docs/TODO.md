
 * Validation function for handoff structure and panic if it smells fishy

 
 Chat about how to not use anything in lower half anymore:
 ´´´
 What do we have to worry about having still in lower half? The bump allocator? Page tables? GDT, IDT? Stack? What else?

Considering stack allocation and mapping
I’m thinking about allocating a high-half stack for KASLR and hardening. First task is to allocate the stack and adjust RSP. 
For page tables, I created a PML4 in temporary static pool in low memory, and CR3 points to that physical address. High-half mapping includes them. 
Then, the bump allocator uses a temp heap in the LOADER_DATA region, which is identity mapped and also mapped in high-half. It seems like addresses in long mode are treated as virtual, but identity mapping ensures everything works fine when using high-half.
Evaluating allocator migration and setup
I'm thinking about migrating the allocator to the high-half region eventually. The handoff structure is in LOADER_DATA at low memory, and I've created a virtual pointer offset, so everything reads fine through a function using RIP.
Currently, the interrupt stub addresses are in high-half, but they don’t preserve the stack and don’t use the IST/TSS. The GDT descriptor seems static in the kernel's .bss, but we use high virtual addresses. I wonder if I should reload the GDT with the high base after the jump; that might be wise. For now, TSS and IST stacks aren't set up, and I'm unsure about AP start.
Reviewing used resources and allocations
I noticed that some resources aren't being used right now. For per-CPU data, there’s nothing set. The ID map for the first 1GiB is still left, and I’ll plan for security and hardening to remove identity mapping later.
EFI memory types for LOADER_DATA remain valid after ExitBS, but I should reclaim them eventually. The framebuffer at 0x80000000 is in low memory and was only mapped low; it doesn’t have a high mapping. Finally, I need to migrate the boot stack to the kernel stack in high-half memory. I can manage CPU MSRs, so I’ll also summarize findings using bold headings in markdown for clarity.
GDT/IDT: After the jump we reload the IDT with high-half addresses (good). The tables themselves still reside in low physical memory, but that’s fine since CR3 uses physical. Optional harden: rebuild GDT/IDT bases to high-half VAs and drop the low-half IDT install.
Stack (RSP): Still a low-half stack from boot. It works (identity map remains), but we should switch to a kernel stack allocated in high-half and update RSP before enabling more features.
Page tables (CR3): Page tables live in low physical memory (by design). That’s fine; CR3 wants physical. No change needed.
Identity map (low-half): We left the 1 GiB identity map in place to keep old pointers working. Harden later by removing the identity map once everything (stack, pointers) is in high-half.
Bump allocator/temp heap: The heap is LOADER_DATA low memory, accessed via identity mapping. It works. Later, move the heap to a proper high-half region and free the temporary low region.
Handoff structure: Now accessible both ways; we compute and use both phys and high-half pointers. This is OK to keep, or later switch fully to the high-half pointer and drop the low path.
Kernel code/data: Now executing from high-half. Verified by IDT entry dumps and successful #DE handler.
Framebuffer: Mapped explicitly and also covered by the 1 GiB mirror; OK. Optional: ensure a high-half alias if you plan to access via high-half VAs only.
Misc: No TSS/IST stacks yet (handlers don’t use IST). If you add IST, allocate those stacks in high-half and update the TSS after the jump.
So what would break if we took out the identity mapping? We'd need to fix the stack, our pointers to the page tables, even if CR3 uses physical? Switch to a more permenent allocator? We also need to think about physical memory allocator.

What breaks without identity
Stack: current RSP points into low VA; remove identity and any touch faults. Fix by allocating a high-half kernel stack and switching RSP (and IST stacks) before dropping identity.
Page-table writes: your paging code derefs next-level tables via physical addresses (assuming identity). Fix get_or_create_page_table and friends to convert physical → virtual with a phys→virt mapping (e.g., va = KERNEL_VIRTUAL_BASE + pa) before deref.
Temporary heap/bump allocator: points at a low LOADER_DATA region via identity VA. Either map that region into high-half and update the heap pointers to the high-half VA, or create a proper high-half heap and migrate.
Handoff structure: any phys-based deref via identity will fault. Use the high-half pointer only after the jump (you already have both; prefer the virt one).
GDT/IDT bases: the CPU uses linear (virtual) bases for GDTR/IDTR. After the jump, ensure you reload GDT/IDT using their high-half addresses (you already reload IDT; do the same for GDT to be safe). IST stack pointers in the TSS must be high-half too.
Framebuffer/MMIO: any direct low VA must be replaced by proper mappings; access via the mapped VA (you already map the framebuffer).
Safe sequence to drop identity
1) Keep the existing high-half mirror mapping of low phys (e.g., map 0..1 GiB at KERNEL_VIRTUAL_BASE + offset).
2) Switch to a high-half kernel stack; update RSP and (if used) TSS IST entries; then reload TR if needed.
3) Reload GDT and IDT using high-half bases; confirm with dumps.
4) Update paging code to deref tables via phys→virt mapping (stop relying on identity).
5) Move the heap: either remap the temp-heap region into high-half and update pointers, or bring up a permanent high-half heap and migrate.
6) Ensure all kernel pointers (handoff, framebuffer, etc.) use high-half VAs only.
7) Clear PML4[0] (and any lower-half mappings) and flush TLB; test exceptions and basic I/O.
Could rust be using any allocations or pointers that would get messed up?

Short answer: yes, if they still reference low-half VAs.
What’s safe
Code/data/rodata: compiled to high-half VAs; fine after the jump.
Trait vtables, &'static str, type metadata: in .rodata high-half; fine.
Page tables: accessed by physical address via CR3; fine.
What can break when identity is removed
Heap objects made before the switch (Vec/Box/String/format!/alloc::vec!, etc.): their pointers target the low-half temp heap. If identity goes away and that region isn’t also mapped in high-half (or the objects aren’t migrated), derefs will fault.
The bump allocator’s state (HEAP_START/NEXT/END): currently low VAs. Must either:
map its physical range at a high-half VA and retarget the allocator pointers, or
bring up a permanent high-half heap, stop allocating from the temp heap, and migrate/free old objects before removing identity.
Any raw pointers or cached addresses into low-half (including saved stack addresses, saved return addresses, or globals you set early) will fault if identity is dropped before they’re updated/mapped.
Stack: if it’s still in low-half and identity is removed, any push/pop/fn call will fault. Switch to a high-half kernel stack first (and IST stacks if used).
GDT/IDT bases: must be reloaded with high-half bases before removing identity (you already reload IDT; do the same for GDT/TSS if needed).
Practical guidance
Freeze allocations until after you switch stacks and allocator to high-half.
Then either remap the temp heap’s physical pages into high-half and flip the allocator pointers to the high-half VA, or bring up a new high-half heap and stop using the temp heap (migrate if you have live objects).
Audit for Box/Vec/String/format! early in init; avoid those before the switch.
Remove identity only after:
RSP moved to a high-half stack (and IST stacks updated),
GDT/IDT/TSS reloaded with high-half bases,
allocator and all live heap objects are high-half reachable,
paging code doesn’t assume identity (use phys→virt mapping when touching tables).
´´´
