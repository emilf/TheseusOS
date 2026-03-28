# TheseusOS Roadmap

**Goal:** A POSIX-compatible enough kernel to compile and run core Unix tools (coreutils, busybox, a shell) without crippling them in the process.

This document is a living roadmap. It reflects what's been built, what comes next, and the long arc toward a usable POSIX surface. Phases are roughly sequential but some work can happen in parallel.

---

## Phase 0 — Foundation ✅ (Done)

Everything here is merged and working. This is what the vibe coding experiment has built so far.

**Boot & UEFI**
- [x] UEFI bootloader (custom, no GRUB/Limine)
- [x] Kernel ELF loaded from ESP
- [x] ExitBootServices + handoff struct to kernel
- [x] Higher-half kernel mapping (0xFFFF800000000000+)
- [x] GDT with kernel code/data segments

**Memory**
- [x] UEFI memory map ingestion
- [x] Physical frame allocator
- [x] Kernel heap (linked_list_allocator)
- [x] DMA allocator + DMA pool
- [x] Temporary mapping window

**CPU & Interrupts**
- [x] IDT + NMI handlers
- [x] xAPIC initialized
- [x] APIC timer interrupts firing (BSP)
- [x] ACPI + MADT parsing

**Drivers**
- [x] Framebuffer (UEFI GOP)
- [x] Serial (debugcon + UART)
- [x] PCI enumeration
- [x] USB xHCI driver (full: rings, MSI/MSI-X, HID boot protocol)
- [x] USB keyboard (HID → ASCII key events)

**Tooling & DX**
- [x] Debug monitor (serial shell with commands: cpu, memory, pci, usb, tables, io, devices)
- [x] Logging subsystem with verbosity filter
- [x] theseus-qemu runner (profiles, relays, timeout, build-before-run)
- [x] QEMU relay sockets (serial, debugcon, HMP, QMP)
- [x] GDB mailbox protocol + gdb-auto.py automation
- [x] tmux live loop for interactive debugging
- [x] Test framework (bare metal + kernel tests)

**Documentation**
- [x] Axioms (boot, memory, arch-x86_64, debug)
- [x] Plans (boot-flow, memory, interrupts-and-platform, drivers-and-io, observability, x2apic-prep)
- [x] docs/index.md, docs/map.md, AGENTS.md

---

## Phase 1 — CPU & Platform Hardening

Before we can do multi-process work, the CPU foundations need to be solid.

- [x] **APIC timer calibration** — measure ticks/ms against HPET or PIT; needed for real preemption (implemented in `interrupts/calibration.rs`)
- [ ] **x2APIC support** — plan exists (`x2apic-prep.md`), just needs implementation
- [ ] **TSS + IST stacks** — separate stacks for NMI/DF/MCE; required for safe exception handling
- [x] **CPUID feature abstraction** — centralized feature detection (SSE, AVX, TSC-Deadline, x2APIC, etc.) (implemented in `cpu_features.rs`)
- [ ] **SMP bring-up** — wake APs via INIT/SIPI, per-CPU GDT/IDT/TSS, IPI infrastructure

**Status:** Partially complete. See `docs/plans/cpu-platform-hardening.md` for detailed status.

---

## Phase 2 — Scheduling & Kernel Threads

The kernel needs to be able to walk and chew gum at the same time.

- [ ] **Kernel threads** — switchable execution contexts with their own stacks
- [ ] **Context switching** — save/restore registers (GP + optional SSE), stack pointer swap
- [ ] **Preemptive scheduler** — timer-driven, round-robin to start; pluggable later
- [ ] **Per-CPU runqueues** — one queue per AP; work-stealing later
- [ ] **Mutex / spinlock / wait queues** — synchronization primitives that interact with the scheduler
- [ ] **Idle tasks** — per-CPU idle threads (HLT loop)

---

## Phase 3 — User-Mode & Address Spaces

This is where the kernel becomes an OS rather than a fancy bootloader.

- [ ] **Per-process page tables** — each process gets its own CR3; kernel mapped in upper half of all
- [ ] **User address space layout** — conventional ELF layout: text/data/BSS/heap/stack below ~0x7FFFFFFFFFFF
- [ ] **Ring 3 entry** — SYSCALL/SYSRET setup (IA32_STAR, IA32_LSTAR, IA32_FMASK)
- [ ] **User-mode stack** — set up at exec time; stack guard page
- [ ] **SMEP/SMAP** — enable supervisor mode execution/access protection (CR4)

---

## Phase 4 — System Calls (POSIX Surface — Core)

The syscall interface is the contract. Start minimal, add as tools demand it.

**Process lifecycle**
- [ ] `exit` / `exit_group`
- [ ] `fork` (or `clone` as the primitive)
- [ ] `exec` / `execve` — load a new ELF into the current address space
- [ ] `wait` / `waitpid`
- [ ] `getpid` / `getppid`

**I/O fundamentals**
- [ ] `read` / `write`
- [ ] `open` / `close`
- [ ] `dup` / `dup2`
- [ ] File descriptor table per-process (stdin/stdout/stderr wired at init)

**Memory**
- [ ] `mmap` (anonymous first — needed by malloc)
- [ ] `munmap`
- [ ] `brk` (optional if mmap anonymous is solid)

**Signals (minimal)**
- [ ] `kill`
- [ ] `signal` / `sigaction`
- [ ] `SIGKILL`, `SIGTERM`, `SIGSEGV`, `SIGCHLD`

---

## Phase 5 — VFS & Filesystems

No OS is useful without a filesystem. Start in-memory, add real storage later.

- [ ] **VFS layer** — inode/dentry abstraction; pluggable backends
- [ ] **tmpfs / ramfs** — memory-backed FS; used for initrd and /tmp
- [ ] **devfs / /dev stubs** — `/dev/null`, `/dev/zero`, `/dev/tty`, `/dev/console`
- [ ] **procfs stubs** — `/proc/self`, `/proc/self/maps`, `/proc/self/exe` (enough for musl)
- [ ] **FAT32 driver** — re-use bootloader knowledge; gives access to ESP / disk images
- [ ] **Pipes** — `pipe()` syscall; anonymous pipe between processes
- [ ] **ext2** — read-only first; gives access to a standard Linux disk image format

---

## Phase 6 — ELF Loader & Init

- [ ] **ELF64 static loader** — parse PT_LOAD segments, map into user address space, jump to entry
- [ ] **Program interpreter field** — detect and reject dynamic ELFs with a clear error (until dynamic linking is ready)
- [ ] **Auxiliary vector (auxv)** — pass AT_PHDR, AT_ENTRY, AT_PAGESZ etc. to new process
- [ ] **Init process (PID 1)** — statically linked, minimal; brings up /dev, mounts tmpfs, execs shell
- [ ] **Dynamic linker** (stretch) — load interpreter, resolve shared libs; needed for non-musl-static binaries

---

## Phase 7 — libc Port

Port a libc so we can compile tools against it.

- [ ] **musl libc** — preferred: clean, static-linking-first, small syscall surface, actively ported to new kernels
- [ ] **Syscall compatibility pass** — audit musl's syscall usage; implement or stub everything it needs
- [ ] **Toolchain** — cross-compiler targeting `x86_64-theseus` (custom target JSON, sysroot)
- [ ] **newlib** (alternative) — simpler but less complete; good fallback if musl is painful

---

## Phase 8 — Core Userspace Tools

This is the "close enough to POSIX" milestone.

- [ ] **Busybox** — single binary with sh, ls, cat, grep, echo, cp, mv, mkdir, etc. Compile against musl.
- [ ] **dash** — minimal POSIX shell; lighter than bash, easier to port
- [ ] **coreutils** (stretch) — GNU or uutils-coreutils (Rust); richer than busybox but more syscall surface
- [ ] **Self-hosting build** — can we build a simple C program inside TheseusOS itself?

---

## Phase 9 — Network Stack

Optional for the core POSIX goal but needed for anything actually useful.

- [ ] **virtio-net driver** — QEMU virtio NIC; simplest possible NIC to implement
- [ ] **e1000 driver** — alternative; well-documented, real hardware target
- [ ] **TCP/IP stack** — port smoltcp (Rust, no_std-friendly) or lwIP
- [ ] **BSD socket API** — `socket`, `bind`, `connect`, `listen`, `accept`, `send`, `recv`
- [ ] **DNS stub** — enough for `getaddrinfo` to work

---

## Phase 10 — Polish & Deeper POSIX Compliance

The long tail. Most tools will work after Phase 8; this phase makes them work *well*.

- [ ] **pthreads** — POSIX threads (`clone` with CLONE_THREAD, per-thread TLS via `arch_prctl`)
- [ ] **TLS (Thread-Local Storage)** — `arch_prctl(ARCH_SET_FS)`, FS.base MSR
- [ ] **mmap file-backed** — map files directly into address space
- [ ] **Proper signals** — signal masks, `sigprocmask`, `SA_RESTART`, sigaltstack
- [ ] **Terminal emulation** — proper TTY/PTY (`/dev/tty`, `tcgetattr`/`tcsetattr`); needed by interactive shells
- [ ] **`/proc` expansion** — `/proc/cpuinfo`, `/proc/meminfo`, `/proc/net/...`
- [ ] **`/sys` stubs** — minimal sysfs enough for tools that probe it
- [ ] **User/group IDs** — UID/GID, `getuid`, `setuid` etc. (even if always root for now)

---

## Open Questions / Risks

- **Fork vs spawn:** True `fork()` (copy-on-write) is expensive to implement correctly. Many modern minimal OSes implement `posix_spawn` as the primitive and fake `fork+exec`. Worth deciding early.
- **Dynamic linking:** Static musl gets you far. Dynamic linking is a lot of work (dynamic linker, GOT/PLT, shared lib loading). Probably defer until after Phase 8.
- **SMP complexity:** Multi-core makes everything harder (TLB shootdowns, per-CPU state, lock contention). Can defer AP bring-up until after the scheduler is solid on BSP.
- **Storage:** Need a real disk image (QEMU `-drive`) for anything beyond ramfs. FAT32 from the ESP is a natural first target.
- **Capability / security model:** Even a simple UID=0-only model needs to be decided early or it'll be painful to retrofit.

---

## Rough Timeline Sense

Not commits to dates — just a feeling for scale:

| Phase | Effort |
|-------|--------|
| 1 — CPU hardening | Small-medium (x2APIC plan exists, SMP is the hard part) |
| 2 — Scheduling | Medium (context switch + preemption is fiddly) |
| 3 — User-mode | Medium (mostly CPU plumbing) |
| 4 — Syscalls | Medium-large (lots of ground to cover, but well-documented) |
| 5 — VFS | Large (abstraction design matters a lot here) |
| 6 — ELF loader | Small-medium (static ELF is actually not that bad) |
| 7 — libc port | Medium (musl is cooperative; toolchain setup is the annoying part) |
| 8 — Core tools | Small if libc works (mostly build system wrangling) |
| 9 — Network | Large |
| 10 — Polish | Ongoing forever |
