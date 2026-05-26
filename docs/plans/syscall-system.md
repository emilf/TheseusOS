# TheseusOS Syscall System Design

**Status:** Design (pre-implementation)
**Depends on:** A1 (x86-interrupt ABI / Microsoft x64 ABI), A2 (GDT/TSS), A3 (APIC interrupts)
**Blocks:** User-mode processes, scheduler, IPC

---

## 1. Overview

Ring 3 → ring 0 transition using the `syscall`/`sysretq` pair. Not `int 0x80`
(wrong for long mode). Not `sysenter`/`sysexit` (32-bit compat only).

The central design problem is the **ABI mismatch**: TheseusOS compiles as a
UEFI PE binary (`extern "C"` = Microsoft x64 ABI: RCX/RDX/R8/R9), but the
x86-64 `syscall` instruction uses the SysV register convention (RAX syscall
number, args in RDI/RSI/RDX/R10/R8/R9). The assembly trampoline must bridge
these.

### Design Principles

- **Thin assembly, thick Rust.** Assembly only for the parts the CPU demands
  (swapgs, stack switch, register save/restore, ABI translation). All policy
  and dispatch in Rust.
- **Match the existing interrupt pattern.** Per-vector stubs → `irq_dispatch_common(vector: u8)` in the
  IRQ registry. Syscalls follow the same shape: assembly entry →
  `syscall_dispatch(number, args) -> i64` in Rust.
- **No dynamic dispatch.** Static table of `SyscallFn` function pointers
  indexed by syscall number. Bounds check, then call. No hash maps, no
  allocator on the hot path.
- **SMP-ready from day one.** Per-CPU data via GS base MSR (not global
  variables). Kernel stack pointer stored in per-CPU data, not hardcoded.

---

## 2. GDT Layout (Modifying `kernel/src/gdt.rs`)

### Current State

```
Offset  Selector  Descriptor
0x00    0x00      Null
0x08    0x08      Kernel Code  (ring 0)  ← Descriptor::kernel_code_segment()
0x10    0x10      Kernel Data  (ring 0)  ← Descriptor::kernel_data_segment()
0x18    0x18      TSS low  (16 bytes, occupies 0x18 AND 0x20)
0x20    0x20      TSS high (second half of 16-byte TSS descriptor)
```

**⚠️ Common mistake:** The TSS descriptor slots at 0x18 and 0x20 are NOT
user segments — they are the two halves of one 16-byte TSS descriptor (long
mode system segments are twice the width of code/data segments). Loading
selector 0x18 or 0x20 as a data/code segment will #GP.

**Missing:** ring 3 code and data segments. User segments must be inserted
**before** the TSS so the TSS moves up to 0x28/0x30. The STAR MSR requires a
specific layout: kernel CS/SS at STAR[47:32], user CS/SS derived from
STAR[63:48]+16 and STAR[63:48]+8 respectively.

### Required Layout

```
Offset  Selector  Descriptor
0x00    0x00      Null
0x08    0x08      Kernel Code  (ring 0, L=1)  ← KERNEL_CS
0x10    0x10      Kernel Data  (ring 0)        ← KERNEL_SS (syscall loads STAR[47:32]+8)
0x18    0x18      User Data    (ring 3)        ← USER_SS (sysretq loads STAR[63:48]+8)
0x20    0x20      User Code    (ring 3, L=1)   ← USER_CS (sysretq loads STAR[63:48]+16)
0x28    0x28      TSS (16 bytes, occupies 2 GDT slots: 0x28 and 0x30)
```

### STAR MSR Programming

```
IA32_STAR (0xC0000081):
  [47:32] = 0x0008  → kernel CS selector; kernel SS = 0x0008 + 8 = 0x0010
  [63:48] = 0x0010  → sysretq CS = 0x0010 + 16 = 0x0020 (ring 3, L=1)
                       sysretq SS = 0x0010 + 8 = 0x0018  (ring 3)

IA32_LSTAR (0xC0000082):
  → address of syscall_entry (the assembly trampoline, must be canonical)

IA32_SFMASK (0xC0000084):
  → bitmask of RFLAGS bits to CLEAR on syscall entry
  → Set: IF (bit 9), TF (bit 8), DF (bit 10), AC (bit 18)
  → Value: 0x0000000000044700
```

**Note on user data being 0x18, not 0x28:** Some references put user data at
0x20 and user code at 0x18 with STAR[63:48] = 0x0018. Both work if the STAR
fields and GDT are consistent. The layout above (data at 0x18, code at 0x20)
is what Linux uses and what the `x86_64` crate's `Descriptor::user_*` helpers
produce. Verify by checking what selector values `Descriptor::user_data_segment()`
and `Descriptor::user_code_segment()` return when added in order.

### Code Changes in `gdt.rs`

Add to `build_gdt_state()`:

```rust
// After kernel_data, before TSS:
let user_data_sel = gdt.add_entry(Descriptor::user_data_segment());
let user_code_sel = gdt.add_entry(Descriptor::user_code_segment());
// Then TSS as before

// Store selectors in GdtState:
struct GdtState {
    gdt: GlobalDescriptorTable,
    code_sel: SegmentSelector,
    data_sel: SegmentSelector,
    user_code_sel: SegmentSelector,   // NEW
    user_data_sel: SegmentSelector,   // NEW
    tss_sel: SegmentSelector,
}
```

Expose selectors publicly for STAR MSR setup:

```rust
pub const KERNEL_CS: u16 = 0x08;
pub const KERNEL_SS: u16 = 0x10;
pub const USER_DS: u16 = 0x18;
pub const USER_CS: u16 = 0x20;
```

---

## 3. Per-CPU Data & GS Base

### Why

`swapgs` exchanges `IA32_GS_BASE` and `IA32_KERNEL_GS_BASE`. On syscall entry
we need the kernel GS base to point at per-CPU state (kernel stack pointer,
CPU ID, etc.). The user GS base points at thread-local storage (TLS) — out of
scope for now, but the swap must be safe even with no user TLS.

### Per-CPU Structure

```rust
/// Per-CPU data, pointed to by IA32_KERNEL_GS_BASE (swapped to GS on syscall/interrupt entry).
#[repr(C)]
pub struct PerCpuData {
    /// Top of the kernel stack for this CPU.
    pub kernel_stack_top: u64,
    /// User RSP saved on syscall entry (stored here before switching to kernel stack).
    pub user_rsp: u64,
    /// CPU index (0 = BSP).
    pub cpu_index: u32,
    /// Padding for alignment.
    _pad: u32,
}
```

### Bootstrap (BSP only for now)

```rust
// In syscall_init(), after GDT is set up:
static BSP_PER_CPU: PerCpuData = PerCpuData {
    kernel_stack_top: 0, // filled at init time
    user_rsp: 0,
    cpu_index: 0,
    _pad: 0,
};

// Set IA32_KERNEL_GS_BASE to &BSP_PER_CPU
// IA32_GS_BASE stays 0 (no user TLS yet)
unsafe {
    Msr::new(0xC0000102).write(&BSP_PER_CPU as *const _ as u64); // KERNEL_GS_BASE
    Msr::new(0xC0000101).write(0);                                // GS_BASE (user)
}
```

### Kernel Stack for Syscalls

The BSP already has a 64 KiB `KERNEL_STACK` in `environment.rs`. The syscall
path needs its **own** stack to avoid corrupting the interrupt/main kernel
stack. Options:

1. **Share the kernel stack** — simple but dangerous if syscall is interrupted.
2. **Dedicated syscall stack** — safer; allocate a new stack in `.bss.stack`
   or via the stack allocator.

**Decision:** Dedicated stack. Add `SYSCALL_STACK` (32 KiB) in `gdt.rs` or
a new `syscall.rs` module. Set `BSP_PER_CPU.kernel_stack_top` to its top.

```rust
#[link_section = ".bss.stack"]
static mut SYSCALL_STACK: [u8; 32 * 1024] = [0; 32 * 1024];
```

> **Note:** `.bss.stack` is verified to work — the linker script's `.bss`
> output section has `*(.bss .bss.*)` which collects all `.bss.*` subsections.
> This is the same pattern used by the IST stacks and the main kernel stack.

This stack must be mapped and included in `ist_stack_ranges()` or a similar
function so `environment.rs` maps it during bring-up.

---

## 4. Assembly Syscall Entry (`syscall_entry`)

This is the heart of the system. Runs at ring 0 after `syscall` instruction.

### CPU State on Entry (what `syscall` does automatically)

- `RIP` ← `IA32_LSTAR` (our `syscall_entry`)
- `CS` ← `IA32_STAR[47:32]` (kernel CS 0x08)
- `SS` ← `IA32_STAR[47:32] + 8` (kernel SS 0x10)
- `RFLAGS` ← `RFLAGS & ~IA32_SFMASK` (IF, TF, DF, AC cleared)
- `RCX` ← saved user RIP
- `R11` ← saved user RFLAGS
- `RSP` ← **UNCHANGED** (still user stack!)

### Full Assembly Listing

```asm
.global syscall_entry
syscall_entry:
    // --- Phase 1: swapgs + save user state ---
    swapgs                              // GS_BASE ↔ KERNEL_GS_BASE
                                        // Now GS points at PerCpuData

    // Save user RSP in per-CPU data
    mov     gs:[8],  rsp                // PerCpuData.user_rsp = user RSP
                                        // (offset 8 = after kernel_stack_top)

    // --- Phase 2: switch to kernel stack ---
    mov     rsp, gs:[0]                 // RSP = PerCpuData.kernel_stack_top

    // --- Phase 3: build syscall frame ---
    // Push everything we need to restore on sysretq.
    // Frame layout (top = RSP after pushes):
    //   [RSP+0]   R15
    //   [RSP+8]   R14
    //   [RSP+16]  R13
    //   [RSP+24]  R12
    //   [RSP+32]  R11 (saved RFLAGS from syscall)
    //   [RSP+40]  RBP
    //   [RSP+48]  RBX
    //   [RSP+56]  RCX (saved user RIP from syscall)
    //   [RSP+64]  R10 (raw, before ABI translation)
    //   [RSP+72]  R9
    //   [RSP+80]  R8
    //   [RSP+88]  RDX
    //   [RSP+96]  RSI
    //   [RSP+104] RDI
    //   [RSP+112] RAX (syscall number)

    push    r15
    push    r14
    push    r13
    push    r12
    push    r11         // saved RFLAGS
    push    rbp
    push    rbx
    push    rcx         // saved user RIP
    push    r10
    push    r9
    push    r8
    push    rdx
    push    rsi
    push    rdi
    push    rax         // syscall number

    // --- Phase 4: ABI translation (SysV → Microsoft x64) ---
    // Syscall convention:  RAX=number, RDI=a1, RSI=a2, RDX=a3, R10=a4, R8=a5, R9=a6
    // Microsoft x64:      RCX=a1,   RDX=a2, R8=a3,  R9=a4
    //
    // We pass (number, args_struct_ptr) to Rust for clean extensibility.
    // But for hot-path efficiency, pass number in RCX and a pointer to the
    // args frame in RDX.

    // Build SyscallArgs on stack (reuse the saved registers).
    // Pass: RCX = syscall number (RAX), RDX = pointer to saved registers
    mov     rcx, rax         // arg1: syscall number
    lea     rdx, [rsp]       // arg2: pointer to saved register frame
    // R8, R9 available for future use (e.g., per-CPU pointer)

    // Ensure 16-byte stack alignment before call (RSP should be 16-aligned
    // after 14 pushes of 8 bytes = 112 bytes; 112 mod 16 = 0 ✓ if entry
    // RSP was 16-aligned, which the user code should guarantee)
    // The `call` pushes 8 more bytes (return addr), so callee sees
    // RSP ≡ 8 mod 16 and adjusts in prologue. This is standard.

    // --- Phase 5: call Rust dispatcher ---
    call    syscall_dispatch      // extern "C" fn(number: u64, frame: *const SyscallFrame) -> i64

    // --- Phase 6: restore and return ---
    // RAX now contains the syscall return value.
    // Move it below the frame so we can restore all saved regs.
    mov     [rsp + 112], rax  // overwrite saved RAX with return value

    pop     rax         // return value (originally syscall number slot)
    pop     rdi
    pop     rsi
    pop     rdx
    pop     r8
    pop     r9
    pop     r10
    pop     rcx         // restore saved user RIP → RCX (for sysretq)
    pop     rbx
    pop     rbp
    pop     r11         // restore saved RFLAGS → R11 (for sysretq)
    pop     r12
    pop     r13
    pop     r14
    pop     r15

    // Restore user RSP from per-CPU data
    mov     rsp, gs:[8]       // PerCpuData.user_rsp

    // swapgs back to user GS base
    swapgs

    // Return to ring 3
    sysretq
```

### Rust-Side SyscallFrame

```rust
/// Register frame saved by syscall_entry assembly.
/// Layout must match the push order in the asm above (top of struct = top of stack).
#[repr(C)]
pub struct SyscallFrame {
    pub rax: u64,    // syscall number on entry, replaced with return value
    pub rdi: u64,
    pub rsi: u64,
    pub rdx: u64,
    pub r8: u64,
    pub r9: u64,
    pub r10: u64,
    pub rcx: u64,    // saved user RIP (from syscall instruction)
    pub rbx: u64,
    pub rbp: u64,
    pub r11: u64,    // saved user RFLAGS (from syscall instruction)
    pub r12: u64,
    pub r13: u64,
    pub r14: u64,
    pub r15: u64,
}
```

The Rust dispatcher extracts **SysV-convention arguments** from the frame:

```rust
impl SyscallFrame {
    /// Syscall arg 1 (SysV: RDI)
    pub fn arg1(&self) -> u64 { self.rdi }
    /// Syscall arg 2 (SysV: RSI)
    pub fn arg2(&self) -> u64 { self.rsi }
    /// Syscall arg 3 (SysV: RDX)
    pub fn arg3(&self) -> u64 { self.rdx }
    /// Syscall arg 4 (SysV: R10)
    pub fn arg4(&self) -> u64 { self.r10 }
    /// Syscall arg 5 (SysV: R8)
    pub fn arg5(&self) -> u64 { self.r8 }
    /// Syscall arg 6 (SysV: R9)
    pub fn arg6(&self) -> u64 { self.r9 }
}
```

---

## 5. Rust Syscall Dispatcher (`kernel/src/syscall/mod.rs`)

### Syscall Numbers

```rust
/// Syscall ABI constants.
/// Syscall number in RAX. Arguments in RDI, RSI, RDX, R10, R8, R9 (SysV convention
/// at the user ←→ assembly boundary). The assembly trampoline translates to
/// Microsoft x64 ABI before calling this function.
pub mod numbers {
    pub const SYS_NULL: u64 = 0;           // no-op, returns 0 (warmup/test)
    pub const SYS_WRITE_SERIAL: u64 = 1;   // write bytes to debug serial port
    pub const SYS_GET_TICKS: u64 = 2;      // get timer tick count
    // Future:
    // pub const SYS_READ: u64 = 3;
    // pub const SYS_WRITE: u64 = 4;
    // pub const SYS_EXIT: u64 = 5;
    // pub const SYS_YIELD: u64 = 6;
    // pub const SYS_MMAP: u64 = 7;
}
```

### SyscallTable

```rust
/// Maximum syscall number + 1. Table is sized to this.
const MAX_SYSCALL: usize = 16;

/// Syscall handler function signature.
/// Takes a reference to the saved register frame, returns i64 (negative = error).
///
/// **ABI note:** These function pointers are called Rust→Rust (from
/// `syscall_dispatch`, not from assembly), so they use the platform's native
/// calling convention — no ABI annotation needed. The only assembly→Rust
/// boundary is `syscall_dispatch(number, frame)` which is explicitly
/// `extern "C"` (Microsoft x64 ABI on this target). This mirrors how
/// `irq_dispatch_common` is `extern "C"` but the handler function pointers
/// in `IRQ_REGISTRY` are plain `fn()`.
type SyscallFn = fn(frame: &SyscallFrame) -> i64;

/// Static syscall table. None entries = unimplemented syscall.
static SYSCALL_TABLE: [Option<SyscallFn>; MAX_SYSCALL] = {
    let mut table = [None; MAX_SYSCALL];
    table[0] = Some(sys_null);
    table[1] = Some(sys_write_serial);
    table[2] = Some(sys_get_ticks);
    table
};
```

### Dispatcher

```rust
/// Rust entry point called from syscall_entry assembly.
///
/// This is compiled with `extern "C"` which means Microsoft x64 ABI:
///   RCX = syscall number
///   RDX = pointer to SyscallFrame on kernel stack
///
/// # Safety
/// Called only from the syscall_entry trampoline with a valid frame pointer.
#[no_mangle]
pub extern "C" fn syscall_dispatch(number: u64, frame: *const SyscallFrame) -> i64 {
    let frame = unsafe { &*frame };

    let idx = number as usize;
    if idx >= MAX_SYSCALL {
        return -1; // ENOSYS
    }

    match SYSCALL_TABLE[idx] {
        Some(handler) => handler(frame),
        None => -1, // ENOSYS
    }
}
```

### Test Syscall Implementations

```rust
/// SYS_NULL: No-op, returns 0. Used to measure syscall overhead.
fn sys_null(_frame: &SyscallFrame) -> i64 {
    0
}

/// SYS_WRITE_SERIAL: Write bytes to COM1 debug port.
///
/// Args (SysV convention at user boundary):
///   RDI (arg1) = pointer to buffer (user virtual address)
///   RSI (arg2) = length in bytes
///
/// Returns: number of bytes written, or negative error.
///
/// SAFETY: User provides a pointer. We must validate it's in user space
/// before dereferencing. For this initial implementation with kernel-embedded
/// user code, we trust the pointer but add bounds checking later.
fn sys_write_serial(frame: &SyscallFrame) -> i64 {
    let buf_ptr = frame.arg1() as *const u8;
    let len = frame.arg2() as usize;

    // TODO: Validate user pointer (must be < USER_SPACE_END, not in kernel range)
    // For now: trust the embedded test binary

    let slice = unsafe { core::slice::from_raw_parts(buf_ptr, len) };

    // Write to COM1 via the same path as serial_debug
    for &byte in slice {
        unsafe {
            crate::interrupts::debug::out_char_0xe9(byte as u8);
            // Or use the driver manager:
            // crate::drivers::manager::driver_manager().lock().write_class(...)
        }
    }

    len as i64
}

/// SYS_GET_TICKS: Return current timer tick count.
///
/// No arguments.
/// Returns: u32 tick count (zero-extended to i64).
fn sys_get_ticks(_frame: &SyscallFrame) -> i64 {
    crate::interrupts::timer_tick_count() as i64
}
```

---

## 6. ABI Translation — Detailed Walkthrough

This is the most subtle part. Let me trace a call:

### User Code (SysV convention — standard for syscall)

```asm
; User wants to call SYS_WRITE_SERIAL(1, buf, len)
mov     rax, 1          ; syscall number
mov     rdi, buf        ; arg1: buffer pointer
mov     rsi, len        ; arg2: length
syscall                 ; → kernel
```

### After `syscall` instruction (CPU state)

```
RCX = user RIP (auto-saved by syscall)
R11 = user RFLAGS (auto-saved by syscall)
RAX = 1 (syscall number, untouched)
RDI = buf
RSI = len
RDX, R10, R8, R9 = whatever user had (unused for 2-arg call)
RSP = user stack (unchanged!)
```

### After assembly trampoline translates for Rust call

```
RCX = 1          (arg1 for Microsoft ABI: syscall number from RAX)
RDX = frame_ptr  (arg2 for Microsoft ABI: pointer to saved SyscallFrame)
```

### Inside `syscall_dispatch` (Rust, Microsoft ABI)

```rust
fn syscall_dispatch(number: u64 /* RCX=1 */, frame: *const SyscallFrame /* RDX */) {
    // frame.rdi == buf  (user arg1, from SysV RDI)
    // frame.rsi == len  (user arg2, from SysV RSI)
}
```

The translation is clean because:
- We don't try to translate individual args register-by-register
- Instead, the assembly saves ALL user registers into `SyscallFrame`
- Rust reads args from the frame using the SysV field names (`.rdi`, `.rsi`, etc.)
- The only Microsoft ABI call is `syscall_dispatch(number, frame_ptr)` with 2 args
- This scales to any number of syscall args (up to 6) with no trampoline changes

---

## 7. MSR Setup — Boot Sequence Integration

### Where: Modify `kernel/src/cpu.rs::setup_msrs()`

Currently only enables `EFER.SCE`. Extend to program STAR/LSTAR/SFMASK.

**This must run after GDT setup** (needs final selector values) and **after**
the syscall entry symbol address is known (linker-resolved).

```rust
pub unsafe fn setup_msrs() {
    // 1. EFER.SCE (already exists)
    {
        use x86_64::registers::model_specific::{Efer, EferFlags};
        let mut e = Efer::read();
        e.insert(EferFlags::SYSTEM_CALL_EXTENSIONS);
        Efer::write(e);
    }

    // 2. STAR — segment selectors for syscall/sysret
    {
        use crate::gdt::{KERNEL_CS, USER_DS};
        // STAR[47:32] = kernel CS (sysret SS = this + 8)
        // STAR[63:48] = user DS - 8... wait, the encoding:
        //   syscall: CS = STAR[47:32],     SS = STAR[47:32] + 8
        //   sysretq: CS = STAR[63:48] + 16, SS = STAR[63:48] + 8
        //
        // With our GDT: KERNEL_CS=0x08, USER_DS=0x18, USER_CS=0x20
        //   STAR[63:48] must be USER_DS - 8 = 0x10... but that's KERNEL_SS.
        //   Actually: sysretq loads CS from STAR[63:48]+16 with RPL=3.
        //   For USER_CS=0x20: STAR[63:48] = 0x20 - 16 = 0x10. ✓
        //   For USER_SS=0x18: STAR[63:48]+8 = 0x10+8 = 0x18. ✓
        //
        // STAR[63:48] = 0x0010  (same as KERNEL_SS selector value)
        // This works because sysretq adds 16 for CS (→ 0x20 = user code)
        // and 8 for SS (→ 0x18 = user data). Both get RPL |= 3.

        let star: u64 = ((USER_DS as u64 - 16) << 48) | ((KERNEL_CS as u64) << 32);
        // star = (0x0010 << 48) | (0x0008 << 32)
        Msr::new(0xC0000081).write(star);
    }

    // 3. LSTAR — syscall entry point
    {
        extern "C" { fn syscall_entry(); }
        Msr::new(0xC0000082).write(syscall_entry as u64);
    }

    // 4. SFMASK — RFLAGS bits to clear on syscall entry
    {
        // Clear: IF (bit 9), TF (bit 8), DF (bit 10), AC (bit 18)
        let sfmask: u64 = (1 << 8) | (1 << 9) | (1 << 10) | (1 << 18);
        Msr::new(0xC0000084).write(sfmask);
    }

    // 5. GS base MSRs for swapgs
    {
        // IA32_GS_BASE (user) — 0 for now (no user TLS)
        Msr::new(0xC0000101).write(0);
        // IA32_KERNEL_GS_BASE — set to per-CPU data by syscall_init()
        // (done separately after per-CPU data is allocated)
    }
}
```

### MSR Helper (if not using raw_cpuid's x86_64 crate)

```rust
/// Simple MSR read/write helper.
struct Msr(u32);

impl Msr {
    const fn new(index: u32) -> Self { Self(index) }

    unsafe fn read(&self) -> u64 {
        let (lo, hi): (u32, u32);
        core::arch::asm!("rdmsr", in("ecx") self.0, out("eax") lo, out("edx") hi,
                         options(nomem, nostack, preserves_flags));
        ((hi as u64) << 32) | (lo as u64)
    }

    unsafe fn write(&self, value: u64) {
        let lo = value as u32;
        let hi = (value >> 32) as u32;
        core::arch::asm!("wrmsr", in("ecx") self.0, in("eax") lo, in("edx") hi,
                         options(nomem, nostack, preserves_flags));
    }
}
```

(Alternatively use `x86_64::registers::model_specific::Msr` if available.)

### Boot Sequence Order (in `environment.rs`)

```
... existing bring-up ...
GDT/TSS setup (with new user segments)     ← already step 2
...
CPU features detect (SMEP/SMAP/etc.)
setup_msrs()                               ← already called, now extended
...
syscall_init()                             ← NEW: after setup_msrs()
  → allocates per-CPU data
  → sets KERNEL_GS_BASE
  → records syscall stack in per-CPU data
  → logs "syscall: STAR/LSTAR/SFMASK programmed"
...
continue to idle loop / monitor
```

---

## 8. Ring 3 Test Code

### Approach: Embedded Flat Binary

The simplest way to get ring 3 code running without a binary loader:

1. Write a small assembly program as `user_test.S`.
2. Assemble to a flat binary.
3. Include in the kernel image via `include_bytes!()` or a linker section.
4. At runtime: map it into user-accessible virtual address space, set up a
   ring 3 stack, then `iretq` (or `sysretq` from a dummy frame) to jump to it.

### user_test.S

```asm
; Simple ring 3 test: call SYS_GET_TICKS, then SYS_WRITE_SERIAL, then loop.

; Syscall numbers
SYS_NULL        equ 0
SYS_WRITE_SERIAL equ 1
SYS_GET_TICKS   equ 2

bits 64
org 0x400000    ; User-space load address

start:
    ; --- Test 1: SYS_NULL ---
    xor     rax, rax            ; SYS_NULL
    syscall
    ; RAX should be 0

    ; --- Test 2: SYS_GET_TICKS ---
    mov     rax, SYS_GET_TICKS
    syscall
    ; RAX = tick count

    ; --- Test 3: SYS_WRITE_SERIAL ---
    mov     rax, SYS_WRITE_SERIAL
    lea     rdi, [rel message]  ; arg1: buffer pointer
    mov     rsi, msg_len        ; arg2: length
    syscall

    ; --- Loop forever ---
.hang:
    hlt
    jmp     .hang

message:
    db "[user] Hello from ring 3 via syscall!", 10
msg_len equ $ - message
```

### User Space Memory Layout (Initial)

```
0x400000 - 0x401000   User code (1 page, R+X, user-accessible)
0x3FF000 - 0x400000   User stack (1 page, R+W, user-accessible)
                       Stack grows down from 0x400000
```

For the initial test, these are **mapped in the kernel's page tables** with
user-accessible bits (PTE_USER). There is no separate address space yet — the
kernel is still mapped but SMEP/SMAP prevent accidental execution/access from
ring 3.

### Jumping to Ring 3

```rust
/// Transition from ring 0 to ring 3 using iretq.
///
/// Sets up a fake interrupt frame pointing at user code, loads user segments,
/// and iretqs into ring 3.
///
/// # Safety
/// Assumes user pages are mapped and user stack is set up.
pub unsafe fn jump_to_usermode(entry: u64, stack_top: u64) -> ! {
    use crate::gdt::{USER_CS, USER_DS};

    core::arch::asm!(
        // Clear all segment registers for clean user state
        "xor ax, ax",
        "mov ds, ax",
        "mov es, ax",
        "mov fs, ax",
        // GS is handled by swapgs in syscall path, leave as 0

        // Build iretq frame: SS, RSP, RFLAGS, CS, RIP
        "push {user_ds}",       // SS  (ring 3)
        "push {user_rsp}",      // RSP (user stack top)
        "push 0x202",           // RFLAGS (IF=1, bit 1 always set)
        "push {user_cs}",       // CS  (ring 3, L=1)
        "push {entry}",         // RIP (user entry point)

        "iretq",

        user_ds = const USER_DS as u64 | 3,   // RPL = 3
        user_cs = const USER_CS as u64 | 3,   // RPL = 3
        user_rsp = in(reg) stack_top,
        entry = in(reg) entry,
        options(noreturn)
    );
}
```

### Integration in Boot Flow

After `syscall_init()` completes, before entering the idle loop:

```rust
// In environment.rs, after syscall_init():
if crate::config::RUN_USERMODE_TEST {
    log_info!("Launching ring 3 test...");
    unsafe {
        crate::syscall::run_usermode_test();
    }
    // Note: if the user test loops forever, we never reach idle.
    // The test should either sysretq back or we kill it via timer.
}
```

---

## 9. File Layout

```
kernel/src/
  syscall/
    mod.rs              — public API: syscall_init(), run_usermode_test()
    entry.rs            — global_asm! for syscall_entry trampoline
    dispatch.rs         — SYSCALL_TABLE, syscall_dispatch(), SyscallFrame
    handlers.rs         — sys_null, sys_write_serial, sys_get_ticks
    percpu.rs           — PerCpuData struct, BSP per-CPU allocation
    usermode.rs         — jump_to_usermode(), user test binary embedding
  gdt.rs                — (modified) add user segments, expose selectors
  cpu.rs                — (modified) extend setup_msrs() for STAR/LSTAR/SFMASK
  environment.rs         — (modified) call syscall_init() in boot sequence
```

---

## 10. Testing & Verification

### Phase 1: MSR + GDT Validation (no ring 3 yet)

1. After `setup_msrs()`, read back STAR, LSTAR, SFMASK and log them.
2. Verify GDT has user segments at the correct offsets.
3. Verify `syscall_entry` symbol address is canonical and matches LSTAR.

### Phase 2: Syscall from Ring 0 (sanity check)

Before jumping to ring 3, test the dispatch table from ring 0:
- Call `syscall_dispatch(0, &dummy_frame)` directly — should return 0.
- Call `syscall_dispatch(2, &dummy_frame)` — should return tick count.
- Call `syscall_dispatch(999, &dummy_frame)` — should return -1.

This validates the Rust side without touching assembly or privilege levels.

### Phase 3: Ring 3 Test

1. Map user code page (R+X, user-accessible) at 0x400000.
2. Map user stack page (R+W, user-accessible) at 0x3FF000.
3. Copy embedded binary to 0x400000.
4. `jump_to_usermode(0x400000, 0x400000)`.
5. Test executes:
   - SYS_NULL → verify RAX=0 (via serial debug or breakpoint)
   - SYS_GET_TICKS → RAX should be > 0 if timer is running
   - SYS_WRITE_SERIAL → message appears in QEMU serial output
6. Test enters hlt loop (observable: CPU usage drops).

### Phase 4: Error Cases

- Invalid syscall number → returns -1
- Bad pointer in SYS_WRITE_SERIAL → should fault (#PF from user mode)
- Verify SMAP: kernel can't accidentally dereference user pointers without
  explicit `stac`/`clac` or user-memory helpers

---

## 11. Security Considerations (Future)

These are **not** blocking for the initial implementation but should be
designed for:

1. **Pointer validation:** All user-provided pointers must be checked against
   USER_SPACE_END before dereference. Use a `copy_from_user()` helper that
   handles page faults gracefully.

2. **SMAP enforcement:** When reading/writing user buffers in syscall handlers,
   use `stac`/`clac` or the `copy_from_user` helper that does this atomically.

3. **SMEP:** Already enabled. User code pages must not be marked supervisor-executable.

4. **Stack alignment:** The assembly trampoline must ensure 16-byte alignment
   before `call syscall_dispatch`. User code is responsible for alignment
   before `syscall`.

5. **Syscall frame on kernel stack:** The frame is 120 bytes (15 registers × 8).
   With a 32 KiB syscall stack this gives ~270 nested calls of headroom
   (should be 1 deep in practice since syscalls don't nest).

---

## 12. Open Questions

1. **Should SYSCALL_STACK be part of the stack allocator?** For BSP-only it's
   fine as a static. For SMP, each AP needs its own — stack allocator is the
   right place.

2. **GS_BASE for user TLS:** Currently 0. When threads exist, GS_BASE needs
   to point at a per-thread TCB. The `swapgs` in the trampoline handles the
   kernel side; the user side needs `wrfsbase`/`wrgsbase` (if FSGSBASE is
   enabled in CR4 — it is, per `cpu_features.rs`) or an MSR write via
   `arch_prctl`-style syscall.

3. **Should the syscall table be const?** It's all function pointers with no
   interior mutability. Could be `const` instead of `static`. Doesn't matter
   for correctness.

4. **What about `int 0x80` compat?** No. We're 64-bit only, no legacy 32-bit
   compat. If someone runs `int 0x80`, it'll hit the IRQ stub for vector
   0x80 and warn about unhandled IRQ. Acceptable.

5. **Should `syscall_dispatch` be allowed to block?** Not yet (no scheduler).
   When a scheduler exists, blocking syscalls save the SyscallFrame into the
   thread struct and context-switch. The trampoline doesn't change.

---

## Appendix: Complete MSR Reference

| MSR                  | Index      | Purpose                               | Value                                |
|----------------------|------------|---------------------------------------|--------------------------------------|
| IA32_EFER           | 0xC0000080 | System Call Extensions (bit 0)        | Set SCE bit                          |
| IA32_STAR           | 0xC0000081 | Syscall/sysret segment selectors      | [47:32]=0x0008, [63:48]=0x0010      |
| IA32_LSTAR          | 0xC0000082 | Syscall entry point (RIP)             | `syscall_entry` symbol address       |
| IA32_CSTAR          | 0xC0000083 | Compat mode syscall entry (unused)    | 0                                    |
| IA32_SFMASK         | 0xC0000084 | RFLAGS mask on syscall entry          | 0x0000000000044700                   |
| IA32_GS_BASE        | 0xC0000101 | User GS base (swapped out on entry)   | 0 (no user TLS yet)                  |
| IA32_KERNEL_GS_BASE | 0xC0000102 | Kernel GS base (swapped in on entry)  | `&BSP_PER_CPU`                       |
