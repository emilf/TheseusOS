//! Syscall entry trampoline — assembly stub for `syscall` instruction target.
//!
//! SOURCE OF TRUTH: docs/plans/syscall-system.md
//!
//! This module contains the `syscall_entry` symbol programmed into IA32_LSTAR.
//! The CPU jumps here on every `syscall` instruction from ring 3.
//!
//! At entry, the CPU has already:
//!   - Loaded CS/SS from STAR[47:32] (kernel segments)
//!   - Saved user RIP → RCX, user RFLAGS → R11
//!   - Cleared RFLAGS bits per SFMASK (IF, TF, AC)
//!   - Left RSP pointing at the USER stack (unchanged!)
//!
//! This trampoline's job:
//!   1. `swapgs` to access per-CPU data (kernel GS base)
//!   2. Save user RSP into per-CPU data
//!   3. Load kernel RSP from per-CPU data
//!   4. Push all callee-saved + caller-saved registers into a SyscallFrame
//!   5. Call syscall_dispatch(number, frame)
//!   6. Restore registers from the (possibly modified) frame
//!   7. Restore user RSP
//!   8. `swapgs` back and `sysretq`

use core::arch::global_asm;

global_asm!(
    r#"
.global syscall_entry
syscall_entry:
    // --- Phase 1: swapgs + save user RSP ---
    swapgs
    // GS now points at PerCpuData (kernel GS base).
    // PerCpuData layout (repr(C)):
    //   [0]  = kernel_stack_top  (u64)
    //   [8]  = user_rsp          (u64)
    //   [16] = cpu_index         (u32)

    // Save user RSP → PerCpuData.user_rsp
    mov     gs:[8],  rsp

    // --- Phase 2: switch to kernel stack ---
    mov     rsp, gs:[0]     // RSP = PerCpuData.kernel_stack_top

    // --- Phase 3: build SyscallFrame ---
    // Push in reverse order so SyscallFrame.rax is at lowest address (top of stack).
    // SyscallFrame layout (top → bottom):
    //   +0    rax (syscall number on entry, return value on exit)
    //   +8    rdi
    //   +16   rsi
    //   +24   rdx
    //   +32   r10
    //   +40   r9
    //   +48   r8
    //   +56   rcx (saved user RIP from syscall)
    //   +64   rbx
    //   +72   rbp
    //   +80   r11 (saved user RFLAGS from syscall)
    //   +88   r12
    //   +96   r13
    //   +104  r14
    //   +112  r15

    push    r15
    push    r14
    push    r13
    push    r12
    push    r11     // saved RFLAGS (from syscall)
    push    rbp
    push    rbx
    push    rcx     // saved user RIP (from syscall)
    push    r8
    push    r9
    push    r10
    push    rdx
    push    rsi
    push    rdi
    push    rax     // syscall number

    // --- Phase 4: ABI bridge (SysV user args → Microsoft x64 for Rust call) ---
    // User convention at syscall boundary: RAX=num, RDI=a1, RSI=a2, RDX=a3,
    //   R10=a4, R8=a5, R9=a6  (standard x86-64 syscall ABI, SysV-like)
    // Microsoft x64 Rust call:  RCX=arg1, RDX=arg2, R8=arg3, R9=arg4
    //
    // We pass:
    //   RCX = syscall number (from RAX before we pushed it)
    //   RDX = pointer to SyscallFrame on kernel stack (RSP after pushes)
    //
    // Note: RCX was already clobbered by `syscall` (CPU saved RIP there).
    // We use RAX (still holding the syscall number from the frame top).

    mov     rcx, rax        // arg1: syscall number
    lea     rdx, [rsp]      // arg2: pointer to SyscallFrame

    // --- Phase 5: call Rust dispatcher ---
    // extern "C" fn syscall_dispatch(number: u64, frame: *const SyscallFrame) -> i64
    // Microsoft ABI: arg1=RCX, arg2=RDX. RAX holds return value after call.
    //
    // Stack alignment: we pushed 15 × 8 = 120 bytes (0x78). If RSP was
    // 16-aligned at entry (user's responsibility for syscall), RSP is now
    // 8 mod 16. The `call` pushes 8 more bytes (return address), so callee
    // sees RSP ≡ 0 mod 16. Rust expects RSP ≡ 0 mod 16 after `call`. ✓
    call    syscall_dispatch

    // --- Phase 6: restore and sysretq ---
    // RAX = return value from syscall_dispatch. Write it back into the frame
    // at offset +0 (rax slot), so the pop below loads it into RAX.
    mov     [rsp], rax

    pop     rax     // return value
    pop     rdi
    pop     rsi
    pop     rdx
    pop     r10
    pop     r9
    pop     r8
    pop     rcx     // restore saved user RIP (needed by sysretq in RCX)
    pop     rbx
    pop     rbp
    pop     r11     // restore saved RFLAGS (needed by sysretq in R11)
    pop     r12
    pop     r13
    pop     r14
    pop     r15

    // --- Phase 7: restore user RSP ---
    mov     rsp, gs:[8]     // PerCpuData.user_rsp

    // --- Phase 8: swapgs back to user GS, return to ring 3 ---
    swapgs
    sysretq
"#
);
