//! Module: stack
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/memory.md#A1:-The-kernel-executes-from-a-higher-half-virtual-base
//! - docs/axioms/arch-x86_64.md#A2:-GDT/TSS-setup-provides-dedicated-IST-stacks-for-critical-faults
//!
//! INVARIANTS:
//! - This module centralizes the non-returning stack-switch helper used during the higher-half transition.
//! - Callers provide the stack top and continuation target; this module does not validate broader boot sequencing.
//!
//! SAFETY:
//! - Changing `rsp` and jumping to a continuation is one of the sharpest operations in the kernel; the supplied stack and continuation must already be valid in the active page tables.
//! - A bad call here is architectural corruption, not a recoverable local bug.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//!
//! Small helper for the non-returning stack switch used during bring-up.
#![allow(dead_code)]

/// Switch to `stack_top` and jump to `cont`.
///
/// The caller must ensure the stack and continuation are already valid in the
/// active page tables.
pub unsafe fn switch_to_kernel_stack_and_jump(stack_top: u64, cont: u64) -> ! {
    core::arch::asm!(
        "mov rsp, {stack_top}",
        "jmp rax",
        stack_top = in(reg) stack_top,
        in("rax") cont,
        options(noreturn)
    );
}
