// Small helper to switch to the kernel stack and jump to a continuation.
// This centralizes the only inline assembly that changes RSP and performs a
// non-returning jump. Callers provide the stack top virtual address and the
// continuation target as a raw u64 address.
#![allow(dead_code)]

/// Safety: caller must ensure `stack_top` is valid and aligned, and `cont` is a
/// valid, executable virtual address mapped in the current page tables.
pub unsafe fn switch_to_kernel_stack_and_jump(stack_top: u64, cont: u64) -> ! {
    core::arch::asm!(
        "mov rsp, {stack_top}",
        "jmp rax",
        stack_top = in(reg) stack_top,
        in("rax") cont,
        options(noreturn)
    );
}
