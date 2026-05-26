; user_test.asm — Minimal ring 3 test binary for TheseusOS syscall system.
; Assembled with: nasm -f bin user_test.asm -o user_test.bin
;
; Syscall convention (x86-64 syscall instruction, SysV-like):
;   RAX = syscall number
;   RDI = arg1, RSI = arg2, RDX = arg3, R10 = arg4, R8 = arg5, R9 = arg6
;
; Syscall numbers (see dispatch.rs):
;   0 = SYS_NULL          (no-op, returns 0)
;   1 = SYS_WRITE_SERIAL  (arg1=buf_ptr, arg2=len)
;   2 = SYS_GET_TICKS     (no args, returns tick count in RAX)

bits 64
org 0x100000000     ; User code mapped at this VA (4 GiB), above all physical memory

start:
    ; --- Test 1: SYS_NULL ---
    xor     eax, eax
    syscall
    ; RAX should be 0; we don't check (no assert infrastructure in ring 3)

    ; --- Test 2: SYS_GET_TICKS ---
    mov     eax, 2
    syscall
    ; RAX = current tick count. Save for debug.
    mov     r12, rax

    ; --- Test 3: SYS_WRITE_SERIAL with test message ---
    mov     eax, 1                  ; SYS_WRITE_SERIAL
    lea     rdi, [rel message]      ; arg1 = buffer pointer (RIP-relative)
    mov     esi, msg_len            ; arg2 = length
    syscall
    ; RAX = bytes written

    ; --- Test 4: SYS_WRITE_SERIAL with ticks message ---
    ; Convert r12 (tick count) to decimal string in buffer, then write.
    ; For simplicity, just write a fixed "ticks done" message.
    mov     eax, 1
    lea     rdi, [rel msg_done]
    mov     esi, msg_done_len
    syscall

    ; --- Spin forever ---
    ; Note: pause is fine at ring 3, but timer interrupts will PF
    ; because the user stack page is too close to the code page.
    ; This is a known limitation for the initial test — a proper
    ; user-space launcher will use separate stack regions.
.hang:
    pause
    jmp     .hang

message:
    db "[syscall-test] Hello from ring 3!", 10
msg_len equ $ - message

msg_done:
    db "[syscall-test] Syscall tests complete. Entering hlt loop.", 10
msg_done_len equ $ - msg_done
