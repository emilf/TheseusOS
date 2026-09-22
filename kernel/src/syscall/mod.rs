//! Syscall subsystem — `syscall`/`sysretq` path for ring 3 → ring 0 transitions.
//!
//! SOURCE OF TRUTH: docs/plans/syscall-system.md
//!
//! This module ties together:
//!   - `entry.rs`: assembly trampoline (`syscall_entry`, IA32_LSTAR target)
//!   - `percpu.rs`: per-CPU data for GS-relative access
//!   - `dispatch.rs`: syscall table and handlers
//!   - `usermode.rs`: ring 3 launch helpers (iretq into user code)

pub mod dispatch;
pub mod entry;
pub mod percpu;
pub mod usermode;

use crate::{log_debug, log_info};

/// MSR indices for syscall configuration.
mod msrs {
    pub const IA32_STAR: u32 = 0xC000_0081;
    pub const IA32_LSTAR: u32 = 0xC000_0082;
    pub const IA32_FMASK: u32 = 0xC000_0084;
    pub const IA32_GS_BASE: u32 = 0xC000_0101;
    pub const IA32_KERNEL_GS_BASE: u32 = 0xC000_0102;
}

/// Write a 64-bit value to an MSR.
///
/// # Safety
/// Caller must ensure `msr` is a valid, writable MSR index and `value` is
/// architecturally correct for that register.
pub unsafe fn wrmsr(msr: u32, value: u64) {
    let lo = value as u32;
    let hi = (value >> 32) as u32;
    core::arch::asm!(
        "wrmsr",
        in("ecx") msr,
        in("eax") lo,
        in("edx") hi,
        options(nomem, nostack, preserves_flags)
    );
}

/// Read a 64-bit value from an MSR.
///
/// # Safety
/// Caller must ensure `msr` is a valid MSR index.
pub unsafe fn rdmsr(msr: u32) -> u64 {
    let lo: u32;
    let hi: u32;
    core::arch::asm!(
        "rdmsr",
        in("ecx") msr,
        out("eax") lo,
        out("edx") hi,
        options(nomem, nostack, preserves_flags)
    );
    ((hi as u64) << 32) | (lo as u64)
}

/// Initialize the syscall subsystem:
///   1. Program STAR/LSTAR/SFMASK MSRs (already partially done in `cpu::setup_msrs`).
///   2. Set up per-CPU data (BSP) and kernel syscall stack.
///   3. Program KERNEL_GS_BASE for `swapgs`.
///
/// Must be called AFTER:
///   - GDT has user segments (user_data_sel at 0x18, user_code_sel at 0x20)
///   - `setup_msrs()` has enabled EFER.SCE
///   - Memory subsystem has mapped the syscall stack
///
/// # Safety
/// Architectural MSR writes; modifies global CPU state.
pub unsafe fn syscall_init() {
    log_debug!("syscall: initializing subsystem");

    // --- Per-CPU data and syscall stack ---
    let stack_top = percpu::syscall_stack_top();
    let percpu_addr = percpu::bsp_percpu_addr();

    {
        let cpu = percpu::bsp_percpu_mut();
        cpu.kernel_stack_top = stack_top;
        cpu.user_rsp = 0;
        cpu.cpu_index = 0;
    }

    log_debug!(
        "syscall: BSP per-CPU at {:#x}, syscall stack top {:#x}",
        percpu_addr,
        stack_top
    );

    // --- Program KERNEL_GS_BASE (IA32_KERNEL_GS_BASE) ---
    // After `swapgs`, GS will point here.
    unsafe {
        wrmsr(msrs::IA32_KERNEL_GS_BASE, percpu_addr);
        // IA32_GS_BASE stays 0 — no user TLS yet.
        wrmsr(msrs::IA32_GS_BASE, 0);
    }

    // --- STAR MSR ---
    // Encodes segment selectors for syscall (kernel) and sysretq (user).
    //
    // STAR[47:32] = kernel CS (0x0008)
    //   → syscall loads CS = 0x0008, SS = 0x0008 + 8 = 0x0010
    // STAR[63:48] = 0x0010
    //   → sysretq loads CS = 0x0010 + 16 = 0x0020 (RPL|=3 → 0x0023)
    //   → sysretq loads SS = 0x0010 + 8  = 0x0018 (RPL|=3 → 0x001B)
    let star: u64 = ((crate::gdt::USER_DS as u64 - 8) << 48)
        | ((crate::gdt::KERNEL_CS as u64) << 32);
    unsafe {
        wrmsr(msrs::IA32_STAR, star);
    }
    log_debug!("syscall: STAR = {:#018x}", star);

    // --- LSTAR MSR ---
    // Address of `syscall_entry` symbol from entry.rs global_asm!
    extern "C" {
        fn syscall_entry();
    }
    let lstar = syscall_entry as *const () as u64;
    unsafe {
        wrmsr(msrs::IA32_LSTAR, lstar);
    }
    log_debug!("syscall: LSTAR = {:#018x}", lstar);

    // --- SFMASK MSR ---
    // Bits to CLEAR from RFLAGS on syscall entry:
    //   Bit 8  = TF (Trap Flag) — prevent single-step in kernel
    //   Bit 9  = IF (Interrupt Flag) — disable interrupts in syscall path
    //   Bit 18 = AC (Alignment Check) — disable alignment faults in kernel
    let sfmask: u64 = (1 << 8) | (1 << 9) | (1 << 18);
    unsafe {
        wrmsr(msrs::IA32_FMASK, sfmask);
    }
    log_debug!("syscall: SFMASK = {:#018x}", sfmask);

    // Verification reads (log_debug to avoid logspam; enable log_info when debugging MSR setup)
    unsafe {
        let star_read = rdmsr(msrs::IA32_STAR);
        let lstar_read = rdmsr(msrs::IA32_LSTAR);
        let fmask_read = rdmsr(msrs::IA32_FMASK);
        let kgs = rdmsr(msrs::IA32_KERNEL_GS_BASE);
        let gs = rdmsr(msrs::IA32_GS_BASE);
        let efer = x86_64::registers::model_specific::Efer::read();
        log_debug!(
            "syscall: verify STAR={:#x} LSTAR={:#x} FMASK={:#x}",
            star_read,
            lstar_read,
            fmask_read
        );
        log_debug!(
            "syscall: KGS_BASE={:#x} GS_BASE={:#x} EFER.SCE={}",
            kgs,
            gs,
            efer.contains(x86_64::registers::model_specific::EferFlags::SYSTEM_CALL_EXTENSIONS)
        );
    }

    log_info!("syscall: subsystem initialized");
}
