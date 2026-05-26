//! Syscall dispatch table and handlers.
//!
//! Called from entry.rs trampoline via `syscall_dispatch(number, frame)`.

use super::percpu;
use crate::log_info;

/// Saved register frame pushed by `syscall_entry` assembly.
///
/// Layout MUST match the push order in entry.rs (top of struct = lowest address = RSP).
#[repr(C)]
pub struct SyscallFrame {
    /// RAX: syscall number on entry; return value on exit.
    pub rax: u64,
    pub rdi: u64,
    pub rsi: u64,
    pub rdx: u64,
    /// R10 holds the 4th syscall argument (replaces RCX, which CPU uses for saved RIP).
    pub r10: u64,
    pub r9: u64,
    pub r8: u64,
    /// Saved user RIP (from RCX after `syscall` instruction).
    pub rcx: u64,
    pub rbx: u64,
    pub rbp: u64,
    /// Saved user RFLAGS (from R11 after `syscall` instruction).
    pub r11: u64,
    pub r12: u64,
    pub r13: u64,
    pub r14: u64,
    pub r15: u64,
}

impl SyscallFrame {
    /// Syscall arg1 (user SysV ABI: RDI).
    pub fn arg1(&self) -> u64 {
        self.rdi
    }
    /// Syscall arg2 (user SysV ABI: RSI).
    pub fn arg2(&self) -> u64 {
        self.rsi
    }
    /// Syscall arg3 (user SysV ABI: RDX).
    pub fn arg3(&self) -> u64 {
        self.rdx
    }
    /// Syscall arg4 (user SysV ABI: R10 — not RCX, which holds saved RIP).
    pub fn arg4(&self) -> u64 {
        self.r10
    }
    /// Syscall arg5 (user SysV ABI: R8).
    pub fn arg5(&self) -> u64 {
        self.r8
    }
    /// Syscall arg6 (user SysV ABI: R9).
    pub fn arg6(&self) -> u64 {
        self.r9
    }
}

/// Syscall numbers (RAX on entry).
pub mod numbers {
    /// No-op; returns 0. Useful for overhead benchmarks.
    pub const SYS_NULL: u64 = 0;
    /// Write bytes to the debug serial port (port 0xE9).
    ///   arg1 (RDI) = buffer pointer
    ///   arg2 (RSI) = length
    pub const SYS_WRITE_SERIAL: u64 = 1;
    /// Return the current LAPIC timer tick count. No arguments.
    pub const SYS_GET_TICKS: u64 = 2;
}

/// Maximum syscall number + 1 (table size).
const MAX_SYSCALL: usize = 16;

/// Syscall handler signature.
type SyscallFn = fn(frame: &mut SyscallFrame) -> i64;

/// Static syscall dispatch table.
/// Index = syscall number. `None` → returns -ENOSYS.
static SYSCALL_TABLE: [Option<SyscallFn>; MAX_SYSCALL] = {
    let mut table: [Option<SyscallFn>; MAX_SYSCALL] = [None; MAX_SYSCALL];
    table[numbers::SYS_NULL as usize] = Some(sys_null);
    table[numbers::SYS_WRITE_SERIAL as usize] = Some(sys_write_serial);
    table[numbers::SYS_GET_TICKS as usize] = Some(sys_get_ticks);
    table
};

/// Rust dispatcher called from the syscall_entry trampoline.
///
/// This is the single assembly → Rust ABI boundary:
///   - Microsoft x64 ABI: arg1=RCX (syscall number), arg2=RDX (&SyscallFrame)
///   - Returns i64 in RAX (copied back into the frame by the trampoline)
///
/// # Safety
/// Called only from entry.rs assembly with a valid, mapped kernel stack frame.
#[no_mangle]
pub extern "C" fn syscall_dispatch(number: u64, frame: *mut SyscallFrame) -> i64 {
    let frame = unsafe { &mut *frame };

    let handler = match SYSCALL_TABLE.get(number as usize) {
        Some(Some(h)) => *h,
        _ => {
            log_info!("syscall: unknown number {}", number);
            return -1; // ENOSYS
        }
    };

    handler(frame)
}

// ============================================================================
// Handlers
// ============================================================================

/// SYS_NULL: returns 0. Used as a baseline latency measurement.
fn sys_null(_frame: &mut SyscallFrame) -> i64 {
    0
}

/// SYS_WRITE_SERIAL: write bytes to debug port 0xE9.
///
/// arg1 = buffer pointer, arg2 = length.
/// Returns bytes written on success, negative on error.
///
/// Uses STAC/CLAC to temporarily disable SMAP for accessing user buffer.
/// Production should validate pointer range against USER_SPACE_END.
fn sys_write_serial(frame: &mut SyscallFrame) -> i64 {
    let ptr = frame.arg1() as *const u8;
    let len = frame.arg2() as usize;

    if len == 0 {
        return 0;
    }

    // Temporarily disable SMAP (set AC flag) to read from user page.
    unsafe { core::arch::asm!("stac", options(nostack)); }
    let slice = unsafe { core::slice::from_raw_parts(ptr, len) };

    let mut written: i64 = 0;
    for &byte in slice {
        // Write to QEMU debug exit port (0xE9) — same path as serial_debug helpers.
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") 0xE9u16,
                in("al") byte,
                options(nostack, preserves_flags)
            );
        }
        written += 1;
    }

    // Re-enable SMAP (clear AC flag).
    unsafe { core::arch::asm!("clac", options(nostack)); }

    written
}

/// SYS_GET_TICKS: return LAPIC timer tick count. No arguments.
fn sys_get_ticks(_frame: &mut SyscallFrame) -> i64 {
    crate::interrupts::timer_tick_count() as i64
}

/// Return the per-CPU user_rsp for the current CPU (BSP only for now).
///
/// Useful for debug/test: the syscall trampoline saves user RSP here.
pub fn debug_user_rsp() -> u64 {
    unsafe { percpu::bsp_percpu_mut().user_rsp }
}
