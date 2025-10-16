//! Boot helpers: centralize early-boot abort handling and helpers.
#![allow(dead_code)]

use crate::log_error;

/// Abort the boot process with a message and optional address context.
/// This prints the message to the kernel console, emits a QEMU error exit, and
/// panics to stop execution.
pub fn abort_boot(msg: &str, addr: Option<u64>) -> ! {
    if let Some(a) = addr {
        log_error!("BOOT ABORT: {} address={:#x}", msg, a);
    } else {
        log_error!("BOOT ABORT: {}", msg);
    }
    theseus_shared::qemu_exit_error!();
    panic!("BOOT ABORT: {}", msg);
}

/// Abort with file/line context and register/backtrace dump.
pub fn abort_with_context(msg: &str, file: &str, line: u32, addr: Option<u64>) -> ! {
    if let Some(a) = addr {
        log_error!("BOOT ABORT: {} at {}:{} address={:#x}", msg, file, line, a);
    } else {
        log_error!("BOOT ABORT: {} at {}:{}", msg, file, line);
    }

    // Dump key registers
    dump_registers();

    // Dump a small backtrace using frame pointers
    dump_backtrace(8);

    // Finalize via the original abort
    abort_boot(msg, addr);
}

fn dump_registers() {
    use x86_64::registers::control::Cr3;
    let rip: u64;
    let rsp: u64;
    let rbp: u64;
    let rdi: u64;
    let rsi: u64;
    let rdx: u64;
    let rcx: u64;
    let rbx: u64;
    let rax: u64;
    let flags: u64;
    unsafe {
        core::arch::asm!("lea {}, [rip + 0]", out(reg) rip, options(nostack));
        core::arch::asm!("mov {}, rsp", out(reg) rsp, options(nomem, preserves_flags));
        core::arch::asm!("mov {}, rbp", out(reg) rbp, options(nomem, preserves_flags));
        core::arch::asm!("mov {}, rdi", out(reg) rdi, options(nomem, preserves_flags));
        core::arch::asm!("mov {}, rsi", out(reg) rsi, options(nomem, preserves_flags));
        core::arch::asm!("mov {}, rdx", out(reg) rdx, options(nomem, preserves_flags));
        core::arch::asm!("mov {}, rcx", out(reg) rcx, options(nomem, preserves_flags));
        core::arch::asm!("mov {}, rbx", out(reg) rbx, options(nomem, preserves_flags));
        core::arch::asm!("mov {}, rax", out(reg) rax, options(nomem, preserves_flags));
        core::arch::asm!("pushfq; pop {}", out(reg) flags, options(nomem));
    }
    let (cr3_frame, _f) = Cr3::read();
    let cr3 = cr3_frame.start_address().as_u64();

    log_error!("  registers:");
    log_error!("   RIP={:#x} RSP={:#x} RBP={:#x}", rip, rsp, rbp);
    log_error!("   RAX={:#x} RBX={:#x} RCX={:#x} RDX={:#x}", rax, rbx, rcx, rdx);
    log_error!("   RSI={:#x} RDI={:#x} RFLAGS={:#x}", rsi, rdi, flags);
    log_error!("   CR3={:#x}", cr3);
}

fn dump_backtrace(max_frames: usize) {
    log_error!("  backtrace:");
    // Get initial RBP
    let mut rbp: u64;
    unsafe {
        core::arch::asm!("mov {}, rbp", out(reg) rbp, options(nomem, preserves_flags));
    }
    for i in 0..max_frames {
        if rbp == 0 {
            break;
        }
        // saved return address is at [rbp + 8]
        let ret: u64 = unsafe { core::ptr::read_volatile((rbp + 8) as *const u64) };
        log_error!("    #{} {:#x}", i, ret);
        // next rbp is at [rbp]
        let next = unsafe { core::ptr::read_volatile(rbp as *const u64) };
        if next == rbp {
            break;
        }
        rbp = next;
    }
}

#[macro_export]
macro_rules! boot_abort {
    ($msg:expr) => {
        $crate::boot::abort_with_context($msg, file!(), line!(), None)
    };
    ($msg:expr, $addr:expr) => {
        $crate::boot::abort_with_context($msg, file!(), line!(), Some($addr as u64))
    };
}
