//! Boot helpers: centralize early-boot abort handling and helpers.
#![allow(dead_code)]

/// Abort the boot process with a message and optional address context.
/// This prints the message to the kernel console, emits a QEMU error exit, and
/// panics to stop execution.
pub fn abort_boot(msg: &str, addr: Option<u64>) -> ! {
    crate::display::kernel_write_line(msg);
    if let Some(a) = addr {
        crate::display::kernel_write_line(" address=");
        theseus_shared::print_hex_u64_0xe9!(a);
        crate::display::kernel_write_line("\n");
    }
    theseus_shared::qemu_exit_error!();
    panic!("BOOT ABORT: {}", msg);
}

/// Abort with file/line context and register/backtrace dump.
pub fn abort_with_context(msg: &str, file: &str, line: u32, addr: Option<u64>) -> ! {
    crate::display::kernel_write_line("BOOT ABORT: ");
    crate::display::kernel_write_line(msg);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("  at ");
    crate::display::kernel_write_line(file);
    crate::display::kernel_write_line(":");
    crate::display::kernel_write_line(&alloc::format!("{}", line));
    crate::display::kernel_write_line("\n");
    if let Some(a) = addr {
        crate::display::kernel_write_line("  address=");
        theseus_shared::print_hex_u64_0xe9!(a);
        crate::display::kernel_write_line("\n");
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

    crate::display::kernel_write_line("  registers:\n");
    crate::display::kernel_write_line("   RIP=");
    theseus_shared::print_hex_u64_0xe9!(rip);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RSP=");
    theseus_shared::print_hex_u64_0xe9!(rsp);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RBP=");
    theseus_shared::print_hex_u64_0xe9!(rbp);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RAX=");
    theseus_shared::print_hex_u64_0xe9!(rax);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RBX=");
    theseus_shared::print_hex_u64_0xe9!(rbx);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RCX=");
    theseus_shared::print_hex_u64_0xe9!(rcx);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RDX=");
    theseus_shared::print_hex_u64_0xe9!(rdx);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RSI=");
    theseus_shared::print_hex_u64_0xe9!(rsi);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RDI=");
    theseus_shared::print_hex_u64_0xe9!(rdi);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   RFLAGS=");
    theseus_shared::print_hex_u64_0xe9!(flags);
    crate::display::kernel_write_line("\n");
    crate::display::kernel_write_line("   CR3=");
    theseus_shared::print_hex_u64_0xe9!(cr3);
    crate::display::kernel_write_line("\n");
}

fn dump_backtrace(max_frames: usize) {
    crate::display::kernel_write_line("  backtrace:\n");
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
        crate::display::kernel_write_line("    #");
        crate::display::kernel_write_line(&alloc::format!("{} ", i));
        theseus_shared::print_hex_u64_0xe9!(ret);
        crate::display::kernel_write_line("\n");
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
