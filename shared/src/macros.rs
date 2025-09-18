//! Macros for QEMU debug output and exit
//!
//! These macros centralize low-level I/O port sequences so callers can
//! print to QEMU's debug port (0xE9) and exit via isa-debug-exit (0xF4)
//! without duplicating inline assembly.

#[macro_export]
macro_rules! qemu_print {
    ($s:expr) => {{
        let bytes: &[u8] = ($s).as_bytes();
        for &b in bytes {
            unsafe {
                core::arch::asm!(
                    "out dx, al",
                    in("dx") $crate::constants::io_ports::QEMU_DEBUG,
                    in("al") b,
                    options(nomem, nostack, preserves_flags)
                );
            }
        }
    }};
}

#[macro_export]
macro_rules! qemu_print_bytes {
    ($bytes:expr) => {{
        let bytes: &[u8] = ($bytes);
        for &b in bytes {
            unsafe {
                core::arch::asm!(
                    "out dx, al",
                    in("dx") $crate::constants::io_ports::QEMU_DEBUG,
                    in("al") b,
                    options(nomem, nostack, preserves_flags)
                );
            }
        }
    }};
}

#[macro_export]
macro_rules! qemu_println {
    ($s:expr) => {{
        $crate::qemu_print!($s);
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") $crate::constants::io_ports::QEMU_DEBUG,
                in("al") b'\n',
                options(nomem, nostack, preserves_flags)
            );
        }
    }};
}

#[macro_export]
macro_rules! out_char_0xe9 {
    ($byte:expr) => {{
        let b: u8 = $byte;
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") $crate::constants::io_ports::QEMU_DEBUG,
                in("al") b,
                options(nomem, nostack, preserves_flags)
            );
        }
    }};
}

#[macro_export]
macro_rules! print_hex_u64_0xe9 {
    ($value:expr) => {{
        let mut v: u64 = $value as u64;
        $crate::out_char_0xe9!(b'0');
        $crate::out_char_0xe9!(b'x');
        let mut i: i32 = 15;
        while i >= 0 {
            let nib: u8 = ((v >> (i * 4)) & 0xF) as u8;
            let ch: u8 = if nib < 10 { b'0' + nib } else { b'A' + (nib - 10) };
            $crate::out_char_0xe9!(ch);
            i -= 1;
        }
    }};
}

#[macro_export]
macro_rules! qemu_exit {
    ($code:expr) => {{
        let code: u8 = $code;
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") $crate::constants::io_ports::QEMU_EXIT,
                in("al") code,
                options(nomem, nostack, preserves_flags)
            );
        }
    }};
}

#[macro_export]
macro_rules! qemu_exit_ok {
    () => {{
        $crate::qemu_exit!($crate::constants::exit_codes::QEMU_SUCCESS)
    }};
}

#[macro_export]
macro_rules! qemu_exit_error {
    () => {{
        $crate::qemu_exit!($crate::constants::exit_codes::QEMU_ERROR)
    }};
}


