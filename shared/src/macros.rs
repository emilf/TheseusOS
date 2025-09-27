//! Macros for QEMU debug output and exit
//!
//! These macros centralize low-level I/O port sequences so callers can
//! print to QEMU's debug port (0xE9) and exit via isa-debug-exit (0xF4)
//! without duplicating inline assembly.

#[macro_export]
/// Print a string to QEMU debug port (0xE9)
///
/// This macro outputs each byte of the string to the QEMU debug port using
/// the x86 OUT instruction. QEMU captures this output and displays it in
/// the console or debug log.
///
/// # Assembly Details
/// - `out dx, al`: Output byte in AL register to I/O port in DX register
/// - `dx` = 0xE9: QEMU debug port I/O address
/// - `al` = byte: The character to output
macro_rules! qemu_print {
    ($s:expr) => {{
        let bytes: &[u8] = ($s).as_bytes();
        for &b in bytes {
            unsafe {
                core::arch::asm!(
                    "out dx, al",  // Output byte in AL to I/O port in DX
                    in("dx") $crate::constants::io_ports::QEMU_DEBUG,  // QEMU debug port (0xE9)
                    in("al") b,    // Byte to output
                    options(nomem, nostack, preserves_flags)
                );
            }
        }
    }};
}

#[macro_export]
/// Print raw bytes to QEMU debug port (0xE9)
///
/// This macro outputs each byte from a byte slice to the QEMU debug port.
/// Useful for printing binary data or when you already have bytes.
///
/// # Assembly Details
/// - `out dx, al`: Output byte in AL register to I/O port in DX register
/// - `dx` = 0xE9: QEMU debug port I/O address
/// - `al` = byte: The byte to output
macro_rules! qemu_print_bytes {
    ($bytes:expr) => {{
        let bytes: &[u8] = ($bytes);
        for &b in bytes {
            unsafe {
                core::arch::asm!(
                    "out dx, al",  // Output byte in AL to I/O port in DX
                    in("dx") $crate::constants::io_ports::QEMU_DEBUG,  // QEMU debug port (0xE9)
                    in("al") b,    // Byte to output
                    options(nomem, nostack, preserves_flags)
                );
            }
        }
    }};
}

#[macro_export]
/// Print a string followed by a newline to QEMU debug port (0xE9)
///
/// This macro prints the string and then outputs a newline character (0x0A)
/// to create a complete line of output.
///
/// # Assembly Details
/// - First calls `qemu_print!` to output the string
/// - Then `out dx, al`: Output newline byte (0x0A) to QEMU debug port
macro_rules! qemu_println {
    ($s:expr) => {{
        $crate::qemu_print!($s);
        unsafe {
            core::arch::asm!(
                "out dx, al",  // Output newline byte to I/O port
                in("dx") $crate::constants::io_ports::QEMU_DEBUG,  // QEMU debug port (0xE9)
                in("al") b'\n',  // Newline character (0x0A)
                options(nomem, nostack, preserves_flags)
            );
        }
    }};
}

#[macro_export]
/// Output a single byte to QEMU debug port (0xE9)
///
/// This macro outputs a single byte to the QEMU debug port. Useful for
/// outputting individual characters or control bytes.
///
/// # Assembly Details
/// - `out dx, al`: Output byte in AL register to I/O port in DX register
/// - `dx` = 0xE9: QEMU debug port I/O address
/// - `al` = byte: The byte to output
macro_rules! out_char_0xe9 {
    ($byte:expr) => {{
        let b: u8 = $byte;
        unsafe {
            core::arch::asm!(
                "out dx, al",  // Output byte in AL to I/O port in DX
                in("dx") $crate::constants::io_ports::QEMU_DEBUG,  // QEMU debug port (0xE9)
                in("al") b,    // Byte to output
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
            let ch: u8 = if nib < 10 {
                b'0' + nib
            } else {
                b'A' + (nib - 10)
            };
            $crate::out_char_0xe9!(ch);
            i -= 1;
        }
    }};
}

#[macro_export]
/// Exit QEMU with a specific exit code
///
/// This macro causes QEMU to exit with the specified exit code. This is useful
/// for cleanly terminating the virtual machine when the kernel is done.
///
/// # Assembly Details
/// - `out dx, al`: Output exit code in AL register to I/O port in DX register
/// - `dx` = 0xF4: QEMU isa-debug-exit device I/O address
/// - `al` = code: The exit code (0 = success, non-zero = error)
macro_rules! qemu_exit {
    ($code:expr) => {{
        let code: u8 = $code;
        unsafe {
            core::arch::asm!(
                "out dx, al",  // Output exit code to I/O port
                in("dx") $crate::constants::io_ports::QEMU_EXIT,  // QEMU exit port (0xF4)
                in("al") code,  // Exit code
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
