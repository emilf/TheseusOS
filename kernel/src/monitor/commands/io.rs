//! I/O and interrupt commands
//!
//! This module implements low-level I/O operations:
//! - `io`: Read/write I/O ports (8/16/32-bit)
//! - `int`: Trigger software interrupts

use crate::monitor::Monitor;
use crate::monitor::parsing::parse_number;
use alloc::format;
use x86_64::instructions::port::Port;

impl Monitor {
    /// I/O port access
    pub(in crate::monitor) fn cmd_io(&self, args: &[&str]) {
        if args.len() < 2 {
            self.writeln("Usage: io (r|w)[8|16|32] PORT [VALUE]");
            self.writeln("  io r 0x10C         # read byte (default width 8)");
            self.writeln("  io r16 0x64        # read 16-bit value");
            self.writeln("  io w32 0xCF8 0x80000010");
            return;
        }

        #[derive(Copy, Clone)]
        enum IoOp {
            Read,
            Write,
        }

        fn parse_io_token(token: &str) -> Option<(IoOp, u8)> {
            if token.eq_ignore_ascii_case("r") || token.eq_ignore_ascii_case("read") {
                return Some((IoOp::Read, 8));
            }
            if token.eq_ignore_ascii_case("w") || token.eq_ignore_ascii_case("write") {
                return Some((IoOp::Write, 8));
            }

            if token.len() > 1 {
                let (prefix, width_str) = token.split_at(1);
                let op = if prefix.eq_ignore_ascii_case("r") {
                    IoOp::Read
                } else if prefix.eq_ignore_ascii_case("w") {
                    IoOp::Write
                } else {
                    return None;
                };
                match width_str.parse::<u8>() {
                    Ok(8) => Some((op, 8)),
                    Ok(16) => Some((op, 16)),
                    Ok(32) => Some((op, 32)),
                    _ => None,
                }
            } else {
                None
            }
        }

        let (op, width) = match parse_io_token(args[0]) {
            Some(res) => res,
            None => {
                self.writeln(
                    "Invalid operation token (expected r|w optionally suffixed with 8/16/32)",
                );
                return;
            }
        };

        let port = match parse_number(args[1]) {
            Some(p) if p <= 0xFFFF => p as u16,
            _ => {
                self.writeln("Invalid port (must be 0-0xFFFF)");
                return;
            }
        };

        match op {
            IoOp::Read => unsafe {
                match width {
                    8 => {
                        let mut p: Port<u8> = Port::new(port);
                        let value = p.read();
                        self.writeln(&format!("IN8  0x{:04X} = 0x{:02X}", port, value));
                    }
                    16 => {
                        let mut p: Port<u16> = Port::new(port);
                        let value = p.read();
                        self.writeln(&format!("IN16 0x{:04X} = 0x{:04X}", port, value));
                    }
                    32 => {
                        let mut p: Port<u32> = Port::new(port);
                        let value = p.read();
                        self.writeln(&format!("IN32 0x{:04X} = 0x{:08X}", port, value));
                    }
                    _ => unreachable!(),
                }
            },
            IoOp::Write => {
                if args.len() < 3 {
                    self.writeln("Usage: io w[width] PORT VALUE");
                    return;
                }
                let raw_value = match parse_number(args[2]) {
                    Some(v) => v,
                    None => {
                        self.writeln("Invalid value");
                        return;
                    }
                };

                unsafe {
                    match width {
                        8 => {
                            let mut p: Port<u8> = Port::new(port);
                            let value = (raw_value & 0xFF) as u8;
                            p.write(value);
                            self.writeln(&format!("OUT8  0x{:04X} <- 0x{:02X}", port, value));
                        }
                        16 => {
                            let mut p: Port<u16> = Port::new(port);
                            let value = (raw_value & 0xFFFF) as u16;
                            p.write(value);
                            self.writeln(&format!("OUT16 0x{:04X} <- 0x{:04X}", port, value));
                        }
                        32 => {
                            let mut p: Port<u32> = Port::new(port);
                            let value = raw_value as u32;
                            p.write(value);
                            self.writeln(&format!("OUT32 0x{:04X} <- 0x{:08X}", port, value));
                        }
                        _ => unreachable!(),
                    }
                }
            }
        }
    }

    /// Trigger software interrupt
    pub(in crate::monitor) fn cmd_int(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: int NUM");
            self.writeln("  NUM - Interrupt vector number (0-255)");
            self.writeln("");
            self.writeln("WARNING: This will trigger a real interrupt!");
            return;
        }

        let vector = match parse_number(args[0]) {
            Some(v) if v <= 255 => v as u8,
            _ => {
                self.writeln("Invalid vector (must be 0-255)");
                return;
            }
        };

        self.writeln(&format!("Triggering interrupt 0x{:02X}...", vector));

        unsafe {
            match vector {
                3 => core::arch::asm!("int3"),
                0x80 => core::arch::asm!("int 0x80"),
                _ => {
                    self.writeln("Only INT3 and INT 0x80 are permitted from the monitor");
                    return;
                }
            }
        }

        self.writeln("Interrupt completed");
    }
}
