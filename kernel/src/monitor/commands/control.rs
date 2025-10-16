//! System control commands
//!
//! This module implements system control commands:
//! - `reset`: Reboot the system
//! - `halt`: Halt the CPU
//! - `clear`: Clear the terminal screen
//! - `call`: Call a function at an address (dangerous!)

use crate::monitor::Monitor;
use crate::monitor::parsing::parse_number;
use alloc::format;
use x86_64::instructions::port::Port;

impl Monitor {
    /// Reset the system
    pub(in crate::monitor) fn cmd_reset(&self) {
        self.writeln("Resetting system via keyboard controller...");

        // Small delay to let the message get out
        for _ in 0..100_000 {
            core::hint::spin_loop();
        }

        unsafe {
            // Try keyboard controller reset (port 0x64, command 0xFE)
            let mut kbd: Port<u8> = Port::new(0x64);
            kbd.write(0xFE);
        }

        // If that didn't work, try triple fault
        self.writeln("Keyboard reset failed, attempting triple fault...");
        unsafe {
            core::arch::asm!("lidt [{}]", in(reg) 0usize, options(nostack));
            core::arch::asm!("int3", options(nostack));
        }
    }

    /// Halt the CPU
    pub(in crate::monitor) fn cmd_halt(&self) {
        self.writeln("Halting CPU (use reset to recover)...");

        // Small delay to let the message get out
        for _ in 0..100_000 {
            core::hint::spin_loop();
        }

        loop {
            x86_64::instructions::hlt();
        }
    }

    /// Clear screen
    pub(in crate::monitor) fn cmd_clear(&self) {
        self.write_bytes(b"\x1B[2J\x1B[H"); // ANSI clear screen + cursor home
    }

    /// Call a function at an address
    pub(in crate::monitor) fn cmd_call(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: call ADDR");
            self.writeln("  ADDR - Function address to call");
            self.writeln("");
            self.writeln("WARNING: This can crash the system if address is invalid!");
            return;
        }

        let addr = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid address");
                return;
            }
        };

        self.writeln(&format!("Calling function at 0x{:016X}...", addr));

        unsafe {
            let func: extern "C" fn() = core::mem::transmute(addr);
            func();
        }

        self.writeln("Function returned");
    }
}
