//! System control commands
//!
//! This module implements system control commands:
//! - `reset`: Reboot the system
//! - `halt`: Halt the CPU
//! - `clear`: Clear the terminal screen
//! - `call`: Call a function at an address (dangerous!)

use crate::monitor::parsing::parse_number;
use crate::monitor::Monitor;
use alloc::format;
use x86_64::instructions::port::Port;

impl Monitor {
    /// Reset the system
    ///
    /// Reboots the computer using UEFI runtime services. Falls back to
    /// legacy methods if UEFI reset is unavailable or fails.
    ///
    /// Reset methods (in order of preference):
    /// 1. UEFI ResetSystem runtime service (proper firmware reset)
    /// 2. Keyboard controller reset (port 0x64, command 0xFE)
    /// 3. Triple fault (last resort)
    ///
    /// # Examples
    /// ```text
    /// reset             # Reboot the system
    /// ```
    ///
    /// # Note
    /// This command will terminate the current session and reboot the machine.
    /// All unsaved data will be lost.
    #[allow(unreachable_code)] // UEFI reset never returns, but we keep fallbacks for robustness
    pub(in crate::monitor) fn cmd_reset(&self) {
        self.writeln("Resetting system via UEFI runtime services...");

        // Small delay to let the message get out
        for _ in 0..100_000 {
            core::hint::spin_loop();
        }

        // Try UEFI ResetSystem runtime service first (best method)
        // This function never returns on success
        use uefi::runtime::ResetType;
        uefi::runtime::reset(ResetType::COLD, uefi::Status::SUCCESS, None);

        // If we get here, UEFI reset is not available (no runtime services)
        // Fall back to legacy reset methods

        self.writeln("UEFI reset unavailable, trying keyboard controller...");
        unsafe {
            let mut kbd: Port<u8> = Port::new(0x64);
            kbd.write(0xFE);
        }

        // Small delay to give keyboard reset a chance
        for _ in 0..100_000 {
            core::hint::spin_loop();
        }

        // Last resort: triple fault
        self.writeln("Keyboard reset failed, attempting triple fault...");
        unsafe {
            core::arch::asm!("lidt [{}]", in(reg) 0usize, options(nostack));
            core::arch::asm!("int3", options(nostack));
        }
    }

    /// Halt the CPU
    ///
    /// Stops the CPU using the HLT instruction in an infinite loop.
    /// The CPU will remain halted until a hardware reset or interrupt occurs
    /// (though interrupts should be disabled in this state).
    ///
    /// # Examples
    /// ```text
    /// halt              # Halt the CPU
    /// ```
    ///
    /// # Note
    /// This command effectively freezes the system. Use `reset` command to recover,
    /// or power cycle the machine.
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
    ///
    /// Clears the terminal screen using ANSI escape sequences:
    /// - ESC[2J: Clear entire screen
    /// - ESC[H: Move cursor to home position (top-left)
    ///
    /// # Examples
    /// ```text
    /// clear             # Clear the screen
    /// cls               # Alias for clear
    /// ```
    ///
    /// # Note
    /// Requires terminal support for ANSI escape sequences (most modern terminals).
    pub(in crate::monitor) fn cmd_clear(&self) {
        self.write_bytes(b"\x1B[2J\x1B[H"); // ANSI clear screen + cursor home
    }

    /// Call a function at an address
    ///
    /// Calls an arbitrary function at the specified memory address.
    /// The function is called with C calling convention and no arguments.
    ///
    /// # Arguments
    /// * `args` - Function address (hex or decimal)
    ///
    /// # Examples
    /// ```text
    /// call 0xFFFFFFFF80001000   # Call function at address
    /// ```
    ///
    /// # Safety
    /// ⚠️ **EXTREMELY DANGEROUS** ⚠️
    /// - Calling invalid addresses will crash the system
    /// - Functions may have unexpected side effects
    /// - No argument passing or return value handling
    /// - Stack state may be corrupted
    /// - Only use if you know exactly what you're doing!
    ///
    /// This command is primarily useful for calling known kernel functions
    /// during debugging sessions.
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
