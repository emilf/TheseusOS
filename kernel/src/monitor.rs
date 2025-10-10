//! Kernel Monitor - Interactive debugging console
//!
//! Inspired by Wozmon but extended with modern debugging features:
//! - Memory examination and modification
//! - Register inspection
//! - Stack traces
//! - ACPI table dumps
//! - Device enumeration  
//! - Execution control
//! - I/O port access
//! - MSR reading
//!
//! # Usage
//! The monitor provides an interactive command-line interface over the COM1 serial port.
//! Connect via QEMU serial (-serial stdio) or physical serial cable to interact.
//!
//! # Commands
//! Type 'help' in the monitor to see all available commands.

use crate::display::kernel_write_line;
use crate::drivers::manager::driver_manager;
use crate::drivers::traits::DeviceId;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use x86_64::instructions::port::Port;

/// Monitor prompt string
const PROMPT: &str = "\r\n> ";

/// Maximum line buffer size
const MAX_LINE: usize = 128;

/// Kernel monitor state
pub struct Monitor {
    /// Current input line buffer
    line_buffer: String,
    /// Last memory address examined (for continuation)
    last_addr: u64,
    /// COM1 device ID for communication
    com1: DeviceId,
}

impl Monitor {
    /// Create a new monitor instance
    pub fn new() -> Self {
        Self {
            line_buffer: String::with_capacity(MAX_LINE),
            last_addr: 0,
            com1: DeviceId::Raw("com1"),
        }
    }

    /// Write a string to the serial port
    fn write(&self, s: &str) {
        let _ = driver_manager().lock().write_to_device(&self.com1, s.as_bytes());
    }

    /// Write a string followed by newline
    fn writeln(&self, s: &str) {
        self.write(s);
        self.write("\r\n");
    }

    /// Main monitor loop - processes input and executes commands
    pub fn run(&mut self) -> ! {
        self.writeln("\r\n");
        self.writeln("=================================================");
        self.writeln("  TheseusOS Kernel Monitor v0.1");
        self.writeln("  Inspired by Wozmon - Extended for modern debug");
        self.writeln("=================================================");
        self.writeln("");
        self.writeln("Type 'help' for available commands");
        self.write(PROMPT);

        loop {
            // Try to read a character from the serial port
            let mut buf = [0u8; 1];
            if let Ok(1) = driver_manager().lock().read_from_device(&self.com1, &mut buf) {
                self.handle_char(buf[0]);
            } else {
                // No data available - halt to save power
                x86_64::instructions::hlt();
            }
        }
    }

    /// Handle a single input character
    fn handle_char(&mut self, ch: u8) {
        match ch {
            b'\r' | b'\n' => {
                // Enter key - process command
                self.write("\r\n");
                if !self.line_buffer.is_empty() {
                    self.process_command();
                    self.line_buffer.clear();
                }
                self.write(PROMPT);
            }
            0x08 | 0x7F => {
                // Backspace / DEL
                if !self.line_buffer.is_empty() {
                    self.line_buffer.pop();
                    self.write("\x08 \x08"); // Backspace, space, backspace
                }
            }
            0x03 => {
                // Ctrl+C - clear line and show new prompt
                self.line_buffer.clear();
                self.write("^C");
                self.write(PROMPT);
            }
            0x0C => {
                // Ctrl+L - clear screen
                self.write("\x1B[2J\x1B[H"); // ANSI clear screen + home
                self.line_buffer.clear();
                self.write(PROMPT);
            }
            byte if byte >= 0x20 && byte < 0x7F => {
                // Printable character
                if self.line_buffer.len() < MAX_LINE {
                    self.line_buffer.push(byte as char);
                    // Echo the character
                    let ch_buf = [byte];
                    let _ = driver_manager().lock().write_to_device(&self.com1, &ch_buf);
                }
            }
            _ => {
                // Ignore other control characters
            }
        }
    }

    /// Parse and execute a command line
    fn process_command(&mut self) {
        // Clone the line buffer to avoid borrow conflicts
        let line = self.line_buffer.clone();
        let parts: Vec<&str> = line.trim().split_whitespace().collect();

        if parts.is_empty() {
            return;
        }

        match parts[0] {
            "help" | "?" => self.cmd_help(),
            "regs" | "r" => self.cmd_registers(),
            "mem" | "m" => self.cmd_memory(&parts[1..]),
            "dump" | "d" => self.cmd_dump(&parts[1..]),
            "write" | "w" => self.cmd_write(&parts[1..]),
            "fill" | "f" => self.cmd_fill(&parts[1..]),
            "devices" | "dev" => self.cmd_devices(),
            "acpi" => self.cmd_acpi(),
            "stack" | "bt" => self.cmd_stack_trace(),
            "idt" => self.cmd_idt(),
            "gdt" => self.cmd_gdt(),
            "mmap" => self.cmd_memory_map(),
            "cpuid" => self.cmd_cpuid(),
            "msr" => self.cmd_msr(&parts[1..]),
            "io" => self.cmd_io(&parts[1..]),
            "int" => self.cmd_int(&parts[1..]),
            "call" => self.cmd_call(&parts[1..]),
            "reset" => self.cmd_reset(),
            "halt" => self.cmd_halt(),
            "clear" | "cls" => self.cmd_clear(),
            _ => {
                self.writeln(&format!("Unknown command: '{}'", parts[0]));
                self.writeln("Type 'help' for available commands");
            }
        }
    }

    /// Display help information
    fn cmd_help(&self) {
        self.writeln("Available commands:");
        self.writeln("");
        self.writeln("Memory Operations:");
        self.writeln("  mem ADDR           - Examine 16 bytes at ADDR");
        self.writeln("  dump ADDR [LEN]    - Hex dump memory region");
        self.writeln("  write ADDR VAL     - Write byte to memory");
        self.writeln("  fill ADDR LEN VAL  - Fill memory region");
        self.writeln("");
        self.writeln("System Information:");
        self.writeln("  regs, r            - Display CPU registers");
        self.writeln("  devices, dev       - List hardware devices");
        self.writeln("  acpi               - ACPI platform info");
        self.writeln("  mmap               - Memory map from UEFI");
        self.writeln("  cpuid              - CPU identification");
        self.writeln("");
        self.writeln("System Tables:");
        self.writeln("  idt                - Show IDT information");
        self.writeln("  gdt                - Show GDT information");
        self.writeln("");
        self.writeln("Debugging:");
        self.writeln("  stack, bt          - Stack backtrace");
        self.writeln("  msr ADDR           - Read model-specific register");
        self.writeln("  io r PORT          - Read from I/O port");
        self.writeln("  io w PORT VAL      - Write to I/O port");
        self.writeln("  int NUM            - Trigger software interrupt");
        self.writeln("  call ADDR          - Call function at address");
        self.writeln("");
        self.writeln("System Control:");
        self.writeln("  reset              - Reset system");
        self.writeln("  halt               - Halt CPU");
        self.writeln("  clear, cls         - Clear screen");
        self.writeln("");
        self.writeln("Addresses can be in hex (0x...) or decimal");
    }

    /// Display CPU registers
    fn cmd_registers(&self) {
        self.writeln("CPU Registers:");

        unsafe {
            let rsp: u64;
            let rbp: u64;
            let rax: u64;
            let rbx: u64;
            let rcx: u64;
            let rdx: u64;
            let rsi: u64;
            let rdi: u64;

            core::arch::asm!(
                "mov {rsp}, rsp",
                "mov {rbp}, rbp",
                "mov {rax}, rax",
                "mov {rbx}, rbx",
                "mov {rcx}, rcx",
                "mov {rdx}, rdx",
                "mov {rsi}, rsi",
                "mov {rdi}, rdi",
                rsp = out(reg) rsp,
                rbp = out(reg) rbp,
                rax = out(reg) rax,
                rbx = out(reg) rbx,
                rcx = out(reg) rcx,
                rdx = out(reg) rdx,
                rsi = out(reg) rsi,
                rdi = out(reg) rdi,
                options(nostack)
            );

            self.writeln(&format!("  RAX: 0x{:016X}  RBX: 0x{:016X}", rax, rbx));
            self.writeln(&format!("  RCX: 0x{:016X}  RDX: 0x{:016X}", rcx, rdx));
            self.writeln(&format!("  RSI: 0x{:016X}  RDI: 0x{:016X}", rsi, rdi));
            self.writeln(&format!("  RSP: 0x{:016X}  RBP: 0x{:016X}", rsp, rbp));

            // Control registers
            let cr0: u64;
            let cr2: u64;
            let cr3: u64;
            let cr4: u64;

            core::arch::asm!("mov {}, cr0", out(reg) cr0, options(nomem, nostack));
            core::arch::asm!("mov {}, cr2", out(reg) cr2, options(nomem, nostack));
            core::arch::asm!("mov {}, cr3", out(reg) cr3, options(nomem, nostack));
            core::arch::asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack));

            self.writeln("");
            self.writeln("Control Registers:");
            self.writeln(&format!("  CR0: 0x{:016X}  CR2: 0x{:016X}", cr0, cr2));
            self.writeln(&format!("  CR3: 0x{:016X}  CR4: 0x{:016X}", cr3, cr4));

            // Segment registers
            let cs: u16;
            let ds: u16;
            let ss: u16;
            let es: u16;

            core::arch::asm!("mov {:x}, cs", out(reg) cs, options(nomem, nostack));
            core::arch::asm!("mov {:x}, ds", out(reg) ds, options(nomem, nostack));
            core::arch::asm!("mov {:x}, ss", out(reg) ss, options(nomem, nostack));
            core::arch::asm!("mov {:x}, es", out(reg) es, options(nomem, nostack));

            self.writeln("");
            self.writeln("Segment Registers:");
            self.writeln(&format!("  CS: 0x{:04X}  DS: 0x{:04X}  SS: 0x{:04X}  ES: 0x{:04X}", 
                cs, ds, ss, es));
        }
    }

    /// Examine memory at address
    fn cmd_memory(&mut self, args: &[&str]) {
        if args.is_empty() {
            // Continue from last address
            if self.last_addr == 0 {
                self.writeln("Usage: mem ADDR");
                self.writeln("  ADDR can be hex (0x...) or decimal");
                return;
            }
        } else {
            // Parse new address
            if let Some(addr) = parse_number(args[0]) {
                self.last_addr = addr;
            } else {
                self.writeln("Invalid address");
                return;
            }
        }

        // Display 16 bytes in hex + ASCII
        unsafe {
            let ptr = self.last_addr as *const u8;
            self.write(&format!("{:016X}: ", self.last_addr));

            // Hex values
            for i in 0..16 {
                let byte = core::ptr::read_volatile(ptr.add(i));
                self.write(&format!("{:02X} ", byte));
            }

            // ASCII representation
            self.write(" |");
            for i in 0..16 {
                let byte = core::ptr::read_volatile(ptr.add(i));
                if byte >= 0x20 && byte < 0x7F {
                    self.write(&format!("{}", byte as char));
                } else {
                    self.write(".");
                }
            }
            self.writeln("|");
        }

        self.last_addr += 16;
    }

    /// Dump memory region
    fn cmd_dump(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: dump ADDR [LENGTH]");
            self.writeln("  ADDR   - Starting address (hex or decimal)");
            self.writeln("  LENGTH - Number of bytes to dump (default: 256)");
            return;
        }

        let addr = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid address");
                return;
            }
        };

        let len = if args.len() > 1 {
            parse_number(args[1]).unwrap_or(256)
        } else {
            256
        };

        self.writeln(&format!("Memory dump at 0x{:X}, {} bytes:", addr, len));

        unsafe {
            for offset in (0..len).step_by(16) {
                let line_addr = addr + offset;
                let ptr = line_addr as *const u8;

                self.write(&format!("{:016X}: ", line_addr));

                // Hex values
                for i in 0..16usize {
                    if offset + (i as u64) < len {
                        let byte = core::ptr::read_volatile(ptr.add(i));
                        self.write(&format!("{:02X} ", byte));
                    } else {
                        self.write("   ");
                    }
                }

                // ASCII representation
                self.write(" |");
                for i in 0..16usize {
                    if offset + (i as u64) < len {
                        let byte = core::ptr::read_volatile(ptr.add(i));
                        if byte >= 0x20 && byte < 0x7F {
                            self.write(&format!("{}", byte as char));
                        } else {
                            self.write(".");
                        }
                    }
                }
                self.writeln("|");
            }
        }
    }

    /// Write a byte to memory
    fn cmd_write(&self, args: &[&str]) {
        if args.len() < 2 {
            self.writeln("Usage: write ADDR VALUE");
            self.writeln("  ADDR  - Memory address (hex or decimal)");
            self.writeln("  VALUE - Byte value to write (hex or decimal)");
            return;
        }

        let addr = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid address");
                return;
            }
        };

        let value = match parse_number(args[1]) {
            Some(v) => (v & 0xFF) as u8,
            None => {
                self.writeln("Invalid value");
                return;
            }
        };

        unsafe {
            core::ptr::write_volatile(addr as *mut u8, value);
            self.writeln(&format!("Wrote 0x{:02X} to 0x{:016X}", value, addr));
        }
    }

    /// Fill memory region with a value
    fn cmd_fill(&self, args: &[&str]) {
        if args.len() < 3 {
            self.writeln("Usage: fill ADDR LENGTH VALUE");
            self.writeln("  ADDR   - Starting address");
            self.writeln("  LENGTH - Number of bytes to fill");
            self.writeln("  VALUE  - Byte value to write");
            return;
        }

        let addr = match parse_number(args[0]) {
            Some(a) => a,
            None => {
                self.writeln("Invalid address");
                return;
            }
        };

        let len = match parse_number(args[1]) {
            Some(l) => l as usize,
            None => {
                self.writeln("Invalid length");
                return;
            }
        };

        let value = match parse_number(args[2]) {
            Some(v) => (v & 0xFF) as u8,
            None => {
                self.writeln("Invalid value");
                return;
            }
        };

        unsafe {
            let ptr = addr as *mut u8;
            for i in 0..len {
                core::ptr::write_volatile(ptr.add(i), value);
            }
            self.writeln(&format!("Filled {} bytes at 0x{:016X} with 0x{:02X}", 
                len, addr, value));
        }
    }

    /// List registered devices
    fn cmd_devices(&self) {
        self.writeln("Registered devices:");
        let mgr = driver_manager().lock();
        let devices = mgr.devices();

        if devices.is_empty() {
            self.writeln("  (no devices registered)");
            return;
        }

        for (i, dev) in devices.iter().enumerate() {
            let bound = if dev.driver_data.is_some() { "[BOUND]" } else { "" };
            let addr_str = match dev.phys_addr {
                Some(a) => format!("0x{:X}", a),
                None => String::from("none"),
            };
            let irq_str = match dev.irq {
                Some(i) => format!("{}", i),
                None => String::from("none"),
            };

            self.writeln(&format!("  [{:2}] {} {} addr:{} irq:{}",
                i, dev.id, bound, addr_str, irq_str));
        }
    }

    /// Display ACPI information
    fn cmd_acpi(&self) {
        self.writeln("ACPI Information:");

        let handoff = unsafe {
            &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff)
        };

        self.writeln(&format!("  RSDP Address: 0x{:X}", handoff.acpi_rsdp));
        self.writeln(&format!("  CPU Count: {}", handoff.cpu_count));

        // Could extend to show MADT, FADT, DSDT info
        self.writeln("  (Extended ACPI table info would be displayed here)");
    }

    /// Display stack backtrace
    fn cmd_stack_trace(&self) {
        self.writeln("Stack trace:");

        unsafe {
            let mut rbp: u64;
            core::arch::asm!("mov {}, rbp", out(reg) rbp, options(nostack));

            for frame in 0..16 {
                // Validate RBP is in kernel space
                if rbp == 0 || rbp < 0xFFFF_8000_0000_0000 {
                    break;
                }

                // Read return address and previous RBP
                let ret_addr_ptr = (rbp + 8) as *const u64;
                let prev_rbp_ptr = rbp as *const u64;

                let ret_addr = core::ptr::read_volatile(ret_addr_ptr);
                let prev_rbp = core::ptr::read_volatile(prev_rbp_ptr);

                self.writeln(&format!("  Frame #{:2}: RIP=0x{:016X} RBP=0x{:016X}",
                    frame, ret_addr, rbp));

                // Move to next frame
                rbp = prev_rbp;

                // Prevent infinite loops
                if rbp == prev_rbp {
                    break;
                }
            }
        }
    }

    /// Display IDT information
    fn cmd_idt(&self) {
        self.writeln("Interrupt Descriptor Table:");

        use x86_64::instructions::tables::sidt;
        let idtr = sidt();
        self.writeln(&format!("  IDTR Base:  0x{:016X}", idtr.base.as_u64()));
        self.writeln(&format!("  IDTR Limit: 0x{:04X} ({} entries)",
            idtr.limit, (idtr.limit + 1) / 16));

        // Show some key vectors
        self.writeln("");
        self.writeln("Key interrupt vectors:");
        self.writeln("  0x00: Divide Error");
        self.writeln("  0x03: Breakpoint");
        self.writeln("  0x06: Invalid Opcode");
        self.writeln("  0x0D: General Protection Fault");
        self.writeln("  0x0E: Page Fault");
        self.writeln("  0x24: COM1 Serial (IRQ 4)");
        self.writeln("  0x40: APIC Timer");
        self.writeln("  0xFE: APIC Error");
        self.writeln("  0xFF: Spurious");
    }

    /// Display GDT information
    fn cmd_gdt(&self) {
        self.writeln("Global Descriptor Table:");

        use x86_64::instructions::tables::sgdt;
        let gdtr = sgdt();
        self.writeln(&format!("  GDTR Base:  0x{:016X}", gdtr.base.as_u64()));
        self.writeln(&format!("  GDTR Limit: 0x{:04X} ({} entries)",
            gdtr.limit, (gdtr.limit + 1) / 8));
    }

    /// Display memory map
    fn cmd_memory_map(&self) {
        self.writeln("UEFI Memory Map:");

        let handoff = unsafe {
            &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff)
        };

        self.writeln(&format!("  Descriptor Size: {} bytes", handoff.memory_map_descriptor_size));
        self.writeln(&format!("  Entry Count: {}", handoff.memory_map_entries));
        self.writeln(&format!("  Total Size: {} bytes", handoff.memory_map_size));
        self.writeln("");
        self.writeln("  (Full memory map parsing would be displayed here)");
    }

    /// Display CPUID information
    fn cmd_cpuid(&self) {
        self.writeln("CPUID Information:");

        use raw_cpuid::CpuId;
        let cpuid = CpuId::new();

        if let Some(vi) = cpuid.get_vendor_info() {
            self.writeln(&format!("  Vendor: {}", vi.as_str()));
        }

        if let Some(fi) = cpuid.get_feature_info() {
            self.writeln(&format!("  Family: 0x{:X}", fi.family_id()));
            self.writeln(&format!("  Model: 0x{:X}", fi.model_id()));
            self.writeln(&format!("  Stepping: 0x{:X}", fi.stepping_id()));

            self.writeln("");
            self.writeln("Features:");
            if fi.has_fpu() { self.writeln("  - FPU (Floating Point Unit)"); }
            if fi.has_pae() { self.writeln("  - PAE (Physical Address Extension)"); }
            if fi.has_apic() { self.writeln("  - APIC (Advanced PIC)"); }
            if fi.has_msr() { self.writeln("  - MSR (Model Specific Registers)"); }
            if fi.has_tsc() { self.writeln("  - TSC (Time Stamp Counter)"); }
            if fi.has_sse() { self.writeln("  - SSE"); }
            if fi.has_sse2() { self.writeln("  - SSE2"); }
            if fi.has_x2apic() { self.writeln("  - x2APIC"); }
        }

        if let Some(ef) = cpuid.get_extended_feature_info() {
            if ef.has_fsgsbase() { self.writeln("  - FSGSBASE"); }
        }
    }

    /// Read a model-specific register
    fn cmd_msr(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: msr ADDRESS");
            self.writeln("  ADDRESS - MSR address in hex (e.g., 0x1B for APIC_BASE)");
            self.writeln("");
            self.writeln("Common MSRs:");
            self.writeln("  0x1B  - IA32_APIC_BASE");
            self.writeln("  0x10  - IA32_TIME_STAMP_COUNTER");
            self.writeln("  0x174 - IA32_SYSENTER_CS");
            self.writeln("  0xC0000080 - IA32_EFER");
            return;
        }

        let msr_addr = match parse_number(args[0]) {
            Some(a) => a as u32,
            None => {
                self.writeln("Invalid MSR address");
                return;
            }
        };

        unsafe {
            let value = x86_64::registers::model_specific::Msr::new(msr_addr).read();
            self.writeln(&format!("MSR 0x{:X} = 0x{:016X}", msr_addr, value));
        }
    }

    /// I/O port access
    fn cmd_io(&self, args: &[&str]) {
        if args.len() < 2 {
            self.writeln("Usage: io r/w PORT [VALUE]");
            self.writeln("  r PORT      - Read byte from port");
            self.writeln("  w PORT VAL  - Write byte to port");
            return;
        }

        let port = match parse_number(args[1]) {
            Some(p) if p <= 0xFFFF => p as u16,
            _ => {
                self.writeln("Invalid port (must be 0-0xFFFF)");
                return;
            }
        };

        match args[0] {
            "r" | "read" => {
                unsafe {
                    let mut p: Port<u8> = Port::new(port);
                    let value = p.read();
                    self.writeln(&format!("IN 0x{:X} = 0x{:02X}", port, value));
                }
            }
            "w" | "write" => {
                if args.len() < 3 {
                    self.writeln("Usage: io w PORT VALUE");
                    return;
                }

                let value = match parse_number(args[2]) {
                    Some(v) => (v & 0xFF) as u8,
                    None => {
                        self.writeln("Invalid value");
                        return;
                    }
                };

                unsafe {
                    let mut p: Port<u8> = Port::new(port);
                    p.write(value);
                    self.writeln(&format!("OUT 0x{:X} = 0x{:02X}", port, value));
                }
            }
            _ => {
                self.writeln("Invalid operation (use 'r' or 'w')");
            }
        }
    }

    /// Trigger software interrupt
    fn cmd_int(&self, args: &[&str]) {
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
                3 => core::arch::asm!("int3", options(nostack)),
                _ => {
                    self.writeln("Only INT3 is currently safe to trigger");
                    return;
                }
            }
        }

        self.writeln("Returned from interrupt");
    }

    /// Call a function at an address
    fn cmd_call(&self, args: &[&str]) {
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

    /// Reset the system
    fn cmd_reset(&self) {
        self.writeln("Resetting system via keyboard controller...");

        // Small delay to let the message get out
        for _ in 0..100000 {
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
    fn cmd_halt(&self) {
        self.writeln("Halting CPU (use reset to recover)...");

        // Small delay to let the message get out
        for _ in 0..100000 {
            core::hint::spin_loop();
        }

        loop {
            x86_64::instructions::hlt();
        }
    }

    /// Clear screen
    fn cmd_clear(&self) {
        self.write("\x1B[2J\x1B[H"); // ANSI clear screen + cursor home
    }
}

/// Parse a number from string (supports hex with 0x prefix and decimal)
fn parse_number(s: &str) -> Option<u64> {
    let s = s.trim();

    if s.starts_with("0x") || s.starts_with("0X") {
        // Hex number
        let hex = s.trim_start_matches("0x").trim_start_matches("0X");
        u64::from_str_radix(hex, 16).ok()
    } else {
        // Decimal number
        s.parse::<u64>().ok()
    }
}

/// Start the kernel monitor (never returns)
pub fn start_monitor() -> ! {
    kernel_write_line("[monitor] starting kernel monitor");

    let mut monitor = Monitor::new();
    monitor.run()
}

