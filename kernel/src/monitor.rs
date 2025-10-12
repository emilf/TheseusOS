//! # Kernel Monitor - Interactive Debugging Console
//!
//! This module implements an interactive serial monitor inspired by the legendary **Wozmon**
//! (the Apple I monitor program written by Steve Wozniak in 1976). While maintaining the
//! spirit of Wozmon's simplicity and direct hardware access, this implementation extends
//! the concept for modern x86-64 systems.
//!
//! ## Features
//!
//! ### Memory Operations
//! - **Examination**: View memory contents in hex and ASCII
//! - **Modification**: Write individual bytes or fill regions
//! - **Dumping**: Large memory region dumps with formatting
//! - **Continuation**: Press Enter without arguments to continue from last address
//!
//! ### System Inspection
//! - **Registers**: View general-purpose, control, and segment registers
//! - **Stack Traces**: Walk frame pointers to display call chain
//! - **ACPI**: Display platform configuration information
//! - **CPU Info**: Show processor features via CPUID
//! - **Device List**: Enumerate all registered hardware devices
//!
//! ### Low-Level Access
//! - **I/O Ports**: Read/write to x86 I/O space
//! - **MSRs**: Read Model-Specific Registers
//! - **Interrupts**: Trigger software interrupts (carefully!)
//! - **Function Calls**: Execute arbitrary code (dangerous!)
//!
//! ### System Control
//! - **Reset**: Reboot the system
//! - **Halt**: Stop CPU execution
//! - **Clear**: Clear terminal screen
//!
//! ## Usage
//!
//! The monitor provides an interactive command-line interface over the COM1 serial port.
//! Connect using:
//!
//! - QEMU: `-serial stdio` or `-serial unix:/tmp/qemu.sock,server,nowait`
//! - Physical hardware: `minicom -D /dev/ttyS0 -b 115200` or `screen /dev/ttyS0 115200`
//!
//! Type 'help' at the prompt to see all available commands.
//!
//! ## Design
//!
//! The monitor operates entirely in kernel mode with full privileges. It receives input
//! via serial interrupts (IRQ 4), processes commands synchronously, and outputs results
//! back through the serial port. The implementation is deliberately simple, avoiding
//! complex buffering or asynchronous processing.
//!
//! ### Thread Safety
//!
//! The monitor state is protected by a spin mutex (`MONITOR`), allowing safe access
//! from both the serial IRQ handler and the main kernel thread. Commands execute
//! atomically while holding the lock.
//!
//! ## History
//!
//! Wozmon was written in 1976 by Steve Wozniak for the Apple I computer. It provided
//! a minimalist interface for examining and modifying memory, entering programs, and
//! executing code - all in just 256 bytes. This implementation honors that legacy while
//! adapting to 64-bit architectures and modern debugging needs.

use crate::config;
use crate::display::kernel_write_line;
use crate::drivers::manager::driver_manager;
use crate::drivers::serial;
use crate::drivers::traits::DeviceClass;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use spin::Mutex;
use x86_64::instructions::port::Port;

/// Monitor prompt string
const PROMPT: &str = "\r\n> ";

/// Maximum line buffer size
const MAX_LINE: usize = 128;

/// Shared monitor instance guarded by a spin mutex.
static MONITOR: Mutex<Option<Monitor>> = Mutex::new(None);

/// Prepare the monitor subsystem. Safe to call multiple times.
pub fn init() {
    if !config::ENABLE_KERNEL_MONITOR {
        return;
    }

    let mut guard = MONITOR.lock();
    if guard.is_none() {
        let mut monitor = Monitor::new();
        monitor.activate();
        *guard = Some(monitor);
        kernel_write_line("[monitor] interactive console ready");
    } else if let Some(monitor) = guard.as_mut() {
        monitor.activate();
    }
}

/// Process a byte received from the serial device.
pub fn notify_serial_byte(byte: u8) {
    push_serial_byte(byte);
}

/// Allow drivers (e.g., from IRQ context) to push incoming bytes.
pub fn push_serial_byte(byte: u8) {
    if !config::ENABLE_KERNEL_MONITOR {
        return;
    }

    let mut guard = MONITOR.lock();
    if guard.is_none() {
        let mut monitor = Monitor::new();
        monitor.activate();
        *guard = Some(monitor);
        kernel_write_line("[monitor] interactive console ready");
    }

    if let Some(monitor) = guard.as_mut() {
        monitor.handle_char(byte);
    }
}

/// Low-level helper for writing raw strings to the monitor output.
pub fn write_serial(msg: &str) {
    if serial::write_bytes_direct(msg.as_bytes()).is_err() {
        let _ = driver_manager()
            .lock()
            .write_class(DeviceClass::Serial, msg.as_bytes());
    }
}

/// Start the monitor loop (used when we want to park the CPU in monitor mode).
pub fn start_monitor() -> ! {
    init();
    loop {
        x86_64::instructions::hlt();
    }
}

/// Kernel monitor state
///
/// This structure maintains the state for a single monitor session. The monitor
/// is designed to be lightweight and stateless between commands, storing only
/// the minimal information needed for interactive use.
///
/// # Thread Safety
///
/// Monitor instances are protected by the `MONITOR` mutex and should never be
/// accessed without acquiring the lock first. The serial driver may call into
/// the monitor from IRQ context.
pub struct Monitor {
    /// Current input line buffer
    ///
    /// Accumulates characters as the user types. The buffer has a maximum size
    /// of `MAX_LINE` (128) characters to prevent unbounded growth. When the user
    /// presses Enter, this buffer is parsed as a command and then cleared.
    line_buffer: String,

    /// Last memory address examined (for continuation)
    ///
    /// The `mem` command uses this to implement Wozmon-style continuation: if you
    /// examine memory at address X, then press Enter without specifying an address,
    /// the monitor continues from X+16. This allows quickly scanning through memory.
    last_addr: u64,

    /// Serial device class for communication
    ///
    /// Always set to `DeviceClass::Serial`. Used when calling the driver manager
    /// to output text. This allows the monitor to work with any serial driver
    /// registered under the Serial class.
    serial_class: DeviceClass,

    /// Tracks whether the banner/prompt has been shown
    ///
    /// The monitor lazily activates on first use. This flag prevents showing the
    /// welcome banner multiple times if the monitor is reinitialized.
    active: bool,
}

impl Monitor {
    /// Create a new monitor instance
    pub fn new() -> Self {
        Self {
            line_buffer: String::with_capacity(MAX_LINE),
            last_addr: 0,
            serial_class: DeviceClass::Serial,
            active: false,
        }
    }

    fn activate(&mut self) {
        if self.active {
            return;
        }
        self.writeln("\r\n");
        self.writeln("=================================================");
        self.writeln("  TheseusOS Kernel Monitor v0.1");
        self.writeln("  Inspired by Wozmon - Extended for modern debug");
        self.writeln("=================================================");
        self.writeln("");
        self.writeln("Type 'help' for available commands");
        self.write(PROMPT);
        self.active = true;
    }

    fn write_bytes(&self, bytes: &[u8]) {
        if serial::write_bytes_direct(bytes).is_err() {
            let _ = driver_manager()
                .lock()
                .write_class(self.serial_class, bytes);
        }
    }

    /// Write a string to the serial port
    fn write(&self, s: &str) {
        self.write_bytes(s.as_bytes());
    }

    /// Write a string followed by newline
    fn writeln(&self, s: &str) {
        self.write(s);
        self.write("\r\n");
    }

    /// Handle a single input character
    ///
    /// This is the core input processing function, called by the serial IRQ handler
    /// whenever a byte arrives. It implements a simple line editor with basic
    /// terminal control character support.
    ///
    /// # Character Handling
    ///
    /// - **Printable (0x20-0x7E)**: Added to line buffer and echoed back
    /// - **Enter (CR/LF)**: Process the current line as a command
    /// - **Backspace (0x08/0x7F)**: Remove last character with visual feedback
    /// - **Ctrl+C (0x03)**: Cancel current line
    /// - **Ctrl+L (0x0C)**: Clear screen using ANSI escape sequences
    /// - **Other control characters**: Silently ignored
    ///
    /// # Echo Behavior
    ///
    /// Most terminals expect immediate echo of typed characters. The monitor provides
    /// this by writing each printable character back as it's received. For special
    /// keys, visual feedback is provided (e.g., "^C" for Ctrl+C, or backspace sequence).
    fn handle_char(&mut self, ch: u8) {
        // Ensure monitor is activated (show banner on first input)
        self.activate();

        match ch {
            b'\r' | b'\n' => {
                // Enter key - process command
                // Echo newline for proper terminal formatting
                self.write_bytes(b"\r\n");

                if !self.line_buffer.is_empty() {
                    // Execute the command synchronously
                    self.process_command();
                    self.line_buffer.clear();
                }

                // Show prompt for next command
                self.write(PROMPT);
            }
            0x08 | 0x7F => {
                // Backspace (0x08) or DEL (0x7F) - delete last character
                if !self.line_buffer.is_empty() {
                    self.line_buffer.pop();
                    // Visual feedback: move cursor back, write space, move back again
                    // This erases the character on most terminals
                    self.write_bytes(b"\x08 \x08");
                }
            }
            0x03 => {
                // Ctrl+C - cancel current line and start fresh
                self.line_buffer.clear();
                self.write("^C"); // Show ^C to indicate cancellation
                self.write(PROMPT);
            }
            0x0C => {
                // Ctrl+L - clear screen
                // ANSI escape sequence: ESC[2J (clear screen) + ESC[H (cursor home)
                self.write_bytes(b"\x1B[2J\x1B[H");
                self.line_buffer.clear();
                self.write(PROMPT);
            }
            byte if byte >= 0x20 && byte < 0x7F => {
                // Printable ASCII character (space through tilde)
                if self.line_buffer.len() < MAX_LINE {
                    self.line_buffer.push(byte as char);
                    // Echo the character back for terminal display
                    self.write_bytes(&[byte]);
                }
                // Silently ignore if buffer is full (could add bell character here)
            }
            _ => {
                // Ignore other control characters (ESC sequences, arrow keys, etc.)
                // A more sophisticated implementation could handle arrow keys for
                // cursor movement and command history
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
            self.writeln(&format!(
                "  CS: 0x{:04X}  DS: 0x{:04X}  SS: 0x{:04X}  ES: 0x{:04X}",
                cs, ds, ss, es
            ));
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
            self.writeln(&format!(
                "Filled {} bytes at 0x{:016X} with 0x{:02X}",
                len, addr, value
            ));
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
            let bound = if dev.driver_data.is_some() {
                "[BOUND]"
            } else {
                ""
            };
            let addr_str = match dev.phys_addr {
                Some(a) => format!("0x{:X}", a),
                None => String::from("none"),
            };
            let irq_str = match dev.irq {
                Some(i) => format!("{}", i),
                None => String::from("none"),
            };

            self.writeln(&format!(
                "  [{:2}] {} {} addr:{} irq:{}",
                i, dev.id, bound, addr_str, irq_str
            ));
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

                self.writeln(&format!(
                    "  Frame #{:2}: RIP=0x{:016X} RBP=0x{:016X}",
                    frame, ret_addr, rbp
                ));

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
        self.writeln(&format!(
            "  IDTR Limit: 0x{:04X} ({} entries)",
            idtr.limit,
            (idtr.limit + 1) / 16
        ));

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
        self.writeln(&format!(
            "  GDTR Limit: 0x{:04X} ({} entries)",
            gdtr.limit,
            (gdtr.limit + 1) / 8
        ));
    }

    /// Display memory map
    fn cmd_memory_map(&self) {
        self.writeln("UEFI Memory Map:");

        let handoff = unsafe {
            &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff)
        };

        self.writeln(&format!(
            "  Descriptor Size: {} bytes",
            handoff.memory_map_descriptor_size
        ));
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
            if fi.has_fpu() {
                self.writeln("  - FPU (Floating Point Unit)");
            }
            if fi.has_pae() {
                self.writeln("  - PAE (Physical Address Extension)");
            }
            if fi.has_apic() {
                self.writeln("  - APIC (Advanced PIC)");
            }
            if fi.has_msr() {
                self.writeln("  - MSR (Model Specific Registers)");
            }
            if fi.has_tsc() {
                self.writeln("  - TSC (Time Stamp Counter)");
            }
            if fi.has_sse() {
                self.writeln("  - SSE");
            }
            if fi.has_sse2() {
                self.writeln("  - SSE2");
            }
            if fi.has_x2apic() {
                self.writeln("  - x2APIC");
            }
        }

        if let Some(ef) = cpuid.get_extended_feature_info() {
            if ef.has_fsgsbase() {
                self.writeln("  - FSGSBASE");
            }
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
            "r" | "read" => unsafe {
                let mut p: Port<u8> = Port::new(port);
                let value = p.read();
                self.writeln(&format!("IN 0x{:X} = 0x{:02X}", port, value));
            },
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
