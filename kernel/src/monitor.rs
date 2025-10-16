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

use crate::acpi;
use crate::config;
use crate::display::kernel_write_line;
use crate::drivers::manager::driver_manager;
use crate::drivers::serial;
use crate::drivers::traits::DeviceClass;
use crate::memory;
use crate::physical_memory;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::cmp::min;
use core::str;
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
            "phys" | "physmem" => self.cmd_phys(),
            "stack" | "bt" => self.cmd_stack_trace(),
            "idt" => self.cmd_idt(&parts[1..]),
            "gdt" => self.cmd_gdt(&parts[1..]),
            "mmap" => self.cmd_memory_map(&parts[1..]),
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
        self.writeln(
            "  mem [ADDR]             - Examine 16 bytes (continues from prior if omitted)",
        );
        self.writeln("  dump ADDR [LEN]        - Hex dump memory region");
        self.writeln("  write ADDR VAL         - Write byte to memory");
        self.writeln("  fill ADDR LEN VAL      - Fill memory region with byte");
        self.writeln("");
        self.writeln("System Information:");
        self.writeln("  regs                 - Display general/control/segment registers");
        self.writeln("  phys, physmem       - Persistent physical allocator summary");
        self.writeln("  devices, dev         - List registered hardware devices");
        self.writeln("  acpi                 - ACPI RSDP + platform summary");
        self.writeln("  mmap [summary|entries [N]|entry IDX]");
        self.writeln("                       - Inspect UEFI memory map");
        self.writeln("  cpuid                - CPU identification and feature flags");
        self.writeln("");
        self.writeln("System Tables:");
        self.writeln("  idt [N]              - Show first N IDT descriptors (default 16)");
        self.writeln("  gdt [N]              - Show first N GDT descriptors (default all)");
        self.writeln("");
        self.writeln("Debugging:");
        self.writeln("  stack, bt           - Stack backtrace");
        self.writeln("  msr [r|w] ADDR [VAL]- Read/write model-specific register");
        self.writeln("  io (r|w)[8|16|32] PORT [VAL]");
        self.writeln("                       - Read/write I/O port with width selection");
        self.writeln("  int NUM             - Trigger software interrupt (limited set)");
        self.writeln("  call ADDR           - Call function at address");
        self.writeln("");
        self.writeln("System Control:");
        self.writeln("  reset               - Reset system");
        self.writeln("  halt                - Halt CPU");
        self.writeln("  clear, cls          - Clear screen");
        self.writeln("");
        self.writeln("Numbers accept hex (0x...) or decimal input.");
    }

    fn cmd_phys(&self) {
        if let Some(stats) = physical_memory::stats() {
            let page_size = memory::PAGE_SIZE as u64;
            let total_bytes = stats.total_frames.saturating_mul(page_size);
            let free_bytes = stats.free_frames.saturating_mul(page_size);
            let guard = memory::runtime_kernel_lower_guard();
            self.writeln("Persistent Physical Memory Manager:");
            self.writeln(&format!(
                "  base PFN:    0x{:016X}",
                stats.base_pfn
            ));
            self.writeln(&format!(
                "  total frames: {} ({:#X} bytes)",
                stats.total_frames, total_bytes
            ));
            self.writeln(&format!(
                "  free frames:  {} ({:#X} bytes)",
                stats.free_frames, free_bytes
            ));
            self.writeln(&format!(
                "  guard window: {:#X} bytes below runtime base",
                guard
            ));
        } else {
            self.writeln("Persistent allocator not initialised yet");
        }
    }

    /// Display CPU registers
    fn cmd_registers(&self) {
        self.writeln("CPU Registers:");

        macro_rules! read_gpr {
            ($reg:tt) => {{
                let value: u64;
                unsafe {
                    core::arch::asm!(
                        concat!("mov {0}, ", stringify!($reg)),
                        out(reg) value,
                        options(nomem, preserves_flags, nostack)
                    );
                }
                value
            }};
        }

        let rax = read_gpr!(rax);
        let rbx = read_gpr!(rbx);
        let rcx = read_gpr!(rcx);
        let rdx = read_gpr!(rdx);
        let rsi = read_gpr!(rsi);
        let rdi = read_gpr!(rdi);
        let rsp = read_gpr!(rsp);
        let rbp = read_gpr!(rbp);
        let r8 = read_gpr!(r8);
        let r9 = read_gpr!(r9);
        let r10 = read_gpr!(r10);
        let r11 = read_gpr!(r11);
        let r12 = read_gpr!(r12);
        let r13 = read_gpr!(r13);
        let r14 = read_gpr!(r14);
        let r15 = read_gpr!(r15);

        let rip: u64;
        unsafe {
            core::arch::asm!(
                "lea {0}, [rip]",
                out(reg) rip,
                options(nomem, preserves_flags)
            );
        }

        self.writeln(&format!("  RAX: 0x{:016X}  RBX: 0x{:016X}", rax, rbx));
        self.writeln(&format!("  RCX: 0x{:016X}  RDX: 0x{:016X}", rcx, rdx));
        self.writeln(&format!("  RSI: 0x{:016X}  RDI: 0x{:016X}", rsi, rdi));
        self.writeln(&format!("  RSP: 0x{:016X}  RBP: 0x{:016X}", rsp, rbp));
        self.writeln(&format!("  R8 : 0x{:016X}  R9 : 0x{:016X}", r8, r9));
        self.writeln(&format!("  R10: 0x{:016X}  R11: 0x{:016X}", r10, r11));
        self.writeln(&format!("  R12: 0x{:016X}  R13: 0x{:016X}", r12, r13));
        self.writeln(&format!("  R14: 0x{:016X}  R15: 0x{:016X}", r14, r15));
        self.writeln(&format!("  RIP: 0x{:016X}", rip));

        // Control registers
        let cr0: u64;
        let cr2: u64;
        let cr3: u64;
        let cr4: u64;
        let cr8: u64;

        unsafe {
            core::arch::asm!("mov {}, cr0", out(reg) cr0, options(nomem, nostack));
            core::arch::asm!("mov {}, cr2", out(reg) cr2, options(nomem, nostack));
            core::arch::asm!("mov {}, cr3", out(reg) cr3, options(nomem, nostack));
            core::arch::asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack));
            core::arch::asm!("mov {}, cr8", out(reg) cr8, options(nomem, nostack));
        }

        self.writeln("");
        self.writeln("Control Registers:");
        self.writeln(&format!("  CR0: 0x{:016X}  CR2: 0x{:016X}", cr0, cr2));
        self.writeln(&format!("  CR3: 0x{:016X}  CR4: 0x{:016X}", cr3, cr4));
        self.writeln(&format!("  CR8: 0x{:016X}", cr8));

        // Segment registers
        let cs: u16;
        let ds: u16;
        let ss: u16;
        let es: u16;
        let fs: u16;
        let gs: u16;

        unsafe {
            core::arch::asm!("mov {:x}, cs", out(reg) cs, options(nomem, nostack));
            core::arch::asm!("mov {:x}, ds", out(reg) ds, options(nomem, nostack));
            core::arch::asm!("mov {:x}, ss", out(reg) ss, options(nomem, nostack));
            core::arch::asm!("mov {:x}, es", out(reg) es, options(nomem, nostack));
            core::arch::asm!("mov {:x}, fs", out(reg) fs, options(nomem, nostack));
            core::arch::asm!("mov {:x}, gs", out(reg) gs, options(nomem, nostack));
        }

        self.writeln("");
        self.writeln("Segment Registers:");
        self.writeln(&format!(
            "  CS: 0x{:04X}  DS: 0x{:04X}  SS: 0x{:04X}  ES: 0x{:04X}",
            cs, ds, ss, es
        ));
        self.writeln(&format!("  FS: 0x{:04X}  GS: 0x{:04X}", fs, gs));

        let rflags = x86_64::registers::rflags::read().bits();
        self.writeln("");
        self.writeln(&format!("RFLAGS: 0x{:016X}", rflags));
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
        let mgr = driver_manager().lock();
        let devices = mgr.devices();

        self.writeln(&format!("Registered devices ({}):", devices.len()));

        if devices.is_empty() {
            self.writeln("  (no devices registered)");
            return;
        }

        for (i, dev) in devices.iter().enumerate() {
            self.writeln(&format!(
                "  [{:02}] {:<24} class:{:?} status:{} phys:{} irq:{}",
                i,
                format!("{}", dev.id),
                dev.class,
                if dev.driver_data.is_some() {
                    "bound"
                } else {
                    "pending"
                },
                dev.phys_addr
                    .map(|addr| format!("0x{:X}", addr))
                    .unwrap_or_else(|| "none".into()),
                dev.irq
                    .map(|irq| format!("{}", irq))
                    .unwrap_or_else(|| "none".into())
            ));
            if let Some(state) = dev.driver_data {
                self.writeln(&format!("      driver_state=0x{:016X}", state as u64));
            }
        }
    }

    /// Display ACPI information
    fn cmd_acpi(&self) {
        self.writeln("ACPI Information:");

        let handoff = unsafe {
            &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff)
        };

        if handoff.acpi_rsdp == 0 {
            self.writeln("  RSDP: not present in handoff");
        } else {
            self.writeln(&format!(
                "  RSDP Physical Address: 0x{:016X}",
                handoff.acpi_rsdp
            ));

            match parse_rsdp_info(handoff.acpi_rsdp) {
                Ok(info) => {
                    let oem_str = match str::from_utf8(&info.oem_id) {
                        Ok(s) => s.trim_end_matches(char::from(0)),
                        Err(_) => "???",
                    };
                    self.writeln(&format!(
                        "  Signature: {}  Revision: {} ({})",
                        info.signature_string(),
                        info.revision,
                        info.revision_label()
                    ));
                    self.writeln(&format!("  OEM ID: {}", oem_str));
                    self.writeln(&format!("  RSDT Address: 0x{:016X}", info.rsdt_address));
                    if let Some(xsdt) = info.xsdt_address {
                        self.writeln(&format!("  XSDT Address: 0x{:016X}", xsdt));
                    }
                    self.writeln(&format!(
                        "  Checksum: {}",
                        if info.checksum_ok { "valid" } else { "INVALID" }
                    ));
                    if let Some(ok) = info.extended_checksum_ok {
                        self.writeln(&format!(
                            "  Extended Checksum: {}",
                            if ok { "valid" } else { "INVALID" }
                        ));
                    }
                }
                Err(err) => {
                    self.writeln(&format!("  RSDP decode failed: {}", err));
                }
            }
        }

        self.writeln("");
        self.writeln("Platform Summary:");
        if let Some(info) = acpi::cached_platform_info() {
            self.writeln(&format!("  CPUs reported: {}", info.cpu_count));
            self.writeln(&format!(
                "  IO APICs: {} (present: {})",
                info.io_apic_count,
                if info.has_io_apic { "yes" } else { "no" }
            ));
            self.writeln(&format!("  Local APIC: 0x{:016X}", info.local_apic_address));
            self.writeln(&format!(
                "  Legacy PIC present: {}",
                if info.has_legacy_pic { "yes" } else { "no" }
            ));

            if let Some(ref madt) = info.madt_info {
                let apic_ids: Vec<String> = madt
                    .cpu_apic_ids
                    .iter()
                    .map(|id| format!("0x{:02X}", id))
                    .collect();
                self.writeln("  MADT:");
                if apic_ids.is_empty() {
                    self.writeln("    CPU APIC IDs: (none)");
                } else {
                    self.writeln(&format!("    CPU APIC IDs: {}", apic_ids.join(", ")));
                }
                if madt.io_apics.is_empty() {
                    self.writeln("    IO APICs: (none)");
                } else {
                    for entry in &madt.io_apics {
                        self.writeln(&format!(
                            "    IO APIC id:{} addr:0x{:016X} gsi_base:{}",
                            entry.id, entry.address, entry.gsi_base
                        ));
                    }
                }
                self.writeln(&format!(
                    "    Has 8259 PIC: {}",
                    if madt.has_8259_pic { "yes" } else { "no" }
                ));
            } else {
                self.writeln("  MADT: not parsed (platform info missing details)");
            }
        } else {
            self.writeln("  (ACPI platform info cache is empty)");
            self.writeln("  Hint: ensure driver system initialization has completed.");
        }
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
    fn cmd_idt(&self, args: &[&str]) {
        self.writeln("Interrupt Descriptor Table:");

        use x86_64::instructions::tables::sidt;
        let idtr = sidt();
        let base = idtr.base.as_u64();
        let total_bytes = (idtr.limit as usize) + 1;
        let entry_size = core::mem::size_of::<RawIdtEntry>();
        let entry_count = total_bytes / entry_size;

        self.writeln(&format!("  IDTR Base:  0x{:016X}", base));
        self.writeln(&format!(
            "  IDTR Limit: 0x{:04X} ({} bytes, {} entries)",
            idtr.limit, total_bytes, entry_count
        ));

        if entry_count == 0 {
            self.writeln("  (IDT empty)");
            return;
        }

        let max_entries = args
            .get(0)
            .and_then(|s| parse_number(s))
            .map(|n| n as usize)
            .unwrap_or(16);

        let ptr = base as *const RawIdtEntry;
        let count = min(entry_count, max_entries);

        for idx in 0..count {
            let entry = unsafe { core::ptr::read_unaligned(ptr.add(idx)) };
            self.print_idt_entry(idx, &entry);
        }

        if entry_count > count {
            self.writeln(&format!("  ({} entries not shown)", entry_count - count));
        }
    }

    fn print_idt_entry(&self, index: usize, entry: &RawIdtEntry) {
        let entry = *entry;
        let empty = entry.offset_low == 0
            && entry.offset_mid == 0
            && entry.offset_high == 0
            && entry.type_attr == 0
            && entry.selector == 0;
        if empty {
            self.writeln(&format!("  [{:02}] <empty>", index));
            return;
        }

        let selector = entry.selector;
        let type_attr = entry.type_attr;
        let ist = entry.ist & 0x7;
        let offset = (entry.offset_low as u64)
            | ((entry.offset_mid as u64) << 16)
            | ((entry.offset_high as u64) << 32);
        let gate_type = type_attr & 0x0F;
        let present = (type_attr & 0x80) != 0;
        let dpl = (type_attr >> 5) & 0x3;

        self.writeln(&format!(
            "  [{:02}] selector=0x{:04X} offset=0x{:016X} type:{} dpl:{} ist:{} attr=0x{:02X} {}",
            index,
            selector,
            offset,
            describe_idt_gate(gate_type),
            dpl,
            ist,
            type_attr,
            if present { "present" } else { "absent" }
        ));
    }

    /// Display GDT information
    fn cmd_gdt(&self, args: &[&str]) {
        self.writeln("Global Descriptor Table:");

        use x86_64::instructions::tables::sgdt;
        let gdtr = sgdt();
        let base = gdtr.base.as_u64();
        let total_bytes = (gdtr.limit as usize) + 1;
        let mut entry_count = total_bytes / 8;
        if total_bytes % 8 != 0 {
            entry_count += 1;
        }

        self.writeln(&format!("  GDTR Base:  0x{:016X}", base));
        self.writeln(&format!(
            "  GDTR Limit: 0x{:04X} ({} bytes, {} entries)",
            gdtr.limit, total_bytes, entry_count
        ));

        if entry_count == 0 {
            self.writeln("  (GDT empty)");
            return;
        }

        let max_entries = args
            .get(0)
            .and_then(|s| parse_number(s))
            .map(|n| n as usize)
            .unwrap_or(entry_count);

        let ptr = base as *const u8;
        let mut idx = 0usize;
        let mut shown = 0usize;

        while idx < entry_count && shown < max_entries {
            let entry_ptr = unsafe { ptr.add(idx * 8) } as *const u64;
            let raw_low = unsafe { core::ptr::read_unaligned(entry_ptr) };
            let next = if idx + 1 < entry_count {
                Some(unsafe { core::ptr::read_unaligned(entry_ptr.add(1)) })
            } else {
                None
            };
            let consumed = self.print_gdt_entry(idx, raw_low, next);
            shown += 1;
            idx += if consumed { 2 } else { 1 };
        }

        if idx < entry_count {
            self.writeln(&format!("  ({} descriptors not shown)", entry_count - idx));
        }
    }

    fn print_gdt_entry(&self, index: usize, raw_low: u64, raw_next: Option<u64>) -> bool {
        let uses_second = gdt_entry_needs_extra(raw_low);
        let raw_high = if uses_second { raw_next } else { None };

        if raw_low == 0 && raw_high.unwrap_or(0) == 0 {
            self.writeln(&format!("  [{:02}] <null>", index));
            return uses_second;
        }

        let access = ((raw_low >> 40) & 0xFF) as u8;
        let s = (access & 0x10) != 0;
        let typ = access & 0x0F;
        let dpl = (access >> 5) & 0x03;
        let present = (access & 0x80) != 0;

        let limit_low = (raw_low & 0xFFFF) as u32;
        let limit_high = ((raw_low >> 48) & 0xF) as u32;
        let limit = (limit_low | (limit_high << 16)) as u32;

        let base_low = ((raw_low >> 16) & 0xFFFFFF) as u32;
        let base_high = ((raw_low >> 56) & 0xFF) as u32;
        let mut base = ((base_high as u64) << 24) | base_low as u64;

        let flags = ((raw_low >> 52) & 0xF) as u8;
        let avl = (flags & 0x1) != 0;
        let l = (flags & 0x2) != 0;
        let db = (flags & 0x4) != 0;
        let g = (flags & 0x8) != 0;

        if let Some(high) = raw_high {
            base |= ((high & 0xFFFF_FFFF) as u64) << 32;
        }

        let limit_bytes = if g {
            ((limit as u64) << 12) | 0xFFF
        } else {
            limit as u64
        };

        let mut flag_parts = Vec::new();
        if g {
            flag_parts.push("G");
        }
        if l {
            flag_parts.push("L");
        }
        if db {
            flag_parts.push("DB");
        }
        if avl {
            flag_parts.push("AVL");
        }

        let type_desc = describe_segment_type(s, typ);

        self.writeln(&format!(
            "  [{:02}] base=0x{:016X} limit=0x{:016X} type:{} dpl:{} attr=0x{:02X} {} flags:{}",
            index,
            base,
            limit_bytes,
            type_desc,
            dpl,
            access,
            if present { "present" } else { "not-present" },
            if flag_parts.is_empty() {
                "-".into()
            } else {
                flag_parts.join("|")
            }
        ));

        uses_second
    }

    /// Display memory map
    fn cmd_memory_map(&self, args: &[&str]) {
        self.writeln("UEFI Memory Map:");

        let handoff = unsafe {
            &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff)
        };

        let desc_size = handoff.memory_map_descriptor_size as usize;
        let entry_count = handoff.memory_map_entries as usize;
        let buffer_ptr = handoff.memory_map_buffer_ptr;

        if buffer_ptr == 0 || entry_count == 0 || desc_size == 0 {
            self.writeln("  (memory map not available in handoff)");
            return;
        }

        if desc_size < 32 {
            self.writeln(&format!(
                "  Descriptor size {} is unexpectedly small (< 32)",
                desc_size
            ));
            return;
        }

        let buffer = buffer_ptr as *const u8;
        let mut total_pages: u128 = 0;
        let mut type_totals: Vec<TypeSummary> = Vec::new();
        let mut descriptors: Vec<UefiMemoryDescriptor> = Vec::with_capacity(entry_count);

        for idx in 0..entry_count {
            let desc = unsafe { read_uefi_descriptor(buffer, desc_size, idx) };
            total_pages = total_pages.wrapping_add(desc.num_pages as u128);

            if let Some(entry) = type_totals.iter_mut().find(|e| e.typ == desc.typ) {
                entry.entries += 1;
                entry.pages = entry.pages.wrapping_add(desc.num_pages);
            } else {
                type_totals.push(TypeSummary {
                    typ: desc.typ,
                    entries: 1,
                    pages: desc.num_pages,
                });
            }

            descriptors.push(desc);
        }

        type_totals.sort_by(|a, b| b.pages.cmp(&a.pages));

        let total_bytes = total_pages.saturating_mul(EFI_PAGE_SIZE as u128);

        enum MemoryMapMode {
            SummaryOnly,
            SummaryAndSample { sample: usize },
            Entries { limit: Option<usize> },
            Single { index: usize },
        }

        let mode = if let Some(first) = args.get(0) {
            match *first {
                "summary" => MemoryMapMode::SummaryOnly,
                "entries" | "all" => {
                    let limit = args
                        .get(1)
                        .and_then(|s| parse_number(s))
                        .map(|n| n as usize);
                    MemoryMapMode::Entries { limit }
                }
                "entry" => {
                    if args.len() < 2 {
                        self.writeln("Usage: mmap entry INDEX");
                        return;
                    }
                    match parse_number(args[1]) {
                        Some(idx) => MemoryMapMode::Single {
                            index: idx as usize,
                        },
                        None => {
                            self.writeln("Invalid index");
                            return;
                        }
                    }
                }
                other => match parse_number(other) {
                    Some(idx) => MemoryMapMode::Single {
                        index: idx as usize,
                    },
                    None => {
                        self.writeln("Usage: mmap [summary | entries [N] | entry INDEX]");
                        return;
                    }
                },
            }
        } else {
            MemoryMapMode::SummaryAndSample { sample: 8 }
        };

        self.writeln(&format!(
            "  Entries: {}  Descriptor Size: {} bytes",
            entry_count, desc_size
        ));
        self.writeln(&format!("  Total reported: {}", format_bytes(total_bytes)));
        self.writeln("  Per-type summary:");
        for summary in &type_totals {
            let bytes = (summary.pages as u128).saturating_mul(EFI_PAGE_SIZE as u128);
            self.writeln(&format!(
                "    {:>2} {:<20} entries:{:>2} pages:{:>8} size:{}",
                summary.typ,
                uefi_memory_type_to_str(summary.typ),
                summary.entries,
                summary.pages,
                format_bytes(bytes)
            ));
        }

        match mode {
            MemoryMapMode::SummaryOnly => {}
            MemoryMapMode::SummaryAndSample { sample } => {
                self.writeln("");
                let count = min(sample, descriptors.len());
                self.writeln(&format!("  First {} descriptors:", count));
                for (idx, desc) in descriptors.iter().take(count).enumerate() {
                    self.print_memory_descriptor(idx, desc);
                }
                if descriptors.len() > count {
                    self.writeln(&format!(
                        "  ({} additional descriptors hidden; use 'mmap entries' to show all)",
                        descriptors.len() - count
                    ));
                }
            }
            MemoryMapMode::Entries { limit } => {
                self.writeln("");
                let max = limit.unwrap_or(descriptors.len());
                let count = min(max, descriptors.len());
                self.writeln(&format!("  Listing {} descriptors:", count));
                for (idx, desc) in descriptors.iter().take(count).enumerate() {
                    self.print_memory_descriptor(idx, desc);
                }
                if descriptors.len() > count {
                    self.writeln(&format!(
                        "  ({} additional descriptors not shown)",
                        descriptors.len() - count
                    ));
                }
            }
            MemoryMapMode::Single { index } => {
                self.writeln("");
                if let Some(desc) = descriptors.get(index) {
                    self.writeln(&format!("  Descriptor {}:", index));
                    self.print_memory_descriptor(index, desc);
                } else {
                    self.writeln("  Descriptor index out of range");
                }
            }
        }
    }

    fn print_memory_descriptor(&self, index: usize, desc: &UefiMemoryDescriptor) {
        let size_bytes = (desc.num_pages as u128).saturating_mul(EFI_PAGE_SIZE as u128);
        let phys_end = if desc.num_pages == 0 {
            desc.phys_start
        } else {
            desc.phys_start
                .saturating_add(desc.num_pages.saturating_mul(EFI_PAGE_SIZE) - 1)
        };
        let attrs = describe_memory_attributes(desc.attributes);
        self.writeln(&format!(
            "  [{:03}] {:<20} phys:0x{:016X}-0x{:016X} pages:{:>6} size:{} attrs:{} (0x{:016X})",
            index,
            uefi_memory_type_to_str(desc.typ),
            desc.phys_start,
            phys_end,
            desc.num_pages,
            format_bytes(size_bytes),
            attrs,
            desc.attributes
        ));
        if desc.virt_start != 0 {
            self.writeln(&format!("        virt:0x{:016X}", desc.virt_start));
        }
    }

    /// Display CPUID information
    fn cmd_cpuid(&self) {
        self.writeln("CPUID Information:");

        use raw_cpuid::CpuId;
        let cpuid = CpuId::new();

        if let Some(vi) = cpuid.get_vendor_info() {
            self.writeln(&format!("  Vendor: {}", vi.as_str()));
        }

        if let Some(brand) = cpuid.get_processor_brand_string() {
            let text = brand.as_str().trim_end_matches(char::from(0));
            if !text.is_empty() {
                self.writeln(&format!("  Brand: {}", text));
            }
        }

        if let Some(fi) = cpuid.get_feature_info() {
            self.writeln(&format!(
                "  Family: 0x{:X}  Model: 0x{:X}  Stepping: 0x{:X}",
                fi.family_id(),
                fi.model_id(),
                fi.stepping_id()
            ));
            self.writeln(&format!(
                "  APIC ID: {}  Logical CPUs/package: {}",
                fi.initial_local_apic_id(),
                fi.max_logical_processor_ids()
            ));
            self.writeln(&format!(
                "  CLFLUSH line size: {} bytes",
                fi.cflush_cache_line_size() * 8
            ));

            let mut standard = Vec::new();
            if fi.has_fpu() {
                standard.push("FPU");
            }
            if fi.has_vme() {
                standard.push("VME");
            }
            if fi.has_de() {
                standard.push("DE");
            }
            if fi.has_pse() {
                standard.push("PSE");
            }
            if fi.has_tsc() {
                standard.push("TSC");
            }
            if fi.has_msr() {
                standard.push("MSR");
            }
            if fi.has_pae() {
                standard.push("PAE");
            }
            if fi.has_mce() {
                standard.push("MCE");
            }
            if fi.has_cmpxchg8b() {
                standard.push("CMPXCHG8B");
            }
            if fi.has_apic() {
                standard.push("APIC");
            }
            if fi.has_sysenter_sysexit() {
                standard.push("SYSENTER/SYSEXIT");
            }
            if fi.has_mtrr() {
                standard.push("MTRR");
            }
            if fi.has_pge() {
                standard.push("PGE");
            }
            if fi.has_mca() {
                standard.push("MCA");
            }
            if fi.has_cmov() {
                standard.push("CMOV");
            }
            if fi.has_pat() {
                standard.push("PAT");
            }
            if fi.has_pse36() {
                standard.push("PSE36");
            }
            if fi.has_clflush() {
                standard.push("CLFLUSH");
            }
            if fi.has_mmx() {
                standard.push("MMX");
            }
            if fi.has_fxsave_fxstor() {
                standard.push("FXSAVE/FXRSTOR");
            }
            if fi.has_sse() {
                standard.push("SSE");
            }
            if fi.has_sse2() {
                standard.push("SSE2");
            }
            if fi.has_sse3() {
                standard.push("SSE3");
            }
            if fi.has_ssse3() {
                standard.push("SSSE3");
            }
            if fi.has_sse41() {
                standard.push("SSE4.1");
            }
            if fi.has_sse42() {
                standard.push("SSE4.2");
            }
            if fi.has_avx() {
                standard.push("AVX");
            }
            if fi.has_x2apic() {
                standard.push("x2APIC");
            }
            if fi.has_hypervisor() {
                standard.push("HYPERVISOR");
            }

            if standard.is_empty() {
                self.writeln("  Standard Features: (none)");
            } else {
                self.writeln(&format!("  Standard Features: {}", standard.join(", ")));
            }
        }

        if let Some(ef) = cpuid.get_extended_feature_info() {
            let mut features = Vec::new();
            if ef.has_fsgsbase() {
                features.push("FSGSBASE");
            }
            if ef.has_bmi1() {
                features.push("BMI1");
            }
            if ef.has_bmi2() {
                features.push("BMI2");
            }
            if ef.has_avx2() {
                features.push("AVX2");
            }
            if ef.has_smap() {
                features.push("SMAP");
            }
            if ef.has_smep() {
                features.push("SMEP");
            }
            if ef.has_rep_movsb_stosb() {
                features.push("REP MOVSB/STOSB");
            }
            if ef.has_invpcid() {
                features.push("INVPCID");
            }
            if ef.has_rdseed() {
                features.push("RDSEED");
            }
            if ef.has_rtm() {
                features.push("RTM");
            }
            if !features.is_empty() {
                self.writeln(&format!(
                    "  Extended Features (leaf 7): {}",
                    features.join(", ")
                ));
            }
        }

        if let Some(ext) = cpuid.get_extended_processor_and_feature_identifiers() {
            let mut features = Vec::new();
            if ext.has_64bit_mode() {
                features.push("LM");
            }
            if ext.has_execute_disable() {
                features.push("NX");
            }
            if ext.has_1gib_pages() {
                features.push("1GiB pages");
            }
            if ext.has_rdtscp() {
                features.push("RDTSCP");
            }
            if ext.has_sse4a() {
                features.push("SSE4A");
            }
            if ext.has_prefetchw() {
                features.push("PREFETCHW");
            }
            if ext.has_lahf_sahf() {
                features.push("LAHF/SAHF");
            }
            if ext.has_syscall_sysret() {
                features.push("SYSCALL/SYSRET");
            }
            if ext.has_lzcnt() {
                features.push("LZCNT");
            }
            if ext.has_mmx_extensions() {
                features.push("MMXEXT");
            }
            if !features.is_empty() {
                self.writeln(&format!(
                    "  Extended Features (leaf 0x80000001): {}",
                    features.join(", ")
                ));
            }
        }
    }

    /// Read or write a model-specific register
    fn cmd_msr(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: msr [r|w] ADDRESS [VALUE]");
            self.writeln("  r ADDRESS          - Read MSR (default operation)");
            self.writeln("  w ADDRESS VALUE    - Write MSR");
            self.writeln("Examples:");
            self.writeln("  msr 0x1B                 # read IA32_APIC_BASE");
            self.writeln("  msr w 0x1B 0x00000000    # write new value");
            return;
        }

        enum MsrOp {
            Read(u32),
            Write(u32, u64),
        }

        let op = match args[0] {
            "r" | "read" => {
                if args.len() < 2 {
                    self.writeln("Missing MSR address");
                    return;
                }
                match parse_number(args[1]) {
                    Some(addr) => MsrOp::Read(addr as u32),
                    None => {
                        self.writeln("Invalid MSR address");
                        return;
                    }
                }
            }
            "w" | "write" => {
                if args.len() < 3 {
                    self.writeln("Usage: msr w ADDRESS VALUE");
                    return;
                }
                let addr = match parse_number(args[1]) {
                    Some(addr) => addr as u32,
                    None => {
                        self.writeln("Invalid MSR address");
                        return;
                    }
                };
                let value = match parse_number(args[2]) {
                    Some(val) => val,
                    None => {
                        self.writeln("Invalid MSR value");
                        return;
                    }
                };
                MsrOp::Write(addr, value)
            }
            _ => match parse_number(args[0]) {
                Some(addr) => MsrOp::Read(addr as u32),
                None => {
                    self.writeln("Invalid MSR address or operation");
                    return;
                }
            },
        };

        unsafe {
            match op {
                MsrOp::Read(addr) => {
                    let value = x86_64::registers::model_specific::Msr::new(addr).read();
                    self.writeln(&format!("MSR 0x{:X} = 0x{:016X}", addr, value));
                }
                MsrOp::Write(addr, value) => {
                    x86_64::registers::model_specific::Msr::new(addr).write(value);
                    self.writeln(&format!("MSR 0x{:X} <- 0x{:016X}", addr, value));
                }
            }
        }
    }

    /// I/O port access
    fn cmd_io(&self, args: &[&str]) {
        if args.len() < 2 {
            self.writeln("Usage: io (r|w)[8|16|32] PORT [VALUE]");
            self.writeln("  io r0x10C         # read byte (default width 8)");
            self.writeln("  io r16 0x64       # read 16-bit value");
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
                            self.writeln(&format!("OUT8 0x{:04X} <- 0x{:02X}", port, value));
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

const EFI_PAGE_SIZE: u64 = 4096;

#[derive(Copy, Clone)]
struct UefiMemoryDescriptor {
    typ: u32,
    phys_start: u64,
    virt_start: u64,
    num_pages: u64,
    attributes: u64,
}

#[derive(Clone)]
struct TypeSummary {
    typ: u32,
    entries: usize,
    pages: u64,
}

unsafe fn read_uefi_descriptor(
    buffer: *const u8,
    desc_size: usize,
    index: usize,
) -> UefiMemoryDescriptor {
    let ptr = buffer.add(index * desc_size);
    let typ = core::ptr::read_unaligned(ptr as *const u32);
    let phys_start = if desc_size >= 16 {
        core::ptr::read_unaligned(ptr.add(8) as *const u64)
    } else {
        0
    };
    let virt_start = if desc_size >= 24 {
        core::ptr::read_unaligned(ptr.add(16) as *const u64)
    } else {
        0
    };
    let num_pages = if desc_size >= 32 {
        core::ptr::read_unaligned(ptr.add(24) as *const u64)
    } else {
        0
    };
    let attributes = if desc_size >= 40 {
        core::ptr::read_unaligned(ptr.add(32) as *const u64)
    } else {
        0
    };
    UefiMemoryDescriptor {
        typ,
        phys_start,
        virt_start,
        num_pages,
        attributes,
    }
}

fn uefi_memory_type_to_str(typ: u32) -> &'static str {
    match typ {
        0 => "Reserved",
        1 => "LoaderCode",
        2 => "LoaderData",
        3 => "BootServicesCode",
        4 => "BootServicesData",
        5 => "RuntimeServicesCode",
        6 => "RuntimeServicesData",
        7 => "ConventionalMemory",
        8 => "UnusableMemory",
        9 => "ACPIReclaim",
        10 => "ACPINVS",
        11 => "MMIO",
        12 => "MMIOPort",
        13 => "PalCode",
        14 => "PersistentMemory",
        15 => "Unaccepted",
        _ => "Unknown",
    }
}

fn describe_memory_attributes(attrs: u64) -> String {
    const EFI_MEMORY_UC: u64 = 0x0000_0000_0000_0001;
    const EFI_MEMORY_WC: u64 = 0x0000_0000_0000_0002;
    const EFI_MEMORY_WT: u64 = 0x0000_0000_0000_0004;
    const EFI_MEMORY_WP: u64 = 0x0000_0000_0000_0008;
    const EFI_MEMORY_WB: u64 = 0x0000_0000_0000_0010;
    const EFI_MEMORY_UCE: u64 = 0x0000_0000_0000_0020;
    const EFI_MEMORY_RUNTIME: u64 = 0x8000_0000_0000_0000;
    const EFI_MEMORY_XP: u64 = 0x1000_0000_0000_0000;

    let mut flags = Vec::new();
    if attrs & EFI_MEMORY_UC != 0 {
        flags.push("UC");
    }
    if attrs & EFI_MEMORY_WC != 0 {
        flags.push("WC");
    }
    if attrs & EFI_MEMORY_WT != 0 {
        flags.push("WT");
    }
    if attrs & EFI_MEMORY_WP != 0 {
        flags.push("WP");
    }
    if attrs & EFI_MEMORY_WB != 0 {
        flags.push("WB");
    }
    if attrs & EFI_MEMORY_UCE != 0 {
        flags.push("UCE");
    }
    if attrs & EFI_MEMORY_XP != 0 {
        flags.push("XP");
    }
    if attrs & EFI_MEMORY_RUNTIME != 0 {
        flags.push("RT");
    }

    if flags.is_empty() {
        "-".into()
    } else {
        flags.join("|")
    }
}

fn format_bytes(value: u128) -> String {
    const UNITS: [&str; 6] = ["B", "KiB", "MiB", "GiB", "TiB", "PiB"];
    let mut scaled = value;
    let mut unit = 0usize;
    while scaled >= 1024 && unit < UNITS.len() - 1 {
        scaled /= 1024;
        unit += 1;
    }
    if unit == 0 {
        format!("{} {}", scaled, UNITS[unit])
    } else {
        format!("{} bytes (~{} {})", value, scaled, UNITS[unit])
    }
}

struct RsdpInfo {
    signature: [u8; 8],
    oem_id: [u8; 6],
    revision: u8,
    rsdt_address: u64,
    xsdt_address: Option<u64>,
    checksum_ok: bool,
    extended_checksum_ok: Option<bool>,
}

impl RsdpInfo {
    fn signature_string(&self) -> String {
        self.signature.iter().map(|&b| b as char).collect()
    }

    fn revision_label(&self) -> &'static str {
        match self.revision {
            0 | 1 => "ACPI 1.0",
            2 | 3 => "ACPI 2.0+",
            _ => "ACPI",
        }
    }
}

fn parse_rsdp_info(rsdp_phys: u64) -> Result<RsdpInfo, &'static str> {
    if rsdp_phys == 0 {
        return Err("RSDP address is zero");
    }
    if !memory::phys_offset_is_active() {
        return Err("PHYS_OFFSET mapping inactive; RSDP inaccessible");
    }

    let rsdp_va = memory::phys_to_virt_pa(rsdp_phys);
    let ptr = rsdp_va as *const u8;

    unsafe {
        let mut signature = [0u8; 8];
        for i in 0..8 {
            signature[i] = core::ptr::read_volatile(ptr.add(i));
        }
        if &signature != b"RSD PTR " {
            return Err("Invalid RSDP signature");
        }

        let mut oem_id = [0u8; 6];
        for i in 0..6 {
            oem_id[i] = core::ptr::read_volatile(ptr.add(9 + i));
        }
        let revision = core::ptr::read_volatile(ptr.add(15));
        let rsdt_address = core::ptr::read_unaligned(ptr.add(16) as *const u32) as u64;

        let mut length = if revision >= 2 {
            core::ptr::read_unaligned(ptr.add(20) as *const u32)
        } else {
            20
        };
        if length < 20 {
            length = 20;
        }
        if revision >= 2 && length < 36 {
            length = 36;
        }

        let xsdt_address = if revision >= 2 {
            Some(core::ptr::read_unaligned(ptr.add(24) as *const u64))
        } else {
            None
        };

        let checksum_ok = verify_checksum(ptr, 20);
        let extended_checksum_ok = if revision >= 2 {
            Some(verify_checksum(ptr, length as usize))
        } else {
            None
        };

        Ok(RsdpInfo {
            signature,
            oem_id,
            revision,
            rsdt_address,
            xsdt_address,
            checksum_ok,
            extended_checksum_ok,
        })
    }
}

fn verify_checksum(ptr: *const u8, len: usize) -> bool {
    let mut sum: u8 = 0;
    for i in 0..len {
        unsafe {
            sum = sum.wrapping_add(core::ptr::read_volatile(ptr.add(i)));
        }
    }
    sum == 0
}

fn gdt_entry_needs_extra(raw_low: u64) -> bool {
    let access = ((raw_low >> 40) & 0xFF) as u8;
    let is_code_or_data = (access & 0x10) != 0;
    if is_code_or_data {
        false
    } else {
        match access & 0x0F {
            0x2 | 0x9 | 0xB | 0xC | 0xE | 0xF => true,
            _ => false,
        }
    }
}

fn describe_segment_type(is_code_or_data: bool, typ: u8) -> String {
    if is_code_or_data {
        let is_code = (typ & 0x8) != 0;
        if is_code {
            let readable = (typ & 0x2) != 0;
            let conforming = (typ & 0x4) != 0;
            format!(
                "Code{}{}",
                if readable { "+R" } else { "" },
                if conforming { " (conf)" } else { "" }
            )
        } else {
            let writable = (typ & 0x2) != 0;
            let expand_down = (typ & 0x4) != 0;
            format!(
                "Data{}{}",
                if writable { "+W" } else { "" },
                if expand_down { " (exp-down)" } else { "" }
            )
        }
    } else {
        match typ {
            0x2 => "LDT".into(),
            0x9 => "TSS (available)".into(),
            0xB => "TSS (busy)".into(),
            0xC => "Call Gate".into(),
            0xE => "Interrupt Gate".into(),
            0xF => "Trap Gate".into(),
            _ => "System".into(),
        }
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone)]
struct RawIdtEntry {
    offset_low: u16,
    selector: u16,
    ist: u8,
    type_attr: u8,
    offset_mid: u16,
    offset_high: u32,
    zero: u32,
}

fn describe_idt_gate(typ: u8) -> &'static str {
    match typ {
        0x5 => "Task Gate",
        0x6 => "16-bit Interrupt",
        0x7 => "16-bit Trap",
        0xE => "Interrupt Gate",
        0xF => "Trap Gate",
        _ => "Reserved",
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
