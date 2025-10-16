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
//! ## Module Organization
//!
//! The monitor is organized into focused submodules:
//! - `commands/`: All interactive commands, organized by category
//! - `parsing`: Number parsing utilities

mod commands;
mod parsing;

use crate::config;
use crate::display::kernel_write_line;
use crate::drivers::manager::driver_manager;
use crate::drivers::serial;
use crate::drivers::traits::DeviceClass;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use spin::Mutex;

/// Monitor prompt string
const PROMPT: &str = "\r\n> ";

/// Maximum line buffer size
const MAX_LINE: usize = 128;

/// Shared monitor instance guarded by a spin mutex.
static MONITOR: Mutex<Option<Monitor>> = Mutex::new(None);

/// Prepare the monitor subsystem
///
/// Initializes the kernel monitor if ENABLE_KERNEL_MONITOR config is true.
/// Safe to call multiple times - will reactivate existing monitor.
///
/// # Note
/// This function checks the config flag and returns early if the monitor
/// is disabled, making it safe to call unconditionally during boot.
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

/// Process a byte received from the serial device
///
/// Entry point for serial interrupt handler to feed data to the monitor.
/// This function is called from IRQ context when a byte arrives on COM1.
///
/// # Arguments
/// * `byte` - Received byte from serial port
pub fn notify_serial_byte(byte: u8) {
    push_serial_byte(byte);
}

/// Push a byte to the monitor for processing
///
/// Internal function that handles monitor initialization and byte processing.
/// Creates monitor on first byte if needed, then feeds the byte to the
/// input handler.
///
/// # Arguments
/// * `byte` - Byte to process (from serial port)
///
/// # Note
/// Safe to call from IRQ context - uses spin mutex for synchronization.
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

/// Start the monitor loop
///
/// Parks the CPU in an infinite loop waiting for serial input.
/// Used when you want the system to drop into interactive monitor mode
/// instead of continuing normal execution.
///
/// # Note
/// This function never returns. The CPU will hlt waiting for interrupts
/// (serial IRQ will wake it up to process commands).
pub fn start_monitor() -> ! {
    init();
    loop {
        x86_64::instructions::hlt();
    }
}

/// Kernel monitor state
///
/// This structure maintains the state for a single monitor session.
pub(crate) struct Monitor {
    /// Current input line buffer
    line_buffer: String,
    /// Last memory address examined (for continuation)
    pub(crate) last_addr: u64,
    /// Serial device class for communication
    serial_class: DeviceClass,
    /// Tracks whether the banner/prompt has been shown
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

    pub(crate) fn write_bytes(&self, bytes: &[u8]) {
        if serial::write_bytes_direct(bytes).is_err() {
            let _ = driver_manager()
                .lock()
                .write_class(self.serial_class, bytes);
        }
    }

    /// Write a string to the serial port
    pub(crate) fn write(&self, s: &str) {
        self.write_bytes(s.as_bytes());
    }

    /// Write a string followed by newline
    pub(crate) fn writeln(&self, s: &str) {
        self.write(s);
        self.write("\r\n");
    }

    /// Handle incoming character from serial port
    ///
    /// Implements a simple line editor with basic terminal control:
    /// - Printable chars: Added to buffer and echoed
    /// - Enter: Process command and clear buffer
    /// - Backspace/DEL: Remove last character
    /// - Ctrl+C: Cancel current line
    /// - Ctrl+L: Clear screen
    fn handle_char(&mut self, ch: u8) {
        self.activate();

        match ch {
            b'\r' | b'\n' => {
                // Enter key - process the command
                self.write_bytes(b"\r\n");
                if !self.line_buffer.is_empty() {
                    self.process_command();
                    self.line_buffer.clear();
                }
                self.write(PROMPT);
            }
            0x08 | 0x7F => {
                // Backspace (0x08) or DEL (0x7F)
                if !self.line_buffer.is_empty() {
                    self.line_buffer.pop();
                    // Visual feedback: backspace, space, backspace (erases character)
                    self.write_bytes(b"\x08 \x08");
                }
            }
            0x03 => {
                // Ctrl+C - cancel current line
                self.line_buffer.clear();
                self.write("^C");  // Show cancellation indicator
                self.write(PROMPT);
            }
            0x0C => {
                // Ctrl+L - clear screen (ANSI sequences)
                self.write_bytes(b"\x1B[2J\x1B[H");
                self.line_buffer.clear();
                self.write(PROMPT);
            }
            byte if byte >= 0x20 && byte < 0x7F => {
                // Printable ASCII (space through tilde)
                if self.line_buffer.len() < MAX_LINE {
                    self.line_buffer.push(byte as char);
                    self.write_bytes(&[byte]);  // Echo character
                }
            }
            _ => {
                // Ignore other control characters
            }
        }
    }

    /// Parse and execute a command line
    ///
    /// Splits the line into whitespace-separated parts and dispatches
    /// to the appropriate command handler based on the first word.
    fn process_command(&mut self) {
        // Clone to avoid borrow conflicts during command execution
        let line = self.line_buffer.clone();
        let parts: Vec<&str> = line.trim().split_whitespace().collect();

        if parts.is_empty() {
            return;
        }

        // Command dispatch table
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

    fn cmd_help(&self) {
        self.writeln("Available commands:");
        self.writeln("");
        self.writeln("Memory Operations:");
        self.writeln("  mem|m ADDR          - Examine memory (16 bytes, hex+ASCII)");
        self.writeln("  dump|d ADDR [LEN]   - Dump memory region (default: 256 bytes)");
        self.writeln("  write|w ADDR VALUE  - Write byte to memory");
        self.writeln("  fill|f ADDR LEN VAL - Fill memory region with value");
        self.writeln("");
        self.writeln("System Inspection:");
        self.writeln("  regs|r              - Display CPU registers");
        self.writeln("  stack|bt            - Display stack backtrace");
        self.writeln("  acpi                - Display ACPI information");
        self.writeln("  phys|physmem        - Display physical memory statistics");
        self.writeln("  devices|dev         - List registered devices");
        self.writeln("");
        self.writeln("Tables & Maps:");
        self.writeln("  idt [VECTOR]        - Display IDT information");
        self.writeln("  gdt [INDEX]         - Display GDT information");
        self.writeln("  mmap                - Display UEFI memory map");
        self.writeln("");
        self.writeln("CPU:");
        self.writeln("  cpuid               - Display CPUID information");
        self.writeln("  msr MSR             - Read Model-Specific Register");
        self.writeln("");
        self.writeln("I/O:");
        self.writeln("  io PORT [VALUE]     - Read/write I/O port");
        self.writeln("  int VECTOR          - Trigger software interrupt");
        self.writeln("");
        self.writeln("System Control:");
        self.writeln("  reset               - Reboot system");
        self.writeln("  halt                - Halt CPU");
        self.writeln("  clear|cls           - Clear screen");
        self.writeln("  help|?              - Show this help");
    }
}


