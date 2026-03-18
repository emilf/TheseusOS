//! Module: monitor
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//!
//! INVARIANTS:
//! - The monitor is the interactive runtime inspection surface layered on top of serial input and kernel command handlers.
//! - Serial IRQ paths enqueue input cheaply; heavier command processing runs later in thread/context-safe monitor code.
//! - The monitor may inspect devices, memory, tables, ACPI, PCI, and USB state, but those views are observational helpers rather than separate sources of architectural truth.
//!
//! SAFETY:
//! - Monitor commands that inspect or mutate low-level state must treat user input as privileged debugging input, not as validated protocol traffic.
//! - IRQ-fed serial input paths must keep synchronization simple and bounded so they do not turn observability into a source of reentrancy bugs.
//! - Commands that expose raw memory, MMIO, MSRs, or arbitrary function calls remain dangerous by design and must not be mistaken for safe production interfaces.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//!
//! Interactive kernel monitor and serial-debug console.
//!
//! This module provides the current runtime inspection shell layered on top of
//! serial input, command parsing, and monitor command handlers.

mod commands;
mod parsing;

use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, Ordering};
use spin::Mutex;
use x86_64::instructions::interrupts;

use crate::config;
use crate::drivers::manager::driver_manager;
use crate::drivers::serial;
use crate::drivers::traits::DeviceClass;
use crate::input::keyboard::{pop_event as keyboard_pop_event, KeyTransition};

use crate::log_debug;

/// Monitor prompt string
const PROMPT: &str = "\r\n> ";

/// Maximum line buffer size
const MAX_LINE: usize = 128;

/// Shared monitor instance guarded by a spin mutex.
static MONITOR: Mutex<Option<Monitor>> = Mutex::new(None);
static SERIAL_INPUT_QUEUE: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());
static SERIAL_PENDING: AtomicBool = AtomicBool::new(false);

/// Initialise or reactivate the shared monitor instance when enabled.
pub fn init() {
    if !config::ENABLE_KERNEL_MONITOR {
        return;
    }

    let mut guard = MONITOR.lock();
    if guard.is_none() {
        let mut monitor = Monitor::new();
        monitor.activate();
        *guard = Some(monitor);
        log_debug!("Monitor interactive console ready");
    } else if let Some(monitor) = guard.as_mut() {
        monitor.activate();
    }
}

/// IRQ-facing entry point for serial input.
pub fn notify_serial_byte(byte: u8) {
    push_serial_byte(byte);
}

/// Queue one byte for deferred monitor processing.
pub fn push_serial_byte(byte: u8) {
    if !config::ENABLE_KERNEL_MONITOR {
        return;
    }

    {
        let mut queue = SERIAL_INPUT_QUEUE.lock();
        queue.push_back(byte);
    }
    SERIAL_PENDING.store(true, Ordering::Release);
}

pub fn process_pending_serial() {
    if !config::ENABLE_KERNEL_MONITOR {
        return;
    }
    if !SERIAL_PENDING.swap(false, Ordering::AcqRel) {
        return;
    }

    let drained = {
        let mut queue = SERIAL_INPUT_QUEUE.lock();
        let mut items = Vec::with_capacity(queue.len());
        while let Some(byte) = queue.pop_front() {
            items.push(byte);
        }
        items
    };

    if drained.is_empty() {
        return;
    }

    with_monitor(move |monitor| {
        for byte in drained {
            monitor.handle_char(byte);
        }
    });
}

fn with_monitor<F>(f: F)
where
    F: FnOnce(&mut Monitor),
{
    let _guard = InterruptGuard::new();

    let mut monitor_guard = MONITOR.lock();
    if monitor_guard.is_none() {
        let mut monitor = Monitor::new();
        monitor.activate();
        *monitor_guard = Some(monitor);
        log_debug!("Monitor interactive console ready");
    }

    if let Some(monitor) = monitor_guard.as_mut() {
        f(monitor);
    }
}

struct InterruptGuard {
    restore: bool,
}

impl InterruptGuard {
    fn new() -> Self {
        let was_enabled = interrupts::are_enabled();
        if !was_enabled {
            interrupts::enable();
        }
        Self {
            restore: !was_enabled,
        }
    }
}

impl Drop for InterruptGuard {
    fn drop(&mut self) {
        if self.restore {
            interrupts::disable();
        }
    }
}

/// Enter the dedicated monitor loop and never return.
pub fn start_monitor() -> ! {
    init();
    loop {
        process_pending_serial();
        x86_64::instructions::hlt();
    }
}

/// Repaint the prompt after non-monitor serial output (for example normal kernel logs)
/// so interactive monitor sessions stay readable.
pub fn notify_external_serial_output() {
    if !config::ENABLE_KERNEL_MONITOR {
        return;
    }

    let Some(mut guard) = MONITOR.try_lock() else {
        return;
    };

    if let Some(monitor) = guard.as_mut() {
        monitor.restore_prompt_after_external_output();
    }
}

/// State for the shared monitor instance.
pub(crate) struct Monitor {
    /// Current input line buffer.
    line_buffer: String,
    /// Last memory address examined for continuation-style commands.
    pub(crate) last_addr: u64,
    /// Serial device class used for monitor output.
    serial_class: DeviceClass,
    /// Tracks whether the banner/prompt has already been shown.
    active: bool,
}

impl Monitor {
    /// Construct a fresh monitor state object.
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
            if let Some(mut mgr) = driver_manager().try_lock() {
                let _ = mgr.write_class(self.serial_class, bytes);
            }
        }
    }

    /// Write a string to the monitor output path.
    pub(crate) fn write(&self, s: &str) {
        self.write_bytes(s.as_bytes());
    }

    /// Write a string followed by CRLF.
    pub(crate) fn writeln(&self, s: &str) {
        self.write(s);
        self.write("\r\n");
    }

    /// Repaint the prompt and any in-progress line buffer after unrelated serial output.
    fn restore_prompt_after_external_output(&self) {
        if !self.active {
            return;
        }

        self.write(PROMPT);
        if !self.line_buffer.is_empty() {
            self.write(&self.line_buffer);
        }
    }

    /// Handle one incoming monitor character.
    ///
    /// This is a small line editor with basic terminal control for monitor input.
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
                self.write("^C"); // Show cancellation indicator
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
                    self.write_bytes(&[byte]); // Echo character
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
            "status" | "stat" => self.cmd_status(),
            "regs" | "r" => self.cmd_registers(),
            "mem" | "m" => self.cmd_memory(&parts[1..]),
            "dump" | "d" => self.cmd_dump(&parts[1..]),
            "write" | "w" => self.cmd_write(&parts[1..]),
            "fill" | "f" => self.cmd_fill(&parts[1..]),
            "ptwalk" | "pt" => self.cmd_ptwalk(&parts[1..]),
            "ptdump" => self.cmd_ptdump(&parts[1..]),
            "devices" | "dev" => self.cmd_devices(),
            "pci" => self.cmd_pci(&parts[1..]),
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
            "log" | "loglevel" | "logoutput" => self.cmd_log(&parts[1..]),
            "reset" => self.cmd_reset(),
            "halt" => self.cmd_halt(),
            "clear" | "cls" => self.cmd_clear(),
            "usb" => self.cmd_usb(&parts[1..]),
            "kbd" => self.cmd_kbd(),
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
        self.writeln("  ptwalk|pt VIRT      - Walk page tables for virtual address");
        self.writeln("  ptdump LEVEL [...]  - Dump entries from a page-table level");
        self.writeln("");
        self.writeln("System Inspection:");
        self.writeln("  status|stat         - Quick system summary");
        self.writeln("  regs|r              - Display CPU registers");
        self.writeln("  stack|bt            - Display stack backtrace");
        self.writeln("  acpi                - Display ACPI information");
        self.writeln("  phys|physmem        - Display physical memory statistics");
        self.writeln("  devices|dev         - List registered devices");
        self.writeln("  pci                 - Enumerate PCI functions and BARs");
        self.writeln("  usb [OPTIONS]       - USB/xHCI diagnostics (use 'usb help' for options)");
        self.writeln("  kbd                 - Dump buffered keyboard events");
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
        self.writeln("Logging:");
        self.writeln("  log level [MODULE] LEVEL     - Set log level");
        self.writeln("  log output LEVEL TARGET      - Set output target");
        self.writeln("");
        self.writeln("System Control:");
        self.writeln("  reset               - Reboot system");
        self.writeln("  halt                - Halt CPU");
        self.writeln("  clear|cls           - Clear screen");
        self.writeln("  help|?              - Show this help");
    }

    /// Drain the shared keyboard input queue and print each transition.
    ///
    /// This remains a handy teaching aid: students can validate the keyboard stack before they
    /// wire higher-level input routing.
    fn cmd_kbd(&mut self) {
        let mut count = 0;
        loop {
            match keyboard_pop_event() {
                Some(event) => {
                    count += 1;
                    let state = match event.transition {
                        KeyTransition::Pressed => "press",
                        KeyTransition::Released => "release",
                    };
                    if let Some(ch) = event.ascii {
                        let ascii_repr = match ch {
                            '\n' => String::from("\\n"),
                            '\t' => String::from("\\t"),
                            ' ' => String::from("' '"),
                            _ => format!("'{}'", ch),
                        };
                        self.writeln(&format!(
                            "{:>7} usage=0x{:02x} label={} ascii={}",
                            state, event.usage, event.label, ascii_repr
                        ));
                    } else {
                        self.writeln(&format!(
                            "{:>7} usage=0x{:02x} label={}",
                            state, event.usage, event.label
                        ));
                    }
                }
                None => break,
            }
        }

        if count == 0 {
            self.writeln("(keyboard buffer empty)");
        }
    }
}
