//! Serial monitor inspired by WozMon but extended for Theseus.
//! Provides an interactive REPL on the serial console for inspecting state,
//! executing helper commands, and collecting diagnostic output.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt::Write;
use core::str;
use spin::Mutex;

use crate::drivers::manager::driver_manager;
use crate::drivers::traits::DeviceClass;
use crate::kernel_write_line;

static MONITOR_STATE: Mutex<Monitor> = Mutex::new(Monitor::new());
const PROMPT: &str = "*: ";

/// Public entry points ------------------------------------------------------
pub fn init() {
    MONITOR_STATE.lock().activate();
}

pub fn notify_serial_byte(byte: u8) {
    let mut monitor = MONITOR_STATE.lock();
    monitor.activate();
    monitor.process_byte(byte);
}

pub fn write_serial(msg: &str) {
    let mut mgr = driver_manager().lock();
    let _ = mgr.write_class(DeviceClass::Serial, msg.as_bytes());
}

/// Monitor implementation ---------------------------------------------------
struct Monitor {
    input: String,
    history: Vec<String>,
    active: bool,
}

impl Monitor {
    const fn new() -> Self {
        Self {
            input: String::new(),
            history: Vec::new(),
            active: false,
        }
    }

    fn activate(&mut self) {
        if !self.active {
            kernel_write_line("[monitor] serial monitor ready");
            self.write_serial("Theseus Monitor (type 'help' for commands)\r\n");
            self.prompt();
            self.active = true;
        }
    }

    fn prompt(&mut self) {
        self.write_serial(PROMPT);
    }

    fn write_serial(&self, msg: &str) {
        let mut mgr = driver_manager().lock();
        let _ = mgr.write_class(DeviceClass::Serial, msg.as_bytes());
    }

    fn process_byte(&mut self, byte: u8) {
        match byte {
            b'\r' | b'\n' => {
                self.write_serial("\r\n");
                let command = self.input.trim().to_string();
                self.input.clear();
                if !command.is_empty() {
                    self.history.push(command.clone());
                    if let Some(output) = self.execute(&command) {
                        self.write_serial(&output);
                        if !output.ends_with("\r\n") {
                            self.write_serial("\r\n");
                        }
                    }
                }
                self.prompt();
            }
            0x08 | 0x7F => {
                if self.input.pop().is_some() {
                    self.write_serial("\x08 \x08");
                }
            }
            0x15 => {
                while self.input.pop().is_some() {
                    self.write_serial("\x08 \x08");
                }
            }
            byte => {
                if self.input.len() < 128 {
                    self.input.push(byte as char);
                    let buf = [byte];
                    if let Ok(chr) = str::from_utf8(&buf) {
                        self.write_serial(chr);
                    }
                }
            }
        }
    }

    fn execute(&mut self, cmd: &str) -> Option<String> {
        let mut parts = cmd.split_whitespace();
        let token = parts.next()?;
        let result = match token {
            "help" | "?" => self.cmd_help(),
            "regs" => self.cmd_regs(),
            "ticks" => self.cmd_ticks(),
            "history" => self.cmd_history(),
            "dump" => {
                let addr = parts.next().and_then(parse_u64);
                let len = parts.next().and_then(parse_u64).unwrap_or(64);
                self.cmd_dump(addr, len as usize)
            }
            "apic" => self.cmd_apic(parts.next()),
            "trace" => self.cmd_trace(parts.next()),
            _ => "?".to_string(),
        };
        Some(result)
    }

    fn cmd_help(&self) -> String {
        let mut out = String::new();
        out.push_str("Commands:\r\n");
        out.push_str("  help/?             Show this help\r\n");
        out.push_str("  regs               Dump control registers\r\n");
        out.push_str("  ticks              Show LAPIC timer ticks\r\n");
        out.push_str("  dump <addr> [len]  Hex dump physical/virtual memory\r\n");
        out.push_str("  apic [reg]         Read LAPIC register (hex)\r\n");
        out.push_str("  history            Show command history\r\n");
        out.push_str("  trace <on|off>     Toggle serial trace streaming\r\n");
        out
    }

    fn cmd_regs(&self) -> String {
        use x86_64::registers::control::{Cr0, Cr2, Cr3, Cr4};
        let mut out = String::new();
        let _ = write!(
            &mut out,
            "CR0={:016x} CR2={:016x} CR3={:016x} CR4={:016x}\r\n",
            Cr0::read_raw(),
            Cr2::read().as_u64(),
            Cr3::read().0.start_address().as_u64(),
            Cr4::read_raw()
        );
        out
    }

    fn cmd_ticks(&self) -> String {
        let ticks = crate::interrupts::TIMER_TICKS.load(core::sync::atomic::Ordering::Relaxed);
        format!("ticks={}\r\n", ticks)
    }

    fn cmd_history(&self) -> String {
        if self.history.is_empty() {
            return "<empty>\r\n".to_string();
        }
        let mut out = String::new();
        for (idx, entry) in self.history.iter().enumerate().rev() {
            let _ = write!(&mut out, "{:02}: {}\r\n", idx, entry);
        }
        out
    }

    fn cmd_dump(&self, addr: Option<u64>, len: usize) -> String {
        let Some(start) = addr else {
            return "Usage: dump <addr> [len]\r\n".to_string();
        };
        let mut out = String::new();
        unsafe {
            let ptr = start as *const u8;
            for offset in 0..len {
                if offset % 16 == 0 {
                    if offset > 0 {
                        out.push_str("\r\n");
                    }
                    let _ = write!(&mut out, "{:016x}: ", start + offset as u64);
                }
                let byte = core::ptr::read_volatile(ptr.add(offset));
                let _ = write!(&mut out, "{:02x} ", byte);
            }
        }
        out.push_str("\r\n");
        out
    }

    fn cmd_apic(&self, reg: Option<&str>) -> String {
        let base = unsafe { crate::interrupts::get_apic_base() };
        let addr = if let Some(r) = reg.and_then(parse_u64) {
            base + r
        } else {
            base
        };
        let val = unsafe { crate::interrupts::read_apic_register(addr, 0) };
        format!("apic[{:#x}] = {:#x}\r\n", addr, val)
    }

    fn cmd_trace(&self, state: Option<&str>) -> String {
        match state {
            Some("on") => "trace on\r\n".to_string(),
            Some("off") => "trace off\r\n".to_string(),
            _ => "usage: trace <on|off>\r\n".to_string(),
        }
    }
}

fn parse_u64(s: &str) -> Option<u64> {
    if let Some(hex) = s.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).ok()
    } else if let Some(hex) = s.strip_prefix('$') {
        u64::from_str_radix(hex, 16).ok()
    } else {
        u64::from_str_radix(s, 10).ok()
    }
}
