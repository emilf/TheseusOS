//! Device enumeration command
//!
//! This module implements the `devices` command for listing registered hardware devices.

use crate::drivers::manager::driver_manager;
use crate::monitor::Monitor;
use alloc::format;

impl Monitor {
    /// List all registered devices
    pub(in crate::monitor) fn cmd_devices(&self) {
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
}

