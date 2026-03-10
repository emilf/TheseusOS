//! Device enumeration command
//!
//! This module implements the `devices` command for listing registered hardware devices.

use crate::acpi;
use crate::drivers::manager::driver_manager;
use crate::drivers::traits::DeviceResource;
use crate::monitor::Monitor;
use alloc::{format, string::String, vec::Vec};

impl Monitor {
    /// List all registered devices
    ///
    /// Enumerates all hardware devices registered with the driver manager,
    /// showing their class, status, physical address, and IRQ assignments.
    ///
    /// # Examples
    /// ```text
    /// devices           # List all devices
    /// dev               # Short alias
    /// ```
    ///
    /// # Output Format
    /// For each device, displays:
    /// - Index number
    /// - Device ID
    /// - Device class (Serial, Storage, etc.)
    /// - Status (bound/pending)
    /// - Physical address (if any)
    /// - IRQ number (if any)
    /// - Driver state pointer (if bound)
    pub(in crate::monitor) fn cmd_devices(&self) {
        let devices: Vec<_> = {
            let mgr = driver_manager().lock();
            mgr.devices().to_vec()
        };

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
            if !dev.resources.is_empty() {
                self.writeln("      resources:");
                for res in dev.resources.iter() {
                    match res {
                        DeviceResource::Memory {
                            base,
                            size,
                            prefetchable,
                        } => {
                            let attr = if *prefetchable {
                                "prefetch"
                            } else {
                                "non-prefetch"
                            };
                            self.writeln(&format!(
                                "        MEM {:#016x}-{:#016x} ({})",
                                base,
                                base.saturating_add(size.saturating_sub(1)),
                                attr
                            ));
                        }
                        DeviceResource::Io { base, size } => {
                            self.writeln(&format!(
                                "        IO  {:#06x}-{:#06x}",
                                base,
                                base.saturating_add(size.saturating_sub(1))
                            ));
                        }
                    }
                }
            }
        }

        if let Some(info) = acpi::cached_platform_info() {
            if !info.pci_bridges.is_empty() {
                self.writeln("\nPCI bridges:");
                for bridge in info.pci_bridges.iter() {
                    let io = format_bridge_window(&bridge.io_window);
                    let mem = format_bridge_window(&bridge.mem_window);
                    let pref = format_bridge_window(&bridge.pref_mem_window);
                    self.writeln(&format!(
                        "  {:04x}:{:02x}:{:02x}.{} -> secondary {:02x}, subordinate {:02x}, max_child {:02x} | io:{} mem:{} pref:{}",
                        bridge.segment,
                        bridge.bus,
                        bridge.device,
                        bridge.function,
                        bridge.secondary_bus,
                        bridge.subordinate_bus,
                        bridge.max_child_bus,
                        io,
                        mem,
                        pref
                    ));
                }
            }
        }
    }
}

fn format_bridge_window(window: &Option<acpi::BridgeWindow>) -> String {
    match window {
        Some(w) => format!("0x{:016x}-0x{:016x}", w.base, w.limit),
        None => "none".into(),
    }
}
