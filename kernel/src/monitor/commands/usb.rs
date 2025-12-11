//! USB diagnostics command
//!
//! Provides `usb` monitor command for inspecting xHCI controller state.

use crate::drivers::usb;
use crate::monitor::parsing::parse_number;
use crate::monitor::Monitor;
use alloc::format;

impl Monitor {
    /// Inspect xHCI controllers, ports, and discovered HID endpoints.
    ///
    /// # Usage
    /// ```text
    /// usb                 # Summary of controllers
    /// usb summary         # Same as above
    /// usb ports [index]   # Detailed port status (default index 0)
    /// usb hid             # List controllers exposing HID boot keyboards
    /// usb help            # Command usage
    /// ```
    pub(in crate::monitor) fn cmd_usb(&self, args: &[&str]) {
        let snapshot = usb::diagnostics_snapshot();

        if snapshot.is_empty() {
            self.writeln("No xHCI controllers discovered.");
            return;
        }

        let subcommand = args.first().copied().unwrap_or("summary");
        match subcommand {
            "summary" => {
                self.writeln(&format!("xHCI controllers ({})", snapshot.len()));
                for (index, ctrl) in snapshot.iter().enumerate() {
                    let state = if ctrl.controller_running {
                        "RUN"
                    } else {
                        "HALT"
                    };
                    let slots = if ctrl.slots_enabled {
                        "enabled"
                    } else {
                        "disabled"
                    };
                    let attached = ctrl
                        .attached_port
                        .map(|port| format!("port{:02}", port))
                        .unwrap_or_else(|| "none".into());
                    let speed = ctrl.attached_speed.as_deref().unwrap_or("-");
                    let hid = if ctrl.hid_keyboard.is_some() {
                        "yes"
                    } else {
                        "no"
                    };
                    let irq = if ctrl.msi_enabled {
                        ctrl.msi_vector
                            .map(|vec| format!("msi vec={:#04x}", vec))
                            .unwrap_or_else(|| "msi vec=??".into())
                    } else {
                        "legacy".into()
                    };
                    let iman = format!(
                        "iman ie={} ip={} raw={:#010x}",
                        yes_no(ctrl.interrupt_enabled),
                        yes_no(ctrl.interrupt_pending),
                        ctrl.iman_raw
                    );
                    self.writeln(&format!(
                        "  [{:02}] {:<16} phys={:#014x} mmio={:#x} ports={} slots={} state={} slots={} attached={} ({}) hid={} irq={} {}",
                        index,
                        ctrl.ident,
                        ctrl.phys_base,
                        ctrl.mmio_length,
                        ctrl.max_ports,
                        ctrl.max_slots,
                        state,
                        slots,
                        attached,
                        speed,
                        hid,
                        irq,
                        iman
                    ));
                }
                self.writeln(
                    "Use 'usb ports <index>' for per-port detail, 'usb hid' for HID endpoints.",
                );
            }
            "ports" => {
                let target_index = args
                    .get(1)
                    .and_then(|arg| parse_number(arg))
                    .map(|value| value as usize)
                    .unwrap_or(0);

                if target_index >= snapshot.len() {
                    self.writeln(&format!(
                        "Invalid controller index {} ({} controllers available)",
                        target_index,
                        snapshot.len()
                    ));
                    return;
                }

                let ctrl = &snapshot[target_index];
                self.writeln(&format!(
                    "Ports for controller [{:02}] {}",
                    target_index, ctrl.ident
                ));
                self.writeln("  Port Conn Enab Pwr OC  Speed     Link       Register");
                for port in ctrl.ports.iter() {
                    self.writeln(&format!(
                        "  {:>4} {:>4} {:>4} {:>3} {:>3} {:<8} {:<10} {:#010x}",
                        port.index,
                        yes_no(port.connected),
                        yes_no(port.enabled),
                        yes_no(port.powered),
                        yes_no(port.overcurrent),
                        port.speed,
                        port.link_state,
                        port.raw
                    ));
                }
                if let Some(ep) = ctrl.hid_keyboard.as_ref() {
                    self.writeln(&format!(
                        "  HID keyboard endpoint: interface={} endpoint={:#04x} max_packet={} interval={}",
                        ep.interface_number, ep.endpoint_address, ep.max_packet_size, ep.interval
                    ));
                }
            }
            "hid" => {
                self.writeln("HID boot keyboards discovered:");
                let mut found = false;
                for (index, ctrl) in snapshot.iter().enumerate() {
                    if let Some(ep) = ctrl.hid_keyboard.as_ref() {
                        found = true;
                        let attached = ctrl
                            .attached_port
                            .map(|port| format!("port{:02}", port))
                            .unwrap_or_else(|| "unknown".into());
                        let speed = ctrl.attached_speed.as_deref().unwrap_or("-");
                        self.writeln(&format!(
                            "  [{:02}] {} {} speed={} interface={} endpoint={:#04x} max_packet={} interval={}",
                            index,
                            ctrl.ident,
                            attached,
                            speed,
                            ep.interface_number,
                            ep.endpoint_address,
                            ep.max_packet_size,
                            ep.interval
                        ));
                    }
                }
                if !found {
                    self.writeln("  (no HID boot keyboards detected)");
                }
            }
            "help" | "?" => {
                self.writeln("usb diagnostics command:");
                self.writeln("  usb                 - summary of xHCI controllers");
                self.writeln("  usb ports [index]   - detailed port state (default index 0)");
                self.writeln(
                    "  usb hid             - list controllers exposing HID boot keyboards",
                );
            }
            other => {
                self.writeln(&format!("Unknown usb subcommand '{}'", other));
                self.writeln("Use 'usb help' for usage.");
            }
        }
    }
}

fn yes_no(value: bool) -> &'static str {
    if value {
        "yes"
    } else {
        "no"
    }
}
