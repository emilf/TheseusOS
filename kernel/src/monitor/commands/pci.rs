//! PCI inspection command.

use alloc::format;

use crate::acpi;
use crate::drivers::pci;
use crate::monitor::Monitor;

impl Monitor {
    pub(crate) fn cmd_pci(&self, _args: &[&str]) {
        let Some(platform_info) = acpi::cached_platform_info() else {
            self.writeln("No cached ACPI platform info; PCI config regions unavailable.");
            return;
        };

        if platform_info.pci_config_regions.is_empty() {
            self.writeln("No PCI configuration regions exposed by firmware.");
            return;
        }

        if let Some(legacy) = platform_info.legacy_usb.as_ref() {
            self.writeln("Legacy USB ownership (from FADT):");
            self.writeln(&format!(
                "  SCI={} SMI_CMD={:#x} acpi_enable={:#04x} acpi_disable={:#04x} pstate={:#04x} cstate={:#04x}",
                legacy.sci_interrupt,
                legacy.smi_cmd_port,
                legacy.acpi_enable,
                legacy.acpi_disable,
                legacy.pstate_control,
                legacy.c_state_control
            ));
            self.writeln(&format!(
                "  legacy_devices={} prohibit_msi={} facs={:#014x?} dsdt={:#014x?}",
                legacy.legacy_devices,
                legacy.prohibit_msi,
                legacy.facs_address,
                legacy.dsdt_address
            ));
        } else {
            self.writeln("Legacy USB ownership: (no FADT data cached)");
        }

        if !platform_info.xhci_descriptors.is_empty() {
            self.writeln("ACPI XHCI descriptors:");
            for desc in platform_info.xhci_descriptors.iter() {
                let oem_id = core::str::from_utf8(&desc.oem_id)
                    .unwrap_or("")
                    .trim_end_matches(char::from(0))
                    .trim();
                let oem_table_id = core::str::from_utf8(&desc.oem_table_id)
                    .unwrap_or("")
                    .trim_end_matches(char::from(0))
                    .trim();
                self.writeln(&format!(
                    "  phys=0x{:012x} len={:#x} rev={} checksum={:#04x} OEM={} {}",
                    desc.physical_address,
                    desc.length,
                    desc.revision,
                    desc.checksum,
                    if oem_id.is_empty() { "-" } else { oem_id },
                    if oem_table_id.is_empty() {
                        "-"
                    } else {
                        oem_table_id
                    }
                ));
            }
        }
        self.writeln("");

        let topology = pci::enumerate(&platform_info.pci_config_regions);
        self.writeln(&format!(
            "PCI functions discovered: {} ({} bridges)",
            topology.functions.len(),
            topology.bridges.len()
        ));

        for func in topology.functions.iter() {
            let (class, subclass, prog_if) = func.class_triplet();
            self.writeln(&format!(
                "  {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x} hdr={:#04x}",
                func.segment,
                func.bus,
                func.device,
                func.function,
                func.vendor_id,
                func.device_id,
                class,
                subclass,
                prog_if,
                func.header_type
            ));
            for (idx, bar) in func.bars.iter().enumerate() {
                match bar {
                    pci::PciBar::Memory32 {
                        base,
                        size,
                        prefetchable,
                        ..
                    }
                    | pci::PciBar::Memory64 {
                        base,
                        size,
                        prefetchable,
                        ..
                    } => {
                        if *size > 0 {
                            self.writeln(&format!(
                                "      BAR{} MEM base={:#014x} size={:#x} {}",
                                idx,
                                base,
                                size,
                                if *prefetchable {
                                    "prefetch"
                                } else {
                                    "non-prefetch"
                                }
                            ));
                        }
                    }
                    pci::PciBar::Io { base, size, .. } => {
                        if *size > 0 {
                            self.writeln(&format!(
                                "      BAR{} IO  base={:#06x} size={:#x}",
                                idx, base, size
                            ));
                        }
                    }
                    pci::PciBar::None => {}
                }
            }
        }

        if topology.bridges.is_empty() {
            return;
        }

        self.writeln("");
        self.writeln("PCI bridges:");
        for bridge in topology.bridges.iter() {
            self.writeln(&format!(
                "  {:04x}:{:02x}:{:02x}.{} secondary={:02x} subordinate={:02x} max_child={:02x}",
                bridge.segment,
                bridge.bus,
                bridge.device,
                bridge.function,
                bridge.secondary_bus,
                bridge.subordinate_bus,
                bridge.max_child_bus
            ));
        }
    }
}
