//! PCI Express enhanced configuration access (ECAM) support.
//!
//! This module reads PCIe configuration space using the MMIO windows described
//! by the ACPI MCFG table. It assumes the boot environment has exposed those
//! regions via `PlatformInfo::pci_config_regions`.

use crate::acpi::{PciBridgeInfo, PciConfigRegion};
use crate::{log_debug, log_trace};
use alloc::collections::BTreeSet;
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{AtomicU64, Ordering};

const MAX_DEVICE: u8 = 31;
const MAX_FUNCTION: u8 = 7;

const BUS_STRIDE: u64 = 0x1_0000; // 1 MiB per bus
const DEVICE_STRIDE: u64 = 0x8000; // 32 KiB per device
const FUNCTION_STRIDE: u64 = 0x1000; // 4 KiB per function
const HIGH_MMIO_GRANULARITY: u64 = 0x0010_0000; // 1 MiB windows for reassigned device MMIO
static NEXT_HIGH_MMIO_BASE: AtomicU64 = AtomicU64::new(0xC000_0000);

/// Decoded Base Address Register (BAR) entry.
#[derive(Clone, Copy, Debug)]
pub enum PciBar {
    None,
    Memory32 { base: u64, size: u64, prefetchable: bool },
    Memory64 { base: u64, size: u64, prefetchable: bool },
    Io { base: u64, size: u64 },
}

impl PciBar {
    pub fn memory_base(&self) -> Option<u64> {
        match self {
            PciBar::Memory32 { base, .. } | PciBar::Memory64 { base, .. } => Some(*base),
            _ => None,
        }
    }

    pub fn size(&self) -> Option<u64> {
        match self {
            PciBar::Memory32 { size, .. }
            | PciBar::Memory64 { size, .. }
            | PciBar::Io { size, .. } => Some(*size),
            PciBar::None => None,
        }
    }
}

/// Capabilities advertised by a PCI function.
#[derive(Clone, Copy, Debug, Default)]
pub struct PciCapabilities {
    pub msi: bool,
    pub msix: bool,
}

/// Summary of a discovered PCI function.
#[derive(Clone, Copy)]
pub struct PciDeviceInfo {
    pub segment: u16,
    pub bus: u8,
    pub device: u8,
    pub function: u8,
    pub vendor_id: u16,
    pub device_id: u16,
    pub class_code: u8,
    pub subclass: u8,
    pub prog_if: u8,
    pub revision_id: u8,
    pub header_type: u8,
    pub command: u16,
    pub status: u16,
    pub bars: [PciBar; 6],
    pub interrupt_line: u8,
    pub interrupt_pin: u8,
    pub capabilities: PciCapabilities,
}

impl PciDeviceInfo {
    pub fn is_multi_function(&self) -> bool {
        self.header_type & 0x80 != 0
    }

    pub fn class_triplet(&self) -> (u8, u8, u8) {
        (self.class_code, self.subclass, self.prog_if)
    }

    pub fn interrupt_line(&self) -> Option<u8> {
        match self.interrupt_line {
            0xFF => None,
            value => Some(value),
        }
    }

    pub fn first_memory_bar(&self) -> Option<PciBar> {
        self.bars
            .iter()
            .copied()
            .find(|bar| matches!(bar, PciBar::Memory32 { .. } | PciBar::Memory64 { .. }))
    }
}

impl fmt::Debug for PciDeviceInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PciDeviceInfo")
            .field("segment", &self.segment)
            .field("bus", &self.bus)
            .field("device", &self.device)
            .field("function", &self.function)
            .field("vendor_id", &format_args!("{:#06x}", self.vendor_id))
            .field("device_id", &format_args!("{:#06x}", self.device_id))
            .field(
                "class",
                &format_args!(
                    "{:02x}:{:02x}:{:02x}",
                    self.class_code, self.subclass, self.prog_if
                ),
            )
            .field("revision_id", &self.revision_id)
            .field("header_type", &format_args!("{:#04x}", self.header_type))
            .field("command", &format_args!("{:#06x}", self.command))
            .field("status", &format_args!("{:#06x}", self.status))
            .field("bars", &self.bars)
            .field("interrupt_line", &self.interrupt_line)
            .field("interrupt_pin", &self.interrupt_pin)
            .field("capabilities", &self.capabilities)
            .finish()
    }
}

pub struct PciTopology {
    pub functions: Vec<PciDeviceInfo>,
    pub bridges: Vec<PciBridgeInfo>,
}

pub fn enumerate(regions: &[PciConfigRegion]) -> PciTopology {
    let mut devices = Vec::new();
    let mut bridges = Vec::new();
    let mut visited_buses: BTreeSet<(u16, u8)> = BTreeSet::new();

    for region in regions {
        scan_bus(
            region,
            region.bus_start,
            &mut visited_buses,
            &mut devices,
            &mut bridges,
        );
    }

    log_debug!(
        "PCI scan complete: {} function(s) discovered ({} bridges)",
        devices.len(),
        bridges.len()
    );

    PciTopology {
        functions: devices,
        bridges,
    }
}

fn scan_bus(
    region: &PciConfigRegion,
    bus: u8,
    visited_buses: &mut BTreeSet<(u16, u8)>,
    devices: &mut Vec<PciDeviceInfo>,
    bridges: &mut Vec<PciBridgeInfo>,
) -> u8 {
    if bus < region.bus_start || bus > region.bus_end {
        return bus;
    }
    if !visited_buses.insert((region.segment, bus)) {
        return bus;
    }

    let mut max_bus = bus;

    for device in 0..=MAX_DEVICE {
        if let Some(info) = read_function(region, bus, device, 0) {
            max_bus = max_bus.max(handle_function(region, info, visited_buses, devices, bridges));

            if info.is_multi_function() {
                for function in 1..=MAX_FUNCTION {
                    if let Some(other) = read_function(region, bus, device, function) {
                        max_bus = max_bus.max(handle_function(
                            region,
                            other,
                            visited_buses,
                            devices,
                            bridges,
                        ));
                    }
                }
            }
        }
    }

    max_bus
}

fn handle_function(
    region: &PciConfigRegion,
    mut info: PciDeviceInfo,
    visited_buses: &mut BTreeSet<(u16, u8)>,
    devices: &mut Vec<PciDeviceInfo>,
    bridges: &mut Vec<PciBridgeInfo>,
) -> u8 {
    let mut max_bus = info.bus;

    let function_base = function_config_base(region, info.bus, info.device, info.function);

    let is_bridge = info.header_type & 0x7F == 0x01;
    if is_bridge {
        log_trace!(
            "PCI bridge {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
            info.segment,
            info.bus,
            info.device,
            info.function,
            info.vendor_id,
            info.device_id,
            info.class_code,
            info.subclass,
            info.prog_if
        );

        let secondary = config_read_u8(function_base, 0x19);
        let subordinate = config_read_u8(function_base, 0x1A);
        let mut child_max = secondary;

        if secondary != 0 && secondary <= region.bus_end {
            let scanned = scan_bus(region, secondary, visited_buses, devices, bridges);
            child_max = child_max.max(scanned);
        }

        bridges.push(PciBridgeInfo {
            segment: info.segment,
            bus: info.bus,
            device: info.device,
            function: info.function,
            secondary_bus: secondary,
            subordinate_bus: subordinate,
            vendor_id: info.vendor_id,
            device_id: info.device_id,
            max_child_bus: child_max,
        });

        max_bus = max_bus.max(child_max);
    } else {
        log_debug!(
            "PCI {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x} bar0={:?}",
            info.segment,
            info.bus,
            info.device,
            info.function,
            info.vendor_id,
            info.device_id,
            info.class_code,
            info.subclass,
            info.prog_if,
            info.first_memory_bar()
        );
        if info.class_code == 0x01 && info.subclass == 0x08 {
            log_debug!("    NVMe detail: {:?}", info);
        }
        if info.class_code == 0x01 && info.subclass == 0x08 {
            let new_command = info.command & !0x0007;
            if new_command != info.command {
                config_write_u16(function_base, 0x04, new_command);
                info.command = new_command;
                log_debug!(
                    "    NVMe {:04x}:{:02x}:{:02x}.{} memory/io decode disabled pending reconfiguration",
                    info.segment,
                    info.bus,
                    info.device,
                    info.function
                );
            }
            let new_base = NEXT_HIGH_MMIO_BASE.fetch_add(HIGH_MMIO_GRANULARITY, Ordering::Relaxed);
            let attr = config_read_u32(function_base, 0x10) & 0xF;
            let low = (new_base as u32 & 0xFFFF_FFF0) | attr;
            let high = (new_base >> 32) as u32;
            config_write_u32(function_base, 0x10, low);
            config_write_u32(function_base, 0x14, high);
            if let PciBar::Memory64 {
                ref mut base, ..
            } = info.bars[0]
            {
                *base = new_base;
            }
            log_debug!(
                "    NVMe {:04x}:{:02x}:{:02x}.{} BAR0 reassigned to {:#010x}",
                info.segment,
                info.bus,
                info.device,
                info.function,
                new_base
            );
        }
    }

    devices.push(info);
    max_bus
}

fn read_function(
    region: &PciConfigRegion,
    bus: u8,
    device: u8,
    function: u8,
) -> Option<PciDeviceInfo> {
    let base = function_config_base(region, bus, device, function);
    let vendor_id = config_read_u16(base, 0x00);
    if vendor_id == 0xFFFF {
        return None;
    }

    let device_id = config_read_u16(base, 0x02);
    let command = config_read_u16(base, 0x04);
    let status = config_read_u16(base, 0x06);
    let revision_id = config_read_u8(base, 0x08);
    let prog_if = config_read_u8(base, 0x09);
    let subclass = config_read_u8(base, 0x0A);
    let class_code = config_read_u8(base, 0x0B);
    let header_type = config_read_u8(base, 0x0E);

    let probe_bar_sizes = !(class_code == 0x01 && subclass == 0x08 && prog_if == 0x02);

    let bars = read_type0_bars(base, probe_bar_sizes);
    if class_code == 0x01 && subclass == 0x08 {
        log_debug!(
            "NVMe {:04x}:{:02x}:{:02x}.{} BAR snapshot: {:?}",
            region.segment,
            bus,
            device,
            function,
            bars
        );
    }

    let interrupt_line = config_read_u8(base, 0x3C);
    let interrupt_pin = config_read_u8(base, 0x3D);
    let capabilities = parse_capabilities(base, header_type, status);

    Some(PciDeviceInfo {
        segment: region.segment,
        bus,
        device,
        function,
        vendor_id,
        device_id,
        class_code,
        subclass,
        prog_if,
        revision_id,
        header_type,
        command,
        status,
        bars,
        interrupt_line,
        interrupt_pin,
        capabilities,
    })
}

fn function_config_base(
    region: &PciConfigRegion,
    bus: u8,
    device: u8,
    function: u8,
) -> u64 {
    region.virt_base
        + ((bus - region.bus_start) as u64 * BUS_STRIDE)
        + (device as u64 * DEVICE_STRIDE)
        + (function as u64 * FUNCTION_STRIDE)
}

fn read_type0_bars(function_base: u64, probe_sizes: bool) -> [PciBar; 6] {
    let mut bars = [PciBar::None; 6];
    let mut index = 0usize;
    let mut original_command = 0u16;
    let mut had_decode_enabled = false;
    if probe_sizes {
        original_command = config_read_u16(function_base, 0x04);
        had_decode_enabled = (original_command & 0x0007) != 0;
        if had_decode_enabled {
            // Disable IO, memory, and bus mastering while probing BAR sizes so devices
            // do not observe the temporary all-ones values we write below.
            config_write_u16(function_base, 0x04, original_command & !0x0007);
        }
    }
    while index < 6 {
        let offset = 0x10 + (index as u8) * 4;
        let raw = config_read_u32(function_base, offset);
        if raw == 0 {
            bars[index] = PciBar::None;
            index += 1;
            continue;
        }

        if raw & 0x1 == 0x1 {
            let base = (raw & 0xFFFF_FFFC) as u64;
            let size = if probe_sizes {
                config_write_u32(function_base, offset, 0xFFFF_FFFC);
                let mask = config_read_u32(function_base, offset);
                config_write_u32(function_base, offset, raw);
                let masked = mask & 0xFFFF_FFFC;
                if masked == 0 || masked == 0xFFFF_FFFC {
                    0
                } else {
                    ((!masked).wrapping_add(1) & 0xFFFF_FFFC) as u64
                }
            } else {
                0
            };
            bars[index] = if size == 0 {
                PciBar::Io { base, size: 0 }
            } else {
                PciBar::Io { base, size }
            };
            index += 1;
            continue;
        }

        let prefetchable = (raw & (1 << 3)) != 0;
        let bar_type = (raw >> 1) & 0x3;
        match bar_type {
            0x0 => {
                let base = (raw & 0xFFFF_FFF0) as u64;
                let size = if probe_sizes {
                    config_write_u32(function_base, offset, 0xFFFF_FFFF);
                    let mask = config_read_u32(function_base, offset) & 0xFFFF_FFF0;
                    config_write_u32(function_base, offset, raw);
                    if mask == 0 || mask == 0xFFFF_FFF0 {
                        0
                    } else {
                        ((!mask).wrapping_add(1) & 0xFFFF_FFF0) as u64
                    }
                } else {
                    0
                };
                if size == 0 {
                    bars[index] = PciBar::Memory32 {
                        base,
                        size: 0,
                        prefetchable,
                    };
                    index += 1;
                    continue;
                }
                bars[index] = PciBar::Memory32 {
                    base,
                    size,
                    prefetchable,
                };
                index += 1;
            }
            0x2 => {
                if index + 1 >= 6 {
                    bars[index] = PciBar::None;
                    index += 1;
                    continue;
                }
                let base_low = (raw & 0xFFFF_FFF0) as u64;
                let raw_high = config_read_u32(function_base, offset + 4);
                let base_high = (raw_high as u64) << 32;
                let base = base_high | base_low;
                let size = if probe_sizes {
                    config_write_u32(function_base, offset, 0xFFFF_FFFF);
                    config_write_u32(function_base, offset + 4, 0xFFFF_FFFF);
                    let mask_low = config_read_u32(function_base, offset) & 0xFFFF_FFF0;
                    let mask_high = config_read_u32(function_base, offset + 4);
                    config_write_u32(function_base, offset, raw);
                    config_write_u32(function_base, offset + 4, raw_high);
                    let mask = ((mask_high as u64) << 32) | (mask_low as u64);
                    if mask == 0 || mask == 0xFFFF_FFFF_FFFFFFF0 {
                        0
                    } else {
                        (!mask).wrapping_add(1) & 0xFFFF_FFFF_FFFFFFF0
                    }
                } else {
                    0
                };
                if size == 0 {
                    bars[index] = PciBar::Memory64 {
                        base,
                        size: 0,
                        prefetchable,
                    };
                    bars[index + 1] = PciBar::None;
                    index += 2;
                    continue;
                }
                bars[index] = PciBar::Memory64 {
                    base,
                    size,
                    prefetchable,
                };
                bars[index + 1] = PciBar::None;
                index += 2;
            }
            _ => {
                bars[index] = PciBar::None;
                index += 1;
            }
        }
    }
    if had_decode_enabled {
        config_write_u16(function_base, 0x04, original_command);
    }
    bars
}

fn parse_capabilities(function_base: u64, header_type: u8, status: u16) -> PciCapabilities {
    if (status & (1 << 4)) == 0 {
        return PciCapabilities::default();
    }

    let cap_ptr = match header_type & 0x7F {
        0x00 | 0x01 => config_read_u8(function_base, 0x34),
        _ => 0,
    };

    if cap_ptr < 0x40 {
        return PciCapabilities::default();
    }

    let mut pointer = cap_ptr;
    let mut guard = 0;
    let mut caps = PciCapabilities::default();

    while pointer >= 0x40 && guard < 64 {
        let addr = function_base + pointer as u64;
        let cap_id = unsafe { core::ptr::read_volatile(addr as *const u8) };
        let next = unsafe { core::ptr::read_volatile((addr + 1) as *const u8) };
        match cap_id {
            0x05 => caps.msi = true,
            0x11 => caps.msix = true,
            _ => {}
        }
        if next == 0 || next == pointer {
            break;
        }
        pointer = next;
        guard += 1;
    }

    caps
}

fn config_read_u32(function_base: u64, offset: u8) -> u32 {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

fn config_read_u16(function_base: u64, offset: u8) -> u16 {
    let aligned = offset & !0x3;
    let value = config_read_u32(function_base, aligned);
    let shift = (offset & 0x3) * 8;
    ((value >> shift) & 0xFFFF) as u16
}

fn config_read_u8(function_base: u64, offset: u8) -> u8 {
    let aligned = offset & !0x3;
    let value = config_read_u32(function_base, aligned);
    let shift = (offset & 0x3) * 8;
    ((value >> shift) & 0xFF) as u8
}

fn config_write_u32(function_base: u64, offset: u8, value: u32) {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) };
}

fn config_write_u16(function_base: u64, offset: u8, value: u16) {
    let aligned = offset & !0x3;
    let mut word = config_read_u32(function_base, aligned);
    let shift = (offset & 0x3) * 8;
    let mask = !(0xFFFFu32 << shift);
    word = (word & mask) | ((value as u32) << shift);
    config_write_u32(function_base, aligned, word);
}
