//! PCI Express enhanced configuration access (ECAM) support.
//!
//! This module reads PCIe configuration space using the MMIO windows described
//! by the ACPI MCFG table. It assumes the boot environment has exposed those
//! regions via `PlatformInfo::pci_config_regions`.

use crate::acpi::PciConfigRegion;
use crate::{log_debug, log_trace};
use alloc::vec::Vec;
use core::fmt;

const MAX_DEVICE: u8 = 31;
const MAX_FUNCTION: u8 = 7;

const BUS_STRIDE: u64 = 0x1_0000; // 1 MiB per bus
const DEVICE_STRIDE: u64 = 0x8000; // 32 KiB per device
const FUNCTION_STRIDE: u64 = 0x1000; // 4 KiB per function

/// Decoded Base Address Register (BAR) entry.
#[derive(Clone, Copy, Debug)]
pub enum PciBar {
    None,
    Memory32 { base: u32, prefetchable: bool },
    Memory64 { base: u64, prefetchable: bool },
    Io { base: u32 },
}

impl PciBar {
    pub fn memory_base(&self) -> Option<u64> {
        match self {
            PciBar::Memory32 { base, .. } => Some(*base as u64),
            PciBar::Memory64 { base, .. } => Some(*base),
            PciBar::Io { .. } | PciBar::None => None,
        }
    }
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
            .finish()
    }
}

/// Enumerate all PCI functions accessible via the provided ECAM regions.
pub fn enumerate(regions: &[PciConfigRegion]) -> Vec<PciDeviceInfo> {
    let mut devices = Vec::new();

    for region in regions {
        for bus in region.bus_start..=region.bus_end {
            // Always probe function 0; multi-function devices will be handled below.
            if let Some(info) = read_function(region, bus, 0, 0) {
                let multifunction = info.is_multi_function();
                log_debug!(
                    "PCI {:04x}:{:02x}:{:02x}.0 vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
                    info.segment,
                    info.bus,
                    info.device,
                    info.vendor_id,
                    info.device_id,
                    info.class_code,
                    info.subclass,
                    info.prog_if
                );
                devices.push(info);

                if multifunction {
                    for function in 1..=MAX_FUNCTION {
                        if let Some(extra) = read_function(region, bus, info.device, function) {
                            log_debug!(
                                "PCI {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
                                extra.segment,
                                extra.bus,
                                extra.device,
                                extra.function,
                                extra.vendor_id,
                                extra.device_id,
                                extra.class_code,
                                extra.subclass,
                                extra.prog_if
                            );
                            devices.push(extra);
                        }
                    }
                }
            }

            for device in 1..=MAX_DEVICE {
                if let Some(info) = read_function(region, bus, device, 0) {
                    let multifunction = info.is_multi_function();
                    log_debug!(
                        "PCI {:04x}:{:02x}:{:02x}.0 vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
                        info.segment,
                        info.bus,
                        info.device,
                        info.vendor_id,
                        info.device_id,
                        info.class_code,
                        info.subclass,
                        info.prog_if
                    );
                    devices.push(info);

                    if multifunction {
                        for function in 1..=MAX_FUNCTION {
                            if let Some(extra) = read_function(region, bus, device, function) {
                                log_debug!(
                                    "PCI {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
                                    extra.segment,
                                    extra.bus,
                                    extra.device,
                                    extra.function,
                                    extra.vendor_id,
                                    extra.device_id,
                                    extra.class_code,
                                    extra.subclass,
                                    extra.prog_if
                                );
                                devices.push(extra);
                            }
                        }
                    }
                }
            }
        }
    }

    log_debug!(
        "PCI scan complete: {} function(s) discovered",
        devices.len()
    );
    devices
}

fn read_function(
    region: &PciConfigRegion,
    bus: u8,
    device: u8,
    function: u8,
) -> Option<PciDeviceInfo> {
    let base = function_base(region, bus, device, function)?;
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
    let cache_line_size = config_read_u8(base, 0x0C);
    let latency_timer = config_read_u8(base, 0x0D);
    let header_type = config_read_u8(base, 0x0E);
    let bist = config_read_u8(base, 0x0F);

    log_trace!(
        "PCI {:04x}:{:02x}:{:02x}.{} raw header cache_line={} latency={} header={:#04x} bist={:#04x}",
        region.segment,
        bus,
        device,
        function,
        cache_line_size,
        latency_timer,
        header_type,
        bist
    );

    let bars = if (header_type & 0x7F) == 0x00 {
        read_type0_bars(base)
    } else {
        [PciBar::None; 6]
    };

    let interrupt_line = config_read_u8(base, 0x3C);
    let interrupt_pin = config_read_u8(base, 0x3D);

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
    })
}

fn function_base(region: &PciConfigRegion, bus: u8, device: u8, function: u8) -> Option<u64> {
    if bus < region.bus_start
        || bus > region.bus_end
        || device > MAX_DEVICE
        || function > MAX_FUNCTION
    {
        return None;
    }

    let bus_offset = u64::from(bus - region.bus_start) * BUS_STRIDE;
    let device_offset = u64::from(device) * DEVICE_STRIDE;
    let function_offset = u64::from(function) * FUNCTION_STRIDE;

    Some(region.virt_base + bus_offset + device_offset + function_offset)
}

fn read_type0_bars(function_base: u64) -> [PciBar; 6] {
    let mut bars = [PciBar::None; 6];
    let mut index = 0usize;
    while index < 6 {
        let offset = 0x10 + (index as u8) * 4;
        let raw = config_read_u32(function_base, offset);
        if raw == 0 {
            bars[index] = PciBar::None;
            index += 1;
            continue;
        }

        if raw & 0x1 == 0x1 {
            let base = raw & 0xFFFF_FFFC;
            bars[index] = PciBar::Io { base };
            index += 1;
            continue;
        }

        let prefetchable = (raw & (1 << 3)) != 0;
        let bar_type = (raw >> 1) & 0x3;
        match bar_type {
            0x0 => {
                let base = raw & 0xFFFF_FFF0;
                bars[index] = PciBar::Memory32 { base, prefetchable };
                index += 1;
            }
            0x2 => {
                let next_raw = if index + 1 < 6 {
                    config_read_u32(function_base, offset + 4)
                } else {
                    0
                };
                let base_low = (raw & 0xFFFF_FFF0) as u64;
                let base_high = (next_raw as u64) << 32;
                bars[index] = PciBar::Memory64 {
                    base: base_high | base_low,
                    prefetchable,
                };
                if index + 1 < 6 {
                    bars[index + 1] = PciBar::None;
                }
                index += 2;
            }
            _ => {
                bars[index] = PciBar::None;
                index += 1;
            }
        }
    }
    bars
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
