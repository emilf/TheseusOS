//! PCI Express enhanced configuration access (ECAM) support.
//!
//! This module reads PCIe configuration space using the MMIO windows described
//! by the ACPI MCFG table. It assumes the boot environment has exposed those
//! regions via `PlatformInfo::pci_config_regions`.

use crate::acpi::{PciBridgeInfo, PciConfigRegion};
use crate::{log_debug, log_trace, log_warn};
use alloc::collections::BTreeSet;
use alloc::vec::Vec;
use core::fmt;

const MAX_DEVICE: u8 = 31;
const MAX_FUNCTION: u8 = 7;

const BUS_STRIDE: u64 = 0x1_0000; // 1 MiB per bus
const DEVICE_STRIDE: u64 = 0x8000; // 32 KiB per device
const FUNCTION_STRIDE: u64 = 0x1000; // 4 KiB per function
const MEM_WINDOW_SIZE: u32 = 0x0100_0000; // 16 MiB
const PREF_WINDOW_SIZE: u32 = 0x0100_0000; // 16 MiB

struct ResourceAllocator {
    next_mem: u64,
    next_prefetch: u64,
}

impl ResourceAllocator {
    fn new() -> Self {
        Self {
            next_mem: 0x8000_0000,
            next_prefetch: 0x9000_0000,
        }
    }

    fn alloc_mem_window(&mut self, size: u32) -> Option<u64> {
        let size = size.max(0x0010_0000);
        let base = align_up(self.next_mem, size as u64);
        let end = base.checked_add(size as u64)?.checked_sub(1)?;
        self.next_mem = end.checked_add(1)?;
        Some(base)
    }

    fn alloc_prefetch_window(&mut self, size: u32) -> Option<u64> {
        let size = size.max(0x0010_0000);
        let base = align_up(self.next_prefetch, size as u64);
        let end = base.checked_add(size as u64)?.checked_sub(1)?;
        self.next_prefetch = end.checked_add(1)?;
        Some(base)
    }
}

fn align_up(value: u64, alignment: u64) -> u64 {
    if alignment == 0 {
        return value;
    }
    ((value + alignment - 1) / alignment) * alignment
}

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

/// Enumerate all PCI functions accessible via the provided ECAM regions.
pub fn enumerate(regions: &[PciConfigRegion]) -> PciTopology {
    let mut devices = Vec::new();
    let mut bridges = Vec::new();
    let mut visited_buses: BTreeSet<(u16, u8)> = BTreeSet::new();
    let mut allocator = ResourceAllocator::new();

    for region in regions {
        let mut next_bus = region.bus_start.saturating_add(1);
        scan_bus(
            region,
            region.bus_start,
            &mut visited_buses,
            &mut devices,
            &mut bridges,
            &mut next_bus,
            &mut allocator,
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
    next_bus: &mut u8,
    alloc: &mut ResourceAllocator,
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
            let is_bridge = info.header_type & 0x7F == 0x01;
            if is_bridge {
                log_trace!(
                    "PCI bridge {:04x}:{:02x}:{:02x}.0 vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
                    info.segment,
                    info.bus,
                    info.device,
                    info.vendor_id,
                    info.device_id,
                    info.class_code,
                    info.subclass,
                    info.prog_if
                );
            } else {
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
            }
            devices.push(info);

            if info.is_multi_function() {
                for function in 1..=MAX_FUNCTION {
                    if let Some(extra) = read_function(region, bus, device, function) {
                        let is_bridge_fn = extra.header_type & 0x7F == 0x01;
                        if is_bridge_fn {
                            log_trace!(
                                "PCI bridge {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
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
                        } else {
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
                        }
                        devices.push(extra);
                    }
                }
            }

            if is_bridge {
                if let Some(base) = function_base(region, bus, device, 0) {
                    if config_read_u8(base, 0x18) != bus {
                        config_write_u8(base, 0x18, bus);
                    }
                    let mut secondary = config_read_u8(base, 0x19);
                    let subordinate_reg = config_read_u8(base, 0x1A);
                    let command = config_read_u16(base, 0x04);

                    let desired_command = command | 0x0007; // I/O, Memory, Bus Master
                    if desired_command != command {
                        config_write_u16(base, 0x04, desired_command);
                    }

                    if secondary == 0
                        || secondary <= bus
                        || visited_buses.contains(&(region.segment, secondary))
                    {
                        if let Some(new_bus) = allocate_bus(region, next_bus, visited_buses) {
                            secondary = new_bus;
                            config_write_u8(base, 0x19, secondary);
                            config_write_u8(base, 0x1A, region.bus_end);
                        } else {
                            log_warn!(
                                "PCI bridge {:04x}:{:02x}:{:02x}.{}: unable to allocate secondary bus",
                                info.segment,
                                info.bus,
                                info.device,
                                info.function
                            );
                            continue;
                        }
                    } else if subordinate_reg < secondary {
                        config_write_u8(base, 0x1A, region.bus_end);
                    }

                    // Pulse Secondary Bus Reset so devices behind the bridge re-enumerate.
                    let bridge_control = config_read_u16(base, 0x3E);
                    config_write_u16(base, 0x3E, bridge_control | 0x0400);
                    config_write_u16(base, 0x3E, bridge_control & !0x0400);

                    let child_max = scan_bus(
                        region,
                        secondary,
                        visited_buses,
                        devices,
                        bridges,
                        next_bus,
                        alloc,
                    );

                    let new_subordinate = child_max.max(secondary);
                    if subordinate_reg != new_subordinate {
                        config_write_u8(base, 0x1A, new_subordinate);
                    }

                    if child_max >= secondary {
                        if let Some(mem_base) = alloc.alloc_mem_window(MEM_WINDOW_SIZE) {
                            if !program_memory_window(base, mem_base, MEM_WINDOW_SIZE) {
                                log_warn!(
                                    "PCI bridge {:04x}:{:02x}:{:02x}.{}: failed to program memory window",
                                    info.segment,
                                    info.bus,
                                    info.device,
                                    info.function
                                );
                            }
                        } else {
                            log_warn!(
                                "PCI bridge {:04x}:{:02x}:{:02x}.{}: out of memory window space",
                                info.segment,
                                info.bus,
                                info.device,
                                info.function
                            );
                        }

                        if let Some(pref_base) = alloc.alloc_prefetch_window(PREF_WINDOW_SIZE) {
                            program_prefetch_window(base, pref_base, PREF_WINDOW_SIZE);
                        } else {
                            log_warn!(
                                "PCI bridge {:04x}:{:02x}:{:02x}.{}: out of prefetch window space",
                                info.segment,
                                info.bus,
                                info.device,
                                info.function
                            );
                        }
                    }

                    bridges.push(PciBridgeInfo {
                        segment: info.segment,
                        bus: info.bus,
                        device: info.device,
                        function: info.function,
                        secondary_bus: secondary,
                        subordinate_bus: new_subordinate,
                        vendor_id: info.vendor_id,
                        device_id: info.device_id,
                        max_child_bus: child_max,
                    });

                    if child_max > max_bus {
                        max_bus = child_max;
                    }
                } else {
                    log_warn!(
                        "PCI bridge {:04x}:{:02x}:{:02x}.{} missing function base",
                        info.segment,
                        info.bus,
                        info.device,
                        info.function
                    );
                }
            } else if info.bus > max_bus {
                max_bus = info.bus;
            }
        }
    }

    max_bus
}

fn allocate_bus(
    region: &PciConfigRegion,
    next_bus: &mut u8,
    visited: &BTreeSet<(u16, u8)>,
) -> Option<u8> {
    let mut candidate = *next_bus;
    while candidate <= region.bus_end {
        *next_bus = candidate.saturating_add(1);
        if candidate != 0 && !visited.contains(&(region.segment, candidate)) {
            return Some(candidate);
        }
        if candidate == u8::MAX {
            break;
        }
        candidate = *next_bus;
    }
    None
}

fn program_memory_window(base: u64, window_base: u64, size: u32) -> bool {
    let limit = match window_base
        .checked_add(size as u64)
        .and_then(|v| v.checked_sub(1))
    {
        Some(lim) => lim,
        None => return false,
    };
    let base_reg = ((window_base >> 16) & 0xFFF0) as u16;
    let limit_reg = ((limit >> 16) & 0xFFF0) as u16;
    config_write_u16(base, 0x20, base_reg);
    config_write_u16(base, 0x22, limit_reg);
    true
}

fn program_prefetch_window(base: u64, window_base: u64, size: u32) {
    let limit = window_base
        .checked_add(size as u64)
        .and_then(|v| v.checked_sub(1))
        .unwrap_or(window_base);
    let base_reg = ((window_base >> 16) & 0xFFF0) as u16;
    let limit_reg = ((limit >> 16) & 0xFFF0) as u16;
    config_write_u16(base, 0x24, base_reg);
    config_write_u16(base, 0x26, limit_reg);
    // Clear upper 32-bit registers for 64-bit windows for now.
    config_write_u32(base, 0x28, 0);
    config_write_u32(base, 0x2C, 0);
    // Disable I/O window until we have an allocator for it.
    config_write_u16(base, 0x1C, 0xF0);
    config_write_u16(base, 0x1E, 0x00);
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
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u16, value) };
}

fn config_write_u8(function_base: u64, offset: u8, value: u8) {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u8, value) };
}
