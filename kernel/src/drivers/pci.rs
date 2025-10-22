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
const MIN_IO_WINDOW: u64 = 0x1000; // 4 KiB
const MIN_MEM_WINDOW: u64 = 0x0010_0000; // 1 MiB

struct ResourceAllocator {
    next_io: u64,
    next_mem: u64,
    next_prefetch: u64,
}

impl ResourceAllocator {
    fn new() -> Self {
        Self {
            next_io: 0x1000,
            next_mem: 0x8000_0000,
            next_prefetch: 0x9000_0000,
        }
    }

    fn alloc_io(&mut self, size: u64) -> Option<u64> {
        let size = size.max(MIN_IO_WINDOW);
        let base = align_up(self.next_io, size);
        let end = base.checked_add(size)?.checked_sub(1)?;
        self.next_io = end.checked_add(1)?;
        Some(base)
    }

    fn alloc_mem(&mut self, size: u64) -> Option<u64> {
        let size = size.max(MIN_MEM_WINDOW);
        let base = align_up(self.next_mem, size);
        let end = base.checked_add(size)?.checked_sub(1)?;
        self.next_mem = end.checked_add(1)?;
        Some(base)
    }

    fn alloc_prefetch(&mut self, size: u64) -> Option<u64> {
        let size = size.max(MIN_MEM_WINDOW);
        let base = align_up(self.next_prefetch, size);
        let end = base.checked_add(size)?.checked_sub(1)?;
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

#[derive(Clone, Copy, Debug, Default)]
struct ResourceRange {
    start: u64,
    end: u64,
}

impl ResourceRange {
    fn new(base: u64, size: u64) -> Option<Self> {
        if size == 0 {
            return None;
        }
        let end = base.checked_add(size)?.checked_sub(1)?;
        Some(Self { start: base, end })
    }

    fn size(&self) -> u64 {
        self.end.saturating_sub(self.start).saturating_add(1)
    }

    fn merge(&mut self, other: &ResourceRange) {
        self.start = self.start.min(other.start);
        self.end = self.end.max(other.end);
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ResourceSummary {
    io: Option<ResourceRange>,
    mem: Option<ResourceRange>,
    pref_mem: Option<ResourceRange>,
}

impl ResourceSummary {
    fn include_io(&mut self, base: u64, size: u64) {
        if let Some(range) = ResourceRange::new(base, size) {
            match &mut self.io {
                Some(existing) => existing.merge(&range),
                None => self.io = Some(range),
            }
        }
    }

    fn include_mem(&mut self, base: u64, size: u64, prefetchable: bool) {
        if let Some(range) = ResourceRange::new(base, size) {
            let slot = if prefetchable {
                &mut self.pref_mem
            } else {
                &mut self.mem
            };
            match slot {
                Some(existing) => existing.merge(&range),
                None => *slot = Some(range),
            }
        }
    }

    fn merge(&mut self, other: &ResourceSummary) {
        if let Some(range) = other.io {
            self.include_io(range.start, range.size());
        }
        if let Some(range) = other.mem {
            self.include_mem(range.start, range.size(), false);
        }
        if let Some(range) = other.pref_mem {
            self.include_mem(range.start, range.size(), true);
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct FunctionScan {
    max_bus: u8,
    resources: ResourceSummary,
}
/// Decoded Base Address Register (BAR) entry.
#[derive(Clone, Copy, Debug)]
pub enum PciBar {
    None,
    Memory32 {
        base: u64,
        size: u64,
        prefetchable: bool,
        raw_flags: u32,
    },
    Memory64 {
        base: u64,
        size: u64,
        prefetchable: bool,
        raw_flags: u32,
    },
    Io {
        base: u64,
        size: u64,
        raw_flags: u32,
    },
}

impl PciBar {
    pub fn memory_base(&self) -> Option<u64> {
        match self {
            PciBar::Memory32 { base, .. } | PciBar::Memory64 { base, .. } => Some(*base),
            PciBar::Io { .. } | PciBar::None => None,
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

/// Enumerate all PCI functions accessible via the provided ECAM regions.
pub fn enumerate(regions: &[PciConfigRegion]) -> PciTopology {
    let mut devices = Vec::new();
    let mut bridges = Vec::new();
    let mut visited_buses: BTreeSet<(u16, u8)> = BTreeSet::new();
    let mut allocator = ResourceAllocator::new();

    for region in regions {
        let mut next_bus = region.bus_start.saturating_add(1);
        let _ = scan_bus(
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
) -> FunctionScan {
    if bus < region.bus_start || bus > region.bus_end {
        return FunctionScan {
            max_bus: bus,
            resources: ResourceSummary::default(),
        };
    }
    if !visited_buses.insert((region.segment, bus)) {
        return FunctionScan {
            max_bus: bus,
            resources: ResourceSummary::default(),
        };
    }

    let mut max_bus = bus;
    let mut resources = ResourceSummary::default();

    for device in 0..=MAX_DEVICE {
        if let Some(info0) = read_function(region, bus, device, 0) {
            let is_multi = info0.is_multi_function();
            let scan0 = handle_function(
                region,
                info0,
                visited_buses,
                devices,
                bridges,
                next_bus,
                alloc,
            );
            max_bus = max_bus.max(scan0.max_bus);
            resources.merge(&scan0.resources);

            if is_multi {
                for function in 1..=MAX_FUNCTION {
                    if let Some(extra) = read_function(region, bus, device, function) {
                        let scan = handle_function(
                            region,
                            extra,
                            visited_buses,
                            devices,
                            bridges,
                            next_bus,
                            alloc,
                        );
                        max_bus = max_bus.max(scan.max_bus);
                        resources.merge(&scan.resources);
                    }
                }
            }
        }
    }

    FunctionScan { max_bus, resources }
}

fn handle_function(
    region: &PciConfigRegion,
    mut info: PciDeviceInfo,
    visited_buses: &mut BTreeSet<(u16, u8)>,
    devices: &mut Vec<PciDeviceInfo>,
    bridges: &mut Vec<PciBridgeInfo>,
    next_bus: &mut u8,
    alloc: &mut ResourceAllocator,
) -> FunctionScan {
    let is_bridge = info.header_type & 0x7F == 0x01;

    let function_base = match function_base(region, info.bus, info.device, info.function) {
        Some(base) => base,
        None => {
            log_warn!(
                "PCI {:04x}:{:02x}:{:02x}.{} missing configuration base",
                info.segment,
                info.bus,
                info.device,
                info.function
            );
            devices.push(info);
            return FunctionScan {
                max_bus: info.bus,
                resources: ResourceSummary::default(),
            };
        }
    };

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
        let scan = configure_bridge(
            region,
            &mut info,
            function_base,
            visited_buses,
            devices,
            bridges,
            next_bus,
            alloc,
        );
        devices.push(info);
        scan
    } else {
        log_debug!(
            "PCI {:04x}:{:02x}:{:02x}.{} vendor={:04x} device={:04x} class={:02x}{:02x}{:02x}",
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

        let resources = configure_endpoint(function_base, &mut info, alloc);
        devices.push(info);
        FunctionScan {
            max_bus: info.bus,
            resources,
        }
    }
}

fn configure_endpoint(
    function_base: u64,
    info: &mut PciDeviceInfo,
    alloc: &mut ResourceAllocator,
) -> ResourceSummary {
    let mut summary = ResourceSummary::default();
    let mut index = 0usize;
    let mut enable_io = false;

    while index < info.bars.len() {
        let offset = 0x10 + (index as u8) * 4;
        match info.bars[index] {
            PciBar::Io {
                size,
                raw_flags,
                ..
            } => {
                if size > 0 {
                    if let Some(base) = alloc.alloc_io(size) {
                        let new_value = ((base as u32) & 0xFFFF_FFFC) | raw_flags;
                        config_write_u32(function_base, offset, new_value);
                        info.bars[index] = PciBar::Io {
                            base,
                            size,
                            raw_flags,
                        };
                        summary.include_io(base, size);
                        enable_io = true;
                    } else {
                        log_warn!(
                            "PCI {:04x}:{:02x}:{:02x}.{} failed to allocate IO window of {:#x} byte(s)",
                            info.segment,
                            info.bus,
                            info.device,
                            info.function,
                            size
                        );
                    }
                }
                index += 1;
            }
            PciBar::Memory32 {
                size,
                prefetchable,
                raw_flags,
                ..
            } => {
                if size > 0 {
                    let base_opt = if prefetchable {
                        alloc.alloc_prefetch(size)
                    } else {
                        alloc.alloc_mem(size)
                    };
                    if let Some(base) = base_opt {
                        let new_value = ((base as u32) & 0xFFFF_FFF0) | raw_flags;
                        config_write_u32(function_base, offset, new_value);
                        info.bars[index] = PciBar::Memory32 {
                            base,
                            size,
                            prefetchable,
                            raw_flags,
                        };
                        summary.include_mem(base, size, prefetchable);
                    } else {
                        log_warn!(
                            "PCI {:04x}:{:02x}:{:02x}.{} failed to allocate memory window of {:#x} byte(s)",
                            info.segment,
                            info.bus,
                            info.device,
                            info.function,
                            size
                        );
                    }
                }
                index += 1;
            }
            PciBar::Memory64 {
                size,
                prefetchable,
                raw_flags,
                ..
            } => {
                if size > 0 {
                    let base_opt = if prefetchable {
                        alloc.alloc_prefetch(size)
                    } else {
                        alloc.alloc_mem(size)
                    };
                    if let Some(base) = base_opt {
                        let new_low = ((base as u32) & 0xFFFF_FFF0) | raw_flags;
                        let new_high = (base >> 32) as u32;
                        config_write_u32(function_base, offset, new_low);
                        config_write_u32(function_base, offset + 4, new_high);
                        info.bars[index] = PciBar::Memory64 {
                            base,
                            size,
                            prefetchable,
                            raw_flags,
                        };
                        if index + 1 < info.bars.len() {
                            info.bars[index + 1] = PciBar::None;
                        }
                        summary.include_mem(base, size, prefetchable);
                    } else {
                        log_warn!(
                            "PCI {:04x}:{:02x}:{:02x}.{} failed to allocate memory window of {:#x} byte(s)",
                            info.segment,
                            info.bus,
                            info.device,
                            info.function,
                            size
                        );
                    }
                }
                index += 2; // 64-bit BAR consumes two slots
            }
            PciBar::None => {
                index += 1;
            }
        }
    }

    let mut command = config_read_u16(function_base, 0x04);
    let has_mem = summary.mem.is_some() || summary.pref_mem.is_some();
    if enable_io {
        command |= 0x0001;
    }
    if has_mem {
        command |= 0x0002; // memory space
    }
    if enable_io || has_mem {
        command |= 0x0004; // bus master
    }
    config_write_u16(function_base, 0x04, command);
    info.command = command;

    summary
}

fn configure_bridge(
    region: &PciConfigRegion,
    info: &mut PciDeviceInfo,
    function_base: u64,
    visited_buses: &mut BTreeSet<(u16, u8)>,
    devices: &mut Vec<PciDeviceInfo>,
    bridges: &mut Vec<PciBridgeInfo>,
    next_bus: &mut u8,
    alloc: &mut ResourceAllocator,
) -> FunctionScan {
    let mut secondary = config_read_u8(function_base, 0x19);
    let mut subordinate = config_read_u8(function_base, 0x1A);
    let mut command = config_read_u16(function_base, 0x04);
    let desired_command = command | 0x0007;
    if desired_command != command {
        config_write_u16(function_base, 0x04, desired_command);
        command = desired_command;
    }
    info.command = command;

    if secondary == 0
        || secondary <= info.bus
        || visited_buses.contains(&(region.segment, secondary))
    {
        if let Some(new_bus) = allocate_bus(region, next_bus, visited_buses) {
            secondary = new_bus;
            subordinate = region.bus_end;
            config_write_u8(function_base, 0x19, secondary);
            config_write_u8(function_base, 0x1A, subordinate);
        } else {
            log_warn!(
                "PCI bridge {:04x}:{:02x}:{:02x}.{}: unable to allocate secondary bus",
                info.segment,
                info.bus,
                info.device,
                info.function
            );
            return FunctionScan {
                max_bus: info.bus,
                resources: ResourceSummary::default(),
            };
        }
    }

    if subordinate < secondary {
        subordinate = region.bus_end;
        config_write_u8(function_base, 0x1A, subordinate);
    }

    // Pulse secondary bus reset so devices re-enumerate.
    let bridge_control = config_read_u16(function_base, 0x3E);
    config_write_u16(function_base, 0x3E, bridge_control | 0x0400);
    config_write_u16(function_base, 0x3E, bridge_control & !0x0400);

    let child_scan = scan_bus(
        region,
        secondary,
        visited_buses,
        devices,
        bridges,
        next_bus,
        alloc,
    );

    if child_scan.max_bus > subordinate {
        subordinate = child_scan.max_bus;
        config_write_u8(function_base, 0x1A, subordinate);
    }

    program_bridge_windows(function_base, &child_scan.resources);

    bridges.push(PciBridgeInfo {
        segment: info.segment,
        bus: info.bus,
        device: info.device,
        function: info.function,
        secondary_bus: secondary,
        subordinate_bus: subordinate,
        vendor_id: info.vendor_id,
        device_id: info.device_id,
        max_child_bus: child_scan.max_bus,
    });

    FunctionScan {
        max_bus: child_scan.max_bus.max(info.bus),
        resources: child_scan.resources,
    }
}

fn program_bridge_windows(function_base: u64, resources: &ResourceSummary) {
    program_mem_window(function_base, resources.mem);
    program_prefetch_window(function_base, resources.pref_mem);
    program_io_window(function_base, resources.io);
}

fn program_mem_window(function_base: u64, range: Option<ResourceRange>) {
    if let Some(range) = range {
        let base = range.start;
        let limit = range.end;
        let base_reg = ((base >> 16) & 0xFFF0) as u16;
        let limit_reg = ((limit >> 16) & 0xFFF0) as u16;
        config_write_u16(function_base, 0x20, base_reg);
        config_write_u16(function_base, 0x22, limit_reg);
    } else {
        config_write_u16(function_base, 0x20, 0xFFF0);
        config_write_u16(function_base, 0x22, 0x0000);
    }
}

fn program_prefetch_window(function_base: u64, range: Option<ResourceRange>) {
    if let Some(range) = range {
        let base = range.start;
        let limit = range.end;
        let base_low = ((base >> 16) & 0xFFF0) as u16;
        let limit_low = ((limit >> 16) & 0xFFF0) as u16;
        let base_high = (base >> 32) as u32;
        let limit_high = (limit >> 32) as u32;
        config_write_u16(function_base, 0x24, base_low);
        config_write_u16(function_base, 0x26, limit_low);
        config_write_u32(function_base, 0x28, base_high);
        config_write_u32(function_base, 0x2C, limit_high);
    } else {
        config_write_u16(function_base, 0x24, 0x0000);
        config_write_u16(function_base, 0x26, 0x0000);
        config_write_u32(function_base, 0x28, 0);
        config_write_u32(function_base, 0x2C, 0);
    }
}

fn program_io_window(function_base: u64, range: Option<ResourceRange>) {
    if let Some(_range) = range {
        // TODO: Implement 32-bit I/O window programming when needed.
        // For now disable the window to avoid stale settings.
    }
    config_write_u8(function_base, 0x1C, 0xF0);
    config_write_u8(function_base, 0x1D, 0x00);
    config_write_u16(function_base, 0x30, 0x0000);
    config_write_u16(function_base, 0x32, 0x0000);
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
    let original_command = config_read_u16(function_base, 0x04);
    let decode_mask = 0x0007; // IO, memory, bus mastering
    let had_decode_enabled = (original_command & decode_mask) != 0;
    if had_decode_enabled {
        config_write_u16(function_base, 0x04, original_command & !decode_mask);
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
            let size = probe_bar_size(function_base, offset, raw, BarKind::Io);
            bars[index] = PciBar::Io {
                base,
                size,
                raw_flags: raw & 0x3,
            };
            index += 1;
            continue;
        }

        let prefetchable = (raw & (1 << 3)) != 0;
        let bar_type = (raw >> 1) & 0x3;
        match bar_type {
            0x0 => {
                let base = (raw & 0xFFFF_FFF0) as u64;
                let size = probe_bar_size(function_base, offset, raw, BarKind::Mem32);
                bars[index] = if size == 0 {
                    PciBar::None
                } else {
                    PciBar::Memory32 {
                        base,
                        size,
                        prefetchable,
                        raw_flags: raw & 0xF,
                    }
                };
                index += 1;
            }
            0x2 => {
                if index + 1 >= 6 {
                    bars[index] = PciBar::None;
                    index += 1;
                    continue;
                }
                let raw_high = config_read_u32(function_base, offset + 4);
                let base_low = (raw & 0xFFFF_FFF0) as u64;
                let base_high = (raw_high as u64) << 32;
                let base = base_high | base_low;
                let size = probe_bar_size64(function_base, offset, raw, raw_high);
                if size == 0 {
                    bars[index] = PciBar::None;
                    bars[index + 1] = PciBar::None;
                    index += 2;
                    continue;
                }
                bars[index] = PciBar::Memory64 {
                    base,
                    size,
                    prefetchable,
                    raw_flags: raw & 0xF,
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
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u16, value) };
}

fn config_write_u8(function_base: u64, offset: u8, value: u8) {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u8, value) };
}

#[derive(Clone, Copy, Debug)]
enum BarKind {
    Io,
    Mem32,
}

fn probe_bar_size(function_base: u64, offset: u8, original: u32, kind: BarKind) -> u64 {
    config_write_u32(function_base, offset, 0xFFFF_FFFF);
    let raw_mask = config_read_u32(function_base, offset);
    config_write_u32(function_base, offset, original);

    let mask = match kind {
        BarKind::Io => raw_mask & 0xFFFF_FFFC,
        BarKind::Mem32 => raw_mask & 0xFFFF_FFF0,
    };

    if mask == 0 {
        return 0;
    }

    let size = (!mask).wrapping_add(1);
    let align_mask = match kind {
        BarKind::Io => 0xFFFF_FFFC_u32,
        BarKind::Mem32 => 0xFFFF_FFF0_u32,
    };
    (size as u64) & align_mask as u64
}

fn probe_bar_size64(function_base: u64, offset: u8, original_low: u32, original_high: u32) -> u64 {
    config_write_u32(function_base, offset, 0xFFFF_FFFF);
    config_write_u32(function_base, offset + 4, 0xFFFF_FFFF);
    let low_mask = config_read_u32(function_base, offset) & 0xFFFF_FFF0;
    let high_mask = config_read_u32(function_base, offset + 4);
    config_write_u32(function_base, offset, original_low);
    config_write_u32(function_base, offset + 4, original_high);

    let mask = ((high_mask as u64) << 32) | (low_mask as u64);
    if mask == 0 || mask == 0xFFFF_FFFF_FFFFFFF0 {
        return 0;
    }

    (!mask).wrapping_add(1) & 0xFFFF_FFFF_FFFFFFF0
}
