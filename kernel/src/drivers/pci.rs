//! PCI Express enhanced configuration access (ECAM) support.
//!
//! This module provides comprehensive PCI/PCIe device enumeration and configuration
//! using the Enhanced Configuration Access Mechanism (ECAM). It reads PCIe configuration
//! space using the MMIO windows described by the ACPI MCFG table.
//!
//! ## Key Features
//!
//! - **Device Enumeration**: Scans all PCI buses and discovers all functions
//! - **Bridge Handling**: Automatically configures PCI bridges and discovers downstream devices
//! - **BAR Parsing**: Reads and decodes Base Address Registers (BARs) for memory/I/O resources
//! - **Capability Discovery**: Parses PCI capabilities like MSI/MSI-X
//! - **MSI Support**: Configures Message Signaled Interrupts for devices
//!
//! ## Architecture Overview
//!
//! The PCI subsystem follows this flow:
//! 1. **ACPI Discovery**: MCFG table provides ECAM regions
//! 2. **Bus Scanning**: Recursively scan buses starting from bus 0
//! 3. **Device Discovery**: Read configuration space for each function
//! 4. **Bridge Configuration**: Set up secondary/subordinate buses for bridges
//! 5. **Resource Parsing**: Decode BARs and capabilities
//! 6. **Device Registration**: Create device descriptors for the driver framework
//!
//! ## ECAM Addressing
//!
//! PCIe uses a flat address space where each function has a 4KB configuration space:
//! - Bus stride: 1MB (0x1_0000)
//! - Device stride: 32KB (0x8000)  
//! - Function stride: 4KB (0x1000)
//!
//! Address = ECAM_base + (bus * 1MB) + (device * 32KB) + (function * 4KB)
//!
//! ## Example Usage
//!
//! ```rust,no_run
//! // Enumerate all PCI devices
//! let topology = pci::enumerate(&platform_info.pci_config_regions);
//! 
//! // Find a specific device
//! for device in topology.functions.iter() {
//!     if device.vendor_id == 0x8086 && device.device_id == 0x1234 {
//!         // Found our device
//!     }
//! }
//! 
//! // Enable MSI for a device
//! pci::enable_msi(&device, &regions, apic_id, vector)?;
//! ```

use crate::acpi::{PciBridgeInfo, PciConfigRegion};
use crate::{log_debug, log_trace, log_warn};
use alloc::collections::BTreeSet;
use alloc::vec::Vec;
use core::fmt;

// PCI configuration space constants
const MAX_DEVICE: u8 = 31;    // Maximum device number (0-31)
const MAX_FUNCTION: u8 = 7;   // Maximum function number (0-7)

// ECAM addressing constants (PCIe specification)
const BUS_STRIDE: u64 = 0x1_0000;     // 1 MiB per bus
const DEVICE_STRIDE: u64 = 0x8000;    // 32 KiB per device  
const FUNCTION_STRIDE: u64 = 0x1000; // 4 KiB per function

// MSI configuration constants
const MSI_ADDR_BASE: u32 = 0xFEE0_0000; // MSI message address base

/// Decoded Base Address Register (BAR) entry.
///
/// PCI devices use BARs to declare their memory and I/O resource requirements.
/// This enum represents the different types of BARs that can be found in a
/// PCI device's configuration space.
///
/// # BAR Types
///
/// - **Memory32**: 32-bit memory space BAR (up to 4GB)
/// - **Memory64**: 64-bit memory space BAR (up to 16EB) 
/// - **Io**: I/O space BAR (up to 64KB)
/// - **None**: Unused or invalid BAR
///
/// # Prefetchable Memory
///
/// Memory BARs can be marked as prefetchable, which means the device can
/// safely prefetch data from the memory region. This allows for optimizations
/// like write-combining and read-ahead.
#[derive(Clone, Copy, Debug)]
pub enum PciBar {
    /// Unused or invalid BAR
    None,
    /// 32-bit memory space BAR
    Memory32 {
        /// Base address of the memory region
        base: u64,
        /// Size of the memory region in bytes
        size: u64,
        /// Whether this memory region is prefetchable
        prefetchable: bool,
    },
    /// 64-bit memory space BAR (uses two consecutive BAR slots)
    Memory64 {
        /// Base address of the memory region
        base: u64,
        /// Size of the memory region in bytes
        size: u64,
        /// Whether this memory region is prefetchable
        prefetchable: bool,
    },
    /// I/O space BAR
    Io {
        /// Base I/O port address
        base: u64,
        /// Size of the I/O region in bytes
        size: u64,
    },
}

impl PciBar {
    /// Get the base address if this is a memory BAR.
    ///
    /// # Returns
    /// * `Some(u64)` - Base address for memory BARs
    /// * `None` - For I/O BARs or unused BARs
    pub fn memory_base(&self) -> Option<u64> {
        match self {
            PciBar::Memory32 { base, .. } | PciBar::Memory64 { base, .. } => Some(*base),
            PciBar::Io { .. } | PciBar::None => None,
        }
    }

    /// Get the size of the BAR region.
    ///
    /// # Returns
    /// * `Some(u64)` - Size in bytes for valid BARs
    /// * `None` - For unused BARs
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
///
/// PCI devices can advertise various capabilities through a linked list
/// structure in their configuration space. This struct tracks the most
/// commonly used capabilities for interrupt handling.
///
/// # Capabilities Tracked
///
/// - **MSI**: Message Signaled Interrupts (legacy)
/// - **MSI-X**: Extended Message Signaled Interrupts (preferred)
/// - **Pointers**: Offset into configuration space where capability is located
#[derive(Clone, Copy, Debug, Default)]
pub struct PciCapabilities {
    /// Whether the device supports MSI
    pub msi: bool,
    /// Whether the device supports MSI-X
    pub msix: bool,
    /// Offset to MSI capability structure (if present)
    pub msi_pointer: Option<u8>,
    /// Offset to MSI-X capability structure (if present)
    pub msix_pointer: Option<u8>,
}

/// Summary of a discovered PCI function.
///
/// This structure contains all the essential information about a PCI device
/// function that was discovered during enumeration. It includes identification
/// information, resource requirements, and capabilities.
///
/// # Device Identification
///
/// Each PCI function is uniquely identified by its segment, bus, device, and
/// function numbers. The vendor ID and device ID provide hardware identification.
///
/// # Resource Information
///
/// The BARs array contains up to 6 Base Address Registers that describe the
/// device's memory and I/O resource requirements. The interrupt line and pin
/// provide legacy interrupt routing information.
///
/// # Capabilities
///
/// The capabilities field indicates which advanced features the device supports,
/// particularly for interrupt handling (MSI/MSI-X).
#[derive(Clone, Copy)]
pub struct PciDeviceInfo {
    /// PCI segment number (for multi-segment systems)
    pub segment: u16,
    /// PCI bus number
    pub bus: u8,
    /// PCI device number (0-31)
    pub device: u8,
    /// PCI function number (0-7)
    pub function: u8,
    /// Vendor ID (assigned by PCI SIG)
    pub vendor_id: u16,
    /// Device ID (assigned by vendor)
    pub device_id: u16,
    /// Device class code (high byte)
    pub class_code: u8,
    /// Device subclass code (middle byte)
    pub subclass: u8,
    /// Programming interface (low byte)
    pub prog_if: u8,
    /// Device revision ID
    pub revision_id: u8,
    /// Header type (0=standard, 1=bridge, 2=cardbus)
    pub header_type: u8,
    /// Command register value
    pub command: u16,
    /// Status register value
    pub status: u16,
    /// Base Address Registers (up to 6)
    pub bars: [PciBar; 6],
    /// Legacy interrupt line assignment
    pub interrupt_line: u8,
    /// Legacy interrupt pin assignment
    pub interrupt_pin: u8,
    /// Device capabilities
    pub capabilities: PciCapabilities,
}

impl PciDeviceInfo {
    /// Check if this device has multiple functions.
    ///
    /// PCI devices can have multiple functions (0-7). If bit 7 of the header
    /// type is set, the device has multiple functions and we need to scan
    /// all function numbers.
    ///
    /// # Returns
    /// * `true` - Device has multiple functions
    /// * `false` - Device has only function 0
    pub fn is_multi_function(&self) -> bool {
        self.header_type & 0x80 != 0
    }

    /// Get the device class as a tuple.
    ///
    /// # Returns
    /// A tuple of (class_code, subclass, prog_if) for easy matching
    pub fn class_triplet(&self) -> (u8, u8, u8) {
        (self.class_code, self.subclass, self.prog_if)
    }

    /// Get the interrupt line if valid.
    ///
    /// # Returns
    /// * `Some(u8)` - Valid interrupt line (0-15)
    /// * `None` - No interrupt line assigned (0xFF)
    pub fn interrupt_line(&self) -> Option<u8> {
        match self.interrupt_line {
            0xFF => None,
            value => Some(value),
        }
    }

    /// Get the first memory BAR from the device.
    ///
    /// This is a convenience method to find the first memory-mapped BAR,
    /// which is often the primary MMIO region for the device.
    ///
    /// # Returns
    /// * `Some(PciBar)` - First memory BAR found
    /// * `None` - No memory BARs present
    pub fn first_memory_bar(&self) -> Option<PciBar> {
        self.bars
            .iter()
            .copied()
            .find(|bar| matches!(bar, PciBar::Memory32 { .. } | PciBar::Memory64 { .. }))
    }

    /// Get the MSI capability pointer if present.
    ///
    /// # Returns
    /// * `Some(u8)` - Offset to MSI capability structure
    /// * `None` - MSI not supported
    pub fn msi_capability(&self) -> Option<u8> {
        self.capabilities.msi_pointer
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

/// PCI topology information discovered during enumeration.
///
/// This structure contains the complete PCI topology discovered during
/// enumeration, including all functions and bridges found in the system.
pub struct PciTopology {
    /// All discovered PCI functions
    pub functions: Vec<PciDeviceInfo>,
    /// All discovered PCI bridges
    pub bridges: Vec<PciBridgeInfo>,
}

/// Enumerate all PCI functions accessible via the provided ECAM regions.
///
/// This is the main entry point for PCI device discovery. It scans all
/// ECAM regions provided by the ACPI MCFG table and discovers all PCI
/// devices, bridges, and functions in the system.
///
/// # Arguments
/// * `regions` - Array of ECAM regions from ACPI MCFG table
///
/// # Returns
/// * `PciTopology` - Complete topology including all devices and bridges
///
/// # Algorithm
/// 1. For each ECAM region, start scanning from the base bus
/// 2. Recursively scan buses, handling PCI bridges
/// 3. For each device, read configuration space and decode BARs
/// 4. Parse capabilities and build device information
/// 5. Return complete topology
///
/// # Bridge Handling
/// The function automatically configures PCI bridges by:
/// - Setting up secondary and subordinate bus numbers
/// - Enabling I/O, memory, and bus master capabilities
/// - Performing secondary bus reset to re-enumerate devices
/// - Recursively scanning downstream buses
pub fn enumerate(regions: &[PciConfigRegion]) -> PciTopology {
    let mut devices = Vec::new();
    let mut bridges = Vec::new();
    let mut visited_buses: BTreeSet<(u16, u8)> = BTreeSet::new();

    // Scan each ECAM region
    for region in regions {
        let mut next_bus = region.bus_start.saturating_add(1);
        scan_bus(
            region,
            region.bus_start,
            &mut visited_buses,
            &mut devices,
            &mut bridges,
            &mut next_bus,
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
                    let mut subordinate = config_read_u8(base, 0x1A);
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
                            subordinate = region.bus_end;
                            config_write_u8(base, 0x19, secondary);
                            config_write_u8(base, 0x1A, subordinate);
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
                    }

                    if subordinate < secondary {
                        subordinate = region.bus_end;
                        config_write_u8(base, 0x1A, subordinate);
                    }

                    // Pulse Secondary Bus Reset so devices behind the bridge re-enumerate.
                    let bridge_control = config_read_u16(base, 0x3E);
                    config_write_u16(base, 0x3E, bridge_control | 0x0400);
                    config_write_u16(base, 0x3E, bridge_control & !0x0400);

                    let child_max =
                        scan_bus(region, secondary, visited_buses, devices, bridges, next_bus);
                    if child_max > subordinate {
                        subordinate = child_max;
                        config_write_u8(base, 0x1A, subordinate);
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
                        io_window: None,
                        mem_window: None,
                        pref_mem_window: None,
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
    let decode_mask = 0x0007; // IO space | memory space | bus master
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
            config_write_u32(function_base, offset, 0xFFFF_FFFC);
            let mask = config_read_u32(function_base, offset) & 0xFFFF_FFFC;
            config_write_u32(function_base, offset, raw);
            let size = if mask == 0 || mask == 0xFFFF_FFFC {
                0
            } else {
                (!mask).wrapping_add(1) as u64 & 0xFFFF_FFFC
            };
            bars[index] = if size == 0 {
                PciBar::None
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
                config_write_u32(function_base, offset, 0xFFFF_FFF0);
                let mask = config_read_u32(function_base, offset) & 0xFFFF_FFF0;
                config_write_u32(function_base, offset, raw);
                let size = if mask == 0 || mask == 0xFFFF_FFF0 {
                    0
                } else {
                    (!mask).wrapping_add(1) as u64 & 0xFFFF_FFF0
                };
                let base = (raw & 0xFFFF_FFF0) as u64;
                bars[index] = if size == 0 {
                    PciBar::None
                } else {
                    PciBar::Memory32 {
                        base,
                        size,
                        prefetchable,
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
                config_write_u32(function_base, offset, 0xFFFF_FFF0);
                config_write_u32(function_base, offset + 4, 0xFFFF_FFFF);
                let mask_low = config_read_u32(function_base, offset) & 0xFFFF_FFF0;
                let mask_high = config_read_u32(function_base, offset + 4);
                config_write_u32(function_base, offset, raw);
                config_write_u32(function_base, offset + 4, raw_high);
                let mask = ((mask_high as u64) << 32) | (mask_low as u64);
                let size = if mask == 0 || mask == 0xFFFF_FFFF_FFFFFFF0 {
                    0
                } else {
                    (!mask).wrapping_add(1) & 0xFFFF_FFFF_FFFFFFF0
                };
                let base_low = (raw & 0xFFFF_FFF0) as u64;
                let base_high = (raw_high as u64) << 32;
                let base = base_high | base_low;
                if size == 0 {
                    bars[index] = PciBar::None;
                    bars[index + 1] = PciBar::None;
                } else {
                    bars[index] = PciBar::Memory64 {
                        base,
                        size,
                        prefetchable,
                    };
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
        let current = pointer;
        let addr = function_base + current as u64;
        let cap_id = unsafe { core::ptr::read_volatile(addr as *const u8) };
        let next = unsafe { core::ptr::read_volatile((addr + 1) as *const u8) };
        match cap_id {
            0x05 => {
                caps.msi = true;
                caps.msi_pointer = Some(current);
            }
            0x11 => {
                caps.msix = true;
                caps.msix_pointer = Some(current);
            }
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

/// Enable MSI for the given device using the supplied APIC destination ID and vector.
///
/// This function configures a PCI device to use Message Signaled Interrupts (MSI)
/// instead of legacy interrupt routing. MSI provides better performance and
/// scalability compared to legacy interrupts.
///
/// # Arguments
/// * `device` - PCI device information (must have MSI capability)
/// * `regions` - ECAM regions for accessing device configuration space
/// * `apic_id` - Destination APIC ID for the interrupt
/// * `vector` - Interrupt vector number (0-255)
///
/// # Returns
/// * `Ok(())` - MSI successfully enabled
/// * `Err(&'static str)` - Error message if MSI setup failed
///
/// # Prerequisites
/// The caller must ensure:
/// - An interrupt vector is reserved in the system
/// - Any legacy interrupt routing (IOAPIC) is masked
/// - The device supports MSI (check `device.capabilities.msi`)
///
/// # MSI Configuration
/// This function programs the MSI capability structure with:
/// - Message address: APIC base + (apic_id << 12)
/// - Message data: Interrupt vector
/// - Control register: MSI enabled, single vector
///
/// # Example
/// ```rust,no_run
/// // Check if device supports MSI
/// if device.capabilities.msi {
///     // Enable MSI with vector 32 for APIC 0
///     pci::enable_msi(&device, &regions, 0, 32)?;
/// }
/// ```
pub fn enable_msi(
    device: &PciDeviceInfo,
    regions: &[PciConfigRegion],
    apic_id: u8,
    vector: u8,
) -> Result<(), &'static str> {
    // Find the MSI capability pointer
    let cap_ptr = device
        .capabilities
        .msi_pointer
        .ok_or("MSI capability not present")?;

    // Find the ECAM region for this device
    let mut config_base = None;
    for region in regions {
        if region.segment != device.segment {
            continue;
        }
        if device.bus < region.bus_start || device.bus > region.bus_end {
            continue;
        }
        config_base = function_base(region, device.bus, device.device, device.function);
        if config_base.is_some() {
            break;
        }
    }

    let base = config_base.ok_or("PCI function configuration space not accessible")?;

    // Read MSI control register to determine capabilities
    let control_offset = cap_ptr + 2;
    let mut control = config_read_u16(base, control_offset);
    let is_64bit = (control & (1 << 7)) != 0;
    let per_vector_mask = (control & (1 << 8)) != 0;

    // Program MSI for a single vector
    control &= !0x000E; // Clear multi-message bits (single vector)
    let msg_addr = MSI_ADDR_BASE | ((apic_id as u32) << 12);
    let msg_data = vector as u16;

    // Write message address
    config_write_u32(base, cap_ptr + 4, msg_addr);
    let mut data_offset = cap_ptr + 8;
    
    // Handle 64-bit address capability
    if is_64bit {
        config_write_u32(base, cap_ptr + 8, 0); // Upper address bits
        data_offset = cap_ptr + 12;
    }
    
    // Write message data (interrupt vector)
    config_write_u16(base, data_offset, msg_data);
    let mut next_offset = data_offset + 2;

    // Handle per-vector masking if supported
    if per_vector_mask {
        config_write_u32(base, next_offset, 0); // mask bits
        next_offset += 4;
        config_write_u32(base, next_offset, 0); // pending bits
    }

    // Enable MSI
    control |= 0x0001; // MSI enable bit
    config_write_u16(base, control_offset, control);

    Ok(())
}

// Configuration space access functions
// These functions provide safe access to PCI configuration space through ECAM

/// Read a 32-bit value from PCI configuration space.
///
/// # Arguments
/// * `function_base` - Base address of the function's configuration space
/// * `offset` - Byte offset within the configuration space
///
/// # Returns
/// The 32-bit value read from configuration space
fn config_read_u32(function_base: u64, offset: u8) -> u32 {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Read a 16-bit value from PCI configuration space.
///
/// This function handles unaligned 16-bit reads by reading the containing
/// 32-bit value and extracting the appropriate 16-bit field.
///
/// # Arguments
/// * `function_base` - Base address of the function's configuration space
/// * `offset` - Byte offset within the configuration space
///
/// # Returns
/// The 16-bit value read from configuration space
fn config_read_u16(function_base: u64, offset: u8) -> u16 {
    let aligned = offset & !0x3;
    let value = config_read_u32(function_base, aligned);
    let shift = (offset & 0x3) * 8;
    ((value >> shift) & 0xFFFF) as u16
}

/// Read an 8-bit value from PCI configuration space.
///
/// This function handles unaligned 8-bit reads by reading the containing
/// 32-bit value and extracting the appropriate 8-bit field.
///
/// # Arguments
/// * `function_base` - Base address of the function's configuration space
/// * `offset` - Byte offset within the configuration space
///
/// # Returns
/// The 8-bit value read from configuration space
fn config_read_u8(function_base: u64, offset: u8) -> u8 {
    let aligned = offset & !0x3;
    let value = config_read_u32(function_base, aligned);
    let shift = (offset & 0x3) * 8;
    ((value >> shift) & 0xFF) as u8
}

/// Write a 32-bit value to PCI configuration space.
///
/// # Arguments
/// * `function_base` - Base address of the function's configuration space
/// * `offset` - Byte offset within the configuration space
/// * `value` - 32-bit value to write
fn config_write_u32(function_base: u64, offset: u8, value: u32) {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) };
}

/// Write a 16-bit value to PCI configuration space.
///
/// # Arguments
/// * `function_base` - Base address of the function's configuration space
/// * `offset` - Byte offset within the configuration space
/// * `value` - 16-bit value to write
fn config_write_u16(function_base: u64, offset: u8, value: u16) {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u16, value) };
}

/// Write an 8-bit value to PCI configuration space.
///
/// # Arguments
/// * `function_base` - Base address of the function's configuration space
/// * `offset` - Byte offset within the configuration space
/// * `value` - 8-bit value to write
fn config_write_u8(function_base: u64, offset: u8, value: u8) {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::write_volatile(addr as *mut u8, value) };
}
