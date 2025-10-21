//! ACPI (Advanced Configuration and Power Interface) support module
//!
//! This module provides ACPI table parsing capabilities for the kernel using the acpi crate.
//! It implements the AcpiHandler trait to map physical memory regions using PHYS_OFFSET.

use crate::handoff::handoff_phys_ptr;
use crate::memory::{
    current_pml4_phys, map_range_with_policy, phys_to_virt_pa, PageTable, PTE_GLOBAL, PTE_NO_EXEC,
    PTE_PCD, PTE_PRESENT, PTE_PWT, PTE_WRITABLE,
};
use crate::physical_memory;
use crate::{log_debug, log_error, log_info, log_trace, log_warn};
use acpi::{sdt::SdtHeader, AcpiHandler, AcpiTables, PhysicalMapping};
use alloc::vec::Vec;
use core::{
    ptr::NonNull,
    sync::atomic::{AtomicBool, Ordering},
};
use spin::Mutex;

/// ACPI submodule exports
pub use madt::MadtInfo;
const ACPI_WINDOW_BASE: u64 = 0xFFFF_FF80_0000_0000;
static ACPI_MAPPING_LOCK: Mutex<()> = Mutex::new(());
static PHYS_OFFSET_FALLBACK_WARNED: AtomicBool = AtomicBool::new(false);

/// ACPI Handler for kernel environment
///
/// This implements the AcpiHandler trait to allow the acpi crate to map physical memory
/// in the kernel environment. We use PHYS_OFFSET mapping to convert physical addresses
/// to virtual addresses.
#[derive(Clone)]
pub struct KernelAcpiHandler;

impl AcpiHandler for KernelAcpiHandler {
    /// Map a physical memory region to a virtual address
    ///
    /// In the kernel, we use PHYS_OFFSET mapping to convert physical addresses
    /// to virtual addresses. This allows us to access ACPI tables that are
    /// located in physical memory.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the physical address is valid and the size is correct.
    /// The physical memory region must be accessible and not overlap with critical kernel data.
    unsafe fn map_physical_region<T>(
        &self,
        physical_address: usize,
        size: usize,
    ) -> PhysicalMapping<Self, T> {
        let virt_addr = ensure_acpi_virtual_mapping(physical_address as u64, size as usize);
        let virtual_address = virt_addr as *mut T;
        let non_null_virtual_address =
            NonNull::new(virtual_address).expect("Physical address should be valid");

        PhysicalMapping::new(physical_address, non_null_virtual_address, size, size, Self)
    }

    /// Unmap a physical memory region
    ///
    /// In the kernel, we don't need to unmap physical regions since the PHYS_OFFSET
    /// mapping is persistent and covers all physical memory.
    /// TODO: PHYS_OFFSET mapping is not persistent in the kernel environment, so we
    /// need to unmap the physical region when the mapping is no longer needed.
    fn unmap_physical_region<T>(_region: &PhysicalMapping<Self, T>) {
        // No-op in kernel environment - PHYS_OFFSET mapping is persistent
    }
}

#[derive(Default)]
#[allow(dead_code)]
struct MadtParseContext {
    entry_phys: u64,
    entry_len: u64,
    offset: u64,
}

fn ensure_acpi_virtual_mapping(phys_addr: u64, size: usize) -> u64 {
    if crate::memory::phys_offset_is_active() {
        if !PHYS_OFFSET_FALLBACK_WARNED.load(Ordering::Relaxed) {
            PHYS_OFFSET_FALLBACK_WARNED.store(true, Ordering::Relaxed);
            log_debug!("Using PHYS_OFFSET mapping for ACPI access");
        }
        return phys_to_virt_pa(phys_addr);
    }

    let page_size = crate::memory::PAGE_SIZE as u64;
    let phys_base = phys_addr & !(page_size - 1);
    let offset = phys_addr - phys_base;
    let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;
    let virt_base = ACPI_WINDOW_BASE + phys_base;

    let _guard = ACPI_MAPPING_LOCK.lock();

    unsafe {
        let handoff = &*(handoff_phys_ptr() as *const theseus_shared::handoff::Handoff);

        // Ensure the runtime memory map buffer is accessible via PHYS_OFFSET.
        let memmap_len = handoff.memory_map_size as u64;
        if memmap_len != 0 {
            let pml4_pa = current_pml4_phys();
            let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
            let pml4 = &mut *pml4_ptr;
            let phys_page_base = handoff.memory_map_buffer_ptr & !(page_size - 1);
            let offset_in_page = handoff.memory_map_buffer_ptr - phys_page_base;
            let total = ((offset_in_page + memmap_len + page_size - 1) / page_size) * page_size;
            let mut persistent = physical_memory::PersistentFrameAllocator;
            map_range_with_policy(
                pml4,
                crate::memory::PHYS_OFFSET.wrapping_add(phys_page_base),
                phys_page_base,
                total,
                PTE_PRESENT | PTE_WRITABLE,
                &mut persistent,
            );
        }

        let pml4_pa = current_pml4_phys();
        let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
        let pml4 = &mut *pml4_ptr;
        let mut persistent = physical_memory::PersistentFrameAllocator;
        map_range_with_policy(
            pml4,
            virt_base,
            phys_base,
            size_aligned,
            PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC,
            &mut persistent,
        );
    }

    virt_base + offset
}

fn map_mmconfig_region(phys_addr: u64, size: usize) -> u64 {
    let page_size = crate::memory::PAGE_SIZE as u64;
    let phys_base = phys_addr & !(page_size - 1);
    let offset = phys_addr - phys_base;
    let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;
    let virt_base = ACPI_WINDOW_BASE + phys_base;

    let _guard = ACPI_MAPPING_LOCK.lock();

    unsafe {
        let pml4_pa = current_pml4_phys();
        let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
        let pml4 = &mut *pml4_ptr;
        let mut persistent = physical_memory::PersistentFrameAllocator;
        map_range_with_policy(
            pml4,
            virt_base,
            phys_base,
            size_aligned,
            PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_PCD | PTE_PWT,
            &mut persistent,
        );
    }

    virt_base + offset
}

fn map_sdt_bytes(phys_addr: u64) -> (*const u8, usize) {
    let header_ptr = ensure_acpi_virtual_mapping(phys_addr, core::mem::size_of::<SdtHeader>())
        as *const SdtHeader;
    let length = unsafe { (*header_ptr).length as usize };
    let ptr = ensure_acpi_virtual_mapping(phys_addr, length) as *const u8;
    (ptr, length)
}

fn ensure_all_mirrored<H: AcpiHandler>(tables: &AcpiTables<H>) {
    for (_, sdt) in tables.sdts.iter() {
        ensure_acpi_virtual_mapping(sdt.physical_address as u64, sdt.length as usize);
    }
    if let Some(dsdt) = &tables.dsdt {
        let header_addr = dsdt.address as u64 - core::mem::size_of::<SdtHeader>() as u64;
        let total_len = dsdt.length as usize + core::mem::size_of::<SdtHeader>();
        ensure_acpi_virtual_mapping(header_addr, total_len);
    }
    for ssdt in &tables.ssdts {
        let header_addr = ssdt.address as u64 - core::mem::size_of::<SdtHeader>() as u64;
        let total_len = ssdt.length as usize + core::mem::size_of::<SdtHeader>();
        ensure_acpi_virtual_mapping(header_addr, total_len);
    }
}

/// Simple ACPI platform information
#[derive(Debug, Clone)]
pub struct PlatformInfo {
    /// Total number of CPUs (BSP + APs)
    pub cpu_count: usize,
    /// Whether the system has IO APICs
    pub has_io_apic: bool,
    /// Number of IO APICs found
    pub io_apic_count: usize,
    /// Local APIC address
    pub local_apic_address: u64,
    /// Whether legacy PICs are present alongside APIC
    pub has_legacy_pic: bool,
    /// Detailed MADT-derived information, if available
    pub madt_info: Option<madt::MadtInfo>,
    /// PCI Express Enhanced Configuration Access Mechanism regions
    pub pci_config_regions: Vec<PciConfigRegion>,
}

impl PlatformInfo {
    /// Create a new PlatformInfo with default values
    pub fn new() -> Self {
        Self {
            cpu_count: 1, // At least the BSP
            has_io_apic: false,
            io_apic_count: 0,
            local_apic_address: 0xFEE00000, // Default Local APIC address
            has_legacy_pic: false,
            madt_info: None,
            pci_config_regions: Vec::new(),
        }
    }
}

/// Description of a single ECAM mapping for PCI configuration space.
#[derive(Debug, Clone)]
pub struct PciConfigRegion {
    /// Physical base address of the ECAM window.
    pub phys_base: u64,
    /// Virtual base address where the window has been mapped.
    pub virt_base: u64,
    /// PCI segment group number served by this region.
    pub segment: u16,
    /// First bus number handled by this region (inclusive).
    pub bus_start: u8,
    /// Last bus number handled by this region (inclusive).
    pub bus_end: u8,
}

static PLATFORM_INFO_CACHE: Mutex<Option<PlatformInfo>> = Mutex::new(None);

/// Retrieve the cached platform summary gathered during ACPI initialization.
///
/// Returns `None` if ACPI has not been initialized successfully yet.
pub fn cached_platform_info() -> Option<PlatformInfo> {
    PLATFORM_INFO_CACHE.lock().clone()
}

fn update_platform_cache(info: &PlatformInfo) {
    let mut cache = PLATFORM_INFO_CACHE.lock();
    *cache = Some(info.clone());
}

/// Initialize ACPI parsing from handoff structure
///
/// This function extracts the ACPI RSDP address from the handoff structure
/// and parses the ACPI tables to extract platform information.
///
/// # Arguments
///
/// * `acpi_rsdp` - Physical address of the ACPI RSDP table from handoff
///
/// # Returns
///
/// * `Ok(PlatformInfo)` - Platform information extracted from ACPI
/// * `Err(&'static str)` - If parsing failed
pub fn initialize_acpi(acpi_rsdp: u64) -> Result<PlatformInfo, &'static str> {
    log_info!("ACPI initialize");

    if acpi_rsdp == 0 {
        log_warn!("No RSDP in handoff");
        return Err("No ACPI RSDP available");
    }

    log_debug!("RSDP address: {:#x}", acpi_rsdp);
    dump_rsdp_bytes(acpi_rsdp);

    if let Err(e) = validate_rsdp_signature(acpi_rsdp) {
        return Err(e);
    }

    // Parse ACPI tables using the acpi crate
    let handler = KernelAcpiHandler;

    log_debug!("Parsing ACPI tables");
    dump_table_header("RSDP", acpi_rsdp, 36);
    log_trace!("About to call AcpiTables::from_rsdp");
    let tables = match unsafe { AcpiTables::from_rsdp(handler, acpi_rsdp as usize) } {
        Ok(t) => t,
        Err(_e) => {
            log_error!("ACPI parse error - unexpected error parsing ACPI tables");
            return Err("Failed to parse ACPI tables");
        }
    };
    log_trace!("AcpiTables::from_rsdp returned Ok");

    log_debug!("ACPI tables parsed");

    ensure_all_mirrored(&tables);

    // Extract platform information
    let platform_info = extract_platform_info(&tables)?;
    update_platform_cache(&platform_info);

    log_info!("✓ ACPI initialization completed successfully");
    Ok(platform_info)
}

/// Extract platform information from parsed ACPI tables
///
/// # Arguments
///
/// * `tables` - Parsed ACPI tables
///
/// # Returns
///
/// * `Ok(PlatformInfo)` - Platform information extracted from ACPI
/// * `Err(&'static str)` - If extraction failed
fn extract_platform_info(
    tables: &AcpiTables<KernelAcpiHandler>,
) -> Result<PlatformInfo, &'static str> {
    log_debug!("Extracting platform info");

    let mut platform_info = PlatformInfo::new();

    // Get platform info from acpi crate
    let platform_info_acpi = tables
        .platform_info()
        .map_err(|_| "Failed to get platform info")?;

    // Extract processor information
    if let Some(processor_info) = &platform_info_acpi.processor_info {
        platform_info.cpu_count = processor_info.application_processors.len() + 1; // BSP + APs

        log_debug!(
            "BSP APIC ID: {:#x}",
            processor_info.boot_processor.local_apic_id
        );

        for ap in &processor_info.application_processors {
            log_debug!("AP APIC ID: {:#x}", ap.local_apic_id);
        }
    }

    // Extract interrupt model information
    match &platform_info_acpi.interrupt_model {
        acpi::InterruptModel::Apic(apic_info) => {
            platform_info.has_io_apic = !apic_info.io_apics.is_empty();
            platform_info.io_apic_count = apic_info.io_apics.len();

            // Extract Local APIC address
            platform_info.local_apic_address = apic_info.local_apic_address as u64;
            platform_info.has_legacy_pic = apic_info.also_has_legacy_pics;

            log_debug!(
                "Local APIC address: {:#x}",
                platform_info.local_apic_address
            );

            // Extract IO APIC information
            for io_apic in &apic_info.io_apics {
                log_debug!(
                    "IO APIC ID: {:#x} Address: {:#x} GSI Base: {}",
                    io_apic.id,
                    io_apic.address,
                    io_apic.global_system_interrupt_base
                );
            }

            // Check for 8259 PIC support
            log_debug!(
                "Has 8259 PIC: {}",
                if apic_info.also_has_legacy_pics {
                    "Yes"
                } else {
                    "No"
                }
            );
        }
        _ => {
            log_warn!("No APIC interrupt model found");
        }
    }

    match crate::acpi::madt::parse_madt(tables) {
        Ok(madt_info) => {
            log_debug!("MADT parsed");
            platform_info.cpu_count = madt_info.cpu_apic_ids.len();
            platform_info.has_io_apic = !madt_info.io_apics.is_empty();
            platform_info.io_apic_count = madt_info.io_apics.len();
            platform_info.local_apic_address = madt_info.local_apic_address;
            platform_info.has_legacy_pic = madt_info.has_8259_pic;
            platform_info.madt_info = Some(madt_info);
        }
        Err(e) => {
            log_warn!("MADT parsing failed: {}", e);
        }
    }

    collect_pci_config_regions(tables, &mut platform_info);

    log_info!(
        "Total CPUs: {} IO APICs: {}",
        platform_info.cpu_count,
        platform_info.io_apic_count
    );

    Ok(platform_info)
}

#[repr(C, packed)]
struct RawMcfg {
    header: SdtHeader,
    _reserved: u64,
}

#[repr(C, packed)]
#[derive(Copy, Clone)]
struct RawMcfgEntry {
    base_address: u64,
    segment: u16,
    bus_start: u8,
    bus_end: u8,
    _reserved: u32,
}

fn collect_pci_config_regions(
    tables: &AcpiTables<KernelAcpiHandler>,
    platform_info: &mut PlatformInfo,
) {
    use acpi::sdt::Signature;
    let sdt = match tables.sdts.get(&Signature::MCFG) {
        Some(s) => s,
        None => {
            log_debug!("ACPI MCFG table not present");
            return;
        }
    };

    let table_len = sdt.length as usize;
    if table_len < core::mem::size_of::<RawMcfg>() {
        log_warn!("MCFG table too short ({})", table_len);
        return;
    }

    let base_ptr = map_mmconfig_region(sdt.physical_address as u64, table_len) as *const u8;
    let mut offset = core::mem::size_of::<RawMcfg>();
    let entry_size = core::mem::size_of::<RawMcfgEntry>();

    while offset + entry_size <= table_len {
        let entry_ptr = unsafe { base_ptr.add(offset) as *const RawMcfgEntry };
        let raw_entry = unsafe { core::ptr::read_unaligned(entry_ptr) };
        let segment = raw_entry.segment;
        let bus_start = raw_entry.bus_start;
        let bus_end = raw_entry.bus_end;
        let base_address = raw_entry.base_address;

        if bus_end < bus_start {
            log_warn!(
                "Skipping malformed MCFG entry: segment={} bus_start={} bus_end={}",
                segment,
                bus_start,
                bus_end
            );
            offset += entry_size;
            continue;
        }

        let bus_count = (bus_end as u32 - bus_start as u32 + 1) as u64;
        let window_size = bus_count * 0x10_0000; // 1 MiB per bus
        let virt_base = map_mmconfig_region(base_address, window_size as usize);

        log_info!(
            "MCFG region: segment={} buses={}..={} phys=0x{:012x} virt=0x{:016x} size={:#x}",
            segment,
            bus_start,
            bus_end,
            base_address,
            virt_base,
            window_size
        );

        platform_info.pci_config_regions.push(PciConfigRegion {
            phys_base: base_address,
            virt_base,
            segment,
            bus_start,
            bus_end,
        });

        offset += entry_size;
    }
}

/// Parse RSDP (Root System Description Pointer) table
///
/// # Arguments
///
/// * `rsdp_address` - Physical address of the RSDP table
///
/// # Returns
///
/// * `Ok(u64)` - Physical address of RSDT/XSDT table
/// * `Err(&'static str)` - If parsing failed
#[allow(dead_code)]
fn parse_rsdp(rsdp_address: u64) -> Result<u64, &'static str> {
    let rsdp_ptr = ensure_acpi_virtual_mapping(rsdp_address, 64) as *const u8;

    // Verify RSDP signature "RSD PTR "
    unsafe {
        let signature = core::ptr::read_volatile(rsdp_ptr);
        if signature != b'R' {
            return Err("Invalid RSDP signature");
        }

        // Read the full signature
        let mut sig_bytes = [0u8; 8];
        for i in 0..8 {
            sig_bytes[i] = core::ptr::read_volatile(rsdp_ptr.add(i));
        }

        if &sig_bytes != b"RSD PTR " {
            log_error!("Invalid RSDP signature: {:02x?}", sig_bytes);
            return Err("Invalid RSDP signature");
        }

        log_trace!("RSDP signature verified");

        // Read RSDT address (offset 16 for ACPI 1.0, offset 24 for ACPI 2.0)
        let rsdt_addr_1_0 = core::ptr::read_volatile((rsdp_ptr.add(16)) as *const u32) as u64;
        let xsdt_addr_2_0 = core::ptr::read_volatile((rsdp_ptr.add(24)) as *const u64);

        // Prefer XSDT (ACPI 2.0) if available and valid
        if xsdt_addr_2_0 != 0 {
            log_debug!("Using XSDT (ACPI 2.0): {:#x}", xsdt_addr_2_0);
            Ok(xsdt_addr_2_0)
        } else if rsdt_addr_1_0 != 0 {
            log_debug!("Using RSDT (ACPI 1.0): {:#x}", rsdt_addr_1_0);
            Ok(rsdt_addr_1_0 as u64)
        } else {
            Err("No valid RSDT/XSDT address found")
        }
    }
}

/// Find MADT (Multiple APIC Description Table) in RSDT/XSDT
///
/// # Arguments
///
/// * `rsdt_address` - Physical address of RSDT/XSDT table
///
/// # Returns
///
/// * `Ok(u64)` - Physical address of MADT table
/// * `Err(&'static str)` - If MADT not found
#[allow(dead_code)]
fn find_madt(rsdt_address: u64) -> Result<u64, &'static str> {
    let (rsdt_ptr, rsdt_len) = map_sdt_bytes(rsdt_address);

    unsafe {
        let signature = core::ptr::read_volatile(rsdt_ptr);
        let is_xsdt = signature == b'X';

        let entry_count = core::ptr::read_volatile((rsdt_ptr.add(4)) as *const u32);
        let table_type = if is_xsdt { "XSDT" } else { "RSDT" };
        log_trace!("{} entries: {}", table_type, entry_count);

        let entry_width = if is_xsdt {
            core::mem::size_of::<u64>()
        } else {
            core::mem::size_of::<u32>()
        };
        let entries_base = rsdt_ptr.add(core::mem::size_of::<SdtHeader>());
        let available = rsdt_len.saturating_sub(core::mem::size_of::<SdtHeader>());

        for i in 0..entry_count as usize {
            let offset = i * entry_width;
            if offset + entry_width > available {
                break;
            }
            let table_address = if is_xsdt {
                core::ptr::read_unaligned(entries_base.add(offset) as *const u64)
            } else {
                core::ptr::read_unaligned(entries_base.add(offset) as *const u32) as u64
            };

            if table_address == 0 {
                continue;
            }

            let table_ptr = ensure_acpi_virtual_mapping(table_address, core::mem::size_of::<u32>())
                as *const u8;
            let mut sig_bytes = [0u8; 4];
            for j in 0..4 {
                sig_bytes[j] = core::ptr::read_volatile(table_ptr.add(j));
            }

            if &sig_bytes == b"APIC" {
                log_debug!("Found MADT at {:#x}", table_address);
                return Ok(table_address);
            }
        }

        Err("MADT table not found")
    }
}

/// Parse MADT (Multiple APIC Description Table)
///
/// # Arguments
///
/// * `madt_address` - Physical address of MADT table
///
/// # Returns
///
/// * `Ok(PlatformInfo)` - Platform information extracted from MADT
/// * `Err(&'static str)` - If parsing failed
#[allow(dead_code)]
fn parse_madt(madt_address: u64) -> Result<PlatformInfo, &'static str> {
    let (madt_ptr, madt_len) = map_sdt_bytes(madt_address);

    unsafe {
        let madt_length = (*(madt_ptr as *const SdtHeader)).length as usize;
        let mut platform_info = PlatformInfo::new();

        if madt_length < core::mem::size_of::<SdtHeader>() + 8 {
            return Err("MADT too small");
        }

        let local_apic_address = core::ptr::read_unaligned(madt_ptr.add(36) as *const u32) as u64;
        platform_info.local_apic_address = local_apic_address;

        let mut offset = core::mem::size_of::<SdtHeader>() + 8;
        let limit = core::cmp::min(madt_len, madt_length);
        while offset + 2 <= limit {
            let entry_ptr = madt_ptr.add(offset);
            let entry_type = core::ptr::read_unaligned(entry_ptr);
            let entry_len = core::ptr::read_unaligned(entry_ptr.add(1)) as usize;
            if entry_len < 2 || offset + entry_len > limit {
                break;
            }

            crate::interrupts::set_double_fault_context(
                entry_ptr as u64,
                entry_type as u64,
                entry_len as u64,
                [
                    offset as u64,
                    limit as u64,
                    madt_address,
                    local_apic_address,
                    platform_info.cpu_count as u64,
                    platform_info.io_apic_count as u64,
                ],
            );

            match entry_type {
                0 => {
                    let flags = core::ptr::read_unaligned(entry_ptr.add(4) as *const u32);
                    if flags & 1 != 0 {
                        platform_info.cpu_count += 1;
                        let apic_id = core::ptr::read_unaligned(entry_ptr.add(3));
                        log_trace!("CPU APIC ID: {:#x}", apic_id);
                    }
                }
                1 => {
                    platform_info.io_apic_count += 1;
                    platform_info.has_io_apic = true;
                    let io_apic_id = core::ptr::read_unaligned(entry_ptr.add(2));
                    let io_apic_addr =
                        core::ptr::read_unaligned(entry_ptr.add(4) as *const u32) as u64;
                    let gsi_base = core::ptr::read_unaligned(entry_ptr.add(8) as *const u32);
                    log_trace!(
                        "IO APIC ID: {:#x} Address: {:#x} GSI Base: {}",
                        io_apic_id,
                        io_apic_addr,
                        gsi_base
                    );
                }
                5 => {
                    let override_addr = core::ptr::read_unaligned(entry_ptr.add(4) as *const u64);
                    platform_info.local_apic_address = override_addr;
                }
                2 => {
                    platform_info.has_legacy_pic = true;
                }
                _ => {}
            }

            offset += entry_len;
        }

        if platform_info.cpu_count == 0 {
            platform_info.cpu_count = 1;
        }

        log_debug!(
            "Local APIC address: {:#x}, Total CPUs: {}, IO APICs: {}",
            platform_info.local_apic_address,
            platform_info.cpu_count,
            platform_info.io_apic_count
        );

        Ok(platform_info)
    }
}

fn validate_rsdp_signature(rsdp_address: u64) -> Result<(), &'static str> {
    let ptr = ensure_acpi_virtual_mapping(rsdp_address, 8) as *const u8;
    for (idx, byte) in b"RSD PTR ".iter().enumerate() {
        let val = unsafe { core::ptr::read_volatile(ptr.add(idx)) };
        if val != *byte {
            log_error!("RSDP signature mismatch during pre-check");
            return Err("Invalid RSDP signature");
        }
    }
    log_trace!("RSDP signature pre-check passed");
    Ok(())
}

fn dump_rsdp_bytes(rsdp_address: u64) {
    log_trace!("Dumping first 32 bytes of RSDP");
    let ptr = ensure_acpi_virtual_mapping(rsdp_address, 32) as *const u8;
    // Skip detailed hex dump in log - can be added if needed for debugging
    let _ = ptr;
}

fn dump_table_header(tag: &str, phys_addr: u64, min_len: usize) {
    let header_ptr = ensure_acpi_virtual_mapping(phys_addr, min_len) as *const SdtHeader;
    unsafe {
        let sig = (*header_ptr).signature;
        let length = (*header_ptr).length;
        let sig_bytes = core::slice::from_raw_parts(&sig as *const _ as *const u8, 4);
        let sig_str = core::str::from_utf8_unchecked(sig_bytes);
        log_trace!(
            "Mapping header for {} @ {:#x}: signature={} length={:#x}",
            tag,
            phys_addr,
            sig_str,
            length
        );
    }
}

// Re-export MADT parser for compatibility
pub mod madt;
