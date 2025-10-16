//! ACPI (Advanced Configuration and Power Interface) support module
//!
//! This module provides ACPI table parsing capabilities for the kernel using the acpi crate.
//! It implements the AcpiHandler trait to map physical memory regions using PHYS_OFFSET.

use crate::log_debug;
use crate::display::kernel_write_line;
use crate::handoff::handoff_phys_ptr;
use crate::memory::{
    current_pml4_phys, map_range_with_policy, phys_to_virt_pa, PageTable, PTE_GLOBAL, PTE_NO_EXEC,
    PTE_PRESENT, PTE_WRITABLE,
};
use crate::physical_memory;
use acpi::{sdt::SdtHeader, AcpiHandler, AcpiTables, PhysicalMapping};
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
}

impl PlatformInfo {
    /// Create a new PlatformInfo with default values
    pub const fn new() -> Self {
        Self {
            cpu_count: 1, // At least the BSP
            has_io_apic: false,
            io_apic_count: 0,
            local_apic_address: 0xFEE00000, // Default Local APIC address
            has_legacy_pic: false,
            madt_info: None,
        }
    }
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
    kernel_write_line("  [driver/acpi] initialize");

    if acpi_rsdp == 0 {
        kernel_write_line("  [driver/acpi] no RSDP in handoff");
        return Err("No ACPI RSDP available");
    }

    kernel_write_line("  [driver/acpi] RSDP address: 0x");
    theseus_shared::print_hex_u64_0xe9!(acpi_rsdp);
    kernel_write_line("\n");
    dump_rsdp_bytes(acpi_rsdp);

    if let Err(e) = validate_rsdp_signature(acpi_rsdp) {
        return Err(e);
    }

    // Parse ACPI tables using the acpi crate
    let handler = KernelAcpiHandler;

    kernel_write_line("  [driver/acpi] parsing tables");
    dump_table_header("RSDP", acpi_rsdp, 36);
    kernel_write_line("  [driver/acpi] about to call AcpiTables::from_rsdp");
    let tables = match unsafe { AcpiTables::from_rsdp(handler, acpi_rsdp as usize) } {
        Ok(t) => t,
        Err(_e) => {
            kernel_write_line("  [driver/acpi] parse error");
            kernel_write_line("    -> unexpected error parsing ACPI tables");
            return Err("Failed to parse ACPI tables");
        }
    };
    kernel_write_line("  [driver/acpi] AcpiTables::from_rsdp returned Ok");

    kernel_write_line("  [driver/acpi] tables parsed");

    ensure_all_mirrored(&tables);

    // Extract platform information
    let platform_info = extract_platform_info(&tables)?;
    update_platform_cache(&platform_info);

    kernel_write_line("✓ ACPI initialization completed successfully");
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
    kernel_write_line("  [driver/acpi] extracting platform info");

    let mut platform_info = PlatformInfo::new();

    // Get platform info from acpi crate
    let platform_info_acpi = tables
        .platform_info()
        .map_err(|_| "Failed to get platform info")?;

    // Extract processor information
    if let Some(processor_info) = &platform_info_acpi.processor_info {
        platform_info.cpu_count = processor_info.application_processors.len() + 1; // BSP + APs

        kernel_write_line("  [driver/acpi] BSP APIC ID: ");
        theseus_shared::print_hex_u64_0xe9!(processor_info.boot_processor.local_apic_id as u64);
        kernel_write_line("\n");

        for ap in &processor_info.application_processors {
            kernel_write_line("  [driver/acpi] AP APIC ID: ");
            theseus_shared::print_hex_u64_0xe9!(ap.local_apic_id as u64);
            kernel_write_line("\n");
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

            kernel_write_line("  [driver/acpi] Local APIC address: 0x");
            theseus_shared::print_hex_u64_0xe9!(platform_info.local_apic_address);
            kernel_write_line("\n");

            // Extract IO APIC information
            for io_apic in &apic_info.io_apics {
                kernel_write_line("  [driver/acpi] IO APIC ID: ");
                theseus_shared::print_hex_u64_0xe9!(io_apic.id as u64);
                kernel_write_line(" Address: 0x");
                theseus_shared::print_hex_u64_0xe9!(io_apic.address as u64);
                kernel_write_line(" GSI Base: ");
                theseus_shared::print_hex_u64_0xe9!(io_apic.global_system_interrupt_base as u64);
                kernel_write_line("\n");
            }

            // Check for 8259 PIC support
            kernel_write_line("  [driver/acpi] Has 8259 PIC: ");
            if apic_info.also_has_legacy_pics {
                kernel_write_line("Yes\n");
            } else {
                kernel_write_line("No\n");
            }
        }
        _ => {
            kernel_write_line("  No APIC interrupt model found");
        }
    }

    match crate::acpi::madt::parse_madt(tables) {
        Ok(madt_info) => {
            kernel_write_line("  [driver/acpi] MADT parsed");
            platform_info.cpu_count = madt_info.cpu_apic_ids.len();
            platform_info.has_io_apic = !madt_info.io_apics.is_empty();
            platform_info.io_apic_count = madt_info.io_apics.len();
            platform_info.local_apic_address = madt_info.local_apic_address;
            platform_info.has_legacy_pic = madt_info.has_8259_pic;
            platform_info.madt_info = Some(madt_info);
        }
        Err(e) => {
            kernel_write_line("  [driver/acpi] MADT parsing failed: ");
            kernel_write_line(e);
        }
    }

    kernel_write_line("  [driver/acpi] Total CPUs: ");
    theseus_shared::print_hex_u64_0xe9!(platform_info.cpu_count as u64);
    kernel_write_line("\n");
    kernel_write_line("  [driver/acpi] IO APICs: ");
    theseus_shared::print_hex_u64_0xe9!(platform_info.io_apic_count as u64);
    kernel_write_line("\n");

    Ok(platform_info)
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
            kernel_write_line("  Invalid RSDP signature: ");
            for byte in &sig_bytes {
                theseus_shared::print_hex_u64_0xe9!(*byte as u64);
                kernel_write_line(" ");
            }
            kernel_write_line("\n");
            return Err("Invalid RSDP signature");
        }

        kernel_write_line("  RSDP signature verified");

        // Read RSDT address (offset 16 for ACPI 1.0, offset 24 for ACPI 2.0)
        let rsdt_addr_1_0 = core::ptr::read_volatile((rsdp_ptr.add(16)) as *const u32) as u64;
        let xsdt_addr_2_0 = core::ptr::read_volatile((rsdp_ptr.add(24)) as *const u64);

        // Prefer XSDT (ACPI 2.0) if available and valid
        if xsdt_addr_2_0 != 0 {
            kernel_write_line("  Using XSDT (ACPI 2.0): 0x");
            theseus_shared::print_hex_u64_0xe9!(xsdt_addr_2_0);
            kernel_write_line("\n");
            Ok(xsdt_addr_2_0)
        } else if rsdt_addr_1_0 != 0 {
            kernel_write_line("  Using RSDT (ACPI 1.0): 0x");
            theseus_shared::print_hex_u64_0xe9!(rsdt_addr_1_0 as u64);
            kernel_write_line("\n");
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
        if is_xsdt {
            kernel_write_line("  XSDT entries: ");
        } else {
            kernel_write_line("  RSDT entries: ");
        }
        theseus_shared::print_hex_u64_0xe9!(entry_count as u64);
        kernel_write_line("\n");

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
                kernel_write_line("  Found MADT at 0x");
                theseus_shared::print_hex_u64_0xe9!(table_address);
                kernel_write_line("\n");
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
                        kernel_write_line("    CPU APIC ID: ");
                        theseus_shared::print_hex_u64_0xe9!(apic_id as u64);
                        kernel_write_line("\n");
                    }
                }
                1 => {
                    platform_info.io_apic_count += 1;
                    platform_info.has_io_apic = true;
                    let io_apic_id = core::ptr::read_unaligned(entry_ptr.add(2));
                    let io_apic_addr =
                        core::ptr::read_unaligned(entry_ptr.add(4) as *const u32) as u64;
                    let gsi_base = core::ptr::read_unaligned(entry_ptr.add(8) as *const u32);
                    kernel_write_line("    IO APIC ID: ");
                    theseus_shared::print_hex_u64_0xe9!(io_apic_id as u64);
                    kernel_write_line(" Address: 0x");
                    theseus_shared::print_hex_u64_0xe9!(io_apic_addr);
                    kernel_write_line(" GSI Base: ");
                    theseus_shared::print_hex_u64_0xe9!(gsi_base as u64);
                    kernel_write_line("\n");
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

        kernel_write_line("  Local APIC address: 0x");
        theseus_shared::print_hex_u64_0xe9!(platform_info.local_apic_address);
        kernel_write_line("\n");

        kernel_write_line("  Total CPUs: ");
        theseus_shared::print_hex_u64_0xe9!(platform_info.cpu_count as u64);
        kernel_write_line("\n");
        kernel_write_line("  IO APICs: ");
        theseus_shared::print_hex_u64_0xe9!(platform_info.io_apic_count as u64);
        kernel_write_line("\n");

        Ok(platform_info)
    }
}

fn validate_rsdp_signature(rsdp_address: u64) -> Result<(), &'static str> {
    let ptr = ensure_acpi_virtual_mapping(rsdp_address, 8) as *const u8;
    for (idx, byte) in b"RSD PTR ".iter().enumerate() {
        let val = unsafe { core::ptr::read_volatile(ptr.add(idx)) };
        if val != *byte {
            kernel_write_line("  [driver/acpi] RSDP signature mismatch during pre-check");
            return Err("Invalid RSDP signature");
        }
    }
    kernel_write_line("  [driver/acpi] RSDP signature pre-check passed");
    Ok(())
}

fn dump_rsdp_bytes(rsdp_address: u64) {
    kernel_write_line("  [driver/acpi] dumping first 32 bytes of RSDP");
    let ptr = ensure_acpi_virtual_mapping(rsdp_address, 32) as *const u8;
    unsafe {
        for i in 0..32 {
            if i % 8 == 0 {
                kernel_write_line("    ");
            }
            let byte = core::ptr::read_volatile(ptr.add(i));
            theseus_shared::print_hex_u64_0xe9!(byte as u64);
            kernel_write_line(" ");
        }
        kernel_write_line("\n");
    }
}

fn dump_table_header(tag: &str, phys_addr: u64, min_len: usize) {
    kernel_write_line("  [driver/acpi] mapping header for ");
    kernel_write_line(tag);
    kernel_write_line(" @ 0x");
    theseus_shared::print_hex_u64_0xe9!(phys_addr);
    kernel_write_line("\n");
    let header_ptr = ensure_acpi_virtual_mapping(phys_addr, min_len) as *const SdtHeader;
    unsafe {
        let sig = (*header_ptr).signature;
        let length = (*header_ptr).length;
        kernel_write_line("    signature: ");
        let sig_ptr = &sig as *const _ as *const u8;
        for i in 0..4 {
            let byte = core::ptr::read_unaligned(sig_ptr.add(i));
            theseus_shared::out_char_0xe9!(byte);
        }
        kernel_write_line(" length: 0x");
        theseus_shared::print_hex_u64_0xe9!(length as u64);
        kernel_write_line("\n");
    }
}

// Re-export MADT parser for compatibility
pub mod madt;
