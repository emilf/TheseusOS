//! ACPI (Advanced Configuration and Power Interface) support module
//!
//! This module provides ACPI table parsing capabilities for the kernel using the acpi crate.
//! It implements the AcpiHandler trait to map physical memory regions using PHYS_OFFSET.

use crate::display::kernel_write_line;
use crate::handoff::handoff_phys_ptr;
use crate::memory::{
    current_pml4_phys, map_range_with_policy, phys_to_virt_pa, BootFrameAllocator, PageTable,
    PHYS_OFFSET, PTE_GLOBAL, PTE_NO_EXEC, PTE_PRESENT, PTE_WRITABLE,
};
use acpi::{AcpiHandler, AcpiTables, PhysicalMapping};
use core::ptr::NonNull;
use spin::Mutex;

/// ACPI submodule exports
pub use madt::MadtInfo;
const ACPI_WINDOW_BASE: u64 = 0xFFFF_FF80_0000_0000;
static ACPI_MAPPING_LOCK: Mutex<()> = Mutex::new(());

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

fn ensure_acpi_virtual_mapping(phys_addr: u64, size: usize) -> u64 {
    if !crate::memory::phys_offset_is_active() {
        return PHYS_OFFSET + phys_addr;
    }

    let page_size = crate::memory::PAGE_SIZE as u64;
    let phys_base = phys_addr & !(page_size - 1);
    let offset = phys_addr - phys_base;
    let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;
    let virt_base = ACPI_WINDOW_BASE + phys_base;

    if crate::memory::virt_range_has_flags(virt_base, size_aligned as usize, PTE_PRESENT) {
        return virt_base + offset;
    }

    let _guard = ACPI_MAPPING_LOCK.lock();
    if crate::memory::virt_range_has_flags(virt_base, size_aligned as usize, PTE_PRESENT) {
        return virt_base + offset;
    }

    unsafe {
        let handoff = &*(handoff_phys_ptr() as *const theseus_shared::handoff::Handoff);
        let mut temp_alloc = BootFrameAllocator::from_handoff(handoff);
        let pml4_pa = current_pml4_phys();
        let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
        let pml4 = &mut *pml4_ptr;
        map_range_with_policy(
            pml4,
            virt_base,
            phys_base,
            size_aligned,
            PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC,
            &mut temp_alloc,
        );
    }

    virt_base + offset
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
    kernel_write_line("");

    if let Err(e) = validate_rsdp_signature(acpi_rsdp) {
        return Err(e);
    }

    // Parse ACPI tables using the acpi crate
    let handler = KernelAcpiHandler;

    kernel_write_line("  [driver/acpi] parsing tables");
    let tables = match unsafe { AcpiTables::from_rsdp(handler, acpi_rsdp as usize) } {
        Ok(t) => t,
        Err(_e) => {
            kernel_write_line("  [driver/acpi] parse error");
            kernel_write_line("    -> provided RSDP pointer may be invalid");
            kernel_write_line(
                "    -> crashing before allocating per ACPI tables may indicate mapping issues",
            );
            return Err("Failed to parse ACPI tables");
        }
    };
    // TODO(map-ACPI): tables currently rely on PHYS_OFFSET identity. Mirror key tables into
    // dedicated high-half regions and tighten PHYS_OFFSET coverage afterwards.

    kernel_write_line("  [driver/acpi] tables parsed");

    // Extract platform information
    let platform_info = extract_platform_info(&tables)?;

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
        kernel_write_line("");

        for ap in &processor_info.application_processors {
            kernel_write_line("  [driver/acpi] AP APIC ID: ");
            theseus_shared::print_hex_u64_0xe9!(ap.local_apic_id as u64);
            kernel_write_line("");
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
            kernel_write_line("");

            // Extract IO APIC information
            for io_apic in &apic_info.io_apics {
                kernel_write_line("  [driver/acpi] IO APIC ID: ");
                theseus_shared::print_hex_u64_0xe9!(io_apic.id as u64);
                kernel_write_line(" Address: 0x");
                theseus_shared::print_hex_u64_0xe9!(io_apic.address as u64);
                kernel_write_line(" GSI Base: ");
                theseus_shared::print_hex_u64_0xe9!(io_apic.global_system_interrupt_base as u64);
                kernel_write_line("");
            }

            // Check for 8259 PIC support
            kernel_write_line("  [driver/acpi] Has 8259 PIC: ");
            if apic_info.also_has_legacy_pics {
                kernel_write_line("Yes");
            } else {
                kernel_write_line("No");
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
    kernel_write_line("");
    kernel_write_line("  [driver/acpi] IO APICs: ");
    theseus_shared::print_hex_u64_0xe9!(platform_info.io_apic_count as u64);
    kernel_write_line("");

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
    let rsdp_virt = PHYS_OFFSET + rsdp_address;
    let rsdp_ptr = rsdp_virt as *const u8;

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
            kernel_write_line(&alloc::format!(
                "  Invalid RSDP signature: {:02X?}",
                sig_bytes
            ));
            return Err("Invalid RSDP signature");
        }

        kernel_write_line("  RSDP signature verified");

        // Read RSDT address (offset 16 for ACPI 1.0, offset 24 for ACPI 2.0)
        let rsdt_addr_1_0 = core::ptr::read_volatile((rsdp_ptr.add(16)) as *const u32) as u64;
        let xsdt_addr_2_0 = core::ptr::read_volatile((rsdp_ptr.add(24)) as *const u64);

        // Prefer XSDT (ACPI 2.0) if available and valid
        if xsdt_addr_2_0 != 0 {
            kernel_write_line(&alloc::format!(
                "  Using XSDT (ACPI 2.0): 0x{:016X}",
                xsdt_addr_2_0
            ));
            Ok(xsdt_addr_2_0)
        } else if rsdt_addr_1_0 != 0 {
            kernel_write_line(&alloc::format!(
                "  Using RSDT (ACPI 1.0): 0x{:016X}",
                rsdt_addr_1_0
            ));
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
    let rsdt_virt = PHYS_OFFSET + rsdt_address;
    let rsdt_ptr = rsdt_virt as *const u8;

    unsafe {
        // Read RSDT header
        let signature = core::ptr::read_volatile(rsdt_ptr);
        let is_xsdt = signature == b'X'; // XSDT vs RSDT

        // Read entry count (offset 4)
        let entry_count = core::ptr::read_volatile((rsdt_ptr.add(4)) as *const u32);
        kernel_write_line(&alloc::format!(
            "  {} entries: {}",
            if is_xsdt { "XSDT" } else { "RSDT" },
            entry_count
        ));

        // Search for MADT (signature "APIC")
        for i in 0..entry_count {
            let entry_offset = 36 + (i as usize * if is_xsdt { 8 } else { 4 });
            let table_address = if is_xsdt {
                core::ptr::read_volatile((rsdt_ptr.add(entry_offset)) as *const u64)
            } else {
                core::ptr::read_volatile((rsdt_ptr.add(entry_offset)) as *const u32) as u64
            };

            if table_address == 0 {
                continue;
            }

            // Check table signature
            let table_virt = PHYS_OFFSET + table_address;
            let table_ptr = table_virt as *const u8;
            let mut sig_bytes = [0u8; 4];
            for j in 0..4 {
                sig_bytes[j] = core::ptr::read_volatile(table_ptr.add(j));
            }

            if &sig_bytes == b"APIC" {
                kernel_write_line(&alloc::format!("  Found MADT at 0x{:016X}", table_address));
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
    let madt_virt = PHYS_OFFSET + madt_address;
    let madt_ptr = madt_virt as *const u8;

    unsafe {
        // Verify MADT signature
        let mut sig_bytes = [0u8; 4];
        for i in 0..4 {
            sig_bytes[i] = core::ptr::read_volatile(madt_ptr.add(i));
        }

        if &sig_bytes != b"APIC" {
            return Err("Invalid MADT signature");
        }

        // Read MADT length and Local APIC address
        let madt_length = core::ptr::read_volatile((madt_ptr.add(4)) as *const u32);
        let local_apic_address = core::ptr::read_volatile((madt_ptr.add(36)) as *const u32) as u64;

        kernel_write_line(&alloc::format!("  MADT length: {} bytes", madt_length));
        kernel_write_line(&alloc::format!(
            "  Local APIC address: 0x{:016X}",
            local_apic_address
        ));

        let mut platform_info = PlatformInfo::new();
        platform_info.local_apic_address = local_apic_address;

        // Parse MADT entries
        let mut offset = 44; // Start of entries
        while offset < madt_length as usize {
            let entry_type = core::ptr::read_volatile(madt_ptr.add(offset + 1));
            let entry_length = core::ptr::read_volatile(madt_ptr.add(offset + 2));

            match entry_type {
                0 => {
                    // Processor Local APIC
                    let apic_id = core::ptr::read_volatile(madt_ptr.add(offset + 3));
                    let flags = core::ptr::read_volatile(madt_ptr.add(offset + 4));

                    if (flags & 1) != 0 {
                        // Processor enabled
                        platform_info.cpu_count += 1;
                        kernel_write_line(&alloc::format!("    CPU APIC ID: {}", apic_id));
                    }
                }
                1 => {
                    // IO APIC
                    let io_apic_id = core::ptr::read_volatile(madt_ptr.add(offset + 3));
                    let io_apic_address =
                        core::ptr::read_volatile((madt_ptr.add(offset + 4)) as *const u32) as u64;
                    let gsi_base =
                        core::ptr::read_volatile((madt_ptr.add(offset + 8)) as *const u32);

                    platform_info.io_apic_count += 1;
                    platform_info.has_io_apic = true;
                    kernel_write_line(&alloc::format!(
                        "    IO APIC ID: {}, Address: 0x{:016X}, GSI Base: {}",
                        io_apic_id,
                        io_apic_address,
                        gsi_base
                    ));
                }
                _ => {
                    // Other entry types - skip
                }
            }

            offset += entry_length as usize;
        }

        kernel_write_line(&alloc::format!("  Total CPUs: {}", platform_info.cpu_count));
        kernel_write_line(&alloc::format!(
            "  IO APICs: {}",
            platform_info.io_apic_count
        ));

        Ok(platform_info)
    }
}

fn validate_rsdp_signature(rsdp_address: u64) -> Result<(), &'static str> {
    let ptr = (PHYS_OFFSET + rsdp_address) as *const u8;
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

// Re-export MADT parser for compatibility
pub mod madt;
