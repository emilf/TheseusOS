//! Module: bootloader::acpi
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//! - docs/axioms/arch-x86_64.md#A4:-SMP-discovery-exists-but-AP-startup-is-not-yet-a-documented-implemented-invariant
//!
//! INVARIANTS:
//! - This module is the firmware-side ACPI discovery path used during bootloader handoff preparation.
//! - The bootloader records table roots/derived ACPI information for later kernel use; it does not become the long-term owner of ACPI runtime behavior.
//!
//! SAFETY:
//! - Firmware table pointers are structured input, not sacred truth.
//! - Identity-style access in the UEFI environment is a boot-time convenience, not a statement about later kernel mapping behavior.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/interrupts-and-platform.md
//!
//! Firmware-side ACPI discovery and parsing helpers.

use crate::drivers::manager::write_line;
use acpi::{AcpiHandler, AcpiTables, PhysicalMapping};
use core::ptr::NonNull;

/// ACPI Handler for UEFI environment
///
/// This implements the AcpiHandler trait to allow the acpi crate to map physical memory
/// in the UEFI environment. UEFI provides a flat memory model, so we can use identity
/// mapping for physical addresses.
#[derive(Clone)]
pub struct UefiAcpiHandler;

impl AcpiHandler for UefiAcpiHandler {
    /// Map a physical memory region to a virtual address
    ///
    /// In UEFI, we can use identity mapping for physical addresses since UEFI provides
    /// a flat memory model where physical and virtual addresses are the same.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the physical address is valid and the size is correct.
    unsafe fn map_physical_region<T>(
        &self,
        physical_address: usize,
        size: usize,
    ) -> PhysicalMapping<Self, T> {
        let virtual_address = physical_address as *mut T;
        let non_null_virtual_address =
            NonNull::new(virtual_address).expect("Physical address should be valid");

        PhysicalMapping::new(physical_address, non_null_virtual_address, size, size, Self)
    }

    /// Unmap a physical memory region
    ///
    /// In UEFI, we don't need to unmap physical regions since the memory remains
    /// accessible until exit_boot_services is called.
    fn unmap_physical_region<T>(_region: &PhysicalMapping<Self, T>) {
        // No-op in UEFI environment
    }
}

/// Find the ACPI RSDP in the UEFI configuration table.
///
/// This prefers ACPI 2.0 entries when both legacy and newer roots are present.
pub fn find_acpi_rsdp(verbose: bool) -> Option<u64> {
    use core::slice;
    use uefi::table;
    use uefi::table::cfg::{ConfigTableEntry, ACPI2_GUID, ACPI_GUID};


    // Get the system table
    let system_table = match table::system_table_raw() {
        Some(st) => st,
        None => {
            return None;
        }
    };

    // SAFETY: The system table is valid as it was obtained from the global state
    let st = unsafe { system_table.as_ref() };

    // Check if configuration table is available
    if st.configuration_table.is_null() || st.number_of_configuration_table_entries == 0 {
        return None;
    }

    // SAFETY: We have a valid pointer and count from the system table
    let config_entries = unsafe {
        slice::from_raw_parts(
            st.configuration_table as *const ConfigTableEntry,
            st.number_of_configuration_table_entries,
        )
    };

    // Search for ACPI 2.0 RSDP first (preferred)
    for (i, entry) in config_entries.iter().enumerate() {
        if verbose {
            write_line(&alloc::format!("  ACPI cfg entry {} GUID: {:?}", i, entry.guid));
        }

        if entry.guid == ACPI2_GUID {
            if verbose {
                write_line(&alloc::format!("  Found ACPI 2.0 RSDP at 0x{:016X}", entry.address as u64));
            }
            return Some(entry.address as u64);
        }
    }

    // Fall back to ACPI 1.0 RSDP
    for entry in config_entries {
        if entry.guid == ACPI_GUID {
            if verbose {
                write_line(&alloc::format!("  Found ACPI 1.0 RSDP at 0x{:016X}", entry.address as u64));
            }
            return Some(entry.address as u64);
        }
    }

    if verbose {
        write_line("  No ACPI RSDP table found in configuration table");
    }

    // No ACPI table found
    None
}

/// Parse ACPI tables from a discovered RSDP address.
///
/// This is a best-effort firmware-phase validation step using the `acpi` crate.
pub fn parse_acpi_tables(rsdp_address: u64) -> Result<(), &'static str> {
    if rsdp_address == 0 {
        return Err("Invalid RSDP address");
    }

    let handler = UefiAcpiHandler;

    // Parse the ACPI tables
    match unsafe { AcpiTables::from_rsdp(handler, rsdp_address as usize) } {
        Ok(tables) => {
            // Get platform information
            let platform_info = tables.platform_info();

            // Log successful parsing
            write_line("✓ ACPI tables parsed successfully");
            write_line(&alloc::format!(
                "  Platform info available: {}",
                platform_info.is_ok()
            ));

            Ok(())
        }
        Err(e) => {
            write_line(&alloc::format!("✗ Failed to parse ACPI tables: {:?}", e));
            Err("Failed to parse ACPI tables")
        }
    }
}
