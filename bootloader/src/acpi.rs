//! ACPI (Advanced Configuration and Power Interface) support module
//! 
//! This module provides ACPI table discovery and parsing capabilities for the UEFI bootloader.
//! It implements the AcpiHandler trait required by the acpi crate to map physical memory
//! regions in the UEFI environment.

use acpi::{AcpiHandler, PhysicalMapping, AcpiTables};
use core::ptr::NonNull;
use crate::drivers::manager::write_line;

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
        let non_null_virtual_address = NonNull::new(virtual_address)
            .expect("Physical address should be valid");
        
        PhysicalMapping::new(
            physical_address,
            non_null_virtual_address,
            size,
            size,
            Self,
        )
    }

    /// Unmap a physical memory region
    /// 
    /// In UEFI, we don't need to unmap physical regions since the memory remains
    /// accessible until exit_boot_services is called.
    fn unmap_physical_region<T>(_region: &PhysicalMapping<Self, T>) {
        // No-op in UEFI environment
    }
}

/// Find the ACPI RSDP (Root System Description Pointer) table
/// 
/// This function searches for the ACPI RSDP table in the UEFI configuration table.
/// The RSDP is the entry point for all ACPI tables. It searches for both ACPI 1.0
/// and ACPI 2.0 RSDP tables, preferring ACPI 2.0 if both are available.
/// 
/// # Arguments
/// 
/// * `verbose` - Whether to output debug information
/// 
/// # Returns
/// 
/// * `Some(u64)` - Physical address of the RSDP table if found
/// * `None` - If the RSDP table is not found
/// 
/// # Safety
/// 
/// This function accesses the UEFI system table and configuration table.
/// It is safe to call during UEFI boot services phase.
pub fn find_acpi_rsdp(verbose: bool) -> Option<u64> {
    use uefi::table::cfg::{ACPI_GUID, ACPI2_GUID, ConfigTableEntry};
    use uefi::table;
    use core::slice;
    
    // Debug: Function called
    if verbose {
        write_line("  Debug: find_acpi_rsdp function called");
    }
    
    // Get the system table
    let system_table = match table::system_table_raw() {
        Some(st) => st,
        None => {
            if verbose {
                write_line("  Debug: System table not available");
            }
            return None;
        }
    };
    
    // SAFETY: The system table is valid as it was obtained from the global state
    let st = unsafe { system_table.as_ref() };
    
    // Check if configuration table is available
    if st.configuration_table.is_null() || st.number_of_configuration_table_entries == 0 {
        if verbose {
            write_line("  Debug: Configuration table not available");
        }
        return None;
    }
    
    // SAFETY: We have a valid pointer and count from the system table
    let config_entries = unsafe {
        slice::from_raw_parts(
            st.configuration_table as *const ConfigTableEntry,
            st.number_of_configuration_table_entries
        )
    };
    
    // Search for ACPI 2.0 RSDP first (preferred)
    for (i, entry) in config_entries.iter().enumerate() {
        if verbose {
            write_line(&alloc::format!("  Debug: Entry {} - GUID: {:?}", i, entry.guid));
        }
        
        if entry.guid == ACPI2_GUID {
            if verbose {
                write_line(&alloc::format!("  Debug: Found ACPI 2.0 RSDP at 0x{:016X}", entry.address as u64));
            }
            return Some(entry.address as u64);
        }
    }
    
    // Fall back to ACPI 1.0 RSDP
    for entry in config_entries {
        if entry.guid == ACPI_GUID {
            if verbose {
                write_line(&alloc::format!("  Debug: Found ACPI 1.0 RSDP at 0x{:016X}", entry.address as u64));
            }
            return Some(entry.address as u64);
        }
    }
    
    if verbose {
        write_line("  Debug: No ACPI RSDP table found in configuration table");
    }
    
    // No ACPI table found
    None
}

/// Parse ACPI tables from an RSDP address
/// 
/// This function attempts to parse ACPI tables using the acpi crate,
/// given a valid RSDP table address.
/// 
/// # Arguments
/// 
/// * `serial_handle` - Optional handle for serial output
/// * `rsdp_address` - Physical address of the RSDP table
/// 
/// # Returns
/// 
/// * `Ok(())` - If ACPI tables were successfully parsed
/// * `Err(&'static str)` - If parsing failed
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
            write_line(&alloc::format!("  Platform info available: {}", platform_info.is_ok()));
            
            Ok(())
        }
        Err(e) => {
            write_line(&alloc::format!("✗ Failed to parse ACPI tables: {:?}", e));
            Err("Failed to parse ACPI tables")
        }
    }
}
