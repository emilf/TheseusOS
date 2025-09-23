//! MADT (Multiple APIC Description Table) parser
//! 
//! This module provides parsing capabilities for the MADT table, which contains
//! information about APIC controllers, CPU entries, and interrupt sources.

use acpi::AcpiTables;
use crate::display::kernel_write_line;

/// MADT information extracted from ACPI tables
#[derive(Debug, Clone)]
pub struct MadtInfo {
    /// List of CPU APIC IDs found in the system
    pub cpu_apic_ids: alloc::vec::Vec<u8>,
    /// Local APIC address (physical)
    pub local_apic_address: u64,
    /// IO APIC entries
    pub io_apics: alloc::vec::Vec<IoApicEntry>,
    /// Whether the system supports 8259 PIC
    pub has_8259_pic: bool,
}

/// IO APIC entry information
#[derive(Debug, Clone)]
pub struct IoApicEntry {
    /// IO APIC ID
    pub id: u8,
    /// IO APIC address (physical)
    pub address: u64,
    /// Global system interrupt base
    pub gsi_base: u32,
}

/// Parse MADT information from ACPI tables
/// 
/// This function extracts MADT information including CPU APIC IDs,
/// Local APIC address, and IO APIC entries.
/// 
/// # Arguments
/// 
/// * `tables` - Parsed ACPI tables
/// 
/// # Returns
/// 
/// * `Ok(MadtInfo)` - MADT information extracted successfully
/// * `Err(&'static str)` - If parsing failed
pub fn parse_madt(tables: &AcpiTables<impl acpi::AcpiHandler>) -> Result<MadtInfo, &'static str> {
    kernel_write_line("Parsing MADT (Multiple APIC Description Table)...");
    
    let mut info = MadtInfo {
        cpu_apic_ids: alloc::vec::Vec::new(),
        local_apic_address: 0,
        io_apics: alloc::vec::Vec::new(),
        has_8259_pic: false,
    };
    
    // Get platform info to access processor and interrupt information
    let platform_info = tables.platform_info()
        .map_err(|_| "Failed to get platform info")?;
    
    // Extract processor information
    if let Some(processor_info) = &platform_info.processor_info {
        // Add BSP (Bootstrap Processor) - always present
        info.cpu_apic_ids.push(processor_info.boot_processor.local_apic_id as u8);
        kernel_write_line(&alloc::format!("  BSP APIC ID: {}", processor_info.boot_processor.local_apic_id));
        
        // Add Application Processors (APs)
        for ap in &processor_info.application_processors {
            info.cpu_apic_ids.push(ap.local_apic_id as u8);
            kernel_write_line(&alloc::format!("  AP APIC ID: {}", ap.local_apic_id));
        }
    }
    
    // Extract interrupt model information
    match &platform_info.interrupt_model {
        acpi::InterruptModel::Apic(apic_info) => {
            // Extract Local APIC information
            info.local_apic_address = apic_info.local_apic_address as u64;
            kernel_write_line(&alloc::format!("  Local APIC address: 0x{:016X}", info.local_apic_address));
            
            // Extract IO APIC information
            for io_apic in &apic_info.io_apics {
                let entry = IoApicEntry {
                    id: io_apic.id as u8,
                    address: io_apic.address as u64,
                    gsi_base: io_apic.global_system_interrupt_base,
                };
                info.io_apics.push(entry);
                kernel_write_line(&alloc::format!("  IO APIC ID: {}, Address: 0x{:016X}, GSI Base: {}", 
                    io_apic.id, io_apic.address as u64, io_apic.global_system_interrupt_base));
            }
            
            // Check for 8259 PIC support
            info.has_8259_pic = apic_info.also_has_legacy_pics;
        }
        _ => {
            kernel_write_line("  No APIC interrupt model found");
        }
    }
    kernel_write_line(&alloc::format!("  Has 8259 PIC: {}", info.has_8259_pic));
    
    kernel_write_line(&alloc::format!("  Total CPUs found: {}", info.cpu_apic_ids.len()));
    kernel_write_line(&alloc::format!("  IO APICs found: {}", info.io_apics.len()));
    
    Ok(info)
}

/// Get CPU count from MADT information
/// 
/// # Arguments
/// 
/// * `madt_info` - MADT information
/// 
/// # Returns
/// 
/// Total number of CPUs (BSP + APs)
pub fn get_cpu_count(madt_info: &MadtInfo) -> usize {
    madt_info.cpu_apic_ids.len()
}

/// Get APIC IDs of all CPUs
/// 
/// # Arguments
/// 
/// * `madt_info` - MADT information
/// 
/// # Returns
/// 
/// Vector of APIC IDs for all CPUs
pub fn get_cpu_apic_ids(madt_info: &MadtInfo) -> &[u8] {
    &madt_info.cpu_apic_ids
}

/// Get Local APIC physical address
/// 
/// # Arguments
/// 
/// * `madt_info` - MADT information
/// 
/// # Returns
/// 
/// Physical address of the Local APIC
pub fn get_local_apic_address(madt_info: &MadtInfo) -> u64 {
    madt_info.local_apic_address
}

/// Get IO APIC entries
/// 
/// # Arguments
/// 
/// * `madt_info` - MADT information
/// 
/// # Returns
/// 
/// Vector of IO APIC entries
pub fn get_io_apics(madt_info: &MadtInfo) -> &[IoApicEntry] {
    &madt_info.io_apics
}
