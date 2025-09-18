//! System information collection module
//! 
//! This module provides functions to collect various system information
//! during UEFI boot, including device tree, firmware, boot time, and CPU information.

use acpi::AcpiTables;
use crate::acpi::UefiAcpiHandler;
use crate::drivers::manager::write_line;

/// Find the device tree blob (DTB)
/// 
/// This function searches for a device tree blob, which is typically used
/// on ARM/ARM64 systems for hardware description.
/// 
/// # Returns
/// 
/// * `Some((u64, u64))` - Tuple of (DTB pointer, DTB size) if found
/// * `None` - If no device tree is available
/// 
/// # Note
/// 
/// This is currently a placeholder implementation. Device tree support
/// needs to be implemented based on the target architecture.
pub fn find_device_tree() -> Option<(u64, u64)> {
    None
}

/// Collect firmware information
/// 
/// This function gathers information about the UEFI firmware, including
/// vendor string and revision information.
/// 
/// # Returns
/// 
/// * `Some((u64, u32, u32))` - Tuple of (vendor_ptr, vendor_len, revision) if available
/// * `None` - If firmware information is not available
/// 
/// # Note
/// 
/// This is currently a placeholder implementation. Firmware information
/// collection needs to be implemented using UEFI system table access.
pub fn collect_firmware_info() -> Option<(u64, u32, u32)> {
    None
}

/// Collect boot time information
/// 
/// This function gathers information about when the boot process started,
/// including timestamp and timing information.
/// 
/// # Returns
/// 
/// * `Some((u64, u32))` - Tuple of (seconds, nanoseconds) if available
/// * `None` - If boot time information is not available
/// 
/// # Note
/// 
/// This is currently a placeholder implementation. Boot time information
/// collection needs to be implemented using UEFI runtime services.
pub fn collect_boot_time_info() -> Option<(u64, u32)> {
    None
}

/// Collect boot device path information
/// 
/// This function gathers information about the device path from which
/// the current EFI application was booted.
/// 
/// # Returns
/// 
/// * `Some((u64, u32))` - Tuple of (device_path_ptr, device_path_size) if available
/// * `None` - If boot device path information is not available
/// 
/// # Note
/// 
/// This is currently a placeholder implementation. Boot device path
/// collection needs to be implemented using the LoadedImage protocol.
pub fn collect_boot_device_path() -> Option<(u64, u32)> {
    None
}

/// Collect CPU information using ACPI tables (MADT)
/// 
/// Returns (cpu_count, cpu_features, microcode_revision). Features and microcode
/// are placeholders for now; proper CPUID/MSR probing can be added later.
pub fn collect_cpu_info() -> Option<(u32, u64, u32)> {
    // We rely on a previously discovered RSDP stored in the handoff
    let rsdp = unsafe { theseus_shared::handoff::HANDOFF.acpi_rsdp };
    if rsdp == 0 {
        write_line("  CPU: No RSDP available; cannot determine CPU count");
        return None;
    }

    // Parse ACPI tables via the `acpi` crate
    let tables = match unsafe { AcpiTables::from_rsdp(UefiAcpiHandler, rsdp as usize) } {
        Ok(t) => t,
        Err(e) => {
            write_line(&alloc::format!("  CPU: Failed to parse ACPI tables: {:?}", e));
            return None;
        }
    };

    let platform = match tables.platform_info() {
        Ok(p) => p,
        Err(e) => {
            write_line(&alloc::format!("  CPU: Failed to get platform info: {:?}", e));
            return None;
        }
    };

    // Processor information (MADT)
    let proc_info = match platform.processor_info {
        Some(pi) => pi,
        None => {
            write_line("  CPU: No processor info in ACPI tables");
            return None;
        }
    };

    // Count boot processor + application processors
    let mut count: u32 = 1; // BSP
    for _ap in proc_info.application_processors.iter() {
        // Count all AP entries; refine with state filtering later if needed
        count += 1;
    }

    write_line(&alloc::format!("✓ CPU count determined from ACPI: {}", count));

    let cpu_features: u64 = 0; // TODO: CPUID feature bits
    let microcode_revision: u32 = 0; // TODO: Read microcode revision if desired
    Some((count, cpu_features, microcode_revision))
}
