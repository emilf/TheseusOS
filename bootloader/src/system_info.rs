//! System information collection module
//! 
//! This module provides functions to collect various system information
//! during UEFI boot, including firmware, boot time, and CPU information.

use acpi::AcpiTables;
use crate::acpi::UefiAcpiHandler;
use crate::drivers::manager::write_line;
use uefi::table;
use uefi::proto::loaded_image::LoadedImage;

// Device tree support removed (x86-only build)

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
    let system_table = match table::system_table_raw() {
        Some(st) => st,
        None => return None,
    };
    let st = unsafe { system_table.as_ref() };

    // Firmware vendor is a UCS-2 string pointer in system table
    let vendor_ptr = st.firmware_vendor as u64;
    let revision = st.firmware_revision;

    // Best-effort length: walk until NUL (16-bit) with a reasonable cap
    let mut len_chars: u32 = 0;
    if !st.firmware_vendor.is_null() {
        unsafe {
            let mut p = st.firmware_vendor;
            let mut count: u32 = 0;
            while count < 256 {
                if core::ptr::read(p) == 0 { break; }
                p = p.add(1);
                count += 1;
            }
            len_chars = count;
        }
    }

    Some((vendor_ptr, len_chars, revision))
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
    // Use UEFI runtime service get_time(). Convert to UNIX seconds if possible.
    let system_table = match table::system_table_raw() {
        Some(st) => st,
        None => return None,
    };
    let st = unsafe { system_table.as_ref() };
    if st.runtime_services.is_null() { return None; }

    // Safe wrapper provided by crate: uefi::runtime::get_time()
    let tm = match uefi::runtime::get_time() {
        Ok(t) => t,
        Err(_) => return None,
    };

    // Convert UEFI time (year, month, day, hour, minute, second) to UNIX seconds.
    // Implement minimal days-since-epoch calculation (UTC; leap years considered).
    let y = tm.year() as i32;
    let m = tm.month() as i32; // 1..=12
    let d = tm.day() as i32;   // 1..=31
    let hour = tm.hour() as i64;
    let min = tm.minute() as i64;
    let sec = tm.second() as i64;

    // Days from 1970-01-01 to current date
    fn is_leap(y: i32) -> bool { (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0) }
    const MDAYS: [i32; 12] = [31,28,31,30,31,30,31,31,30,31,30,31];
    let mut days: i64 = 0;
    let mut yy = 1970;
    while yy < y { days += if is_leap(yy) { 366 } else { 365 } as i64; yy += 1; }
    let mut mm = 1;
    while mm < m { days += (if mm == 2 && is_leap(y) { 29 } else { MDAYS[(mm-1) as usize] }) as i64; mm += 1; }
    days += (d as i64) - 1;

    let seconds = days * 86400 + hour * 3600 + min * 60 + sec;
    let nanos = (tm.nanosecond() as u32).min(999_999_999);
    Some((seconds as u64, nanos))
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
    // Prefer using our existing helper in hardware.rs, but implement a local fallback.
    // Try to open LoadedImage and extract file_path pointer.
    let image_handle = uefi::boot::image_handle();
    let loaded_image = match uefi::boot::open_protocol_exclusive::<LoadedImage>(image_handle) {
        Ok(img) => img,
        Err(_) => return None,
    };
    if let Some(dp) = loaded_image.file_path() {
        let ptr = core::ptr::addr_of!(*dp) as *const u8 as usize as u64;
        Some((ptr, 0))
    } else {
        None
    }
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

    let cpu_features: u64 = 0; // Note: CPUID feature detection is handled in kernel
    let microcode_revision: u32 = 0; // Note: Microcode revision reading deferred to kernel
    Some((count, cpu_features, microcode_revision))
}
