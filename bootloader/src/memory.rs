//! Module: bootloader::memory
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//! - docs/axioms/memory.md#A3:-The-boot-path-keeps-a-temporary-heap-before-switching-to-a-permanent-kernel-heap
//!
//! INVARIANTS:
//! - This module owns the firmware-side allocation helpers used before boot services are exited.
//! - Temporary-heap allocation and handoff-copy memory allocation are part of the current single-binary boot contract.
//! - Allocation helpers here operate on UEFI boot-services semantics, not on the later kernel memory model.
//!
//! SAFETY:
//! - Firmware-side allocations must remain valid across the exact transition paths that depend on them.
//! - The non-overlapping allocation helper is a pragmatic boot-time strategy, not proof that the kernel image extent accounting is exact.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//!
//! Firmware-side memory allocation helpers for the boot path.

use crate::drivers::manager::write_line;
use alloc::format;
use uefi::boot::{self, AllocateType};
use uefi::mem::memory_map::{MemoryAttribute, MemoryDescriptor, MemoryMap, MemoryType};
use uefi::runtime;
use uefi::Status;

/// Result type for bootloader-side UEFI memory operations.
pub type MemoryResult<T> = Result<T, Status>;

/// Metadata describing an allocated firmware memory region.
#[derive(Debug, Clone, Copy)]
pub struct MemoryRegion {
    /// Physical start address of the allocation.
    pub physical_address: u64,
    /// Allocation size in bytes.
    pub size: u64,
    /// Number of 4 KiB pages backing the allocation.
    pub page_count: u64,
}

/// Allocate a contiguous firmware-managed memory region.
///
/// The returned region follows UEFI boot-services lifetime rules; callers must
/// free it or intentionally carry it across the handoff boundary by design.
pub fn allocate_memory(size: u64, memory_type: MemoryType) -> MemoryResult<MemoryRegion> {
    write_line(&format!(
        "Allocating {} bytes using UEFI allocate_pages (type: {:?})",
        size, memory_type
    ));

    let page_size = 4096u64; // UEFI page size is 4 KiB
    let page_count = (size + page_size - 1) / page_size;

    write_line(&format!(
        "  Requesting {} pages ({} bytes)",
        page_count,
        page_count * page_size
    ));

    // Allocate memory using UEFI Boot Services
    let memory_ptr =
        match boot::allocate_pages(AllocateType::AnyPages, memory_type, page_count as usize) {
            Ok(ptr) => ptr,
            Err(err) => {
                write_line(&format!(
                    "  ✗ UEFI allocate_pages failed with error: {:?}",
                    err
                ));
                return Err(err.status());
            }
        };

    let physical_address = memory_ptr.as_ptr() as u64;
    write_line(&format!(
        "  ✓ UEFI memory allocated at: 0x{:016X}",
        physical_address
    ));

    Ok(MemoryRegion {
        physical_address,
        size: page_count * page_size,
        page_count,
    })
}

/// Allocate a region that avoids a forbidden physical-address range.
///
/// This is a boot-time mitigation strategy, not proof that the forbidden range is
/// an exact kernel-image extent.
pub fn allocate_memory_non_overlapping(
    size: u64,
    memory_type: MemoryType,
    forbid_start: u64,
    forbid_end: u64,
) -> MemoryResult<MemoryRegion> {
    let mut attempt: u32 = 0;
    let mut req_size = size;
    while attempt < 8 {
        match allocate_memory(req_size, memory_type) {
            Ok(region) => {
                let start = region.physical_address;
                let end = start.saturating_add(region.size);
                let overlap = !(end <= forbid_start || start >= forbid_end);
                if !overlap {
                    return Ok(region);
                }
                // Free and retry with a slightly different size to perturb the allocator.
                let _ = free_memory(region);
            }
            Err(_) => {
                // fall through to retry with larger size
            }
        }
        attempt += 1;
        req_size = req_size.saturating_add(4096);
    }
    Err(Status::OUT_OF_RESOURCES)
}

/// Free a region previously allocated through the bootloader UEFI wrappers.
///
/// # Arguments
///
/// * `region` - Memory region to free
///
/// # Returns
///
/// * `Ok(())` - Memory was freed successfully
/// * `Err(Status)` - UEFI free failed
///
/// # Note
///
/// The region must originate from the matching bootloader allocation path.
pub fn free_memory(region: MemoryRegion) -> MemoryResult<()> {
    write_line(&format!(
        "Freeing UEFI memory region: 0x{:016X} ({} bytes)",
        region.physical_address, region.size
    ));

    // Free memory using UEFI Boot Services
    // Convert physical address back to NonNull<u8>
    let memory_ptr = match core::ptr::NonNull::new(region.physical_address as *mut u8) {
        Some(ptr) => ptr,
        None => {
            write_line("  ✗ Invalid memory pointer for free_pages");
            return Err(Status::INVALID_PARAMETER);
        }
    };

    match unsafe { boot::free_pages(memory_ptr, region.page_count as usize) } {
        Ok(()) => {
            write_line(&format!(
                "  ✓ UEFI memory freed: 0x{:016X}",
                region.physical_address
            ));
        }
        Err(err) => {
            write_line(&format!("  ✗ UEFI free_pages failed with error: {:?}", err));
            return Err(err.status());
        }
    }

    Ok(())
}

/// Allocate `LOADER_DATA` intended to remain usable across the handoff boundary.
///
/// # Arguments
///
/// * `size` - Minimum size in bytes to allocate
///
/// # Returns
///
/// * `Ok(MemoryRegion)` - Information about the allocated region
/// * `Err(Status)` - UEFI allocation failed
#[allow(dead_code)]
pub fn allocate_persistent_memory(size: u64) -> MemoryResult<MemoryRegion> {
    write_line(&format!(
        "Allocating {} bytes of persistent LOADER_DATA for the handoff structure",
        size
    ));

    let page_size = 4096u64; // UEFI page size is 4 KiB
    let page_count = (size + page_size - 1) / page_size;

    write_line(&format!(
        "  Requesting {} pages ({} bytes) as LOADER_DATA",
        page_count,
        page_count * page_size
    ));

    // Allocate `LOADER_DATA` so the region remains suitable for the handoff path.
    let memory_ptr = match boot::allocate_pages(
        AllocateType::AnyPages,
        MemoryType::LOADER_DATA,
        page_count as usize,
    ) {
        Ok(ptr) => ptr,
        Err(err) => {
            write_line(&format!(
                "  ✗ UEFI allocate_pages failed with error: {:?}",
                err
            ));
            return Err(err.status());
        }
    };

    let physical_address = memory_ptr.as_ptr() as u64;
    write_line(&format!(
        "  ✓ Persistent memory allocated at: 0x{:016X}",
        physical_address
    ));

    Ok(MemoryRegion {
        physical_address,
        size: page_count * page_size,
        page_count,
    })
}

/// Attempt UEFI runtime virtual-address-map setup.
///
/// This helper prepares a descriptor array for `set_virtual_address_map()` and is
/// intended for firmware-runtime mapping experiments after boot services are gone.
/// It is not the main architectural path the current kernel relies on for its own
/// higher-half paging model.
#[allow(dead_code)]
pub fn setup_virtual_memory_mapping(
    memory_map: &uefi::mem::memory_map::MemoryMapOwned,
    kernel_physical_base: u64,
    kernel_virtual_base: u64,
) -> MemoryResult<()> {
    write_line("Setting up virtual memory mapping...");

    // Get the system table to access runtime services
    let system_table = match uefi::table::system_table_raw() {
        Some(st) => st,
        None => {
            write_line("  ✗ System table not available");
            return Err(Status::UNSUPPORTED);
        }
    };

    // SAFETY: We have a valid system table pointer
    let st = unsafe { &*system_table.as_ptr() };

    // Check if runtime services are available
    if st.runtime_services.is_null() {
        write_line("  ✗ Runtime services not available");
        return Err(Status::UNSUPPORTED);
    }

    write_line("  ✓ Runtime services available");

    // Build a fixed-size descriptor array because this path is meant for the
    // post-boot-services phase where ad-hoc heap use is undesirable.
    const MAX_DESCRIPTORS: usize = 200; // Should be enough for most systems
    let mut descriptors: [MemoryDescriptor; MAX_DESCRIPTORS] = unsafe { core::mem::zeroed() };
    let mut descriptor_count = 0;

    // Convert memory map entries to descriptors
    for entry in memory_map.entries() {
        if descriptor_count >= MAX_DESCRIPTORS {
            write_line("  ⚠ Too many memory descriptors, truncating");
            break;
        }

        descriptors[descriptor_count] = MemoryDescriptor {
            ty: entry.ty,
            phys_start: entry.phys_start,
            virt_start: entry.phys_start,
            page_count: entry.page_count,
            att: entry.att,
        };
        descriptor_count += 1;
    }

    write_line("  Created memory descriptors");

    // Add a simple kernel mapping descriptor for the runtime-services experiment.
    if descriptor_count < MAX_DESCRIPTORS {
        descriptors[descriptor_count] = MemoryDescriptor {
            ty: MemoryType::LOADER_DATA,
            phys_start: kernel_physical_base,
            virt_start: kernel_virtual_base,
            page_count: 32, // 32 pages = 128KB for kernel
            att: MemoryAttribute::from_bits_truncate(0x0000000f), // Standard attributes
        };
        descriptor_count += 1;
        write_line("  Added kernel mapping");
    } else {
        write_line("  ⚠ No space for kernel mapping descriptor");
    }

    // Call set_virtual_address_map
    write_line("  Calling UEFI set_virtual_address_map...");

    // SAFETY: We're calling this after exit_boot_services as required
    match unsafe {
        runtime::set_virtual_address_map(
            &mut descriptors[..descriptor_count],
            system_table.as_ptr(),
        )
    } {
        Ok(()) => {
            write_line("  ✓ Virtual memory mapping established successfully");
            write_line("  ✓ Kernel mapped to virtual address");
        }
        Err(err) => {
            write_line("  ✗ set_virtual_address_map failed");
            return Err(err.status());
        }
    }

    Ok(())
}

/// Run a small UEFI allocation/free smoke test.
///
/// This is optional boot-time validation for the firmware allocator wrappers.
/// The normal repo workflow keeps it disabled for faster, quieter boots.
pub fn test_memory_allocation() -> MemoryResult<()> {
    write_line("=== Testing Memory Allocation ===");

    // Test 1: Allocate small block (1 page)
    write_line("Test 1: Allocating 1 page (4 KiB)");
    match allocate_memory(4096, MemoryType::LOADER_DATA) {
        Ok(region) => {
            write_line(&format!(
                "  ✓ Allocated: 0x{:016X} ({} bytes)",
                region.physical_address, region.size
            ));

            // Free the memory
            free_memory(region)?;
            write_line("  ✓ Freed successfully");
        }
        Err(status) => {
            write_line(&format!("  ✗ Allocation failed: {:?}", status));
            return Err(status);
        }
    }

    // Test 2: Allocate medium block (16 pages)
    write_line("Test 2: Allocating 16 pages (64 KiB)");
    match allocate_memory(65536, MemoryType::LOADER_DATA) {
        Ok(region) => {
            write_line(&format!(
                "  ✓ Allocated: 0x{:016X} ({} bytes)",
                region.physical_address, region.size
            ));

            // Free the memory
            free_memory(region)?;
            write_line("  ✓ Freed successfully");
        }
        Err(status) => {
            write_line(&format!("  ✗ Allocation failed: {:?}", status));
            return Err(status);
        }
    }

    // Test 3: Allocate large block (256 pages = 1 MiB)
    write_line("Test 3: Allocating 256 pages (1 MiB)");
    match allocate_memory(1048576, MemoryType::LOADER_DATA) {
        Ok(region) => {
            write_line(&format!(
                "  ✓ Allocated: 0x{:016X} ({} bytes)",
                region.physical_address, region.size
            ));

            // Free the memory
            free_memory(region)?;
            write_line("  ✓ Freed successfully");
        }
        Err(status) => {
            write_line(&format!("  ✗ Allocation failed: {:?}", status));
            return Err(status);
        }
    }

    write_line("=== Memory Allocation Tests Complete ===");
    Ok(())
}
