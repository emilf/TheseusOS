//! UEFI Memory Management Module
//!
//! This module provides safe wrappers around UEFI memory allocation and virtual memory
//! mapping functions. It implements proper memory management using UEFI Boot Services
//! before calling exit_boot_services.

use crate::drivers::manager::write_line;
use alloc::format;
use uefi::boot::{self, AllocateType};
use uefi::mem::memory_map::{MemoryAttribute, MemoryDescriptor, MemoryMap, MemoryType};
use uefi::runtime;
use uefi::Status;

/// Memory allocation result
pub type MemoryResult<T> = Result<T, Status>;

/// Allocated memory region information
#[derive(Debug, Clone, Copy)]
pub struct MemoryRegion {
    /// Physical address of the allocated memory
    pub physical_address: u64,
    /// Size of the allocated memory in bytes
    pub size: u64,
    /// Number of pages allocated
    pub page_count: u64,
}

/// Allocate memory using UEFI Boot Services
///
/// This function allocates a contiguous block of memory using UEFI's allocate_pages
/// function. The memory is allocated in pages (4KB each) and must be freed before
/// calling exit_boot_services.
///
/// # Arguments
///
/// * `size` - Size in bytes to allocate
/// * `memory_type` - Type of memory to allocate (e.g., LOADER_DATA, CONVENTIONAL)
///
/// # Returns
///
/// * `Ok(MemoryRegion)` - Information about the allocated memory region
/// * `Err(Status)` - UEFI error status if allocation failed
///
/// # Safety
///
/// This function is safe to call before exit_boot_services. The returned memory
/// must be freed using free_memory before calling exit_boot_services.
pub fn allocate_memory(size: u64, memory_type: MemoryType) -> MemoryResult<MemoryRegion> {
    write_line(&format!(
        "Allocating {} bytes using UEFI allocate_pages (type: {:?})",
        size, memory_type
    ));

    let page_size = 4096u64; // UEFI page size is 4KB
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

/// Allocate memory ensuring it does not overlap a forbidden physical range.
/// Retries a few times and increases the request size to get a different page run.
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
                // Free and retry with a different size to perturb allocator
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

/// Free memory using UEFI Boot Services
///
/// This function frees memory that was previously allocated using allocate_memory.
/// The memory must be freed before calling exit_boot_services.
///
/// # Arguments
///
/// * `region` - Memory region to free
///
/// # Returns
///
/// * `Ok(())` - Memory successfully freed
/// * `Err(Status)` - UEFI error status if freeing failed
///
/// # Safety
///
/// This function is safe to call before exit_boot_services. The memory region
/// must have been allocated using allocate_memory.
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

/// Allocate persistent memory for handoff structure
///
/// This function allocates memory that will remain accessible after exit_boot_services.
/// The memory is allocated as LOADER_DATA type as per UEFI specifications.
///
/// # Arguments
///
/// * `size` - Size in bytes to allocate
///
/// # Returns
///
/// * `Ok(MemoryRegion)` - Information about the allocated memory region
/// * `Err(Status)` - UEFI error status if allocation failed
#[allow(dead_code)]
pub fn allocate_persistent_memory(size: u64) -> MemoryResult<MemoryRegion> {
    write_line(&format!(
        "Allocating {} bytes of persistent memory for handoff structure",
        size
    ));

    let page_size = 4096u64; // UEFI page size is 4KB
    let page_count = (size + page_size - 1) / page_size;

    write_line(&format!(
        "  Requesting {} pages ({} bytes) as LOADER_DATA",
        page_count,
        page_count * page_size
    ));

    // Allocate memory using UEFI Boot Services as LOADER_DATA
    // This ensures the memory remains accessible after exit_boot_services
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

/// Set up virtual memory mapping using UEFI Runtime Services
///
/// This function sets up virtual memory mapping using UEFI's set_virtual_address_map
/// function. This is typically called after exit_boot_services to establish
/// virtual memory mappings for runtime services.
///
/// # Arguments
///
/// * `memory_map` - The UEFI memory map to use for virtual mapping
/// * `kernel_physical_base` - Physical base address of the kernel
/// * `kernel_virtual_base` - Virtual base address where kernel should be mapped
///
/// # Returns
///
/// * `Ok(())` - Virtual memory mapping successfully established
/// * `Err(Status)` - UEFI error status if mapping failed
///
/// # Safety
///
/// This function should be called after exit_boot_services. It modifies the
/// virtual memory mapping of the system.
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

    // Create a memory descriptor array for set_virtual_address_map
    // We need to avoid heap allocations after exit_boot_services, so we'll use a fixed-size array
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
            virt_start: if entry.ty == MemoryType::CONVENTIONAL {
                // Map conventional memory 1:1 (identity mapping)
                entry.phys_start
            } else {
                // Keep other memory types at their physical addresses
                entry.phys_start
            },
            page_count: entry.page_count,
            att: entry.att,
        };
        descriptor_count += 1;
    }

    write_line("  Created memory descriptors");

    // Set up kernel mapping: map kernel physical address to virtual address
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

/// Test memory allocation and deallocation
///
/// This function tests the memory allocation system by allocating and freeing
/// various sizes of memory. It's useful for debugging and validation.
///
/// # Returns
///
/// * `Ok(())` - All tests passed
/// * `Err(Status)` - Test failed with UEFI error status
pub fn test_memory_allocation() -> MemoryResult<()> {
    write_line("=== Testing Memory Allocation ===");

    // Test 1: Allocate small block (1 page)
    write_line("Test 1: Allocating 1 page (4KB)");
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
    write_line("Test 2: Allocating 16 pages (64KB)");
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

    // Test 3: Allocate large block (256 pages = 1MB)
    write_line("Test 3: Allocating 256 pages (1MB)");
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
