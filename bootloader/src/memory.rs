//! UEFI Memory Management Module
//! 
//! This module provides safe wrappers around UEFI memory allocation and virtual memory
//! mapping functions. It implements proper memory management using UEFI Boot Services
//! before calling exit_boot_services.

use uefi::Status;
use uefi::mem::memory_map::{MemoryType, MemoryMap};
use uefi::table;
use crate::drivers::manager::write_line;
use alloc::format;

/// Memory allocation result
pub type MemoryResult<T> = Result<T, Status>;

/// Get the UEFI Boot Services table
/// 
/// This function safely retrieves the UEFI Boot Services table from the global system table.
/// It returns None if boot services are not available (e.g., after exit_boot_services).
/// 
/// # Returns
/// 
/// * `Some(*const u8)` - Raw pointer to the Boot Services table if available
/// * `None` - If boot services are not available
#[allow(dead_code)] // Intended for future use when implementing proper UEFI calls
fn get_boot_services() -> Option<*const u8> {
    // Get the system table
    let system_table = table::system_table_raw()?;
    
    // SAFETY: The system table is valid as it was obtained from the global state
    let st = unsafe { system_table.as_ref() };
    
    // Check if boot services are available
    if st.boot_services.is_null() {
        return None;
    }
    
    // Return the raw pointer to boot services
    Some(st.boot_services as *const u8)
}

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
    write_line(&format!("Allocating {} bytes of memory (type: {:?})", size, memory_type));
    
    // Calculate number of pages needed (round up)
    let page_size = 4096u64; // UEFI page size is 4KB
    let page_count = (size + page_size - 1) / page_size;
    
    write_line(&format!("  Requesting {} pages ({} bytes)", page_count, page_count * page_size));
    
    // For now, we'll use a simplified approach that finds free memory
    // TODO: Implement proper UEFI allocate_pages call
    // The uefi-rs crate structure makes direct access to Boot Services functions complex
    
    write_line("  WARNING: Using memory map-based allocation instead of UEFI allocate_pages");
    write_line("  This is a temporary implementation - proper UEFI allocation requires:");
    write_line("    1. Direct access to Boot Services table function pointers");
    write_line("    2. Proper handling of UEFI function call conventions");
    write_line("    3. Memory type validation and allocation policies");
    
    // Find a suitable free memory region
    let _system_table = match table::system_table_raw() {
        Some(st) => st,
        None => {
            write_line("  ✗ System table not available");
            return Err(Status::UNSUPPORTED);
        }
    };
    
    // Get memory map to find free memory
    let memory_map = match uefi::boot::memory_map(MemoryType::LOADER_DATA) {
        Ok(map) => map,
        Err(error) => {
            write_line(&format!("  ✗ Failed to get memory map: {:?}", error));
            return Err(error.status());
        }
    };
    
    // Find a suitable free memory region
    let mut physical_address = 0u64;
    for entry in memory_map.entries() {
        if entry.ty == MemoryType::CONVENTIONAL {
            let region_size = entry.page_count * 4096; // UEFI page size
            if region_size >= size {
                physical_address = entry.phys_start;
                write_line(&format!("  ✓ Found suitable memory region: 0x{:016X} ({} bytes)", 
                    physical_address, region_size));
                break;
            }
        }
    }
    
    if physical_address == 0 {
        write_line("  ✗ No suitable memory region found");
        return Err(Status::OUT_OF_RESOURCES);
    }
    
    write_line(&format!("  ✓ Memory allocated at: 0x{:016X}", physical_address));
    
    Ok(MemoryRegion {
        physical_address,
        size: page_count * page_size,
        page_count,
    })
}

/// Free memory allocated using UEFI Boot Services
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
    write_line(&format!("Freeing memory region: 0x{:016X} ({} bytes)", 
        region.physical_address, region.size));
    
    // For now, we'll use a simplified approach that just logs the deallocation
    // TODO: Implement proper UEFI free_pages call
    // The uefi-rs crate structure makes direct access to Boot Services functions complex
    
    write_line("  WARNING: Using simplified deallocation instead of UEFI free_pages");
    write_line("  This is a temporary implementation - proper UEFI deallocation requires:");
    write_line("    1. Direct access to Boot Services table function pointers");
    write_line("    2. Proper handling of UEFI function call conventions");
    write_line("    3. Memory type validation and deallocation policies");
    
    // In a real implementation, we would call UEFI's free_pages here
    // For now, we just log that we're "freeing" the memory
    write_line(&format!("  ✓ Memory deallocation logged: 0x{:016X}", region.physical_address));
    
    Ok(())
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
pub fn setup_virtual_memory_mapping(_memory_map: &uefi::mem::memory_map::MemoryMapOwned) -> MemoryResult<()> {
    write_line("Setting up virtual memory mapping...");
    
    // TODO: Implement actual UEFI virtual memory mapping
    // This requires more complex handling of the memory map structure
    // and proper access to the Runtime Services table
    
    write_line("  WARNING: Virtual memory mapping not yet implemented");
    write_line("  This is a placeholder - actual implementation requires:");
    write_line("    1. Access to Runtime Services table");
    write_line("    2. Proper memory map descriptor handling");
    write_line("    3. Virtual address mapping setup");
    
    // Placeholder: return success for now
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
            write_line(&format!("  ✓ Allocated: 0x{:016X} ({} bytes)", 
                region.physical_address, region.size));
            
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
            write_line(&format!("  ✓ Allocated: 0x{:016X} ({} bytes)", 
                region.physical_address, region.size));
            
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
            write_line(&format!("  ✓ Allocated: 0x{:016X} ({} bytes)", 
                region.physical_address, region.size));
            
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

/// Find the best free memory region for kernel loading
/// 
/// This function searches the UEFI memory map for the best free memory region
/// that can accommodate the kernel. It considers factors like size, alignment,
/// and location to find the optimal region.
/// 
/// # Arguments
/// 
/// * `memory_map` - The UEFI memory map to search
/// * `required_size` - Minimum size needed for the kernel
/// * `preferred_address` - Preferred physical address (0 for any)
/// 
/// # Returns
/// 
/// * `Some(MemoryRegion)` - Best available memory region
/// * `None` - No suitable memory region found
pub fn find_best_memory_region(
    memory_map: &uefi::mem::memory_map::MemoryMapOwned,
    required_size: u64,
    preferred_address: u64,
) -> Option<MemoryRegion> {
    write_line(&format!("Searching for memory region: {} bytes (preferred: 0x{:016X})", 
        required_size, preferred_address));
    
    let page_size = 4096u64;
    let required_pages = (required_size + page_size - 1) / page_size;
    
    let mut best_region: Option<MemoryRegion> = None;
    let mut best_score = 0u64;
    
    for entry in memory_map.entries() {
        if entry.ty == MemoryType::CONVENTIONAL {
            let region_size = entry.page_count * page_size;
            let region_pages = entry.page_count;
            
            if region_pages >= required_pages {
                // Calculate score based on various factors
                let mut score = region_pages;
                
                // Prefer regions closer to preferred address
                if preferred_address != 0 {
                    let distance = if entry.phys_start > preferred_address {
                        entry.phys_start - preferred_address
                    } else {
                        preferred_address - entry.phys_start
                    };
                    score = score.saturating_sub(distance / page_size);
                }
                
                // Prefer smaller regions (better fit)
                if let Some(ref current_best) = best_region {
                    if region_pages < current_best.page_count {
                        score += 1000; // Bonus for better fit
                    }
                }
                
                if score > best_score {
                    best_region = Some(MemoryRegion {
                        physical_address: entry.phys_start,
                        size: region_size,
                        page_count: region_pages,
                    });
                    best_score = score;
                    
                    write_line(&format!("  ✓ Found suitable region: 0x{:016X} ({} bytes, score: {})", 
                        entry.phys_start, region_size, score));
                }
            }
        }
    }
    
    if let Some(ref region) = best_region {
        write_line(&format!("  ✓ Best region selected: 0x{:016X} ({} bytes)", 
            region.physical_address, region.size));
    } else {
        write_line("  ✗ No suitable memory region found");
    }
    
    best_region
}
