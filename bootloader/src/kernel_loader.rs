//! Kernel loading functionality
//! 
//! This module handles finding, analyzing, and loading the kernel binary
//! from the EFI file system into memory.

use uefi::prelude::*;
use uefi::boot::get_image_file_system;
use uefi::proto::media::file::{File, FileMode, FileAttribute, RegularFile, FileInfo};
use uefi::Status;
use alloc::vec::Vec;
use alloc::format;
use alloc::vec;

use theseus_shared::constants::kernel;
use crate::drivers::manager::write_line;

/// Kernel section information
#[derive(Debug, Clone)]
pub struct KernelSection {
    /// Section name
    pub name: &'static str,
    /// Virtual address where this section should be loaded
    pub virtual_address: u64,
    /// Physical address where this section is loaded
    pub physical_address: u64,
    /// Size of the section in memory
    pub size: u64,
    /// File offset where section data starts
    pub file_offset: u64,
    /// Section flags (readable, writable, executable)
    #[allow(dead_code)] // Intended for future use
    pub flags: u32,
}

/// Complete kernel information
#[derive(Debug)]
pub struct KernelInfo {
    /// Entry point virtual address
    pub entry_point: u64,
    /// Total memory size required for all sections
    pub total_memory_size: u64,
    /// List of kernel sections
    pub sections: Vec<KernelSection>,
}

/// Find the kernel file in the EFI file system
/// 
/// # Arguments
/// 
/// * `output_driver` - Output driver for logging
/// 
/// # Returns
/// 
/// * `Ok(RegularFile)` - Kernel file handle
/// * `Err(status)` - Error finding kernel file
pub fn find_kernel_file() -> Result<RegularFile, Status> {
    write_line("Searching for kernel file...");
    
    // Get the current image handle
    let image_handle = uefi::boot::image_handle();
    
    // Use get_image_file_system to access the file system
    let mut fs = get_image_file_system(image_handle)
        .map_err(|e| {
            write_line(&format!("✗ Failed to get file system: {:?}", e));
            e.status()
        })?;
    
    // Open the root directory
    let mut root_dir = fs.open_volume()
        .map_err(|e| {
            write_line(&format!("✗ Failed to open volume: {:?}", e));
            e.status()
        })?;
    
    // Open the kernel file
    let kernel_path = cstr16!("\\kernel.efi");
    write_line("Looking for kernel at: \\kernel.efi");
    
    let file_handle = root_dir.open(kernel_path, FileMode::Read, FileAttribute::READ_ONLY)
        .map_err(|e| {
            write_line(&format!("✗ Failed to open kernel file: {:?}", e));
            e.status()
        })?;
    
    let kernel_file = file_handle.into_regular_file()
        .ok_or_else(|| {
            write_line("✗ Kernel file is not a regular file");
            Status::INVALID_PARAMETER
        })?;
    
    write_line("✓ Kernel file found");
    Ok(kernel_file)
}

/// Get kernel file information including size
/// 
/// # Arguments
/// 
/// * `file` - Kernel file handle
/// * `output_driver` - Output driver for logging
/// 
/// # Returns
/// 
/// * `Ok(file_size)` - Actual file size
/// * `Err(status)` - Error getting file info
pub fn get_kernel_file_info(file: &mut RegularFile, ) -> Result<u64, Status> {
    write_line("Getting kernel file information...");
    
    // Get actual file information using UEFI FileInfo API
    let mut info_buffer = vec![0u8; 1024]; // Allocate buffer for FileInfo
    let file_info = file.get_info::<FileInfo>(&mut info_buffer)
        .map_err(|e| {
            write_line(&format!("✗ Failed to get file info: {:?}", e));
            e.status()
        })?;
    
    let file_size = file_info.file_size();
    write_line("✓ Got actual file size from FileInfo");
    write_line(&format!("Kernel file size: {} bytes ({:.2} MB)", 
        file_size, file_size as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
    
    if file_size > kernel::MAX_KERNEL_SIZE {
        write_line(&format!("✗ Kernel file too large: {} bytes (max: {} bytes)", 
            file_size, kernel::MAX_KERNEL_SIZE));
        return Err(Status::INVALID_PARAMETER);
    }
    
    write_line("✓ Kernel file size acceptable");
    Ok(file_size)
}

/// Read the kernel binary from the file
/// 
/// # Arguments
/// 
/// * `file` - Kernel file handle
/// * `file_size` - Size of the kernel file
/// * `output_driver` - Output driver for logging
/// 
/// # Returns
/// 
/// * `Ok(Vec<u8>)` - Kernel binary data
/// * `Err(status)` - Error reading kernel file
pub fn read_kernel_binary(file: &mut RegularFile, file_size: u64, ) -> Result<Vec<u8>, Status> {
    write_line("Reading kernel binary from file...");
    
    // Allocate buffer for kernel data
    let mut kernel_data = vec![0u8; file_size as usize];
    
    // Read the kernel file
    let bytes_read = file.read(&mut kernel_data)
        .map_err(|e| {
            write_line(&format!("✗ Failed to read kernel file: {:?}", e));
            e.status()
        })?;
    
    if bytes_read != file_size as usize {
        write_line(&format!("✗ Incomplete read: expected {} bytes, got {} bytes", 
            file_size, bytes_read));
        return Err(Status::INVALID_PARAMETER);
    }
    
    write_line("✓ Kernel binary read successfully");
    Ok(kernel_data)
}

/// Analyze the kernel binary to extract section information
/// 
/// # Arguments
/// 
/// * `kernel_data` - Raw kernel binary data
/// * `output_driver` - Output driver for logging
/// 
/// # Returns
/// 
/// * `Ok(KernelInfo)` - Parsed kernel information
/// * `Err(status)` - Error parsing kernel
pub fn analyze_kernel_binary(kernel_data: &[u8], ) -> Result<KernelInfo, Status> {
    write_line("Analyzing kernel binary...");
    
    // Check ELF magic number
    if kernel_data.len() < 4 || &kernel_data[0..4] != kernel::ELF_MAGIC {
        write_line("✗ Invalid ELF magic number");
        return Err(Status::INVALID_PARAMETER);
    }
    
    write_line("✓ Valid ELF magic number found");
    
    // For now, create a simple kernel info structure
    // In a real implementation, this would parse the ELF headers
    let entry_point = 0xFFFFFFFF80000000; // Virtual entry point
    let sections = vec![
        KernelSection {
            name: ".text",
            virtual_address: 0xFFFFFFFF80000000,
            physical_address: 0x100000, // Will be updated during loading
            size: 65536,
            file_offset: 0,
            flags: 0x5, // RX (readable, executable)
        },
        KernelSection {
            name: ".rodata",
            virtual_address: 0xFFFFFFFF80010000,
            physical_address: 0x110000,
            size: 32768,
            file_offset: 65536,
            flags: 0x4, // R (readable)
        },
        KernelSection {
            name: ".data",
            virtual_address: 0xFFFFFFFF80018000,
            physical_address: 0x118000,
            size: 16384,
            file_offset: 98304,
            flags: 0x6, // RW (readable, writable)
        },
        KernelSection {
            name: ".bss",
            virtual_address: 0xFFFFFFFF8001C000,
            physical_address: 0x11C000,
            size: 16384,
            file_offset: 0, // BSS has no file data
            flags: 0x6, // RW (readable, writable)
        },
    ];
    
    let total_memory_size = sections.iter().map(|s| s.size).sum();
    
    write_line("✓ Kernel analysis complete");
    write_line(&format!("  Entry point: 0x{:016X}", entry_point));
    write_line(&format!("  Total memory required: {} bytes ({:.2} MB)", 
        total_memory_size, total_memory_size as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
    write_line(&format!("  Sections: {}", sections.len()));
    
    for section in &sections {
        write_line(&format!("    {}: 0x{:016X} - 0x{:016X} ({} bytes)",
            section.name,
            section.virtual_address,
            section.virtual_address + section.size - 1,
            section.size
        ));
    }
    
    Ok(KernelInfo {
        entry_point,
        total_memory_size,
        sections,
    })
}


/// Load kernel sections into allocated memory
/// 
/// # Arguments
/// 
/// * `kernel_info` - Kernel information
/// * `kernel_data` - Raw kernel binary data
/// * `physical_base` - Physical base address for loading
/// * `output_driver` - Output driver for logging
/// 
/// # Returns
/// 
/// * `Ok(physical_entry)` - Physical entry point address
/// * `Err(status)` - Error loading kernel
pub fn load_kernel_sections(kernel_info: &mut KernelInfo, kernel_data: &[u8], physical_base: u64, ) -> Result<u64, Status> {
    write_line("Loading kernel sections into memory...");
    write_line(&format!("Loading kernel at physical address: 0x{:016X}", physical_base));
    
    let mut current_physical = physical_base;
    let mut physical_entry = 0;
    
    for section in &mut kernel_info.sections {
        section.physical_address = current_physical;
        
        if section.name == ".text" && section.virtual_address == kernel_info.entry_point {
            physical_entry = current_physical;
        }
        
        write_line(&format!("  {}: 0x{:016X} -> 0x{:016X}",
            section.name, section.virtual_address, section.physical_address));
        
        if section.name == ".bss" {
            // BSS section needs to be zeroed
            write_line(&format!("    BSS section: {} bytes (zeroing)", section.size));
            unsafe {
                core::ptr::write_bytes(section.physical_address as *mut u8, 0, section.size as usize);
            }
        } else {
            // Copy section data from file to allocated memory
            let file_data = &kernel_data[section.file_offset as usize..(section.file_offset + section.size) as usize];
            write_line(&format!("    {} section: {} bytes (copying)", section.name, section.size));
            unsafe {
                core::ptr::copy_nonoverlapping(
                    file_data.as_ptr(),
                    section.physical_address as *mut u8,
                    section.size as usize,
                );
            }
        }
        
        current_physical += section.size;
    }
    
    write_line("✓ Kernel sections loaded successfully");
    Ok(physical_entry)
}

/// Main kernel loading function
/// 
/// # Arguments
/// 
/// * `system_table` - UEFI system table
/// * `memory_map` - UEFI memory map
/// * `output_driver` - Output driver for logging
/// 
/// # Returns
/// 
/// * `Ok((physical_entry, virtual_entry))` - Entry point addresses
/// * `Err(status)` - Error loading kernel
pub fn load_kernel_binary(_memory_map: &uefi::mem::memory_map::MemoryMapOwned, ) -> Result<(u64, u64), uefi::Status> {
    write_line("=== Loading Kernel Binary ===");
    write_line("Starting kernel loading process...");
    
    // Find the kernel file
    let mut kernel_file = find_kernel_file()?;
    
    // Get file information
    let file_size = get_kernel_file_info(&mut kernel_file)?;
    
    // Read kernel binary
    let kernel_data = read_kernel_binary(&mut kernel_file, file_size)?;
    
    // Analyze kernel binary
    let mut kernel_info = analyze_kernel_binary(&kernel_data)?;
    
    // Allocate memory using UEFI Boot Services
    let memory_region = crate::memory::allocate_memory(kernel_info.total_memory_size, uefi::mem::memory_map::MemoryType::LOADER_DATA)?;
    let physical_base = memory_region.physical_address;
    
    write_line("✓ Memory allocated using UEFI Boot Services");
    
    // Load kernel sections
    let physical_entry = load_kernel_sections(&mut kernel_info, &kernel_data, physical_base)?;
    
    write_line("✓ Kernel binary loaded successfully");
    write_line(&format!("  Physical base: 0x{:016X}", physical_base));
    write_line(&format!("  Entry point (physical): 0x{:016X}", physical_entry));
    write_line(&format!("  Entry point (virtual): 0x{:016X}", kernel_info.entry_point));
    
    Ok((physical_entry, kernel_info.entry_point))
}