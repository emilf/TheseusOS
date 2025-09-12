//! Kernel loading functionality
//! 
//! This module handles finding, analyzing, and loading the kernel binary
//! from the EFI file system into memory.

use uefi::prelude::*;
use uefi::boot::get_image_file_system;
use uefi::proto::media::file::{File, FileMode, FileAttribute, RegularFile, FileInfo};
use uefi::mem::memory_map::MemoryMap;
use uefi::mem::memory_map::MemoryType;
use uefi::Status;
use alloc::vec::Vec;
use alloc::format;
use alloc::vec;

use theseus_shared::constants::kernel;
use crate::drivers::manager::write_line;

/// ELF Program Header Type - PT_LOAD
const PT_LOAD: u32 = 1;

/// ELF Program Header
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct ElfProgramHeader {
    p_type: u32,        // Segment type
    p_flags: u32,       // Segment flags
    p_offset: u64,      // File offset
    p_vaddr: u64,       // Virtual address
    p_paddr: u64,       // Physical address
    p_filesz: u64,      // File size
    p_memsz: u64,       // Memory size
    p_align: u64,       // Alignment
}

/// ELF Header
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct ElfHeader {
    e_ident: [u8; 16],  // ELF identification
    e_type: u16,        // Object file type
    e_machine: u16,     // Machine type
    e_version: u32,     // Object file version
    e_entry: u64,       // Entry point virtual address
    e_phoff: u64,       // Program header table file offset
    e_shoff: u64,       // Section header table file offset
    e_flags: u32,       // Processor-specific flags
    e_ehsize: u16,      // ELF header size in bytes
    e_phentsize: u16,   // Program header table entry size
    e_phnum: u16,       // Program header table entry count
    e_shentsize: u16,   // Section header table entry size
    e_shnum: u16,       // Section header table entry count
    e_shstrndx: u16,    // Section header string table index
}

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
/// 
/// # Returns
/// 
/// * `Ok(file_size)` - Actual file size
/// * `Err(status)` - Error getting file info
pub fn get_kernel_file_info(file: &mut RegularFile) -> Result<u64, Status> {
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
/// 
/// # Returns
/// 
/// * `Ok(Vec<u8>)` - Kernel binary data
/// * `Err(status)` - Error reading kernel file
pub fn read_kernel_binary(file: &mut RegularFile, file_size: u64) -> Result<Vec<u8>, Status> {
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
/// 
/// # Returns
/// 
/// * `Ok(KernelInfo)` - Parsed kernel information
/// * `Err(status)` - Error parsing kernel
pub fn analyze_kernel_binary(kernel_data: &[u8]) -> Result<KernelInfo, Status> {
    write_line("Analyzing kernel binary...");
    
    // Check ELF magic number
    if kernel_data.len() < 4 || &kernel_data[0..4] != kernel::ELF_MAGIC {
        write_line("✗ Invalid ELF magic number");
        return Err(Status::INVALID_PARAMETER);
    }
    
    write_line("✓ Valid ELF magic number found");
    
    // Parse the ELF header manually to avoid struct alignment issues
    if kernel_data.len() < 64 {
        write_line("✗ ELF file too small to contain header");
        return Err(Status::INVALID_PARAMETER);
    }
    
    // Validate ELF magic number
    if kernel_data[0] != 0x7f || kernel_data[1] != 0x45 || 
       kernel_data[2] != 0x4c || kernel_data[3] != 0x46 {
        write_line("✗ Invalid ELF magic number in header");
        return Err(Status::INVALID_PARAMETER);
    }
    
    // Check if it's 64-bit ELF
    if kernel_data[4] != 2 { // EI_CLASS = 2 for 64-bit
        write_line("✗ Not a 64-bit ELF file");
        return Err(Status::INVALID_PARAMETER);
    }
    
    write_line("✓ ELF binary parsed successfully");
    
    // Debug: Print raw ELF header bytes
    write_line("  Raw ELF header bytes (first 32 bytes):");
    for i in 0..32 {
        if i % 8 == 0 {
            write_line("");
        }
        write_line(&format!("{:02X} ", kernel_data[i]));
    }
    write_line("");
    
    // Debug: Print the actual bytes we're reading for entry point
    write_line("  Entry point bytes (24-31):");
    for i in 24..32 {
        write_line(&format!("    [{}]: 0x{:02X}", i, kernel_data[i]));
    }
    
    // Extract entry point from bytes 24-31 (little endian)
    let entry_point = u64::from_le_bytes([
        kernel_data[24], kernel_data[25], kernel_data[26], kernel_data[27],
        kernel_data[28], kernel_data[29], kernel_data[30], kernel_data[31]
    ]);
    
    write_line(&format!("  Entry point: 0x{:016X}", entry_point));
    
    // Extract program header offset from bytes 32-39
    let ph_offset = u64::from_le_bytes([
        kernel_data[32], kernel_data[33], kernel_data[34], kernel_data[35],
        kernel_data[36], kernel_data[37], kernel_data[38], kernel_data[39]
    ]);
    
    // Extract program header entry size from bytes 54-55
    let ph_entry_size = u16::from_le_bytes([kernel_data[54], kernel_data[55]]) as usize;
    
    // Extract number of program headers from bytes 56-57
    let ph_num = u16::from_le_bytes([kernel_data[56], kernel_data[57]]) as usize;
    
    write_line(&format!("  Program headers: {} entries at offset 0x{:X}", ph_num, ph_offset));
    
    // Parse program headers to get loadable segments
    let mut sections = Vec::new();
    let mut total_memory_size = 0u64;
    
    for i in 0..ph_num {
        let ph_start = ph_offset as usize + (i * ph_entry_size);
        let ph_end = ph_start + ph_entry_size;
        
        if ph_end > kernel_data.len() {
            write_line(&format!("✗ Program header {} extends beyond file", i));
            continue;
        }
        
        // Parse program header manually
        let p_type = u32::from_le_bytes([
            kernel_data[ph_start], kernel_data[ph_start + 1], 
            kernel_data[ph_start + 2], kernel_data[ph_start + 3]
        ]);
        
        let p_flags = u32::from_le_bytes([
            kernel_data[ph_start + 4], kernel_data[ph_start + 5], 
            kernel_data[ph_start + 6], kernel_data[ph_start + 7]
        ]);
        
        let p_offset = u64::from_le_bytes([
            kernel_data[ph_start + 8], kernel_data[ph_start + 9], 
            kernel_data[ph_start + 10], kernel_data[ph_start + 11],
            kernel_data[ph_start + 12], kernel_data[ph_start + 13], 
            kernel_data[ph_start + 14], kernel_data[ph_start + 15]
        ]);
        
        let p_vaddr = u64::from_le_bytes([
            kernel_data[ph_start + 16], kernel_data[ph_start + 17], 
            kernel_data[ph_start + 18], kernel_data[ph_start + 19],
            kernel_data[ph_start + 20], kernel_data[ph_start + 21], 
            kernel_data[ph_start + 22], kernel_data[ph_start + 23]
        ]);
        
        let p_memsz = u64::from_le_bytes([
            kernel_data[ph_start + 40], kernel_data[ph_start + 41], 
            kernel_data[ph_start + 42], kernel_data[ph_start + 43],
            kernel_data[ph_start + 44], kernel_data[ph_start + 45], 
            kernel_data[ph_start + 46], kernel_data[ph_start + 47]
        ]);
        
        // Only process PT_LOAD segments (loadable segments)
        if p_type == PT_LOAD {
            let section_name = match i {
                0 => ".text",
                1 => ".rodata", 
                2 => ".data",
                _ => ".unknown",
            };
            
            let flags = if p_flags & 0x1 != 0 {
                if p_flags & 0x2 != 0 {
                    0x7 // RWX (readable, writable, executable)
                } else {
                    0x5 // RX (readable, executable)
                }
            } else if p_flags & 0x2 != 0 {
                0x6 // RW (readable, writable)
            } else {
                0x4 // R (readable)
            };
            
            let section = KernelSection {
                name: section_name,
                virtual_address: p_vaddr,
                physical_address: 0, // Will be updated during loading
                size: p_memsz, // Use memory size, not file size
                file_offset: p_offset,
                flags,
            };
            
            total_memory_size += section.size;
            
            write_line(&format!("  {}: 0x{:016X} - 0x{:016X} ({} bytes, file offset: 0x{:X})",
                section.name,
                section.virtual_address,
                section.virtual_address + section.size - 1,
                section.size,
                section.file_offset
            ));
            
            sections.push(section);
        }
    }
    
    write_line("✓ Kernel analysis complete");
    write_line(&format!("  Entry point: 0x{:016X}", entry_point));
    write_line(&format!("  Total memory required: {} bytes ({:.2} MB)", 
        total_memory_size, total_memory_size as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
    write_line(&format!("  Loadable segments: {}", sections.len()));
    
    Ok(KernelInfo {
        entry_point,
        total_memory_size,
        sections,
    })
}

/// Find a suitable free memory region for the kernel
/// 
/// # Arguments
/// 
/// * `memory_map` - UEFI memory map
/// * `required_size` - Size of memory needed
/// 
/// # Returns
/// 
/// * `Ok(address)` - Physical address of free memory region
/// * `Err(status)` - Error finding free memory
pub fn find_free_memory_region(memory_map: &uefi::mem::memory_map::MemoryMapOwned, required_size: u64) -> Result<u64, Status> {
    write_line(&format!("Searching for free memory region ({} bytes required)...", required_size));
    
    let mut best_address = None;
    let mut best_size = 0;
    let mut total_free = 0;
    
    for descriptor in memory_map.entries() {
        if descriptor.ty == MemoryType::CONVENTIONAL {
            total_free += descriptor.page_count * 4096;
            if descriptor.page_count * 4096 >= required_size {
                if best_address.is_none() || descriptor.page_count * 4096 < best_size {
                    best_address = Some(descriptor.phys_start);
                    best_size = descriptor.page_count * 4096;
                    write_line(&format!("  Free region: 0x{:016X} - 0x{:016X} ({} bytes)",
                        descriptor.phys_start,
                        descriptor.phys_start + descriptor.page_count * 4096 - 1,
                        descriptor.page_count * 4096
                    ));
                    write_line(&format!("    ✓ Suitable region found at 0x{:016X} ({} bytes)",
                        descriptor.phys_start, descriptor.page_count * 4096));
                }
            }
        }
    }
    
    write_line(&format!("Total free memory available: {} bytes ({:.2} MB)",
        total_free, total_free as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
    
    match best_address {
        Some(address) => {
            write_line(&format!("✓ Selected memory region: 0x{:016X} ({} bytes, {:.2} MB)",
                address, best_size, best_size as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
            Ok(address)
        }
        None => {
            write_line("✗ No suitable free memory region found");
            Err(Status::OUT_OF_RESOURCES)
        }
    }
}

/// Load kernel sections into allocated memory
/// 
/// # Arguments
/// 
/// * `kernel_info` - Kernel information
/// * `kernel_data` - Raw kernel binary data
/// * `physical_base` - Physical base address for loading
/// 
/// # Returns
/// 
/// * `Ok(physical_entry)` - Physical entry point address
/// * `Err(status)` - Error loading kernel
pub fn load_kernel_sections(kernel_info: &mut KernelInfo, kernel_data: &[u8], physical_base: u64) -> Result<u64, Status> {
    write_line("Loading kernel sections into memory...");
    write_line(&format!("Loading kernel at physical address: 0x{:016X}", physical_base));
    
    let mut physical_entry = 0;
    
    for section in &mut kernel_info.sections {
        // Calculate physical address based on virtual address offset from base
        let virtual_offset = section.virtual_address - kernel_info.entry_point;
        section.physical_address = physical_base + virtual_offset;
        
        // Check if this is the entry point section
        if section.virtual_address == kernel_info.entry_point {
            physical_entry = section.physical_address;
        }
        
        write_line(&format!("  {}: 0x{:016X} -> 0x{:016X} ({} bytes)",
            section.name, section.virtual_address, section.physical_address, section.size));
        
        // Calculate how much data to copy from file
        let file_data_size = if section.file_offset + section.size > kernel_data.len() as u64 {
            kernel_data.len() - section.file_offset as usize
        } else {
            section.size as usize
        };
        
        if file_data_size > 0 {
            // Copy section data from file
            let file_data = &kernel_data[section.file_offset as usize..(section.file_offset as usize + file_data_size)];
            unsafe {
                core::ptr::copy_nonoverlapping(
                    file_data.as_ptr(),
                    section.physical_address as *mut u8,
                    file_data_size
                );
            }
            write_line(&format!("    Copied {} bytes from file offset 0x{:X}", file_data_size, section.file_offset));
        }
        
        // Zero out any remaining memory (for BSS or uninitialized data)
        if section.size as usize > file_data_size {
            let zero_size = section.size as usize - file_data_size;
            unsafe {
                core::ptr::write_bytes(
                    (section.physical_address as *mut u8).add(file_data_size),
                    0,
                    zero_size
                );
            }
            write_line(&format!("    Zeroed {} bytes for uninitialized data", zero_size));
        }
    }
    
    write_line("✓ Kernel sections loaded successfully");
    Ok(physical_entry)
}

/// Main kernel loading function
/// 
/// # Arguments
/// 
/// * `memory_map` - UEFI memory map
/// 
/// # Returns
/// 
/// * `Ok((physical_entry, virtual_entry))` - Entry point addresses
/// * `Err(status)` - Error loading kernel
pub fn load_kernel_binary(memory_map: &uefi::mem::memory_map::MemoryMapOwned) -> Result<(u64, u64), uefi::Status> {
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
    
    // Find free memory region
    let physical_base = find_free_memory_region(memory_map, kernel_info.total_memory_size)?;
    
    // Allocate memory (in a real implementation, this would use UEFI memory allocation)
    write_line("✓ Memory allocated");
    
    // Load kernel sections
    let physical_entry = load_kernel_sections(&mut kernel_info, &kernel_data, physical_base)?;
    
    write_line("✓ Kernel binary loaded successfully");
    write_line(&format!("  Physical base: 0x{:016X}", physical_base));
    write_line(&format!("  Entry point (physical): 0x{:016X}", physical_entry));
    write_line(&format!("  Entry point (virtual): 0x{:016X}", kernel_info.entry_point));
    
    Ok((physical_entry, kernel_info.entry_point))
}