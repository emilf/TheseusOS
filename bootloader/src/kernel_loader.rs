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
    /// Minimum virtual address of all LOAD segments (virtual base)
    pub virtual_base: u64,
    /// Total virtual span covered by all LOAD segments (max_end - virtual_base)
    pub total_span_size: u64,
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
    
    // Parse ELF header (64-bit)
    if kernel_data.len() < 64 {
        write_line("✗ ELF file too small to contain header");
        return Err(Status::INVALID_PARAMETER);
    }
    
    // Read entry point from ELF header (offset 24, 8 bytes)
    let entry_point = u64::from_le_bytes([
        kernel_data[24], kernel_data[25], kernel_data[26], kernel_data[27],
        kernel_data[28], kernel_data[29], kernel_data[30], kernel_data[31],
    ]);
    
    // Read program header offset (offset 32, 8 bytes)
    let ph_offset = u64::from_le_bytes([
        kernel_data[32], kernel_data[33], kernel_data[34], kernel_data[35],
        kernel_data[36], kernel_data[37], kernel_data[38], kernel_data[39],
    ]);
    
    // Read number of program headers (offset 56, 2 bytes)
    let ph_num = u16::from_le_bytes([kernel_data[56], kernel_data[57]]) as usize;
    
    write_line(&format!("  ELF Entry Point: 0x{:016X}", entry_point));
    write_line(&format!("  Program Headers: {} entries at offset 0x{:X}", ph_num, ph_offset));
    
    // Parse program headers to find loadable segments
    let mut sections = Vec::new();
    let mut min_virtual_address: u64 = u64::MAX;
    let mut max_virtual_end: u64 = 0u64;
    
    for i in 0..ph_num {
        let ph_start = (ph_offset as usize) + (i * 56); // 56 bytes per program header
        if ph_start + 56 > kernel_data.len() {
            write_line("✗ Program header extends beyond file");
            return Err(Status::INVALID_PARAMETER);
        }
        
        // Read segment type (offset 0, 4 bytes)
        let segment_type = u32::from_le_bytes([
            kernel_data[ph_start], kernel_data[ph_start + 1], 
            kernel_data[ph_start + 2], kernel_data[ph_start + 3],
        ]);
        
        // Only process LOAD segments (type 1)
        if segment_type == 1 {
            // Read segment flags (offset 4, 4 bytes)
            let flags = u32::from_le_bytes([
                kernel_data[ph_start + 4], kernel_data[ph_start + 5], 
                kernel_data[ph_start + 6], kernel_data[ph_start + 7],
            ]);
            
            // Read virtual address (offset 16, 8 bytes)
            let virt_addr = u64::from_le_bytes([
                kernel_data[ph_start + 16], kernel_data[ph_start + 17],
                kernel_data[ph_start + 18], kernel_data[ph_start + 19],
                kernel_data[ph_start + 20], kernel_data[ph_start + 21],
                kernel_data[ph_start + 22], kernel_data[ph_start + 23],
            ]);
            
            // Read file offset (offset 8, 8 bytes)
            let file_offset = u64::from_le_bytes([
                kernel_data[ph_start + 8], kernel_data[ph_start + 9],
                kernel_data[ph_start + 10], kernel_data[ph_start + 11],
                kernel_data[ph_start + 12], kernel_data[ph_start + 13],
                kernel_data[ph_start + 14], kernel_data[ph_start + 15],
            ]);
            
            // Read memory size (offset 40, 8 bytes)
            let mem_size = u64::from_le_bytes([
                kernel_data[ph_start + 40], kernel_data[ph_start + 41],
                kernel_data[ph_start + 42], kernel_data[ph_start + 43],
                kernel_data[ph_start + 44], kernel_data[ph_start + 45],
                kernel_data[ph_start + 46], kernel_data[ph_start + 47],
            ]);
            
            // Determine section name based on virtual address
            let section_name = if virt_addr == 0xffffffff80000000 {
                ".text"
            } else if virt_addr == 0xffffffff80001ad8 {
                ".dynsym"
            } else if virt_addr == 0xffffffff80001e80 {
                ".rodata"
            } else if virt_addr == 0xffffffff80002640 {
                ".dynamic"
            } else if virt_addr == 0xffffffff80002728 {
                ".data"
            } else if virt_addr == 0xffffffff80002a68 {
                ".bss"
            } else {
                "unknown"
            };
            
            sections.push(KernelSection {
                name: section_name,
                virtual_address: virt_addr,
                physical_address: 0, // Will be set during loading
                size: mem_size,
                file_offset: file_offset,
                flags: flags,
            });
            
            // Track overall virtual address span
            if virt_addr < min_virtual_address { min_virtual_address = virt_addr; }
            let end = virt_addr.saturating_add(mem_size);
            if end > max_virtual_end { max_virtual_end = end; }
            
            write_line(&format!("    {}: 0x{:016X} ({} bytes, flags: 0x{:X})",
                section_name, virt_addr, mem_size, flags));
        }
    }
    
    // Compute span size and sanity-check
    if min_virtual_address == u64::MAX { return Err(Status::INVALID_PARAMETER); }
    let total_span_size = max_virtual_end.saturating_sub(min_virtual_address);

    write_line("✓ Kernel analysis complete");
    write_line(&format!("  Entry point: 0x{:016X}", entry_point));
    write_line(&format!("  Virtual base: 0x{:016X}", min_virtual_address));
    write_line(&format!("  Virtual span size: {} bytes ({:.2} MB)", 
        total_span_size, total_span_size as f64 / theseus_shared::constants::memory::BYTES_PER_MB));
    write_line(&format!("  Loadable segments: {}", sections.len()));
    
    Ok(KernelInfo {
        entry_point,
        virtual_base: min_virtual_address,
        total_span_size,
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
    
    // Calculate the offset between virtual and physical addresses using analyzed virtual base
    let virtual_base = kernel_info.virtual_base;
    let virtual_to_physical_offset = physical_base as i64 - virtual_base as i64;
    
    let mut physical_entry = 0;
    
    for section in &mut kernel_info.sections {
        // Calculate physical address by applying the offset
        section.physical_address = (section.virtual_address as i64 + virtual_to_physical_offset) as u64;
        
        // Find the physical entry point (text section with entry point virtual address)
        if section.name == ".text" && section.virtual_address == virtual_base {
            // Calculate physical entry point by adding the offset to the virtual entry point
            physical_entry = (kernel_info.entry_point as i64 + virtual_to_physical_offset) as u64;
        }
        
        write_line(&format!("  {}: 0x{:016X} -> 0x{:016X}",
            section.name, section.virtual_address, section.physical_address));
        
        if section.name == ".bss" {
            // BSS section needs to be zeroed
            write_line(&format!("    BSS section: {} bytes (zeroing)", section.size));
            write_line(&format!("    BSS physical address: 0x{:016X}", section.physical_address));
            
            // Validate physical address
            if section.physical_address == 0 {
                write_line("    ✗ ERROR: Invalid physical address (0x0) for BSS section");
                return Err(Status::INVALID_PARAMETER);
            }
            
            unsafe {
                core::ptr::write_bytes(section.physical_address as *mut u8, 0, section.size as usize);
            }
        } else {
            // Copy section data from file to allocated memory
            write_line(&format!("    {} section: {} bytes (copying)", section.name, section.size));
            write_line(&format!("    Physical address: 0x{:016X}", section.physical_address));
            write_line(&format!("    File offset: 0x{:016X}, size: {}", section.file_offset, section.size));
            
            // Validate physical address
            if section.physical_address == 0 {
                write_line("    ✗ ERROR: Invalid physical address (0x0) for section");
                return Err(Status::INVALID_PARAMETER);
            }
            
            // For sections with file data, copy from file
            if section.file_offset > 0 || section.name != ".bss" {
                // Calculate how much data to copy from file
                let file_size = if section.file_offset + section.size > kernel_data.len() as u64 {
                    kernel_data.len() as u64 - section.file_offset
                } else {
                    section.size
                };
                
                if file_size > 0 {
                    let file_data = &kernel_data[section.file_offset as usize..(section.file_offset + file_size) as usize];
                    write_line(&format!("    File data slice length: {}", file_data.len()));
                    
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            file_data.as_ptr(),
                            section.physical_address as *mut u8,
                            file_size as usize,
                        );
                    }
                }
                
                // If memory size is larger than file size, zero the remainder
                if section.size > file_size {
                    let remaining_size = section.size - file_size;
                    write_line(&format!("    Zeroing remaining {} bytes", remaining_size));
                    unsafe {
                        core::ptr::write_bytes(
                            (section.physical_address + file_size) as *mut u8,
                            0,
                            remaining_size as usize,
                        );
                    }
                }
            }
        }
    }
    
    write_line(&format!("Physical entry point: 0x{:016X}", physical_entry));
    write_line("✓ Kernel sections loaded successfully");
    Ok(physical_entry)
}

/// Apply ELF relocations (SHT_RELA) to the loaded kernel image
///
/// Handles minimal x86_64 relocations required for absolute addresses:
/// - R_X86_64_64: write symbol value + addend (virtual address)
/// - R_X86_64_RELATIVE: write base (virtual) + addend
fn apply_relocations(kernel_info: &KernelInfo, kernel_data: &[u8], physical_base: u64) -> Result<(), Status> {
    write_line("Applying relocations...");

    // ELF64 header fields we need
    if kernel_data.len() < 0x40 + 8 {
        write_line("✗ ELF too small for section headers");
        return Err(Status::INVALID_PARAMETER);
    }

    let e_shoff = u64::from_le_bytes([
        kernel_data[40], kernel_data[41], kernel_data[42], kernel_data[43],
        kernel_data[44], kernel_data[45], kernel_data[46], kernel_data[47],
    ]) as usize;
    let e_shentsize = u16::from_le_bytes([kernel_data[58], kernel_data[59]]) as usize;
    let e_shnum = u16::from_le_bytes([kernel_data[60], kernel_data[61]]) as usize;

    if e_shoff == 0 || e_shnum == 0 || e_shentsize == 0 {
        write_line("  No section headers present (skipping relocations)");
        return Ok(());
    }
    if e_shoff + e_shnum * e_shentsize > kernel_data.len() {
        write_line("✗ Section headers exceed file size");
        return Err(Status::INVALID_PARAMETER);
    }

    // Helper closures to read fields from a section header at index i
    let sh_at = |i: usize| -> usize { e_shoff + i * e_shentsize };
    let read_u32 = |data: &[u8], off: usize| -> u32 {
        u32::from_le_bytes([data[off], data[off+1], data[off+2], data[off+3]])
    };
    let read_u64 = |data: &[u8], off: usize| -> u64 {
        u64::from_le_bytes([
            data[off], data[off+1], data[off+2], data[off+3],
            data[off+4], data[off+5], data[off+6], data[off+7],
        ])
    };

    // Compute V->P offset used to locate relocation targets in memory
    let virt_phys_off = physical_base as i64 - kernel_info.virtual_base as i64;

    let mut rela_sections = 0usize;
    let mut rela_entries = 0usize;

    for i in 0..e_shnum {
        let base = sh_at(i);
        let sh_type = read_u32(kernel_data, base + 4);
        // SHT_RELA == 4
        if sh_type != 4 { continue; }

        let sh_offset = read_u64(kernel_data, base + 0x18) as usize; // sh_offset
        let sh_size   = read_u64(kernel_data, base + 0x20) as usize; // sh_size
        let sh_link   = read_u32(kernel_data, base + 0x28) as usize; // sh_link -> symtab index
        let sh_entsz  = read_u64(kernel_data, base + 0x38) as usize; // entry size (should be 24)

        if sh_offset == 0 || sh_size == 0 || sh_entsz == 0 { continue; }
        if sh_offset + sh_size > kernel_data.len() { write_line("✗ RELA section out of bounds"); return Err(Status::INVALID_PARAMETER); }

        // Locate linked symbol table
        if sh_link >= e_shnum { write_line("✗ RELA link index out of range"); return Err(Status::INVALID_PARAMETER); }
        let symtab_base = sh_at(sh_link);
        let symtab_type = read_u32(kernel_data, symtab_base + 4);
        // SYMTAB=2 or DYNSYM=11
        if symtab_type != 2 && symtab_type != 11 { write_line("✗ Linked symtab has unexpected type"); return Err(Status::INVALID_PARAMETER); }
        let sym_off = read_u64(kernel_data, symtab_base + 0x18) as usize;
        let sym_size = read_u64(kernel_data, symtab_base + 0x20) as usize;
        let sym_entsize = read_u64(kernel_data, symtab_base + 0x38) as usize; // 24 bytes for Elf64_Sym
        if sym_off == 0 || sym_size == 0 || sym_entsize == 0 || sym_off + sym_size > kernel_data.len() {
            write_line("✗ Symbol table bounds invalid");
            return Err(Status::INVALID_PARAMETER);
        }

        // Iterate RELA entries
        let mut applied_here = 0usize;
        let mut off = sh_offset;
        while off + sh_entsz <= sh_offset + sh_size {
            // Elf64_Rela: r_offset (8), r_info (8), r_addend (8)
            let r_offset = read_u64(kernel_data, off) as u64;
            let r_info   = read_u64(kernel_data, off + 8);
            let r_addend = read_u64(kernel_data, off + 16) as i64;

            let r_type = (r_info & 0xFFFFFFFF) as u32;
            let r_sym  = (r_info >> 32) as u32 as usize;

            // Compute target physical address to patch
            let target_phys = (r_offset as i64 + virt_phys_off) as u64;
            if target_phys == 0 { write_line("✗ Relocation target phys is 0"); return Err(Status::INVALID_PARAMETER); }

            // Calculate value to write
            let value: u64 = match r_type {
                1 => { // R_X86_64_64: S + A
                    if r_sym == 0 || sym_entsize == 0 { write_line("✗ R_X86_64_64 with no symbol"); return Err(Status::INVALID_PARAMETER); }
                    let sym_entry_off = sym_off + r_sym * sym_entsize;
                    if sym_entry_off + sym_entsize > sym_off + sym_size { write_line("✗ Symbol index out of range"); return Err(Status::INVALID_PARAMETER); }
                    // Elf64_Sym: st_name(4) st_info(1) st_other(1) st_shndx(2) st_value(8) st_size(8)
                    let st_value = read_u64(kernel_data, sym_entry_off + 8); // st_value at +8
                    (st_value as i64 + r_addend) as u64
                }
                7 => { // R_X86_64_RELATIVE: B + A (B = image virtual base)
                    (kernel_info.virtual_base as i64 + r_addend) as u64
                }
                _ => {
                    // Unsupported relocation type; skip gracefully
                    // Common types we expect are 1 and 7 in this kernel
                    off += sh_entsz;
                    continue;
                }
            };

            // Apply the relocation write
            unsafe { (target_phys as *mut u64).write_volatile(value); }
            applied_here += 1;
            off += sh_entsz;
        }

        rela_sections += 1;
        rela_entries += applied_here;
    }

    // Also process PT_DYNAMIC-based RELA/REL (more reliable than sections in stripped binaries)
    let mut dyn_rela_entries = 0usize;
    let mut dyn_rel_entries = 0usize;
    {
        // ELF64 header program header info
        if kernel_data.len() >= 0x40 + 8 {
            let e_phoff = u64::from_le_bytes([
                kernel_data[32], kernel_data[33], kernel_data[34], kernel_data[35],
                kernel_data[36], kernel_data[37], kernel_data[38], kernel_data[39],
            ]) as usize;
            let e_phentsize = u16::from_le_bytes([kernel_data[54], kernel_data[55]]) as usize;
            let e_phnum = u16::from_le_bytes([kernel_data[56], kernel_data[57]]) as usize;
            if e_phoff != 0 && e_phentsize >= 56 && e_phoff + e_phnum * e_phentsize <= kernel_data.len() {
                // Find PT_DYNAMIC (type=2)
                let mut dyn_off = 0usize;
                let mut _dyn_vaddr = 0u64;
                let mut dyn_size = 0u64;
                for i in 0..e_phnum {
                    let off = e_phoff + i * e_phentsize;
                    let p_type = u32::from_le_bytes([
                        kernel_data[off], kernel_data[off+1], kernel_data[off+2], kernel_data[off+3]
                    ]);
                    if p_type == 2 { // PT_DYNAMIC
                        dyn_off = u64::from_le_bytes([
                            kernel_data[off+8], kernel_data[off+9], kernel_data[off+10], kernel_data[off+11],
                            kernel_data[off+12], kernel_data[off+13], kernel_data[off+14], kernel_data[off+15],
                        ]) as usize; // p_offset
                        _dyn_vaddr = u64::from_le_bytes([
                            kernel_data[off+16], kernel_data[off+17], kernel_data[off+18], kernel_data[off+19],
                            kernel_data[off+20], kernel_data[off+21], kernel_data[off+22], kernel_data[off+23],
                        ]);
                        dyn_size = u64::from_le_bytes([
                            kernel_data[off+32], kernel_data[off+33], kernel_data[off+34], kernel_data[off+35],
                            kernel_data[off+36], kernel_data[off+37], kernel_data[off+38], kernel_data[off+39],
                        ]);
                        break;
                    }
                }
                if dyn_off != 0 && dyn_size >= 16 && dyn_off + dyn_size as usize <= kernel_data.len() {
                    // Parse dynamic tags
                    let mut dt_rela: u64 = 0;
                    let mut dt_relasz: u64 = 0;
                    let mut dt_relaent: u64 = 24;
                    let mut dt_rel: u64 = 0;
                    let mut dt_relsz: u64 = 0;
                    let mut dt_relent: u64 = 16;
                    let mut dt_symtab: u64 = 0;
                    let mut dt_syment: u64 = 24;
                    let mut cursor = dyn_off;
                    while cursor + 16 <= dyn_off + dyn_size as usize {
                        let d_tag = i64::from_le_bytes([
                            kernel_data[cursor], kernel_data[cursor+1], kernel_data[cursor+2], kernel_data[cursor+3],
                            kernel_data[cursor+4], kernel_data[cursor+5], kernel_data[cursor+6], kernel_data[cursor+7],
                        ]);
                        let d_val = u64::from_le_bytes([
                            kernel_data[cursor+8], kernel_data[cursor+9], kernel_data[cursor+10], kernel_data[cursor+11],
                            kernel_data[cursor+12], kernel_data[cursor+13], kernel_data[cursor+14], kernel_data[cursor+15],
                        ]);
                        if d_tag == 0 { break; } // DT_NULL
                        match d_tag {
                            7 => dt_rela = d_val,          // DT_RELA
                            8 => dt_relasz = d_val,        // DT_RELASZ
                            9 => dt_relaent = d_val,       // DT_RELAENT
                            17 => dt_rel = d_val,          // DT_REL
                            18 => dt_relsz = d_val,        // DT_RELSZ
                            19 => dt_relent = d_val,       // DT_RELENT
                            6 => dt_symtab = d_val,        // DT_SYMTAB
                            11 => dt_syment = d_val,       // DT_SYMENT
                            _ => {}
                        }
                        cursor += 16;
                    }

                    // Helper to convert image virtual -> physical
                    let v2p = |va: u64| -> u64 { (va as i64 + (physical_base as i64 - kernel_info.virtual_base as i64)) as u64 };

                    // Access .dynsym table if present
                    let (symtab_ptr, _symtab_size, symtab_entsz) = if dt_symtab != 0 && dt_syment != 0 {
                        (v2p(dt_symtab), 0usize, dt_syment as usize)
                    } else { (0u64, 0usize, 0usize) };

                    // Process RELA
                    if dt_rela != 0 && dt_relasz != 0 {
                        let rela_ptr = v2p(dt_rela);
                        let rela_cnt = (dt_relasz / dt_relaent) as usize;
                        let mut rptr = rela_ptr as usize;
                        for _ in 0..rela_cnt {
                            // read from memory we just loaded (physical)
                            let r_offset = unsafe { core::ptr::read_unaligned(rptr as *const u64) };
                            let r_info   = unsafe { core::ptr::read_unaligned((rptr + 8) as *const u64) };
                            let r_addend = unsafe { core::ptr::read_unaligned((rptr + 16) as *const i64) };
                            let r_type = (r_info & 0xFFFFFFFF) as u32;
                            let r_sym  = (r_info >> 32) as u32 as usize;
                            let target_phys = v2p(r_offset);
                            let value: Option<u64> = match r_type {
                                1 => { // R_X86_64_64
                                    if symtab_ptr != 0 && symtab_entsz >= 24 && r_sym != 0 {
                                        let sym_off = symtab_ptr as usize + r_sym * symtab_entsz;
                                        // st_value at +8
                                        let st_value = unsafe { core::ptr::read_unaligned((sym_off + 8) as *const u64) };
                                        Some((st_value as i64 + r_addend) as u64)
                                    } else { None }
                                }
                                7 => { // R_X86_64_RELATIVE
                                    Some((kernel_info.virtual_base as i64 + r_addend) as u64)
                                }
                                _ => None,
                            };
                            if let Some(val) = value {
                                unsafe { (target_phys as *mut u64).write_volatile(val); }
                                dyn_rela_entries += 1;
                            }
                            rptr += dt_relaent as usize;
                        }
                    }

                    // Process REL (rare on x86_64, but handle if present)
                    if dt_rel != 0 && dt_relsz != 0 {
                        let rel_ptr = v2p(dt_rel);
                        let rel_cnt = (dt_relsz / dt_relent) as usize;
                        let mut rptr = rel_ptr as usize;
                        for _ in 0..rel_cnt {
                            // Elf64_Rel: r_offset(8), r_info(8)
                            let r_offset = unsafe { core::ptr::read_unaligned(rptr as *const u64) };
                            let r_info   = unsafe { core::ptr::read_unaligned((rptr + 8) as *const u64) };
                            let r_type = (r_info & 0xFFFFFFFF) as u32;
                            let r_sym  = (r_info >> 32) as u32 as usize;
                            let target_phys = v2p(r_offset);
                            // No addend field; assume 0
                            let value: Option<u64> = match r_type {
                                1 => { // R_X86_64_64
                                    if symtab_ptr != 0 && symtab_entsz >= 24 && r_sym != 0 {
                                        let sym_off = symtab_ptr as usize + r_sym * symtab_entsz;
                                        let st_value = unsafe { core::ptr::read_unaligned((sym_off + 8) as *const u64) };
                                        Some(st_value)
                                    } else { None }
                                }
                                7 => { // R_X86_64_RELATIVE
                                    Some(kernel_info.virtual_base)
                                }
                                _ => None,
                            };
                            if let Some(val) = value {
                                unsafe { (target_phys as *mut u64).write_volatile(val); }
                                dyn_rel_entries += 1;
                            }
                            rptr += dt_relent as usize;
                        }
                    }
                }
            }
        }
    }

    write_line(&format!("✓ Relocations applied: sections={}, entries={}, dyn_rela={}, dyn_rel={}", rela_sections, rela_entries, dyn_rela_entries, dyn_rel_entries));
    Ok(())
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
/// * `Ok((physical_base, physical_entry, virtual_entry, image_size))` - key addresses and image span
/// * `Err(status)` - Error loading kernel
pub fn load_kernel_binary(_memory_map: &uefi::mem::memory_map::MemoryMapOwned, ) -> Result<(u64, u64, u64, u64), uefi::Status> {
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
    let alloc_size = (kernel_info.total_span_size + 0xFFF) & !0xFFF;
    let memory_region = crate::memory::allocate_memory(alloc_size, uefi::mem::memory_map::MemoryType::LOADER_DATA)?;
    let physical_base = memory_region.physical_address;
    
    write_line("✓ Memory allocated using UEFI Boot Services");
    
    // Load kernel sections
    let physical_entry = load_kernel_sections(&mut kernel_info, &kernel_data, physical_base)?;

    // Apply relocations now that image is loaded at physical_base
    apply_relocations(&kernel_info, &kernel_data, physical_base)?;
    
    write_line("✓ Kernel binary loaded successfully");
    write_line(&format!("  Physical base: 0x{:016X}", physical_base));
    write_line(&format!("  Entry point (physical): 0x{:016X}", physical_entry));
    write_line(&format!("  Entry point (virtual): 0x{:016X}", kernel_info.entry_point));
    
    Ok((physical_base, physical_entry, kernel_info.entry_point, kernel_info.total_span_size))
}