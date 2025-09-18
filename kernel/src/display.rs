//! Display and output module
//! 
//! This module provides functions for displaying kernel information and debug output.

use theseus_shared::handoff::Handoff;

/// Simple kernel output function that writes directly to QEMU debug port
pub fn kernel_write_line(message: &str) { 
    theseus_shared::qemu_println!(message); 
}

/// Display information from the handoff structure
pub fn display_handoff_info(handoff: &Handoff) {
    kernel_write_line("");
    kernel_write_line("┌─────────────────────────────────────────────────────────┐");
    kernel_write_line("│                Kernel Handoff Information               │");
    kernel_write_line("├─────────────────────────────────────────────────────────┤");
    
    kernel_write_line("│ Handoff Size: Available");
    kernel_write_line("│ Memory Map Entries: Available");
    kernel_write_line("│ Memory Map Size: Available");
    kernel_write_line("│ Memory Map Buffer: Available");
    
    if handoff.acpi_rsdp != 0 {
        kernel_write_line("│ ACPI RSDP: Available");
    } else {
        kernel_write_line("│ ACPI RSDP: Not available");
    }
    
    if handoff.gop_fb_base != 0 {
        kernel_write_line("│ Framebuffer: Available");
    } else {
        kernel_write_line("│ Framebuffer: Not available");
    }
    
    kernel_write_line("│ Hardware Devices: Available");
    
    // Virtual memory information
    kernel_write_line("│");
    kernel_write_line("│ Virtual Memory Information:");
    kernel_write_line("│   Virtual Base: Available");
    kernel_write_line("│   Physical Base: Available");
    kernel_write_line("│   Virtual Entry: Available");
    kernel_write_line("│   Page Table Root: Available");
    kernel_write_line("│   Virtual Memory: Available");
    
    kernel_write_line("└─────────────────────────────────────────────────────────┘");
    kernel_write_line("");
}
