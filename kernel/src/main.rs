#![no_std]
#![no_main]

extern crate alloc;

use hobbyos_shared::handoff::HANDOFF;
use hobbyos_shared::constants::{io_ports, exit_codes};

/// Simple global allocator for the kernel
/// This is a placeholder - in a real kernel you'd implement proper memory management
#[global_allocator]
static ALLOCATOR: DummyAllocator = DummyAllocator;

struct DummyAllocator;

unsafe impl core::alloc::GlobalAlloc for DummyAllocator {
    unsafe fn alloc(&self, _layout: core::alloc::Layout) -> *mut u8 {
        core::ptr::null_mut()
    }

    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: core::alloc::Layout) {
        // No-op
    }
}

/// Simple kernel output function that writes directly to QEMU debug port
fn kernel_write_line(message: &str) {
    // Write message directly to QEMU debug port (0xe9)
    for byte in message.bytes() {
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") io_ports::QEMU_DEBUG,
                in("al") byte,
                options(nomem, nostack, preserves_flags)
            );
        }
    }
    
    // Write newline
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") io_ports::QEMU_DEBUG,
            in("al") b'\n',
            options(nomem, nostack, preserves_flags)
        );
    }
}

/// Kernel entry point
/// 
/// This is where the kernel starts after the bootloader has:
/// 1. Collected all system information
/// 2. Exited boot services
/// 3. Jumped to this location
/// 
/// The handoff structure should be available at a known location in memory.
#[no_mangle]
pub extern "C" fn kernel_main() -> ! {
    // Initialize kernel logging (placeholder)
    // TODO: Set up kernel logging system
    
    kernel_write_line("=== HobbyOS Kernel Starting ===");
    kernel_write_line("Kernel entry point reached successfully");
    
    // Access the handoff structure
    unsafe {
        if HANDOFF.size > 0 {
            kernel_write_line(&alloc::format!("Handoff structure found (size: {} bytes)", HANDOFF.size));
            
            // Display system information from handoff
            display_handoff_info();
            
            // Initialize kernel subsystems
            initialize_kernel_subsystems();
            
        } else {
            kernel_write_line("ERROR: Handoff structure not found or invalid");
        }
    }
    
    // For now, just exit QEMU
    kernel_write_line("Kernel initialization complete");
    kernel_write_line("Exiting QEMU...");
    
    unsafe {
        core::arch::asm!("out dx, al", 
            in("dx") io_ports::QEMU_EXIT, 
            in("al") exit_codes::QEMU_SUCCESS, 
            options(nomem, nostack, preserves_flags)
        );
    }
    
    loop {}
}

/// Display information from the handoff structure
fn display_handoff_info() {
    unsafe {
        kernel_write_line("");
        kernel_write_line("┌─────────────────────────────────────────────────────────┐");
        kernel_write_line("│                Kernel Handoff Information               │");
        kernel_write_line("├─────────────────────────────────────────────────────────┤");
        
        kernel_write_line(&alloc::format!("│ Handoff Size: {} bytes", HANDOFF.size));
        kernel_write_line(&alloc::format!("│ Memory Map Entries: {}", HANDOFF.memory_map_entries));
        kernel_write_line(&alloc::format!("│ Memory Map Size: {} bytes", HANDOFF.memory_map_size));
        kernel_write_line(&alloc::format!("│ Memory Map Buffer: 0x{:016X}", HANDOFF.memory_map_buffer_ptr));
        
        if HANDOFF.acpi_rsdp != 0 {
            kernel_write_line(&alloc::format!("│ ACPI RSDP: 0x{:016X}", HANDOFF.acpi_rsdp));
        } else {
            kernel_write_line("│ ACPI RSDP: Not available");
        }
        
        if HANDOFF.gop_fb_base != 0 {
            kernel_write_line(&alloc::format!("│ Framebuffer: 0x{:016X} ({}x{})", 
                HANDOFF.gop_fb_base, HANDOFF.gop_width, HANDOFF.gop_height));
        } else {
            kernel_write_line("│ Framebuffer: Not available");
        }
        
        kernel_write_line(&alloc::format!("│ Hardware Devices: {}", HANDOFF.hardware_device_count));
        
        kernel_write_line("└─────────────────────────────────────────────────────────┘");
        kernel_write_line("");
    }
}

/// Initialize kernel subsystems
fn initialize_kernel_subsystems() {
    kernel_write_line("Initializing kernel subsystems...");
    
    // TODO: Initialize memory management
    kernel_write_line("  - Memory management: TODO");
    
    // TODO: Initialize ACPI subsystem
    kernel_write_line("  - ACPI subsystem: TODO");
    
    // TODO: Initialize device drivers
    kernel_write_line("  - Device drivers: TODO");
    
    // TODO: Initialize process management
    kernel_write_line("  - Process management: TODO");
    
    kernel_write_line("Kernel subsystem initialization complete");
}

/// Panic handler for kernel
#[panic_handler]
fn panic_handler(panic_info: &core::panic::PanicInfo) -> ! {
    // Try to output panic information
    let message = alloc::format!("KERNEL PANIC: {:?}", panic_info.message());
    
    // Write directly to QEMU debug port
    for byte in message.bytes() {
        unsafe {
            core::arch::asm!(
                "out dx, al",
                in("dx") io_ports::QEMU_DEBUG,
                in("al") byte,
                options(nomem, nostack, preserves_flags)
            );
        }
    }
    
    // Write newline
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") io_ports::QEMU_DEBUG,
            in("al") b'\n',
            options(nomem, nostack, preserves_flags)
        );
    }
    
    // Exit QEMU with error
    unsafe {
        core::arch::asm!(
            "out dx, al",
            in("dx") io_ports::QEMU_EXIT,
            in("al") exit_codes::QEMU_ERROR,
            options(nomem, nostack, preserves_flags)
        );
    }
    
    loop {}
}
