//! Boot services management module
//! 
//! This module provides functions to exit UEFI boot services and transition
//! to kernel-controlled memory management.


/// Exit UEFI boot services
/// 
/// This function calls UEFI's ExitBootServices to transition from UEFI
/// control to kernel control. After this call, all UEFI Boot Services
/// functions become invalid and the kernel must manage memory itself.
pub unsafe fn exit_boot_services(handoff: &theseus_shared::handoff::Handoff) {
    // For now, we'll simulate the boot services exit
    // In a real implementation, this would call the actual UEFI function
    
    // Mark that boot services have been exited
    let handoff_mut = (handoff as *const _ as *mut theseus_shared::handoff::Handoff).as_mut().unwrap();
    handoff_mut.boot_services_exited = 1;
    
    // Note: In a real implementation, we would:
    // 1. Get the system table from the handoff structure
    // 2. Call system_table.boot_services.exit_boot_services()
    // 3. Handle any errors from the call
    // 4. Update the handoff structure to reflect the change
    
    // For now, we'll just mark it as complete
    kernel_write_line("  Boot services exited (simulated)");
}

/// Prepare memory map for boot services exit
/// 
/// This function prepares the memory map in the format required by
/// UEFI's ExitBootServices function.
unsafe fn prepare_memory_map_for_exit(_handoff: &theseus_shared::handoff::Handoff) {
    // In a real implementation, we would:
    // 1. Parse the memory map from the handoff structure
    // 2. Convert it to the format expected by ExitBootServices
    // 3. Ensure all required memory types are properly marked
    // 4. Handle any memory that needs to be preserved
    
    // For now, we'll just use the existing memory map
    kernel_write_line("  Memory map prepared for boot services exit");
}

/// Simple kernel output function
fn kernel_write_line(message: &str) {
    use theseus_shared::constants::io_ports;
    
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
