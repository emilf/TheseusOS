#![no_std]
#![no_main]

//extern crate alloc;

use log::info;
use uefi::prelude::*;
//use uefi::proto::console::serial::Serial;
//use uefi::proto::console::gop::GraphicsOutput;
//use uefi::boot::{MemoryType, SearchType};
//use uefi::Identify;
//use alloc::string::ToString;
use core::arch::asm;
//use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;

/* // Global allocator for UEFI
#[global_allocator]
static ALLOCATOR: uefi::alloc::UefiAllocator = uefi::alloc::UefiAllocator;

// Panic handler
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        unsafe { asm!("hlt", options(nomem, nostack, preserves_flags)) }
    }
} */

/// Handoff structure passed to the kernel.
/// Layout is stable (repr C) to allow consumption from any language.
/* #[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct Handoff {
    /// Total size of this struct in bytes
    size: u32,
    /// Handoff format version
    handoff_version: u32,
    /// UEFI firmware revision
    firmware_revision: u32,
    /// Framebuffer base address
    gop_fb_base: u64,
    /// Framebuffer size in bytes
    gop_fb_size: u64,
    /// Framebuffer width in pixels
    gop_width: u32,
    /// Framebuffer height in pixels
    gop_height: u32,
    /// Framebuffer stride
    gop_stride: u32,
    /// Pixel format enum
    gop_pixel_format: u32,
}

/// Static storage for handoff data
static mut HANDOFF: Handoff = Handoff {
    size: 0,
    handoff_version: 1,
    firmware_revision: 0,
    gop_fb_base: 0,
    gop_fb_size: 0,
    gop_width: 0,
    gop_height: 0,
    gop_stride: 0,
    gop_pixel_format: 0,
}; */

// /// Helper function to write to serial
// fn serial_write(serial_handle: Option<Handle>, data: &[u8]) {
//     if let Some(handle) = serial_handle {
//         if let Ok(mut serial) = uefi::boot::open_protocol_exclusive::<Serial>(handle) {
//             let _ = serial.write(data);
//         }
//     }
// }

// /// Helper function to write a line to serial
// fn serial_write_line(serial_handle: Option<Handle>, line: &str) {
//     serial_write(serial_handle, line.as_bytes());
//     serial_write(serial_handle, b"\r\n");
// }

/// Main UEFI entry point
#[entry]
fn efi_main() -> Status {
    // Step 1: Initialize UEFI logger
    uefi::helpers::init().unwrap();
    info!("UEFI Bootloader Starting...");

    // // Step 2: Get system table
    // let system_table = uefi::table::system_table_raw()
    //     .expect("System table not available");
    // let system_table = unsafe { system_table.as_ref() };

    // // Step 3: Get firmware information
    // let vendor_str = if !system_table.firmware_vendor.is_null() {
    //     unsafe { (*system_table.firmware_vendor).to_string() }
    // } else {
    //     "Unknown".to_string()
    // };
    // let rev = system_table.firmware_revision;
    
    // unsafe { HANDOFF.firmware_revision = rev; }

    // // Step 4: Count handles
    // let handle_count = match uefi::boot::locate_handle_buffer(SearchType::AllHandles) {
    //     Ok(buf) => buf.len(),
    //     Err(_) => 0,
    // };

    // // Step 5: Initialize serial
    // let serial_handle = uefi::boot::locate_handle_buffer(
    //     SearchType::ByProtocol(&Serial::GUID)
    // )
    // .ok()
    // .and_then(|buf| buf.first().copied());

    // // Step 6: Output basic information
    // serial_write_line(serial_handle, "=== HobbyOS UEFI Loader ===");
    // serial_write_line(serial_handle, &alloc::format!("Firmware: {} rev {}", vendor_str, rev));
    // serial_write_line(serial_handle, &alloc::format!("Handles: {}", handle_count));

    // // Step 7: Set handoff size
    // unsafe { HANDOFF.size = core::mem::size_of::<Handoff>() as u32; }

    // // Step 8: Query GOP
    // let gop_info = match uefi::boot::locate_handle_buffer(
    //     SearchType::ByProtocol(&GraphicsOutput::GUID)
    // ) {
    //     Ok(gop_handles) => {
    //         if let Some(&handle) = gop_handles.first() {
    //             if let Ok(mut gop) = uefi::boot::open_protocol_exclusive::<GraphicsOutput>(handle) {
    //                 let mode = gop.current_mode_info();
    //                 let res = mode.resolution();
    //                 let pf = mode.pixel_format();
    //                 let mut fb = gop.frame_buffer();
                    
    //                 unsafe {
    //                     HANDOFF.gop_fb_base = fb.as_mut_ptr() as u64;
    //                     HANDOFF.gop_fb_size = fb.size() as u64;
    //                     HANDOFF.gop_width = res.0 as u32;
    //                     HANDOFF.gop_height = res.1 as u32;
    //                     HANDOFF.gop_stride = mode.stride() as u32;
    //                     HANDOFF.gop_pixel_format = match pf {
    //                         UefiPixelFormat::Rgb => 0,
    //                         UefiPixelFormat::Bgr => 1,
    //                         UefiPixelFormat::Bitmask => 2,
    //                         UefiPixelFormat::BltOnly => 3,
    //                     };
    //                 }
    //                 Some((res.0, res.1, pf))
    //             } else {
    //                 None
    //             }
    //         } else {
    //             None
    //         }
    //     }
    //     Err(_) => None,
    // };

    // if let Some((w, h, pf)) = gop_info {
    //     serial_write_line(serial_handle, &alloc::format!("GOP: {}x{} {:?}", w, h, pf));
    //     info!("GOP initialized: {}x{} {:?}", w, h, pf);
    // } else {
    //     serial_write_line(serial_handle, "GOP: not available");
    //     info!("GOP not available");
    // }

    // // Step 9: Exit boot services
    // let _memory_map_after = unsafe {
    //     uefi::boot::exit_boot_services(Some(MemoryType::LOADER_DATA))
    // };
    
    // serial_write_line(serial_handle, "Exited boot services");
    // info!("Exited UEFI boot services");

    // // Step 10: Prepare for kernel handoff
    // let handoff_addr = core::ptr::addr_of!(HANDOFF) as u64;
    
    // serial_write_line(serial_handle, &alloc::format!("Handoff ready at: 0x{:X}", handoff_addr));
    // info!("Handoff prepared at 0x{:X}", handoff_addr);
    
    // // Set handoff pointer in RDI
    // unsafe {
    //     asm!("mov rdi, {}", in(reg) handoff_addr, options(nomem, nostack, preserves_flags));
    // }

    // // Step 11: Final status
    // serial_write_line(serial_handle, "=== UEFI Loader Complete ===");
    // serial_write_line(serial_handle, "Ready for kernel handoff");
    // serial_write_line(serial_handle, "Halting CPU - replace with kernel jump");
    
    // info!("UEFI loader complete, ready for kernel handoff");

    // Halt CPU (placeholder for kernel jump)
    loop {
        unsafe { asm!("hlt", options(nomem, nostack, preserves_flags)) }
    }
}


