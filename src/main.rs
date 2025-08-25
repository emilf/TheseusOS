#![no_std]
#![no_main]

extern crate alloc;

use core::fmt::Write as _;
use log::info;
use uefi::prelude::*;
use uefi::proto::console::serial::Serial;
use uefi::proto::console::gop::GraphicsOutput;
use uefi::table::boot::SearchType;
use uefi::Identify;
use alloc::string::ToString;
use core::arch::asm;
use uefi::table::boot::{MemoryType, MemoryDescriptor};
use uefi::proto::console::gop::PixelFormat as UefiPixelFormat;
use uefi::table::cfg::{ACPI2_GUID, ACPI_GUID};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct Handoff {
    firmware_revision: u32,
    rsdp_address: u64,
    gop_fb_base: u64,
    gop_fb_size: u64,
    gop_width: u32,
    gop_height: u32,
    gop_stride: u32,
    gop_pixel_format: u32,
    mmap_ptr: u64,
    mmap_len: u64,
    mmap_desc_size: u32,
    mmap_desc_version: u32,
}

static mut HANDOFF: Handoff = Handoff {
    firmware_revision: 0,
    rsdp_address: 0,
    gop_fb_base: 0,
    gop_fb_size: 0,
    gop_width: 0,
    gop_height: 0,
    gop_stride: 0,
    gop_pixel_format: 0,
    mmap_ptr: 0,
    mmap_len: 0,
    mmap_desc_size: 0,
    mmap_desc_version: 0,
};

#[entry]
fn efi_main(image_handle: Handle, mut system_table: SystemTable<Boot>) -> Status {
    let _ = uefi_services::init(&mut system_table);

    info!("Hello, world");

    let bt = system_table.boot_services();
    if let Ok(handles) = bt.locate_handle_buffer(SearchType::ByProtocol(&Serial::GUID)) {
        if let Some(&handle) = handles.first() {
            if let Ok(mut serial) = bt.open_protocol_exclusive::<Serial>(handle) {
                let _ = serial.write(b"Hello, world\r\n");
            }
        }
    }

    // Collect boot info
    let vendor = system_table.firmware_vendor();
    let vendor_str = vendor.to_string();
    let rev = system_table.firmware_revision();
    unsafe { HANDOFF.firmware_revision = rev; }

    // Count all handles in the system
    let handle_count = match bt.locate_handle_buffer(SearchType::AllHandles) {
        Ok(buf) => buf.len(),
        Err(_) => 0,
    };

    // Query Graphics Output Protocol (if present)
    let gop_info = match bt.locate_handle_buffer(SearchType::ByProtocol(&GraphicsOutput::GUID)) {
        Ok(gop_handles) => {
            if let Some(&gh) = gop_handles.first() {
                if let Ok(mut gop) = bt.open_protocol_exclusive::<GraphicsOutput>(gh) {
                    let mode = gop.current_mode_info();
                    let res = mode.resolution();
                    let pf = mode.pixel_format();
                    let mut fb = gop.frame_buffer();
                    unsafe {
                        HANDOFF.gop_fb_base = fb.as_mut_ptr() as u64;
                        HANDOFF.gop_fb_size = fb.size() as u64;
                        HANDOFF.gop_width = res.0 as u32;
                        HANDOFF.gop_height = res.1 as u32;
                        HANDOFF.gop_stride = mode.stride() as u32;
                        HANDOFF.gop_pixel_format = match pf {
                            UefiPixelFormat::Rgb => 0,
                            UefiPixelFormat::Bgr => 1,
                            UefiPixelFormat::Bitmask => 2,
                            UefiPixelFormat::BltOnly => 3,
                        };
                    }
                    Some((res.0, res.1, pf))
                } else {
                    None
                }
            } else {
                None
            }
        }
        Err(_) => None,
    };

    // Locate ACPI RSDP in configuration table
    for entry in system_table.config_table() {
        if entry.guid == ACPI2_GUID || entry.guid == ACPI_GUID {
            unsafe { HANDOFF.rsdp_address = entry.address as u64; }
        }
    }

    // Print boot info to screen (logger) and serial (if available)
    info!("Firmware: {} rev {}", vendor_str, rev);
    info!("Handles: {}", handle_count);
    if let Some((w, h, pf)) = gop_info {
        info!("GOP: {}x{} {:?}", w, h, pf);
    } else {
        info!("GOP: not available");
    }

    // Serial again, with formatted info if we still have a serial handle
    if let Ok(handles) = bt.locate_handle_buffer(SearchType::ByProtocol(&Serial::GUID)) {
        if let Some(&handle) = handles.first() {
            if let Ok(mut serial) = bt.open_protocol_exclusive::<Serial>(handle) {
                let _ = write!(serial, "Firmware: {} rev {}\r\n", vendor_str, rev);
                let _ = write!(serial, "Handles: {}\r\n", handle_count);
                if let Some((w, h, pf)) = gop_info {
                    let _ = write!(serial, "GOP: {}x{} {:?}\r\n", w, h, pf);
                } else {
                    let _ = serial.write(b"GOP: not available\r\n");
                }
            }
        }
    }

    let _ = image_handle;
    // Keep the message visible until a key is pressed
    let stdout = system_table.stdout();
    let _ = stdout.clear();
    info!("Hello, world");
    let _ = stdout.write_str("\r\nPress any key to exit...\r\n");

    let _ = system_table.stdin().reset(false);
    loop {
        let key = system_table.stdin().read_key();
        match key {
            Ok(Some(_)) => break,
            _ => system_table.boot_services().stall(100_000),
        }
    }

    // Exit boot services; returns runtime system table and memory map storage
    let (_runtime_st, _mmap) = unsafe { system_table.exit_boot_services(MemoryType::LOADER_DATA) };
    // TODO: Capture raw pointer and length to the memory map without allocation

    // Halt the CPU in a tight loop
    loop {
        unsafe { asm!("hlt", options(nomem, nostack, preserves_flags)) }
    }

    Status::SUCCESS
}


