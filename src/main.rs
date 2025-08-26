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
use alloc::vec;
use uefi::table::boot::{AllocateType, PAGE_SIZE};

/// Handoff structure passed to the kernel.
///
/// Layout is stable (repr C) to allow consumption from any language.
/// The address of this struct is placed into RDI immediately after
/// ExitBootServices so the kernel can retrieve it on entry.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct Handoff {
	/// Total size of this struct in bytes. Kernel can reserve memory accordingly.
	size: u32,
	/// Handoff format version, increment when fields change.
	handoff_version: u32,
	/// UEFI firmware revision (from SystemTable).
	firmware_revision: u32,
	/// Physical address of the ACPI RSDP (ACPI 2.0 preferred; fallback to 1.0).
	rsdp_address: u64,
	/// Framebuffer base address (if GOP available).
	gop_fb_base: u64,
	/// Framebuffer size in bytes.
	gop_fb_size: u64,
	/// Framebuffer width in pixels.
	gop_width: u32,
	/// Framebuffer height in pixels.
	gop_height: u32,
	/// Framebuffer stride (pixels per scanline).
	gop_stride: u32,
	/// Pixel format enum (0=Rgb, 1=Bgr, 2=Bitmask, 3=BltOnly).
	gop_pixel_format: u32,
	/// Pixel bitmasks if format == Bitmask (else zeroed)
	gop_red_mask: u32,
	gop_green_mask: u32,
	gop_blue_mask: u32,
	gop_reserved_mask: u32,
	/// Memory map pointer (page-allocated buffer we copy to before exit).
	mmap_ptr: u64,
	/// Length of memory map buffer in bytes.
	mmap_len: u64,
	/// Size of each memory descriptor.
	mmap_desc_size: u32,
	/// Memory descriptor version.
	mmap_desc_version: u32,
}

/// Static storage for the handoff data that persists after ExitBootServices.
static mut HANDOFF: Handoff = Handoff {
	size: 0,
	handoff_version: 1,
	firmware_revision: 0,
	rsdp_address: 0,
	gop_fb_base: 0,
	gop_fb_size: 0,
	gop_width: 0,
	gop_height: 0,
	gop_stride: 0,
	gop_pixel_format: 0,
	gop_red_mask: 0,
	gop_green_mask: 0,
	gop_blue_mask: 0,
	gop_reserved_mask: 0,
	mmap_ptr: 0,
	mmap_len: 0,
	mmap_desc_size: 0,
	mmap_desc_version: 0,
};

#[inline]
fn serial_write_line(bt: &BootServices, serial_handle: Option<Handle>, line: &str) {
	if let Some(h) = serial_handle {
		if let Ok(mut serial) = bt.open_protocol_exclusive::<Serial>(h) {
			let _ = serial.write(line.as_bytes());
			let _ = serial.write(b"\r\n");
		}
	}
}

#[inline]
fn screen_write_line(out: &mut uefi::proto::console::text::Output, line: &str) {
	let _ = out.write_str(line);
	let _ = out.write_str("\r\n");
}

#[entry]
fn efi_main(image_handle: Handle, mut system_table: SystemTable<Boot>) -> Status {
	// Initialize UEFI logger/panic
    uefi::helpers::init(&mut system_table).unwrap();

	// Basic greeting to screen and serial
	info!("Initializing...");

	// Buffer lines destined for screen to avoid borrowing stdout across system_table uses
	let mut screen_lines: alloc::vec::Vec<alloc::string::String> = vec![];

	// Firmware vendor and revision
	let vendor = system_table.firmware_vendor();
	let vendor_str = vendor.to_string();
	let rev = system_table.firmware_revision();
	unsafe { HANDOFF.firmware_revision = rev; }

	// Count all handles in a short scope to avoid holding the borrow during stdout writes
	let handle_count: usize = {
		let bt_tmp = system_table.boot_services();
		match bt_tmp.locate_handle_buffer(SearchType::AllHandles) {
			Ok(buf) => buf.len(),
			Err(_) => 0,
		}
	};

	// Queue diagnostics for screen and flush BEFORE touching GOP
    screen_lines.push(alloc::format!("Initializing OS loader..."));
	screen_lines.push(alloc::format!("Firmware: {} rev {}", vendor_str, rev));
	screen_lines.push(alloc::format!("Handles: {}", handle_count));
	{
		let stdout = system_table.stdout();
		//let _ = stdout.clear();
		//let _ = stdout.reset(false);
		for line in &screen_lines {
			screen_write_line(stdout, line);
		}
	}

	// Now proceed with boot services work (serial, GOP, memory map)
	let bt = system_table.boot_services();
	// Cache first serial handle (if any) and print greeting
	let serial_handle = bt
		.locate_handle_buffer(SearchType::ByProtocol(&Serial::GUID))
		.ok()
		.and_then(|buf| buf.first().copied());
	if let Some(h) = serial_handle {
		if let Ok(mut serial) = bt.open_protocol_exclusive::<Serial>(h) {
			let _ = serial.write(b"Hello, world\r\n");
		}
	}

	serial_write_line(bt, serial_handle, &alloc::format!("Firmware: {} rev {}", vendor_str, rev));
	serial_write_line(bt, serial_handle, &alloc::format!("Handles: {}", handle_count));

	// Size field for kernel reservation of this metadata block
	unsafe { HANDOFF.size = core::mem::size_of::<Handoff>() as u32; }

	// Query Graphics Output Protocol for framebuffer parameters (if available)
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
						if let Some(mask) = mode.pixel_bitmask() {
							HANDOFF.gop_red_mask = mask.red;
							HANDOFF.gop_green_mask = mask.green;
							HANDOFF.gop_blue_mask = mask.blue;
							HANDOFF.gop_reserved_mask = mask.reserved;
						}
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

	if let Some((w, h, pf)) = gop_info {
		serial_write_line(bt, serial_handle, &alloc::format!("GOP: {}x{} {:?}", w, h, pf));
	} else {
		serial_write_line(bt, serial_handle, "GOP: not available");
	}

	// Locate ACPI RSDP for kernel ACPI initialization (prefer ACPI 2.0)
	let mut rsdp_addr: u64 = 0;
	for entry in system_table.config_table() {
		if entry.guid == ACPI2_GUID { rsdp_addr = entry.address as u64; break; }
	}
	if rsdp_addr == 0 {
		for entry in system_table.config_table() {
			if entry.guid == ACPI_GUID { rsdp_addr = entry.address as u64; break; }
		}
	}
	unsafe { HANDOFF.rsdp_address = rsdp_addr; }

	// Manual memory map path: allocate buffer, fetch current map and map_key
	let mm_sizes = system_table.boot_services().memory_map_size();
	let mut mm_buf = vec![0u8; mm_sizes.map_size + 8 * mm_sizes.entry_size];
	let mm = match system_table.boot_services().memory_map(&mut mm_buf) {
		Ok(map) => map,
		Err(_e) => return Status::ABORTED,
	};

	// Summarize memory map (entries count and total conventional memory)
	let mut entry_count = 0usize;
	let mut conventional_bytes: u64 = 0;
	for desc in mm.entries() {
		entry_count += 1;
		if desc.ty == MemoryType::CONVENTIONAL {
			conventional_bytes = conventional_bytes.saturating_add(desc.page_count * PAGE_SIZE as u64);
		}
	}
	let conventional_mb = conventional_bytes / (1024 * 1024) as u64;
	serial_write_line(bt, serial_handle, &alloc::format!(
		"Memory map: {} entries, conventional: {} MiB",
		entry_count, conventional_mb
	));

	// Copy the memory map to page-allocated memory that survives ExitBootServices
	let copy_len = mm_buf.len() as u64;
	let pages = ((copy_len + (PAGE_SIZE as u64 - 1)) / PAGE_SIZE as u64) as usize;
	let dest_addr = match bt.allocate_pages(AllocateType::AnyPages, MemoryType::LOADER_DATA, pages) {
		Ok(addr) => addr,
		Err(_e) => return Status::ABORTED,
	};
	unsafe {
		core::ptr::copy_nonoverlapping(
			mm_buf.as_ptr(),
			dest_addr as *mut u8,
			mm_buf.len(),
		);
		HANDOFF.mmap_ptr = dest_addr as u64;
		HANDOFF.mmap_len = mm_buf.len() as u64;
		HANDOFF.mmap_desc_size = core::mem::size_of::<MemoryDescriptor>() as u32;
		HANDOFF.mmap_desc_version = 0;
	}

	// Exit boot services using the convenience API for this crate version
	let (_rt_st, _mm_after) = system_table.exit_boot_services(MemoryType::LOADER_DATA);

	// ABI: pass handoff pointer in RDI for kernel entry to consume
	let handoff_addr = core::ptr::addr_of!(HANDOFF) as u64;
	unsafe {
		asm!("mov rdi, {}", in(reg) handoff_addr, options(nomem, nostack, preserves_flags));
	}

	// Placeholder: halt the CPU. Replace with kernel jump when ready.
	loop {
		unsafe { asm!("hlt", options(nomem, nostack, preserves_flags)) }
	}
}


