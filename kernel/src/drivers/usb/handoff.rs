//! USB legacy ownership handoff helpers.
//!
//! These routines release USB host controllers from firmware emulation so the
//! kernel can safely program them. Both EHCI and xHCI controllers expose a
//! "USB Legacy Support" capability that coordinates BIOS- and OS-owned
//! semaphores; we locate that capability, assert OS ownership, and clear any
//! lingering SMI traps before higher-level drivers begin initialization.

use alloc::collections::BTreeSet;
use core::convert::TryFrom;
use core::hint::spin_loop;

use crate::drivers::pci::{PciBar, PciDeviceInfo};
use crate::memory::{
    current_pml4_phys, map_range_with_policy, phys_to_virt_pa, PageTable, PTE_GLOBAL, PTE_NO_EXEC,
    PTE_PCD, PTE_PRESENT, PTE_PWT, PTE_WRITABLE,
};
use crate::physical_memory::PersistentFrameAllocator;
use crate::{log_debug, log_info, log_warn};
use spin::Mutex;

const USB_MMIO_WINDOW_BASE: u64 = 0xFFFF_FFA0_0000_0000;
const USB_HANDOFF_MIN_MMIO: usize = 0x1000;
const USB_LEGACY_CAP_ID: u8 = 0x01;
const BIOS_OWNED_SEMAPHORE: u32 = 1 << 16;
const OS_OWNED_SEMAPHORE: u32 = 1 << 24;
const HANDOFF_SPIN_LIMIT: u32 = 1_000_000;
static USB_MMIO_MAPPING_LOCK: Mutex<()> = Mutex::new(());

#[derive(Debug)]
struct LegacyResult {
    bios_released: bool,
    bios_wait_iters: u32,
}

/// Ensure the kernel owns all USB controllers that advertise a legacy handoff path.
///
/// The routine walks the enumerated PCI functions, looks for EHCI (prog_if 0x20)
/// and xHCI (prog_if 0x30) controllers, and performs the standard ownership
/// handshake if a legacy capability is present.
pub fn ensure_legacy_usb_handoff(devices: &[PciDeviceInfo]) {
    let mut visited_mmio = BTreeSet::new();

    for device in devices {
        match device.class_triplet() {
            (0x0C, 0x03, 0x30) => {
                if let Err(err) = handoff_xhci(device, &mut visited_mmio) {
                    log_warn!(
                        "xHCI legacy handoff failed for {:04x}:{:02x}:{:02x}.{}: {}",
                        device.segment,
                        device.bus,
                        device.device,
                        device.function,
                        err
                    );
                }
            }
            (0x0C, 0x03, 0x20) => {
                if let Err(err) = handoff_ehci(device, &mut visited_mmio) {
                    log_warn!(
                        "EHCI legacy handoff failed for {:04x}:{:02x}:{:02x}.{}: {}",
                        device.segment,
                        device.bus,
                        device.device,
                        device.function,
                        err
                    );
                }
            }
            _ => {}
        }
    }
}

fn handoff_xhci(device: &PciDeviceInfo, visited: &mut BTreeSet<u64>) -> Result<(), &'static str> {
    let (phys, size) = controller_mmio_region(device)?;

    if !visited.insert(phys) {
        log_debug!(
            "Skipping duplicate xHCI legacy handoff for {:04x}:{:02x}:{:02x}.{} (phys=0x{:012x})",
            device.segment,
            device.bus,
            device.device,
            device.function,
            phys
        );
        return Ok(());
    }

    let virt = map_controller_mmio(phys, size);
    let legacy_ptr =
        unsafe { get_xhci_legacy_pointer(virt) }.ok_or("xHCI extended capabilities missing")?;

    let result = unsafe { perform_usb_legacy_handoff(virt, legacy_ptr) }?;
    if result.bios_released {
        log_info!(
            "xHCI legacy handoff complete for {:04x}:{:02x}:{:02x}.{} (waited {} iters)",
            device.segment,
            device.bus,
            device.device,
            device.function,
            result.bios_wait_iters
        );
    } else {
        log_warn!(
            "xHCI BIOS ownership stuck for {:04x}:{:02x}:{:02x}.{} (waited {} iters)",
            device.segment,
            device.bus,
            device.device,
            device.function,
            result.bios_wait_iters
        );
    }

    Ok(())
}

fn handoff_ehci(device: &PciDeviceInfo, visited: &mut BTreeSet<u64>) -> Result<(), &'static str> {
    let (phys, size) = controller_mmio_region(device)?;

    if !visited.insert(phys) {
        log_debug!(
            "Skipping duplicate EHCI legacy handoff for {:04x}:{:02x}:{:02x}.{} (phys=0x{:012x})",
            device.segment,
            device.bus,
            device.device,
            device.function,
            phys
        );
        return Ok(());
    }

    let virt = map_controller_mmio(phys, size);
    let legacy_ptr =
        unsafe { get_ehci_legacy_pointer(virt) }.ok_or("EHCI extended capabilities missing")?;

    let result = unsafe { perform_usb_legacy_handoff(virt, legacy_ptr) }?;
    if result.bios_released {
        log_info!(
            "EHCI legacy handoff complete for {:04x}:{:02x}:{:02x}.{} (waited {} iters)",
            device.segment,
            device.bus,
            device.device,
            device.function,
            result.bios_wait_iters
        );
    } else {
        log_warn!(
            "EHCI BIOS ownership stuck for {:04x}:{:02x}:{:02x}.{} (waited {} iters)",
            device.segment,
            device.bus,
            device.device,
            device.function,
            result.bios_wait_iters
        );
    }

    Ok(())
}

fn controller_mmio_region(device: &PciDeviceInfo) -> Result<(u64, usize), &'static str> {
    let bar = device
        .first_memory_bar()
        .ok_or("controller missing memory BAR")?;

    match bar {
        PciBar::Memory32 { base, size, .. } | PciBar::Memory64 { base, size, .. } => {
            let len = match usize::try_from(size) {
                Ok(value) if value > 0 => value,
                Ok(_) => USB_HANDOFF_MIN_MMIO,
                Err(_) => {
                    log_warn!(
                        "BAR size {:#x} too large for usize, defaulting to {} bytes",
                        size,
                        USB_HANDOFF_MIN_MMIO
                    );
                    USB_HANDOFF_MIN_MMIO
                }
            };
            Ok((base, len.max(USB_HANDOFF_MIN_MMIO)))
        }
        _ => Err("controller BAR is not MMIO"),
    }
}

fn map_controller_mmio(phys_addr: u64, size: usize) -> u64 {
    let page_size = crate::memory::PAGE_SIZE as u64;
    let phys_base = phys_addr & !(page_size - 1);
    let offset = phys_addr - phys_base;
    let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;
    let virt_base = USB_MMIO_WINDOW_BASE + phys_base;

    let _guard = USB_MMIO_MAPPING_LOCK.lock();

    unsafe {
        let pml4_pa = current_pml4_phys();
        let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
        let pml4 = &mut *pml4_ptr;
        let mut allocator = PersistentFrameAllocator;
        let flags = PTE_PRESENT | PTE_WRITABLE | PTE_GLOBAL | PTE_NO_EXEC | PTE_PCD | PTE_PWT;
        map_range_with_policy(
            pml4,
            virt_base,
            phys_base,
            size_aligned,
            flags,
            &mut allocator,
        );
    }

    virt_base + offset
}

unsafe fn get_xhci_legacy_pointer(base: u64) -> Option<u32> {
    let hccparams1 = core::ptr::read_volatile((base + 0x10) as *const u32);
    let ptr = ((hccparams1 >> 16) & 0xFFFF) as u32;
    if ptr == 0 {
        None
    } else {
        Some(ptr << 2)
    }
}

unsafe fn get_ehci_legacy_pointer(base: u64) -> Option<u32> {
    let hccparams = core::ptr::read_volatile((base + 0x08) as *const u32);
    let ptr = ((hccparams >> 8) & 0xFF) as u32;
    if ptr == 0 {
        None
    } else {
        Some(ptr << 2)
    }
}

unsafe fn perform_usb_legacy_handoff(
    base: u64,
    mut offset: u32,
) -> Result<LegacyResult, &'static str> {
    if offset == 0 {
        return Err("legacy capability pointer is zero");
    }

    let mut guard = 0;
    while offset != 0 && guard < 32 {
        let cap_addr = base + offset as u64;
        let cap_header = core::ptr::read_volatile(cap_addr as *const u32);
        let cap_id = (cap_header & 0xFF) as u8;
        let next = ((cap_header >> 8) & 0xFF) as u8;

        if cap_id == USB_LEGACY_CAP_ID {
            if (cap_header & OS_OWNED_SEMAPHORE) == 0 {
                core::ptr::write_volatile(cap_addr as *mut u32, cap_header | OS_OWNED_SEMAPHORE);
            }

            let mut spins = 0;
            let mut bios_released = false;
            loop {
                let current = core::ptr::read_volatile(cap_addr as *const u32);
                if (current & BIOS_OWNED_SEMAPHORE) == 0 {
                    if (current & OS_OWNED_SEMAPHORE) == 0 {
                        core::ptr::write_volatile(
                            cap_addr as *mut u32,
                            current | OS_OWNED_SEMAPHORE,
                        );
                    }
                    bios_released = true;
                    break;
                }
                if spins >= HANDOFF_SPIN_LIMIT {
                    break;
                }
                spins += 1;
                spin_loop();
            }

            core::ptr::write_volatile((cap_addr + 4) as *mut u32, 0);

            return Ok(LegacyResult {
                bios_released,
                bios_wait_iters: spins,
            });
        }

        let next_offset = (next as u32) << 2;
        if next == 0 || next_offset == offset {
            break;
        }
        offset = next_offset;
        guard += 1;
    }

    Err("legacy capability not found")
}
