//! xHCI host controller driver scaffold.
//!
//! This module sets up the groundwork for a fully featured xHCI driver by
//! mapping controller MMIO, inspecting capabilities, and wiring the controller
//! into the driver manager. The focus is on the modern PCIe/MSI path; legacy
//! emulation stays disabled once firmware handoff completes.

use alloc::{format, string::String, vec::Vec};
use core::convert::TryFrom;
use core::hint::spin_loop;
use core::sync::atomic::AtomicBool;
use spin::Mutex;

use crate::acpi;
use crate::drivers::manager::driver_manager;
use crate::drivers::pci;
use crate::drivers::traits::{Device, DeviceClass, DeviceId, DeviceResource, Driver};
use crate::memory::dma::DmaBuffer;
use crate::memory::{
    current_pml4_phys, map_range_with_policy, phys_to_virt_pa, PageTable, PTE_GLOBAL, PTE_NO_EXEC,
    PTE_PCD, PTE_PRESENT, PTE_PWT, PTE_WRITABLE,
};
use crate::{log_debug, log_info, log_warn};

const XHCI_MMIO_WINDOW_BASE: u64 = 0xFFFF_FFB0_0000_0000;
const XHCI_MIN_MMIO: usize = 0x2000;
const USBCMD_RUN_STOP: u32 = 1 << 0;
const USBCMD_HCRST: u32 = 1 << 1;
const USBSTS_HCHALTED: u32 = 1 << 0;
const USBSTS_HOST_CONTROLLER_ERROR: u32 = 1 << 2;
const USBSTS_TRANSFER_EVENT: u32 = 1 << 3;
const USBSTS_CONTROLLER_NOT_READY: u32 = 1 << 11;
const RESET_SPIN_LIMIT: usize = 1_000_000;
const TRB_SIZE: usize = 16;
const COMMAND_RING_TRBS: usize = 256;
const EVENT_RING_TRBS: usize = 256;
const RING_ALIGNMENT: usize = 64;
const ERST_ENTRY_SIZE: usize = 16;
const INTERRUPTER_STRIDE: u32 = 0x20;
const IMAN_IE: u32 = 1 << 1;
const MAX_SLOTS_REGISTER_OFFSET: u32 = 0x2C;
const USBCMD_ENABLE_SLOT: u32 = 1 << 8;
const DCBAA_ENTRY_SIZE: usize = 8;
const RUN_STOP_TIMEOUT: usize = 1_000_000;
static MMIO_MAPPING_LOCK: Mutex<()> = Mutex::new(());
static XHCI_DRIVER: XhciDriver = XhciDriver;
static CONTROLLERS: Mutex<Vec<XhciController>> = Mutex::new(Vec::new());

#[allow(dead_code)]
struct XhciController {
    phys_base: u64,
    virt_base: u64,
    mmio_length: usize,
    cap_length: u8,
    hci_version: u16,
    operational_offset: u32,
    runtime_offset: u32,
    doorbell_offset: u32,
    max_ports: u8,
    max_slots: u8,
    msi_enabled: AtomicBool,
    command_ring: Option<DmaBuffer>,
    event_ring: Option<DmaBuffer>,
    event_ring_table: Option<DmaBuffer>,
    command_ring_cycle_state: bool,
    slots_enabled: bool,
    dcbaa: Option<DmaBuffer>,
    controller_running: bool,
}

pub fn register_xhci_driver() {
    driver_manager().lock().register_driver(&XHCI_DRIVER);
}

struct XhciDriver;

impl XhciDriver {
    fn pci_ident(dev: &Device) -> (u16, u8, u8, u8) {
        match dev.id {
            DeviceId::Pci {
                segment,
                bus,
                device,
                function,
            } => (segment, bus, device, function),
            _ => (0, 0, 0, 0),
        }
    }

    fn pci_display(dev: &Device) -> String {
        let (segment, bus, device, function) = Self::pci_ident(dev);
        format!("{:04x}:{:02x}:{:02x}.{}", segment, bus, device, function)
    }

    fn locate_mmio(&self, dev: &Device) -> Result<(u64, usize), &'static str> {
        for res in dev.resources.iter() {
            if let DeviceResource::Memory { base, size, .. } = res {
                let length = usize::try_from(*size).unwrap_or(XHCI_MIN_MMIO);
                return Ok((*base, length.max(XHCI_MIN_MMIO)));
            }
        }
        dev.phys_addr
            .map(|base| (base, XHCI_MIN_MMIO))
            .ok_or("xHCI controller lacks memory resource")
    }

    fn map_mmio(&self, phys_addr: u64, size: usize) -> u64 {
        let page_size = crate::memory::PAGE_SIZE as u64;
        let phys_base = phys_addr & !(page_size - 1);
        let offset = phys_addr - phys_base;
        let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;
        let virt_base = XHCI_MMIO_WINDOW_BASE + phys_base;

        let _guard = MMIO_MAPPING_LOCK.lock();

        unsafe {
            let pml4_pa = current_pml4_phys();
            let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
            let pml4 = &mut *pml4_ptr;
            let mut allocator = crate::physical_memory::PersistentFrameAllocator;
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

    fn initialise_controller(
        &'static self,
        dev: &mut Device,
        phys_base: u64,
        mmio_length: usize,
    ) -> Result<bool, &'static str> {
        let ident = Self::pci_display(dev);
        let virt = self.map_mmio(phys_base, mmio_length);
        unsafe {
            let base = virt as *const u8;
            let cap_length = core::ptr::read_volatile(base);
            let hci_version = core::ptr::read_volatile(base.add(2) as *const u16);
            let hcsparams1 = core::ptr::read_volatile(base.add(0x04) as *const u32);
            let hccparams1 = core::ptr::read_volatile(base.add(0x10) as *const u32);
            let operational_offset = cap_length as u32;
            let runtime_offset = core::ptr::read_volatile(base.add(0x18) as *const u32) & !0b11;
            let doorbell_offset = core::ptr::read_volatile(base.add(0x14) as *const u32) & !0b11;

            {
                let list = CONTROLLERS.lock();
                if let Some(existing) = list.iter().find(|ctrl| ctrl.phys_base == phys_base) {
                    log_debug!(
                        "xHCI {} shares phys base {:#x}; reusing existing mapping",
                        ident,
                        phys_base
                    );
                    dev.driver_data = Some(existing as *const XhciController as usize);
                    return Ok(false);
                }
            }

            self.reset_controller(virt, operational_offset, &ident)?;
            self.log_capabilities(
                &ident,
                hci_version,
                hcsparams1,
                hccparams1,
                cap_length,
                runtime_offset,
                doorbell_offset,
            );

            self.log_operational_state(virt, operational_offset, &ident);

            let controller = XhciController {
                phys_base,
                virt_base: virt,
                mmio_length,
                cap_length,
                hci_version,
                operational_offset,
                runtime_offset,
                doorbell_offset,
                max_ports: ((hcsparams1 >> 24) & 0xFF) as u8,
                max_slots: ((hcsparams1 & 0xFF) + 1) as u8,
                msi_enabled: AtomicBool::new(false),
                command_ring: None,
                event_ring: None,
                event_ring_table: None,
                command_ring_cycle_state: true,
                slots_enabled: false,
                dcbaa: None,
                controller_running: false,
            };

            let mut list = CONTROLLERS.lock();
            list.push(controller);
            let controller_ref = list.last_mut().unwrap();
            if let Err(err) = self.initialise_memory(controller_ref, &ident) {
                log_warn!("xHCI {}: memory init failed: {}", ident, err);
            }
            if let Err(err) = self.configure_command_ring(controller_ref, &ident) {
                log_warn!("xHCI {}: command ring setup failed: {}", ident, err);
            }
            if let Err(err) = self.configure_event_ring(controller_ref, &ident) {
                log_warn!("xHCI {}: event ring setup failed: {}", ident, err);
            }
            if let Err(err) = self.configure_dcbaa(controller_ref, &ident) {
                log_warn!("xHCI {}: DCBAA setup failed: {}", ident, err);
            }
            if let Err(err) = self.enable_slots(controller_ref, &ident) {
                log_warn!("xHCI {}: slot configuration failed: {}", ident, err);
            }
            if let Err(err) = self.start_controller(controller_ref, &ident) {
                log_warn!("xHCI {}: run-state transition failed: {}", ident, err);
            }
            let stored = controller_ref as *const XhciController;
            dev.driver_data = Some(stored as usize);
        }

        Ok(true)
    }

    fn try_enable_msi(&self, dev: &Device) {
        let ident = Self::pci_display(dev);
        let DeviceId::Pci {
            segment,
            bus,
            device,
            function,
        } = dev.id
        else {
            log_warn!("xHCI driver: device lacks PCI identity; cannot configure MSI");
            return;
        };

        let platform = match acpi::cached_platform_info() {
            Some(info) => info,
            None => {
                log_debug!("xHCI {}: platform info unavailable for MSI setup", ident);
                return;
            }
        };

        let topology = pci::enumerate(&platform.pci_config_regions);
        let Some(pci_info) = topology.functions.iter().find(|info| {
            info.segment == segment
                && info.bus == bus
                && info.device == device
                && info.function == function
        }) else {
            log_debug!("xHCI {}: PCI info not found while enabling MSI", ident);
            return;
        };

        if !pci_info.capabilities.msi {
            log_info!(
                "xHCI {:02x}:{:02x}.{}: MSI not advertised, skipping setup",
                bus,
                device,
                function
            );
            return;
        }

        // TODO: integrate with interrupt allocator to source a vector dynamically.
        log_info!(
            "xHCI {:02x}:{:02x}.{} advertises MSI/MSI-X (msi={} msix={})",
            bus,
            device,
            function,
            pci_info.capabilities.msi,
            pci_info.capabilities.msix
        );
    }

    fn reset_controller(
        &self,
        virt_base: u64,
        operational_offset: u32,
        ident: &str,
    ) -> Result<(), &'static str> {
        let op_base = (virt_base + operational_offset as u64) as *mut u32;
        unsafe {
            let cmd_ptr = op_base.add((0x00 / 4) as usize);
            let sts_ptr = op_base.add((0x04 / 4) as usize);

            if core::ptr::read_volatile(sts_ptr) & USBSTS_HCHALTED == 0 {
                let mut cmd = core::ptr::read_volatile(cmd_ptr);
                cmd &= !USBCMD_RUN_STOP;
                core::ptr::write_volatile(cmd_ptr, cmd);
                if !self.poll_with_timeout(
                    || core::ptr::read_volatile(sts_ptr) & USBSTS_HCHALTED != 0,
                    RESET_SPIN_LIMIT,
                ) {
                    log_warn!("xHCI {}: halt before reset timed out", ident);
                    return Err("xhci halt timeout");
                }
            }

            let mut cmd = core::ptr::read_volatile(cmd_ptr);
            cmd |= USBCMD_HCRST;
            core::ptr::write_volatile(cmd_ptr, cmd);
            if !self.poll_with_timeout(
                || core::ptr::read_volatile(cmd_ptr) & USBCMD_HCRST == 0,
                RESET_SPIN_LIMIT,
            ) {
                log_warn!("xHCI {}: controller reset timed out", ident);
                return Err("xhci reset timeout");
            }

            if !self.poll_with_timeout(
                || core::ptr::read_volatile(sts_ptr) & USBSTS_HCHALTED != 0,
                RESET_SPIN_LIMIT,
            ) {
                log_warn!("xHCI {}: controller not halted after reset", ident);
            }
        }

        log_info!("xHCI {} controller reset complete", ident);
        Ok(())
    }

    fn poll_with_timeout<F>(&self, mut predicate: F, limit: usize) -> bool
    where
        F: FnMut() -> bool,
    {
        for _ in 0..limit {
            if predicate() {
                return true;
            }
            spin_loop();
        }
        false
    }

    fn initialise_memory(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        if controller.command_ring.is_some() {
            return Ok(());
        }

        let command_ring = self.allocate_ring(COMMAND_RING_TRBS, "command")?;
        let event_ring = self.allocate_ring(EVENT_RING_TRBS, "event")?;

        log_info!(
            "xHCI {} ring allocation: command phys={:#012x} size={} event phys={:#012x} size={}",
            ident,
            command_ring.phys_addr(),
            command_ring.len(),
            event_ring.phys_addr(),
            event_ring.len()
        );

        controller.command_ring = Some(command_ring);
        controller.event_ring = Some(event_ring);
        Ok(())
    }

    fn allocate_ring(&self, trbs: usize, tag: &str) -> Result<DmaBuffer, &'static str> {
        let size = trbs * TRB_SIZE;
        DmaBuffer::allocate(size, RING_ALIGNMENT).map_err(|_| match tag {
            "command" => "failed to allocate command ring",
            "event" => "failed to allocate event ring",
            _ => "failed to allocate ring",
        })
    }

    fn configure_command_ring(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let Some(ring) = controller.command_ring.as_ref() else {
            return Err("command ring not allocated");
        };

        let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
        unsafe {
            let crcr_ptr = op_base.add(0x18) as *mut u64;
            let cycle_bit = if controller.command_ring_cycle_state {
                1u64
            } else {
                0
            };
            let value = (ring.phys_addr() & !0xF) | cycle_bit;
            core::ptr::write_volatile(crcr_ptr, value);
        }

        log_info!(
            "xHCI {} command ring configured: crcr={:#012x} (cycle {})",
            ident,
            ring.phys_addr(),
            if controller.command_ring_cycle_state {
                1
            } else {
                0
            }
        );

        Ok(())
    }

    fn configure_event_ring(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let Some(event_ring) = controller.event_ring.as_ref() else {
            return Err("event ring not allocated");
        };

        if controller.event_ring_table.is_none() {
            let table = DmaBuffer::allocate(ERST_ENTRY_SIZE, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate ERST")?;
            controller.event_ring_table = Some(table);
        }

        let table = controller.event_ring_table.as_mut().unwrap();
        unsafe {
            let entry_ptr = table.virt_addr() as *mut u8;
            core::ptr::write_volatile(entry_ptr as *mut u64, event_ring.phys_addr());
            core::ptr::write_volatile(entry_ptr.add(8) as *mut u32, EVENT_RING_TRBS as u32);
            core::ptr::write_volatile(entry_ptr.add(12) as *mut u32, 0);
        }

        let runtime_base = (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
        unsafe {
            let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
            let iman_ptr = interrupter0.add(0x00) as *mut u32;
            let imod_ptr = interrupter0.add(0x04) as *mut u32;
            let erstsz_ptr = interrupter0.add(0x08) as *mut u32;
            let erstba_ptr = interrupter0.add(0x10) as *mut u64;
            let erdp_ptr = interrupter0.add(0x18) as *mut u64;

            core::ptr::write_volatile(imod_ptr, 0);
            core::ptr::write_volatile(erstsz_ptr, 1);
            core::ptr::write_volatile(erstba_ptr, table.phys_addr());
            core::ptr::write_volatile(erdp_ptr, event_ring.phys_addr() & !0xF);

            let iman = core::ptr::read_volatile(iman_ptr);
            core::ptr::write_volatile(iman_ptr, (iman & !1) | IMAN_IE);
        }

        log_info!(
            "xHCI {} event ring configured: erst={:#012x} entries=1 erdp={:#012x}",
            ident,
            controller.event_ring_table.as_ref().unwrap().phys_addr(),
            event_ring.phys_addr()
        );

        Ok(())
    }

    fn configure_dcbaa(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        if controller.dcbaa.is_none() {
            let entries = controller.max_slots as usize + 1;
            let size = entries * DCBAA_ENTRY_SIZE;
            let dcbaa = DmaBuffer::allocate(size, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate DCBAA")?;
            controller.dcbaa = Some(dcbaa);
        }

        let dcbaa = controller.dcbaa.as_ref().unwrap();
        let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
        unsafe {
            let dcbaap_ptr = op_base.add(0x30) as *mut u64;
            core::ptr::write_volatile(dcbaap_ptr, dcbaa.phys_addr());
        }

        log_info!(
            "xHCI {} DCBAA configured: ptr={:#012x} entries={}",
            ident,
            dcbaa.phys_addr(),
            controller.max_slots as usize + 1
        );

        Ok(())
    }

    fn enable_slots(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        if controller.slots_enabled {
            return Ok(());
        }

        unsafe {
            let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
            let config_ptr = op_base.add(MAX_SLOTS_REGISTER_OFFSET as usize) as *mut u32;
            core::ptr::write_volatile(config_ptr, controller.max_slots as u32);

            let cmd_ptr = op_base.add(0x00) as *mut u32;
            let mut cmd = core::ptr::read_volatile(cmd_ptr);
            cmd |= USBCMD_ENABLE_SLOT;
            core::ptr::write_volatile(cmd_ptr, cmd);
        }

        controller.slots_enabled = true;
        log_info!(
            "xHCI {} slot configuration: max_slots={}",
            ident,
            controller.max_slots
        );

        Ok(())
    }

    fn start_controller(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        if controller.controller_running {
            return Ok(());
        }
        if controller.command_ring.is_none()
            || controller.event_ring.is_none()
            || controller.dcbaa.is_none()
        {
            return Err("controller buffers incomplete");
        }

        unsafe {
            let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
            let cmd_ptr = op_base.add(0x00) as *mut u32;
            let sts_ptr = op_base.add(0x04) as *mut u32;

            let mut cmd = core::ptr::read_volatile(cmd_ptr);
            if cmd & USBCMD_RUN_STOP == 0 {
                cmd |= USBCMD_RUN_STOP;
                core::ptr::write_volatile(cmd_ptr, cmd);
            }

            if !self.poll_with_timeout(
                || core::ptr::read_volatile(sts_ptr) & USBSTS_HCHALTED == 0,
                RUN_STOP_TIMEOUT,
            ) {
                return Err("controller failed to leave halt state");
            }

            let status = core::ptr::read_volatile(sts_ptr);
            let ready = (status & USBSTS_CONTROLLER_NOT_READY) == 0;
            if !ready {
                log_warn!("xHCI {} controller not ready after run", ident);
            }
        }

        controller.controller_running = true;
        log_info!("xHCI {} controller running", ident);
        Ok(())
    }

    fn describe_status(&self, sts: u32) -> String {
        let mut parts = Vec::new();
        if sts & USBSTS_HCHALTED != 0 {
            parts.push("HALTED");
        }
        if sts & USBSTS_HOST_CONTROLLER_ERROR != 0 {
            parts.push("ERROR");
        }
        if sts & USBSTS_TRANSFER_EVENT != 0 {
            parts.push("EVENT");
        }
        if sts & USBSTS_CONTROLLER_NOT_READY != 0 {
            parts.push("NOT_READY");
        }
        if parts.is_empty() {
            "none".into()
        } else {
            parts.join("|")
        }
    }

    fn log_capabilities(
        &self,
        ident: &str,
        hci_version: u16,
        hcsparams1: u32,
        hccparams1: u32,
        cap_length: u8,
        runtime_offset: u32,
        doorbell_offset: u32,
    ) {
        let context_size = if (hccparams1 & (1 << 2)) != 0 { 64 } else { 32 };
        let port_count = (hcsparams1 >> 24) & 0xFF;
        let max_slots = (hcsparams1 & 0xFF) + 1;
        log_info!(
            "xHCI {} HCI v{}.{} slots={} ports={} context_size={}",
            ident,
            hci_version >> 8,
            hci_version & 0xFF,
            max_slots,
            port_count,
            context_size
        );
        log_debug!(
            "xHCI {} capability offsets: caplen={:#x} operational={:#x} runtime={:#x} doorbell={:#x}",
            ident,
            cap_length,
            cap_length,
            runtime_offset,
            doorbell_offset
        );
    }

    fn log_operational_state(&self, virt_base: u64, operational_offset: u32, ident: &str) {
        unsafe {
            let op_base = (virt_base + operational_offset as u64) as *const u32;
            let cmd = core::ptr::read_volatile(op_base.add((0x00 / 4) as usize));
            let sts = core::ptr::read_volatile(op_base.add((0x04 / 4) as usize));
            let config = core::ptr::read_volatile(op_base.add((0x38 / 4) as usize));
            let status_flags = self.describe_status(sts);
            log_debug!(
                "xHCI {} operational state: USBCMD={:#010x} USBSTS={:#010x} ({}) CONFIG={:#010x}",
                ident,
                cmd,
                sts,
                status_flags,
                config
            );
        }
    }
}

impl Driver for XhciDriver {
    fn supported_classes(&self) -> &'static [DeviceClass] {
        &[DeviceClass::UsbController]
    }

    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        if dev.class != DeviceClass::UsbController {
            return Err("not USB class");
        }
        if self.locate_mmio(dev).is_err() {
            return Err("missing MMIO resource");
        }
        Ok(())
    }

    fn init(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        let (phys, length) = self.locate_mmio(dev)?;
        if dev.phys_addr.is_none() {
            dev.phys_addr = Some(phys);
        }

        let newly_mapped = self.initialise_controller(dev, phys, length)?;
        if newly_mapped {
            self.try_enable_msi(dev);
            log_debug!("xHCI controller initialised at phys={:#012x}", phys);
        }

        Ok(())
    }

    fn irq_handler(&'static self, _dev: &mut Device, _irq: u32) -> bool {
        // Placeholder: mainline path will be implemented alongside interrupt allocator work.
        false
    }
}
