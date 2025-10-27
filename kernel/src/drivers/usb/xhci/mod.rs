//! xHCI host controller driver scaffold for the teaching kernel.
//!
//! The goal of this module is educational: it walks through the steps required
//! to take ownership of a modern USB 3 controller on x86-64 while exercising
//! Rust's safety features. The code demonstrates how to map MMIO regions,
//! interpret capability structures, set up command/event rings, transition the
//! controller to `RUN`, and submit a minimal NOOP command to verify the data
//! path. Later lessons will extend this scaffolding to cover slot contexts and
//! full USB enumeration.

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
const TRB_TYPE_NOOP_COMMAND: u32 = 0x17;
const TRB_COMPLETION_CODE_MASK: u32 = 0xFF << 24;
const TRB_TYPE_MASK: u32 = 0x3F << 10;
const TRB_TYPE_COMMAND_COMPLETION: u32 = 0x21;
const TRB_COMPLETION_SUCCESS: u32 = 1;
const TRB_CYCLE_BIT: u32 = 1;
const DOORBELL_HOST: u32 = 0;
const IMAN_IP: u32 = 1 << 0;
const DEVICE_CONTEXT_ALIGN: usize = 64;
const DEVICE_CONTEXT_SIZE: usize = 2048;
static MMIO_MAPPING_LOCK: Mutex<()> = Mutex::new(());
static XHCI_DRIVER: XhciDriver = XhciDriver;
static CONTROLLERS: Mutex<Vec<XhciController>> = Mutex::new(Vec::new());

#[allow(dead_code)]
/// Aggregated state for a discovered controller.
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
    command_ring: Option<CommandRing>,
    event_ring: Option<EventRing>,
    slots_enabled: bool,
    dcbaa: Option<DmaBuffer>,
    controller_running: bool,
    last_command_status: Option<u32>,
    pending_commands: usize,
    slot_contexts: Vec<DmaBuffer>,
}

struct CommandRing {
    buffer: DmaBuffer,
    index: usize,
    cycle: bool,
}

struct EventRing {
    buffer: DmaBuffer,
    table: DmaBuffer,
    index: usize,
    cycle: bool,
}

impl CommandRing {
    /// Construct a new command ring wrapper around the provided DMA buffer.
    fn new(buffer: DmaBuffer) -> Self {
        Self {
            buffer,
            index: 0,
            cycle: true,
        }
    }

    /// Physical base address advertised to hardware via CRCR.
    fn phys_addr(&self) -> u64 {
        self.buffer.phys_addr()
    }

    /// Pointer to the next TRB slot to be written by software.
    fn current_trb(&self) -> *mut u32 {
        (self.buffer.virt_addr() + (self.index * TRB_SIZE) as u64) as *mut u32
    }

    /// Cycle bit value to encode in the TRB control dword.
    fn cycle_bit_u32(&self) -> u32 {
        if self.cycle {
            TRB_CYCLE_BIT
        } else {
            0
        }
    }

    /// Cycle bit value as u64 for CRCR programming.
    fn cycle_bit_u64(&self) -> u64 {
        if self.cycle {
            TRB_CYCLE_BIT as u64
        } else {
            0
        }
    }

    /// Advance to the next TRB, flipping the cycle bit when wrapping.
    fn advance(&mut self) {
        self.index += 1;
        if self.index == COMMAND_RING_TRBS {
            self.index = 0;
            self.cycle = !self.cycle;
        }
    }
}

impl EventRing {
    /// Construct a new event ring wrapper with its associated ERST entry.
    fn new(buffer: DmaBuffer, table: DmaBuffer) -> Self {
        Self {
            buffer,
            table,
            index: 0,
            cycle: true,
        }
    }

    /// Physical base address of the event ring buffer.
    fn phys_addr(&self) -> u64 {
        self.buffer.phys_addr()
    }

    /// Physical address of the ERST entry.
    fn table_phys_addr(&self) -> u64 {
        self.table.phys_addr()
    }

    /// Pointer to the next TRB that software should inspect.
    fn current_trb(&self) -> *mut u32 {
        (self.buffer.virt_addr() + (self.index * TRB_SIZE) as u64) as *mut u32
    }

    /// Advance the consumer pointer, toggling the expected cycle on wrap.
    fn advance(&mut self) {
        self.index += 1;
        if self.index == EVENT_RING_TRBS {
            self.index = 0;
            self.cycle = !self.cycle;
        }
    }
}

/// Register the xHCI scaffold driver with the global driver manager.
///
/// # Returns
/// Nothing. The driver is inserted into the global registry and will be probed
/// during device enumeration.
pub fn register_xhci_driver() {
    driver_manager().lock().register_driver(&XHCI_DRIVER);
}

struct XhciDriver;

impl XhciDriver {
    /// Extract the PCI segment/bus/device/function tuple from a device entry.
    ///
    /// # Parameters
    /// * `dev` - Device descriptor obtained from the driver manager.
    ///
    /// # Returns
    /// `(segment, bus, device, function)` values identifying the controller.
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

    /// Human readable PCI identifier used in log messages.
    ///
    /// # Parameters
    /// * `dev` - Device descriptor obtained from the driver manager.
    ///
    /// # Returns
    /// String in the form `ssss:bb:dd.f` (segment, bus, device, function).
    fn pci_display(dev: &Device) -> String {
        let (segment, bus, device, function) = Self::pci_ident(dev);
        format!("{:04x}:{:02x}:{:02x}.{}", segment, bus, device, function)
    }

    /// Determine the controller's MMIO base and size from the reported BARs.
    ///
    /// # Parameters
    /// * `dev` - Device descriptor containing resource information.
    ///
    /// # Returns
    /// On success, the physical base address and length of the MMIO window.
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

    /// Map the xHCI MMIO aperture into the kernel address space with uncached
    /// attributes.
    ///
    /// # Parameters
    /// * `phys_addr` - Physical base of the MMIO window.
    /// * `size` - Size of the requested mapping.
    ///
    /// # Returns
    /// Virtual address where the registers are accessible.
    fn map_mmio(&self, phys_addr: u64, size: usize) -> u64 {
        let page_size = crate::memory::PAGE_SIZE as u64;
        // Bring the physical base down to a page boundary so we can map with
        // standard paging helpers.
        let phys_base = phys_addr & !(page_size - 1);
        let offset = phys_addr - phys_base;
        // Expand the mapping to cover the entire window plus any leading
        // offset introduced by alignment.
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

    /// Map, reset, and cache metadata about a controller.
    ///
    /// # Parameters
    /// * `dev` - Device descriptor representing the controller.
    /// * `phys_base` - Physical base address of the MMIO window.
    /// * `mmio_length` - Length of the MMIO window.
    ///
    /// # Returns
    /// `Ok(true)` if a new controller instance was created, `Ok(false)` if an
    /// existing controller was reused, or an error string on failure.
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
                slots_enabled: false,
                dcbaa: None,
                controller_running: false,
                last_command_status: None,
                pending_commands: 0,
                slot_contexts: Vec::new(),
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
            if let Err(err) = self.submit_noop(controller_ref, &ident) {
                log_warn!("xHCI {}: noop command submission failed: {}", ident, err);
            }
            let stored = controller_ref as *const XhciController;
            dev.driver_data = Some(stored as usize);
        }

        Ok(true)
    }

    /// If advertised, prime the MSI capability so future lessons can route
    /// interrupts without touching the legacy IOAPIC path.
    ///
    /// # Parameters
    /// * `dev` - Device descriptor representing the controller.
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

    /// Reset the controller after ensuring it is halted.
    ///
    /// # Parameters
    /// * `virt_base` - Virtual base address of the controller MMIO window.
    /// * `operational_offset` - Offset to the operational register block.
    /// * `ident` - Identifier used in log messages.
    ///
    /// # Returns
    /// `Ok(())` on success or an error string on timeout.
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

    /// Helper: repeatedly evaluate `predicate` until it returns true or the
    /// supplied spin limit is reached.
    ///
    /// # Parameters
    /// * `predicate` - Closure evaluated each iteration.
    /// * `limit` - Maximum number of iterations before timing out.
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

    /// Allocate the command ring, event ring, and ERST buffers.
    ///
    /// # Parameters
    /// * `controller` - Mutable controller state that will own the buffers.
    /// * `ident` - Identifier used in log messages.
    ///
    /// # Returns
    /// `Ok(())` on success or an error string if allocation fails.
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
        let erst = DmaBuffer::allocate(ERST_ENTRY_SIZE, RING_ALIGNMENT)
            .map_err(|_| "failed to allocate ERST")?;

        log_info!(
            "xHCI {} ring allocation: command phys={:#012x} size={} event phys={:#012x} size={}",
            ident,
            command_ring.phys_addr(),
            command_ring.len(),
            event_ring.phys_addr(),
            event_ring.len()
        );

        controller.command_ring = Some(CommandRing::new(command_ring));
        controller.event_ring = Some(EventRing::new(event_ring, erst));
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
        let Some(ring) = controller.command_ring.as_mut() else {
            return Err("command ring not allocated");
        };

        let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
        unsafe {
            let crcr_ptr = op_base.add(0x18) as *mut u64;
            let value = (ring.phys_addr() & !0xF) | ring.cycle_bit_u64();
            core::ptr::write_volatile(crcr_ptr, value);
        }

        log_info!(
            "xHCI {} command ring configured: crcr={:#012x} (cycle {})",
            ident,
            ring.phys_addr(),
            if ring.cycle { 1 } else { 0 }
        );

        Ok(())
    }

    fn configure_event_ring(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let Some(event_ring) = controller.event_ring.as_mut() else {
            return Err("event ring not allocated");
        };

        unsafe {
            let entry_ptr = event_ring.table.virt_addr() as *mut u8;
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
            core::ptr::write_volatile(erstba_ptr, event_ring.table_phys_addr());
            core::ptr::write_volatile(erdp_ptr, event_ring.phys_addr() & !0xF);

            let iman = core::ptr::read_volatile(iman_ptr);
            core::ptr::write_volatile(iman_ptr, (iman & !IMAN_IP) | IMAN_IE);
        }

        event_ring.index = 0;
        event_ring.cycle = true;

        log_info!(
            "xHCI {} event ring configured: erst={:#012x} entries=1 erdp={:#012x}",
            ident,
            event_ring.table_phys_addr(),
            event_ring.phys_addr()
        );

        Ok(())
    }

    /// Allocate and publish the Device Context Base Address Array pointer.
    ///
    /// # Parameters
    /// * `controller` - Mutable controller state that will own the array.
    /// * `ident` - Identifier used in log messages.
    ///
    /// # Returns
    /// `Ok(())` on success or an allocation error string.
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

        let dcbaa_phys = {
            let dcbaa = controller.dcbaa.as_mut().unwrap();
            let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
            unsafe {
                let dcbaap_ptr = op_base.add(0x30) as *mut u64;
                core::ptr::write_volatile(dcbaap_ptr, dcbaa.phys_addr());
            }
            dcbaa.phys_addr()
        };

        self.initialise_slot_contexts(controller)?;

        log_info!(
            "xHCI {} DCBAA configured: ptr={:#012x} entries={}",
            ident,
            dcbaa_phys,
            controller.max_slots as usize + 1
        );

        Ok(())
    }

    /// Allocate placeholder device contexts for each slot and populate the
    /// DCBAA entries.
    fn initialise_slot_contexts(
        &self,
        controller: &mut XhciController,
    ) -> Result<(), &'static str> {
        controller.slot_contexts.clear();

        let entries = controller.max_slots as usize + 1;
        let dcbaa = controller.dcbaa.as_mut().unwrap();
        let base = dcbaa.virt_addr() as *mut u64;

        unsafe {
            // Entry 0 is reserved for scratchpad pointers. Leave it zeroed for
            // now; later lessons will populate it as needed.
            core::ptr::write_volatile(base, 0);
        }

        for slot in 1..entries {
            let ctx = DmaBuffer::allocate(DEVICE_CONTEXT_SIZE, DEVICE_CONTEXT_ALIGN)
                .map_err(|_| "failed to allocate device context")?;

            unsafe {
                let entry_ptr = base.add(slot);
                core::ptr::write_volatile(entry_ptr, ctx.phys_addr());
            }

            controller.slot_contexts.push(ctx);
        }

        Ok(())
    }

    /// Program CONFIG with the discovered slot count and enable slot
    /// operations via USBCMD.
    ///
    /// # Parameters
    /// * `controller` - Mutable controller state.
    /// * `ident` - Identifier used in log messages.
    ///
    /// # Returns
    /// `Ok(())` on success or an error string if the controller is incomplete.
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

    /// Bring the controller out of HALT by setting RUN/STOP and waiting for
    /// USBSTS.HCH to clear.
    ///
    /// # Parameters
    /// * `controller` - Mutable controller state.
    /// * `ident` - Identifier used in log messages.
    ///
    /// # Returns
    /// `Ok(())` when the controller indicates it left the halted state.
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

    /// Write a NOOP TRB into the command ring, ring the host doorbell, and wait
    /// for the completion event. This demonstrates the full command/doorbell
    /// path for students.
    ///
    /// # Parameters
    /// * `controller` - Mutable controller state.
    /// * `ident` - Identifier used in log messages.
    ///
    /// # Returns
    /// `Ok(())` when completion is observed, otherwise an error string.
    fn submit_noop(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        if !controller.controller_running {
            return Err("controller not running");
        }

        let Some(ring) = controller.command_ring.as_mut() else {
            return Err("command ring not allocated");
        };

        let index = ring.index % COMMAND_RING_TRBS;
        let trb_addr = ring.current_trb();
        let cycle_bit = ring.cycle_bit_u32();
        unsafe {
            // Emit a NOOP TRB; the zeroed parameter fields keep side effects at
            // bay while still exercising the doorbell path.
            core::ptr::write_volatile(trb_addr, 0);
            core::ptr::write_volatile(trb_addr.add(1), 0);
            core::ptr::write_volatile(trb_addr.add(2), 0);
            let control = cycle_bit | (TRB_TYPE_NOOP_COMMAND << 10);
            core::ptr::write_volatile(trb_addr.add(3), control);
        }

        ring.advance();

        unsafe {
            // Doorbells are 32-bit registers indexed by slot/endpoint; 0 hits
            // the host command doorbell.
            let doorbell_base =
                (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(doorbell_base.add(DOORBELL_HOST as usize), 0);
        }

        log_info!("xHCI {} submitted NOOP command (index={})", ident, index);
        controller.pending_commands += 1;
        controller.last_command_status = Some(self.poll_command_completion(controller, ident)?);
        Ok(())
    }

    /// Consume the next entry from the event ring waiting for a command
    /// completion TRB.
    ///
    /// # Parameters
    /// * `controller` - Mutable controller state.
    /// * `ident` - Identifier used in log messages.
    ///
    /// # Returns
    /// The completion code extracted from the event or an error string upon
    /// timeout/desynchronisation.
    fn poll_command_completion(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<u32, &'static str> {
        let Some(event_ring) = controller.event_ring.as_mut() else {
            return Err("event ring missing");
        };

        let runtime_base = (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
        unsafe {
            let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
            let iman_ptr = interrupter0.add(0x00) as *mut u32;
            let erdp_ptr = interrupter0.add(0x18) as *mut u64;

            if !self.poll_with_timeout(
                || (core::ptr::read_volatile(iman_ptr) & IMAN_IP) != 0,
                RUN_STOP_TIMEOUT,
            ) {
                return Err("command completion timeout");
            }

            let mut spins = 0usize;
            let mut trb_ptr;
            let expected_cycle = event_ring.cycle as u32;
            loop {
                trb_ptr = event_ring.current_trb();
                let control = core::ptr::read_volatile(trb_ptr.add(3));
                let cycle = control & TRB_CYCLE_BIT;
                if cycle == expected_cycle {
                    break;
                }
                if spins >= RUN_STOP_TIMEOUT {
                    return Err("event ring desync");
                }
                spins += 1;
                spin_loop();
            }

            let completion = core::ptr::read_volatile(trb_ptr.add(2));
            let control = core::ptr::read_volatile(trb_ptr.add(3));
            let code = (control & TRB_COMPLETION_CODE_MASK) >> 24;
            let trb_type = (control & TRB_TYPE_MASK) >> 10;

            event_ring.advance();

            controller.pending_commands = controller.pending_commands.saturating_sub(1);

            core::ptr::write_volatile(iman_ptr, core::ptr::read_volatile(iman_ptr) & !IMAN_IP);
            core::ptr::write_volatile(
                erdp_ptr,
                (event_ring.phys_addr() + (event_ring.index * TRB_SIZE) as u64) & !0xF,
            );

            log_info!(
                "xHCI {} completion: code={} type={} parameter={:#x}",
                ident,
                code,
                trb_type,
                completion
            );

            if trb_type != TRB_TYPE_COMMAND_COMPLETION {
                log_warn!(
                    "xHCI {} unexpected event type {} while awaiting command completion",
                    ident,
                    trb_type
                );
            }

            if code != TRB_COMPLETION_SUCCESS {
                log_warn!("xHCI {} command completed with status code {}", ident, code);
            }

            Ok(code)
        }
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
