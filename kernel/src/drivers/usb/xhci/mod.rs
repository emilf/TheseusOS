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
use core::slice;
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
const TRB_TYPE_ENABLE_SLOT: u32 = 0x09;
#[allow(dead_code)]
const TRB_TYPE_ADDRESS_DEVICE: u32 = 0x0B;
const ENDPOINT_TYPE_CONTROL: u32 = 4;
const DEFAULT_EP0_RING_TRBS: usize = 64;
const DEVICE_CONTEXT_ALIGN: usize = 64;
const MAX_ENDPOINT_CONTEXTS: usize = 32;
const DEVICE_CONTEXT_ENTRIES: usize = 1 + MAX_ENDPOINT_CONTEXTS;
const SLOT_CONTEXT_INDEX: usize = 0;
const DEFAULT_CONTROL_ENDPOINT: usize = 1;
const PORTSC_REGISTER_OFFSET: usize = 0x400;
const PORT_REGISTER_STRIDE: usize = 0x10;
const PORTSC_CCS: u32 = 1 << 0;
const PORTSC_PED: u32 = 1 << 1;
const PORTSC_OCA: u32 = 1 << 3;
const PORTSC_PR: u32 = 1 << 4;
const PORTSC_LINK_STATE_MASK: u32 = 0xF << 5;
const PORTSC_LINK_STATE_SHIFT: u32 = 5;
const PORTSC_POWER: u32 = 1 << 9;
const PORTSC_SPEED_MASK: u32 = 0xF << 10;
const PORTSC_SPEED_SHIFT: u32 = 10;
const SCRATCHPAD_ENTRY_SIZE: usize = 8;
const SCRATCHPAD_POINTER_ALIGN: usize = 64;
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
    hcsparams2: u32,
    scratchpad_count: u16,
    context_entry_size: usize,
    msi_enabled: AtomicBool,
    command_ring: Option<CommandRing>,
    event_ring: Option<EventRing>,
    input_context: Option<InputContext>,
    slots_enabled: bool,
    dcbaa: Option<DmaBuffer>,
    controller_running: bool,
    last_command_status: Option<CommandCompletion>,
    pending_commands: usize,
    slot_contexts: Vec<DmaBuffer>,
    scratchpad_table: Option<DmaBuffer>,
    scratchpad_buffers: Vec<DmaBuffer>,
    attached_port: Option<u8>,
    attached_speed: Option<PortSpeed>,
    control_context_ready: bool,
    active_slot: Option<u8>,
    control_transfer_ring: Option<TransferRing>,
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

/// DMA-backed input context used when programming slot and endpoint state.
struct InputContext {
    buffer: DmaBuffer,
}

/// Transfer ring backing storage for a device endpoint.
struct TransferRing {
    buffer: DmaBuffer,
    #[allow(dead_code)]
    enqueue_index: usize,
    cycle: bool,
    trb_count: usize,
}

impl InputContext {
    /// Create a new input context wrapper.
    fn new(buffer: DmaBuffer) -> Self {
        Self { buffer }
    }

    /// Physical address advertised to the controller when submitting commands.
    fn phys_addr(&self) -> u64 {
        self.buffer.phys_addr()
    }

    /// Zero the entire input context to a known state.
    fn zero(&mut self) {
        self.buffer.as_mut_slice().fill(0);
    }

    /// Mutable view of the input control context words.
    fn control_words_mut(&mut self, entry_size: usize) -> &mut [u32] {
        let words = entry_size / core::mem::size_of::<u32>();
        unsafe { slice::from_raw_parts_mut(self.buffer.virt_addr() as *mut u32, words) }
    }

    /// Mutable view of the slot context words.
    fn slot_words_mut(&mut self, entry_size: usize) -> &mut [u32] {
        let ptr = (self.buffer.virt_addr() + entry_size as u64) as *mut u32;
        let words = entry_size / core::mem::size_of::<u32>();
        unsafe { slice::from_raw_parts_mut(ptr, words) }
    }

    /// Mutable view of an endpoint context identified by `endpoint`.
    fn endpoint_words_mut(&mut self, entry_size: usize, endpoint: usize) -> Option<&mut [u32]> {
        if endpoint == SLOT_CONTEXT_INDEX || endpoint >= DEVICE_CONTEXT_ENTRIES {
            return None;
        }
        let offset = entry_size * (1 + endpoint);
        if offset + entry_size > self.buffer.len() {
            return None;
        }
        let ptr = (self.buffer.virt_addr() + offset as u64) as *mut u32;
        let words = entry_size / core::mem::size_of::<u32>();
        Some(unsafe { slice::from_raw_parts_mut(ptr, words) })
    }
}

/// Raw representation of a command TRB to be written to the command ring.
#[derive(Debug, Clone, Copy)]
struct RawCommandTrb {
    /// Combined parameter value (typically pointer).
    parameter: u64,
    /// Third dword of the TRB (status field).
    status: u32,
    /// Control dword (type bits, interrupter toggle, etc.).
    control: u32,
}

/// Summary of information extracted from a command completion event.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
struct CommandCompletion {
    /// Completion code reported by hardware.
    code: u32,
    /// TRB type encoded in the event.
    trb_type: u32,
    /// Full parameter payload echoed by the controller.
    parameter: u64,
    /// Raw status dword from the event TRB.
    status: u32,
    /// Raw control dword from the event TRB.
    control: u32,
    /// Slot ID associated with the command completion, when applicable.
    slot_id: Option<u8>,
}

/// Enumerated representation of the link state reported in PORTSC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum PortLinkState {
    U0,
    U1,
    U2,
    U3,
    Disabled,
    RxDetect,
    Inactive,
    Polling,
    Compliance,
    Recovery,
    HotReset,
    Resume,
    Reserved,
    Test,
    ResumePending,
    Unknown(u32),
}

impl PortLinkState {
    /// Convert the raw link state field into the strongly typed variant.
    fn from_raw(raw: u32) -> Self {
        match raw {
            0 => PortLinkState::U0,
            1 => PortLinkState::U1,
            2 => PortLinkState::U2,
            3 => PortLinkState::U3,
            4 => PortLinkState::Disabled,
            5 => PortLinkState::RxDetect,
            6 => PortLinkState::Inactive,
            7 => PortLinkState::Polling,
            8 => PortLinkState::Compliance,
            9 => PortLinkState::Recovery,
            10 => PortLinkState::HotReset,
            11 => PortLinkState::Resume,
            12 => PortLinkState::Reserved,
            13 => PortLinkState::Test,
            14 => PortLinkState::ResumePending,
            other => PortLinkState::Unknown(other),
        }
    }

    /// String label suitable for logging.
    fn as_str(&self) -> &'static str {
        match self {
            PortLinkState::U0 => "U0",
            PortLinkState::U1 => "U1",
            PortLinkState::U2 => "U2",
            PortLinkState::U3 => "U3",
            PortLinkState::Disabled => "Disabled",
            PortLinkState::RxDetect => "RxDetect",
            PortLinkState::Inactive => "Inactive",
            PortLinkState::Polling => "Polling",
            PortLinkState::Compliance => "Compliance",
            PortLinkState::Recovery => "Recovery",
            PortLinkState::HotReset => "HotReset",
            PortLinkState::Resume => "Resume",
            PortLinkState::Reserved => "Reserved",
            PortLinkState::Test => "Test",
            PortLinkState::ResumePending => "ResumePending",
            PortLinkState::Unknown(_) => "Unknown",
        }
    }
}

/// USB port speeds supported under the xHCI specification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum PortSpeed {
    Invalid,
    Full,
    Low,
    High,
    Super,
    SuperPlus,
    Reserved(u32),
}

impl PortSpeed {
    fn from_raw(raw: u32) -> Self {
        match raw {
            0 => PortSpeed::Invalid,
            1 => PortSpeed::Full,
            2 => PortSpeed::Low,
            3 => PortSpeed::High,
            4 => PortSpeed::Super,
            5 => PortSpeed::SuperPlus,
            other => PortSpeed::Reserved(other),
        }
    }

    fn as_str(&self) -> &'static str {
        match self {
            PortSpeed::Invalid => "invalid",
            PortSpeed::Full => "full",
            PortSpeed::Low => "low",
            PortSpeed::High => "high",
            PortSpeed::Super => "super",
            PortSpeed::SuperPlus => "super+",
            PortSpeed::Reserved(_) => "reserved",
        }
    }

    /// Raw encoding used in PORTSC and slot contexts.
    fn raw(&self) -> u32 {
        match self {
            PortSpeed::Invalid => 0,
            PortSpeed::Full => 1,
            PortSpeed::Low => 2,
            PortSpeed::High => 3,
            PortSpeed::Super => 4,
            PortSpeed::SuperPlus => 5,
            PortSpeed::Reserved(value) => *value,
        }
    }
}

/// Snapshot of a root hub port as reported through `PORTSC`.
#[derive(Debug, Clone, Copy)]
struct PortState {
    /// 1-based port index.
    index: usize,
    /// Raw `PORTSC` value.
    register: u32,
    connected: bool,
    enabled: bool,
    powered: bool,
    overcurrent: bool,
    resetting: bool,
    link_state: PortLinkState,
    speed: PortSpeed,
}

impl PortState {
    /// Construct a `PortState` from a `PORTSC` register snapshot.
    fn from_register(index: usize, portsc: u32) -> Self {
        let link_bits = (portsc & PORTSC_LINK_STATE_MASK) >> PORTSC_LINK_STATE_SHIFT;
        let speed_bits = (portsc & PORTSC_SPEED_MASK) >> PORTSC_SPEED_SHIFT;
        Self {
            index,
            register: portsc,
            connected: (portsc & PORTSC_CCS) != 0,
            enabled: (portsc & PORTSC_PED) != 0,
            powered: (portsc & PORTSC_POWER) != 0,
            overcurrent: (portsc & PORTSC_OCA) != 0,
            resetting: (portsc & PORTSC_PR) != 0,
            link_state: PortLinkState::from_raw(link_bits),
            speed: PortSpeed::from_raw(speed_bits),
        }
    }

    /// Human readable summary used in bring-up logs.
    fn summary(&self) -> String {
        let mut parts: Vec<String> = Vec::new();
        parts.push(if self.connected {
            "connected".into()
        } else {
            "disconnected".into()
        });
        if self.enabled {
            parts.push("enabled".into());
        }
        if self.powered {
            parts.push("powered".into());
        }
        if self.resetting {
            parts.push("reset".into());
        }
        if self.overcurrent {
            parts.push("overcurrent".into());
        }
        parts.push(format!("link={}", self.link_state.as_str()));
        if !matches!(self.speed, PortSpeed::Invalid | PortSpeed::Reserved(_)) {
            parts.push(format!("speed={}", self.speed.as_str()));
        }
        parts.push(format!("raw={:#010x}", self.register));
        parts.join(" ")
    }

    /// Route string to be programmed into the slot context.
    ///
    /// For root hub ports the route string is the 5-bit port index encoded in
    /// the least significant bits.
    fn route_string(&self) -> u32 {
        (self.index as u32) & 0xFFFFF
    }
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

impl TransferRing {
    fn new(buffer: DmaBuffer, trb_count: usize) -> Self {
        Self {
            buffer,
            enqueue_index: 0,
            cycle: true,
            trb_count,
        }
    }

    fn phys_addr(&self) -> u64 {
        self.buffer.phys_addr()
    }

    fn dequeue_pointer(&self) -> u64 {
        (self.buffer.phys_addr() & !0xF) | if self.cycle { 1 } else { 0 }
    }

    fn capacity(&self) -> usize {
        self.trb_count
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

/// Round `value` up to the nearest multiple of `align`.
///
/// # Parameters
/// * `value` - Quantity that must be aligned.
/// * `align` - Alignment boundary, which must be a power of two.
fn align_up(value: usize, align: usize) -> usize {
    debug_assert!(align.is_power_of_two());
    (value + align - 1) & !(align - 1)
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

    /// Decode the Max Scratchpad Buffers field from `HCSParams2`.
    ///
    /// The field spans `MaxScratchpadBuffersHi` (bits 27:31) and
    /// `MaxScratchpadBuffersLo` (bits 21:25). The combined value indicates the
    /// number of 4 KiB scratchpad buffers the controller expects.
    fn scratchpad_count_from_params(hcsparams2: u32) -> u16 {
        let low = (hcsparams2 >> 21) & 0x1F;
        let high = (hcsparams2 >> 27) & 0x1F;
        ((high << 5) | low) as u16
    }

    /// Determine the size of each context entry from `HCCParams1`.
    ///
    /// # Parameters
    /// * `hccparams1` - First Host Controller Capability Parameters register.
    ///
    /// # Returns
    /// `64` when 64-byte contexts are required, otherwise `32`.
    fn context_entry_size_from_params(hccparams1: u32) -> usize {
        if (hccparams1 & (1 << 2)) != 0 {
            64
        } else {
            32
        }
    }

    /// Compute the total size of a device context buffer for the controller.
    ///
    /// The xHCI specification defines `DeviceContextEntries` (slot context +
    /// endpoint contexts) with each entry sized according to the
    /// `Context Size` bit in `HCCParams1`. We ensure the allocation respects
    /// the required 64-byte alignment.
    fn device_context_bytes(&self, controller: &XhciController) -> usize {
        let raw = controller.context_entry_size * DEVICE_CONTEXT_ENTRIES;
        align_up(raw, DEVICE_CONTEXT_ALIGN)
    }

    /// Compute the total size of the input context buffer (control + device contexts).
    fn input_context_bytes(&self, controller: &XhciController) -> usize {
        let raw = controller.context_entry_size * (1 + DEVICE_CONTEXT_ENTRIES);
        align_up(raw, DEVICE_CONTEXT_ALIGN)
    }

    /// Determine the default control endpoint packet size for a given port speed.
    fn control_max_packet_size(speed: PortSpeed) -> u32 {
        match speed {
            PortSpeed::Low => 8,
            PortSpeed::Full | PortSpeed::High => 64,
            PortSpeed::Super => 512,
            PortSpeed::SuperPlus => 1024,
            PortSpeed::Invalid | PortSpeed::Reserved(_) => 8,
        }
    }

    /// Number of 32-bit words in a single context entry.
    fn context_word_count(controller: &XhciController) -> usize {
        controller.context_entry_size / core::mem::size_of::<u32>()
    }

    #[allow(dead_code)]
    /// Obtain a mutable pointer to the slot context for the given slot ID.
    ///
    /// # Safety
    /// The caller is responsible for interpreting and writing the slot context
    /// fields according to the xHCI specification. The returned pointer stays
    /// valid as long as the referenced `DmaBuffer` remains alive.
    fn slot_context_mut(
        &self,
        controller: &mut XhciController,
        slot_id: usize,
    ) -> Option<*mut u32> {
        if slot_id == 0 || slot_id > controller.slot_contexts.len() {
            return None;
        }
        let buffer = controller.slot_contexts.get_mut(slot_id - 1)?;
        Some(buffer.virt_addr() as *mut u32)
    }

    /// Obtain a mutable slice over the slot context words.
    fn slot_context_words_mut(
        &self,
        controller: &mut XhciController,
        slot_id: usize,
    ) -> Option<&mut [u32]> {
        let ptr = self.slot_context_mut(controller, slot_id)?;
        let words = Self::context_word_count(controller);
        Some(unsafe { slice::from_raw_parts_mut(ptr, words) })
    }

    /// Obtain a mutable pointer to an endpoint context within the device context.
    ///
    /// # Parameters
    /// * `slot_id` - Slot whose device context should be accessed (1-based).
    /// * `endpoint` - Endpoint index (1-31, with `DEFAULT_CONTROL_ENDPOINT` selecting the
    ///   default control endpoint).
    #[allow(dead_code)]
    fn endpoint_context_mut(
        &self,
        controller: &mut XhciController,
        slot_id: usize,
        endpoint: usize,
    ) -> Option<*mut u32> {
        if endpoint == SLOT_CONTEXT_INDEX || endpoint >= DEVICE_CONTEXT_ENTRIES {
            return None;
        }
        let buffer = controller.slot_contexts.get_mut(slot_id.checked_sub(1)?)?;
        let offset = endpoint * controller.context_entry_size;
        Some((buffer.virt_addr() + offset as u64) as *mut u32)
    }

    /// Obtain a mutable slice over an endpoint context for the provided slot.
    fn endpoint_context_words_mut(
        &self,
        controller: &mut XhciController,
        slot_id: usize,
        endpoint: usize,
    ) -> Option<&mut [u32]> {
        let ptr = self.endpoint_context_mut(controller, slot_id, endpoint)?;
        let words = Self::context_word_count(controller);
        Some(unsafe { slice::from_raw_parts_mut(ptr, words) })
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
            let hcsparams2 = core::ptr::read_volatile(base.add(0x08) as *const u32);
            let hccparams1 = core::ptr::read_volatile(base.add(0x10) as *const u32);
            let operational_offset = cap_length as u32;
            let runtime_offset = core::ptr::read_volatile(base.add(0x18) as *const u32) & !0b11;
            let doorbell_offset = core::ptr::read_volatile(base.add(0x14) as *const u32) & !0b11;
            let scratchpad_count = Self::scratchpad_count_from_params(hcsparams2);
            let context_entry_size = Self::context_entry_size_from_params(hccparams1);

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
                hcsparams2,
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
                hcsparams2,
                scratchpad_count,
                context_entry_size,
                msi_enabled: AtomicBool::new(false),
                command_ring: None,
                event_ring: None,
                input_context: None,
                slots_enabled: false,
                dcbaa: None,
                controller_running: false,
                last_command_status: None,
                pending_commands: 0,
                slot_contexts: Vec::new(),
                scratchpad_table: None,
                scratchpad_buffers: Vec::new(),
                attached_port: None,
                attached_speed: None,
                control_context_ready: false,
                active_slot: None,
                control_transfer_ring: None,
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
            let mut first_connected_port = None;
            match self.start_controller(controller_ref, &ident) {
                Ok(()) => {
                    first_connected_port = self.log_ports(&*controller_ref, &ident);
                }
                Err(err) => log_warn!("xHCI {}: run-state transition failed: {}", ident, err),
            }
            controller_ref.attached_port = first_connected_port.map(|state| state.index as u8);
            controller_ref.attached_speed = first_connected_port.map(|state| state.speed);
            if let Some(port_state) = first_connected_port {
                match self.prepare_default_control_context(controller_ref, &ident, port_state) {
                    Ok(()) => {
                        if let Err(err) = self.enable_device_slot(controller_ref, &ident) {
                            log_warn!(
                                "xHCI {}: enable slot failed after port {} preparation: {}",
                                ident,
                                port_state.index,
                                err
                            );
                        }
                        if let Err(err) = self.address_device_command(controller_ref, &ident) {
                            log_warn!(
                                "xHCI {}: address device failed in slot {}: {}",
                                ident,
                                controller_ref.active_slot.unwrap_or_default(),
                                err
                            );
                        }
                    }
                    Err(err) => {
                        log_warn!(
                            "xHCI {}: control endpoint scaffold failed for port {}: {}",
                            ident,
                            port_state.index,
                            err
                        );
                    }
                }
            } else {
                log_info!(
                    "xHCI {} awaiting device connection before programming contexts",
                    ident
                );
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
        self.allocate_input_context(controller, ident)?;
        Ok(())
    }

    /// Allocate a DMA ring buffer with the requested TRB capacity.
    fn allocate_ring(&self, trbs: usize, tag: &str) -> Result<DmaBuffer, &'static str> {
        let size = trbs * TRB_SIZE;
        DmaBuffer::allocate(size, RING_ALIGNMENT).map_err(|_| match tag {
            "command" => "failed to allocate command ring",
            "event" => "failed to allocate event ring",
            _ => "failed to allocate ring",
        })
    }

    /// Allocate and zero an input context for the controller.
    fn allocate_input_context(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        if controller.input_context.is_some() {
            return Ok(());
        }

        let size = self.input_context_bytes(controller);
        let buffer = DmaBuffer::allocate(size, DEVICE_CONTEXT_ALIGN)
            .map_err(|_| "failed to allocate input context")?;
        let mut ctx = InputContext::new(buffer);
        ctx.zero();

        log_info!(
            "xHCI {} input context allocated: phys={:#012x} bytes={}",
            ident,
            ctx.phys_addr(),
            size
        );

        controller.input_context = Some(ctx);
        Ok(())
    }

    /// Program CRCR with the command ring base and initial cycle state.
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

    /// Configure ERST, ERDP, and IMAN so interrupter 0 services the event ring.
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

        self.initialise_slot_contexts(controller, ident)?;
        self.configure_scratchpad_buffers(controller, ident)?;

        log_info!(
            "xHCI {} DCBAA configured: ptr={:#012x} entries={} ctx_entry={}",
            ident,
            dcbaa_phys,
            controller.max_slots as usize + 1,
            controller.context_entry_size
        );

        Ok(())
    }

    /// Allocate placeholder device contexts for each slot and populate the
    /// DCBAA entries.
    fn initialise_slot_contexts(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let entries = controller.max_slots as usize + 1;
        controller.slot_contexts.clear();
        controller.slot_contexts.reserve(entries - 1);
        let dcbaa = controller.dcbaa.as_mut().unwrap();
        let base = dcbaa.virt_addr() as *mut u64;
        let context_bytes = self.device_context_bytes(controller);

        unsafe {
            // Entry 0 is reserved for scratchpad pointers and will be populated
            // by `configure_scratchpad_buffers`.
            core::ptr::write_volatile(base, 0);
        }

        for slot in 1..entries {
            let ctx = DmaBuffer::allocate(context_bytes, DEVICE_CONTEXT_ALIGN)
                .map_err(|_| "failed to allocate device context")?;

            unsafe {
                let entry_ptr = base.add(slot);
                core::ptr::write_volatile(entry_ptr, ctx.phys_addr());
            }

            controller.slot_contexts.push(ctx);

            if let Some(words) = self.slot_context_words_mut(controller, slot) {
                words.fill(0);
            }
            if let Some(endpoint0) =
                self.endpoint_context_words_mut(controller, slot, DEFAULT_CONTROL_ENDPOINT)
            {
                endpoint0.fill(0);
            }
        }

        log_info!(
            "xHCI {} context arena: entry_size={} total_bytes={} per_slot={}",
            ident,
            controller.context_entry_size,
            context_bytes * (entries - 1),
            context_bytes
        );

        Ok(())
    }

    /// Populate the scratchpad pointer array and allocate per-buffer storage.
    ///
    /// # Parameters
    /// * `controller` - Mutable controller descriptor with an allocated DCBAA.
    /// * `ident` - Identifier string used for log messages.
    ///
    /// # Returns
    /// `Ok(())` when the controller state reflects the advertised scratchpad
    /// count or an error string when allocations fail.
    fn configure_scratchpad_buffers(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let count = controller.scratchpad_count as usize;
        if count == 0 {
            return Ok(());
        }

        let table_bytes = count * SCRATCHPAD_ENTRY_SIZE;
        let needs_table = controller
            .scratchpad_table
            .as_ref()
            .map_or(true, |table| table.len() != table_bytes);
        if needs_table {
            controller.scratchpad_table = Some(
                DmaBuffer::allocate(table_bytes, SCRATCHPAD_POINTER_ALIGN)
                    .map_err(|_| "failed to allocate scratchpad pointer table")?,
            );
        }

        if controller.scratchpad_buffers.len() != count {
            controller.scratchpad_buffers.clear();
            for _ in 0..count {
                let buffer = DmaBuffer::allocate(
                    crate::memory::PAGE_SIZE as usize,
                    crate::memory::PAGE_SIZE as usize,
                )
                .map_err(|_| "failed to allocate scratchpad buffer")?;
                controller.scratchpad_buffers.push(buffer);
            }
        }

        let table = controller.scratchpad_table.as_mut().unwrap();
        unsafe {
            let mut entry_ptr = table.virt_addr() as *mut u64;
            for scratchpad in controller.scratchpad_buffers.iter() {
                core::ptr::write_volatile(entry_ptr, scratchpad.phys_addr());
                entry_ptr = entry_ptr.add(1);
            }
        }

        let dcbaa = controller.dcbaa.as_mut().unwrap();
        unsafe {
            let dcbaa_entries = dcbaa.virt_addr() as *mut u64;
            core::ptr::write_volatile(dcbaa_entries, table.phys_addr());
        }

        log_info!(
            "xHCI {} scratchpad buffers: count={} table={:#012x}",
            ident,
            count,
            table.phys_addr()
        );

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

    /// Issue an arbitrary command TRB, ring the host doorbell, and wait for the
    /// completion event.
    fn issue_command(
        &self,
        controller: &mut XhciController,
        ident: &str,
        trb: RawCommandTrb,
    ) -> Result<(CommandCompletion, usize), &'static str> {
        if !controller.controller_running {
            return Err("controller not running");
        }

        let Some(ring) = controller.command_ring.as_mut() else {
            return Err("command ring not allocated");
        };

        let index = ring.index % COMMAND_RING_TRBS;
        let trb_ptr = ring.current_trb();
        unsafe {
            core::ptr::write_volatile(trb_ptr, trb.parameter as u32);
            core::ptr::write_volatile(trb_ptr.add(1), (trb.parameter >> 32) as u32);
            core::ptr::write_volatile(trb_ptr.add(2), trb.status);
            let control = ring.cycle_bit_u32() | trb.control;
            core::ptr::write_volatile(trb_ptr.add(3), control);
        }

        ring.advance();

        unsafe {
            let doorbell_base =
                (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(doorbell_base.add(DOORBELL_HOST as usize), 0);
        }

        controller.pending_commands += 1;
        let completion = self.poll_command_completion(controller, ident)?;
        controller.last_command_status = Some(completion);
        Ok((completion, index))
    }

    /// Write a NOOP TRB into the command ring, ring the host doorbell, and wait
    /// for the completion event. This demonstrates the full command/doorbell
    /// path for students.
    fn submit_noop(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let trb = RawCommandTrb {
            parameter: 0,
            status: 0,
            control: TRB_TYPE_NOOP_COMMAND << 10,
        };

        let (completion, index) = self.issue_command(controller, ident, trb)?;
        log_info!(
            "xHCI {} submitted NOOP command (index={}) -> completion code {}",
            ident,
            index,
            completion.code
        );
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
    /// A `CommandCompletion` describing the completion TRB or an error string if
    /// a timeout/desynchronisation occurs.
    fn poll_command_completion(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<CommandCompletion, &'static str> {
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

            let parameter_lo = core::ptr::read_volatile(trb_ptr);
            let parameter_hi = core::ptr::read_volatile(trb_ptr.add(1));
            let parameter = ((parameter_hi as u64) << 32) | parameter_lo as u64;
            let status = core::ptr::read_volatile(trb_ptr.add(2));
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
                parameter
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

            let slot_id = match ((control >> 24) & 0xFF) as u8 {
                0 => None,
                value => Some(value),
            };

            Ok(CommandCompletion {
                code,
                trb_type,
                parameter,
                status,
                control,
                slot_id,
            })
        }
    }

    /// Read the PORTSC register for each root port and return decoded states.
    fn collect_port_states(&self, controller: &XhciController) -> Vec<PortState> {
        let mut ports = Vec::new();
        if controller.max_ports == 0 {
            return ports;
        }

        unsafe {
            let op_base =
                (controller.virt_base + controller.operational_offset as u64) as *const u8;
            for port_index in 0..controller.max_ports as usize {
                let portsc_ptr = op_base
                    .add(PORTSC_REGISTER_OFFSET + (port_index * PORT_REGISTER_STRIDE))
                    as *const u32;
                let portsc = core::ptr::read_volatile(portsc_ptr);
                ports.push(PortState::from_register(port_index + 1, portsc));
            }
        }

        ports
    }

    /// Emit a human-readable summary of every root hub port and surface the first connected one.
    fn log_ports(&self, controller: &XhciController, ident: &str) -> Option<PortState> {
        let states = self.collect_port_states(controller);
        if states.is_empty() {
            log_info!("xHCI {} reports zero root ports", ident);
            return None;
        }

        let mut first_connected = None;
        for state in &states {
            log_info!("xHCI {} port{:02} {}", ident, state.index, state.summary());
            if first_connected.is_none() && state.connected && !state.overcurrent {
                first_connected = Some(*state);
            }
        }

        if let Some(state) = first_connected {
            log_info!(
                "xHCI {} first connected port{:02} speed={} link={}",
                ident,
                state.index,
                state.speed.as_str(),
                state.link_state.as_str()
            );
        } else {
            log_info!("xHCI {} no connected ports detected", ident);
        }

        first_connected
    }

    /// Prepare the input context for the default control endpoint on the selected port.
    fn prepare_default_control_context(
        &self,
        controller: &mut XhciController,
        ident: &str,
        port: PortState,
    ) -> Result<(), &'static str> {
        self.allocate_input_context(controller, ident)?;
        let Some(ctx) = controller.input_context.as_mut() else {
            return Err("input context unavailable");
        };

        ctx.zero();

        {
            let input_control = ctx.control_words_mut(controller.context_entry_size);
            input_control.fill(0);
            if input_control.len() > 1 {
                input_control[1] = (1 << SLOT_CONTEXT_INDEX) | (1 << DEFAULT_CONTROL_ENDPOINT);
            }
        }

        let route_string = port.route_string();
        let slot_context_entries = DEFAULT_CONTROL_ENDPOINT as u32;
        let port_number = port.index as u32;
        let speed_code = port.speed.raw();

        let transfer_ring = if let Some(ring) = controller.control_transfer_ring.as_mut() {
            ring
        } else {
            let size = DEFAULT_EP0_RING_TRBS * TRB_SIZE;
            let buffer = DmaBuffer::allocate(size, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate control transfer ring")?;
            let ring = TransferRing::new(buffer, DEFAULT_EP0_RING_TRBS);
            controller.control_transfer_ring = Some(ring);
            let ring_ref = controller.control_transfer_ring.as_mut().unwrap();
            log_info!(
                "xHCI {} control transfer ring allocated: phys={:#012x} trbs={}",
                ident,
                ring_ref.phys_addr(),
                ring_ref.capacity()
            );
            ring_ref
        };

        {
            let slot_words = ctx.slot_words_mut(controller.context_entry_size);
            slot_words.fill(0);
            if !slot_words.is_empty() {
                slot_words[0] = route_string | (speed_code << 20);
            }
            if slot_words.len() > 1 {
                let context_entry = (slot_context_entries & 0x1F) << 27;
                let port_field = (port_number & 0xFF) << 16;
                slot_words[1] = context_entry | port_field;
            }
        }

        {
            let ep0_words = ctx
                .endpoint_words_mut(controller.context_entry_size, DEFAULT_CONTROL_ENDPOINT)
                .ok_or("failed to map endpoint context")?;
            self.populate_endpoint_zero(ep0_words, port.speed, transfer_ring.dequeue_pointer());
        }

        controller.control_context_ready = true;

        log_info!(
            "xHCI {} input context primed for port{:02}: route={:#x} speed={} ictx={:#012x}",
            ident,
            port.index,
            route_string,
            port.speed.as_str(),
            ctx.phys_addr()
        );

        Ok(())
    }

    /// Initialise the default control endpoint context template used during address assignment.
    fn populate_endpoint_zero(
        &self,
        endpoint_words: &mut [u32],
        speed: PortSpeed,
        dequeue_pointer: u64,
    ) {
        endpoint_words.fill(0);

        let error_recovery_count: u32 = 3;
        let max_packet = Self::control_max_packet_size(speed);

        if endpoint_words.len() > 1 {
            endpoint_words[1] =
                (ENDPOINT_TYPE_CONTROL << 3) | (error_recovery_count << 1) | (max_packet << 16);
        }
        if endpoint_words.len() > 2 {
            endpoint_words[2] = dequeue_pointer as u32;
        }
        if endpoint_words.len() > 3 {
            endpoint_words[3] = (dequeue_pointer >> 32) as u32;
        }
        if endpoint_words.len() > 7 {
            endpoint_words[7] = max_packet as u32;
        }
    }

    /// Issue an Enable Slot command and capture the returned slot identifier.
    fn enable_device_slot(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<u8, &'static str> {
        if let Some(slot) = controller.active_slot {
            return Ok(slot);
        }
        if controller.attached_port.is_none() {
            return Err("no attached port to enable slot");
        }

        let trb = RawCommandTrb {
            parameter: 0,
            status: 0,
            control: TRB_TYPE_ENABLE_SLOT << 10,
        };

        let (completion, index) = self.issue_command(controller, ident, trb)?;
        let Some(slot_id) = completion.slot_id else {
            return Err("enable slot completion missing slot id");
        };

        controller.active_slot = Some(slot_id);

        log_info!(
            "xHCI {} enable-slot command index={} -> slot {}",
            ident,
            index,
            slot_id
        );

        Ok(slot_id)
    }

    /// Issue the Address Device command for the active slot using the prepared input context.
    fn address_device_command(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let slot_id = controller
            .active_slot
            .ok_or("address device requested without slot")?;
        if !controller.control_context_ready {
            return Err("control context not initialised");
        }

        let ctx = controller
            .input_context
            .as_ref()
            .ok_or("input context missing")?;

        let ics_flag = if controller.context_entry_size == 64 {
            1
        } else {
            0
        }; // ICS bit
        let parameter = ctx.phys_addr() & !0xF;
        let trb = RawCommandTrb {
            parameter,
            status: ics_flag,
            control: (TRB_TYPE_ADDRESS_DEVICE << 10) | ((slot_id as u32) << 24),
        };

        let (completion, index) = self.issue_command(controller, ident, trb)?;
        if completion.code != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} address-device (slot {}) returned completion code {}",
                ident,
                slot_id,
                completion.code
            );
        } else {
            log_info!(
                "xHCI {} address-device success (slot {} index={})",
                ident,
                slot_id,
                index
            );
        }

        Ok(())
    }

    /// Decode USBSTS into a pipe-separated list of active flags.
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

    /// Emit a structured summary of the controller's capability registers.
    fn log_capabilities(
        &self,
        ident: &str,
        hci_version: u16,
        hcsparams1: u32,
        hcsparams2: u32,
        hccparams1: u32,
        cap_length: u8,
        runtime_offset: u32,
        doorbell_offset: u32,
    ) {
        let context_size = Self::context_entry_size_from_params(hccparams1);
        let port_count = (hcsparams1 >> 24) & 0xFF;
        let max_slots = (hcsparams1 & 0xFF) + 1;
        let scratchpads = Self::scratchpad_count_from_params(hcsparams2);
        log_info!(
            "xHCI {} HCI v{}.{} slots={} ports={} context_size={} scratchpads={}",
            ident,
            hci_version >> 8,
            hci_version & 0xFF,
            max_slots,
            port_count,
            context_size,
            scratchpads
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

    /// Capture the current controller state from the operational register block.
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
