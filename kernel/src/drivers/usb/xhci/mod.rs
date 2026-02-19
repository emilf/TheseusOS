//! xHCI host controller driver for the teaching kernel.
//!
//! The goal of this module is educational: it walks through the steps required
//! to take ownership of a modern USB 3 controller on x86-64 while exercising
//! Rust's safety features. The code demonstrates how to map MMIO regions,
//! interpret capability structures, set up command/event rings, transition the
//! controller to `RUN`, and service the event ring via MSI/MSI-X when available.
//! In addition to the core bring-up, the driver implements a minimal xHCI slot
//! + endpoint configuration path sufficient to enumerate QEMU's virtual USB HID
//! boot keyboard and deliver interrupt-driven key events through the shared
//! `input::keyboard` hub.

use alloc::string::ToString;
use alloc::{format, string::String, vec::Vec};
use core::convert::TryFrom;
use core::hint::spin_loop;
use core::slice;
use core::sync::atomic::{AtomicBool, Ordering};
use spin::Mutex;

use crate::acpi;
use crate::config;
use crate::drivers::manager::driver_manager;
use crate::drivers::pci;
use crate::drivers::traits::{Device, DeviceClass, DeviceId, DeviceResource, Driver};
use crate::interrupts::{get_apic_base, read_apic_register, XHCI_MSI_VECTOR};
use crate::memory::dma::DmaBuffer;
use crate::memory::{
    current_pml4_phys, map_range_with_policy, phys_to_virt_pa, PageTable, PTE_GLOBAL, PTE_NO_EXEC,
    PTE_PCD, PTE_PRESENT, PTE_PWT, PTE_WRITABLE,
};
use crate::{log_debug, log_info, log_trace, log_warn};

mod rings;
use rings::{CommandRing, EventRing, TransferRing, RING_RECYCLER};
mod hid;
mod trb;

pub(crate) use trb::{
    RawCommandTrb, RawEventTrb, TRB_CHAIN, TRB_COMPLETION_SUCCESS, TRB_CYCLE_BIT, TRB_DIR_IN,
    TRB_IDT, TRB_IOC, TRB_LINK_TOGGLE_CYCLE, TRB_TYPE_ADDRESS_DEVICE, TRB_TYPE_COMMAND_COMPLETION,
    TRB_TYPE_CONFIGURE_ENDPOINT, TRB_TYPE_DATA_STAGE, TRB_TYPE_ENABLE_SLOT,
    TRB_TYPE_EVALUATE_CONTEXT, TRB_TYPE_LINK, TRB_TYPE_NOOP_COMMAND, TRB_TYPE_NORMAL,
    TRB_TYPE_PORT_STATUS_CHANGE_EVENT, TRB_TYPE_SET_TR_DEQUEUE_POINTER, TRB_TYPE_SETUP_STAGE,
    TRB_TYPE_STATUS_STAGE, TRB_TYPE_TRANSFER_EVENT,
 };

/// Base virtual address used when mapping xHCI MMIO windows into the kernel.
const XHCI_MMIO_WINDOW_BASE: u64 = 0xFFFF_FFB0_0000_0000;
const XHCI_MIN_MMIO: usize = 0x2000;
const USBCMD_RUN_STOP: u32 = 1 << 0;
const USBCMD_HCRST: u32 = 1 << 1;
const USBCMD_INTERRUPT_ENABLE: u32 = 1 << 2;
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
const DOORBELL_HOST: u32 = 0;
const IMAN_IP: u32 = 1 << 0;
const ENDPOINT_TYPE_CONTROL: u32 = 4;
const ENDPOINT_TYPE_INTERRUPT_IN: u32 = 7;
const DEFAULT_EP0_RING_TRBS: usize = 64;
const MSIX_ENTRY_SIZE: usize = 16;
const MSI_MESSAGE_ADDR_BASE: u64 = 0xFEE0_0000;

/// Request a DMA buffer sized for a TRB ring, reusing cached allocations when available.
fn request_ring_buffer(size: usize, tag: &str) -> Result<DmaBuffer, &'static str> {
    {
        let mut recycler = RING_RECYCLER.lock();
        if let Some(mut buffer) = recycler.acquire(size) {
            buffer.as_mut_slice().fill(0);
            log_debug!(
                "xHCI ring recycler reused {} buffer: size={} bytes phys={:#012x}",
                tag,
                size,
                buffer.phys_addr()
            );
            return Ok(buffer);
        }
    }

    DmaBuffer::allocate(size, RING_ALIGNMENT).map_err(|_| match tag {
        "command" => "failed to allocate command ring",
        "event" => "failed to allocate event ring",
        _ => "failed to allocate ring",
    })
}

const HID_INTERRUPT_RING_TRBS: usize = 32;
const DEVICE_DESCRIPTOR_LENGTH: usize = 18;
const CONFIG_DESCRIPTOR_LENGTH: usize = 9;
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
const PORTSC_CSC: u32 = 1 << 17;
const PORTSC_PEC: u32 = 1 << 18;
const PORTSC_WRC: u32 = 1 << 19;
const PORTSC_OCC: u32 = 1 << 20;
const PORTSC_PRC: u32 = 1 << 21;
const PORTSC_PLC: u32 = 1 << 22;
const PORTSC_CEC: u32 = 1 << 23;
const SCRATCHPAD_ENTRY_SIZE: usize = 8;
const SCRATCHPAD_POINTER_ALIGN: usize = 64;
static MMIO_MAPPING_LOCK: Mutex<()> = Mutex::new(());
static XHCI_DRIVER: XhciDriver = XhciDriver;
static CONTROLLERS: Mutex<Vec<XhciController>> = Mutex::new(Vec::new());
static MSI_DEFERRED_RUNTIME_SERVICE: AtomicBool = AtomicBool::new(false);

#[allow(dead_code)]
/// Aggregated state for a discovered controller.
struct XhciController {
    ident: String,
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
    msi_vector: Option<u8>,
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
    /// Transfer ring backing the default control endpoint (EP0).
    control_transfer_ring: Option<TransferRing>,
    ep0_descriptor_buffer: Option<DmaBuffer>,
    /// Cached transfer event for the default control endpoint while we wait for completion handling.
    pending_ep0_event: Option<RawEventTrb>,
    /// Cached interrupt endpoint discovered in the HID boot interface (if any).
    hid_boot_keyboard: Option<HidEndpoint>,
    /// Active configuration value selected via `SET_CONFIGURATION`, if any.
    hid_configuration_value: Option<u8>,
    /// Transfer ring dedicated to the HID interrupt endpoint (if armed).
    hid_interrupt_ring: Option<TransferRing>,
    /// DMA buffer that receives the raw HID reports from the interrupt pipe.
    hid_report_buffer: Option<DmaBuffer>,
    /// Running count of HID interrupt reports successfully captured.
    hid_reports_seen: u64,
    /// Cached copy of the last HID report observed (boot protocol -> 8 bytes).
    hid_last_report: [u8; 8],
    /// Length of the cached report (≤ 8 bytes for boot keyboards).
    hid_last_report_len: usize,
    /// Debug latch used to avoid spamming logs when MSI appears stuck pending.
    msi_pending_logged: bool,
    /// Debug latch used to report when IMAN.IE is unexpectedly cleared.
    msi_disarmed_logged: bool,
    /// When set, we have queued an MSI/MSI-X self-test NOOP and are waiting for its completion
    /// to be observed via the interrupt-driven event ring path.
    msix_self_test_pending: bool,
    /// Port-change notification latched for thread-context handling.
    pending_port_change: Option<u8>,
}

/// Minimal description of the interrupt endpoint exposed by a HID boot keyboard.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
struct HidEndpoint {
    interface_number: u8,
    endpoint_address: u8,
    max_packet_size: u16,
    interval: u8,
    /// xHCI endpoint identifier derived from the USB endpoint address.
    endpoint_id: u8,
}

/// Human-readable snapshot of an xHCI controller used for diagnostics.
#[derive(Clone, Debug)]
pub struct ControllerDiagnostics {
    pub ident: String,
    pub phys_base: u64,
    pub mmio_length: usize,
    pub max_ports: u8,
    pub max_slots: u8,
    pub controller_running: bool,
    pub slots_enabled: bool,
    pub active_slot: Option<u8>,
    pub attached_port: Option<u8>,
    pub attached_speed: Option<String>,
    pub msi_enabled: bool,
    pub msi_vector: Option<u8>,
    pub interrupt_enabled: bool,
    pub interrupt_pending: bool,
    pub iman_raw: u32,
    pub hid_keyboard: Option<HidEndpointSummary>,
    pub ports: Vec<PortDiagnostics>,
}

/// Diagnostic snapshot of a root-port line state.
#[derive(Clone, Debug)]
pub struct PortDiagnostics {
    pub index: u8,
    pub connected: bool,
    pub enabled: bool,
    pub powered: bool,
    pub overcurrent: bool,
    pub speed: String,
    pub link_state: String,
    pub raw: u32,
}

/// Summary of the HID boot keyboard endpoint discovered during enumeration.
#[derive(Clone, Debug)]
pub struct HidEndpointSummary {
    pub interface_number: u8,
    pub endpoint_address: u8,
    pub max_packet_size: u16,
    pub interval: u8,
    pub endpoint_id: u8,
    pub reports_seen: u64,
}

/// DMA-backed input context used when programming slot and endpoint state.
struct InputContext {
    buffer: DmaBuffer,
}

/// Encoded view of a USB control SETUP packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct UsbControlSetup {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
}

impl UsbControlSetup {
    /// Construct the canonical GET_DESCRIPTOR(Device) setup packet.
    #[allow(dead_code)]
    fn device_descriptor() -> Self {
        Self::get_descriptor(1, 0, DEVICE_DESCRIPTOR_LENGTH as u16)
    }

    /// Encode the packet as the immediate data payload used by setup TRBs.
    fn to_immediate(self) -> u64 {
        (self.request_type as u64)
            | ((self.request as u64) << 8)
            | ((self.value as u64) << 16)
            | ((self.index as u64) << 32)
            | ((self.length as u64) << 48)
    }

    /// Construct a GET_DESCRIPTOR request for the specified descriptor type and index.
    fn get_descriptor(descriptor_type: u8, descriptor_index: u8, length: u16) -> Self {
        Self {
            request_type: 0x80,
            request: 6,
            value: ((descriptor_type as u16) << 8) | descriptor_index as u16,
            index: 0,
            length,
        }
    }

    /// Construct a SET_CONFIGURATION request selecting the provided configuration value.
    fn set_configuration(configuration_value: u8) -> Self {
        Self {
            request_type: 0x00, // Host-to-device | Standard | Device
            request: 0x09,      // SET_CONFIGURATION
            value: configuration_value as u16,
            index: 0,
            length: 0,
        }
    }

    /// Construct a SET_PROTOCOL request for HID interfaces.
    fn set_protocol(interface: u8, boot_protocol: bool) -> Self {
        Self {
            request_type: 0x21, // Class | Interface | Host-to-device
            request: 0x0B,      // SET_PROTOCOL
            value: if boot_protocol { 0 } else { 1 },
            index: interface as u16,
            length: 0,
        }
    }
}

/// Direction flag used when staging data/status TRBs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
enum TransferDir {
    In,
    Out,
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

    /// Compute the `ROUTE_STRING` field that will be written into a slot context.
    ///
    /// Downstream hubs populate the five 4-bit nibbles with the traversed port
    /// path. Root ports sit directly beneath the host controller, so the field
    /// must be zeroed per xHCI §6.2.2.1.
    fn route_string(&self) -> u32 {
        0
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

/// Concrete implementation of the generic `Driver` trait for xHCI controllers.
///
/// The driver is responsible for mapping the controller MMIO aperture,
/// staging command/event rings, negotiating firmware ownership, and eventually
/// handling device enumeration. All of its state lives in `XhciController`
/// instances tracked within `CONTROLLERS`.
/// Concrete implementation of the USB host-controller driver.
///
/// The `XhciDriver` registers itself with the driver manager and handles the
/// PCI discovery/path for each xHCI controller that shows up during boot.
struct XhciDriver;

impl XhciDriver {
    /// Read the local APIC ID of the current processor so MSI/MSI-X targets the
    /// correct destination.
    fn local_apic_id() -> u8 {
        unsafe {
            let apic_base = get_apic_base();
            (read_apic_register(apic_base, 0x20) >> 24) as u8
        }
    }

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
        log_info!(
            "xHCI {} initialising controller (phys={:#x})",
            ident,
            phys_base
        );
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
                ident: ident.clone(),
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
                msi_vector: None,
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
                ep0_descriptor_buffer: None,
                pending_ep0_event: None,
                hid_boot_keyboard: None,
                hid_configuration_value: None,
                hid_interrupt_ring: None,
                hid_report_buffer: None,
                hid_reports_seen: 0,
                hid_last_report: [0; 8],
                hid_last_report_len: 0,
                msi_pending_logged: false,
                msi_disarmed_logged: false,
                msix_self_test_pending: false,
                pending_port_change: None,
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
            // With static resources staged we can bring the controller out of
            // the halted state and exercise the command ring.
            let mut first_connected_port = None;
            match self.start_controller(controller_ref, &ident) {
                Ok(()) => {
                    first_connected_port = self.log_ports(&*controller_ref, &ident);
                }
                Err(err) => log_warn!("xHCI {}: run-state transition failed: {}", ident, err),
            }
            if let Some(mut port_state) = first_connected_port {
                match self.reset_port(controller_ref, &ident, port_state) {
                    Ok(updated) => {
                        port_state = updated;
                        controller_ref.attached_port = Some(port_state.index as u8);
                        controller_ref.attached_speed = Some(port_state.speed);
                    }
                    Err(err) => {
                        log_warn!(
                            "xHCI {}: port{} reset failed: {}",
                            ident,
                            port_state.index,
                            err
                        );
                        controller_ref.attached_port = Some(port_state.index as u8);
                        controller_ref.attached_speed = Some(port_state.speed);
                    }
                }

                match self.prepare_default_control_context(controller_ref, &ident, port_state) {
                    Ok(()) => {
                        if let Err(err) = self.enable_device_slot(controller_ref, &ident) {
                            log_warn!(
                                "xHCI {}: enable slot failed after port {} preparation: {}",
                                ident,
                                port_state.index,
                                err
                            );
                        } else if let Err(err) =
                            self.enumerate_default_control_endpoint(controller_ref, &ident)
                        {
                            log_warn!(
                                "xHCI {}: default control enumeration failed in slot {}: {}",
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
        log_info!("xHCI {} entering try_enable_msi", Self::pci_display(dev));
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

        if let Err(err) = pci::enable_mmio_busmaster(pci_info, &platform.pci_config_regions) {
            log_warn!("xHCI {}: unable to enable PCI MEM/BUSMASTER: {}", ident, err);
        }

        let local_apic_id = Self::local_apic_id();
        log_info!(
            "xHCI {} attempting MSI setup (local APIC {:02x})",
            ident,
            local_apic_id
        );

        if let Some(irq) = pci_info.interrupt_line() {
            log_debug!(
                "xHCI {:02x}:{:02x}.{} legacy IRQ {} will be masked when IOAPIC plumbing lands",
                bus,
                device,
                function,
                irq
            );
        }

        log_info!(
            "xHCI {:02x}:{:02x}.{} capabilities: msi={} ptr={:?} msix={} ptr={:?}",
            bus,
            device,
            function,
            pci_info.capabilities.msi,
            pci_info.capabilities.msi_pointer,
            pci_info.capabilities.msix,
            pci_info.capabilities.msix_pointer
        );

        let mut controllers = CONTROLLERS.lock();
        let mut controller_match: Option<&mut XhciController> = None;
        if let Some(raw) = dev.driver_data {
            let target = raw as *const XhciController;
            for ctrl in controllers.iter_mut() {
                let ctrl_ptr = ctrl as *mut XhciController as *const XhciController;
                if ctrl_ptr == target {
                    controller_match = Some(ctrl);
                    break;
                }
            }
        }
        if controller_match.is_none() {
            controller_match = controllers.iter_mut().find(|ctrl| ctrl.ident == ident);
        }
        let Some(controller) = controller_match else {
            log_warn!("xHCI {}: controller record missing during MSI setup", ident);
            return;
        };

        let msi_active = controller.msi_enabled.load(Ordering::Relaxed);
        let vector_configured = controller.msi_vector.is_some();
        log_debug!(
            "xHCI {} initial MSI enabled flag={} vector={:?}",
            ident,
            msi_active,
            controller.msi_vector
        );

        if msi_active && vector_configured {
            log_debug!("xHCI {}: MSI already active", ident);
            return;
        }

        if pci_info.capabilities.msi {
            match pci::enable_msi(
                pci_info,
                &platform.pci_config_regions,
                local_apic_id,
                XHCI_MSI_VECTOR,
            ) {
                Ok(()) => {
                    controller.msi_enabled.store(true, Ordering::Release);
                    controller.msi_vector = Some(XHCI_MSI_VECTOR);
                    log_info!(
                        "xHCI {:02x}:{:02x}.{} routed via MSI vector 0x{:02x}",
                        bus,
                        device,
                        function,
                        XHCI_MSI_VECTOR
                    );
                    // Re-assert IMAN.IE after MSI is enabled so QEMU (and real
                    // hardware) sees the interrupter mapping as active.
                    unsafe {
                        let runtime_base =
                            (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
                        let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
                        let iman_ptr = interrupter0.add(0x00) as *mut u32;
                        let iman = core::ptr::read_volatile(iman_ptr);
                        // IMAN.IP is RW1C; do not set it when we only mean to (re-)enable IE.
                        core::ptr::write_volatile(iman_ptr, iman | IMAN_IE);
                    }
                    return;
                }
                Err(err) => {
                    log_warn!(
                        "xHCI {:02x}:{:02x}.{}: enabling MSI failed: {}",
                        bus,
                        device,
                        function,
                        err
                    );
                }
            }
        }

        if pci_info.capabilities.msix {
            match self.configure_msix(
                controller,
                pci_info,
                &platform.pci_config_regions,
                bus,
                device,
                function,
            ) {
                Ok(()) => {
                    log_info!(
                        "xHCI {} routed via MSI-X vector 0x{:02x}",
                        ident,
                        XHCI_MSI_VECTOR
                    );
                    // Re-assert IMAN.IE after MSI-X is enabled so the PCI shim
                    // can "use" the vector for interrupter 0.
                    unsafe {
                        let runtime_base =
                            (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
                        let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
                        let iman_ptr = interrupter0.add(0x00) as *mut u32;
                        let iman = core::ptr::read_volatile(iman_ptr);
                        // IMAN.IP is RW1C; do not set it when we only mean to (re-)enable IE.
                        core::ptr::write_volatile(iman_ptr, iman | IMAN_IE);
                    }
                    return;
                }
                Err(err) => {
                    log_warn!(
                        "xHCI {:02x}:{:02x}.{}: enabling MSI-X failed: {}",
                        bus,
                        device,
                        function,
                        err
                    );
                }
            }
        } else {
            log_info!(
                "xHCI {:02x}:{:02x}.{}: no MSI/MSI-X capability; controller will use polling",
                bus,
                device,
                function
            );
        }
    }

    fn configure_msix(
        &self,
        controller: &mut XhciController,
        pci_info: &pci::PciDeviceInfo,
        regions: &[acpi::PciConfigRegion],
        bus: u8,
        device: u8,
        function: u8,
    ) -> Result<(), &'static str> {
        let msix = pci::msix_capability(pci_info, regions)
            .map_err(|_| "MSI-X capability not accessible")?;
        if msix.table_size == 0 {
            return Err("MSI-X table advertises zero entries");
        }

        log_debug!(
            "xHCI {:02x}:{:02x}.{} MSI-X table bir={} offset=0x{:05x} entries={}",
            bus,
            device,
            function,
            msix.table_bir,
            msix.table_offset,
            msix.table_size
        );

        let bar_index = msix.table_bir as usize;
        if bar_index >= pci_info.bars.len() {
            return Err("MSI-X table BAR index out of range");
        }

        let bar_phys = match &pci_info.bars[bar_index] {
            pci::PciBar::Memory32 { base, .. } | pci::PciBar::Memory64 { base, .. } => *base,
            _ => return Err("MSI-X table located in non-memory BAR"),
        };

        let table_phys = bar_phys + msix.table_offset as u64;
        let table_bytes = msix.table_size as u64 * MSIX_ENTRY_SIZE as u64;
        if table_bytes == 0 {
            return Err("MSI-X table size calculation underflowed");
        }

        let controller_range_end = controller.phys_base + controller.mmio_length as u64;
        let table_virt = if table_phys >= controller.phys_base
            && table_phys + table_bytes <= controller_range_end
        {
            controller.virt_base + (table_phys - controller.phys_base)
        } else {
            self.map_mmio(table_phys, table_bytes as usize)
        };

        log_debug!(
            "xHCI {:02x}:{:02x}.{} MSI-X table mapped: phys=0x{:012x} bytes={} virt=0x{:012x}",
            bus,
            device,
            function,
            table_phys,
            table_bytes,
            table_virt
        );

        let control_word = msix.control();
        if (control_word & (1 << 14)) == 0 {
            msix.write_control(control_word | (1 << 14));
        }

        let entry_ptr = table_virt as *mut u8;
        let local_apic_id = Self::local_apic_id();

        unsafe {
            let vector_control_ptr = entry_ptr.add(12) as *mut u32;
            let mut entry_control = core::ptr::read_volatile(vector_control_ptr);
            if (entry_control & 1) == 0 {
                entry_control |= 1;
                core::ptr::write_volatile(vector_control_ptr, entry_control);
            }

            core::ptr::write_volatile(
                entry_ptr as *mut u64,
                MSI_MESSAGE_ADDR_BASE | ((local_apic_id as u64) << 12),
            );
            core::ptr::write_volatile(entry_ptr.add(8) as *mut u32, XHCI_MSI_VECTOR as u32);

            entry_control &= !1;
            core::ptr::write_volatile(vector_control_ptr, entry_control);
        }

        let mut control_word = msix.control();
        control_word |= 1 << 15; // Enable MSI-X
        control_word &= !(1 << 14); // Clear function mask
        msix.write_control(control_word);
        let final_control = msix.control();
        log_debug!(
            "xHCI {:02x}:{:02x}.{} MSI-X control final={:#06x}",
            bus,
            device,
            function,
            final_control
        );

        controller.msi_enabled.store(true, Ordering::Release);
        controller.msi_vector = Some(XHCI_MSI_VECTOR);

        log_info!(
            "xHCI {:02x}:{:02x}.{} routed via MSI-X vector 0x{:02x}",
            bus,
            device,
            function,
            XHCI_MSI_VECTOR
        );

        Ok(())
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
        request_ring_buffer(size, tag)
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
            let entry_ptr = event_ring.table_virt_addr() as *mut u8;
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
            // Clear Event Handler Busy (EHB) using the RW1C semantics (QEMU and
            // most real controllers latch EHB across reset).
            core::ptr::write_volatile(erdp_ptr, (event_ring.phys_addr() & !0xF) | (1 << 3));

            // Enable interrupts and clear any stale IP bit using RW1C semantics.
            core::ptr::write_volatile(iman_ptr, IMAN_IE | IMAN_IP);
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
            let mut updated = false;

            if cmd & USBCMD_INTERRUPT_ENABLE == 0 {
                cmd |= USBCMD_INTERRUPT_ENABLE;
                updated = true;
            }

            if !controller.controller_running && (cmd & USBCMD_RUN_STOP) == 0 {
                cmd |= USBCMD_RUN_STOP;
                updated = true;
            }

            if updated {
                core::ptr::write_volatile(cmd_ptr, cmd);
            }

            if !controller.controller_running {
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
        }

        if !controller.controller_running {
            controller.controller_running = true;
            log_info!("xHCI {} controller running", ident);
        }
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
        // Program the next slot in the command ring with the raw TRB payload
        // before toggling the producer cycle (xHCI §4.5.1).
        let trb_ptr = ring.current_trb();
        unsafe {
            core::ptr::write_volatile(trb_ptr, trb.parameter as u32);
            core::ptr::write_volatile(trb_ptr.add(1), (trb.parameter >> 32) as u32);
            core::ptr::write_volatile(trb_ptr.add(2), trb.status);
            let control = ring.cycle_bit_u32() | trb.control;
            core::ptr::write_volatile(trb_ptr.add(3), control);
        }

        ring.advance();

        // Ring the host controller doorbell so it notices the newly produced TRB.
        unsafe {
            let doorbell_base =
                (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(doorbell_base.add(DOORBELL_HOST as usize), 0);
        }

        controller.pending_commands += 1;
        // Block until the matching command-completion event arrives and cache
        // the last completion for higher-level diagnostics.
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

    /// Enqueue a NOOP command TRB and ring the host doorbell, but do **not**
    /// wait for the completion.
    ///
    /// This is used as a minimal interrupt-delivery self-test: if MSI/MSI-X is
    /// configured correctly, the command-completion event will arrive on the
    /// event ring and be observed by the interrupt handler.
    fn submit_noop_async(&self, controller: &mut XhciController) -> Result<(), &'static str> {
        let Some(ring) = controller.command_ring.as_mut() else {
            return Err("command ring not allocated");
        };

        let trb = RawCommandTrb {
            parameter: 0,
            status: 0,
            control: TRB_TYPE_NOOP_COMMAND << 10,
        };

        // Program the next command-ring entry and ring the host doorbell.
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
            let doorbell_base = (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(doorbell_base.add(DOORBELL_HOST as usize), 0);
        }

        controller.pending_commands += 1;
        Ok(())
    }

    /// Update an endpoint's dequeue pointer via the `SET_TR_DEQUEUE_POINTER` command.
    ///
    /// This helper is primarily useful while debugging ring desynchronisation issues
    /// and mirrors the sequence described in xHCI §4.6.9. The target endpoint must be
    /// stopped before issuing the command.
    #[allow(dead_code)]
    fn set_tr_dequeue_pointer(
        &self,
        controller: &mut XhciController,
        ident: &str,
        slot_id: u8,
        endpoint_id: u8,
        dequeue: u64,
        dcs: bool,
    ) -> Result<(), &'static str> {
        let parameter = dequeue & !0xF;
        let status = if dcs { 1 } else { 0 };
        let control = (TRB_TYPE_SET_TR_DEQUEUE_POINTER << 10)
            | ((endpoint_id as u32) << 16)
            | ((slot_id as u32) << 24);
        let trb = RawCommandTrb {
            parameter,
            status,
            control,
        };
        let (completion, index) = self.issue_command(controller, ident, trb)?;
        if completion.code != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} set-tr-dequeue-pointer slot {} ep {} returned code {}",
                ident,
                slot_id,
                endpoint_id,
                completion.code
            );
        } else {
            log_debug!(
                "xHCI {} set-tr-dequeue-pointer slot {} ep {} (index={} code={})",
                ident,
                slot_id,
                endpoint_id,
                index,
                completion.code
            );
        }
        Ok(())
    }

    /// Issue a Configure Endpoint command for the active slot using the current input context.
    ///
    /// The Input Control Context's Add/Drop masks must be primed by the caller.
    /// On success the controller copies the queued endpoint contexts into the
    /// operational device context and transitions the slot into the configured state.
    fn configure_endpoint_command(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let slot_id = controller
            .active_slot
            .ok_or("configure endpoint requested without slot")?;
        let ctx = controller
            .input_context
            .as_ref()
            .ok_or("input context missing for configure-endpoint")?;

        let parameter = ctx.phys_addr() & !0xF;
        let ics_flag = if controller.context_entry_size == 64 {
            1
        } else {
            0
        };
        let trb = RawCommandTrb {
            parameter,
            status: ics_flag,
            control: (TRB_TYPE_CONFIGURE_ENDPOINT << 10) | ((slot_id as u32) << 24),
        };

        let (completion, index) = self.issue_command(controller, ident, trb)?;
        if completion.code != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} configure-endpoint (slot {}) completion code {}",
                ident,
                slot_id,
                completion.code
            );
            return Err("configure endpoint failed");
        }

        log_info!(
            "xHCI {} configure-endpoint success (slot {} index={})",
            ident,
            slot_id,
            index
        );

        Ok(())
    }

    /// Consume the next TRB from the event ring without interpreting it.
    fn fetch_event_trb(
        &self,
        controller: &mut XhciController,
        ident: &str,
        timeout: usize,
    ) -> Result<RawEventTrb, &'static str> {
        let Some(event_ring) = controller.event_ring.as_mut() else {
            return Err("event ring missing");
        };
        let runtime_base = (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
        let (parameter, status, control) = unsafe {
            let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
            let iman_ptr = interrupter0.add(0x00) as *mut u32;
            let erdp_ptr = interrupter0.add(0x18) as *mut u64;

            // Wait for the controller to flag an interrupt-pending condition
            // before attempting to consume a TRB from the event ring.
            if !self.poll_with_timeout(
                || (core::ptr::read_volatile(iman_ptr) & IMAN_IP) != 0,
                timeout,
            ) {
                return Err("event ring timeout");
            }

            let expected_cycle = event_ring.cycle as u32;
            let trb_ptr = event_ring.current_trb();
            if !self.poll_with_timeout(
                || (core::ptr::read_volatile(trb_ptr.add(3)) & TRB_CYCLE_BIT) == expected_cycle,
                timeout,
            ) {
                let control = core::ptr::read_volatile(trb_ptr.add(3));
                let iman_snapshot = core::ptr::read_volatile(iman_ptr);
                let op_base =
                    (controller.virt_base + controller.operational_offset as u64) as *const u32;
                let usbsts = core::ptr::read_volatile(op_base.add((0x04 / 4) as usize));
                let parameter_lo = core::ptr::read_volatile(trb_ptr);
                let parameter_hi = core::ptr::read_volatile(trb_ptr.add(1));
                let parameter = ((parameter_hi as u64) << 32) | (parameter_lo as u64 & 0xFFFF_FFFF);
                let status_snapshot = core::ptr::read_volatile(trb_ptr.add(2));
                if config::USB_XHCI_EVENT_RING_DIAGNOSTICS {
                    log_warn!(
                        "xHCI {} event ring cycle mismatch (expected={} observed={:#x} index={} cycle={} iman={:#x} param={:#x} status={:#x})",
                        ident,
                        expected_cycle,
                        control,
                        event_ring.index,
                        event_ring.cycle,
                        iman_snapshot,
                        parameter,
                        status_snapshot
                    );
                    log_warn!(
                        "xHCI {} USBSTS snapshot during mismatch: {:#010x}",
                        ident,
                        usbsts
                    );
                    let buffer = event_ring
                        .buffer
                        .as_ref()
                        .expect("event ring buffer not initialised");
                    for sample in 0..8 {
                        let sample_ptr =
                            (buffer.virt_addr() + (sample * TRB_SIZE) as u64) as *const u32;
                        let sample_parameter_lo = core::ptr::read_volatile(sample_ptr);
                        let sample_parameter_hi = core::ptr::read_volatile(sample_ptr.add(1));
                        let sample_status = core::ptr::read_volatile(sample_ptr.add(2));
                        let sample_control = core::ptr::read_volatile(sample_ptr.add(3));
                        let sample_parameter = ((sample_parameter_hi as u64) << 32)
                            | (sample_parameter_lo as u64 & 0xFFFF_FFFF);
                        log_warn!(
                            "xHCI {} event ring entry {} snapshot: control={:#x} status={:#x} parameter={:#x}",
                            ident,
                            sample,
                            sample_control,
                            sample_status,
                            sample_parameter
                        );
                    }
                } else {
                    log_warn!(
                        "xHCI {} event ring did not produce an entry before timeout (expected_cycle={} control={:#x} iman={:#x} usbsts={:#x})",
                        ident,
                        expected_cycle,
                        control,
                        iman_snapshot,
                        usbsts
                    );
                }
                return Err("event ring desync");
            }

            let parameter_lo = core::ptr::read_volatile(trb_ptr);
            let parameter_hi = core::ptr::read_volatile(trb_ptr.add(1));
            let status = core::ptr::read_volatile(trb_ptr.add(2));
            let control = core::ptr::read_volatile(trb_ptr.add(3));
            let parameter = ((parameter_hi as u64) << 32) | parameter_lo as u64;

            event_ring.advance();

            let iman = core::ptr::read_volatile(iman_ptr);
            core::ptr::write_volatile(iman_ptr, iman | IMAN_IP);
            let erdp = (event_ring.phys_addr() + (event_ring.index * TRB_SIZE) as u64) & !0xF;
            // Per xHCI §4.9.4 software must set the Event Handler Busy (EHB) bit
            // whenever it advances ERDP so the controller can post new events.
            core::ptr::write_volatile(erdp_ptr, erdp | (1 << 3));
            log_debug!(
                "xHCI {} ERDP -> {:#x} (event_index={} cycle={})",
                ident,
                erdp,
                event_ring.index,
                if event_ring.cycle { 1 } else { 0 }
            );

            (parameter, status, control)
        };

        Ok(RawEventTrb {
            parameter,
            status,
            control,
        })
    }

    /// Attempt to consume the next event TRB without blocking.
    ///
    /// Returns `Ok(Some(event))` when the ring held an entry, `Ok(None)` if the ring is idle,
    /// or an error when the controller has not been initialised fully.
    fn try_fetch_event_trb(
        &self,
        controller: &mut XhciController,
    ) -> Result<Option<RawEventTrb>, &'static str> {
        let Some(event_ring) = controller.event_ring.as_mut() else {
            return Err("event ring missing");
        };

        unsafe {
            let runtime_base = (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
            let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
            let iman_ptr = interrupter0.add(0x00) as *mut u32;
            let erdp_ptr = interrupter0.add(0x18) as *mut u64;

            let expected_cycle = event_ring.cycle as u32;
            let trb_ptr = event_ring.current_trb();
            let control = core::ptr::read_volatile(trb_ptr.add(3));

            if (control & TRB_CYCLE_BIT) != expected_cycle {
                // This is the expected "ring is empty" signal: the consumer
                // cycle bit doesn't match the producer's cycle yet.
                let iman = core::ptr::read_volatile(iman_ptr);
                if iman & IMAN_IP != 0 {
                    core::ptr::write_volatile(iman_ptr, iman | IMAN_IP);
                }
                return Ok(None);
            }

            let parameter_lo = core::ptr::read_volatile(trb_ptr);
            let parameter_hi = core::ptr::read_volatile(trb_ptr.add(1));
            let status = core::ptr::read_volatile(trb_ptr.add(2));
            let parameter = ((parameter_hi as u64) << 32) | parameter_lo as u64;

            event_ring.advance();

            let iman = core::ptr::read_volatile(iman_ptr);
            core::ptr::write_volatile(iman_ptr, iman | IMAN_IP);

            let erdp = (event_ring.phys_addr() + (event_ring.index * TRB_SIZE) as u64) & !0xF;
            core::ptr::write_volatile(erdp_ptr, erdp | (1 << 3));

            log_trace!(
                "xHCI {} fetched event TRB: parameter={:#x} status={:#x} control={:#x}",
                controller.ident,
                parameter,
                status,
                control
            );

            Ok(Some(RawEventTrb {
                parameter,
                status,
                control,
            }))
        }
    }

    /// Handle a transfer event by caching EP0 completions for synchronous consumers.
    ///
    /// The default control pipe is driven synchronously during early bring-up, so
    /// we stash the latest completion and let the waiting code pull it once the
    /// status stage has been observed.
    fn handle_transfer_event(
        &self,
        controller: &mut XhciController,
        ident: &str,
        event: &RawEventTrb,
    ) -> bool {
        let slot_id = event.slot_id();
        let endpoint_id = event.endpoint_id();
        let code = event.completion_code();
        let residual = event.residual_length();

        log_info!(
            "xHCI {} transfer event: slot={} ep={} code={} residual={}",
            ident,
            slot_id,
            endpoint_id,
            code,
            residual
        );

        let Some(active_slot) = controller.active_slot else {
            log_warn!(
                "xHCI {} transfer event dropped (no active slot) parameter={:#x}",
                ident,
                event.parameter
            );
            return false;
        };

        if slot_id != active_slot {
            log_debug!(
                "xHCI {} transfer event for slot {} ignored while active slot is {}",
                ident,
                slot_id,
                active_slot
            );
            return false;
        }

        if endpoint_id == DEFAULT_CONTROL_ENDPOINT as u8 {
            if controller.pending_ep0_event.is_some() {
                log_warn!(
                    "xHCI {} overriding pending EP0 event; previous completion will be dropped",
                    ident
                );
            }

            controller.pending_ep0_event = Some(*event);
            return false;
        }

        if let Some(hid) = controller.hid_boot_keyboard {
            if endpoint_id == hid.endpoint_id {
                return self.handle_hid_report_event(controller, ident, event, hid);
            }
        }

        log_debug!(
            "xHCI {} transfer event for endpoint {} ignored (unsupported)",
            ident,
            endpoint_id
        );
        false
    }

    /// Process a HID interrupt transfer completion, emit diagnostics, and re-arm the pipe.
    ///
    /// Returns `true` when the caller should break out of the current polling loop so the
    /// CPU can drop back into `hlt` before the next completion arrives. This avoids spinning
    /// through back-to-back events when QEMU immediately satisfies the rearmed TRB.
    fn handle_hid_report_event(
        &self,
        controller: &mut XhciController,
        ident: &str,
        event: &RawEventTrb,
        hid: HidEndpoint,
    ) -> bool {
        let prev_len = controller.hid_last_report_len;
        let mut prev_bytes = [0u8; 8];
        let copy_prev = core::cmp::min(prev_len, prev_bytes.len());
        prev_bytes[..copy_prev].copy_from_slice(&controller.hid_last_report[..copy_prev]);

        let code = event.completion_code();
        if code != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} HID transfer completion code {} (slot={} residual={})",
                ident,
                code,
                event.slot_id(),
                event.residual_length()
            );
        }

        let requested = core::cmp::min(
            controller
                .hid_report_buffer
                .as_ref()
                .map(|buffer| buffer.len())
                .unwrap_or(0) as u32,
            hid.max_packet_size as u32,
        );

        if requested == 0 {
            log_warn!(
                "xHCI {} HID report buffer unavailable (requested length zero); skipping rearm",
                ident
            );
            return false;
        }

        let buffer = match controller.hid_report_buffer.as_mut() {
            Some(buffer) => buffer,
            None => {
                log_warn!(
                    "xHCI {} HID report buffer missing during interrupt completion",
                    ident
                );
                return false;
            }
        };

        let actual_bytes =
            requested.saturating_sub(core::cmp::min(event.residual_length(), requested)) as usize;
        let slice = buffer.as_mut_slice();
        let report_len = core::cmp::min(actual_bytes, slice.len()).min(8);
        let mut report = [0u8; 8];
        report[..report_len].copy_from_slice(&slice[..report_len]);

        let copy_len = core::cmp::min(report_len, prev_len);
        let changed = prev_len != report_len
            || prev_bytes[..copy_len] != report[..copy_len]
            || prev_len > report_len && prev_bytes[report_len..prev_len].iter().any(|&b| b != 0);

        let dump = if report_len == 0 {
            "<empty>".to_string()
        } else {
            report[..report_len]
                .iter()
                .map(|byte| format!("{:02x}", byte))
                .collect::<Vec<_>>()
                .join(" ")
        };

        controller.hid_last_report.fill(0);
        controller.hid_last_report[..report_len].copy_from_slice(&report[..report_len]);
        controller.hid_last_report_len = report_len;

        if changed {
            let sequence = controller.hid_reports_seen;
            controller.hid_reports_seen = controller.hid_reports_seen.saturating_add(1);
            log_info!(
                "xHCI {} HID report #{} ({} bytes): {}",
                ident,
                sequence,
                report_len,
                dump
            );
            hid::emit_keyboard_events(ident, &prev_bytes, prev_len, &report[..report_len]);
        } else {
            log_trace!("xHCI {} HID report unchanged ({} bytes)", ident, report_len);
        }

        // Reset the capture buffer to a known state before re-queuing.
        for byte in slice.iter_mut().take(requested as usize) {
            *byte = 0;
        }

        let ring = match controller.hid_interrupt_ring.as_mut() {
            Some(ring) => ring,
            None => {
                log_warn!(
                    "xHCI {} HID interrupt ring missing while attempting to re-arm endpoint",
                    ident
                );
                return false;
            }
        };
        let start_index = ring.enqueue_index;
        ring.enqueue(
            buffer.phys_addr(),
            requested,
            // Direction is implied by the endpoint context; do not set the
            // Setup/Status-stage DIR bit on a Normal TRB.
            (TRB_TYPE_NORMAL << 10) | TRB_IOC,
        );
        log_debug!(
            "xHCI {} HID ring re-armed: start={} next={} cycle={}",
            ident,
            start_index,
            ring.enqueue_index,
            if ring.cycle { 1 } else { 0 }
        );

        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

        unsafe {
            let doorbell = (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(
                doorbell.add(event.slot_id() as usize),
                hid.endpoint_id as u32,
            );
        }

        // Break out of the polling loop so the CPU can `hlt` before the controller posts the
        // next completion. This keeps the runtime trace and polling cadence tamer under QEMU.
        true
    }

    // HID key transition decoding lives in `xhci::hid`.

    /// Read the current PORTSC value for a specific port.
    fn read_port_state(&self, controller: &XhciController, port_id: usize) -> Option<PortState> {
        if port_id == 0 || port_id > controller.max_ports as usize {
            return None;
        }

        unsafe {
            let op_base = (controller.virt_base + controller.operational_offset as u64) as *const u8;
            let portsc_ptr = op_base
                .add(PORTSC_REGISTER_OFFSET + ((port_id - 1) * PORT_REGISTER_STRIDE))
                as *const u32;
            let portsc = core::ptr::read_volatile(portsc_ptr);
            Some(PortState::from_register(port_id, portsc))
        }
    }

    /// Clear the RW1C change bits for the given port so future notifications can fire.
    fn acknowledge_port_change(&self, controller: &XhciController, port_id: usize, portsc: u32) {
        unsafe {
            let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
            let portsc_ptr = op_base
                .add(PORTSC_REGISTER_OFFSET + ((port_id - 1) * PORT_REGISTER_STRIDE))
                as *mut u32;
            let clear_mask = PORTSC_CSC
                | PORTSC_PEC
                | PORTSC_WRC
                | PORTSC_OCC
                | PORTSC_PRC
                | PORTSC_PLC
                | PORTSC_CEC;
            core::ptr::write_volatile(portsc_ptr, portsc | clear_mask);
        }
    }

    /// Handle a port-status-change event by latching the port ID for thread-context handling.
    fn handle_port_status_change_event(
        &self,
        controller: &mut XhciController,
        ident: &str,
        event: &RawEventTrb,
    ) {
        let Some(port_id) = event.port_id() else {
            log_warn!(
                "xHCI {} PSC event missing port id (parameter={:#x} status={:#x})",
                ident,
                event.parameter,
                event.status
            );
            return;
        };

        let Some(state) = self.read_port_state(controller, port_id as usize) else {
            log_warn!(
                "xHCI {} PSC for port {} ignored (out of range, max={})",
                ident,
                port_id,
                controller.max_ports
            );
            return;
        };

        log_info!(
            "xHCI {} port-status-change: port{:02} {}",
            ident,
            state.index,
            state.summary()
        );
        self.acknowledge_port_change(controller, state.index, state.register);

        controller.pending_port_change = Some(state.index as u8);
        // Ensure the idle loop will revisit runtime polling so heavy work (reset/enumeration)
        // runs outside interrupt context.
        MSI_DEFERRED_RUNTIME_SERVICE.store(true, Ordering::Release);
    }

    /// Service any latched port changes in thread context, triggering enumeration for new devices.
    fn service_pending_port_change(&self, controller: &mut XhciController, ident: &str) {
        let Some(port_id) = controller.pending_port_change.take() else {
            return;
        };

        let Some(state) = self.read_port_state(controller, port_id as usize) else {
            log_warn!(
                "xHCI {} pending port{:02} change could not be read (max={})",
                ident,
                port_id,
                controller.max_ports
            );
            return;
        };

        if !state.connected || state.overcurrent {
            log_info!(
                "xHCI {} port{:02} now disconnected/overcurrent; skipping enumeration",
                ident,
                state.index
            );
            return;
        }

        if controller.active_slot.is_some() {
            log_info!(
                "xHCI {} port{:02} connected but slot {} already active; hotplug handling deferred until multi-slot support exists",
                ident,
                state.index,
                controller.active_slot.unwrap()
            );
            return;
        }

        match self.reset_port(controller, ident, state) {
            Ok(updated) => {
                controller.attached_port = Some(updated.index as u8);
                controller.attached_speed = Some(updated.speed);
                log_info!(
                    "xHCI {} port{:02} reset after PSC -> {}",
                    ident,
                    updated.index,
                    updated.summary()
                );
                if let Err(err) = self.prepare_default_control_context(controller, ident, updated) {
                    log_warn!(
                        "xHCI {} failed to prepare control context for port{:02}: {}",
                        ident,
                        updated.index,
                        err
                    );
                    return;
                }
                if let Err(err) = self.enable_device_slot(controller, ident) {
                    log_warn!(
                        "xHCI {} enable-slot failed after port{:02} change: {}",
                        ident,
                        updated.index,
                        err
                    );
                    return;
                }
                if let Err(err) = self.enumerate_default_control_endpoint(controller, ident) {
                    log_warn!(
                        "xHCI {} enumeration failed after port{:02} change: {}",
                        ident,
                        updated.index,
                        err
                    );
                }
            }
            Err(err) => log_warn!(
                "xHCI {} port{:02} reset failed after PSC: {}",
                ident,
                state.index,
                err
            ),
        }
    }

    /// Drain pending runtime events for a controller without blocking.
    fn poll_runtime_events_for_controller(&self, controller: &mut XhciController) {
        let ident = controller.ident.clone();
        loop {
            let event = match self.try_fetch_event_trb(controller) {
                Ok(Some(event)) => event,
                Ok(None) => break,
                Err(err) => {
                    log_warn!("xHCI {} runtime poll failed: {}", ident, err);
                    break;
                }
            };

            match event.trb_type() {
                TRB_TYPE_TRANSFER_EVENT => {
                    if self.handle_transfer_event(controller, &ident, &event) {
                        break;
                    }
                }
                TRB_TYPE_COMMAND_COMPLETION => {
                    controller.pending_commands = controller.pending_commands.saturating_sub(1);
                    let slot_id = match event.slot_id() {
                        0 => None,
                        value => Some(value),
                    };
                    let completion = CommandCompletion {
                        code: event.completion_code(),
                        trb_type: event.trb_type(),
                        parameter: event.parameter,
                        status: event.status,
                        control: event.control,
                        slot_id,
                    };
                    controller.last_command_status = Some(completion);
                    if controller.msix_self_test_pending {
                        log_info!(
                            "xHCI {} MSI/MSI-X self-test: observed command completion via runtime event ring",
                            ident
                        );
                        controller.msix_self_test_pending = false;
                    }
                }
                TRB_TYPE_PORT_STATUS_CHANGE_EVENT => {
                    self.handle_port_status_change_event(controller, &ident, &event);
                }
                other => log_debug!("xHCI {} runtime event ignored: type={}", ident, other),
            }
        }
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
        loop {
            let event = self.fetch_event_trb(controller, ident, RUN_STOP_TIMEOUT)?;
            let trb_type = event.trb_type();
            let code = event.completion_code();

            if trb_type == TRB_TYPE_COMMAND_COMPLETION {
                controller.pending_commands = controller.pending_commands.saturating_sub(1);

                log_info!(
                    "xHCI {} completion: code={} type={} parameter={:#x}",
                    ident,
                    code,
                    trb_type,
                    event.parameter
                );

                if code != TRB_COMPLETION_SUCCESS {
                    log_warn!("xHCI {} command completed with status code {}", ident, code);
                }

                let slot_id = match event.slot_id() {
                    0 => None,
                    value => Some(value),
                };

                return Ok(CommandCompletion {
                    code,
                    trb_type,
                    parameter: event.parameter,
                    status: event.status,
                    control: event.control,
                    slot_id,
                });
            }

            if trb_type == TRB_TYPE_TRANSFER_EVENT {
                let _ = self.handle_transfer_event(controller, ident, &event);
                continue;
            }

            if trb_type == TRB_TYPE_PORT_STATUS_CHANGE_EVENT {
                self.handle_port_status_change_event(controller, ident, &event);
                continue;
            }
            log_warn!(
                "xHCI {} unexpected event type {} while awaiting command completion",
                ident,
                trb_type
            );
        }
    }

    /// Block until a queued EP0 transfer reports completion, caching unrelated events.
    fn wait_for_ep0_transfer_event(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<RawEventTrb, &'static str> {
        loop {
            if let Some(event) = controller.pending_ep0_event.take() {
                return Ok(event);
            }

            let event = self.fetch_event_trb(controller, ident, RUN_STOP_TIMEOUT * 3)?;
            let trb_type = event.trb_type();
            let code = event.completion_code();

            if trb_type == TRB_TYPE_TRANSFER_EVENT {
                let _ = self.handle_transfer_event(controller, ident, &event);
                continue;
            }

            if trb_type == TRB_TYPE_COMMAND_COMPLETION {
                controller.pending_commands = controller.pending_commands.saturating_sub(1);
                let slot_id = match event.slot_id() {
                    0 => None,
                    value => Some(value),
                };

                let completion = CommandCompletion {
                    code,
                    trb_type,
                    parameter: event.parameter,
                    status: event.status,
                    control: event.control,
                    slot_id,
                };
                controller.last_command_status = Some(completion);

                log_info!(
                    "xHCI {} command completion observed while awaiting EP0 transfer: code={} parameter={:#x}",
                    ident,
                    code,
                    event.parameter
                );

                if code != TRB_COMPLETION_SUCCESS {
                    log_warn!(
                        "xHCI {} command completion reported status code {}",
                        ident,
                        code
                    );
                }

                continue;
            }

            if trb_type == TRB_TYPE_PORT_STATUS_CHANGE_EVENT {
                self.handle_port_status_change_event(controller, ident, &event);
                continue;
            }
            log_warn!(
                "xHCI {} unexpected event type {} while awaiting EP0 transfer",
                ident,
                trb_type
            );
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

    /// Issue a port reset to transition the link into U0 and enable the port.
    fn reset_port(
        &self,
        controller: &mut XhciController,
        ident: &str,
        port: PortState,
    ) -> Result<PortState, &'static str> {
        unsafe {
            let op_base = (controller.virt_base + controller.operational_offset as u64) as *mut u8;
            let offset =
                PORTSC_REGISTER_OFFSET + ((port.index.saturating_sub(1)) * PORT_REGISTER_STRIDE);
            let portsc_ptr = op_base.add(offset) as *mut u32;

            let rw1c_mask = PORTSC_CSC
                | PORTSC_PEC
                | PORTSC_WRC
                | PORTSC_OCC
                | PORTSC_PRC
                | PORTSC_PLC
                | PORTSC_CEC;
            let mut value = core::ptr::read_volatile(portsc_ptr);
            value &= !rw1c_mask;
            value |= PORTSC_PR;
            core::ptr::write_volatile(portsc_ptr, value);

            if !self.poll_with_timeout(
                || (core::ptr::read_volatile(portsc_ptr) & PORTSC_PR) == 0,
                RUN_STOP_TIMEOUT,
            ) {
                return Err("port reset timed out (PR bit stuck)");
            }

            if !self.poll_with_timeout(
                || {
                    let value = core::ptr::read_volatile(portsc_ptr);
                    let enabled = (value & PORTSC_PED) != 0;
                    let link_bits = (value & PORTSC_LINK_STATE_MASK) >> PORTSC_LINK_STATE_SHIFT;
                    enabled && matches!(PortLinkState::from_raw(link_bits), PortLinkState::U0)
                },
                RUN_STOP_TIMEOUT,
            ) {
                log_warn!(
                    "xHCI {} port{:02} reset completed without enabling link state U0",
                    ident,
                    port.index
                );
            }

            let updated = core::ptr::read_volatile(portsc_ptr);
            let state = PortState::from_register(port.index, updated);
            let clear_mask = PORTSC_CSC
                | PORTSC_PEC
                | PORTSC_WRC
                | PORTSC_OCC
                | PORTSC_PRC
                | PORTSC_PLC
                | PORTSC_CEC;
            let mut current = core::ptr::read_volatile(portsc_ptr);
            current &= !rw1c_mask;
            current |= clear_mask;
            core::ptr::write_volatile(portsc_ptr, current);
            log_info!(
                "xHCI {} port{:02} reset -> {}",
                ident,
                state.index,
                state.summary()
            );
            Ok(state)
        }
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
            // The transfer ring already exists; reuse it so hardware retains
            // ownership of the same dequeue pointer and cycle state.
            ring
        } else {
            // Allocate the transfer ring on first use so that EP0 has space for
            // setup/data/status TRBs once the slot is addressed.
            let size = DEFAULT_EP0_RING_TRBS * TRB_SIZE;
            let buffer = DmaBuffer::allocate(size, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate control transfer ring")?;
            let ring = TransferRing::new(buffer, DEFAULT_EP0_RING_TRBS);
            controller.control_transfer_ring = Some(ring);
            let ring_ref = controller.control_transfer_ring.as_mut().unwrap();
            log_info!(
                "xHCI {} control transfer ring allocated: phys={:#012x} usable_trbs={} total_trbs={}",
                ident,
                ring_ref.phys_addr(),
                ring_ref.capacity(),
                ring_ref.total_trbs()
            );
            ring_ref
        };

        if controller.ep0_descriptor_buffer.is_none() {
            let buffer = DmaBuffer::allocate(64, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate descriptor buffer")?;
            controller.ep0_descriptor_buffer = Some(buffer);
        }

        {
            let slot_words = ctx.slot_words_mut(controller.context_entry_size);
            slot_words.fill(0);
            if !slot_words.is_empty() {
                // ROUTE STRING (bits 0-19) + SPEED (bits 20-23).
                // Root ports report a zeroed route string while the speed field
                // captures the negotiated link rate.
                slot_words[0] = route_string | (speed_code << 20);
            }
            if slot_words.len() > 1 {
                // CONTEXT ENTRIES (bits 27-31) + ROOT HUB PORT (bits 16-23)
                let context_field = (slot_context_entries & 0x1F) << 27;
                let port_field = (port_number & 0xFF) << 16;
                slot_words[1] = context_field | port_field;
            }
            log_debug!(
                "xHCI {} slot context template: d0={:#010x} d1={:#010x}",
                ident,
                slot_words.get(0).copied().unwrap_or(0),
                slot_words.get(1).copied().unwrap_or(0)
            );
        }

        {
            let ep0_words = ctx
                .endpoint_words_mut(controller.context_entry_size, DEFAULT_CONTROL_ENDPOINT)
                .ok_or("failed to map endpoint context")?;
            self.populate_endpoint_zero(
                ep0_words,
                port.speed,
                transfer_ring.dequeue_pointer(),
                transfer_ring.cycle_state(),
            );
            log_debug!(
                "xHCI {} EP0 context template: d1={:#010x} d2={:#010x} d3={:#010x}",
                ident,
                ep0_words.get(1).copied().unwrap_or(0),
                ep0_words.get(2).copied().unwrap_or(0),
                ep0_words.get(3).copied().unwrap_or(0)
            );
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

    #[allow(dead_code)]
    fn enqueue_setup_stage(&self, ring: &mut TransferRing, setup: UsbControlSetup, chain: bool) {
        let control = (TRB_TYPE_SETUP_STAGE << 10) | TRB_IDT | if chain { TRB_CHAIN } else { 0 };
        ring.enqueue(setup.to_immediate(), 8, control);
    }

    #[allow(dead_code)]
    fn enqueue_data_stage(
        &self,
        ring: &mut TransferRing,
        buffer_addr: u64,
        length: usize,
        direction: TransferDir,
        chain: bool,
        interrupt_on_completion: bool,
        td_size: u32,
    ) {
        assert!(length <= u32::MAX as usize);
        let mut control = TRB_TYPE_DATA_STAGE << 10;
        if matches!(direction, TransferDir::In) {
            control |= TRB_DIR_IN;
        }
        if chain {
            control |= TRB_CHAIN;
        }
        if interrupt_on_completion {
            control |= TRB_IOC;
        }
        let status = (length as u32) | ((td_size & 0x1F) << 17);
        ring.enqueue(buffer_addr, status, control);
    }

    #[allow(dead_code)]
    fn enqueue_status_stage(
        &self,
        ring: &mut TransferRing,
        direction: TransferDir,
        interrupt_on_completion: bool,
    ) {
        let mut control = TRB_TYPE_STATUS_STAGE << 10;
        if matches!(direction, TransferDir::In) {
            control |= TRB_DIR_IN;
        }
        if interrupt_on_completion {
            control |= TRB_IOC;
        }
        ring.enqueue(0, 0, control);
    }

    /// Ensure the shared EP0 descriptor buffer is large enough for the next transfer.
    fn ensure_ep0_buffer<'a>(
        &self,
        controller: &'a mut XhciController,
        required: usize,
    ) -> Result<&'a mut DmaBuffer, &'static str> {
        let current = controller
            .ep0_descriptor_buffer
            .as_ref()
            .map(|buffer| buffer.len())
            .unwrap_or(0);

        if current < required {
            let size = core::cmp::max(required, 64);
            let buffer = DmaBuffer::allocate(size, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate descriptor buffer")?;
            controller.ep0_descriptor_buffer = Some(buffer);
        }

        controller
            .ep0_descriptor_buffer
            .as_mut()
            .ok_or("descriptor buffer missing")
    }

    /// Queue a generic GET_DESCRIPTOR control transfer on EP0.
    fn queue_descriptor_transfer(
        &self,
        controller: &mut XhciController,
        ident: &str,
        mut setup: UsbControlSetup,
        length: usize,
        label: &str,
    ) -> Result<(), &'static str> {
        let slot_id = controller
            .active_slot
            .ok_or("cannot queue descriptor request without active slot")?;
        setup.length = length.min(u16::MAX as usize) as u16;

        let buffer_phys = {
            let buffer = self.ensure_ep0_buffer(controller, length)?;
            let slice = buffer.as_mut_slice();
            let clear_len = length.min(slice.len());
            slice[..clear_len].fill(0);
            buffer.phys_addr()
        };

        {
            let ring = controller
                .control_transfer_ring
                .as_mut()
                .ok_or("control transfer ring missing")?;

            let start_index = ring.enqueue_index;
            let capacity = ring.capacity().max(1);
            self.enqueue_setup_stage(ring, setup, true);
            // Data stage returns the descriptor body from the device (IN transfer).
            self.enqueue_data_stage(ring, buffer_phys, length, TransferDir::In, true, true, 0);
            // Status stage toggles direction back OUT to complete the control transfer.
            self.enqueue_status_stage(ring, TransferDir::Out, true);

            // The ring is circular; note which indices were actually touched so
            // the debug output mirrors the slots hardware will consume.
            let queued_indices = [
                start_index % capacity,
                (start_index + 1) % capacity,
                (start_index + 2) % capacity,
            ];
            for &index in &queued_indices {
                let trb_ptr = ring.trb_ptr(index);
                unsafe {
                    let parameter_lo = core::ptr::read_volatile(trb_ptr);
                    let parameter_hi = core::ptr::read_volatile(trb_ptr.add(1));
                    let status_dword = core::ptr::read_volatile(trb_ptr.add(2));
                    let control_dword = core::ptr::read_volatile(trb_ptr.add(3));
                    let parameter =
                        ((parameter_hi as u64) << 32) | (parameter_lo as u64 & 0xFFFF_FFFF);
                    log_debug!(
                        "xHCI {} EP0 TRB[{}]: parameter={:#x} status={:#010x} control={:#08x}",
                        ident,
                        index,
                        parameter,
                        status_dword,
                        control_dword
                    );
                }
            }

            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            log_debug!(
                "xHCI {} EP0 ring state after queue: enqueue_index={} cycle={}",
                ident,
                ring.enqueue_index,
                if ring.cycle { 1 } else { 0 }
            );
        }

        unsafe {
            let doorbell_base =
                (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(
                doorbell_base.add(slot_id as usize),
                DEFAULT_CONTROL_ENDPOINT as u32,
            );
        }

        log_info!(
            "xHCI {} queued {} on slot {} buffer={:#x}",
            ident,
            label,
            slot_id,
            buffer_phys
        );

        Ok(())
    }

    /// Populate the EP0 transfer ring with a `GET_DESCRIPTOR(Device)` TD.
    ///
    /// Once the descriptor lands we can confirm the controller/slot wiring and
    /// learn the initial max packet size before moving on to configuration parsing.
    /// The ring is reused across transfers, so we log whichever TRB slots were
    /// touched rather than assuming we always start at index zero.
    fn queue_device_descriptor_request(
        &self,
        controller: &mut XhciController,
        ident: &str,
        length: usize,
    ) -> Result<(), &'static str> {
        let setup = UsbControlSetup::get_descriptor(1, 0, 0);
        self.queue_descriptor_transfer(controller, ident, setup, length, "GET_DESCRIPTOR(Device)")
    }

    /// Queue a `GET_DESCRIPTOR(Configuration)` request for the default configuration.
    fn queue_configuration_descriptor_request(
        &self,
        controller: &mut XhciController,
        ident: &str,
        length: usize,
    ) -> Result<(), &'static str> {
        let setup = UsbControlSetup::get_descriptor(2, 0, 0);
        self.queue_descriptor_transfer(
            controller,
            ident,
            setup,
            length,
            "GET_DESCRIPTOR(Configuration)",
        )
    }

    fn fetch_device_descriptor_bytes(
        &self,
        controller: &mut XhciController,
        ident: &str,
        length: usize,
    ) -> Result<Vec<u8>, &'static str> {
        self.queue_device_descriptor_request(controller, ident, length)?;
        self.finalize_ep0_transfer_data(controller, ident, length, "device descriptor")
    }

    fn fetch_configuration_descriptor_bytes(
        &self,
        controller: &mut XhciController,
        ident: &str,
        length: usize,
        label: &str,
    ) -> Result<Vec<u8>, &'static str> {
        self.queue_configuration_descriptor_request(controller, ident, length)?;
        self.finalize_ep0_transfer_data(controller, ident, length, label)
    }

    /// Issue a control transfer without a data stage (setup + status only).
    fn queue_control_transfer_no_data(
        &self,
        controller: &mut XhciController,
        ident: &str,
        setup: UsbControlSetup,
        label: &str,
    ) -> Result<(), &'static str> {
        let slot_id = controller
            .active_slot
            .ok_or("cannot submit control transfer without active slot")?;

        {
            let ring = controller
                .control_transfer_ring
                .as_mut()
                .ok_or("control transfer ring missing")?;

            let start_index = ring.enqueue_index;
            self.enqueue_setup_stage(ring, setup, false);
            let status_direction = if (setup.request_type & 0x80) != 0 {
                TransferDir::Out
            } else {
                TransferDir::In
            };
            self.enqueue_status_stage(ring, status_direction, true);

            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            log_debug!(
                "xHCI {} control(no-data) TRBs queued for {} starting at index {}",
                ident,
                label,
                start_index
            );
        }

        unsafe {
            let doorbell_base =
                (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(
                doorbell_base.add(slot_id as usize),
                DEFAULT_CONTROL_ENDPOINT as u32,
            );
        }

        let event = self.wait_for_ep0_transfer_event(controller, ident)?;
        if event.completion_code() != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} {} completion code {}",
                ident,
                label,
                event.completion_code()
            );
            return Err("control transfer (no data) failed");
        }

        log_info!("xHCI {} {} completed successfully", ident, label);
        Ok(())
    }

    /// Select the specified configuration value on the attached USB device.
    ///
    /// Without this request, interfaces/endpoints beyond EP0 may legally STALL.
    fn set_device_configuration(
        &self,
        controller: &mut XhciController,
        ident: &str,
        configuration_value: u8,
    ) -> Result<(), &'static str> {
        self.queue_control_transfer_no_data(
            controller,
            ident,
            UsbControlSetup::set_configuration(configuration_value),
            "SET_CONFIGURATION",
        )?;
        controller.hid_configuration_value = Some(configuration_value);
        Ok(())
    }

    fn retrieve_configuration_descriptor(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<Vec<u8>, &'static str> {
        let prefix = self.fetch_configuration_descriptor_bytes(
            controller,
            ident,
            CONFIG_DESCRIPTOR_LENGTH,
            "configuration descriptor (prefix)",
        )?;
        if prefix.len() < CONFIG_DESCRIPTOR_LENGTH {
            return Err("configuration descriptor prefix too short");
        }

        let total_length = u16::from_le_bytes([prefix[2], prefix[3]]) as usize;
        if total_length <= prefix.len() {
            Ok(prefix)
        } else {
            self.fetch_configuration_descriptor_bytes(
                controller,
                ident,
                total_length,
                "configuration descriptor",
            )
        }
    }

    /// Translate a USB endpoint address into the corresponding xHCI endpoint ID.
    ///
    /// USB endpoint addresses encode:
    /// - endpoint number in bits 0..=3
    /// - direction (IN) in bit 7
    ///
    /// In practice (and in QEMU's xHCI implementation), endpoint IDs are encoded as:
    /// - **EP0**: endpoint ID 1
    /// - **EPn OUT**: endpoint ID = n * 2
    /// - **EPn IN**:  endpoint ID = n * 2 + 1
    ///
    /// This helper converts the USB encoding into the endpoint ID used by transfer events and
    /// the slot doorbell target.
    fn endpoint_id_from_address(&self, address: u8) -> Option<u8> {
        let number = address & 0x0F;
        let direction_in = (address & 0x80) != 0;
        // Endpoint number is 4 bits, so this is defensive only.
        if number > 15 {
            return None;
        }
        if number == 0 {
            return Some(1);
        }
        let direction_bit = if direction_in { 1 } else { 0 };
        Some(number.saturating_mul(2).saturating_add(direction_bit))
    }

    fn parse_configuration_descriptor(
        &self,
        controller: &mut XhciController,
        ident: &str,
        descriptor: &[u8],
    ) {
        if descriptor.len() < CONFIG_DESCRIPTOR_LENGTH {
            log_warn!(
                "xHCI {} configuration descriptor too short ({} bytes)",
                ident,
                descriptor.len()
            );
            return;
        }

        let total_length = u16::from_le_bytes([descriptor[2], descriptor[3]]) as usize;
        let num_interfaces = descriptor[4];
        let configuration_value = descriptor[5];
        let attributes = descriptor[7];
        let max_power_ma = (descriptor[8] as u16) * 2;

        log_info!(
            "xHCI {} configuration value={} total_length={} interfaces={} attributes={:#04x} max_power={}mA",
            ident,
            configuration_value,
            total_length,
            num_interfaces,
            attributes,
            max_power_ma
        );
        let raw_dump = descriptor
            .iter()
            .map(|byte| format!("{:02x}", byte))
            .collect::<Vec<_>>()
            .join(" ");
        log_debug!(
            "xHCI {} raw configuration descriptor bytes: {}",
            ident,
            raw_dump
        );

        let mut offset = CONFIG_DESCRIPTOR_LENGTH;
        let mut active_boot_interface: Option<u8> = None;
        let mut hid_endpoint: Option<HidEndpoint> = None;

        while offset + 1 < descriptor.len() {
            let length = descriptor[offset] as usize;
            if length < 2 {
                log_warn!(
                    "xHCI {} descriptor with invalid length {} at offset {}",
                    ident,
                    length,
                    offset
                );
                break;
            }
            if offset + length > descriptor.len() {
                log_warn!(
                    "xHCI {} descriptor overruns buffer: length {} offset {} total {}",
                    ident,
                    length,
                    offset,
                    descriptor.len()
                );
                break;
            }

            let dtype = descriptor[offset + 1];
            match dtype {
                0x04 => {
                    if length >= 9 {
                        let interface_number = descriptor[offset + 2];
                        let alternate_setting = descriptor[offset + 3];
                        let interface_class = descriptor[offset + 5];
                        let interface_subclass = descriptor[offset + 6];
                        let interface_protocol = descriptor[offset + 7];
                        log_info!(
                            "xHCI {} interface {} alt={} class={:#04x} subclass={:#04x} protocol={:#04x}",
                            ident,
                            interface_number,
                            alternate_setting,
                            interface_class,
                            interface_subclass,
                            interface_protocol
                        );
                        if interface_class == 0x03
                            && interface_subclass == 0x01
                            && interface_protocol == 0x01
                        {
                            active_boot_interface = Some(interface_number);
                            log_info!(
                                "xHCI {} interface {} flagged as HID boot keyboard",
                                ident,
                                interface_number
                            );
                        } else {
                            active_boot_interface = None;
                        }
                    } else {
                        log_warn!(
                            "xHCI {} interface descriptor too short ({} bytes)",
                            ident,
                            length
                        );
                        active_boot_interface = None;
                    }
                }
                0x05 => {
                    if length >= 7 {
                        if let Some(interface_number) = active_boot_interface {
                            let endpoint_address = descriptor[offset + 2];
                            if (endpoint_address & 0x80) != 0 {
                                let max_packet = u16::from_le_bytes([
                                    descriptor[offset + 4],
                                    descriptor[offset + 5],
                                ]);
                                let interval = descriptor[offset + 6];
                                let Some(endpoint_id) =
                                    self.endpoint_id_from_address(endpoint_address)
                                else {
                                    log_warn!(
                                        "xHCI {} could not derive endpoint ID from address {:#04x}",
                                        ident,
                                        endpoint_address
                                    );
                                    offset += length;
                                    continue;
                                };
                                hid_endpoint = Some(HidEndpoint {
                                    interface_number,
                                    endpoint_address,
                                    max_packet_size: max_packet,
                                    interval,
                                    endpoint_id,
                                });
                                log_info!(
                                    "xHCI {} HID keyboard endpoint discovered: addr={:#04x} max_packet={} bInterval={}",
                                    ident,
                                    endpoint_address,
                                    max_packet,
                                    interval
                                );
                                // Avoid tracking additional endpoints for now; retain the first IN endpoint.
                                active_boot_interface = None;
                            }
                        }
                    } else {
                        log_warn!(
                            "xHCI {} endpoint descriptor too short ({} bytes)",
                            ident,
                            length
                        );
                    }
                }
                0x21 => {
                    if length >= 6 {
                        let hid_version =
                            u16::from_le_bytes([descriptor[offset + 2], descriptor[offset + 3]]);
                        let country_code = descriptor[offset + 4];
                        let descriptor_count = descriptor[offset + 5];
                        log_info!(
                            "xHCI {} HID descriptor: version={:#06x} country={} report_desc_count={}",
                            ident,
                            hid_version,
                            country_code,
                            descriptor_count
                        );
                    }
                }
                _ => {
                    // Other descriptor types are ignored for now.
                }
            }

            offset += length;
        }

        controller.hid_boot_keyboard = hid_endpoint;
        if hid_endpoint.is_none() {
            log_info!(
                "xHCI {} HID boot keyboard endpoint not found in configuration",
                ident
            );
        }
    }

    /// Collect the current diagnostic snapshot of all managed xHCI controllers.
    pub fn diagnostics_snapshot(&self) -> Vec<ControllerDiagnostics> {
        let mut snapshot = Vec::new();
        let controllers = CONTROLLERS.lock();
        for controller in controllers.iter() {
            let ports = self.collect_port_states(controller);
            let port_diags = ports
                .iter()
                .map(|state| PortDiagnostics {
                    index: state.index as u8,
                    connected: state.connected,
                    enabled: state.enabled,
                    powered: state.powered,
                    overcurrent: state.overcurrent,
                    speed: state.speed.as_str().to_string(),
                    link_state: state.link_state.as_str().to_string(),
                    raw: state.register,
                })
                .collect();

            let (iman_raw, interrupt_enabled, interrupt_pending) = unsafe {
                let runtime_base =
                    (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
                let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
                let iman_ptr = interrupter0.add(0x00) as *mut u32;
                let iman = core::ptr::read_volatile(iman_ptr);
                (iman, (iman & IMAN_IE) != 0, (iman & IMAN_IP) != 0)
            };

            let hid_keyboard = controller.hid_boot_keyboard.map(|hid| HidEndpointSummary {
                interface_number: hid.interface_number,
                endpoint_address: hid.endpoint_address,
                max_packet_size: hid.max_packet_size,
                interval: hid.interval,
                endpoint_id: hid.endpoint_id,
                reports_seen: controller.hid_reports_seen,
            });

            snapshot.push(ControllerDiagnostics {
                ident: controller.ident.clone(),
                phys_base: controller.phys_base,
                mmio_length: controller.mmio_length,
                max_ports: controller.max_ports,
                max_slots: controller.max_slots,
                controller_running: controller.controller_running,
                slots_enabled: controller.slots_enabled,
                active_slot: controller.active_slot,
                attached_port: controller.attached_port,
                attached_speed: controller
                    .attached_speed
                    .map(|speed| speed.as_str().to_string()),
                msi_enabled: controller.msi_enabled.load(Ordering::Acquire),
                msi_vector: controller.msi_vector,
                interrupt_enabled,
                interrupt_pending,
                iman_raw,
                hid_keyboard,
                ports: port_diags,
            });
        }
        snapshot
    }

    /// Await completion of an EP0 descriptor transfer and return the captured bytes.
    fn finalize_ep0_transfer_data(
        &self,
        controller: &mut XhciController,
        ident: &str,
        expected_length: usize,
        label: &str,
    ) -> Result<Vec<u8>, &'static str> {
        let event = self.wait_for_ep0_transfer_event(controller, ident)?;
        let code = event.completion_code();
        if code != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} {} transfer failed with completion code {}",
                ident,
                label,
                code
            );
            return Err("control transfer failed");
        }

        if event.residual_length() != 0 {
            log_warn!(
                "xHCI {} {} transfer reported residual length {}",
                ident,
                label,
                event.residual_length()
            );
        }

        let ring = controller
            .control_transfer_ring
            .as_ref()
            .ok_or("control transfer ring missing")?;

        let base = ring.phys_addr();
        let span = (ring.total_trbs() * TRB_SIZE) as u64;
        if event.parameter < base || event.parameter >= base + span {
            log_warn!(
                "xHCI {} {} event pointer {:#x} outside EP0 ring [{:#x}, {:#x})",
                ident,
                label,
                event.parameter,
                base,
                base + span
            );
        } else {
            let offset = event.parameter - base;
            if offset % TRB_SIZE as u64 != 0 {
                log_warn!(
                    "xHCI {} device descriptor event pointer misaligned: offset={:#x}",
                    ident,
                    offset
                );
            } else {
                let index = (offset / TRB_SIZE as u64) as usize;
                let stage = match index {
                    0 => "setup",
                    1 => "data",
                    2 => "status",
                    other => {
                        log_warn!(
                            "xHCI {} {} transfer completed on unexpected TRB index {}",
                            ident,
                            label,
                            other
                        );
                        "unknown"
                    }
                };
                log_info!(
                    "xHCI {} {} completion landed on {} stage (TRB index {})",
                    ident,
                    label,
                    stage,
                    index
                );
            }
        }

        let buffer = controller
            .ep0_descriptor_buffer
            .as_mut()
            .ok_or("descriptor buffer missing")?;
        let slice = buffer.as_mut_slice();
        if slice.len() < expected_length {
            return Err("descriptor buffer too small");
        }
        Ok(slice[..expected_length].to_vec())
    }

    /// Emit a structured log for the retrieved device descriptor bytes.
    fn log_device_descriptor(&self, ident: &str, descriptor: &[u8]) {
        if descriptor.len() >= DEVICE_DESCRIPTOR_LENGTH {
            let length = descriptor[0];
            let descriptor_type = descriptor[1];
            let bcd_usb = u16::from_le_bytes([descriptor[2], descriptor[3]]);
            let device_class = descriptor[4];
            let device_subclass = descriptor[5];
            let device_protocol = descriptor[6];
            let max_packet_size = descriptor[7];
            let vendor_id = u16::from_le_bytes([descriptor[8], descriptor[9]]);
            let product_id = u16::from_le_bytes([descriptor[10], descriptor[11]]);
            let device_release = u16::from_le_bytes([descriptor[12], descriptor[13]]);
            let manufacturer_index = descriptor[14];
            let product_index = descriptor[15];
            let serial_index = descriptor[16];
            let configuration_count = descriptor[17];

            if length as usize != DEVICE_DESCRIPTOR_LENGTH {
                log_warn!(
                    "xHCI {} device descriptor length {} differs from expected {}",
                    ident,
                    length,
                    DEVICE_DESCRIPTOR_LENGTH
                );
            }
            if descriptor_type != 1 {
                log_warn!(
                    "xHCI {} unexpected descriptor type {} while reading device descriptor",
                    ident,
                    descriptor_type
                );
            }

            log_info!(
                "xHCI {} device descriptor: usb={:#06x} vid={:#06x} pid={:#06x} release={:#06x}",
                ident,
                bcd_usb,
                vendor_id,
                product_id,
                device_release
            );
            log_info!(
                "xHCI {} class={:#04x} subclass={:#04x} protocol={:#04x} max_packet_ep0={}",
                ident,
                device_class,
                device_subclass,
                device_protocol,
                max_packet_size
            );
            log_info!(
                "xHCI {} strings: manufacturer={} product={} serial={} configurations={}",
                ident,
                manufacturer_index,
                product_index,
                serial_index,
                configuration_count
            );
        } else if descriptor.len() >= 8 {
            let partial = descriptor
                .iter()
                .map(|byte| format!("{:02x}", byte))
                .collect::<Vec<_>>()
                .join(" ");
            log_info!(
                "xHCI {} device descriptor (first {} bytes): {}",
                ident,
                descriptor.len(),
                partial
            );
        } else {
            log_info!(
                "xHCI {} device descriptor returned {} byte(s): {:02x?}",
                ident,
                descriptor.len(),
                descriptor
            );
        }

        let byte_dump = descriptor
            .iter()
            .map(|byte| format!("{:02x}", byte))
            .collect::<Vec<_>>()
            .join(" ");
        log_debug!("xHCI {} raw device descriptor bytes: {}", ident, byte_dump);
    }

    /// Update the EP0 context with a new maximum packet size via Evaluate Context.
    ///
    /// After the initial 8-byte descriptor read the BSR flow requires us to patch
    /// the device context with the true max packet size the device reports. This
    /// helper flips the Input Control Context masks, re-seeds the endpoint context,
    /// and then issues the Evaluate Context command.
    fn update_ep0_max_packet(
        &self,
        controller: &mut XhciController,
        ident: &str,
        max_packet: u8,
    ) -> Result<(), &'static str> {
        let ring = controller
            .control_transfer_ring
            .as_ref()
            .ok_or("control transfer ring missing")?;
        {
            let ctx = controller
                .input_context
                .as_mut()
                .ok_or("input context missing")?;

            let control_words = ctx.control_words_mut(controller.context_entry_size);
            control_words.fill(0);
            if control_words.len() > 1 {
                control_words[1] = 1 << DEFAULT_CONTROL_ENDPOINT;
            }
            let drop_flags = control_words.get(0).copied().unwrap_or(0);
            let add_flags = control_words.get(1).copied().unwrap_or(0);
            log_debug!(
                "xHCI {} evaluate-context ICC: drop={:#010x} add={:#010x}",
                ident,
                drop_flags,
                add_flags
            );

            if let Some(ep_words) =
                ctx.endpoint_words_mut(controller.context_entry_size, DEFAULT_CONTROL_ENDPOINT)
            {
                self.populate_endpoint_zero(
                    ep_words,
                    controller.attached_speed.unwrap_or(PortSpeed::Full),
                    ring.dequeue_pointer(),
                    ring.cycle_state(),
                );
                if ep_words.len() > 1 {
                    ep_words[1] =
                        (ENDPOINT_TYPE_CONTROL << 3) | (3 << 1) | ((max_packet as u32) << 16);
                }
            } else {
                return Err("failed to access endpoint context while updating max packet");
            }
        }

        self.evaluate_context_command(controller, ident)?;

        {
            let ctx = controller
                .input_context
                .as_mut()
                .ok_or("input context missing")?;
            let control_words = ctx.control_words_mut(controller.context_entry_size);
            control_words.fill(0);
            if control_words.len() > 1 {
                control_words[1] = (1 << SLOT_CONTEXT_INDEX) | (1 << DEFAULT_CONTROL_ENDPOINT);
            }
        }

        log_info!(
            "xHCI {} EP0 max-packet updated to {} bytes",
            ident,
            max_packet
        );

        Ok(())
    }

    /// Copy the output device context back into the input context template.
    fn sync_input_context_from_device(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let slot_id = controller
            .active_slot
            .ok_or("sync requested without active slot")?;
        let ctx = controller
            .input_context
            .as_mut()
            .ok_or("input context missing during sync")?;
        let dev_ctx = controller
            .slot_contexts
            .get(slot_id as usize - 1)
            .ok_or("device context missing during sync")?;
        let entry_words = controller.context_entry_size / core::mem::size_of::<u32>();

        unsafe {
            let src_slot =
                core::slice::from_raw_parts(dev_ctx.virt_addr() as *const u32, entry_words);
            let dst_slot = ctx.slot_words_mut(controller.context_entry_size);
            dst_slot.copy_from_slice(src_slot);

            if let Some(dst_ep0) =
                ctx.endpoint_words_mut(controller.context_entry_size, DEFAULT_CONTROL_ENDPOINT)
            {
                let ep_src_ptr =
                    (dev_ctx.virt_addr() + controller.context_entry_size as u64) as *const u32;
                let src_ep0 = core::slice::from_raw_parts(ep_src_ptr, dst_ep0.len());
                dst_ep0.copy_from_slice(src_ep0);
            } else {
                return Err("failed to sync endpoint context");
            }
        }

        {
            let control_words = ctx.control_words_mut(controller.context_entry_size);
            control_words.fill(0);
            if control_words.len() > 1 {
                control_words[1] = (1 << SLOT_CONTEXT_INDEX) | (1 << DEFAULT_CONTROL_ENDPOINT);
            }
        }

        log_debug!(
            "xHCI {} input context synchronised from device context (slot={})",
            ident,
            slot_id
        );

        Ok(())
    }

    fn reset_attached_port(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<PortState, &'static str> {
        let port_index = controller
            .attached_port
            .ok_or("no attached port recorded for reset")?;
        let states = self.collect_port_states(controller);
        let state = states
            .into_iter()
            .find(|p| p.index as u8 == port_index)
            .ok_or("attached port state unavailable")?;
        self.reset_port(controller, ident, state)
    }

    /// Initialise the default control endpoint context template used during address assignment.
    ///
    /// `dequeue_pointer` should match the TR dequeue pointer field programmed in
    /// the endpoint context (dword 2 and 3). `dequeue_cycle` encodes the desired
    /// DCS bit, ensuring the consumer and producer agree on the initial cycle state.
    fn populate_endpoint_zero(
        &self,
        endpoint_words: &mut [u32],
        speed: PortSpeed,
        dequeue_pointer: u64,
        dequeue_cycle: bool,
    ) {
        endpoint_words.fill(0);

        let error_recovery_count: u32 = 3;
        let max_packet = Self::control_max_packet_size(speed);

        if !endpoint_words.is_empty() {
            endpoint_words[0] = 0;
        }
        if endpoint_words.len() > 1 {
            endpoint_words[1] =
                (ENDPOINT_TYPE_CONTROL << 3) | (error_recovery_count << 1) | (max_packet << 16);
        }
        let dequeue_masked = dequeue_pointer & !0xFu64;
        let dequeue_low = (dequeue_masked as u32) | if dequeue_cycle { 1 } else { 0 };
        let dequeue_high = (dequeue_masked >> 32) as u32;
        if endpoint_words.len() > 2 {
            endpoint_words[2] = dequeue_low;
        }
        if endpoint_words.len() > 3 {
            endpoint_words[3] = dequeue_high;
        }
        if endpoint_words.len() > 4 {
            let average_trb_length = core::cmp::max(max_packet as u32, 8);
            let max_burst = match speed {
                PortSpeed::High | PortSpeed::Super | PortSpeed::SuperPlus => 0,
                _ => 0,
            };
            endpoint_words[4] = (max_burst << 16) | average_trb_length;
        }
        if endpoint_words.len() > 5 {
            endpoint_words[5] = 0;
        }
        if endpoint_words.len() > 7 {
            endpoint_words[7] = max_packet as u32;
        }
    }

    /// Seed an interrupt-IN endpoint context for the HID keyboard.
    ///
    /// The endpoint launches disabled (state 0) and advertises the transfer ring
    /// dequeue pointer plus the interrupt polling interval derived from the HID
    /// descriptor.
    fn populate_interrupt_endpoint(
        &self,
        endpoint_words: &mut [u32],
        dequeue_pointer: u64,
        dequeue_cycle: bool,
        max_packet: u16,
        interval: u8,
    ) {
        endpoint_words.fill(0);

        if !endpoint_words.is_empty() {
            endpoint_words[0] = (interval as u32) << 16;
        }
        if endpoint_words.len() > 1 {
            let error_recovery_count: u32 = 3;
            endpoint_words[1] = (ENDPOINT_TYPE_INTERRUPT_IN << 3)
                | (error_recovery_count << 1)
                | ((max_packet as u32) << 16);
        }
        let dequeue_masked = dequeue_pointer & !0xFu64;
        let dequeue_low = (dequeue_masked as u32) | if dequeue_cycle { 1 } else { 0 };
        let dequeue_high = (dequeue_masked >> 32) as u32;
        if endpoint_words.len() > 2 {
            endpoint_words[2] = dequeue_low;
        }
        if endpoint_words.len() > 3 {
            endpoint_words[3] = dequeue_high;
        }
        if endpoint_words.len() > 4 {
            let average_trb_length = core::cmp::max(max_packet as u32, 8);
            let max_esit_payload = max_packet as u32;
            endpoint_words[4] = (max_esit_payload << 16) | average_trb_length;
        }
        if endpoint_words.len() > 5 {
            endpoint_words[5] = 0;
        }
        if endpoint_words.len() > 7 {
            endpoint_words[7] = max_packet as u32;
        }
    }

    /// Ensure a DMA buffer exists for HID interrupt reports with at least `required` bytes.
    fn ensure_hid_report_buffer<'a>(
        &self,
        controller: &'a mut XhciController,
        ident: &str,
        required: usize,
    ) -> Result<&'a mut DmaBuffer, &'static str> {
        let need_allocation = match controller.hid_report_buffer.as_ref() {
            Some(buffer) => buffer.len() < required,
            None => true,
        };

        if need_allocation {
            let size = core::cmp::max(required, 8);
            let buffer = DmaBuffer::allocate(size, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate HID report buffer")?;
            let phys = buffer.phys_addr();
            let len = buffer.len();
            controller.hid_report_buffer = Some(buffer);
            log_info!(
                "xHCI {} HID report buffer allocated: phys={:#012x} bytes={}",
                ident,
                phys,
                len
            );
        }

        controller
            .hid_report_buffer
            .as_mut()
            .ok_or("HID report buffer missing after allocation attempt")
    }

    /// Configure and arm the HID keyboard's interrupt-IN endpoint.
    ///
    /// This allocates a dedicated transfer ring, programs the endpoint context via
    /// `Configure Endpoint`, queues an initial normal TRB, and rings the slot
    /// doorbell so the controller can begin polling for input reports.
    fn configure_hid_interrupt_endpoint(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let Some(hid) = controller.hid_boot_keyboard else {
            log_info!(
                "xHCI {} skipping HID endpoint configuration (no boot keyboard discovered)",
                ident
            );
            return Ok(());
        };

        let endpoint_index = hid.endpoint_id as usize;
        if endpoint_index == 0 || endpoint_index >= DEVICE_CONTEXT_ENTRIES {
            log_warn!(
                "xHCI {} HID endpoint index {} out of range",
                ident,
                endpoint_index
            );
            return Err("hid endpoint index invalid");
        }

        let slot_id = controller
            .active_slot
            .ok_or("HID endpoint requested without active slot")?;

        self.sync_input_context_from_device(controller, ident)?;

        if controller.hid_interrupt_ring.is_none() {
            let size = HID_INTERRUPT_RING_TRBS * TRB_SIZE;
            let buffer = DmaBuffer::allocate(size, RING_ALIGNMENT)
                .map_err(|_| "failed to allocate HID interrupt ring")?;
            let ring = TransferRing::new(buffer, HID_INTERRUPT_RING_TRBS);
            let phys = ring.phys_addr();
            let usable = ring.capacity();
            let total = ring.total_trbs();
            log_info!(
                "xHCI {} HID interrupt ring allocated: phys={:#012x} usable_trbs={} total_trbs={}",
                ident,
                phys,
                usable,
                total
            );
            controller.hid_interrupt_ring = Some(ring);
        }

        let max_packet = hid.max_packet_size;
        let interval = if hid.interval == 0 { 1 } else { hid.interval };
        let required_bytes = core::cmp::max(max_packet as usize, 8);
        let (dequeue_pointer, dequeue_cycle) = {
            let ring = controller
                .hid_interrupt_ring
                .as_ref()
                .ok_or("hid interrupt ring missing after allocation")?;
            (ring.dequeue_pointer(), ring.cycle_state())
        };
        let (buffer_phys, buffer_len) = {
            let buffer = self.ensure_hid_report_buffer(controller, ident, required_bytes)?;
            buffer.as_mut_slice().fill(0);
            (buffer.phys_addr(), buffer.len())
        };
        controller.hid_reports_seen = 0;
        controller.hid_last_report.fill(0);
        controller.hid_last_report_len = 0;

        {
            let ctx = controller
                .input_context
                .as_mut()
                .ok_or("input context missing while configuring HID endpoint")?;
            let entry_size = controller.context_entry_size;

            {
                let control_words = ctx.control_words_mut(entry_size);
                control_words.fill(0);
                if control_words.len() > 1 {
                    control_words[1] = (1 << SLOT_CONTEXT_INDEX) | (1 << endpoint_index);
                }
            }

            {
                let slot_words = ctx.slot_words_mut(entry_size);
                if slot_words.len() > 1 {
                    slot_words[1] &= !(0x1F << 27);
                    slot_words[1] |= ((endpoint_index as u32) & 0x1F) << 27;
                }
            }

            if let Some(ep_words) = ctx.endpoint_words_mut(entry_size, endpoint_index) {
                self.populate_interrupt_endpoint(
                    ep_words,
                    dequeue_pointer,
                    dequeue_cycle,
                    max_packet,
                    interval,
                );
                log_debug!(
                    "xHCI {} HID endpoint context seeded: id={} d0={:#010x} d1={:#010x} d2={:#010x} d3={:#010x}",
                    ident,
                    endpoint_index,
                    ep_words.get(0).copied().unwrap_or(0),
                    ep_words.get(1).copied().unwrap_or(0),
                    ep_words.get(2).copied().unwrap_or(0),
                    ep_words.get(3).copied().unwrap_or(0)
                );
            } else {
                return Err("failed to map HID endpoint context");
            }
        }

        self.configure_endpoint_command(controller, ident)?;
        self.sync_input_context_from_device(controller, ident)
            .unwrap_or_else(|err| {
                log_debug!(
                    "xHCI {} HID endpoint post-config sync skipped: {}",
                    ident,
                    err
                );
            });

        if let Err(err) = self.send_hid_set_protocol(controller, ident, hid, true) {
            log_warn!("xHCI {} failed to send HID SET_PROTOCOL(boot): {}", ident, err);
        }

        if let Err(err) = self.send_hid_set_idle(controller, ident, hid) {
            log_warn!("xHCI {} failed to send HID SET_IDLE: {}", ident, err);
        }

        // TODO: teach the control path to negotiate HID boot protocol via SET_PROTOCOL when the OS needs 8-byte reports.
        let mut transfer_length = core::cmp::min(buffer_len, max_packet as usize) as u32;
        if transfer_length == 0 {
            transfer_length = max_packet as u32;
        }
        {
            let ring = controller
                .hid_interrupt_ring
                .as_mut()
                .ok_or("hid interrupt ring unavailable for queueing")?;
            let start_index = ring.enqueue_index;
            ring.enqueue(
                buffer_phys,
                transfer_length,
                // Direction is implied by the endpoint context; do not set the
                // Setup/Status-stage DIR bit on a Normal TRB.
                (TRB_TYPE_NORMAL << 10) | TRB_IOC,
            );
            log_debug!(
                "xHCI {} HID ring enqueue complete: start={} next={} cycle={}",
                ident,
                start_index,
                ring.enqueue_index,
                if ring.cycle { 1 } else { 0 }
            );
        }

        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

        unsafe {
            let doorbell = (controller.virt_base + controller.doorbell_offset as u64) as *mut u32;
            core::ptr::write_volatile(doorbell.add(slot_id as usize), endpoint_index as u32);
        }

        log_info!(
            "xHCI {} HID endpoint armed: slot={} endpoint={} buffer={:#012x} length={}",
            ident,
            slot_id,
            endpoint_index,
            buffer_phys,
            transfer_length
        );

        Ok(())
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
    ///
    /// The control transfer ring must already exist so that the endpoint
    /// context points at valid TRB storage once the controller enables the slot.
    fn address_device_command(
        &self,
        controller: &mut XhciController,
        ident: &str,
        block_set_address: bool,
    ) -> Result<(), &'static str> {
        let slot_id = controller
            .active_slot
            .ok_or("address device requested without slot")?;
        if !controller.control_context_ready {
            return Err("control context not initialised");
        }

        if controller.control_transfer_ring.is_none() {
            return Err("control transfer ring not initialised");
        }

        let parameter = {
            let ctx = controller
                .input_context
                .as_mut()
                .ok_or("input context missing")?;
            let entry_size = controller.context_entry_size;
            {
                let control_words = ctx.control_words_mut(entry_size);
                control_words.fill(0);
                if control_words.len() > 1 {
                    control_words[1] = (1 << SLOT_CONTEXT_INDEX) | (1 << DEFAULT_CONTROL_ENDPOINT);
                }
            }
            ctx.phys_addr() & !0xF
        };

        let ics_flag = if controller.context_entry_size == 64 {
            1
        } else {
            0
        }; // ICS bit
        let mut control = (TRB_TYPE_ADDRESS_DEVICE << 10) | ((slot_id as u32) << 24);
        if block_set_address {
            control |= 1 << 9;
        }
        let trb = RawCommandTrb {
            parameter,
            status: ics_flag,
            control,
        };

        let (completion, index) = self.issue_command(controller, ident, trb)?;
        if completion.code != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} address-device (slot {}) returned completion code {}",
                ident,
                slot_id,
                completion.code
            );
            return Err("address device failed");
        }

        log_info!(
            "xHCI {} address-device success (slot {} index={})",
            ident,
            slot_id,
            index
        );

        Ok(())
    }

    /// Execute the enumeration sequence for the default control endpoint following xHCI §4.3.4.
    ///
    /// The flow matches the template recommended by the spec and by the Linux and
    /// FreeBSD stacks: block-set-address (BSR) address assignment, an 8-byte
    /// descriptor prefix read, port reset, max-packet update via Evaluate Context,
    /// permanent Address Device, Configure Endpoint, and finally a full 18-byte
    /// descriptor fetch.
    fn enumerate_default_control_endpoint(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        self.address_device_command(controller, ident, true)?;
        self.sync_input_context_from_device(controller, ident)?;

        let descriptor_prefix = self.fetch_device_descriptor_bytes(controller, ident, 8)?;
        self.log_device_descriptor(ident, &descriptor_prefix);
        if descriptor_prefix.len() < 8 {
            log_warn!(
                "xHCI {} device descriptor prefix shorter than expected ({} bytes)",
                ident,
                descriptor_prefix.len()
            );
        }
        let max_packet = descriptor_prefix.get(7).copied().unwrap_or(8).max(8u8);

        let mut refreshed_port = None;
        if let Ok(new_port_state) = self.reset_attached_port(controller, ident) {
            controller.attached_speed = Some(new_port_state.speed);
            controller.attached_port = Some(new_port_state.index as u8);
            log_info!(
                "xHCI {} port{:02} reset complete (speed={})",
                ident,
                new_port_state.index,
                new_port_state.speed.as_str()
            );
            refreshed_port = Some(new_port_state);
        } else {
            log_debug!(
                "xHCI {} unable to reset attached port after prefix read",
                ident
            );
        }

        if let Some(port_state) = refreshed_port {
            // Rebuild the control endpoint template now that the port is in U0 with
            // the newly negotiated speed.
            self.prepare_default_control_context(controller, ident, port_state)?;
        }

        self.update_ep0_max_packet(controller, ident, max_packet)?;
        self.sync_input_context_from_device(controller, ident)?;

        self.address_device_command(controller, ident, false)?;
        self.sync_input_context_from_device(controller, ident)?;

        {
            let ctx = controller
                .input_context
                .as_mut()
                .ok_or("input context missing before configure-endpoint")?;
            let control_words = ctx.control_words_mut(controller.context_entry_size);
            control_words.fill(0);
            if control_words.len() > 1 {
                control_words[1] = 1 << SLOT_CONTEXT_INDEX;
            }
            log_debug!(
                "xHCI {} configure-endpoint ICC primed: drop={:#010x} add={:#010x}",
                ident,
                control_words.get(0).copied().unwrap_or(0),
                control_words.get(1).copied().unwrap_or(0)
            );
        }

        if let Err(err) = self.configure_endpoint_command(controller, ident) {
            log_warn!(
                "xHCI {} configure-endpoint failed after address-device: {}",
                ident,
                err
            );
        } else {
            self.sync_input_context_from_device(controller, ident)
                .unwrap_or_else(|err| {
                    log_debug!("xHCI {} input sync after configure skipped: {}", ident, err);
                });
        }

        let descriptor_full =
            self.fetch_device_descriptor_bytes(controller, ident, DEVICE_DESCRIPTOR_LENGTH)?;
        self.log_device_descriptor(ident, &descriptor_full);

        match self.retrieve_configuration_descriptor(controller, ident) {
            Ok(configuration) => {
                self.parse_configuration_descriptor(controller, ident, &configuration);

                // Select the configuration before arming non-EP0 endpoints.
                // HID interrupt endpoints may STALL until the device is configured.
                let config_value = configuration.get(5).copied().unwrap_or(1);
                if let Err(err) = self.set_device_configuration(controller, ident, config_value) {
                    log_warn!(
                        "xHCI {} failed to set device configuration {}: {}",
                        ident,
                        config_value,
                        err
                    );
                }
            }
            Err(err) => log_warn!(
                "xHCI {} failed to retrieve configuration descriptor: {}",
                ident,
                err
            ),
        }

        if let Err(err) = self.configure_hid_interrupt_endpoint(controller, ident) {
            log_warn!(
                "xHCI {} failed to configure HID interrupt endpoint: {}",
                ident,
                err
            );
        }

        Ok(())
    }

    /// Issue an Evaluate Context command to apply updated context fields.
    fn evaluate_context_command(
        &self,
        controller: &mut XhciController,
        ident: &str,
    ) -> Result<(), &'static str> {
        let slot_id = controller
            .active_slot
            .ok_or("evaluate context requested without slot")?;
        let ctx = controller
            .input_context
            .as_ref()
            .ok_or("input context missing for evaluate-context")?;

        let parameter = ctx.phys_addr() & !0xF;
        let trb = RawCommandTrb {
            parameter,
            status: 0,
            control: (TRB_TYPE_EVALUATE_CONTEXT << 10) | ((slot_id as u32) << 24),
        };

        let (completion, index) = self.issue_command(controller, ident, trb)?;
        if completion.code != TRB_COMPLETION_SUCCESS {
            log_warn!(
                "xHCI {} evaluate-context (slot {}) completion code {}",
                ident,
                slot_id,
                completion.code
            );
            return Err("evaluate context failed");
        }

        log_info!(
            "xHCI {} evaluate-context success (slot {} index={})",
            ident,
            slot_id,
            index
        );

        Ok(())
    }

    /// Issue the HID class `SET_IDLE` request so the keyboard only reports on state changes.
    fn send_hid_set_idle(
        &self,
        controller: &mut XhciController,
        ident: &str,
        hid: HidEndpoint,
    ) -> Result<(), &'static str> {
        let setup = UsbControlSetup {
            request_type: 0x21, // Class | Interface | Host-to-device
            request: 0x0A,      // SET_IDLE
            value: 0,           // duration=0, report ID=0
            index: hid.interface_number as u16,
            length: 0,
        };
        self.queue_control_transfer_no_data(controller, ident, setup, "SET_IDLE(HID)")
    }

    /// Issue a HID `SET_PROTOCOL` request to select boot protocol (8-byte reports).
    fn send_hid_set_protocol(
        &self,
        controller: &mut XhciController,
        ident: &str,
        hid: HidEndpoint,
        boot_protocol: bool,
    ) -> Result<(), &'static str> {
        let setup = UsbControlSetup::set_protocol(hid.interface_number, boot_protocol);
        let label = if boot_protocol {
            "SET_PROTOCOL(boot)"
        } else {
            "SET_PROTOCOL(report)"
        };
        self.queue_control_transfer_no_data(controller, ident, setup, label)
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

    /// Drain runtime events for the provided controller list, optionally skipping MSI-armed devices.
    fn poll_runtime_events_locked(&self, controllers: &mut Vec<XhciController>, allow_heavy: bool) {
        for controller in controllers.iter_mut() {
            self.poll_runtime_events_for_controller(controller);
            controller.msi_pending_logged = false;
            controller.msi_disarmed_logged = false;
            if allow_heavy {
                let ident = controller.ident.clone();
                self.service_pending_port_change(controller, &ident);
            }
        }
    }

    /// Non-blocking poll hook used during the idle loop to service runtime events.
    fn poll_runtime_events(&self, force: bool) {
        let mut controllers = CONTROLLERS.lock();
        let mut should_force = force;
        if config::USB_ENABLE_POLLING_FALLBACK && !should_force {
            should_force = MSI_DEFERRED_RUNTIME_SERVICE.swap(false, Ordering::AcqRel);
            if should_force {
                log_trace!("xHCI runtime poll forced after deferred interrupt");
            }
        }
        if !config::USB_ENABLE_POLLING_FALLBACK && !should_force {
            // When fallback polling is disabled we must avoid touching runtime
            // registers in the idle loop; QEMU's `trace:usb_*` will otherwise
            // spam the host log with `usb_xhci_runtime_read off 0x0020 ...`.
            //
            // A dedicated diagnostics switch exists for cases where we *do*
            // want to peek at IMAN (IE/IP) without fully draining the ring.
            if config::USB_IDLE_IMAN_DIAGNOSTICS {
                self.trace_pending_interrupts(&mut controllers);
            }
            return;
        }
        self.poll_runtime_events_locked(&mut controllers, should_force);
    }

    /// Interrupt-safe poll hook that attempts to drain runtime events without blocking.
    fn poll_runtime_events_from_interrupt(&self) {
        if let Some(mut controllers) = CONTROLLERS.try_lock() {
            log_trace!("xHCI interrupt servicing event ring immediately");
            self.poll_runtime_events_locked(&mut controllers, false);
        } else {
            log_trace!("xHCI interrupt handling deferred: controller list busy");
            MSI_DEFERRED_RUNTIME_SERVICE.store(true, Ordering::Release);
        }
    }

    /// Debug helper invoked when the fallback poll is disabled so we can detect
    /// controllers that appear to have pending completions without receiving an
    /// interrupt. This keeps the log noise manageable by emitting a warning only
    /// once per pending stretch.
    fn trace_pending_interrupts(&self, controllers: &mut Vec<XhciController>) {
        for controller in controllers.iter_mut() {
            if !controller.msi_enabled.load(Ordering::Acquire) {
                controller.msi_pending_logged = false;
                controller.msi_disarmed_logged = false;
                continue;
            }

            let iman = unsafe {
                let runtime_base =
                    (controller.virt_base + controller.runtime_offset as u64) as *mut u8;
                let interrupter0 = runtime_base.add(INTERRUPTER_STRIDE as usize);
                let iman_ptr = interrupter0.add(0x00) as *mut u32;
                core::ptr::read_volatile(iman_ptr)
            };

            if (iman & IMAN_IE) == 0 {
                if !controller.msi_disarmed_logged {
                    log_warn!(
                        "xHCI {} IMAN.IE cleared while MSI expected active (iman={:#x})",
                        controller.ident,
                        iman
                    );
                    controller.msi_disarmed_logged = true;
                }
                continue;
            } else {
                controller.msi_disarmed_logged = false;
            }

            if (iman & IMAN_IP) != 0 {
                if !controller.msi_pending_logged {
                    log_warn!(
                        "xHCI {} IMAN.IP set without interrupt delivery (iman={:#x})",
                        controller.ident,
                        iman
                    );
                    controller.msi_pending_logged = true;
                }
            } else {
                controller.msi_pending_logged = false;
            }
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
        log_debug!(
            "xHCI init new mapping={} driver_data={:?}",
            newly_mapped,
            dev.driver_data
        );
        self.try_enable_msi(dev);
        if newly_mapped {
            log_debug!("xHCI controller initialised at phys={:#012x}", phys);
        }

        Ok(())
    }

    fn irq_handler(&'static self, _dev: &mut Device, _irq: u32) -> bool {
        self.poll_runtime_events_from_interrupt();
        true
    }
}

/// Public entry point for collecting xHCI diagnostic information.
pub fn diagnostics_snapshot() -> Vec<ControllerDiagnostics> {
    XHCI_DRIVER.diagnostics_snapshot()
}

/// Poll all active xHCI controllers for pending events.
pub fn poll_runtime_events() {
    XHCI_DRIVER.poll_runtime_events(true);
}

/// Poll only controllers that still rely on the fallback polling path.
pub fn poll_runtime_events_fallback() {
    XHCI_DRIVER.poll_runtime_events(false);
}

/// Service an interrupt raised by an xHCI controller by draining its event rings without blocking.
pub fn service_runtime_interrupt() {
    XHCI_DRIVER.poll_runtime_events_from_interrupt();
}

/// Service any deferred runtime work flagged by the interrupt handler.
///
/// When the MSI handler fires while another context holds the controller list
/// lock (e.g. during monitor diagnostics), the handler avoids blocking and sets
/// a latch. This helper lets thread context drain the event ring once the lock
/// becomes available, without enabling full polling fallback.
pub fn service_deferred_runtime() {
    if !MSI_DEFERRED_RUNTIME_SERVICE.swap(false, Ordering::AcqRel) {
        return;
    }

    // Force a drain under the regular lock now that we're in thread context.
    XHCI_DRIVER.poll_runtime_events(true);
}

/// Attempt to kick the MSI/MSI-X delivery self-test for any controller that has
/// MSI enabled but has not yet observed the completion via the interrupt path.
///
/// This is intended to be called after interrupts are enabled (IF=1), such as
/// from the idle loop setup path, so the completion has a chance to actually
/// invoke the handler.
#[allow(dead_code)]
pub fn kick_msix_self_test() {
    if !config::USB_RUN_MSIX_SELF_TEST {
        return;
    }
    {
        let mut controllers = CONTROLLERS.lock();
        for controller in controllers.iter_mut() {
            if !controller.msi_enabled.load(Ordering::Acquire) {
                continue;
            }
            if controller.msix_self_test_pending {
                continue;
            }
            controller.msix_self_test_pending = true;
            match XHCI_DRIVER.submit_noop_async(controller) {
                Ok(()) => log_info!(
                    "xHCI {} MSI/MSI-X self-test (post-IF): NOOP submitted (awaiting interrupt completion)",
                    controller.ident
                ),
                Err(err) => {
                    controller.msix_self_test_pending = false;
                    log_warn!(
                        "xHCI {} MSI/MSI-X self-test (post-IF): failed to submit NOOP: {}",
                        controller.ident,
                        err
                    );
                }
            }
        }
    }

    // If interrupts are misconfigured, the NOOP completion will sit in the
    // event ring and `IMAN.IP` will stay asserted. To keep the system usable
    // (and make debugging deterministic), do a single forced drain pass here.
    //
    // This does *not* re-enable background polling; it only ensures that the
    // one-shot self-test cannot wedge forever.
    XHCI_DRIVER.poll_runtime_events(true);
}
