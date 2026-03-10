//! TRB encodings and helpers for the xHCI driver.
//!
//! This module centralizes the TRB type constants and the small helper structs
//! used when writing to the command ring or consuming from the event ring.

/// TRB types (subset used by the teaching driver).
pub(crate) const TRB_TYPE_NOOP_COMMAND: u32 = 0x17;
pub(crate) const TRB_TYPE_SETUP_STAGE: u32 = 0x02;
pub(crate) const TRB_TYPE_DATA_STAGE: u32 = 0x03;
pub(crate) const TRB_TYPE_STATUS_STAGE: u32 = 0x04;
pub(crate) const TRB_TYPE_LINK: u32 = 0x06;
pub(crate) const TRB_TYPE_ENABLE_SLOT: u32 = 0x09;
pub(crate) const TRB_TYPE_ADDRESS_DEVICE: u32 = 0x0B;
pub(crate) const TRB_TYPE_CONFIGURE_ENDPOINT: u32 = 0x0C;
pub(crate) const TRB_TYPE_EVALUATE_CONTEXT: u32 = 0x0D;
pub(crate) const TRB_TYPE_SET_TR_DEQUEUE_POINTER: u32 = 0x10;
pub(crate) const TRB_TYPE_TRANSFER_EVENT: u32 = 0x20;
pub(crate) const TRB_TYPE_COMMAND_COMPLETION: u32 = 0x21;
pub(crate) const TRB_TYPE_PORT_STATUS_CHANGE_EVENT: u32 = 0x22;
pub(crate) const TRB_TYPE_NORMAL: u32 = 0x01;

/// Completion codes.
pub(crate) const TRB_COMPLETION_SUCCESS: u32 = 1;

/// TRB control/status bit masks.
pub(crate) const TRB_CYCLE_BIT: u32 = 1;
pub(crate) const TRB_CHAIN: u32 = 1 << 4;
pub(crate) const TRB_IOC: u32 = 1 << 5;
pub(crate) const TRB_IDT: u32 = 1 << 6;
pub(crate) const TRB_DIR_IN: u32 = 1 << 16;
pub(crate) const TRB_LINK_TOGGLE_CYCLE: u32 = 1 << 1;

pub(crate) const TRB_COMPLETION_CODE_MASK: u32 = 0xFF << 24;
pub(crate) const TRB_TYPE_MASK: u32 = 0x3F << 10;

/// Raw representation of a command TRB to be written to the command ring.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RawCommandTrb {
    /// Combined parameter value (typically pointer).
    pub(crate) parameter: u64,
    /// Third dword of the TRB (status field).
    pub(crate) status: u32,
    /// Control dword (type bits, interrupter toggle, etc.).
    pub(crate) control: u32,
}

/// Raw representation of an event TRB consumed from the event ring.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RawEventTrb {
    /// Combined parameter payload, frequently a pointer back to the source TRB.
    pub(crate) parameter: u64,
    /// Event status dword containing residual length or contextual flags.
    pub(crate) status: u32,
    /// Control dword encoding cycle, completion code, TRB type, and routing IDs.
    pub(crate) control: u32,
}

impl RawEventTrb {
    /// Extract the TRB type field from the control dword.
    pub(crate) fn trb_type(&self) -> u32 {
        (self.control & TRB_TYPE_MASK) >> 10
    }

    /// Extract the completion code reported by hardware.
    pub(crate) fn completion_code(&self) -> u32 {
        (self.status & TRB_COMPLETION_CODE_MASK) >> 24
    }

    /// Slot identifier reported alongside the event (lower 8 bits of the control dword).
    pub(crate) fn slot_id(&self) -> u8 {
        (self.control & 0xFF) as u8
    }

    /// Endpoint identifier associated with the event.
    pub(crate) fn endpoint_id(&self) -> u8 {
        ((self.control >> 16) & 0xFF) as u8
    }

    /// Remaining bytes reported in the status dword for transfer events.
    pub(crate) fn residual_length(&self) -> u32 {
        self.status & 0x00FF_FFFF
    }

    /// Port identifier reported by a port-status-change event (if present).
    pub(crate) fn port_id(&self) -> Option<u8> {
        let id = (self.parameter >> 24) as u8;
        if id == 0 {
            None
        } else {
            Some(id)
        }
    }
}

