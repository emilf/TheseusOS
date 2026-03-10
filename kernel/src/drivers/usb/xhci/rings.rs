//! TRB ring helpers for the xHCI driver.
//!
//! Encapsulates command, transfer, and event rings plus a tiny recycler so
//! DMA buffers can be reused across controller restarts during bring-up.

use alloc::vec;
use alloc::vec::Vec;
use spin::Mutex;

use crate::log_trace;
use crate::memory::dma::DmaBuffer;

use super::{TRB_CYCLE_BIT, TRB_LINK_TOGGLE_CYCLE, TRB_SIZE};

/// Cache bucket keyed by buffer size.
struct RingCache {
    size: usize,
    buffers: Vec<DmaBuffer>,
}

/// Thread-safe recycler that hands out zeroed DMA buffers for TRB rings.
pub(crate) struct RingRecycler {
    caches: Vec<RingCache>,
}

impl RingRecycler {
    /// Create an empty recycler.
    pub const fn new() -> Self {
        Self { caches: Vec::new() }
    }

    /// Fetch a buffer with the exact size if one is cached.
    pub fn acquire(&mut self, size: usize) -> Option<DmaBuffer> {
        self.caches
            .iter_mut()
            .find(|entry| entry.size == size)
            .and_then(|entry| entry.buffers.pop())
    }

    /// Return a buffer to the recycler for future reuse.
    pub fn recycle(&mut self, buffer: DmaBuffer) {
        let size = buffer.len();
        if let Some(entry) = self.caches.iter_mut().find(|entry| entry.size == size) {
            entry.buffers.push(buffer);
        } else {
            self.caches.push(RingCache {
                size,
                buffers: vec![buffer],
            });
        }
    }
}

pub(crate) static RING_RECYCLER: Mutex<RingRecycler> = Mutex::new(RingRecycler::new());

fn recycle_ring_buffer(buffer: DmaBuffer, tag: &str) {
    let mut recycler = RING_RECYCLER.lock();
    let len = buffer.len();
    recycler.recycle(buffer);
    drop(recycler);
    log_trace!("xHCI recycled {} ring buffer ({} bytes)", tag, len);
}

/// Software producer view of the command ring shared with the controller.
///
/// The driver writes TRBs into this ring and rings the host doorbell so the
/// controller can consume the entries. The controller toggles the cycle bit
/// when it wraps, so we track both the current index and producer cycle.
pub(crate) struct CommandRing {
    buffer: Option<DmaBuffer>,
    pub(crate) index: usize,
    pub(crate) cycle: bool,
}

impl CommandRing {
    /// Construct a command ring over the provided buffer.
    pub(crate) fn new(buffer: DmaBuffer) -> Self {
        Self {
            buffer: Some(buffer),
            index: 0,
            cycle: true,
        }
    }

    /// Physical base of the underlying DMA buffer.
    pub(crate) fn phys_addr(&self) -> u64 {
        self.buffer
            .as_ref()
            .expect("command ring missing buffer")
            .phys_addr()
    }

    /// Pointer to the next TRB slot to be written by software.
    pub(crate) fn current_trb(&self) -> *mut u32 {
        let buffer = self
            .buffer
            .as_ref()
            .expect("command ring buffer missing")
            .virt_addr();
        (buffer + (self.index * TRB_SIZE) as u64) as *mut u32
    }

    /// Cycle bit value to encode in the TRB control dword.
    pub(crate) fn cycle_bit_u32(&self) -> u32 {
        if self.cycle {
            TRB_CYCLE_BIT
        } else {
            0
        }
    }

    /// Cycle bit value as u64 for CRCR programming.
    pub(crate) fn cycle_bit_u64(&self) -> u64 {
        if self.cycle {
            TRB_CYCLE_BIT as u64
        } else {
            0
        }
    }

    /// Advance to the next TRB, flipping the cycle bit when wrapping.
    pub(crate) fn advance(&mut self) {
        self.index += 1;
        if self.index == super::COMMAND_RING_TRBS {
            self.index = 0;
            self.cycle = !self.cycle;
        }
    }
}

impl Drop for CommandRing {
    fn drop(&mut self) {
        if let Some(buffer) = self.buffer.take() {
            recycle_ring_buffer(buffer, "command");
        }
    }
}

/// Software producer view of an endpoint transfer ring.
///
/// The ring is shared between the controller (consumer) and the driver
/// (producer). We track the next enqueue slot and toggle the cycle bit whenever
/// the producer wraps back to the beginning.
pub(crate) struct TransferRing {
    buffer: DmaBuffer,
    pub(crate) enqueue_index: usize,
    pub(crate) cycle: bool,
    trb_count: usize,
    link_index: usize,
}

impl TransferRing {
    /// Construct a transfer ring over the provided buffer and seed the mandatory link TRB.
    pub(crate) fn new(buffer: DmaBuffer, trb_count: usize) -> Self {
        assert!(
            trb_count >= 2,
            "transfer ring requires space for at least one link TRB"
        );
        let mut ring = Self {
            buffer,
            enqueue_index: 0,
            cycle: true,
            trb_count,
            link_index: trb_count - 1,
        };
        ring.initialise_link_trb();
        ring
    }

    /// Physical base of the underlying DMA buffer.
    pub(crate) fn phys_addr(&self) -> u64 {
        self.buffer.phys_addr()
    }

    /// TR dequeue pointer advertised in endpoint contexts (cycle in context).
    pub(crate) fn dequeue_pointer(&self) -> u64 {
        self.buffer.phys_addr() & !0xF
    }

    /// Total number of TRBs the ring can hold.
    pub(crate) fn capacity(&self) -> usize {
        self.link_index
    }

    /// Total TRBs including the reserved link entry.
    pub(crate) fn total_trbs(&self) -> usize {
        self.trb_count
    }

    /// Current producer cycle state; used to seed the consumer's DCS bit.
    pub(crate) fn cycle_state(&self) -> bool {
        self.cycle
    }

    /// Append a TRB and advance the producer index, flipping the cycle on wrap.
    #[allow(dead_code)]
    pub(crate) fn enqueue(&mut self, parameter: u64, status: u32, control: u32) {
        if self.enqueue_index >= self.link_index {
            self.enqueue_index = 0;
            self.cycle = !self.cycle;
            self.initialise_link_trb();
        }

        let trb_ptr = self.trb_ptr(self.enqueue_index);
        unsafe {
            core::ptr::write_volatile(trb_ptr, parameter as u32);
            core::ptr::write_volatile(trb_ptr.add(1), (parameter >> 32) as u32);
            core::ptr::write_volatile(trb_ptr.add(2), status);
            core::ptr::write_volatile(
                trb_ptr.add(3),
                control | if self.cycle { TRB_CYCLE_BIT } else { 0 },
            );
        }

        self.enqueue_index += 1;

        if self.enqueue_index >= self.link_index {
            self.enqueue_index = 0;
            self.cycle = !self.cycle;
            self.initialise_link_trb();
        }
    }

    /// Pointer to a TRB slot by index.
    pub(crate) fn trb_ptr(&self, index: usize) -> *mut u32 {
        (self.buffer.virt_addr() + (index * TRB_SIZE) as u64) as *mut u32
    }

    /// Program the reserved link TRB so hardware wraps back to the ring base.
    fn initialise_link_trb(&mut self) {
        let link_ptr = self.trb_ptr(self.link_index);
        let target = self.buffer.phys_addr() & !0xF;
        unsafe {
            // Link TRB points back to the start of the ring and advertises the
            // producer cycle so the controller can detect wrap-around.
            core::ptr::write_volatile(link_ptr, target as u32);
            core::ptr::write_volatile(link_ptr.add(1), (target >> 32) as u32);
            core::ptr::write_volatile(link_ptr.add(2), 0);
            let mut control = (super::TRB_TYPE_LINK << 10) | TRB_LINK_TOGGLE_CYCLE;
            if self.cycle {
                control |= TRB_CYCLE_BIT;
            }
            core::ptr::write_volatile(link_ptr.add(3), control);
        }
    }
}

/// Minimal wrapper around the controller’s event ring and its ERST entry.
pub(crate) struct EventRing {
    pub(crate) buffer: Option<DmaBuffer>,
    pub(crate) table: Option<DmaBuffer>,
    pub(crate) index: usize,
    pub(crate) cycle: bool,
}

impl EventRing {
    /// Construct a new event ring wrapper with its associated ERST entry.
    pub(crate) fn new(buffer: DmaBuffer, table: DmaBuffer) -> Self {
        Self {
            buffer: Some(buffer),
            table: Some(table),
            index: 0,
            cycle: true,
        }
    }

    /// Physical base of the event ring buffer.
    pub(crate) fn phys_addr(&self) -> u64 {
        self.buffer
            .as_ref()
            .expect("event ring missing buffer")
            .phys_addr()
    }

    /// Physical base of the ERST table.
    pub(crate) fn table_phys_addr(&self) -> u64 {
        self.table
            .as_ref()
            .expect("event table missing")
            .phys_addr()
    }

    /// Virtual base of the ERST table.
    pub(crate) fn table_virt_addr(&self) -> u64 {
        self.table
            .as_ref()
            .expect("event table missing")
            .virt_addr()
    }

    /// Pointer to the current TRB slot.
    pub(crate) fn current_trb(&self) -> *mut u32 {
        let buffer = self
            .buffer
            .as_ref()
            .expect("event ring buffer not initialised");
        (buffer.virt_addr() + (self.index * TRB_SIZE) as u64) as *mut u32
    }

    /// Advance to the next TRB, toggling cycle state on wrap.
    pub(crate) fn advance(&mut self) {
        self.index += 1;
        if self.index >= super::EVENT_RING_TRBS {
            self.index = 0;
            self.cycle = !self.cycle;
        }
    }
}

impl Drop for EventRing {
    fn drop(&mut self) {
        if let Some(buffer) = self.buffer.take() {
            recycle_ring_buffer(buffer, "event");
        }
        if let Some(table) = self.table.take() {
            recycle_ring_buffer(table, "erst");
        }
    }
}

