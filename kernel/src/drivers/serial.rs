//! # Serial Driver - 16550 UART Driver for COM1
//!
//! This module implements a hardware driver for the 16550 UART (Universal Asynchronous
//! Receiver/Transmitter) commonly found on x86 systems as the COM1 serial port. The driver
//! integrates with the kernel's generic driver framework and provides both interrupt-driven
//! reception and polled transmission.
//!
//! ## Hardware Overview
//!
//! ### 16550 UART
//!
//! The 16550 is a classic serial communication chip that dates back to the 1990s but is
//! still widely supported via emulation in modern systems and hypervisors. It provides:
//!
//! - Asynchronous serial communication (RS-232)
//! - Configurable baud rate (we use 115200 bps)
//! - 16-byte transmit and receive FIFOs
//! - Interrupt generation on data received/transmitted
//! - Support for various data formats (we use 8N1: 8 data bits, No parity, 1 stop bit)
//!
//! ### I/O Port Map
//!
//! The COM1 port uses I/O base address 0x3F8 with the following register layout:
//!
//! | Offset | DLAB=0      | DLAB=1           | Access |
//! |--------|-------------|------------------|--------|
//! | +0     | RBR/THR     | Divisor Latch Lo | R/W    |
//! | +1     | IER         | Divisor Latch Hi | R/W    |
//! | +2     | IIR/FCR     | IIR/FCR          | R/W    |
//! | +3     | LCR         | LCR              | R/W    |
//! | +4     | MCR         | MCR              | R/W    |
//! | +5     | LSR         | LSR              | R      |
//! | +6     | MSR         | MSR              | R      |
//! | +7     | SCR         | SCR              | R/W    |
//!
//! Legend:
//! - **DLAB**: Divisor Latch Access Bit (bit 7 of LCR)
//! - **RBR**: Receiver Buffer Register (read)
//! - **THR**: Transmitter Holding Register (write)
//! - **IER**: Interrupt Enable Register
//! - **IIR**: Interrupt Identification Register (read)
//! - **FCR**: FIFO Control Register (write)
//! - **LCR**: Line Control Register
//! - **MCR**: Modem Control Register
//! - **LSR**: Line Status Register
//! - **MSR**: Modem Status Register
//! - **SCR**: Scratch Register
//!
//! ## Operation Modes
//!
//! ### Transmission (Polled)
//!
//! The driver uses polled mode for transmission:
//! 1. Check LSR bit 5 (Transmitter Holding Register Empty)
//! 2. If set, write byte to THR (offset +0)
//! 3. Repeat for each byte
//!
//! This is simple and works well for the low-volume output typical of a debug console.
//!
//! ### Reception (Interrupt-Driven)
//!
//! The driver uses interrupt mode for reception:
//! 1. Enable "Received Data Available" interrupt in IER
//! 2. Configure IOAPIC to route IRQ 4 to a kernel interrupt vector
//! 3. When data arrives, hardware asserts IRQ 4
//! 4. Interrupt handler reads all available bytes from RBR
//! 5. Bytes are stored in a circular buffer for application consumption
//!
//! This approach ensures no data is lost even during CPU-intensive operations.
//!
//! ## Circular Buffer
//!
//! Received data is stored in a 1024-byte circular buffer:
//!
//! ```text
//!     Tail (read position)              Head (write position)
//!          ↓                                  ↓
//!     [....##################....................]
//!          ^--- Data ready to read ---^
//! ```
//!
//! - **Head**: Write position (modified by IRQ handler)
//! - **Tail**: Read position (modified by read() calls)
//! - **Empty**: head == tail
//! - **Full**: (head + 1) % size == tail
//! - **Overflow**: Oldest data is overwritten when buffer fills
//!
//! The head and tail are atomic variables, enabling lock-free operation between the
//! IRQ handler (producer) and application (consumer).
//!
//! ## Driver Framework Integration
//!
//! This driver implements the `Driver` trait and registers with the kernel's driver
//! manager. It provides:
//!
//! - **Probe**: Checks if device is a Serial class device
//! - **Init**: Configures UART hardware and enables interrupts
//! - **IRQ Handler**: Reads received data and enqueues it
//! - **Read**: Dequeues data from circular buffer
//! - **Write**: Transmits data via polled THR writes
//!
//! ## Configuration
//!
//! Key constants (see `theseus_shared::constants`):
//! - Baud rate: 115200 (divisor = 1 for typical UART clock)
//! - Data format: 8N1 (8 data bits, no parity, 1 stop bit)
//! - FIFO trigger: 14 bytes
//! - IRQ: 4 (legacy ISA IRQ for COM1)
//! - I/O base: 0x3F8
//!
//! ## Thread Safety
//!
//! - `SERIAL_STATE`: Protected by Mutex for hardware access
//! - Circular buffer: Lock-free atomics for head/tail
//! - IRQ handler: Can safely interleave with application reads
//!
//! ## References
//!
//! - [16550 UART Datasheet](http://caro.su/msx/ocm_de1/16550.pdf)
//! - [OSDev Wiki - Serial Ports](https://wiki.osdev.org/Serial_Ports)

use alloc::vec::Vec;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use spin::Mutex;
use x86_64::instructions::port::Port;

use crate::drivers::manager::driver_manager;
use crate::drivers::traits::{Device, DeviceClass, Driver};
use crate::interrupts;
use crate::kernel_write_line;
use crate::memory;

pub const LEGACY_COM1_IRQ: u32 = 4;
pub const LEGACY_COM1_BASE: u16 = theseus_shared::constants::io_ports::com1::DATA;
const IOAPIC_REGSEL_OFFSET: u64 = 0x00;
const IOAPIC_WINDOW_OFFSET: u64 = 0x10;
const IOAPIC_REDIRECTION_TABLE_BASE: u32 = 0x10;

/// Serial driver state
///
/// This structure contains all the state needed to manage the serial port,
/// including the hardware port abstraction and the circular buffer for
/// received data.
///
/// # Thread Safety
///
/// The state is protected by `SERIAL_STATE` mutex. The circular buffer uses
/// lock-free atomics for head/tail pointers, allowing the IRQ handler to
/// write while the application reads without additional synchronization.
#[allow(dead_code)]
struct SerialDriverState {
    /// Hardware port abstraction
    ///
    /// Provides methods to read/write UART registers via I/O ports.
    port: SerialPort,

    /// Whether interrupts are enabled
    ///
    /// Currently always true after initialization. Could be used to temporarily
    /// disable interrupts if needed.
    irq_enabled: AtomicBool,

    /// Circular buffer for received data
    ///
    /// Protected by mutex for structural access, but the actual buffer is
    /// accessed via lock-free atomics (head/tail indices).
    rx_buffer: spin::Mutex<[u8; RX_BUFFER_SIZE]>,

    /// Write position in circular buffer (modified by IRQ handler)
    ///
    /// The IRQ handler advances this when new data arrives. Uses Release
    /// ordering when updated to ensure data is visible before the index.
    head: AtomicUsize,

    /// Read position in circular buffer (modified by read calls)
    ///
    /// Application code advances this as data is consumed. Uses Acquire
    /// ordering to ensure data is fully visible before being read.
    tail: AtomicUsize,
}

#[derive(Clone, Copy)]
struct IoApicInfo {
    address: u64,
    gsi_base: u32,
}

static SERIAL_DRIVER: SerialDriver = SerialDriver;
static SERIAL_STATE: Mutex<Option<SerialDriverState>> = Mutex::new(None);
static IO_APIC_INFO: Mutex<Option<IoApicInfo>> = Mutex::new(None);

const RX_BUFFER_SIZE: usize = 1024;

pub fn register_serial_driver() {
    driver_manager().lock().register_driver(&SERIAL_DRIVER);
}

pub fn init_serial() {
    if !crate::config::ENABLE_SERIAL_OUTPUT {
        return;
    }
    register_serial_driver();
}

/// Write bytes directly to the serial port, bypassing the driver manager.
/// This is useful from interrupt context to avoid re-entrant locking.
pub fn write_bytes_direct(buf: &[u8]) -> Result<usize, &'static str> {
    let guard = SERIAL_STATE.lock();
    if let Some(state) = guard.as_ref() {
        state.port.write_buffer(buf);
        Ok(buf.len())
    } else {
        Err("serial port not initialized")
    }
}

#[allow(dead_code)]
fn with_serial_state<F, R>(f: F) -> Result<R, &'static str>
where
    F: FnOnce(&SerialDriverState) -> R,
{
    let guard = SERIAL_STATE.lock();
    let state = guard.as_ref().ok_or("serial port not initialized")?;
    Ok(f(state))
}

fn with_serial_state_mut<F, R>(f: F) -> Result<R, &'static str>
where
    F: FnOnce(&mut SerialDriverState) -> R,
{
    let mut guard = SERIAL_STATE.lock();
    let state = guard.as_mut().ok_or("serial port not initialized")?;
    Ok(f(state))
}

struct SerialDriver;

#[derive(Clone, Copy)]
struct SerialPort {
    base: u16,
}

impl SerialPort {
    const DEFAULT_BASE: u16 = LEGACY_COM1_BASE;

    fn new(base: u16) -> Self {
        Self { base }
    }

    fn write_byte(&self, byte: u8) {
        unsafe {
            let mut port = Port::new(self.base);
            port.write(byte);
        }
    }

    fn line_status(&self) -> u8 {
        unsafe {
            let mut port = Port::new(self.base + 5);
            port.read()
        }
    }

    fn write_buffer(&self, buf: &[u8]) {
        for &b in buf {
            while self.line_status() & 0x20 == 0 {}
            self.write_byte(if b == b'\n' { b'\r' } else { b });
            if b == b'\n' {
                while self.line_status() & 0x20 == 0 {}
                self.write_byte(b'\n');
            }
        }
    }

    fn enable_interrupts(&self) {
        unsafe {
            // Enable Received Data Available interrupt.
            let mut int_enable = Port::<u8>::new(self.base + 1);
            int_enable.write(0x01);
            // LSR Trigger Level (FIFO control register)
            let mut fifo_ctrl = Port::<u8>::new(self.base + 2);
            fifo_ctrl.write(0xC7);
            // Modem Control: assert DTR, RTS, OUT1, OUT2 (OUT2 required for interrupts).
            let mut modem_ctrl = Port::<u8>::new(self.base + 4);
            modem_ctrl.write(0x0B);
        }
    }

    fn read_byte(&self) -> Option<u8> {
        unsafe {
            let mut lsr = Port::<u8>::new(self.base + 5);
            if lsr.read() & 0x01 == 0 {
                return None;
            }
            let mut data = Port::<u8>::new(self.base);
            Some(data.read())
        }
    }
}

impl SerialDriver {
    /// Initialize the serial port hardware
    ///
    /// This function programs the 16550 UART registers to configure:
    /// - Baud rate (115200 bps, divisor = 1)
    /// - Data format (8N1: 8 data bits, no parity, 1 stop bit)
    /// - FIFO operation (enabled, 14-byte threshold)
    /// - Modem control signals (OUT2 must be high for interrupts to work)
    ///
    /// # Initialization Sequence
    ///
    /// 1. Disable interrupts (prevent spurious IRQs during init)
    /// 2. Set DLAB=1 to access divisor latch
    /// 3. Program baud rate divisor (low then high byte)
    /// 4. Set DLAB=0 and configure data format (LCR=0x03 for 8N1)
    /// 5. Enable and clear FIFOs (FCR=0xC7)
    /// 6. Set modem control signals (MCR=0x0B: DTR+RTS+OUT2)
    ///
    /// # Arguments
    ///
    /// * `base` - I/O port base address (typically 0x3F8 for COM1)
    ///
    /// # Safety
    ///
    /// Must be called with exclusive access to the hardware. The UART must not
    /// be in use by other code during initialization.
    fn init_port(&self, base: u16) -> Result<SerialPort, &'static str> {
        unsafe {
            let mut data = Port::<u8>::new(base);
            let mut int_enable = Port::<u8>::new(base + 1);
            let mut fifo_ctrl = Port::<u8>::new(base + 2);
            let mut line_ctrl = Port::<u8>::new(base + 3);
            let mut modem_ctrl = Port::<u8>::new(base + 4);

            // Disable interrupts while configuring to prevent spurious IRQs
            int_enable.write(0x00);

            // Enable DLAB (bit 7 of LCR) to access baud rate divisor registers
            line_ctrl.write(0x80);

            // Program baud rate divisor
            // Divisor = 115200 / desired_baud_rate
            // For 115200 bps, divisor = 1
            let divisor = theseus_shared::constants::hardware::COM1_BAUD_DIVISOR;
            data.write((divisor & 0xFF) as u8); // Divisor low byte
            int_enable.write((divisor >> 8) as u8); // Divisor high byte

            // Configure line control: 8 data bits, no parity, 1 stop bit
            // Also clears DLAB to access normal registers again
            // LCR format: bit 0-1: word length (11=8 bits)
            //            bit 2: stop bits (0=1 stop bit)
            //            bit 3-5: parity (000=none)
            //            bit 6: break control (0=disabled)
            //            bit 7: DLAB (0=normal operation)
            line_ctrl.write(0x03);

            // Configure FIFO Control Register
            // Bit 0: Enable FIFO
            // Bit 1: Clear receive FIFO
            // Bit 2: Clear transmit FIFO
            // Bit 6-7: Interrupt trigger level (11 = 14 bytes)
            fifo_ctrl.write(0xC7);

            // Configure Modem Control Register
            // Bit 0: DTR (Data Terminal Ready) - asserted
            // Bit 1: RTS (Request To Send) - asserted
            // Bit 2: OUT1 - not used
            // Bit 3: OUT2 - MUST be high for interrupts to reach CPU
            // Bit 4: Loopback mode - disabled
            modem_ctrl.write(0x0B); // DTR=1, RTS=1, OUT2=1
        }

        Ok(SerialPort::new(base))
    }

    fn configure_irq_route(&self, dev: &Device) -> Result<(), &'static str> {
        let info_opt = {
            let guard = IO_APIC_INFO.lock();
            *guard
        };
        let Some(info) = info_opt else {
            return Err("no IO APIC information available");
        };

        let gsi = dev.irq.unwrap_or(LEGACY_COM1_IRQ) as u32;
        if gsi < info.gsi_base {
            return Err("serial GSI below IO APIC base");
        }
        let pin = gsi - info.gsi_base;

        unsafe {
            program_io_apic_entry(
                info.address,
                pin,
                interrupts::SERIAL_RX_VECTOR,
                current_apic_id(),
            );
        }

        Ok(())
    }
}

impl Driver for SerialDriver {
    fn supported_classes(&self) -> &'static [DeviceClass] {
        &[DeviceClass::Serial]
    }

    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        match dev.class {
            DeviceClass::Serial => Ok(()),
            _ => Err("not serial"),
        }
    }

    fn init(&'static self, dev: &mut Device) -> Result<(), &'static str> {
        if !crate::config::ENABLE_SERIAL_OUTPUT {
            return Err("serial output disabled");
        }
        let base = dev
            .phys_addr
            .map(|addr| addr as u16)
            .unwrap_or(SerialPort::DEFAULT_BASE);
        let port = self.init_port(base)?;

        if dev.phys_addr.is_none() {
            dev.phys_addr = Some(base as u64);
        }
        if dev.irq.is_none() {
            dev.irq = Some(LEGACY_COM1_IRQ);
        }

        if let Err(err) = self.configure_irq_route(dev) {
            kernel_write_line("[serial] failed to route IO APIC entry");
            kernel_write_line(err);
        } else {
            kernel_write_line("[serial] IO APIC route configured");
        }

        port.enable_interrupts();
        let state = SerialDriverState {
            port,
            irq_enabled: AtomicBool::new(true),
            rx_buffer: spin::Mutex::new([0u8; RX_BUFFER_SIZE]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        };
        {
            let mut guard = SERIAL_STATE.lock();
            *guard = Some(state);
            if let Some(stored) = guard.as_ref() {
                dev.driver_data = Some(stored as *const SerialDriverState as usize);
            }
        }
        kernel_write_line("[serial] initialized");
        Ok(())
    }

    fn irq_handler(&'static self, _dev: &mut Device, _irq: u32) -> bool {
        if !crate::config::ENABLE_SERIAL_OUTPUT {
            return false;
        }
        match with_serial_state_mut(|state| {
            let mut handled = false;
            let mut collected = Vec::new();
            while let Some(byte) = state.port.read_byte() {
                handled = true;
                enqueue_byte(state, byte);
                collected.push(byte);
            }
            (handled, collected)
        }) {
            Ok((handled, collected)) => {
                for byte in collected {
                    crate::monitor::push_serial_byte(byte);
                }
                handled
            }
            Err(_) => false,
        }
    }

    fn write(&'static self, _dev: &mut Device, buf: &[u8]) -> Result<usize, &'static str> {
        write_bytes_direct(buf)
    }

    fn read(&'static self, _dev: &mut Device, buf: &mut [u8]) -> Result<usize, &'static str> {
        if buf.is_empty() {
            return Ok(0);
        }
        with_serial_state_mut(|state| {
            fill_rx_buffer(state);
            dequeue_bytes(state, buf)
        })
    }
}

/// Enqueue a byte into the circular buffer
///
/// This function is called from the IRQ handler to store received data. It implements
/// a circular buffer with overflow handling: when the buffer is full, the oldest data
/// is discarded to make room for new data.
///
/// # Algorithm
///
/// 1. Calculate next head position: (head + 1) % size
/// 2. If next_head == tail, buffer is full:
///    - Write byte at current head
///    - Advance both head and tail (discard oldest byte)
/// 3. Otherwise:
///    - Write byte at current head
///    - Advance head only
///
/// # Memory Ordering
///
/// - Tail: Acquire (ensure we see latest consumer position)
/// - Head: Release after write (ensure byte is visible before index update)
///
/// # Arguments
///
/// * `state` - Mutable reference to serial driver state
/// * `byte` - Byte to enqueue
fn enqueue_byte(state: &mut SerialDriverState, byte: u8) {
    let mut buf = state.rx_buffer.lock();
    let head = state.head.load(Ordering::Relaxed);
    let tail = state.tail.load(Ordering::Acquire);
    let next_head = (head + 1) % RX_BUFFER_SIZE;

    if next_head == tail {
        // Buffer is full - overwrite oldest byte
        // This is a trade-off: we prioritize recent data over old data
        buf[head] = byte;
        state.head.store(next_head, Ordering::Release);
        // Also advance tail to skip the overwritten byte
        state
            .tail
            .store((tail + 1) % RX_BUFFER_SIZE, Ordering::Release);
    } else {
        // Buffer has space - normal enqueue
        buf[head] = byte;
        state.head.store(next_head, Ordering::Release);
    }
}

/// Dequeue bytes from the circular buffer
///
/// This function is called from the read() method to retrieve received data for
/// the application. It copies as many bytes as possible (up to the output buffer
/// size or the number of available bytes, whichever is smaller) and advances the
/// tail pointer.
///
/// # Algorithm
///
/// 1. Check if buffer is empty (head == tail)
/// 2. Copy bytes from tail position up to head
/// 3. Wrap around if necessary (circular buffer)
/// 4. Update tail pointer after all copies complete
///
/// # Memory Ordering
///
/// - Head: Acquire (ensure we see all enqueued data)
/// - Tail: Acquire before loop, Release after (synchronize with producer)
///
/// # Arguments
///
/// * `state` - Reference to serial driver state
/// * `out` - Output buffer to fill with received data
///
/// # Returns
///
/// Number of bytes actually copied to output buffer
fn dequeue_bytes(state: &SerialDriverState, out: &mut [u8]) -> usize {
    let buf_lock = state.rx_buffer.lock();
    let mut tail = state.tail.load(Ordering::Acquire);
    let head = state.head.load(Ordering::Acquire);

    // Buffer is empty
    if head == tail {
        return 0;
    }

    let mut read = 0usize;

    // Copy bytes from buffer to output
    // Stop when we've read everything OR filled the output buffer
    while tail != head && read < out.len() {
        out[read] = buf_lock[tail];
        tail = (tail + 1) % RX_BUFFER_SIZE;
        read += 1;
    }

    // Update tail pointer after all reads complete
    // Release ordering ensures the producer sees the updated tail
    state.tail.store(tail, Ordering::Release);

    read
}

/// Fill the RX buffer by reading all available bytes from the UART
///
/// This helper function polls the Line Status Register (LSR) and reads the
/// Receive Buffer Register (RBR) until no more data is available. It's used
/// to opportunistically drain the UART FIFO before a read() call.
///
/// This is useful because the UART has a 16-byte hardware FIFO, and we want
/// to transfer all available data to our larger software buffer in one go.
///
/// # Arguments
///
/// * `state` - Mutable reference to serial driver state
fn fill_rx_buffer(state: &mut SerialDriverState) {
    // Keep reading while data is available
    // read_byte() checks LSR bit 0 (Data Ready) before reading
    while let Some(byte) = state.port.read_byte() {
        enqueue_byte(state, byte);
    }
}

pub fn install_io_apic_info(address: u64, gsi_base: u32) {
    let mut guard = IO_APIC_INFO.lock();
    *guard = Some(IoApicInfo { address, gsi_base });
}

pub fn current_irq_number() -> Option<u32> {
    let guard = IO_APIC_INFO.lock();
    guard.as_ref().map(|info| info.gsi_base + LEGACY_COM1_IRQ)
}

/// Program an IOAPIC redirection table entry to route an IRQ
///
/// The IOAPIC (I/O Advanced Programmable Interrupt Controller) is responsible for
/// receiving hardware interrupt signals and routing them to LAPIC (Local APIC) cores.
/// Each input pin has a 64-bit redirection entry that configures how the interrupt
/// is delivered.
///
/// # IOAPIC Register Access
///
/// The IOAPIC is accessed via two MMIO registers:
/// - **IOREGSEL** (offset 0x00): Select which internal register to access
/// - **IOWIN** (offset 0x10): Read/write the selected register
///
/// To access a redirection entry:
/// 1. Write the entry index to IOREGSEL
/// 2. Read or write the data via IOWIN
///
/// Each redirection entry is 64 bits (requires two 32-bit accesses):
/// - Lower 32 bits (index 0x10 + pin*2): Vector, delivery mode, flags
/// - Upper 32 bits (index 0x11 + pin*2): Destination APIC ID
///
/// # Redirection Entry Format (Lower 32 bits)
///
/// ```text
/// Bit(s)  | Field              | Value for Serial
/// --------|-------------------|------------------
/// 0-7     | Interrupt Vector  | 0x24 (SERIAL_RX_VECTOR)
/// 8-10    | Delivery Mode     | 000 (Fixed)
/// 11      | Destination Mode  | 0 (Physical)
/// 12      | Delivery Status   | 0 (Idle)
/// 13      | Pin Polarity      | 0 (Active High)
/// 14      | Remote IRR        | 0 (Edge)
/// 15      | Trigger Mode      | 0 (Edge)
/// 16      | Mask              | 0 (Unmasked)
/// 17-31   | Reserved          | 0
/// ```
///
/// # Redirection Entry Format (Upper 32 bits)
///
/// ```text
/// Bit(s)  | Field              | Value
/// --------|-------------------|------------------
/// 0-23    | Reserved          | 0
/// 24-31   | Destination       | APIC ID of target CPU
/// ```
///
/// # Arguments
///
/// * `address` - Physical address of IOAPIC MMIO region
/// * `pin` - IOAPIC input pin number (0-23 typically)
/// * `vector` - Interrupt vector to deliver to CPU (0x20-0xFF)
/// * `destination_apic_id` - APIC ID of target CPU core
///
/// # Safety
///
/// This function is unsafe because:
/// - It performs raw MMIO writes to hardware registers
/// - Invalid parameters can cause system instability
/// - Must be called with correct physical address mapping
unsafe fn program_io_apic_entry(address: u64, pin: u32, vector: u8, destination_apic_id: u8) {
    // Convert physical address to virtual address for MMIO access
    let base = memory::phys_to_virt_pa(address);
    let regsel = (base + IOAPIC_REGSEL_OFFSET) as *mut u32;
    let window = (base + IOAPIC_WINDOW_OFFSET) as *mut u32;

    // Calculate redirection entry indices
    // Each entry is 64 bits, accessed as two 32-bit registers
    let lower_index = IOAPIC_REDIRECTION_TABLE_BASE + pin * 2;
    let upper_index = lower_index + 1;

    // ===== Configure Lower 32 Bits =====

    // Select the lower dword of the redirection entry
    ptr::write_volatile(regsel, lower_index);
    let mut lower = ptr::read_volatile(window);

    // Set interrupt vector (bits 0-7)
    lower &= !0xFF; // Clear existing vector
    lower |= vector as u32; // Set new vector

    // Set delivery mode to Fixed (bits 8-10 = 000)
    // Fixed mode delivers the interrupt to the specified CPU
    lower &= !(0x7 << 8);

    // Set destination mode to Physical (bit 11 = 0)
    // Physical mode uses APIC ID for routing
    lower &= !(1 << 11);

    // Set pin polarity to Active High (bit 13 = 0)
    // Most ISA interrupts are active high
    lower &= !(1 << 13);

    // Set trigger mode to Edge (bit 15 = 0)
    // Legacy ISA interrupts typically use edge triggering
    lower &= !(1 << 15);

    // Unmask the interrupt (bit 16 = 0)
    // Clearing this bit enables the interrupt
    lower &= !(1 << 16);

    // Write back the configured lower dword
    ptr::write_volatile(window, lower);

    // ===== Configure Upper 32 Bits =====

    // Select the upper dword of the redirection entry
    ptr::write_volatile(regsel, upper_index);
    let mut upper = ptr::read_volatile(window);

    // Set destination APIC ID (bits 24-31)
    upper &= !(0xFF << 24); // Clear existing destination
    upper |= (destination_apic_id as u32) << 24; // Set new destination

    // Write back the configured upper dword
    ptr::write_volatile(window, upper);
}

fn current_apic_id() -> u8 {
    unsafe {
        let base = interrupts::get_apic_base();
        (interrupts::read_apic_register(base, 0x20) >> 24) as u8
    }
}
