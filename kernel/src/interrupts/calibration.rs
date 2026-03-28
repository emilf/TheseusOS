//! LAPIC timer calibration using the PIT (8254) as a reference clock.
//!
//! The PIT channel 2 is used to gate a precise timing window (~10 ms),
//! during which we count LAPIC timer decrements. This gives us the
//! LAPIC timer frequency in ticks per millisecond.
//!
//! After calibration, the LAPIC timer is configured for a 100 Hz periodic tick.

use crate::{log_debug, log_info};
use core::sync::atomic::{AtomicU64, Ordering};
use x86_64::instructions::port::Port;

use super::apic::{local_apic_read, local_apic_write};

/// PIT counts for ~10 ms gate: 1_193_182 * 10 / 1000 ≈ 11932
const PIT_10MS_COUNT: u16 = 11_932;

/// Calibrated LAPIC timer ticks per millisecond.
static APIC_TICKS_PER_MS: AtomicU64 = AtomicU64::new(0);

/// Monotonic tick counter incremented by the periodic timer ISR.
static TICK_COUNT: AtomicU64 = AtomicU64::new(0);

/// Calibrate the LAPIC timer using PIT channel 2 as a reference.
///
/// Returns ticks per millisecond for the LAPIC timer (at the current divider).
///
/// # Safety
/// Must be called with interrupts disabled and LAPIC configured (divider set).
pub unsafe fn calibrate_apic_timer() -> u64 {
    // Use PIT channel 2 + port 0x61 gate for the timing window.
    // Channel 2 is not connected to IRQ 0, so we can poll its output bit.
    let mut port_61: Port<u8> = Port::new(0x61);
    let mut pit_cmd: Port<u8> = Port::new(0x43);
    let mut pit_ch2: Port<u8> = Port::new(0x42);

    // Enable PIT channel 2 gate (bit 0 of port 0x61) and disable speaker (bit 1)
    let gate = port_61.read();
    port_61.write((gate & 0xFC) | 0x01); // gate on, speaker off

    // Program PIT channel 2: mode 0 (interrupt on terminal count), binary, lobyte/hibyte
    // Command: channel=2 (bits 7:6 = 10), access=lobyte/hibyte (bits 5:4 = 11),
    //          mode=0 (bits 3:1 = 000), binary (bit 0 = 0)
    pit_cmd.write(0b10_11_000_0);
    pit_ch2.write((PIT_10MS_COUNT & 0xFF) as u8);
    pit_ch2.write((PIT_10MS_COUNT >> 8) as u8);

    // Set LAPIC timer to a large initial count in one-shot mode
    local_apic_write(0x380, 0xFFFF_FFFF);

    // Wait for PIT channel 2 output (bit 5 of port 0x61) to go high
    loop {
        let status = port_61.read();
        if status & 0x20 != 0 {
            break;
        }
    }

    // Read LAPIC timer current count
    let remaining = local_apic_read(0x390);
    let elapsed = 0xFFFF_FFFFu32.wrapping_sub(remaining);

    // elapsed is ticks in ~10 ms, so ticks_per_ms = elapsed / 10
    let ticks_per_ms = (elapsed as u64) / 10;

    // Mask the LAPIC timer after calibration
    let lvt = local_apic_read(0x320);
    local_apic_write(0x320, lvt | (1 << 16));

    // Restore port 0x61 to disable PIT channel 2 gate
    port_61.write(gate);

    log_info!("LAPIC timer calibration: {} ticks/ms", ticks_per_ms);
    APIC_TICKS_PER_MS.store(ticks_per_ms, Ordering::Relaxed);

    ticks_per_ms
}

/// Start the LAPIC timer as a 100 Hz periodic tick using the calibrated frequency.
///
/// # Safety
/// `calibrate_apic_timer()` must have been called first.
pub unsafe fn start_periodic_tick() {
    let ticks_per_ms = APIC_TICKS_PER_MS.load(Ordering::Relaxed);
    if ticks_per_ms == 0 {
        log_debug!("LAPIC timer not calibrated, using fallback count for periodic tick");
        // Fallback: use 50000 as before
        super::lapic_timer_start_periodic(50_000);
        return;
    }

    // 100 Hz = 10 ms per tick
    let initial_count = (ticks_per_ms * 10) as u32;
    log_info!(
        "Starting 100 Hz periodic tick (initial_count={})",
        initial_count
    );
    super::lapic_timer_start_periodic(initial_count);
}

/// Increment the tick counter. Called from the timer ISR.
pub fn tick() {
    TICK_COUNT.fetch_add(1, Ordering::Relaxed);
}

/// Return the current monotonic tick count.
pub fn current_tick() -> u64 {
    TICK_COUNT.load(Ordering::Relaxed)
}

/// Convert ticks to milliseconds using the calibrated frequency.
pub fn ticks_to_ms(ticks: u64) -> u64 {
    let tpm = APIC_TICKS_PER_MS.load(Ordering::Relaxed);
    if tpm == 0 {
        return 0;
    }
    // Each tick is 10 ms at 100 Hz
    ticks * 10
}

/// Return the calibrated ticks-per-ms value.
pub fn apic_ticks_per_ms() -> u64 {
    APIC_TICKS_PER_MS.load(Ordering::Relaxed)
}
