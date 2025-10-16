//! LAPIC timer management
//!
//! This module provides functions for configuring and controlling the Local APIC timer:
//! - Timer initialization and configuration
//! - One-shot and periodic modes
//! - Timer masking/unmasking
//! - Tick counting
//!
//! ## LAPIC Timer Overview
//!
//! The Local APIC timer is a per-CPU programmable timer that can generate
//! periodic interrupts. It's commonly used for:
//! - OS scheduler preemption
//! - Profiling and performance counters
//! - Periodic system tasks
//! - Animation updates (like our heart animation!)
//!
//! ## Timer Modes
//!
//! - **One-shot**: Timer fires once after countdown, then stops
//! - **Periodic**: Timer fires repeatedly at fixed intervals
//! - **TSC-Deadline**: Timer fires when TSC reaches deadline (not used here)
//!
//! ## Register Offsets
//!
//! - 0x320: LVT Timer (vector, mode, mask)
//! - 0x380: Initial Count (reload value)
//! - 0x390: Current Count (decrements to zero)
//! - 0x3E0: Divide Configuration (clock divider)

use crate::{log_debug, log_trace};
use core::sync::atomic::Ordering;
use super::{get_apic_base, read_apic_register, write_apic_register};
use super::{out_char_0xe9, print_hex_u64_0xe9, print_str_0xe9};
use super::{APIC_TIMER_VECTOR, TIMER_TICKS};
use super::handler_timer;
use super::apic::APIC_ERROR_VECTOR;

/// Configure LAPIC timer (does not start it)
///
/// Initializes the LAPIC timer for use but leaves it masked (stopped).
/// Must be called after paging is active so APIC MMIO is accessible.
///
/// # Configuration
/// - Task Priority Register (TPR) = 0 (allow all interrupts)
/// - Spurious Interrupt Vector = 0xFF with APIC enabled (bit 8)
/// - Timer Divide = /16 (provides good granularity)
/// - LVT Timer = vector 0x40, masked, one-shot mode (default)
/// - LINT0/LINT1 = masked
/// - Error Vector = 0xFE
///
/// # Safety
/// Requires:
/// - Paging to be active
/// - LAPIC MMIO region mapped
/// - Called from kernel mode
///
/// # Examples
/// ```rust
/// unsafe {
///     lapic_timer_configure();
///     lapic_timer_start_periodic(100_000);  // Start 100Hz timer
/// }
/// ```
pub unsafe fn lapic_timer_configure() {
    let apic_base = get_apic_base();
    
    log_debug!("LAPIC TPR=0");
    // Set Task Priority Register to 0 (allow all interrupt priorities)
    write_apic_register(apic_base, 0x80, 0x00);
    
    // Enable APIC via SIVR (Spurious Interrupt Vector Register)
    // Bits 0-7: Spurious vector (0xFF)
    // Bit 8: APIC Software Enable (1 = enabled)
    let siv = (read_apic_register(apic_base, 0xF0) & !0xFF) | 0xFF | 0x100;
    write_apic_register(apic_base, 0xF0, siv);
    
    // Set Timer Divide Configuration to /16
    // This divides the bus frequency by 16 before counting
    log_debug!("LAPIC set divide");
    write_apic_register(apic_base, 0x3E0, 0x3);  // 0x3 = divide by 16
    
    // Program LVT Timer register
    // Bits 0-7: Vector number (0x40)
    // Bit 16: Mask (1 = masked/disabled)
    // Bit 17: Timer mode (0 = one-shot, 1 = periodic)
    log_debug!("LAPIC program LVT timer (masked)");
    let lvt_timer = (APIC_TIMER_VECTOR as u32) | (1 << 16);  // Masked one-shot
    write_apic_register(apic_base, 0x320, lvt_timer);
    
    // Mask external interrupts LINT0 and LINT1 (legacy INT/NMI pins)
    let mut lint0 = read_apic_register(apic_base, 0x350);
    lint0 |= 1 << 16;  // Set mask bit
    write_apic_register(apic_base, 0x350, lint0);
    
    let mut lint1 = read_apic_register(apic_base, 0x360);
    lint1 |= 1 << 16;  // Set mask bit
    write_apic_register(apic_base, 0x360, lint1);
    
    // Set error LVT vector
    let lvt_err = (read_apic_register(apic_base, 0x370) & !0xFF) | (APIC_ERROR_VECTOR as u32);
    write_apic_register(apic_base, 0x370, lvt_err);
    
    // Clear Error Status Register (ESR) by writing then reading
    write_apic_register(apic_base, 0x280, 0);
    let esr = read_apic_register(apic_base, 0x280);
    
    // Read and display APIC configuration for debugging
    let id = read_apic_register(apic_base, 0x20);
    let ver = read_apic_register(apic_base, 0x30);
    let sivr = read_apic_register(apic_base, 0xF0);
    let tpr = read_apic_register(apic_base, 0x80);
    let lvt = read_apic_register(apic_base, 0x320);
    
    log_trace!(
        "LAPIC registers: ID={:#x} VER={:#x} SIVR={:#x} TPR={:#x} LVT={:#x} ESR={:#x}",
        id, ver, sivr, tpr, lvt, esr
    );
}

/// Start LAPIC timer in one-shot mode
///
/// Arms the timer to fire once after counting down from the initial count.
/// The timer will generate one interrupt and then stop.
///
/// # Arguments
/// * `initial_count` - Starting count value (decrements to 0)
///
/// # Timing
/// The actual time depends on:
/// - Bus frequency (CPU-specific)
/// - Divide configuration (currently /16)
/// - Initial count value
///
/// Approximate formula: `time ≈ initial_count * 16 / bus_frequency`
///
/// # Safety
/// Requires LAPIC to be configured via `lapic_timer_configure()` first.
///
/// # Examples
/// ```rust
/// unsafe {
///     lapic_timer_configure();
///     lapic_timer_start_oneshot(100_000);  // Fire once after ~100k cycles
/// }
/// ```
pub unsafe fn lapic_timer_start_oneshot(initial_count: u32) {
    let apic_base = get_apic_base();
    
    // Unmask timer (clear bit 16) for one-shot mode
    // Bit 17 is 0 for one-shot mode
    let mut lvt = read_apic_register(apic_base, 0x320);
    lvt &= !(1 << 16);  // Clear mask bit
    write_apic_register(apic_base, 0x320, lvt);
    
    // Write initial count to start countdown
    write_apic_register(apic_base, 0x380, initial_count);
    
    // Read current count for debugging
    let cur = read_apic_register(apic_base, 0x390);
    print_str_0xe9("  [lapic] current=");
    print_hex_u64_0xe9(cur as u64);
    out_char_0xe9(b'\n');
}

/// Start LAPIC timer in periodic mode
///
/// Arms the timer to fire repeatedly at fixed intervals. After each interrupt,
/// the counter automatically reloads from the initial count and continues.
///
/// # Arguments
/// * `initial_count` - Reload value (timer resets to this after each interrupt)
///
/// # Use Cases
/// - OS scheduler tick (e.g., 100Hz for preemption)
/// - Periodic animation updates
/// - Watchdog timers
///
/// # Safety
/// Requires LAPIC to be configured via `lapic_timer_configure()` first.
///
/// # Examples
/// ```rust
/// unsafe {
///     lapic_timer_configure();
///     lapic_timer_start_periodic(100_000);  // ~100Hz periodic interrupts
///     x86_64::instructions::interrupts::enable();  // Enable IF to receive interrupts
/// }
/// ```
pub unsafe fn lapic_timer_start_periodic(initial_count: u32) {
    let apic_base = get_apic_base();
    
    // Configure LVT Timer for periodic mode
    let mut lvt = read_apic_register(apic_base, 0x320);
    lvt |= 1 << 17;      // Set periodic mode (bit 17)
    lvt &= !(1 << 16);   // Clear mask bit (enable timer)
    write_apic_register(apic_base, 0x320, lvt);
    
    // Write initial count to start periodic countdown
    write_apic_register(apic_base, 0x380, initial_count);
}

/// Stop/mask LAPIC timer
///
/// Stops the timer by setting the mask bit in the LVT Timer register.
/// The timer will stop generating interrupts.
///
/// # Safety
/// Safe to call anytime after `lapic_timer_configure()`.
///
/// # Debug Output
/// Prints extensive debug information during masking to help diagnose
/// timer-related issues.
pub unsafe fn lapic_timer_mask() {
    let apic_base = get_apic_base();

    // TODO: When get_apic_base() is updated to return the actual base address, fix this.
    let vbase = crate::memory::PHYS_OFFSET + (apic_base & 0xFFFFF000);
    
    // Debug: print APIC addresses and CR3
    use x86_64::registers::control::Cr3;
    let (frame, _f) = Cr3::read();
    let cr3pa = frame.start_address().as_u64();
    
    print_str_0xe9("[lapic_mask] apic_base=");
    print_hex_u64_0xe9(apic_base);
    print_str_0xe9(" vbase=");
    print_hex_u64_0xe9(vbase);
    print_str_0xe9(" CR3=");
    print_hex_u64_0xe9(cr3pa);
    print_str_0xe9(" offset=0x320 addr=");
    print_hex_u64_0xe9(vbase + 0x320);
    out_char_0xe9(b'\n');
    
    // Read current LVT value
    let val = read_apic_register(apic_base, 0x320);
    print_str_0xe9("[lapic_mask] lvt before=");
    print_hex_u64_0xe9(val as u64);
    out_char_0xe9(b'\n');
    
    // Set mask bit to disable timer
    let new = val | (1 << 16);
    write_apic_register(apic_base, 0x320, new);
    
    // Verify the write succeeded
    let val2 = read_apic_register(apic_base, 0x320);
    print_str_0xe9("[lapic_mask] lvt after=");
    print_hex_u64_0xe9(val2 as u64);
    out_char_0xe9(b'\n');
    
    // Verify CR3 didn't change (helps debug page table issues)
    let (frame2, _f2) = Cr3::read();
    print_str_0xe9("[lapic_mask] CR3 after=");
    print_hex_u64_0xe9(frame2.start_address().as_u64());
    out_char_0xe9(b'\n');
}

/// Get current timer tick count
///
/// Returns the number of times the LAPIC timer interrupt has fired since boot.
///
/// # Returns
/// * Tick count (32-bit counter, wraps after ~4 billion ticks)
///
/// # Examples
/// ```rust
/// let before = timer_tick_count();
/// // ... do some work ...
/// let after = timer_tick_count();
/// let elapsed = after - before;
/// ```
pub fn timer_tick_count() -> u32 {
    TIMER_TICKS.load(Ordering::Relaxed)
}

/// Reinstall timer vector IDT entry at runtime
///
/// Patches the IDT entry for the timer vector (0x40) with the full 64-bit
/// handler address. This is called after switching to high-half virtual addresses
/// to ensure the handler pointer is correct.
///
/// # IDT Entry Structure (16 bytes)
/// - Bytes 0-1: offset_low (bits 0-15 of handler address)
/// - Bytes 2-3: selector (code segment)
/// - Byte 4: ist (Interrupt Stack Table index)
/// - Byte 5: type_attr (0x8E = present, DPL=0, 64-bit interrupt gate)
/// - Bytes 6-7: offset_mid (bits 16-31 of handler address)
/// - Bytes 8-11: offset_high (bits 32-63 of handler address)
/// - Bytes 12-15: reserved (must be 0)
///
/// # Safety
/// - Must be called after IDT is loaded
/// - Handler address must be valid
/// - Should only be called during initialization
pub unsafe fn install_timer_vector_runtime() {
    use x86_64::instructions::tables::sidt;
    
    // Get IDT base address
    let idtr = sidt();
    let base = idtr.base.as_u64();
    
    // Calculate address of timer vector entry (16 bytes per entry)
    let ent = base + ((APIC_TIMER_VECTOR as u64) * 16);
    
    // Get handler address and code segment selector
    let handler = handler_timer as usize as u64;
    let selector = crate::gdt::KERNEL_CS as u64;
    
    // Build IDT entry low 64 bits
    let mut low: u64 = 0;
    low |= (handler & 0xFFFF) as u64;          // Bits 0-15: offset_low
    low |= selector << 16;                      // Bits 16-31: selector
    low |= (0u64) << 32;                        // Bits 32-39: ist=0, reserved
    low |= (0x8Eu64) << 40;                     // Bits 40-47: type_attr (present, interrupt gate)
    low |= ((handler >> 16) & 0xFFFF) << 48;   // Bits 48-63: offset_mid
    
    // Build IDT entry high 64 bits
    let high: u64 = (handler >> 32) & 0xFFFF_FFFF;  // Bits 0-31: offset_high, 32-63: reserved
    
    // Write the 16-byte IDT entry
    core::ptr::write_unaligned(ent as *mut u64, low);
    core::ptr::write_unaligned((ent + 8) as *mut u64, high);
}

