//! Module: interrupts::irq_registry
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//!
//! INVARIANTS:
//! - IRQ vector registry tracks ownership of vectors 0x20‑0xFF (excluding exceptions 0x00‑0x1F).
//! - Timer (0x40) and APIC error (0xFE) keep dedicated handlers.
//! - Drivers register during init() via `register_irq_handler`.
//!
//! SAFETY:
//! - Registration must happen before interrupts are enabled for that vector.
//! - Handlers must be `fn()` with no arguments and must send EOI if needed.
//! - The registry is static and lives for the kernel lifetime.

use alloc::vec::Vec;
use spin::Mutex;

/// Maximum number of interrupt vectors (256).
const MAX_VECTORS: usize = 256;

/// Entry in the IRQ handler registry.
struct IrqEntry {
    /// Driver name for debugging.
    name: &'static str,
    /// Handler function (no arguments, returns ()).
    handler: fn(),
}

/// Global IRQ handler registry.
static IRQ_HANDLERS: Mutex<[Option<IrqEntry>; MAX_VECTORS]> =
    Mutex::new([const { None }; MAX_VECTORS]);

/// Register an IRQ handler for a specific vector.
///
/// # Arguments
/// - `vector`: Interrupt vector (0x20‑0xFF).
/// - `name`: Driver name for debugging.
/// - `handler`: Function to call when interrupt fires.
///
/// # Returns
/// - `Ok(())` on success.
/// - `Err("vector out of range")` if vector < 0x20 or > 0xFF.
/// - `Err("already registered")` if vector already has a handler.
pub fn register_irq_handler(
    vector: u8,
    name: &'static str,
    handler: fn(),
) -> Result<(), &'static str> {
    if vector < 0x20 {
        return Err("vector out of range (must be 0x20‑0xFF)");
    }

    let mut registry = IRQ_HANDLERS.lock();
    let idx = vector as usize;

    if registry[idx].is_some() {
        return Err("already registered");
    }

    registry[idx] = Some(IrqEntry { name, handler });
    Ok(())
}

/// Unregister an IRQ handler.
///
/// # Safety
/// Interrupts for this vector must be disabled before calling.
pub unsafe fn unregister_irq_handler(vector: u8) -> Result<(), &'static str> {
    if vector < 0x20 {
        return Err("vector out of range");
    }

    let mut registry = IRQ_HANDLERS.lock();
    let idx = vector as usize;

    if registry[idx].is_none() {
        return Err("not registered");
    }

    registry[idx] = None;
    Ok(())
}

/// Dispatch an IRQ to the registered handler.
///
/// Called from the general interrupt handler. Looks up the registry,
/// calls the handler if present, logs + EOI if not.
///
/// # Safety
/// Must be called from an interrupt context with interrupts disabled.
pub unsafe fn dispatch_irq(vector: u8) {
    let registry = IRQ_HANDLERS.lock();
    let idx = vector as usize;

    match &registry[idx] {
        Some(entry) => {
            // Call the registered handler
            (entry.handler)();
        }
        None => {
            // No handler registered — log and send EOI
            crate::log_warn!("Unhandled IRQ vector 0x{:02X}", vector);
            super::local_apic_eoi();
        }
    }
}

/// List registered IRQ handlers (for monitor command).
pub fn list_irq_handlers() -> Vec<(u8, &'static str)> {
    let registry = IRQ_HANDLERS.lock();
    let mut result = Vec::new();

    for (vector, entry) in registry.iter().enumerate() {
        if let Some(entry) = entry {
            result.push((vector as u8, entry.name));
        }
    }

    result
}
