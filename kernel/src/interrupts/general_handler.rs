//! Module: interrupts::general_handler
//!
//! General interrupt dispatcher for dynamically registered IRQ handlers.
//!
//! This module provides a general interrupt handler that looks up registered
//! handlers in the IRQ registry and calls them. Handlers for specific vectors
//! (like timer, serial, xHCI) can be registered by drivers during initialization.

use crate::log_debug;
use x86_64::structures::idt::InterruptStackFrame;

use super::irq_registry;

/// General interrupt handler for dynamically registered IRQ vectors.
///
/// This handler looks up the registered handler for the interrupt vector
/// in the IRQ registry and calls it. If no handler is registered, it logs
/// a warning and sends EOI.
pub extern "x86-interrupt" fn general_interrupt_handler(stack: InterruptStackFrame) {
    // Get the interrupt vector from the stack frame
    // Note: In x86_64, the interrupt vector is pushed on the stack
    // We need to extract it from the error code or use a different approach
    // For now, we'll use a placeholder - this needs to be fixed
    
    // TODO: Extract actual vector from interrupt stack frame
    // For now, we'll use a dummy value
    let _vector: u8 = 0;
    
    // This is a placeholder implementation
    // In reality, we need to:
    // 1. Extract the interrupt vector from the stack frame or CS:IP
    // 2. Look it up in the IRQ registry
    // 3. Call the registered handler if found
    // 4. Send EOI
    
    unsafe {
        super::apic::local_apic_eoi();
    }
    
    // Log for debugging
    log_debug!("General interrupt handler called (placeholder)");
}