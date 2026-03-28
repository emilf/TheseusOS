# CPU & Platform Hardening Plan

**Phase:** 1  
**Status:** Implementation complete, validation added  
**Last Updated:** 2026-03-28

## Overview

This document summarizes the CPU and platform hardening work completed in Phase 1 of the TheseusOS roadmap. The goal was to solidify CPU foundations before moving to multi-process work.

## Completed Tasks

### 1.6 CPUID Feature Abstraction ✅
**Implementation:** `kernel/src/cpu_features.rs`  
**Purpose:** Centralized CPU feature detection for SSE, AVX, TSC-Deadline, x2APIC, etc.  
**Usage:** Used by APIC, timer calibration, and future SMP code.  
**Monitor Command:** `cpu features`

### 1.5.2 LAPIC Abstraction ✅  
**Implementation:** `kernel/src/interrupts/apic.rs`  
**Purpose:** Abstract Local APIC operations for timer, IPIs, and interrupt delivery.  
**Status:** Fully abstracted, ready for x2APIC extension.

### 1.4.1 HPET/PIT Calibration + 1.4.2 Periodic Tick ✅
**Implementation:** `kernel/src/interrupts/calibration.rs`  
**Purpose:** Measure ticks/ms against HPET or PIT for real preemption timing.  
**Monitor Command:** `cpu timer`  
**Status:** Calibration working, periodic tick firing.

### 1.2.1 Fix Driver Data Unsoundness ✅
**Implementation:** Various driver files (`serial.rs`, `usb/xhci/mod.rs`, etc.)  
**Purpose:** Fix unsafe driver data handling with proper lifetime/ownership tracking.  
**Status:** Serial and USB drivers now safe.

### 1.1.2 VA Allocator ✅
**Implementation:** `kernel/src/memory/va_alloc.rs`  
**Documentation:** `docs/axioms/memory.md` (A3.5)  
**Purpose:** Manage dedicated dynamic VA region `0xFFFF_9000_0000_0000 .. 0xFFFF_B000_0000_0000`.  
**Status:** Working, ready for runtime mappings and kernel stacks.

### 1.1.3 Runtime Page Table Manager ✅
**Implementation:** `kernel/src/memory/runtime_mapper.rs`  
**Purpose:** Dynamic kernel mappings for devices, stacks, etc.  
**Status:** Working.

### 1.1.4 Kernel Stack Allocator ✅ (with TODOs)
**Implementation:** `kernel/src/memory/stack_alloc.rs`  
**Purpose:** Allocate kernel stacks for threads/exceptions.  
**Status:** Basic allocator exists; needs per-CPU stack support for SMP.

### 1.2.2 PCI → DriverManager Integration ✅
**Implementation:** DriverManager integration across PCI/USB code  
**Purpose:** Proper device registration and ownership.  
**Status:** PCI devices register with DriverManager.

### 1.2.3 IRQ Ownership Model ✅
**Implementation:** `kernel/src/interrupts/irq_registry.rs`  
**Purpose:** Track which driver owns which IRQ line.  
**Status:** IRQ registry implemented.

### 1.1.5 Boot Handoff Documentation + Validation ✅
**Implementation:** `kernel/src/physical_memory.rs`  
**Purpose:** Debug assertions for boot handoff correctness.  
**Features:**
- `validate_no_overlap_with_free_frames()` - catches handoff bugs
- Enhanced logging for boot-consumed regions
- Only active in debug builds (`#[cfg(debug_assertions)]`)

### 1.2.4 Framebuffer as Proper Driver ⏸️
**Status:** Skipped (lowest priority per roadmap)  
**Reason:** Framebuffer works via UEFI GOP; proper driver can wait.

## Validation Added in This Phase

### Physical Memory Validation
```rust
#[cfg(debug_assertions)]
fn validate_no_overlap_with_free_frames(
    manager: &PhysicalMemoryManager,
    consumed: &[ConsumedRegion],
) {
    // Panics if any consumed region overlaps with free frames
}
```
- **Purpose:** Catch missed `record_boot_consumed_region()` calls
- **Impact:** Only in debug builds, negligible performance cost
- **Benefit:** Early detection of handoff bugs

### Enhanced Logging
- Boot-consumed region count and merging
- Physical memory initialization steps
- Debug visibility without release build overhead

### Monitor Commands
- `cpu features` - Display CPU feature flags
- `cpu timer` - Show timer calibration results

## Testing

### Manual Testing
- [x] QEMU boots with debug assertions enabled
- [x] Real hardware (Lenovo Legion) boots
- [x] Monitor commands work (`cpu features`, `cpu timer`)

### Automated Testing
- [x] Existing test suite passes
- [x] No regressions in kernel behavior

## Next Steps

### Immediate (Phase 1 remaining)
1. **x2APIC support** - Plan exists (`x2apic-prep.md`), needs implementation
2. **TSS + IST stacks** - Separate stacks for NMI/DF/MCE exceptions
3. **SMP bring-up** - Wake APs, per-CPU state, IPI infrastructure

### Future (Phase 2+)
- Kernel threads and context switching
- Preemptive scheduler
- User-mode and address spaces

## Design Decisions

### Debug-Only Validation
Validation code is gated with `#[cfg(debug_assertions)]` to:
1. Catch bugs during development
2. Avoid performance cost in release builds
3. Maintain clean separation between debug and production

### VA Allocator Region
Chose `0xFFFF_9000_0000_0000 .. 0xFFFF_B000_0000_0000` because:
1. Well clear of PHYS_OFFSET (`0xFFFF_8000_0000_0000`)
2. Ample space (2 TiB) for runtime allocations
3. Doesn't conflict with static VA windows

### IRQ Registry
Centralized IRQ ownership tracking to:
1. Prevent multiple drivers claiming same IRQ
2. Provide clear ownership model
3. Support future IRQ balancing/sharing

## Files Modified

See PR #27 for complete diff. Key files:
- `docs/axioms/memory.md` - Added A3.5 (VA allocator)
- `kernel/src/physical_memory.rs` - Boot handoff validation
- `kernel/src/monitor/commands/mod.rs` - CPU monitor commands
- Various driver/interrupt files - Minor fixes and documentation