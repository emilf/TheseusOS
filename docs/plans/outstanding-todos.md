# Outstanding TODOs & Incomplete Features

**Last Updated:** 2026-03-28  
**Status:** After Phase 1 Hardening PR #27 merge

## Summary
This document tracks TODOs, FIXMEs, and incomplete features found in the TheseusOS codebase after the Phase 1 Hardening work. Items are categorized by priority and status.

---

## 🔴 High Priority (Blocking Core Functionality)

### 1. Interrupt Vector Extraction
**File:** `kernel/src/interrupts/general_handler.rs:25`  
**Issue:** `general_interrupt_handler` cannot determine which interrupt vector triggered it.  
**Impact:** IRQ registry cannot be used effectively; all interrupts (except hardcoded ones) go to same handler.  
**Solution:** Implement macro-generated handlers or modify x86_64 crate usage.  
**Status:** ❌ Not implemented  
**See:** `/tmp/interrupt_todo.md` for detailed analysis.

### 2. x2APIC Implementation
**Plan:** `docs/plans/x2apic-prep.md`  
**Issue:** x2APIC detection exists but runtime path is still xAPIC/MMIO-only.  
**Impact:** Kernel will GPF if firmware boots into x2APIC mode.  
**Solution:** Implement mode-aware LAPIC access helpers.  
**Status:** ❌ Not implemented (9 TODOs in plan)  
**Priority:** High for SMP readiness.

---

## 🟡 Medium Priority (Feature Enhancements)

### 3. USB HID Boot Protocol Negotiation
**File:** `kernel/src/drivers/usb/xhci/mod.rs:4225`  
**Issue:** Missing `SET_PROTOCOL` request to switch HID devices to Boot Protocol.  
**Impact:** Works with most keyboards but may fail with legacy devices.  
**Solution:** Implement `SET_PROTOCOL` control transfer.  
**Status:** ❌ Not implemented  
**See:** `/tmp/hid_todo.md` for details.

### 4. Framebuffer Handoff Dependency
**File:** `kernel/src/environment.rs:362`  
**Issue:** Framebuffer drawing accesses handoff through global in interrupts module.  
**Impact:** Code coupling, not a functional issue.  
**Solution:** Clean up dependency injection.  
**Status:** ❌ Not implemented  
**Priority:** Low (code cleanup).

### 5. TSS + IST Stacks (Per-CPU)
**Roadmap:** Phase 1, marked as done for BSP only.  
**Issue:** Current implementation is BSP-only; needs per-CPU support for SMP.  
**Impact:** SMP bring-up blocked.  
**Solution:** Extend to per-CPU TSS/IST.  
**Status:** ⏸️ Partially implemented  
**Priority:** Medium (needed for SMP).

### 6. SMP Bring-up
**Roadmap:** Phase 1.7, deferred to after scheduler.  
**Issue:** Not yet started.  
**Impact:** Single-core only.  
**Solution:** Parse AP LAPIC IDs, per-CPU state, INIT-SIPI sequence.  
**Status:** ❌ Not started  
**Priority:** Medium (post-scheduler).

---

## 🟢 Low Priority (Code Quality)

### 7. VA Allocator Free-List
**File:** `kernel/src/memory/va_alloc.rs`  
**Issue:** `free_va()` was a no-op (bump allocator).  
**Status:** ✅ **IMPLEMENTED** in Phase 1 Hardening  
**Changes:** Added free-list with coalescing, statistics.

### 8. Stack Allocator Physical Mapping
**File:** `kernel/src/memory/stack_alloc.rs:79,91`  
**Issue:** Couldn't allocate/free physical frames for stacks.  
**Status:** ✅ **IMPLEMENTED** in Phase 1 Hardening  
**Changes:** Integrated with physical frame allocator and runtime mapper.

### 9. Physical Memory Validation
**File:** `kernel/src/physical_memory.rs`  
**Issue:** Missing overlap validation for boot-consumed regions.  
**Status:** ✅ **IMPLEMENTED** in Phase 1 Hardening  
**Changes:** Added `validate_no_overlap_with_free_frames()` debug assertion.

---

## 📋 Phase 1 Roadmap Completion Status

From `docs/roadmap.md` Phase 1 tasks:

- [x] **APIC timer calibration** — ✅ Implemented (`interrupts/calibration.rs`)
- [x] **CPUID feature abstraction** — ✅ Implemented (`cpu_features.rs`)
- [ ] **x2APIC support** — ❌ Plan exists, needs implementation
- [ ] **TSS + IST stacks** — ⏸️ BSP-only done, needs per-CPU
- [ ] **SMP bring-up** — ❌ Deferred to after scheduler

**Overall Phase 1 Completion:** ~40% (2/5 core tasks)

---

## 🚀 Recommended Implementation Order

1. **Interrupt Vector Extraction** (High priority, blocks IRQ system)
2. **x2APIC Implementation** (High priority, needed for modern hardware)
3. **USB HID Boot Protocol** (Medium, improves keyboard compatibility)
4. **Per-CPU TSS/IST** (Medium, needed for SMP)
5. **Framebuffer Cleanup** (Low, code quality)
6. **SMP Bring-up** (Post-scheduler)

---

## 📚 Documentation Updates Needed

1. **`docs/plans/cpu-platform-hardening.md`** — Already created, covers Phase 1 status.
2. **Interrupt architecture document** — Needed to explain vector handling issues.
3. **x2APIC implementation guide** — Based on `x2apic-prep.md` plan.
4. **SMP bring-up plan** — New document needed when starting SMP work.

---

## 🧪 Testing Considerations

For each implementation:
- **Interrupt vectors**: Test with spurious interrupts, verify vector extraction.
- **x2APIC**: Test in QEMU with `-cpu host,+x2apic`, verify no GPF.
- **HID Boot Protocol**: Test with various USB keyboards.
- **Per-CPU TSS**: Test with dummy AP bring-up in QEMU.

---

## 🔗 Related Files

- `docs/roadmap.md` — Overall project roadmap
- `docs/plans/x2apic-prep.md` — x2APIC implementation plan
- `docs/plans/cpu-platform-hardening.md` — Phase 1 status (created)
- `kernel/src/interrupts/general_handler.rs` — Interrupt vector issue
- `kernel/src/drivers/usb/xhci/mod.rs` — HID protocol TODO
- `kernel/src/environment.rs` — Framebuffer handoff TODO