# Memory Inspection Plan

**Date**: 2026-04-06  
**Status**: Planning Phase  
**Goal**: Create unified memory inspection API for TheseusOS

---

## 1. Problem Statement

### Current State
- Monitor has `mem`, `dump`, `write`, `fill` commands
- Each implements its own memory access with `read_volatile`/`write_volatile`
- No programmatic API for kernel code
- No physical memory support
- Safety issues: volatile access without validation

### Gaps
1. **Programmatic access** - Kernel code can't inspect memory
2. **Safety** - No validation before memory access
3. **Physical memory** - Can't inspect hardware directly
4. **Consistency** - Duplicated logic, different patterns

---

## 2. Solution: Unified Architecture

### New Module Structure
```
kernel/src/memory_inspect/          # NEW: Unified memory inspection
├── mod.rs                          # Public API, safety documentation
├── access.rs                       # Core memory access functions
├── hexdump.rs                      # Hexdump utility
├── physical.rs                     # Physical memory access
└── safety.rs                       # Validation, bounds checking
```

### Key Components
1. **Safe memory access API** - Validated reads/writes
2. **Hexdump utility** - Programmatic memory inspection
3. **Physical memory support** - Via `PHYS_OFFSET` mapping
4. **Monitor integration** - Update existing commands

---

## 3. Implementation Phases

### Phase 1: Foundation (Week 1)
- [ ] Create `memory_inspect` module structure
- [ ] Implement `safety.rs` validation functions
- [ ] Implement `access.rs` basic read/write
- [ ] Add `serial_println!` macro to `serial_debug`
- [ ] Write unit tests for validation

**Deliverables**:
- `memory_inspect` module compiles
- Basic read/write with validation
- `serial_println!` macro available

### Phase 2: Core Features (Week 2)
- [ ] Implement `hexdump.rs` with proper formatting
- [ ] Add physical memory support in `physical.rs`
- [ ] Create comprehensive error types (`MemoryAccessError`)
- [ ] Add more unit tests
- [ ] Document public API

**Deliverables**:
- Hexdump produces correct output
- Physical memory access works
- Error handling complete

### Phase 3: Monitor Integration (Week 3)
- [ ] Update monitor `memory.rs` to use new API
- [ ] Add new monitor commands: `physdump`, `validate`
- [ ] Ensure backward compatibility
- [ ] Update monitor documentation
- [ ] Test all monitor commands work

**Deliverables**:
- Monitor uses new memory_inspect API
- New `physdump` command works
- All existing commands unchanged (behaviorally)

### Phase 4: Enhanced Features (Future)
- [ ] Pattern search (`search 0x1000 0xDEADBEEF`)
- [ ] Memory comparison (`compare 0x1000 0x2000 256`)
- [ ] Checksum calculation (`checksum 0x1000 1024`)
- [ ] Memory testing (`test 0x1000 4096`)

---

## 4. API Design

### Core Access API (`memory_inspect::access`)
```rust
// Safe memory reads
pub fn read_byte(addr: u64) -> Result<u8, MemoryAccessError>;
pub fn read_bytes(addr: u64, buf: &mut [u8]) -> Result<(), MemoryAccessError>;
pub fn read_struct<T>(addr: u64) -> Result<T, MemoryAccessError>;

// Safe memory writes  
pub fn write_byte(addr: u64, value: u8) -> Result<(), MemoryAccessError>;
pub fn write_bytes(addr: u64, data: &[u8]) -> Result<(), MemoryAccessError>;
pub fn fill_bytes(addr: u64, len: usize, value: u8) -> Result<(), MemoryAccessError>;

// Physical memory (via PHYS_OFFSET)
pub fn read_physical(phys_addr: u64, buf: &mut [u8]) -> Result<(), MemoryAccessError>;
pub fn write_physical(phys_addr: u64, data: &[u8]) -> Result<(), MemoryAccessError>;
```

### Hexdump API (`memory_inspect::hexdump`)
```rust
pub fn hexdump_virtual(addr: u64, len: usize, base_addr: Option<u64>) 
    -> Result<(), MemoryAccessError>;
    
pub fn hexdump_physical(phys_addr: u64, len: usize) 
    -> Result<(), MemoryAccessError>;
    
pub fn hexdump_to_writer<W: Write>(
    writer: &mut W, 
    addr: u64, 
    len: usize, 
    base_addr: Option<u64>
) -> Result<(), MemoryAccessError>;
```

### Safety API (`memory_inspect::safety`)
```rust
pub fn validate_virtual_range(addr: u64, len: usize) -> ValidationResult;
pub fn validate_physical_range(phys_addr: u64, len: usize) -> ValidationResult;
pub fn is_kernel_address(addr: u64) -> bool;
pub fn is_user_accessible(addr: u64) -> bool;
```

### Error Types
```rust
#[derive(Debug, Clone, Copy)]
pub enum MemoryAccessError {
    NullPointer,
    InvalidAlignment,
    OutOfBounds,
    NotMapped,
    PageFault(u64),           // Address that caused fault
    ProtectionViolation(u64),  // Address with protection issue
    PhysicalNotMapped,        // PHYS_OFFSET not active
    InterruptContext,         // Cannot validate in interrupt
}
```

---

## 5. Safety Implementation

### Key Safety Features
1. **Validation before access** - Check alignment, bounds, mapping
2. **Use `read()` not `read_volatile()`** - Let page faults happen safely
3. **Interrupt context detection** - Skip validation if in interrupt
4. **Kernel/user separation** - Different validation rules
5. **Physical memory checks** - Verify `PHYS_OFFSET` is active

### Validation Logic
```rust
fn validate_virtual_range(addr: u64, len: usize) -> ValidationResult {
    // 1. Check null pointer
    // 2. Check alignment (if required)
    // 3. Check for overflow
    // 4. Check if in kernel space (always allowed)
    // 5. Check if in user space (needs page table walk)
    // 6. Return specific error for debugging
}
```

---

## 6. Monitor Integration Plan

### Updated Commands
- `mem` → Uses `memory_inspect::read_byte()` with continuation
- `dump` → Uses `memory_inspect::hexdump_virtual()`
- `write` → Uses `memory_inspect::write_byte()`
- `fill` → Uses `memory_inspect::fill_bytes()`

### New Commands
- `physdump ADDR LEN` → Dump physical memory
- `validate ADDR LEN` → Validate memory range
- `search ADDR LEN VALUE` → Search for pattern

### Backward Compatibility
- Keep same command names and basic behavior
- Improve error messages
- Add optional flags for new features

---

## 7. Success Criteria

### Phase 1 Success
- [ ] `memory_inspect` module compiles
- [ ] Basic read/write functions work
- [ ] Validation catches obvious errors
- [ ] Unit tests pass

### Phase 2 Success
- [ ] Hexdump produces correct output
- [ ] Physical memory access works
- [ ] Monitor commands unchanged (behaviorally)
- [ ] No performance regression

### Phase 3 Success
- [ ] Monitor uses new API
- [ ] New `physdump` command works
- [ ] Error messages improved
- [ ] Documentation updated

---

## 8. Risk Mitigation

### Technical Risks
1. **Performance overhead** - Validation adds cost
   - *Mitigation*: Skip validation in interrupt context, cache results
   
2. **Page fault handling** - `read()` can fault
   - *Mitigation*: Catch faults, return `PageFault` error
   
3. **Physical memory mapping** - `PHYS_OFFSET` might not be active
   - *Mitigation*: Check at runtime, return error

### Migration Risks
1. **Breaking monitor commands** - Current `mem`/`dump` work
   - *Mitigation*: Keep old behavior, add deprecation warnings
   
2. **Complexity increase** - New module adds code
   - *Mitigation*: Start minimal, add features gradually

---

## 9. Why This Matters for TheseusOS

1. **Safety** - Current `read_volatile` usage is dangerous
2. **Consistency** - One way to inspect memory, not four
3. **Capability** - Physical memory access enables hardware debugging
4. **Architecture** - Clean separation of concerns
5. **Foundation** - Enables future testing/validation tools

**Alignment with TheseusOS philosophy**:
- Debugging as first-class citizen (axiom A3)
- Safety through validation
- Clean architecture with clear boundaries
- Educational value in implementation

---

## 10. Next Steps

### Immediate (Phase 1)
1. Create `memory_inspect` module structure
2. Implement basic validation in `safety.rs`
3. Add `serial_println!` macro to `serial_debug`
4. Write initial unit tests

### Documentation Updates Needed
1. Update `docs/plans/observability.md`
2. Add `memory_inspect` module documentation
3. Update monitor command documentation
4. Create usage examples

### Testing Strategy
1. Unit tests for validation logic
2. Integration tests for hexdump
3. Monitor command regression tests
4. Performance benchmarks (optional)

---

## 11. References

### Existing Code
- `kernel/src/monitor/commands/memory.rs` - Current implementation
- `kernel/src/serial_debug.rs` - Serial debug utilities
- `kernel/src/memory/mod.rs` - Memory management

### TheseusOS Documentation
- `docs/axioms/debug.md` - Debugging axioms
- `docs/plans/observability.md` - Observability plan
- `docs/plans/memory.md` - Memory management plan

### External References
- x86_64 page table structure
- PHYS_OFFSET mapping pattern
- Safe memory access patterns in OS kernels

---

**Plan approved by**: Emil  
**Plan created by**: Rowan Redwing  
**Last updated**: 2026-04-06