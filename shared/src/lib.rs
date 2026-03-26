#![no_std]

//! Module: shared
//!
//! SOURCE OF TRUTH:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/boot.md#A1:-The-kernel-boots-as-a-single-UEFI-executable
//! - docs/axioms/memory.md#A3:-The-boot-path-keeps-a-temporary-heap-before-switching-to-a-permanent-kernel-heap
//! - docs/axioms/debug.md#A2:-Panic-handling-reports-failure-through-kernel-logging-and-exits-QEMU-with-error-status
//!
//! INVARIANTS:
//! - This crate holds the boot-path ABI and low-level helpers intentionally shared between bootloader and kernel.
//! - Shared code here exists to keep both halves of the current single-binary boot flow aligned on allocator, handoff, constant, and QEMU-helper behavior.
//! - This crate is `no_std` and must remain suitable for the earliest boot/runtime contexts that use it.
//!
//! SAFETY:
//! - Shared ABI types and low-level helpers are consumed on both sides of the boot boundary, so accidental layout or semantic drift is especially dangerous.
//! - Any helper here that touches raw pointers, port I/O, or global state must stay compatible with the constrained boot contexts that call it.
//!
//! PROGRESS:
//! - docs/plans/boot-flow.md
//! - docs/plans/memory.md
//! - docs/plans/observability.md
//!
//! Shared crate for boot ABI, allocator glue, constants, and low-level helper macros.

pub mod allocator;
pub mod constants;
pub mod handoff;
pub mod macros;
