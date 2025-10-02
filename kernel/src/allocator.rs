//! Kernel allocator module (legacy internal allocator no longer used).
//!
//! The project now uses a global allocator shim in `theseus_shared::allocator` that
//! forwards to UEFI before ExitBootServices and switches to a kernel heap after the
//! higher-half mappings are established. This module remains only as a placeholder
//! for potential kernel-local allocators.

// Intentionally empty for now.
