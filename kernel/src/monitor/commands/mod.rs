//! Module: monitor::commands
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - This module tree contains the interactive commands exposed by the kernel monitor.
//! - Command groupings are organizational; they do not create extra safety boundaries around privileged operations.
//!
//! SAFETY:
//! - Monitor commands operate on privileged kernel state and must remain explicit about the danger of touching memory, MMIO, ports, or control paths.
//! - Convenience in command structure must not blur the fact that these are debugging/admin interfaces, not user-safe APIs.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/drivers-and-io.md
//!
//! Monitor command module tree.
//!
//! This module groups the privileged interactive commands exposed by the kernel monitor.

pub mod control;
pub mod cpu;
pub mod devices;
pub mod io;
pub mod memory;
pub mod pci;
pub mod system;
pub mod tables;
pub mod usb;
