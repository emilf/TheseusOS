//! Module: monitor::parsing
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//!
//! INVARIANTS:
//! - Monitor parsing helpers convert user-facing textual input into the small set of primitive values needed by monitor commands.
//! - Parsing logic is intentionally narrow and predictable because it feeds privileged debugging operations.
//!
//! SAFETY:
//! - Parsing success only means the syntax was understood; command handlers still own semantic validation before touching memory, MMIO, ports, or control paths.
//! - Small helpers here should stay boring; clever parsing increases ambiguity in a privileged monitor surface.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//!
//! Small parsing helpers for the kernel monitor.
//!
//! This module keeps monitor input parsing intentionally narrow and boring.

/// Parse a monitor integer argument in decimal or `0x`-prefixed hex.
pub fn parse_number(s: &str) -> Option<u64> {
    let s = s.trim();

    if s.starts_with("0x") || s.starts_with("0X") {
        // Hex number
        let hex = s.trim_start_matches("0x").trim_start_matches("0X");
        u64::from_str_radix(hex, 16).ok()
    } else {
        // Decimal number
        s.parse::<u64>().ok()
    }
}
