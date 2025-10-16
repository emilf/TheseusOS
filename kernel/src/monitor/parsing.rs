//! Parsing utilities for the monitor
//!
//! This module provides helper functions for parsing user input, including
//! hex/decimal number parsing.

/// Parse a number from string (supports hex with 0x prefix and decimal)
///
/// # Arguments
/// * `s` - String to parse (can be hex with "0x" prefix or decimal)
///
/// # Returns
/// * `Some(u64)` - Parsed number
/// * `None` - Invalid format
///
/// # Examples
/// ```
/// assert_eq!(parse_number("42"), Some(42));
/// assert_eq!(parse_number("0x2A"), Some(42));
/// assert_eq!(parse_number("0xFF"), Some(255));
/// ```
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

