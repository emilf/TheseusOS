//! HID boot keyboard helpers for the xHCI driver.
//!
//! This module keeps the "boot protocol keyboard" support (8-byte interrupt IN
//! reports) isolated from the rest of the controller bring-up so the xHCI core
//! can remain focused on rings, events, and enumeration.

use crate::input::keyboard::{KeyEvent, KeyTransition};
use crate::input::keyboard::{self};
use crate::log_info;
use alloc::vec::Vec;

/// Textual names for the modifier bits in the first byte of a boot keyboard report.
pub(crate) const HID_MODIFIER_NAMES: [&str; 8] = [
    "LeftCtrl",
    "LeftShift",
    "LeftAlt",
    "LeftMeta",
    "RightCtrl",
    "RightShift",
    "RightAlt",
    "RightMeta",
];

pub(crate) const HID_MODIFIER_USAGE_BASE: u8 = 0xE0;

/// Usage-code to string mapping for alphabetic keys in the boot protocol.
pub(crate) const HID_LETTER_NAMES: [&str; 26] = [
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S",
    "T", "U", "V", "W", "X", "Y", "Z",
];

/// Usage-code to string mapping for the number row keys.
pub(crate) const HID_NUMBER_NAMES: [&str; 10] = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"];

/// Lookup table for the F1..F12 usages.
pub(crate) const HID_FUNCTION_NAMES: [&str; 12] = [
    "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
];

/// Convert a boot keyboard usage code into a `KeyEvent` understood by the input hub.
///
/// This intentionally stays minimal (letters, digits, function keys, a few controls)
/// and is good enough for early bring-up via the boot protocol.
#[allow(dead_code)]
pub(crate) fn usage_to_key_event(usage: u8, transition: KeyTransition) -> Option<KeyEvent> {
    let name: &str = match usage {
        0x04..=0x1d => HID_LETTER_NAMES[(usage - 0x04) as usize],
        0x1e..=0x27 => HID_NUMBER_NAMES[(usage - 0x1e) as usize],
        0x28 => "Enter",
        0x29 => "Escape",
        0x2a => "Backspace",
        0x2b => "Tab",
        0x2c => "Space",
        0x39 => "CapsLock",
        0x3a..=0x45 => HID_FUNCTION_NAMES[(usage - 0x3a) as usize],
        _ => return None,
    };

    Some(KeyEvent::new(transition, usage, name))
}

/// Translate a HID usage code into a human-readable label for debugging/teaching.
pub(crate) fn usage_to_name(usage: u8) -> &'static str {
    match usage {
        0x04..=0x1d => HID_LETTER_NAMES[(usage - 0x04) as usize],
        0x1e..=0x27 => HID_NUMBER_NAMES[(usage - 0x1e) as usize],
        0x28 => "Enter",
        0x29 => "Escape",
        0x2a => "Backspace",
        0x2b => "Tab",
        0x2c => "Space",
        0x2d => "Minus",
        0x2e => "Equals",
        0x2f => "LeftBracket",
        0x30 => "RightBracket",
        0x31 => "Backslash",
        0x33 => "Semicolon",
        0x34 => "Apostrophe",
        0x35 => "Grave",
        0x36 => "Comma",
        0x37 => "Period",
        0x38 => "Slash",
        0x39 => "CapsLock",
        0x3a..=0x45 => HID_FUNCTION_NAMES[(usage - 0x3a) as usize],
        0x49 => "Insert",
        0x4a => "Home",
        0x4b => "PageUp",
        0x4c => "Delete",
        0x4d => "End",
        0x4e => "PageDown",
        0x4f => "Right",
        0x50 => "Left",
        0x51 => "Down",
        0x52 => "Up",
        _ => "Unknown",
    }
}

/// Collect unique, non-zero usage codes from the boot-report payload, ignoring modifiers.
pub(crate) fn collect_usage_codes(report: &[u8], len: usize) -> Vec<u8> {
    let mut keys = Vec::new();
    let usable_len = core::cmp::min(len, report.len());
    for &usage in report.iter().take(usable_len).skip(2) {
        if usage != 0 && !keys.contains(&usage) {
            keys.push(usage);
        }
    }
    keys
}

/// Decode the delta between two HID boot protocol reports and publish key transitions.
pub(crate) fn emit_keyboard_events(
    ident: &str,
    previous: &[u8; 8],
    previous_len: usize,
    current: &[u8],
) {
    let prev_mod = if previous_len > 0 { previous[0] } else { 0 };
    let curr_mod = current.get(0).copied().unwrap_or(0);
    let changed_mod = prev_mod ^ curr_mod;

    for bit in 0..8 {
        if (changed_mod & (1 << bit)) == 0 {
            continue;
        }

        let name = HID_MODIFIER_NAMES[bit as usize];
        let usage = HID_MODIFIER_USAGE_BASE + bit as u8;
        if (curr_mod & (1 << bit)) != 0 {
            log_info!("xHCI {} key pressed: {}", ident, name);
            keyboard::publish_event(KeyEvent::new(KeyTransition::Pressed, usage, name));
        } else {
            log_info!("xHCI {} key released: {}", ident, name);
            keyboard::publish_event(KeyEvent::new(KeyTransition::Released, usage, name));
        }
    }

    let prev_keys = collect_usage_codes(previous, previous_len);
    let curr_keys = collect_usage_codes(current, current.len());

    for usage in curr_keys.iter() {
        if prev_keys.contains(usage) {
            continue;
        }
        let name = usage_to_name(*usage);
        log_info!("xHCI {} key pressed: {}", ident, name);
        keyboard::publish_event(KeyEvent::new(KeyTransition::Pressed, *usage, name));
    }

    for usage in prev_keys.iter() {
        if curr_keys.contains(usage) {
            continue;
        }
        let name = usage_to_name(*usage);
        log_info!("xHCI {} key released: {}", ident, name);
        keyboard::publish_event(KeyEvent::new(KeyTransition::Released, *usage, name));
    }
}


