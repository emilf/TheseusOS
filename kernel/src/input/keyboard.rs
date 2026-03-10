//! Shared keyboard event distribution.
//!
//! The USB keyboard driver originally buffered key transition events in a
//! driver-local queue that the kernel monitor could poll.  That was useful for
//! early bring-up, but it forced consumers to depend on `drivers::usb`.  This
//! module lifts the queue into a tiny input hub so future subsystems (shells,
//! GUIs, scripting environments) can subscribe without caring about the
//! underlying transport.

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use spin::{Mutex as SpinMutex, Once};

/// Maximum number of keyboard events retained before old entries are evicted.
const DEFAULT_QUEUE_CAPACITY: usize = 64;
/// Hard cap on simultaneous listeners so misbehaving code cannot exhaust heap.
const MAX_LISTENERS: usize = 8;

/// Lookup table for unshifted digits in the boot protocol usage range.
const NUMBER_BASE: [char; 10] = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'];
/// Lookup table for shifted digits in the boot protocol usage range.
const NUMBER_SHIFT: [char; 10] = ['!', '@', '#', '$', '%', '^', '&', '*', '(', ')'];

/// Registers that bridge the keyboard hub to the rest of the runtime.
struct KeyboardHub {
    queue: VecDeque<KeyEvent>,
    listeners: Vec<KeyboardCallback>,
    capacity: usize,
    state: KeyboardState,
}

impl KeyboardHub {
    /// Construct a new hub with an empty queue and listener list.
    fn new() -> Self {
        Self {
            queue: VecDeque::with_capacity(DEFAULT_QUEUE_CAPACITY),
            listeners: Vec::new(),
            capacity: DEFAULT_QUEUE_CAPACITY,
            state: KeyboardState::default(),
        }
    }
}

/// Map `usize` identifiers to listeners so we can hand out stable handles.
#[derive(Clone)]
struct KeyboardCallback {
    id: usize,
    func: fn(&KeyEvent),
}

/// Iterator-friendly handle used for listener removal if desired.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ListenerHandle(usize);

/// Thrown when a listener cannot be registered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegisterListenerError {
    /// Listener table is at capacity.
    ListenerTableFull,
}

/// Indicates whether a key transitioned to the pressed or released state.
#[derive(Clone, Debug, Copy, PartialEq, Eq)]
pub enum KeyTransition {
    /// Key is actively pressed.
    Pressed,
    /// Key returned to the unpressed state.
    Released,
}

/// High-level event surfaced whenever the keyboard state changes.
#[derive(Clone, Debug)]
pub struct KeyEvent {
    /// Which direction the key transitioned.
    pub transition: KeyTransition,
    /// HID usage ID taken directly from the boot protocol report.
    pub usage: u8,
    /// Human-readable label that corresponds to the usage.
    pub label: &'static str,
    /// Printable ASCII representation if this transition corresponds to a character.
    pub ascii: Option<char>,
    /// Snapshot of active modifiers when the event was enqueued.
    pub modifiers: Modifiers,
}

static KEYBOARD_HUB: Once<SpinMutex<KeyboardHub>> = Once::new();
static NEXT_LISTENER_ID: Once<SpinMutex<usize>> = Once::new();

/// Lazily create (and then return) the singleton keyboard hub.
fn hub() -> &'static SpinMutex<KeyboardHub> {
    KEYBOARD_HUB.call_once(|| SpinMutex::new(KeyboardHub::new()))
}

/// Provide access to the monotonically increasing listener identifier.
fn next_listener_id() -> &'static SpinMutex<usize> {
    NEXT_LISTENER_ID.call_once(|| SpinMutex::new(0))
}

/// Push a keyboard event into the shared queue and notify registered listeners.
///
/// The queue retains the most recent `DEFAULT_QUEUE_CAPACITY` transitions to
/// provide a convenient historical view for debugging tools.  Listeners are
/// invoked synchronously after the queue is updated; callers should avoid
/// deadlocks by keeping their handlers short and non-blocking.
pub fn publish_event(mut event: KeyEvent) {
    let listeners = {
        let mut hub = hub().lock();
        event.ascii = translate_ascii(&hub.state, &event);
        hub.state.apply(&event);
        event.modifiers = hub.state.snapshot();
        if hub.queue.len() == hub.capacity {
            hub.queue.pop_front();
        }
        hub.queue.push_back(event.clone());
        hub.listeners.clone()
    };

    for listener in listeners {
        (listener.func)(&event);
    }
}

/// Retrieve the next keyboard event from the queue, if any.
pub fn pop_event() -> Option<KeyEvent> {
    hub().lock().queue.pop_front()
}

/// Subscribe to keyboard events and receive a handle for future removal.
///
/// # Errors
///
/// Returns `RegisterListenerError::ListenerTableFull` if the listener backing
/// store has reached `MAX_LISTENERS`.
pub fn register_listener(callback: fn(&KeyEvent)) -> Result<ListenerHandle, RegisterListenerError> {
    let mut id_lock = next_listener_id().lock();
    let id = *id_lock;

    let mut hub = hub().lock();
    if hub.listeners.len() == MAX_LISTENERS {
        return Err(RegisterListenerError::ListenerTableFull);
    }

    hub.listeners.push(KeyboardCallback { id, func: callback });
    *id_lock = id + 1;

    Ok(ListenerHandle(id))
}

/// Remove a previously registered listener.
///
/// It is safe to call this with a handle that has already been removed; the
/// function simply leaves the listener table unchanged.
pub fn unregister_listener(handle: ListenerHandle) {
    hub().lock().listeners.retain(|entry| entry.id != handle.0);
}

/// Track modifier state so ASCII translation can account for Shift and Caps Lock.
#[derive(Clone, Debug, Default)]
struct KeyboardState {
    left_shift: bool,
    right_shift: bool,
    left_ctrl: bool,
    right_ctrl: bool,
    left_alt: bool,
    right_alt: bool,
    left_gui: bool,
    right_gui: bool,
    caps_lock: bool,
}

impl KeyboardState {
    /// Apply the provided event to the modifier bookkeeping.
    fn apply(&mut self, event: &KeyEvent) {
        match event.usage {
            0xE0 => self.left_ctrl = event.transition == KeyTransition::Pressed,
            0xE1 => self.left_shift = event.transition == KeyTransition::Pressed,
            0xE2 => self.left_alt = event.transition == KeyTransition::Pressed,
            0xE3 => self.left_gui = event.transition == KeyTransition::Pressed,
            0xE4 => self.right_ctrl = event.transition == KeyTransition::Pressed,
            0xE5 => self.right_shift = event.transition == KeyTransition::Pressed,
            0xE6 => self.right_alt = event.transition == KeyTransition::Pressed,
            0xE7 => self.right_gui = event.transition == KeyTransition::Pressed,
            0x39 if event.transition == KeyTransition::Pressed => {
                self.caps_lock = !self.caps_lock;
            }
            _ => {}
        }
    }

    /// Return a snapshot suitable for attaching to outbound events.
    fn snapshot(&self) -> Modifiers {
        Modifiers {
            shift: self.left_shift || self.right_shift,
            ctrl: self.left_ctrl || self.right_ctrl,
            alt: self.left_alt || self.right_alt,
            gui: self.left_gui || self.right_gui,
            caps_lock: self.caps_lock,
        }
    }
}

/// Report which modifiers were held when an event was generated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Modifiers {
    /// Any Shift key (left or right).
    pub shift: bool,
    /// Any Control key (left or right).
    pub ctrl: bool,
    /// Any Alt/Option key (left or right).
    pub alt: bool,
    /// Any GUI/Super key (left or right).
    pub gui: bool,
    /// Whether Caps Lock is currently toggled on.
    pub caps_lock: bool,
}

/// Produce an ASCII character if the usage represents printable output.
fn translate_ascii(state: &KeyboardState, event: &KeyEvent) -> Option<char> {
    if event.transition != KeyTransition::Pressed {
        return None;
    }

    if (0xE0..=0xE7).contains(&event.usage) {
        return None;
    }

    let shift_active = state.left_shift || state.right_shift;
    let effective_upper = shift_active ^ state.caps_lock;

    match event.usage {
        0x04..=0x1d => {
            let base = (event.usage - 0x04) as u8;
            let letter = (b'a' + base) as char;
            if effective_upper {
                Some(letter.to_ascii_uppercase())
            } else {
                Some(letter)
            }
        }
        0x1e..=0x27 => {
            let idx = (event.usage - 0x1e) as usize;
            if shift_active {
                Some(NUMBER_SHIFT[idx])
            } else {
                Some(NUMBER_BASE[idx])
            }
        }
        0x28 => Some('\n'),
        0x2b => Some('\t'),
        0x2c => Some(' '),
        0x2d => Some(if shift_active { '_' } else { '-' }),
        0x2e => Some(if shift_active { '+' } else { '=' }),
        0x2f => Some(if shift_active { '{' } else { '[' }),
        0x30 => Some(if shift_active { '}' } else { ']' }),
        0x31 => Some(if shift_active { '|' } else { '\\' }),
        0x33 => Some(if shift_active { ':' } else { ';' }),
        0x34 => Some(if shift_active { '"' } else { '\'' }),
        0x35 => Some(if shift_active { '~' } else { '`' }),
        0x36 => Some(if shift_active { '<' } else { ',' }),
        0x37 => Some(if shift_active { '>' } else { '.' }),
        0x38 => Some(if shift_active { '?' } else { '/' }),
        _ => None,
    }
}

impl KeyEvent {
    /// Convenience constructor to keep callsites tidy.
    pub fn new(transition: KeyTransition, usage: u8, label: &'static str) -> Self {
        Self {
            transition,
            usage,
            label,
            ascii: None,
            modifiers: Modifiers::default(),
        }
    }
}
