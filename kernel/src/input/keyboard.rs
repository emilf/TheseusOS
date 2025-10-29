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

/// Registers that bridge the keyboard hub to the rest of the runtime.
struct KeyboardHub {
    queue: VecDeque<KeyEvent>,
    listeners: Vec<KeyboardCallback>,
    capacity: usize,
}

impl KeyboardHub {
    /// Construct a new hub with an empty queue and listener list.
    fn new() -> Self {
        Self {
            queue: VecDeque::with_capacity(DEFAULT_QUEUE_CAPACITY),
            listeners: Vec::new(),
            capacity: DEFAULT_QUEUE_CAPACITY,
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
pub fn publish_event(event: KeyEvent) {
    let listeners = {
        let mut hub = hub().lock();
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
