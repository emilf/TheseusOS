use uefi::{prelude::*, proto::console::serial::Serial};

/// Helper function to write data to serial
pub fn serial_write(serial_handle: Option<Handle>, data: &[u8]) {
    if let Some(handle) = serial_handle {
        if let Ok(mut serial) = uefi::boot::open_protocol_exclusive::<Serial>(handle) {
            let _ = serial.write(data);
        }
    }
}

/// Helper function to write a line to serial
pub fn serial_write_line(serial_handle: Option<Handle>, line: &str) {
    serial_write(serial_handle, line.as_bytes());
    serial_write(serial_handle, b"\r\n");
}
