extern crate alloc;

use crate::drivers::manager::write_line;
use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::convert::TryFrom;
use theseus_shared::constants::hardware;
use theseus_shared::handoff::{
    HardwareDevice, DEVICE_TYPE_ACPI, DEVICE_TYPE_PCI, DEVICE_TYPE_SERIAL, DEVICE_TYPE_UNKNOWN,
    DEVICE_TYPE_USB,
};
use uefi::proto::device_path::{
    acpi,
    hardware::{MemoryMapped, Pci},
    messaging::{Usb, UsbClass, UsbWwid},
    DevicePathNode, DeviceSubType, DeviceType,
};
use uefi::proto::device_path::{
    text::{AllowShortcuts, DevicePathToText, DisplayOnly},
    DevicePath,
};
use uefi::proto::loaded_image::LoadedImage;
use uefi::{boot::SearchType, Identify};

struct ClassifiedDevice {
    device: HardwareDevice,
    summary: String,
}

/// Static storage for hardware device inventory data.
///
/// This storage keeps the device array alive across UEFI boot services exit,
/// allowing the kernel to safely access the hardware inventory data after
/// the bootloader has completed execution.
///
/// # Safety
/// This is safe to use in the UEFI bootloader context because:
/// - It is only written to during single-threaded UEFI boot process
/// - No concurrent access occurs during bootloader execution
/// - The kernel receives a copy via the handoff structure and doesn't modify this original
static mut INVENTORY_STORAGE: Option<Box<[HardwareDevice]>> = None;

/// Hardware inventory structure containing metadata about discovered devices.
///
/// This structure is used internally by the bootloader to track the hardware
/// inventory before it's passed to the kernel via the handoff structure.
///
/// # Layout
/// The structure is marked `#[repr(C)]` to ensure stable layout for
/// handoff to the kernel.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct HardwareInventory {
    /// Number of hardware devices discovered
    pub device_count: u32,
    /// Pointer to the array of hardware device structures
    pub devices_ptr: u64,
    /// Total size of the inventory data in bytes
    pub total_size: u64,
}

/// Collect hardware inventory using UEFI device paths.
///
/// This function discovers all UEFI handles that support the DevicePath protocol,
/// analyzes their device paths to classify device types, and creates a persistent
/// inventory that survives boot services exit.
///
/// # Arguments
/// * `verbose` - Whether to print detailed device information during discovery
///
/// # Returns
/// * `Some(HardwareInventory)` - Successfully collected inventory with metadata
/// * `None` - Failed to discover devices or create inventory
///
/// # Process
/// 1. Locate all UEFI handles supporting DevicePath protocol
/// 2. For each handle, open the DevicePath protocol and analyze the path
/// 3. Classify the device based on device path node types
/// 4. Generate human-readable device descriptions when possible
/// 5. Store devices in a boxed slice and leak it to static storage
/// 6. Return inventory metadata for handoff to kernel
///
/// # Device Classification
/// Devices are classified by walking their UEFI device paths and mapping
/// node types to standardized device categories:
/// - PCI devices: `DeviceType::HARDWARE + DeviceSubType::HARDWARE_PCI`
/// - USB devices: `DeviceType::MESSAGING + DeviceSubType::MESSAGING_USB`
/// - SATA devices: `DeviceType::MESSAGING + DeviceSubType::MESSAGING_SATA`
/// - ACPI devices: `DeviceType::ACPI + any subtype`
/// - And many more mappings defined in `classify_device()`
///
/// # Memory Management
/// The function uses a memory leak strategy to keep device data alive:
/// ```rust
/// let boxed_devices = devices.into_boxed_slice();
/// unsafe {
///     INVENTORY_STORAGE = Some(boxed_devices);
/// }
/// ```
/// This ensures the device array remains valid even after boot services exit.
///
/// # Limitations
/// - Limited to `hardware::MAX_HARDWARE_DEVICES` (1000) devices to prevent hangs
/// - Depends on UEFI `DevicePathToText` protocol for human-readable descriptions
/// - Some device types may remain unclassified if their paths aren't recognized
pub fn collect_hardware_inventory(verbose: bool) -> Option<HardwareInventory> {
    write_line("Collecting hardware inventory using device paths...");

    let mut devices: Vec<ClassifiedDevice> = Vec::new();

    // Find all handles that support DevicePath protocol (more comprehensive)
    let handles = match uefi::boot::locate_handle_buffer(SearchType::ByProtocol(&DevicePath::GUID))
    {
        Ok(handles) => handles,
        Err(_) => {
            write_line("✗ Failed to locate device path handles");
            return None;
        }
    };

    write_line(&format!("Found {} device path handles", handles.len()));

    if verbose {
        write_line("Verbose mode: Displaying detailed device information...");
    }

    // Get DevicePathToText protocol for converting paths to human-readable text
    // First, find a handle that supports the DevicePathToText protocol
    let device_path_to_text = match uefi::boot::get_handle_for_protocol::<DevicePathToText>() {
        Ok(handle) => {
            // Now open the protocol on the found handle
            match uefi::boot::open_protocol_exclusive::<DevicePathToText>(handle) {
                Ok(protocol) => Some(protocol),
                Err(_) => {
                    if verbose {
                        write_line("  Warning: Failed to open DevicePathToText protocol");
                    }
                    None
                }
            }
        }
        Err(_) => {
            if verbose {
                write_line(
                    "  Warning: DevicePathToText protocol not available, showing raw pointers only",
                );
            }
            None
        }
    };

    // Enumerate all handles and get device path information
    for (index, handle) in handles.iter().enumerate() {
        let classified = classify_device(handle, device_path_to_text.as_ref().map(|p| &**p));

        if verbose {
            write_line(&format!(
                "  Device {}: Handle 0x{:016x} -> {}",
                index + 1,
                classified.device.address.unwrap_or(0),
                classified.summary
            ));
        }

        devices.push(classified);

        if devices.len() >= hardware::MAX_HARDWARE_DEVICES {
            if verbose {
                write_line(&format!(
                    "  ... (limiting to first {} devices)",
                    hardware::MAX_HARDWARE_DEVICES
                ));
            }
            break;
        }
    }

    if devices.is_empty() {
        write_line("✗ No hardware devices found");
        return None;
    }

    let boxed_devices: Vec<HardwareDevice> = devices.iter().map(|c| c.device).collect();
    let boxed_devices = boxed_devices.into_boxed_slice();
    let device_count = boxed_devices.len();
    let devices_ptr = boxed_devices.as_ptr() as u64;
    let total_size = (device_count * core::mem::size_of::<HardwareDevice>()) as u64;

    let inventory = HardwareInventory {
        device_count: device_count as u32,
        devices_ptr,
        total_size,
    };

    unsafe {
        INVENTORY_STORAGE = Some(boxed_devices);
    }

    if verbose {
        write_line(&format!(
            "✓ Hardware inventory collected: {} devices",
            inventory.device_count
        ));
        write_line(&format!(
            "  Device struct size: {} bytes",
            core::mem::size_of::<HardwareDevice>()
        ));
        write_line(&format!(
            "  Total inventory size: {} bytes ({:.2} KB)",
            inventory.total_size,
            inventory.total_size as f64 / 1024.0
        ));
        write_line(&format!(
            "  Devices array pointer: 0x{:016x}",
            inventory.devices_ptr
        ));
    } else {
        write_line(&format!(
            "✓ Hardware inventory collected: {} devices",
            inventory.device_count
        ));
    }

    Some(inventory)
}

/// Classify a UEFI device handle based on its device path.
///
/// This function analyzes the device path of a UEFI handle to determine its
/// device type and generate a human-readable description. It walks through
/// all nodes in the device path and maps them to standardized device categories.
///
/// # Arguments
/// * `handle` - The UEFI handle to classify
/// * `device_path_to_text` - Optional DevicePathToText protocol for human-readable descriptions
///
/// # Returns
/// A tuple containing:
/// * `u32` - The device type constant (e.g., `DEVICE_TYPE_PCI`, `DEVICE_TYPE_USB`)
/// * `String` - Human-readable description of the device
///
/// # Device Path Analysis
/// The function iterates through each node in the device path and maps
/// (DeviceType, DeviceSubType) combinations to device categories:
///
/// ```rust
/// match (node.device_type(), node.sub_type()) {
///     (DeviceType::HARDWARE, DeviceSubType::HARDWARE_PCI) => DEVICE_TYPE_PCI,
///     (DeviceType::MESSAGING, DeviceSubType::MESSAGING_USB) => DEVICE_TYPE_USB,
///     (DeviceType::MESSAGING, DeviceSubType::MESSAGING_SATA) => DEVICE_TYPE_SATA,
///     (DeviceType::ACPI, _) => DEVICE_TYPE_ACPI,
///     // ... many more mappings
/// }
/// ```
///
/// # Description Generation
/// The function attempts to generate human-readable descriptions in two ways:
/// 1. **UEFI Text Protocol**: If `DevicePathToText` is available, use UEFI's
///    built-in device path to text conversion for the most accurate descriptions
/// 2. **Node-based Fallback**: If the protocol isn't available, build descriptions
///    from individual node types (e.g., "hardware/pci -> messaging/usb")
///
/// # Classification Priority
/// When multiple device types are found in a single path, the function uses
/// the last non-unknown type encountered, giving priority to more specific
/// device types over generic ones.
///
/// # Supported Device Types
/// The function recognizes and classifies many UEFI device path node types:
/// - **Hardware**: PCI, controllers, vendor-specific hardware
/// - **Messaging**: USB, SATA, NVMe, network interfaces, serial ports
/// - **Media**: Hard drives, CD-ROMs, file paths, RAM disks
/// - **ACPI**: ACPI device nodes and expanded ACPI nodes
/// - **End nodes**: Instance and entire path terminators
///
/// # Limitations
/// - Some exotic or vendor-specific device types may remain unclassified
/// - Device paths with unrecognized node types are marked as `DEVICE_TYPE_UNKNOWN`
/// - The function doesn't extract detailed device information (IDs, capabilities, etc.)
fn classify_device(
    handle: &uefi::Handle,
    device_path_to_text: Option<&DevicePathToText>,
) -> ClassifiedDevice {
    let mut result = ClassifiedDevice {
        device: HardwareDevice {
            device_type: DEVICE_TYPE_UNKNOWN,
            address: Some(handle.as_ptr() as usize as u64),
            irq: None,
        },
        summary: String::from("unknown"),
    };

    if let Ok(device_path) = uefi::boot::open_protocol_exclusive::<DevicePath>(*handle) {
        let mut classified = classify_device_path(&device_path);
        if let Some(text) = device_path_to_text.and_then(|to_text| {
            to_text
                .convert_device_path_to_text(&device_path, DisplayOnly(false), AllowShortcuts(true))
                .ok()
        }) {
            let mut buf = String::new();
            if text.as_str_in_buf(&mut buf).is_ok() {
                classified.summary = buf;
            }
        }
        if classified.device.device_type != DEVICE_TYPE_UNKNOWN {
            result.device.device_type = classified.device.device_type;
        }
        if classified.device.address.is_some() {
            result.device.address = classified.device.address;
        }
        result.summary = classified.summary;
    }

    result
}

fn classify_device_path(path: &DevicePath) -> ClassifiedDevice {
    let mut device = HardwareDevice {
        device_type: DEVICE_TYPE_UNKNOWN,
        address: None,
        irq: None,
    };
    let mut parts = Vec::new();
    let mut found_serial = false;

    for node in path.node_iter() {
        let info = classify_device_node(&node);
        if info.device.device_type == DEVICE_TYPE_SERIAL {
            found_serial = true;
        }
        if device.device_type == DEVICE_TYPE_UNKNOWN
            && info.device.device_type != DEVICE_TYPE_UNKNOWN
        {
            device.device_type = info.device.device_type;
        }
        if device.address.is_none() && info.device.address.is_some() {
            device.address = info.device.address;
        }
        if info.device.irq.is_some() {
            device.irq = info.device.irq;
        }
        parts.push(info.summary);
    }

    if found_serial {
        device.device_type = DEVICE_TYPE_SERIAL;
        if device.address.is_none() {
            device.address = Some(0x3F8);
        }
    }

    ClassifiedDevice {
        device,
        summary: parts.join(" -> "),
    }
}

fn classify_device_node(node: &DevicePathNode) -> ClassifiedDevice {
    match node.full_type() {
        (DeviceType::ACPI, DeviceSubType::ACPI) => classify_acpi_node(node),
        (DeviceType::HARDWARE, DeviceSubType::HARDWARE_PCI) => classify_pci_node(node),
        (DeviceType::MESSAGING, DeviceSubType::MESSAGING_UART) => classify_uart_node(),
        (DeviceType::MESSAGING, DeviceSubType::MESSAGING_USB) => classify_usb_node(node),
        (DeviceType::MESSAGING, DeviceSubType::MESSAGING_USB_WWID) => classify_usb_wwid_node(node),
        (DeviceType::MESSAGING, DeviceSubType::MESSAGING_USB_CLASS) => {
            classify_usb_class_node(node)
        }
        (DeviceType::HARDWARE, DeviceSubType::HARDWARE_MEMORY_MAPPED) => {
            classify_memory_mapped_node(node)
        }
        _ => ClassifiedDevice {
            device: HardwareDevice {
                device_type: DEVICE_TYPE_UNKNOWN,
                address: None,
                irq: None,
            },
            summary: format!("other({:?}/{:?})", node.device_type(), node.sub_type()),
        },
    }
}

fn classify_acpi_node(node: &DevicePathNode) -> ClassifiedDevice {
    let acpi: &acpi::Acpi = node.try_into().unwrap();
    let hid = acpi.hid();
    let uid = acpi.uid();
    let eisa = decode_eisa_id(hid);
    let summary = if let Some(ref code) = eisa {
        format!("acpi(hid={}, uid={})", code, uid)
    } else {
        format!("acpi(hid={:08x}, uid={})", hid, uid)
    };
    let device_type = if let Some(ref code) = eisa {
        if is_serial_eisa(code) {
            DEVICE_TYPE_SERIAL
        } else {
            DEVICE_TYPE_ACPI
        }
    } else {
        DEVICE_TYPE_ACPI
    };
    ClassifiedDevice {
        device: HardwareDevice {
            device_type,
            address: None,
            irq: None,
        },
        summary,
    }
}

fn classify_pci_node(node: &DevicePathNode) -> ClassifiedDevice {
    let summary = if let Ok(pci) = <&Pci>::try_from(node) {
        format!(
            "hardware/pci(device={}, function={})",
            pci.device(),
            pci.function()
        )
    } else {
        String::from("hardware/pci")
    };
    ClassifiedDevice {
        device: HardwareDevice {
            device_type: DEVICE_TYPE_PCI,
            address: None,
            irq: None,
        },
        summary,
    }
}

fn classify_usb_node(node: &DevicePathNode) -> ClassifiedDevice {
    let summary = if let Ok(usb) = <&Usb>::try_from(node) {
        format!(
            "messaging/usb(port={}, interface={})",
            usb.parent_port_number(),
            usb.interface()
        )
    } else {
        String::from("messaging/usb")
    };
    ClassifiedDevice {
        device: HardwareDevice {
            device_type: DEVICE_TYPE_USB,
            address: None,
            irq: None,
        },
        summary,
    }
}

fn classify_usb_wwid_node(node: &DevicePathNode) -> ClassifiedDevice {
    let summary = if let Ok(wwid) = <&UsbWwid>::try_from(node) {
        format!(
            "messaging/usb-wwid(vendor={:#06x}, product={:#06x}, interface={})",
            wwid.device_vendor_id(),
            wwid.device_product_id(),
            wwid.interface_number()
        )
    } else {
        String::from("messaging/usb-wwid")
    };
    ClassifiedDevice {
        device: HardwareDevice {
            device_type: DEVICE_TYPE_USB,
            address: None,
            irq: None,
        },
        summary,
    }
}

fn classify_usb_class_node(node: &DevicePathNode) -> ClassifiedDevice {
    let summary = if let Ok(class) = <&UsbClass>::try_from(node) {
        format!(
            "messaging/usb-class(class={:#04x}, subclass={:#04x}, protocol={:#04x}, vid={:#06x}, pid={:#06x})",
            class.device_class(),
            class.device_subclass(),
            class.device_protocol(),
            class.vendor_id(),
            class.product_id()
        )
    } else {
        String::from("messaging/usb-class")
    };
    ClassifiedDevice {
        device: HardwareDevice {
            device_type: DEVICE_TYPE_USB,
            address: None,
            irq: None,
        },
        summary,
    }
}

fn classify_uart_node() -> ClassifiedDevice {
    ClassifiedDevice {
        device: HardwareDevice {
            device_type: DEVICE_TYPE_SERIAL,
            address: None,
            irq: None,
        },
        summary: String::from("messaging/uart"),
    }
}

fn classify_memory_mapped_node(node: &DevicePathNode) -> ClassifiedDevice {
    let mmio: &MemoryMapped = node.try_into().unwrap();
    let summary = format!(
        "memory-mapped(type={:?}, start=0x{:x}, end=0x{:x})",
        mmio.memory_type(),
        mmio.start_address(),
        mmio.end_address()
    );
    ClassifiedDevice {
        device: HardwareDevice {
            device_type: DEVICE_TYPE_UNKNOWN,
            address: Some(mmio.start_address()),
            irq: None,
        },
        summary,
    }
}

fn is_serial_eisa(code: &str) -> bool {
    SERIAL_EISA_CODES.contains(&code)
}

const SERIAL_EISA_CODES: &[&str] = &[
    "PNP0500", "PNP0501", "PNP0502", "PNP0503", "PNP0504", "PNP0505", "PNP0506", "PNP0507",
    "PNP0508", "PNP0509", "PNP0510", "PNP0511", "PNP0512", "PNP0513", "PNP0514", "PNP0515",
    "PNP0516", "PNP0517", "PNP0518", "PNP0519", "PNP0520", "PNP0521", "PNP0522", "PNP0523",
    "PNP0524", "PNP0525", "PNP0526", "PNP0527", "PNP0528", "PNP0529", "PNP052A", "PNP052B",
    "PNP052C", "PNP052D", "PNP052E", "PNP052F", "PNP0530", "PNP0531", "PNP0532", "PNP0533",
    "PNP0534", "PNP0535", "PNP0536", "PNP0537", "PNP0538", "PNP0539", "PNP053A", "PNP053B",
    "PNP053C", "PNP053D", "PNP053E", "PNP053F", "PNP0540", "PNP0541", "PNP0542", "PNP0543",
    "PNP0544", "PNP0545", "PNP0546", "PNP0547", "PNP0548", "PNP0549", "PNP054A", "PNP054B",
    "PNP054C", "PNP054D", "PNP054E", "PNP054F", "PNP0550", "PNP0551", "PNP0552", "PNP0553",
    "PNP0554", "PNP0555", "PNP0556", "PNP0557", "PNP0558", "PNP0559", "PNP055A", "PNP055B",
    "PNP055C", "PNP055D", "PNP055E", "PNP055F", "PNP0560", "PNP0561", "PNP0562", "PNP0563",
    "PNP0564", "PNP0565", "PNP0566", "PNP0567", "PNP0568", "PNP0569", "PNP056A", "PNP056B",
    "PNP056C", "PNP056D", "PNP056E", "PNP056F", "PNP0570", "PNP0571", "PNP0572", "PNP0573",
    "PNP0574", "PNP0575", "PNP0576", "PNP0577", "PNP0578", "PNP0579", "PNP057A", "PNP057B",
    "PNP057C", "PNP057D", "PNP057E", "PNP057F", "IBM0004", "IBM0005", "IBM0006", "IBM0007",
    "IBM0008", "IBM0009", "IBM000A", "IBM000B", "IBM000C", "IBM000D", "IBM000E", "IBM000F",
    "PNP0001", "PNP0103", "PNP0C07", "PNP0C09", "PNP0A03", "PNP0A08", "PNP0C0F", "PNP0F13",
    "PNP0F0E", "PNP0F0D", "PNP0F0C", "PNP0F0B", "PNP0F0A", "PNP0F09", "PNP0F08", "PNP0F07",
    "PNP0F06", "PNP0F05", "PNP0F04", "PNP0F03", "PNP0F02", "PNP0F01", "PNP0F00", "PNP0000",
];

fn decode_eisa_id(raw: u32) -> Option<String> {
    let mut chars = [0u8; 3];
    chars[0] = ((((raw >> 10) & 0x1F) + 0x40) as u8).to_ascii_uppercase();
    chars[1] = ((((raw >> 5) & 0x1F) + 0x40) as u8).to_ascii_uppercase();
    chars[2] = ((((raw >> 0) & 0x1F) + 0x40) as u8).to_ascii_uppercase();
    if !chars.iter().all(|&c| c.is_ascii_alphabetic()) {
        return None;
    }
    let product = ((raw >> 16) & 0xFFFF) as u16;
    Some(format!(
        "{}{}{}{:04X}",
        chars[0] as char, chars[1] as char, chars[2] as char, product
    ))
}

/// Display hardware inventory in a formatted table.
///
/// This function provides a human-readable summary of the collected hardware
/// inventory, including device counts, memory usage, and device type summaries.
///
/// # Arguments
/// * `inventory` - The hardware inventory to display
///
/// # Output Format
/// The function creates a bordered table showing:
/// - Total number of devices discovered
/// - Memory usage (in bytes and KB)
/// - Device array pointer address
/// - Summary of device types found
///
/// # Example Output
/// ```
/// ┌─────────────────────────────────────────────────────────┐
/// │                Hardware Device Inventory               │
/// ├─────────────────────────────────────────────────────────┤
/// │ Total Devices Found: 22                              │
/// │ Inventory Size: 704 bytes (0.69 KB)                  │
/// │ Devices Pointer: 0x000000003EAB9018                  │
/// ├─────────────────────────────────────────────────────────┤
/// │ Device Type Summary:                                  │
/// │   Device Path Handles: Found                         │
/// │   Human-Readable Paths: Available                     │
/// │   Protocol Support: DevicePathToText                  │
/// └─────────────────────────────────────────────────────────┘
/// ```
pub fn display_hardware_inventory(inventory: &HardwareInventory) {
    write_line("");
    write_line("┌─────────────────────────────────────────────────────────┐");
    write_line("│                Hardware Device Inventory               │");
    write_line("├─────────────────────────────────────────────────────────┤");

    // Bounds check for device count
    if inventory.device_count == 0 {
        write_line("│ No hardware devices found                            │");
        write_line("└─────────────────────────────────────────────────────────┘");
        write_line("");
        return;
    }

    // Sanity check: ensure device count is reasonable
    if inventory.device_count > hardware::MAX_HARDWARE_DEVICES as u32 {
        write_line(&format!(
            "│ Warning: Device count ({}) exceeds maximum ({}) │",
            inventory.device_count,
            hardware::MAX_HARDWARE_DEVICES
        ));
    }

    write_line(&alloc::format!(
        "│ Total Devices Found: {}                              │",
        inventory.device_count
    ));
    write_line(&alloc::format!(
        "│ Inventory Size: {} bytes ({:.2} KB)              │",
        inventory.total_size,
        inventory.total_size as f64 / 1024.0
    ));
    write_line(&alloc::format!(
        "│ Devices Pointer: 0x{:016x}                    │",
        inventory.devices_ptr
    ));
    write_line("├─────────────────────────────────────────────────────────┤");

    // Display device types summary
    write_line("│ Device Type Summary:                                  │");
    write_line("│   Device Path Handles: Found                         │");
    write_line("│   Human-Readable Paths: Available                     │");
    write_line("│   Protocol Support: DevicePathToText                  │");

    write_line("└─────────────────────────────────────────────────────────┘");
    write_line("");
}

/// Get the device path information for the currently loaded EFI image.
///
/// This function retrieves the device path of the EFI image that's currently
/// being executed (i.e., the bootloader itself). This is useful for understanding
/// where the bootloader was loaded from and can be used for debugging or
/// configuration purposes.
///
/// # Returns
/// * `Some((u64, u32))` - Tuple containing (path_pointer, path_size)
/// * `None` - Failed to retrieve device path
///
/// # Process
/// 1. Get the current image handle from UEFI
/// 2. Open the LoadedImage protocol on that handle
/// 3. Retrieve the device path from the loaded image
/// 4. Return the pointer and size (size may be 0 if not easily determinable)
///
/// # Limitations
/// - The path size is often 0 because UEFI DevicePath doesn't expose length easily
/// - The returned pointer is only valid while boot services are active
/// - This information is primarily useful for debugging and logging
pub fn get_loaded_image_device_path() -> Option<(u64, u32)> {
    write_line("Getting loaded image device path...");

    let image_handle = uefi::boot::image_handle();

    // Try to open LoadedImage protocol
    let loaded_image = match uefi::boot::open_protocol_exclusive::<LoadedImage>(image_handle) {
        Ok(img) => img,
        Err(_) => {
            write_line("✗ Failed to open LoadedImage protocol");
            return None;
        }
    };

    // Get the device path
    let device_path = loaded_image.file_path();

    if let Some(path) = device_path {
        // For now, we'll just store the pointer without trying to get size
        // The DevicePath doesn't expose len() and as_ptr() methods directly
        let path_ptr = core::ptr::addr_of!(*path) as *const u8 as usize as u64;
        let path_size = 0; // We can't easily get the size without more complex parsing

        write_line(&format!("✓ Loaded image device path found"));
        write_line(&format!("  Path pointer: 0x{:016x}", path_ptr));

        Some((path_ptr, path_size))
    } else {
        write_line("✗ No device path available for loaded image");
        None
    }
}
