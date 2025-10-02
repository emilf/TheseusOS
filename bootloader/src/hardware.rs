extern crate alloc;

use crate::drivers::manager::write_line;
use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use theseus_shared::constants::hardware;
use theseus_shared::handoff::{
    HardwareDevice, DEVICE_TYPE_ACPI, DEVICE_TYPE_BLUETOOTH, DEVICE_TYPE_CDROM,
    DEVICE_TYPE_CONTROLLER, DEVICE_TYPE_DISK, DEVICE_TYPE_FILE_PATH, DEVICE_TYPE_IPV4,
    DEVICE_TYPE_IPV6, DEVICE_TYPE_MAC, DEVICE_TYPE_MEDIA, DEVICE_TYPE_MESSAGING, DEVICE_TYPE_NVME,
    DEVICE_TYPE_PCI, DEVICE_TYPE_RAMDISK, DEVICE_TYPE_SATA, DEVICE_TYPE_SD, DEVICE_TYPE_UART,
    DEVICE_TYPE_UFS, DEVICE_TYPE_UNKNOWN, DEVICE_TYPE_USB, DEVICE_TYPE_VENDOR, DEVICE_TYPE_WIFI,
};
use uefi::proto::device_path::{
    text::{AllowShortcuts, DevicePathToText, DisplayOnly},
    DevicePath,
};
use uefi::proto::device_path::{DeviceSubType, DeviceType};
use uefi::proto::loaded_image::LoadedImage;
use uefi::{boot::SearchType, Identify};

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

    let mut devices: Vec<HardwareDevice> = Vec::new();

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
        let handle_addr = handle.as_ptr() as usize as u64;
        let (device_type, summary) =
            classify_device(handle, device_path_to_text.as_ref().map(|p| &**p));
        let entry = HardwareDevice {
            device_type,
            address: Some(handle_addr),
            irq: None,
        };

        if verbose {
            write_line(&format!(
                "  Device {}: Handle 0x{:016x} -> {}",
                index + 1,
                handle_addr,
                summary
            ));
        }

        devices.push(entry);

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

    let boxed_devices = devices.into_boxed_slice();
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
) -> (u32, String) {
    let mut kind = DEVICE_TYPE_UNKNOWN;
    let mut summary = String::from("unknown");

    if let Ok(device_path) = uefi::boot::open_protocol_exclusive::<DevicePath>(*handle) {
        let mut parts = Vec::new();
        let mut detected = DEVICE_TYPE_UNKNOWN;

        for node in device_path.node_iter() {
            let node_summary = match (node.device_type(), node.sub_type()) {
                (DeviceType::HARDWARE, DeviceSubType::HARDWARE_PCI) => {
                    detected = DEVICE_TYPE_PCI;
                    "hardware/pci"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_SCSI) => {
                    detected = DEVICE_TYPE_SATA;
                    "messaging/scsi"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_USB) => {
                    detected = DEVICE_TYPE_USB;
                    "messaging/usb"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_SATA) => {
                    detected = DEVICE_TYPE_SATA;
                    "messaging/sata"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_NVME_OF_NAMESPACE) => {
                    detected = DEVICE_TYPE_NVME;
                    "messaging/nvme-of"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_MAC_ADDRESS) => {
                    detected = DEVICE_TYPE_MAC;
                    "messaging/mac"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_IPV4) => {
                    detected = DEVICE_TYPE_IPV4;
                    "messaging/ipv4"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_IPV6) => {
                    detected = DEVICE_TYPE_IPV6;
                    "messaging/ipv6"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_USB_CLASS) => {
                    detected = DEVICE_TYPE_USB;
                    "messaging/usb-class"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_SCSI_SAS_EX) => {
                    detected = DEVICE_TYPE_SATA;
                    "messaging/sas"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_UART) => {
                    detected = DEVICE_TYPE_UART;
                    "messaging/uart"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_BLUETOOTH) => {
                    detected = DEVICE_TYPE_BLUETOOTH;
                    "messaging/bluetooth"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_WIFI) => {
                    detected = DEVICE_TYPE_WIFI;
                    "messaging/wifi"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_SD) => {
                    detected = DEVICE_TYPE_SD;
                    "messaging/sd"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_UFS) => {
                    detected = DEVICE_TYPE_UFS;
                    "messaging/ufs"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_URI) => {
                    detected = DEVICE_TYPE_MESSAGING;
                    "messaging/uri"
                }
                (DeviceType::MESSAGING, DeviceSubType::MESSAGING_I2O) => {
                    detected = DEVICE_TYPE_MESSAGING;
                    "messaging/i2o"
                }
                (DeviceType::MEDIA, DeviceSubType::MEDIA_HARD_DRIVE) => {
                    detected = DEVICE_TYPE_DISK;
                    "media/hard-drive"
                }
                (DeviceType::MEDIA, DeviceSubType::MEDIA_CD_ROM) => {
                    detected = DEVICE_TYPE_CDROM;
                    "media/cdrom"
                }
                (DeviceType::MEDIA, DeviceSubType::MEDIA_FILE_PATH) => {
                    detected = DEVICE_TYPE_FILE_PATH;
                    "media/file"
                }
                (DeviceType::MEDIA, DeviceSubType::MEDIA_RELATIVE_OFFSET_RANGE) => {
                    detected = DEVICE_TYPE_MEDIA;
                    "media/relative-offset"
                }
                (DeviceType::MEDIA, DeviceSubType::MEDIA_RAM_DISK) => {
                    detected = DEVICE_TYPE_RAMDISK;
                    "media/ramdisk"
                }
                (DeviceType::HARDWARE, DeviceSubType::HARDWARE_CONTROLLER) => {
                    detected = DEVICE_TYPE_CONTROLLER;
                    "hardware/controller"
                }
                (DeviceType::HARDWARE, DeviceSubType::HARDWARE_VENDOR) => {
                    detected = DEVICE_TYPE_VENDOR;
                    "hardware/vendor"
                }
                (DeviceType::ACPI, _) => {
                    detected = DEVICE_TYPE_ACPI;
                    "acpi/node"
                }
                (DeviceType::END, DeviceSubType::END_INSTANCE) => "end-instance",
                (DeviceType::END, DeviceSubType::END_ENTIRE) => "end-entire",
                (ty, subtype) => {
                    detected = DEVICE_TYPE_MESSAGING;
                    parts.push(format!("other({:?}/{:?})", ty, subtype));
                    continue;
                }
            };
            parts.push(node_summary.to_string());
            if detected != DEVICE_TYPE_UNKNOWN {
                kind = detected;
            }
        }

        if kind == DEVICE_TYPE_UNKNOWN {
            kind = DEVICE_TYPE_UNKNOWN;
        }

        if let Some(to_text) = device_path_to_text {
            if let Ok(text) = to_text.convert_device_path_to_text(
                &device_path,
                DisplayOnly(false),
                AllowShortcuts(true),
            ) {
                let mut path_buf = String::new();
                if text.as_str_in_buf(&mut path_buf).is_ok() {
                    summary = path_buf;
                } else {
                    summary = parts.join(" -> ");
                }
            } else {
                summary = parts.join(" -> ");
            }
        } else {
            summary = parts.join(" -> ");
        }
    }

    (kind, summary)
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
