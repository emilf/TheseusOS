# Hardware Inventory System

This document describes the hardware inventory system implemented in TheseusOS, which collects and classifies hardware devices during UEFI boot and makes them available to the kernel.

## Overview

The hardware inventory system operates in two phases:
1. **Bootloader Phase**: Collects UEFI device handles and classifies them based on their device paths
2. **Kernel Phase**: Consumes the inventory data to register devices with the driver system

## Architecture

### Bootloader Components

#### `bootloader/src/hardware.rs`
- **`collect_hardware_inventory()`**: Main entry point that discovers UEFI handles supporting DevicePath protocol
- **`classify_device()`**: Analyzes device path nodes to determine device type and generate human-readable descriptions
- **`HardwareInventory`**: Temporary structure containing device count, pointer, and size
- **`INVENTORY_STORAGE`**: Static storage that keeps device data alive across boot services exit

#### Device Classification
The system classifies devices by walking their UEFI device paths and mapping node types to standardized device categories:

```rust
// Major device path types and their mappings
DeviceType::HARDWARE + DeviceSubType::HARDWARE_PCI -> DEVICE_TYPE_PCI
DeviceType::MESSAGING + DeviceSubType::MESSAGING_USB -> DEVICE_TYPE_USB  
DeviceType::MESSAGING + DeviceSubType::MESSAGING_SATA -> DEVICE_TYPE_SATA
DeviceType::MESSAGING + DeviceSubType::MESSAGING_NVME_NAMESPACE -> DEVICE_TYPE_NVME
DeviceType::ACPI + any subtype -> DEVICE_TYPE_ACPI
DeviceType::MEDIA + DeviceSubType::MEDIA_HARD_DRIVE -> DEVICE_TYPE_DISK
// ... and many more
```

### Shared Components

#### `shared/src/handoff.rs`
- **`HardwareDevice`**: Standardized device structure passed from bootloader to kernel
- **`DEVICE_TYPE_*` constants**: Enumeration of supported device types (0-25)
- **`device_type_str()`**: Human-readable string conversion for device types

#### Device Types
```rust
pub const DEVICE_TYPE_UNKNOWN: u32 = 0;        // Unclassified device
pub const DEVICE_TYPE_CPU: u32 = 1;            // CPU cores
pub const DEVICE_TYPE_IO_APIC: u32 = 2;        // IO APIC controllers
pub const DEVICE_TYPE_PCI: u32 = 3;            // PCI devices
pub const DEVICE_TYPE_USB: u32 = 4;            // USB devices
pub const DEVICE_TYPE_SATA: u32 = 5;           // SATA/SCSI devices
pub const DEVICE_TYPE_NVME: u32 = 6;           // NVMe devices
pub const DEVICE_TYPE_MAC: u32 = 7;            // Network interfaces
pub const DEVICE_TYPE_IPV4: u32 = 8;           // IPv4 network
pub const DEVICE_TYPE_IPV6: u32 = 9;           // IPv6 network
pub const DEVICE_TYPE_ACPI: u32 = 10;          // ACPI devices
pub const DEVICE_TYPE_VENDOR: u32 = 11;        // Vendor-specific devices
pub const DEVICE_TYPE_FILE_PATH: u32 = 12;     // File system paths
pub const DEVICE_TYPE_DISK: u32 = 13;          // Block storage
pub const DEVICE_TYPE_CDROM: u32 = 14;         // Optical drives
pub const DEVICE_TYPE_RAMDISK: u32 = 15;       // RAM disks
pub const DEVICE_TYPE_URI: u32 = 16;           // URI references
pub const DEVICE_TYPE_MESSAGING: u32 = 17;     // Generic messaging
pub const DEVICE_TYPE_HARDWARE: u32 = 18;      // Generic hardware
pub const DEVICE_TYPE_MEDIA: u32 = 19;         // Media devices
pub const DEVICE_TYPE_CONTROLLER: u32 = 20;    // Hardware controllers
pub const DEVICE_TYPE_BLUETOOTH: u32 = 21;     // Bluetooth devices
pub const DEVICE_TYPE_WIFI: u32 = 22;          // WiFi interfaces
pub const DEVICE_TYPE_SD: u32 = 23;            // SD card devices
pub const DEVICE_TYPE_UFS: u32 = 24;           // UFS storage
pub const DEVICE_TYPE_UART: u32 = 25;          // Serial ports
```

### Kernel Components

#### `kernel/src/drivers/system.rs`
- **`init()`**: Consumes hardware inventory from handoff structure
- **Device Registration**: Creates `Device` objects for each inventory entry and adds them to the driver manager
- **Logging**: Optionally prints device details based on `PRINT_HARDWARE_INVENTORY` config

## Data Flow

```
UEFI Boot Environment
    ↓ (locate_handle_buffer)
Bootloader Hardware Discovery
    ↓ (classify_device for each handle)
Device Classification & Storage
    ↓ (leak boxed slice to static)
Handoff Structure Population
    ↓ (exit boot services)
Kernel Device Registration
    ↓ (read from handoff)
Driver System Integration
```

## Memory Management

### Bootloader Memory Strategy
The bootloader uses a memory leak strategy to keep device data alive:

```rust
static mut INVENTORY_STORAGE: Option<Box<[HardwareDevice]>> = None;

// During collection:
let boxed_devices = devices.into_boxed_slice();
unsafe {
    INVENTORY_STORAGE = Some(boxed_devices);
}
```

This ensures the device array remains valid even after boot services are exited, allowing the kernel to safely access the data.

### Handoff Structure
The `HardwareDevice` structures are passed to the kernel via the handoff structure:

```rust
pub struct Handoff {
    // ... other fields ...
    pub hardware_device_count: u32,
    pub hardware_inventory_ptr: u64,
    pub hardware_inventory_size: u64,
}
```

## Device Path Analysis

### Node Iteration
Each device handle's path is analyzed by iterating through its nodes:

```rust
for node in device_path.node_iter() {
    match (node.device_type(), node.sub_type()) {
        (DeviceType::HARDWARE, DeviceSubType::HARDWARE_PCI) => {
            detected = DEVICE_TYPE_PCI;
            "hardware/pci"
        }
        // ... more mappings
    }
}
```

### Human-Readable Descriptions
When `DevicePathToText` protocol is available, the system generates human-readable device descriptions:

```rust
if let Ok(text) = to_text.convert_device_path_to_text(
    &device_path,
    DisplayOnly(false),
    AllowShortcuts(true),
) {
    // Use UEFI-provided text description
} else {
    // Fall back to node-based description
    summary = parts.join(" -> ");
}
```

## Configuration

### Bootloader Configuration
- **Verbose Mode**: Set via `collect_hardware_inventory(verbose: bool)` parameter
- **Device Limit**: Controlled by `hardware::MAX_HARDWARE_DEVICES` constant (1000 devices)

### Kernel Configuration
- **`PRINT_HARDWARE_INVENTORY`**: Controls whether device details are logged during registration
- **`MAP_LEGACY_PHYS_OFFSET_1GIB`**: Affects memory mapping strategy (currently set to `true`)

## Example Output

### Bootloader (Verbose Mode)
```
Collecting hardware inventory using device paths...
Found 22 device path handles
  Device 1: Handle 0x000000003F8E3518 -> unknown
  Device 2: Handle 0x000000003F4B5698 -> unknown  
  Device 3: Handle 0x000000003F048898 -> acpi/node
  Device 4: Handle 0x000000003ED30598 -> hardware/vendor
  Device 5: Handle 0x000000003ECFCF18 -> hardware/vendor
  ...
  Device 10: Handle 0x000000003EB15F18 -> hardware/pci
  Device 15: Handle 0x000000003EAB2818 -> messaging/sata
✓ Hardware inventory collected: 22 devices
```

### Kernel
```
[driver] registering UEFI hardware inventory
[driver] inventory entry index: 0x0000000000000000 type: unknown address: 0x000000003F8E3518
[driver] inventory entry index: 0x0000000000000002 type: acpi address: 0x000000003F048898
[driver] inventory entry index: 0x0000000000000003 type: vendor address: 0x000000003ED30598
[driver] inventory entry index: 0x0000000000000009 type: pci address: 0x000000003EB15F18
[driver] inventory entry index: 0x0000000000000014 type: sata address: 0x000000003EAB2818
```

## Integration with ACPI

The hardware inventory system works alongside ACPI initialization:

1. **ACPI First**: Platform topology is discovered via ACPI tables (MADT, etc.)
2. **UEFI Inventory Second**: Device enumeration provides additional device information
3. **Combined Registration**: Both systems contribute to the overall device registry

This dual approach ensures comprehensive hardware discovery while maintaining compatibility with both UEFI and ACPI standards.

## Future Enhancements

### Planned Improvements
1. **PCI Configuration Space**: Read PCI device/vendor IDs for more specific device identification
2. **ACPI HID Extraction**: Parse ACPI hardware IDs from device paths
3. **IRQ Mapping**: Associate devices with their interrupt assignments
4. **Device Capabilities**: Extract and store device-specific capabilities
5. **Hotplug Support**: Handle dynamic device addition/removal

### Extensibility
The system is designed to be easily extensible:
- New device types can be added to the `DEVICE_TYPE_*` constants
- Additional device path node types can be handled in `classify_device()`
- Custom device-specific metadata can be stored in future `HardwareDevice` extensions

## Troubleshooting

### Common Issues
1. **Memory Corruption**: If device addresses show `0xAFAFAF...`, the inventory storage was freed too early
2. **Missing Devices**: Check UEFI `locate_handle_buffer` success and device path protocol availability
3. **Classification Failures**: Verify UEFI `DevicePathToText` protocol is accessible

### Debug Configuration
- Enable verbose bootloader output for detailed device discovery logs
- Set `PRINT_HARDWARE_INVENTORY = true` for kernel device registration logs
- Use QEMU debug output (`0xe9` port) to monitor the entire process
