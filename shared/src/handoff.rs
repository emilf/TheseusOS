/// Handoff structure passed from bootloader to kernel.
///
/// This structure contains all the system information collected during UEFI boot
/// and is passed to the kernel during the handoff process. It includes hardware
/// inventory, memory maps, ACPI information, and other platform details.
///
/// # Layout
/// The structure is marked `#[repr(C)]` to ensure stable layout for
/// consumption from any language and to maintain ABI compatibility.
///
/// # Fields
/// - **Graphics**: Framebuffer information from UEFI GOP
/// - **Memory Map**: UEFI memory map and descriptor information
/// - **ACPI**: Root System Description Pointer (RSDP) address
/// - **UEFI**: System table and image handle for boot services exit
/// - **Firmware**: Vendor information and revision
/// - **Boot Time**: Timestamp and device path information
/// - **CPU**: Processor count, features, and microcode revision
/// - **Hardware Inventory**: Device enumeration results
/// - **Kernel Image**: Virtual memory layout information
/// - **Temporary Heap**: Pre-allocated heap for kernel initialization
/// - **Status**: Boot services exit status
#[repr(C)]
pub struct Handoff {
    /// Total size of this struct in bytes
    pub size: u32,
    /// Handoff format version
    pub handoff_version: u32,

    // Graphics Output Protocol (GOP) Information
    /// Framebuffer base address
    pub gop_fb_base: u64,
    /// Framebuffer size in bytes
    pub gop_fb_size: u64,
    /// Framebuffer width in pixels
    pub gop_width: u32,
    /// Framebuffer height in pixels
    pub gop_height: u32,
    /// Framebuffer stride
    pub gop_stride: u32,
    /// Pixel format enum
    pub gop_pixel_format: u32,

    // Memory Map Information
    /// Memory map buffer pointer (0 if not available)
    pub memory_map_buffer_ptr: u64,
    /// Memory map descriptor size in bytes
    pub memory_map_descriptor_size: u32,
    /// Memory map descriptor version
    pub memory_map_descriptor_version: u32,
    /// Number of memory map entries
    pub memory_map_entries: u32,
    /// Memory map buffer size in bytes
    pub memory_map_size: u32,

    // ACPI Information
    /// ACPI RSDP table address (0 if not found)
    pub acpi_rsdp: u64,

    // UEFI System Information
    /// UEFI System Table pointer (for exit_boot_services)
    pub uefi_system_table: u64,
    /// UEFI Image Handle (for exit_boot_services)
    pub uefi_image_handle: u64,

    // Firmware Information
    /// Firmware vendor string pointer (0 if not available)
    pub firmware_vendor_ptr: u64,
    /// Firmware vendor string length
    pub firmware_vendor_len: u32,
    /// Firmware revision
    pub firmware_revision: u32,

    // Boot Time Information
    /// Boot time (seconds since epoch)
    pub boot_time_seconds: u64,
    /// Boot time (nanoseconds)
    pub boot_time_nanoseconds: u32,
    /// Boot device path pointer (0 if not available)
    pub boot_device_path_ptr: u64,
    /// Boot device path size in bytes
    pub boot_device_path_size: u32,

    // CPU Information
    /// CPU count
    pub cpu_count: u32,
    /// CPU features flags
    pub cpu_features: u64,
    /// Microcode revision
    pub microcode_revision: u32,

    // Hardware Inventory Information
    /// Number of hardware devices found
    pub hardware_device_count: u32,
    /// Hardware inventory data pointer (0 if not available)
    pub hardware_inventory_ptr: u64,
    /// Hardware inventory data size in bytes
    pub hardware_inventory_size: u64,

    // Virtual Memory Information
    /// Kernel virtual base address (where kernel expects to be mapped)
    pub kernel_virtual_base: u64,
    /// Kernel physical load address (where kernel is actually loaded)
    pub kernel_physical_base: u64,
    /// Kernel virtual entry point address
    pub kernel_virtual_entry: u64,
    /// Total kernel image size (bytes), covering all loaded sections
    pub kernel_image_size: u64,

    // Temporary Heap Information
    /// Temporary heap base address (allocated by bootloader for kernel setup)
    ///
    /// This field contains the physical address of memory pre-allocated by the bootloader
    /// for use as the kernel's temporary heap during initialization. The kernel can use
    /// this memory immediately upon entry without needing to parse memory maps or set up
    /// complex memory management. Set to 0 if allocation failed.
    pub temp_heap_base: u64,
    /// Temporary heap size in bytes
    ///
    /// This field contains the size of the pre-allocated temporary heap in bytes.
    /// The kernel uses this to determine the bounds of its temporary heap region.
    /// Set to 0 if allocation failed.
    pub temp_heap_size: u64,

    /// Boot services exit status
    ///
    /// This field indicates whether UEFI boot services have been exited.
    /// 0 = boot services still active, 1 = boot services exited.
    pub boot_services_exited: u32,
}

/// Hardware device information structure.
///
/// This structure represents a single hardware device discovered during UEFI boot.
/// It contains the device type, optional address information, and optional IRQ
/// assignment. The structure is used in the hardware inventory passed from
/// bootloader to kernel.
///
/// # Layout
/// The structure is marked `#[repr(C)]` to ensure stable layout for
/// handoff between bootloader and kernel.
///
/// # Fields
/// - `device_type`: Standardized device type constant (see `DEVICE_TYPE_*` constants)
/// - `address`: Optional device address (typically UEFI handle address)
/// - `irq`: Optional interrupt request number (currently unused)
///
/// # Usage
/// Devices are collected during bootloader execution and passed to the kernel
/// via the handoff structure. The kernel uses this information to register
/// devices with the driver system.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct HardwareDevice {
    /// Standardized device type constant (see DEVICE_TYPE_* constants)
    pub device_type: u32,
    /// Optional device address (typically UEFI handle address)
    pub address: Option<u64>,
    /// Optional interrupt request number (currently unused)
    pub irq: Option<u32>,
}

/// Device type constants for hardware inventory classification.
///
/// These constants define standardized device types used throughout the
/// hardware inventory system. They map UEFI device path node types to
/// simplified categories that the kernel can easily understand and process.
///
/// # Device Categories
/// - **System**: CPU cores, IO APICs, ACPI devices
/// - **Storage**: PCI, USB, SATA, NVMe, disks, CD-ROMs, RAM disks
/// - **Network**: MAC addresses, IPv4/IPv6 interfaces, WiFi, Bluetooth
/// - **Media**: File paths, URIs, generic media devices
/// - **Communication**: UART, messaging protocols
/// - **Vendor**: Vendor-specific hardware
/// - **Unknown**: Unclassified or unrecognized devices
///
/// # Usage
/// These constants are used by the bootloader to classify discovered devices
/// and by the kernel to understand device types during driver registration.
pub const DEVICE_TYPE_UNKNOWN: u32 = 0;
/// Unclassified device
pub const DEVICE_TYPE_CPU: u32 = 1;
/// CPU cores
pub const DEVICE_TYPE_IO_APIC: u32 = 2;
/// IO APIC controllers
pub const DEVICE_TYPE_PCI: u32 = 3;
/// PCI devices
pub const DEVICE_TYPE_USB: u32 = 4;
/// USB devices
pub const DEVICE_TYPE_SATA: u32 = 5;
/// SATA/SCSI devices
pub const DEVICE_TYPE_NVME: u32 = 6;
/// NVMe devices
pub const DEVICE_TYPE_MAC: u32 = 7;
/// Network interfaces
pub const DEVICE_TYPE_IPV4: u32 = 8;
/// IPv4 network
pub const DEVICE_TYPE_IPV6: u32 = 9;
/// IPv6 network
pub const DEVICE_TYPE_ACPI: u32 = 10;
/// ACPI devices
pub const DEVICE_TYPE_VENDOR: u32 = 11;
/// Vendor-specific devices
pub const DEVICE_TYPE_FILE_PATH: u32 = 12;
/// File system paths
pub const DEVICE_TYPE_DISK: u32 = 13;
/// Block storage
pub const DEVICE_TYPE_CDROM: u32 = 14;
/// Optical drives
pub const DEVICE_TYPE_RAMDISK: u32 = 15;
/// RAM disks
pub const DEVICE_TYPE_URI: u32 = 16;
/// URI references
pub const DEVICE_TYPE_MESSAGING: u32 = 17;
/// Generic messaging
pub const DEVICE_TYPE_HARDWARE: u32 = 18;
/// Generic hardware
pub const DEVICE_TYPE_MEDIA: u32 = 19;
/// Media devices
pub const DEVICE_TYPE_CONTROLLER: u32 = 20;
/// Hardware controllers
pub const DEVICE_TYPE_BLUETOOTH: u32 = 21;
/// Bluetooth devices
pub const DEVICE_TYPE_WIFI: u32 = 22;
/// WiFi interfaces
pub const DEVICE_TYPE_SD: u32 = 23;
/// SD card devices
pub const DEVICE_TYPE_UFS: u32 = 24;
/// UFS storage
pub const DEVICE_TYPE_UART: u32 = 25;
/// Serial ports

impl HardwareDevice {
    /// Create a `HardwareDevice` from a byte buffer at the specified index.
    ///
    /// This function is used by the kernel to deserialize hardware device
    /// information from the handoff structure's inventory buffer.
    ///
    /// # Arguments
    /// * `buffer` - The byte buffer containing serialized hardware devices
    /// * `index` - The zero-based index of the device to deserialize
    ///
    /// # Returns
    /// * `Some(HardwareDevice)` - Successfully deserialized device
    /// * `None` - Index out of bounds or buffer too small
    ///
    /// # Safety
    /// This function assumes the buffer contains valid `HardwareDevice` structures
    /// laid out sequentially. The caller must ensure the buffer is properly
    /// aligned and contains valid data.
    pub fn from_bytes(buffer: &[u8], index: usize) -> Option<Self> {
        let entry_size = core::mem::size_of::<Self>();
        let start = index.checked_mul(entry_size)?;
        let end = start.checked_add(entry_size)?;
        if end > buffer.len() {
            return None;
        }
        let ptr = buffer[start..end].as_ptr() as *const Self;
        unsafe { Some(core::ptr::read_unaligned(ptr)) }
    }

    /// Convert device type constant to human-readable string.
    ///
    /// This function provides a string representation of the device type
    /// constant for logging and debugging purposes.
    ///
    /// # Returns
    /// A string slice containing the human-readable device type name.
    /// Unknown device types return "unknown".
    ///
    /// # Example
    /// ```rust
    /// let device = HardwareDevice {
    ///     device_type: DEVICE_TYPE_PCI,
    ///     address: Some(0x1234),
    ///     irq: None,
    /// };
    /// assert_eq!(device.device_type_str(), "pci");
    /// ```
    pub fn device_type_str(&self) -> &'static str {
        match self.device_type {
            DEVICE_TYPE_UNKNOWN => "unknown",
            DEVICE_TYPE_CPU => "cpu",
            DEVICE_TYPE_IO_APIC => "ioapic",
            DEVICE_TYPE_PCI => "pci",
            DEVICE_TYPE_USB => "usb",
            DEVICE_TYPE_SATA => "sata",
            DEVICE_TYPE_NVME => "nvme",
            DEVICE_TYPE_MAC => "mac",
            DEVICE_TYPE_IPV4 => "ipv4",
            DEVICE_TYPE_IPV6 => "ipv6",
            DEVICE_TYPE_ACPI => "acpi",
            DEVICE_TYPE_VENDOR => "vendor",
            DEVICE_TYPE_FILE_PATH => "file",
            DEVICE_TYPE_DISK => "disk",
            DEVICE_TYPE_CDROM => "cdrom",
            DEVICE_TYPE_RAMDISK => "ramdisk",
            DEVICE_TYPE_URI => "uri",
            DEVICE_TYPE_MESSAGING => "messaging",
            DEVICE_TYPE_HARDWARE => "hardware",
            DEVICE_TYPE_MEDIA => "media",
            DEVICE_TYPE_CONTROLLER => "controller",
            DEVICE_TYPE_BLUETOOTH => "bluetooth",
            DEVICE_TYPE_WIFI => "wifi",
            DEVICE_TYPE_SD => "sd",
            DEVICE_TYPE_UFS => "ufs",
            DEVICE_TYPE_UART => "uart",
            _ => "unknown",
        }
    }
}

/// Static storage for handoff data
///
/// # Safety
///
/// This global mutable static is safe to use in the UEFI bootloader context because:
/// - It is only written to during the single-threaded UEFI boot process
/// - No other threads or concurrent access occurs during bootloader execution
/// - Once boot services are exited and control is passed to the kernel, this structure
///   is no longer modified by the bootloader
/// - The kernel receives a copy of this data and should not modify the original
///
/// The lack of synchronization primitives is intentional and safe for this use case.
pub static mut HANDOFF: Handoff = Handoff {
    size: 0,
    handoff_version: 1,

    // GOP fields
    gop_fb_base: 0,
    gop_fb_size: 0,
    gop_width: 0,
    gop_height: 0,
    gop_stride: 0,
    gop_pixel_format: 0,

    // Memory map fields
    memory_map_buffer_ptr: 0,
    memory_map_descriptor_size: 0,
    memory_map_descriptor_version: 0,
    memory_map_entries: 0,
    memory_map_size: 0,

    // ACPI fields
    acpi_rsdp: 0,

    // UEFI System Table
    uefi_system_table: 0,
    uefi_image_handle: 0,

    // Firmware fields
    firmware_vendor_ptr: 0,
    firmware_vendor_len: 0,
    firmware_revision: 0,

    // Boot time fields
    boot_time_seconds: 0,
    boot_time_nanoseconds: 0,
    boot_device_path_ptr: 0,
    boot_device_path_size: 0,

    // CPU fields
    cpu_count: 0,
    cpu_features: 0,
    microcode_revision: 0,

    // Hardware inventory fields
    hardware_device_count: 0,
    hardware_inventory_ptr: 0,
    hardware_inventory_size: 0,

    // Virtual memory fields
    kernel_virtual_base: 0,
    kernel_physical_base: 0,
    kernel_virtual_entry: 0,
    kernel_image_size: 0,

    // Temporary heap fields
    temp_heap_base: 0,
    temp_heap_size: 0,

    // Boot services status
    boot_services_exited: 0,
};

impl Handoff {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.size < core::mem::size_of::<Handoff>() as u32 {
            return Err("Handoff size is smaller than expected");
        }
        // Add other validation checks here if needed
        Ok(())
    }
}
