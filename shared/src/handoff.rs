/// Handoff structure passed to the kernel
/// Layout is stable (repr C) to allow consumption from any language
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
    
    // Device Tree Information
    /// Device tree blob address (0 if not found)
    pub device_tree_ptr: u64,
    /// Device tree size in bytes
    pub device_tree_size: u64,
    
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
    /// Page table root address (CR3 value for kernel)
    pub page_table_root: u64,
    /// Whether virtual memory is enabled (1) or identity mapped (0)
    pub virtual_memory_enabled: u32,
    /// Reserved for future virtual memory extensions
    pub _reserved_virtual: u32,
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
    
    // Device tree fields
    device_tree_ptr: 0,
    device_tree_size: 0,
    
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
    page_table_root: 0,
    virtual_memory_enabled: 0,
    _reserved_virtual: 0,
};
