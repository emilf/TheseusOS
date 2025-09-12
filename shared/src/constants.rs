//! Constants and magic numbers used throughout the UEFI loader
//! 
//! This module centralizes all magic numbers, I/O port addresses, and other
//! constants to improve code readability and maintainability.

/// I/O Port Addresses
pub mod io_ports {
    /// QEMU debug output port - writes directly to QEMU's debug console
    pub const QEMU_DEBUG: u16 = 0xe9;
    
    /// QEMU auto-exit port - causes QEMU to exit with specified exit code
    pub const QEMU_EXIT: u16 = 0xf4;
    
    /// COM1 Serial Port I/O addresses
    pub mod com1 {
        /// COM1 data register
        pub const DATA: u16 = 0x3f8;
        /// COM1 interrupt enable register
        pub const INT_ENABLE: u16 = 0x3f9;
        /// COM1 line control register
        pub const LINE_CTRL: u16 = 0x3fb;
        /// COM1 modem control register
        pub const MODEM_CTRL: u16 = 0x3fc;
        /// COM1 line status register
        pub const LINE_STATUS: u16 = 0x3fd;
    }
}

/// Memory and Buffer Constants
pub mod memory {
    /// Default memory alignment for allocations
    #[allow(dead_code)]
    pub const ALIGNMENT: usize = 8;
    
    /// Maximum size for firmware vendor string
    #[allow(dead_code)]
    pub const MAX_FIRMWARE_VENDOR_LEN: usize = 256;
    
    /// Maximum size for device path strings
    #[allow(dead_code)]
    pub const MAX_DEVICE_PATH_LEN: usize = 1024;
    
    /// UEFI page size (4KB)
    pub const UEFI_PAGE_SIZE: u64 = 4096;
    
    /// Default kernel size estimate (1MB)
    pub const DEFAULT_KERNEL_SIZE: u64 = 1024 * 1024;
    
    /// UEFI memory descriptor size (typically 24 bytes in UEFI)
    pub const UEFI_MEMORY_DESCRIPTOR_SIZE: usize = 24;
    
    /// Bytes per megabyte (for display formatting)
    pub const BYTES_PER_MB: f64 = 1024.0 * 1024.0;
}

/// Kernel Loading Constants
pub mod kernel {
    /// Default kernel binary filename in EFI partition
    pub const KERNEL_FILENAME: &str = "kernel.efi";
    
    /// Default kernel binary path in EFI partition
    pub const KERNEL_PATH: &str = "\\kernel.efi";
    
    /// Maximum kernel size we're willing to load (16MB)
    pub const MAX_KERNEL_SIZE: u64 = 16 * 1024 * 1024;
    
    /// ELF header magic number
    pub const ELF_MAGIC: [u8; 4] = [0x7f, 0x45, 0x4c, 0x46]; // "\x7fELF"
}

/// Hardware Constants
pub mod hardware {
    /// Maximum number of CPU cores we can handle
    #[allow(dead_code)]
    pub const MAX_CPU_COUNT: u32 = 256;
    
    /// Maximum number of hardware devices to enumerate (prevents hangs)
    pub const MAX_HARDWARE_DEVICES: usize = 1000;
    
    /// COM1 baud rate divisor for 115200 baud
    pub const COM1_BAUD_DIVISOR: u16 = 1;
}

/// UEFI Constants
pub mod uefi {
    /// Handoff structure version
    #[allow(dead_code)]
    pub const HANDOFF_VERSION: u32 = 1;
    
    /// ACPI RSDP signature
    #[allow(dead_code)]
    pub const ACPI_RSDP_SIGNATURE: &[u8; 8] = b"RSD PTR ";
    
    /// Device Tree signature
    #[allow(dead_code)]
    pub const DTB_SIGNATURE: u32 = 0xd00dfeed;
}

/// Exit Codes
pub mod exit_codes {
    /// QEMU exit code for successful completion
    pub const QEMU_SUCCESS: u8 = 0;
    
    /// QEMU exit code for error
    #[allow(dead_code)]
    pub const QEMU_ERROR: u8 = 1;
}
