# Driver Systems Deep Dive: DMA, PCI Discovery, and Driver Framework

This document provides an exhaustive technical deep-dive into TheseusOS's driver systems, covering the DMA subsystem, PCI device discovery process, and the driver framework architecture. This is intended for kernel developers who need to understand, maintain, or extend these critical subsystems.

## Table of Contents

1. [Driver Framework Architecture](#driver-framework-architecture)
2. [DMA Subsystem](#dma-subsystem)
3. [PCI Discovery Process](#pci-discovery-process)
4. [System Integration](#system-integration)
5. [Implementation Details](#implementation-details)
6. [Troubleshooting Guide](#troubleshooting-guide)

---

## Driver Framework Architecture

### Overview

The TheseusOS driver framework is a unified system that manages device discovery, driver binding, and device operation. It provides a clean abstraction layer between hardware devices and kernel services while maintaining type safety and performance.

### Core Components

#### 1. Driver Trait (`traits.rs`)

The `Driver` trait is the fundamental interface that all device drivers must implement:

```rust
pub trait Driver: Sync + Send {
    fn supported_classes(&self) -> &'static [DeviceClass];
    fn supports_class(&self, class: DeviceClass) -> bool;
    fn on_register(&'static self) -> Result<bool, &'static str>;
    fn init(&'static self, dev: &mut Device) -> Result<(), &'static str>;
    fn probe(&'static self, dev: &mut Device) -> Result<(), &'static str>;
    fn remove(&'static self, dev: &mut Device);
    fn irq_handler(&'static self, dev: &mut Device, irq: u32) -> bool;
    fn write(&'static self, dev: &mut Device, buf: &[u8]) -> Result<usize, &'static str>;
    fn read(&'static self, dev: &mut Device, buf: &mut [u8]) -> Result<usize, &'static str>;
}
```

**Key Design Principles:**
- **Static Lifetime**: All drivers must have `'static` lifetime to ensure they live for the entire kernel execution
- **Thread Safety**: `Sync + Send` bounds ensure drivers can be safely shared across threads
- **Error Handling**: Consistent `Result<(), &'static str>` error reporting
- **Optional Operations**: Most methods have default implementations for unsupported operations

#### 2. Device Descriptor (`traits.rs`)

The `Device` struct represents a hardware device in the system:

```rust
pub struct Device {
    pub id: DeviceId,                    // Unique device identifier
    pub class: DeviceClass,              // Device category
    pub phys_addr: Option<u64>,          // Physical MMIO address
    pub irq: Option<u32>,                // Interrupt request number
    pub resources: Vec<DeviceResource>,  // Memory/I/O resources
    pub driver_data: Option<usize>,      // Driver-specific state
}
```

**Device Identification:**
- **ACPI**: Hardware ID from ACPI tables (e.g., "PNP0A08")
- **PCI**: Segment/Bus/Device/Function tuple
- **Class**: Generic device class (Serial, Storage, etc.)
- **Raw**: Platform-specific identifier

**Resource Management:**
- **Memory Resources**: MMIO regions with prefetchable flags
- **I/O Resources**: Port-based I/O regions
- **Interrupt Resources**: IRQ assignments and MSI capabilities

#### 3. Driver Manager (`manager.rs`)

The `DriverManager` is the central coordinator that maintains registries of drivers and devices:

```rust
pub struct DriverManager {
    drivers: Vec<&'static dyn Driver>,  // Registered drivers
    devices: Vec<Device>,               // Discovered devices
}
```

**Core Responsibilities:**
1. **Driver Registry**: Maintains list of all registered drivers
2. **Device Registry**: Tracks all discovered devices
3. **Binding Logic**: Matches devices to compatible drivers
4. **IRQ Dispatch**: Routes interrupts to appropriate drivers
5. **Class-based I/O**: Provides generic read/write operations

### Driver Lifecycle

#### Phase 1: Registration
```rust
// Driver registers with the system
driver_manager().lock().register_driver(&MY_DRIVER);

// Driver's on_register() method is called
// Returns Ok(true) for immediate probing, Ok(false) for deferred probing
```

#### Phase 2: Device Discovery
```rust
// Hardware discovery adds devices
let device = Device::new(DeviceId::Pci { ... });
driver_manager().lock().add_device(device);

// Probe logic runs automatically
```

#### Phase 3: Binding Algorithm
```rust
fn probe_pending_devices(&mut self) {
    for dev in self.devices.iter_mut() {
        if dev.driver_data.is_some() { continue; } // Already bound
        
        for drv in self.drivers.iter() {
            if !drv.supports_class(dev.class) { continue; }
            
            match drv.probe(dev) {
                Ok(()) => {
                    drv.init(dev)?;  // Initialize device
                    break;           // First-match wins
                }
                Err(_) => continue;  // Try next driver
            }
        }
    }
}
```

#### Phase 4: Operation
```rust
// Interrupt handling
if driver_manager().lock().handle_irq(irq) {
    // Interrupt was handled
}

// I/O operations
driver_manager().lock().write_class(DeviceClass::Serial, b"Hello");
```

### Thread Safety Model

The driver framework uses a global spin mutex (`DRIVER_MANAGER`) to ensure thread safety:

```rust
static DRIVER_MANAGER: Mutex<DriverManager> = Mutex::new(DriverManager::new());

pub fn driver_manager() -> &'static Mutex<DriverManager> {
    &DRIVER_MANAGER
}
```

**Safety Guarantees:**
- Only one thread can modify registries at a time
- IRQ handlers can safely call into the manager
- No race conditions between driver registration and device enumeration
- All operations are atomic within the lock

---

## DMA Subsystem

### Overview

The DMA (Direct Memory Access) subsystem provides memory management for device operations that require physically contiguous, cache-coherent memory regions. It's built on top of the persistent physical allocator and handles both individual buffers and pooled allocations.

### Core Components

#### 1. DMA Buffer (`dma.rs`)

The `DmaBuffer` represents a single contiguous memory region suitable for DMA operations:

```rust
pub struct DmaBuffer {
    phys_addr: u64,        // Physical address (for devices)
    virt_addr: u64,        // Virtual address (for kernel)
    size: usize,           // Buffer size in bytes
    mapped_size: usize,    // Mapped region size (page-aligned)
    cache_policy: CachePolicy, // Cache behavior
}
```

**Key Features:**
- **Dual Addressing**: Maintains both physical and virtual addresses
- **Automatic Cleanup**: Properly unmaps memory when dropped
- **Cache Control**: Configurable cache behavior for different use cases
- **Zero Initialization**: Buffers are automatically zeroed on allocation

#### 2. Cache Policies

The `CachePolicy` enum controls how the CPU and devices interact with memory:

```rust
pub enum CachePolicy {
    WriteBack,      // Normal cached memory (default)
    WriteCombining, // Optimized for bulk writes
    Uncached,       // Bypasses CPU cache entirely
}
```

**Policy Mappings:**
- **WriteBack**: Normal cached memory, good for descriptor rings
- **WriteCombining**: Optimized for frame buffers and bulk transfers
- **Uncached**: Required for MMIO bounce buffers and device registers

**Page Table Flags:**
```rust
impl CachePolicy {
    pub fn page_flags(self, base: u64) -> u64 {
        match self {
            CachePolicy::WriteBack => base,
            CachePolicy::WriteCombining => base | PTE_PWT,
            CachePolicy::Uncached => base | PTE_PCD | PTE_PWT,
        }
    }
}
```

#### 3. DMA Pool (`dma_pool.rs`)

The `DmaPool` provides efficient allocation of fixed-size DMA blocks:

```rust
pub struct DmaPool {
    block_size: usize,                    // Size of each block
    cache_policy: CachePolicy,            // Cache behavior
    blocks_per_slab: usize,              // Blocks per allocation
    slabs: Vec<DmaBuffer>,               // Allocated slabs
    free_list: VecDeque<(usize, usize)>, // (slab_index, block_index)
}
```

**Design Philosophy:**
- **Slab-based**: Allocates large contiguous slabs and subdivides them
- **Fixed-size blocks**: All blocks in a pool are the same size
- **Fast allocation**: O(1) allocation from a free list
- **Automatic cleanup**: Blocks are automatically returned when dropped

### Memory Mapping Process

#### 1. Physical Allocation
```rust
// Allocate physically contiguous memory
let phys = physical_memory::alloc_contiguous(size as u64, alignment as u64)?;
```

#### 2. Virtual Mapping
```rust
fn map_dma_region(phys_addr: u64, size: usize, policy: CachePolicy) -> (u64, usize) {
    let page_size = PAGE_SIZE as u64;
    
    // Align physical address to page boundary
    let phys_base = phys_addr & !(page_size - 1);
    let offset = phys_addr - phys_base;
    
    // Calculate size needed for mapping (including alignment padding)
    let size_aligned = ((offset + size as u64 + page_size - 1) / page_size) * page_size;
    
    // Convert physical base to virtual address
    let virt_base = phys_to_virt_pa(phys_base);

    // Map the region into the current page table
    unsafe {
        let pml4_pa = current_pml4_phys();
        let pml4_ptr = phys_to_virt_pa(pml4_pa) as *mut PageTable;
        let pml4 = &mut *pml4_ptr;
        let mut allocator = PersistentFrameAllocator;
        
        // Apply cache policy to page table flags
        let flags = policy.page_flags(PTE_PRESENT | PTE_WRITABLE);
        
        map_range_with_policy(
            pml4, virt_base, phys_base, size_aligned, flags, &mut allocator
        );
    }

    (virt_base + offset, size_aligned as usize)
}
```

#### 3. Cache Policy Application

The cache policy is applied through x86-64 page table flags:
- **PWT (Page Write Through)**: Forces write-through behavior
- **PCD (Page Cache Disable)**: Disables CPU caching entirely

### Pool Allocation Algorithm

#### 1. Block Allocation
```rust
pub fn allocate(&mut self) -> Result<DmaPoolBlock<'_>, &'static str> {
    // If no free blocks available, allocate a new slab
    if self.free_list.is_empty() {
        let total = self.block_size * self.blocks_per_slab;
        let slab = DmaBuffer::allocate_with_policy(
            total, self.block_size, self.cache_policy
        ).map_err(|_| "dma slab allocation failed")?;
        
        let slab_index = self.slabs.len();
        self.slabs.push(slab);
        
        // Add all blocks from the new slab to the free list
        for block_index in 0..self.blocks_per_slab {
            self.free_list.push_back((slab_index, block_index));
        }
    }

    // Remove a block from the free list
    let (slab_idx, block_idx) = self.free_list.pop_front().unwrap();
    let phys_base = self.slabs[slab_idx].phys_addr();
    
    Ok(DmaPoolBlock {
        pool: self,
        slab_idx, block_idx, phys_base,
        offset: block_idx * self.block_size,
        size: self.block_size,
        _marker: PhantomData,
    })
}
```

#### 2. Block Return
```rust
impl<'pool> Drop for DmaPoolBlock<'pool> {
    fn drop(&mut self) {
        unsafe {
            (*self.pool).release(self.slab_idx, self.block_idx);
        }
    }
}
```

### Use Cases

#### 1. Network Packet Buffers
```rust
let mut pool = DmaPool::new(1514, CachePolicy::WriteBack); // Ethernet MTU
let packet = pool.allocate()?;
// Use packet.phys_addr() for DMA operations
```

#### 2. Storage I/O Buffers
```rust
let mut pool = DmaPool::new(4096, CachePolicy::WriteBack); // 4KB blocks
let buffer = pool.allocate()?;
// Use for disk I/O operations
```

#### 3. Device Descriptor Rings
```rust
let mut pool = DmaPool::new(64, CachePolicy::WriteBack); // Descriptor size
let descriptor = pool.allocate()?;
// Use for device command rings
```

---

## PCI Discovery Process

### Overview

The PCI discovery process is responsible for enumerating all PCI/PCIe devices in the system using the Enhanced Configuration Access Mechanism (ECAM). It handles complex scenarios like PCI bridges, multi-function devices, and resource allocation.

### Architecture Overview

```
┌─────────────────────────────────────────────┐
│              ACPI MCFG Table                │
│  ┌─────────────────────────────────────────┐ │
│  │         ECAM Regions                    │ │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐   │ │
│  │  │Region 0 │ │Region 1 │ │Region N │   │ │
│  │  └─────────┘ └─────────┘ └─────────┘   │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────┐
│            PCI Enumeration                  │
│  ┌─────────────────────────────────────────┐ │
│  │         Bus Scanning                    │ │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐   │ │
│  │  │ Bus 0   │ │ Bus 1   │ │ Bus N   │   │ │
│  │  └─────────┘ └─────────┘ └─────────┘   │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────┐
│          Device Discovery                   │
│  ┌─────────────────────────────────────────┐ │
│  │    Function Scanning                    │ │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐   │ │
│  │  │Func 0   │ │Func 1   │ │Func 7   │   │ │
│  │  └─────────┘ └─────────┘ └─────────┘   │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### ECAM Addressing

PCIe uses a flat address space where each function has a 4KB configuration space:

```
Address = ECAM_base + (bus * 1MB) + (device * 32KB) + (function * 4KB)
```

**Addressing Constants:**
- **Bus stride**: 1MB (0x1_0000)
- **Device stride**: 32KB (0x8000)  
- **Function stride**: 4KB (0x1000)

### Discovery Algorithm

#### 1. Main Enumeration Function
```rust
pub fn enumerate(regions: &[PciConfigRegion]) -> PciTopology {
    let mut devices = Vec::new();
    let mut bridges = Vec::new();
    let mut visited_buses: BTreeSet<(u16, u8)> = BTreeSet::new();

    // Scan each ECAM region
    for region in regions {
        let mut next_bus = region.bus_start.saturating_add(1);
        scan_bus(
            region, region.bus_start, &mut visited_buses,
            &mut devices, &mut bridges, &mut next_bus
        );
    }

    PciTopology { functions: devices, bridges }
}
```

#### 2. Bus Scanning Algorithm
```rust
fn scan_bus(
    region: &PciConfigRegion,
    bus: u8,
    visited_buses: &mut BTreeSet<(u16, u8)>,
    devices: &mut Vec<PciDeviceInfo>,
    bridges: &mut Vec<PciBridgeInfo>,
    next_bus: &mut u8,
) -> u8 {
    // Check if bus is within region and not already visited
    if bus < region.bus_start || bus > region.bus_end { return bus; }
    if !visited_buses.insert((region.segment, bus)) { return bus; }

    let mut max_bus = bus;

    // Scan all devices on this bus
    for device in 0..=MAX_DEVICE {
        if let Some(info) = read_function(region, bus, device, 0) {
            let is_bridge = info.header_type & 0x7F == 0x01;
            
            if is_bridge {
                // Handle PCI bridge
                max_bus = configure_bridge(region, bus, device, info, 
                                         visited_buses, devices, bridges, next_bus);
            } else {
                // Handle regular device
                devices.push(info);
            }

            // Check for multi-function devices
            if info.is_multi_function() {
                for function in 1..=MAX_FUNCTION {
                    if let Some(extra) = read_function(region, bus, device, function) {
                        devices.push(extra);
                    }
                }
            }
        }
    }

    max_bus
}
```

#### 3. Function Reading
```rust
fn read_function(
    region: &PciConfigRegion,
    bus: u8, device: u8, function: u8,
) -> Option<PciDeviceInfo> {
    let base = function_base(region, bus, device, function)?;
    
    // Read vendor ID to check if function exists
    let vendor_id = config_read_u16(base, 0x00);
    if vendor_id == 0xFFFF { return None; }

    // Read all configuration space registers
    let device_id = config_read_u16(base, 0x02);
    let command = config_read_u16(base, 0x04);
    let status = config_read_u16(base, 0x06);
    // ... read all other registers

    // Parse BARs
    let bars = if (header_type & 0x7F) == 0x00 {
        read_type0_bars(base)
    } else {
        [PciBar::None; 6]
    };

    // Parse capabilities
    let capabilities = parse_capabilities(base, header_type, status);

    Some(PciDeviceInfo {
        segment: region.segment, bus, device, function,
        vendor_id, device_id, class_code, subclass, prog_if,
        revision_id, header_type, command, status,
        bars, interrupt_line, interrupt_pin, capabilities,
    })
}
```

### BAR Parsing

Base Address Registers (BARs) describe a device's memory and I/O resource requirements:

#### 1. BAR Types
- **Memory32**: 32-bit memory space BAR (up to 4GB)
- **Memory64**: 64-bit memory space BAR (up to 16EB, uses two consecutive BAR slots)
- **Io**: I/O space BAR (up to 64KB)
- **None**: Unused or invalid BAR

#### 2. BAR Size Detection
```rust
fn read_type0_bars(function_base: u64) -> [PciBar; 6] {
    let mut bars = [PciBar::None; 6];
    let original_command = config_read_u16(function_base, 0x04);
    
    // Disable memory and I/O decoding
    let decode_mask = 0x0007; // IO space | memory space | bus master
    let had_decode_enabled = (original_command & decode_mask) != 0;
    if had_decode_enabled {
        config_write_u16(function_base, 0x04, original_command & !decode_mask);
    }

    let mut index = 0usize;
    while index < 6 {
        let offset = 0x10 + (index as u8) * 4;
        let raw = config_read_u32(function_base, offset);
        
        if raw == 0 {
            bars[index] = PciBar::None;
            index += 1;
            continue;
        }

        if raw & 0x1 == 0x1 {
            // I/O BAR
            let base = (raw & 0xFFFF_FFFC) as u64;
            config_write_u32(function_base, offset, 0xFFFF_FFFC);
            let mask = config_read_u32(function_base, offset) & 0xFFFF_FFFC;
            config_write_u32(function_base, offset, raw);
            
            let size = if mask == 0 || mask == 0xFFFF_FFFC {
                0
            } else {
                (!mask).wrapping_add(1) as u64 & 0xFFFF_FFFC
            };
            
            bars[index] = if size == 0 {
                PciBar::None
            } else {
                PciBar::Io { base, size }
            };
            index += 1;
        } else {
            // Memory BAR
            let prefetchable = (raw & (1 << 3)) != 0;
            let bar_type = (raw >> 1) & 0x3;
            
            match bar_type {
                0x0 => {
                    // 32-bit memory BAR
                    let size = calculate_memory_bar_size(function_base, offset, raw);
                    let base = (raw & 0xFFFF_FFF0) as u64;
                    bars[index] = if size == 0 {
                        PciBar::None
                    } else {
                        PciBar::Memory32 { base, size, prefetchable }
                    };
                    index += 1;
                }
                0x2 => {
                    // 64-bit memory BAR
                    if index + 1 >= 6 { break; }
                    let raw_high = config_read_u32(function_base, offset + 4);
                    let size = calculate_64bit_memory_bar_size(function_base, offset, raw, raw_high);
                    let base_low = (raw & 0xFFFF_FFF0) as u64;
                    let base_high = (raw_high as u64) << 32;
                    let base = base_high | base_low;
                    
                    if size == 0 {
                        bars[index] = PciBar::None;
                        bars[index + 1] = PciBar::None;
                    } else {
                        bars[index] = PciBar::Memory64 { base, size, prefetchable };
                        bars[index + 1] = PciBar::None;
                    }
                    index += 2;
                }
                _ => {
                    bars[index] = PciBar::None;
                    index += 1;
                }
            }
        }
    }

    // Restore original command register
    if had_decode_enabled {
        config_write_u16(function_base, 0x04, original_command);
    }

    bars
}
```

### Bridge Configuration

PCI bridges require special handling to set up secondary and subordinate bus numbers:

#### 1. Bridge Detection
```rust
let is_bridge = info.header_type & 0x7F == 0x01;
```

#### 2. Bridge Configuration
```rust
fn configure_bridge(
    region: &PciConfigRegion,
    bus: u8, device: u8, info: &PciDeviceInfo,
    visited_buses: &mut BTreeSet<(u16, u8)>,
    devices: &mut Vec<PciDeviceInfo>,
    bridges: &mut Vec<PciBridgeInfo>,
    next_bus: &mut u8,
) -> u8 {
    if let Some(base) = function_base(region, bus, device, 0) {
        // Set up primary bus number
        if config_read_u8(base, 0x18) != bus {
            config_write_u8(base, 0x18, bus);
        }
        
        let mut secondary = config_read_u8(base, 0x19);
        let mut subordinate = config_read_u8(base, 0x1A);
        let command = config_read_u16(base, 0x04);

        // Enable I/O, Memory, and Bus Master
        let desired_command = command | 0x0007;
        if desired_command != command {
            config_write_u16(base, 0x04, desired_command);
        }

        // Allocate secondary bus if needed
        if secondary == 0 || secondary <= bus || 
           visited_buses.contains(&(region.segment, secondary)) {
            if let Some(new_bus) = allocate_bus(region, next_bus, visited_buses) {
                secondary = new_bus;
                subordinate = region.bus_end;
                config_write_u8(base, 0x19, secondary);
                config_write_u8(base, 0x1A, subordinate);
            }
        }

        // Perform secondary bus reset
        let bridge_control = config_read_u16(base, 0x3E);
        config_write_u16(base, 0x3E, bridge_control | 0x0400);  // Set reset bit
        config_write_u16(base, 0x3E, bridge_control & !0x0400); // Clear reset bit

        // Recursively scan downstream buses
        let child_max = scan_bus(region, secondary, visited_buses, 
                               devices, bridges, next_bus);
        
        // Update subordinate bus number
        if child_max > subordinate {
            subordinate = child_max;
            config_write_u8(base, 0x1A, subordinate);
        }

        // Record bridge information
        bridges.push(PciBridgeInfo {
            segment: info.segment,
            bus: info.bus, device: info.device, function: info.function,
            secondary_bus: secondary,
            subordinate_bus: subordinate,
            vendor_id: info.vendor_id,
            device_id: info.device_id,
            max_child_bus: child_max,
            io_window: None,
            mem_window: None,
            pref_mem_window: None,
        });

        child_max
    } else {
        bus
    }
}
```

### Capability Parsing

PCI devices can advertise various capabilities through a linked list structure:

#### 1. Capability Discovery
```rust
fn parse_capabilities(function_base: u64, header_type: u8, status: u16) -> PciCapabilities {
    // Check if capabilities pointer is present
    if (status & (1 << 4)) == 0 {
        return PciCapabilities::default();
    }

    // Get capability pointer based on header type
    let cap_ptr = match header_type & 0x7F {
        0x00 | 0x01 => config_read_u8(function_base, 0x34), // Standard/PCI-to-PCI bridge
        _ => 0,
    };

    if cap_ptr < 0x40 { return PciCapabilities::default(); }

    let mut pointer = cap_ptr;
    let mut guard = 0;
    let mut caps = PciCapabilities::default();

    // Walk the capability list
    while pointer >= 0x40 && guard < 64 {
        let current = pointer;
        let addr = function_base + current as u64;
        let cap_id = unsafe { core::ptr::read_volatile(addr as *const u8) };
        let next = unsafe { core::ptr::read_volatile((addr + 1) as *const u8) };
        
        match cap_id {
            0x05 => {
                caps.msi = true;
                caps.msi_pointer = Some(current);
            }
            0x11 => {
                caps.msix = true;
                caps.msix_pointer = Some(current);
            }
            _ => {}
        }
        
        if next == 0 || next == pointer { break; }
        pointer = next;
        guard += 1;
    }

    caps
}
```

### MSI Configuration

Message Signaled Interrupts (MSI) provide better performance than legacy interrupts:

#### 1. MSI Enable Function
```rust
pub fn enable_msi(
    device: &PciDeviceInfo,
    regions: &[PciConfigRegion],
    apic_id: u8,
    vector: u8,
) -> Result<(), &'static str> {
    // Find MSI capability
    let cap_ptr = device.capabilities.msi_pointer
        .ok_or("MSI capability not present")?;

    // Find ECAM region for this device
    let base = find_device_config_base(device, regions)?;

    // Read MSI control register
    let control_offset = cap_ptr + 2;
    let mut control = config_read_u16(base, control_offset);
    let is_64bit = (control & (1 << 7)) != 0;
    let per_vector_mask = (control & (1 << 8)) != 0;

    // Program MSI for single vector
    control &= !0x000E; // Clear multi-message bits
    let msg_addr = MSI_ADDR_BASE | ((apic_id as u32) << 12);
    let msg_data = vector as u16;

    // Write message address
    config_write_u32(base, cap_ptr + 4, msg_addr);
    let mut data_offset = cap_ptr + 8;
    
    // Handle 64-bit address capability
    if is_64bit {
        config_write_u32(base, cap_ptr + 8, 0);
        data_offset = cap_ptr + 12;
    }
    
    // Write message data (interrupt vector)
    config_write_u16(base, data_offset, msg_data);

    // Handle per-vector masking if supported
    if per_vector_mask {
        config_write_u32(base, data_offset + 2, 0); // mask bits
        config_write_u32(base, data_offset + 6, 0); // pending bits
    }

    // Enable MSI
    control |= 0x0001;
    config_write_u16(base, control_offset, control);

    Ok(())
}
```

### Configuration Space Access

The system provides safe access to PCI configuration space through ECAM:

#### 1. Address Calculation
```rust
fn function_base(region: &PciConfigRegion, bus: u8, device: u8, function: u8) -> Option<u64> {
    if bus < region.bus_start || bus > region.bus_end ||
       device > MAX_DEVICE || function > MAX_FUNCTION {
        return None;
    }

    let bus_offset = u64::from(bus - region.bus_start) * BUS_STRIDE;
    let device_offset = u64::from(device) * DEVICE_STRIDE;
    let function_offset = u64::from(function) * FUNCTION_STRIDE;

    Some(region.virt_base + bus_offset + device_offset + function_offset)
}
```

#### 2. Safe Access Functions
```rust
fn config_read_u32(function_base: u64, offset: u8) -> u32 {
    let addr = function_base + offset as u64;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

fn config_read_u16(function_base: u64, offset: u8) -> u16 {
    let aligned = offset & !0x3;
    let value = config_read_u32(function_base, aligned);
    let shift = (offset & 0x3) * 8;
    ((value >> shift) & 0xFFFF) as u16
}

fn config_read_u8(function_base: u64, offset: u8) -> u8 {
    let aligned = offset & !0x3;
    let value = config_read_u32(function_base, aligned);
    let shift = (offset & 0x3) * 8;
    ((value >> shift) & 0xFF) as u8
}
```

---

## System Integration

### Initialization Sequence

The driver system initialization follows a specific sequence:

#### 1. ACPI Initialization
```rust
let mut platform_info = acpi::initialize_acpi(handoff.acpi_rsdp)?;
```

#### 2. Hardware Inventory Processing
```rust
if handoff.hardware_device_count > 0 {
    // Process UEFI hardware inventory
    for idx in 0..handoff.hardware_device_count {
        let entry = HardwareDevice::from_bytes(slice, idx)?;
        let device = create_device_from_inventory(entry);
        driver_manager().lock().add_device(device);
    }
}
```

#### 3. PCI Enumeration
```rust
let pci_topology = pci::enumerate(&platform_info.pci_config_regions);
for info in pci_topology.functions.iter() {
    let device = create_device_from_pci(info);
    driver_manager().lock().add_device(device);
}
```

#### 4. Driver Registration
```rust
// Register built-in drivers
serial::init_serial();
// Other drivers register themselves
```

### Device Classification

PCI devices are classified based on their class codes:

```rust
fn classify_pci_device(info: &pci::PciDeviceInfo) -> DeviceClass {
    match info.class_code {
        0x0C => match info.subclass {
            0x03 => DeviceClass::UsbController, // USB
            _ => DeviceClass::Unknown,
        },
        0x01 => match info.subclass {
            0x06 => DeviceClass::Storage, // SATA AHCI
            0x08 => DeviceClass::Storage, // NVM Express
            _ => DeviceClass::Storage,
        },
        0x02 => DeviceClass::Network,
        0x06 => match info.subclass {
            0x00 | 0x04 => DeviceClass::Bridge,
            _ => DeviceClass::Unknown,
        },
        _ => DeviceClass::Unknown,
    }
}
```

### Resource Management

The system converts PCI resources to device resources:

```rust
// Convert PCI BARs to device resources
for bar in info.bars.iter() {
    match bar {
        PciBar::Memory32 { base, size, prefetchable } |
        PciBar::Memory64 { base, size, prefetchable } => {
            if *size > 0 {
                device.resources.push(DeviceResource::Memory {
                    base: *base, size: *size, prefetchable: *prefetchable,
                });
            }
        }
        PciBar::Io { base, size } => {
            if *size > 0 {
                device.resources.push(DeviceResource::Io {
                    base: *base, size: *size,
                });
            }
        }
        PciBar::None => {}
    }
}
```

---

## Implementation Details

### Memory Layout

#### DMA Buffer Layout
```
┌─────────────────────────────────────────────┐
│              Virtual Memory                 │
│  ┌─────────────────────────────────────────┐ │
│  │         Kernel Address Space            │ │
│  │  ┌─────────────────────────────────────┐ │ │
│  │  │        DMA Buffer Pool              │ │ │
│  │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ │ │ │
│  │  │  │ Slab 0  │ │ Slab 1  │ │ Slab N  │ │ │ │
│  │  │  └─────────┘ └─────────┘ └─────────┘ │ │ │
│  │  └─────────────────────────────────────┘ │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────┐
│             Physical Memory                 │
│  ┌─────────────────────────────────────────┐ │
│  │        Contiguous Regions               │ │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐   │ │
│  │  │Region 0 │ │Region 1 │ │Region N │   │ │
│  │  └─────────┘ └─────────┘ └─────────┘   │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

#### PCI Configuration Space Layout
```
┌─────────────────────────────────────────────┐
│            ECAM Address Space               │
│  ┌─────────────────────────────────────────┐ │
│  │              Bus 0                      │ │
│  │  ┌─────────────────────────────────────┐ │ │
│  │  │            Device 0                 │ │ │
│  │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ │ │ │
│  │  │  │Func 0   │ │Func 1   │ │Func 7   │ │ │ │
│  │  │  └─────────┘ └─────────┘ └─────────┘ │ │ │
│  │  └─────────────────────────────────────┘ │ │
│  │  ┌─────────────────────────────────────┐ │ │
│  │  │            Device 1                 │ │ │
│  │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ │ │ │
│  │  │  │Func 0   │ │Func 1   │ │Func 7   │ │ │ │
│  │  │  └─────────┘ └─────────┘ └─────────┘ │ │ │
│  │  └─────────────────────────────────────┘ │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### Error Handling

#### DMA Errors
- **Allocation Failures**: Out of memory, invalid alignment
- **Mapping Failures**: Page table allocation failures
- **Cache Policy Errors**: Invalid cache configuration

#### PCI Errors
- **Configuration Access**: Invalid ECAM regions, device not present
- **BAR Parsing**: Invalid BAR values, size calculation errors
- **Bridge Configuration**: Bus allocation failures, reset failures
- **MSI Configuration**: Missing capabilities, invalid parameters

#### Driver Errors
- **Probe Failures**: Device not supported, resource conflicts
- **Initialization Failures**: Hardware setup errors, resource allocation
- **Runtime Errors**: I/O failures, interrupt handling errors

### Performance Considerations

#### DMA Performance
- **Pool Allocation**: O(1) allocation from free list
- **Memory Mapping**: Page-aligned regions for efficiency
- **Cache Policies**: Appropriate cache behavior for use case
- **Fragmentation**: Slab-based allocation reduces fragmentation

#### PCI Performance
- **Bus Scanning**: Efficient device discovery algorithm
- **Bridge Handling**: Minimal configuration overhead
- **Resource Parsing**: Fast BAR size calculation
- **Capability Parsing**: Efficient linked list traversal

#### Driver Performance
- **Binding Algorithm**: First-match wins for speed
- **IRQ Dispatch**: O(n) search through devices
- **Class-based I/O**: Direct driver lookup
- **Thread Safety**: Minimal lock contention

### MSI Programming Helper

The `pci::enable_msi` helper wraps the book-keeping needed to transition a
device from legacy INTx to MSI delivery. It locates the MSI capability exposed
through `PciCapabilities`, programmes the LAPIC destination ID and interrupt
vector, copes with 32- versus 64-bit message formats, and clears optional
mask/pending registers when present. Drivers no longer need to hand-roll this
logic—they simply reserve a vector, disable their IOAPIC route, and call the
helper.

### Interactive Inspection Tools

The kernel monitor exposes two commands that are invaluable while validating
PCI discovery and resource sizing:

- **`devices`** – enumerates the driver manager's view of the world, including
  merged `DeviceResource` entries and IRQ assignments. Use this to verify
  that BAR relocations made it into the runtime registry.
- **`pci`** – re-runs ECAM enumeration on demand and prints each function's
  vendor/device IDs, class triplet, header type, and BAR information. It also
  lists the active bridge windows so you can confirm secondary/subordinate bus
  assignments after hot changes.

Run these commands before digging into logs—they mirror exactly what the kernel
discovered during boot and save full reboot cycles when iterating on resource
allocation.

---

## Troubleshooting Guide

### Common Issues

#### 1. DMA Allocation Failures
**Symptoms**: `AllocError::OutOfMemory` when allocating DMA buffers
**Causes**:
- Insufficient physical memory
- Memory fragmentation
- Invalid alignment requirements
**Solutions**:
- Check available physical memory
- Use smaller buffer sizes
- Ensure alignment is power of 2

#### 2. PCI Device Not Discovered
**Symptoms**: Expected PCI device not found during enumeration
**Causes**:
- Device not present
- Bridge configuration issues
- ECAM region problems
**Solutions**:
- Check device presence with `lspci`
- Verify bridge configuration (`pci` monitor command shows secondary/subordinate buses and bridge windows)
- Check ECAM region validity

#### 3. Driver Binding Failures
**Symptoms**: Device discovered but no driver bound
**Causes**:
- No compatible driver registered
- Driver probe failure
- Resource conflicts
**Solutions**:
- Register appropriate driver
- Check driver probe logic
- Verify resource availability

#### 4. MSI Configuration Failures
**Symptoms**: `enable_msi()` returns error
**Causes**:
- Device doesn't support MSI
- Invalid APIC configuration
- Capability parsing errors
**Solutions**:
- Check device MSI capability
- Verify APIC setup
- Check capability pointer validity

### Debugging Tools

#### 1. PCI Monitor Commands
```rust
// List all PCI devices
pci list

// Show device details
pci show <bus>:<device>.<function>

// Enable MSI for device
pci msi <bus>:<device>.<function> <apic_id> <vector>
```

#### 2. Driver Manager Commands
```rust
// List all devices
devices list

// List all drivers
drivers list

// Show device binding
devices show <device_id>
```

#### 3. DMA Pool Commands
```rust
// Show pool statistics
dma pool stats

// Allocate test buffer
dma alloc <size> <alignment>

// Show buffer information
dma show <buffer_id>
```

### Performance Profiling

#### 1. DMA Performance
- Monitor allocation times
- Track memory usage
- Measure cache hit rates
- Profile mapping overhead

#### 2. PCI Performance
- Measure enumeration time
- Track bridge configuration overhead
- Monitor resource parsing time
- Profile capability discovery

#### 3. Driver Performance
- Measure binding time
- Track IRQ dispatch latency
- Monitor I/O operation times
- Profile memory usage

---

## Conclusion

The TheseusOS driver system provides a robust, type-safe, and performant framework for device management. The DMA subsystem efficiently handles memory allocation for device operations, while the PCI discovery process comprehensively enumerates all devices in the system. The driver framework ties everything together with a clean, extensible architecture that supports both simple and complex device scenarios.

This deep-dive document should provide kernel developers with the comprehensive understanding needed to work with, maintain, and extend these critical subsystems. For additional information, refer to the source code documentation and the other technical documents in the TheseusOS documentation suite.
