//! TheseusOS Kernel
//! 
//! This is the main kernel module for TheseusOS, a bare-metal operating system
//! designed for x86-64 systems. The kernel is loaded by a UEFI bootloader and
//! takes control after boot services have been exited.
//! 
//! ## Architecture
//! 
//! The kernel is organized into several modules:
//! - `gdt`: Global Descriptor Table setup and management
//! - `interrupts`: Interrupt handling and control
//! - `cpu`: CPU feature detection and configuration
//! - `memory`: Memory management and page table structures (not yet active)
//! 
//! ## Boot Process
//! 
//! 1. UEFI bootloader loads the kernel binary
//! 2. Bootloader collects system information (memory map, ACPI, etc.)
//! 3. Bootloader exits UEFI boot services
//! 4. Bootloader jumps to kernel entry point
//! 5. Kernel initializes heap from pre-allocated memory
//! 6. Kernel sets up environment (interrupts, GDT, CPU features)
//! 7. Kernel begins normal operation

#![no_std]
#![no_main]

extern crate alloc;

// Include our modules
mod gdt;
mod interrupts;
mod cpu;
mod memory;

use gdt::setup_gdt;
use interrupts::{disable_all_interrupts, setup_idt};
use cpu::{setup_control_registers, detect_cpu_features, setup_floating_point, setup_msrs};
use memory::{MemoryManager, activate_virtual_memory};
// Global access to bootloader handoff
static mut HANDOFF_PHYS_PTR: u64 = 0;
static mut HANDOFF_VIRT_PTR: u64 = 0;
static mut HANDOFF_INITIALIZED: bool = false;

fn set_handoff_pointers(handoff_phys: u64) {
    let virt = memory::KERNEL_VIRTUAL_BASE.wrapping_add(handoff_phys);
    unsafe {
        HANDOFF_PHYS_PTR = handoff_phys;
        HANDOFF_VIRT_PTR = virt;
        HANDOFF_INITIALIZED = true;
    }
    kernel_write_line("[handoff] phys="); theseus_shared::print_hex_u64_0xe9!(handoff_phys);
    kernel_write_line(" virt="); theseus_shared::print_hex_u64_0xe9!(virt); kernel_write_line("\n");
}

#[allow(dead_code)]
fn handoff_ref() -> &'static theseus_shared::handoff::Handoff {
    // Choose phys or high-half pointer based on current RIP
    let virt_base = memory::KERNEL_VIRTUAL_BASE;
    let rip_now: u64; unsafe { core::arch::asm!("lea {}, [rip + 0]", out(reg) rip_now, options(nostack)); }
    let ptr = unsafe { if rip_now >= virt_base { HANDOFF_VIRT_PTR } else { HANDOFF_PHYS_PTR } };
    unsafe { &*(ptr as *const theseus_shared::handoff::Handoff) }
}

// use boot_services::exit_boot_services; // Not needed - bootloader handles this

/// Temporary bump allocator for kernel setup phase
/// 
/// This allocator uses memory pre-allocated by the bootloader and stored in the handoff structure.
/// It's designed for use during kernel initialization before proper page tables and memory management
/// are set up. The allocator provides a simple bump allocation strategy without deallocation support.
#[global_allocator]
static ALLOCATOR: BumpAllocator = BumpAllocator;

struct BumpAllocator;

// Global heap state
static mut HEAP_START: *mut u8 = core::ptr::null_mut();
static mut HEAP_END: *mut u8 = core::ptr::null_mut();
static mut HEAP_NEXT: *mut u8 = core::ptr::null_mut();
static mut HEAP_INITIALIZED: bool = false;

unsafe impl core::alloc::GlobalAlloc for BumpAllocator {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        // Initialize heap if not already done
        if !HEAP_INITIALIZED {
            init_heap_from_memory_map();
        }
        
        if HEAP_START.is_null() || HEAP_END.is_null() || HEAP_NEXT.is_null() {
            return core::ptr::null_mut();
        }
        
        // Align the next pointer to the required alignment
        let align = layout.align();
        let size = layout.size();
        
        // Calculate aligned address
        let addr = HEAP_NEXT as usize;
        let aligned_addr = (addr + align - 1) & !(align - 1);
        let new_next = aligned_addr + size;
        
        // Check if we have enough space
        if new_next <= HEAP_END as usize {
            let result = aligned_addr as *mut u8;
            // Update the next pointer
            HEAP_NEXT = new_next as *mut u8;
            result
        } else {
            // Out of memory
            core::ptr::null_mut()
        }
    }

    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: core::alloc::Layout) {
        // Bump allocator doesn't support deallocation
        // All memory is freed when the allocator is reset
    }
}

/// Initialize heap using temporary heap from handoff structure
/// 
/// This function sets up the kernel's heap using memory pre-allocated by the bootloader.
/// The bootloader allocates a chunk of safe memory and stores its address and size in the
/// handoff structure. If no temporary heap is available, falls back to a fixed safe region.
fn initialize_heap_from_handoff(handoff_addr: u64) {
    if handoff_addr == 0 {
        kernel_write_line("  No handoff structure provided, using fallback heap");
        unsafe {
            init_heap_fallback();
        }
        return;
    }
    
    unsafe {
        let handoff_ptr = handoff_addr as *const theseus_shared::handoff::Handoff;
        let handoff = &*handoff_ptr;
        
        // Check if bootloader allocated a temporary heap for us
        if handoff.temp_heap_base != 0 && handoff.temp_heap_size != 0 {
            kernel_write_line("  Using temporary heap from bootloader");
            
            // Set up the heap using the bootloader-allocated memory
            HEAP_START = handoff.temp_heap_base as *mut u8;
            HEAP_END = (handoff.temp_heap_base + handoff.temp_heap_size) as *mut u8;
            HEAP_NEXT = handoff.temp_heap_base as *mut u8;
            HEAP_INITIALIZED = true;
            
            kernel_write_line("  Heap initialized using bootloader-allocated memory");
        } else {
            kernel_write_line("  No temporary heap available, using fallback");
            init_heap_fallback();
        }
    }
}

/// Initialize heap from memory map in handoff structure (called by allocator)
unsafe fn init_heap_from_memory_map() {
    // This is called by the allocator if heap is not initialized
    // We'll use a fallback approach
    init_heap_fallback();
}

/// Fallback heap initialization using fixed safe memory region
unsafe fn init_heap_fallback() {
    // Use a safe memory region starting at 1MB
    // This is typically safe conventional memory
    let heap_start_addr = 0x100000usize;  // 1MB
    let heap_size = 0x100000;  // 1MB heap
    
    HEAP_START = heap_start_addr as *mut u8;
    HEAP_END = (heap_start_addr + heap_size) as *mut u8;
    HEAP_NEXT = heap_start_addr as *mut u8;
    HEAP_INITIALIZED = true;
    
    kernel_write_line("  Heap initialized using fallback method");
}


/// Test the temporary allocator with basic memory allocations
/// 
/// This function verifies that the bump allocator is working correctly by performing
/// simple string and vector allocations. It uses minimal allocations to avoid complex
/// formatting that could cause issues during early kernel initialization.
#[allow(dead_code)]
fn test_allocator() {
    kernel_write_line("  Testing bump allocator with heap allocations...");
    
    // Test basic allocation without formatting
    kernel_write_line("  Testing basic allocation...");
    let _test_string = alloc::format!("Test");
    kernel_write_line("  Basic allocation successful");
    
    kernel_write_line("  Testing vector allocation...");
    let _vec: alloc::vec::Vec<u32> = alloc::vec![1, 2, 3];
    kernel_write_line("  Vector allocation successful");
    
    kernel_write_line("✓ Bump allocator is working correctly");
}

/// Set up complete kernel environment (correct order)
/// 
/// This function performs the setup sequence in the correct order to establish kernel control:
/// 1. Exit boot services FIRST (to prevent firmware interference)
/// 2. Disable all interrupts including NMI
/// 3. Set up GDT and TSS
/// 4. Configure control registers
/// 5. Set up CPU features
/// 6. Test basic operations
fn setup_kernel_environment(_handoff: &theseus_shared::handoff::Handoff) {
    kernel_write_line("=== Setting up kernel environment ===");
    
    // Boot services have already been exited by the bootloader
    kernel_write_line("✓ Boot services already exited by bootloader");
    
    // 1. Disable all interrupts (now safe to do)
    kernel_write_line("1. Disabling all interrupts...");
    unsafe {
        disable_all_interrupts();
    }
    kernel_write_line("  ✓ All interrupts disabled");
    
    // 2. Set up GDT and TSS
    kernel_write_line("2. Setting up GDT...");
    unsafe {
        setup_gdt();
    }
    kernel_write_line("  ✓ GDT loaded and segments reloaded");
    
    // 3. Configure control registers (PAE etc.)
    kernel_write_line("3. Configuring control registers...");
    unsafe { setup_control_registers(); }
    kernel_write_line("  ✓ Control registers configured");

    // 3.5 Set up paging (identity map + high-half kernel) and load CR3
    kernel_write_line("3.5. Setting up paging...");
    unsafe {
        kernel_write_line("  [vm] before new");
        let mm = MemoryManager::new(_handoff);
        kernel_write_line("  [vm] after new; loading CR3");
        activate_virtual_memory(mm.page_table_root());
        kernel_write_line("  [vm] after CR3");
    }
    kernel_write_line("  ✓ Paging enabled (identity + high-half kernel)");
    
    // 4. Install low-half IDT, then jump to high-half, then reinstall IDT and continue
    //    We need the lower IDT for error handling if the jump to higher half fails
    kernel_write_line("4. Setting up CPU features...");
    unsafe { setup_idt(); }
    kernel_write_line("  IDT (low-half) installed");
    kernel_write_line("  [hh] preparing jump to high-half...");
    // Compute addresses and verify bytes before jumping to high-half
    let virt_base: u64 = memory::KERNEL_VIRTUAL_BASE;
    let rip_now: u64; unsafe { core::arch::asm!("lea {}, [rip + 0]", out(reg) rip_now, options(nostack)); }
    if rip_now >= virt_base {
        kernel_write_line("  [hh] already in high-half, skipping jump\n");
    } else {
        let target: u64 = virt_base.wrapping_add(rip_now);
        // Dump debug info and compare 8 bytes at low_rip vs target
        kernel_write_line("  hh dbg: low_rip="); theseus_shared::print_hex_u64_0xe9!(rip_now);
        kernel_write_line(" virt_base="); theseus_shared::print_hex_u64_0xe9!(virt_base);
        kernel_write_line(" target="); theseus_shared::print_hex_u64_0xe9!(target); kernel_write_line("\n");
        let low_q: u64 = unsafe { core::ptr::read_volatile(rip_now as *const u64) };
        let hi_q: u64  = unsafe { core::ptr::read_volatile(target as *const u64) };
        kernel_write_line("  hh dbg: low_q="); theseus_shared::print_hex_u64_0xe9!(low_q);
        kernel_write_line(" hi_q="); theseus_shared::print_hex_u64_0xe9!(hi_q);
        kernel_write_line(if low_q == hi_q { " equal\n" } else { " DIFF\n" });
        if low_q == hi_q {
            unsafe { core::arch::asm!("jmp rax", in("rax") target, options(noreturn)); }
        } else {
            kernel_write_line("  hh dbg: ABORT high-half jump due to mismatch\n");
        }
    }

    unsafe { setup_idt(); }
    kernel_write_line("  IDT installed");
    kernel_write_line("  Detecting CPU features...");
    unsafe {
        let features = detect_cpu_features();
        kernel_write_line("  CPU features detected");
        kernel_write_line("  Setting up floating point...");
        setup_floating_point(&features);
        kernel_write_line("  Floating point setup complete");
        kernel_write_line("  Setting up MSRs...");
        setup_msrs();
        kernel_write_line("  MSR setup complete");
    }
    kernel_write_line("  ✓ CPU features configured");
    
    kernel_write_line("=== Kernel environment setup complete ===");
    kernel_write_line("Kernel environment test completed successfully");
    
    // For testing interrupts: trigger #DE, then #GP, then #PF in separate runs
    #[allow(unreachable_code)] unsafe {
        // Select which to trigger by changing this constant index (0=DE,1=GP,2=PF)
        const WHICH: u8 = 3;
        if WHICH == 0 { // #BP via int3
            kernel_write_line("Triggering #BP test (int3)...");
            core::arch::asm!(
                "int3",
                options(noreturn)
            );
        } else if WHICH == 1 { // #UD via ud2
            kernel_write_line("Triggering #UD test (ud2)...");
            core::arch::asm!(
                "ud2",
                options(noreturn)
            );
        } else if WHICH == 2 { // #GP: load invalid segment selector into DS
            core::arch::asm!(
                "mov ax, 0xFFFF",
                "mov ds, ax",
                options(noreturn)
            );
        } else if WHICH == 3 { // #DE: divide by zero
            kernel_write_line("Triggering #DE test (divide by zero)...");
            core::arch::asm!(
                "xor rax, rax",
                "mov rdx, 1",
                "div rax",
                options(noreturn)
            );
        } else { // #PF: access unmapped VA
            kernel_write_line("Triggering #PF test (read from 0x50000000)...");
            core::arch::asm!(
                "mov rax, 0x50000000",
                "mov rax, [rax]",
                options(noreturn)
            );
        }
    }
}


/// Simple kernel output function that writes directly to QEMU debug port
fn kernel_write_line(message: &str) { theseus_shared::qemu_println!(message); }

// replaced by shared macros: out_char_0xe9! and print_hex_u64_0xe9!

/// Kernel entry point
/// 
/// This is where the kernel starts after the bootloader has:
/// 1. Collected all system information
/// 2. Exited boot services
/// 3. Jumped to this location
/// 
/// The handoff structure address is passed as a parameter.
#[no_mangle]
pub extern "C" fn kernel_main(handoff_addr: u64) -> ! {
    // Initialize kernel logging (placeholder)
    // TODO: Set up kernel logging system
    
    kernel_write_line("=== TheseusOS Kernel Starting ===");
    kernel_write_line("Kernel entry point reached successfully");

    // Initialize heap from memory map
    kernel_write_line("Initializing heap from memory map...");
    initialize_heap_from_handoff(handoff_addr);
    set_handoff_pointers(handoff_addr);
    
    kernel_write_line("Handoff structure address received");
    
    // Access the handoff structure from the passed address
    unsafe {
        if handoff_addr != 0 {
            let handoff_ptr = handoff_addr as *const theseus_shared::handoff::Handoff;
            let handoff = &*handoff_ptr;
            
            if handoff.size > 0 {
                kernel_write_line("Handoff structure found");
                
                // Display system information from handoff
                display_handoff_info(handoff);
                
                // Set up complete kernel environment (boot services have been exited)
                setup_kernel_environment(handoff);
                
            } else {
                kernel_write_line("ERROR: Handoff structure has invalid size");
            }
        } else {
            kernel_write_line("ERROR: Handoff structure address is null");
        }
    }
    
    // For now, just exit QEMU
    kernel_write_line("Kernel initialization complete");
    kernel_write_line("Exiting QEMU...");
    
    theseus_shared::qemu_exit_ok!();
    
    loop {}
}

/// Display information from the handoff structure
fn display_handoff_info(handoff: &theseus_shared::handoff::Handoff) {
    kernel_write_line("");
    kernel_write_line("┌─────────────────────────────────────────────────────────┐");
    kernel_write_line("│                Kernel Handoff Information               │");
    kernel_write_line("├─────────────────────────────────────────────────────────┤");
    
    kernel_write_line("│ Handoff Size: Available");
    kernel_write_line("│ Memory Map Entries: Available");
    kernel_write_line("│ Memory Map Size: Available");
    kernel_write_line("│ Memory Map Buffer: Available");
    
    if handoff.acpi_rsdp != 0 {
        kernel_write_line("│ ACPI RSDP: Available");
    } else {
        kernel_write_line("│ ACPI RSDP: Not available");
    }
    
    if handoff.gop_fb_base != 0 {
        kernel_write_line("│ Framebuffer: Available");
    } else {
        kernel_write_line("│ Framebuffer: Not available");
    }
    
    kernel_write_line("│ Hardware Devices: Available");
    
    // Virtual memory information
    kernel_write_line("│");
    kernel_write_line("│ Virtual Memory Information:");
    kernel_write_line("│   Virtual Base: Available");
    kernel_write_line("│   Physical Base: Available");
    kernel_write_line("│   Virtual Entry: Available");
    kernel_write_line("│   Page Table Root: Available");
    kernel_write_line("│   Virtual Memory: Available");
    
    kernel_write_line("└─────────────────────────────────────────────────────────┘");
    kernel_write_line("");
}


/// Panic handler for kernel
#[cfg(not(test))]
#[panic_handler]
fn panic_handler(_panic_info: &core::panic::PanicInfo) -> ! {
    // Try to output panic information
    // TODO: Do it without using allocator

    let message = "KERNEL PANIC: Panic occurred";
    
    theseus_shared::qemu_println!(message);
    theseus_shared::qemu_exit_error!();
    
    loop {}
}
