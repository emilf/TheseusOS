//! System inspection commands
//!
//! This module implements commands for inspecting system state:
//! - `phys`: Display physical memory manager statistics
//! - `regs`: Display CPU registers (GPRs, control, segment)
//! - `stack`/`bt`: Display stack backtrace
//! - `acpi`: Display ACPI information

use crate::monitor::Monitor;
use crate::{acpi, memory, physical_memory};
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::str;

impl Monitor {
    /// Display physical memory manager statistics
    ///
    /// Shows statistics from the persistent physical memory allocator, including:
    /// - Base PFN (Page Frame Number)
    /// - Total frames available
    /// - Free frames remaining
    /// - Guard window size below runtime kernel base
    ///
    /// # Examples
    /// ```text
    /// phys              # Show physical memory statistics
    /// physmem           # Alias for phys
    /// ```
    pub(in crate::monitor) fn cmd_phys(&self) {
        if let Some(stats) = physical_memory::stats() {
            let page_size = memory::PAGE_SIZE as u64;
            let total_bytes = stats.total_frames.saturating_mul(page_size);
            let free_bytes = stats.free_frames.saturating_mul(page_size);
            let guard = memory::runtime_kernel_lower_guard();
            self.writeln("Persistent Physical Memory Manager:");
            self.writeln(&format!(
                "  base PFN:    0x{:016X}",
                stats.base_pfn
            ));
            self.writeln(&format!(
                "  total frames: {} ({:#X} bytes)",
                stats.total_frames, total_bytes
            ));
            self.writeln(&format!(
                "  free frames:  {} ({:#X} bytes)",
                stats.free_frames, free_bytes
            ));
            self.writeln(&format!(
                "  guard window: {:#X} bytes below runtime base",
                guard
            ));
        } else {
            self.writeln("Persistent allocator not initialised yet");
        }
    }

    /// Display CPU registers
    ///
    /// Shows a comprehensive snapshot of the current CPU state including:
    /// - General-purpose registers (RAX, RBX, RCX, RDX, RSI, RDI, R8-R15)
    /// - Stack and base pointers (RSP, RBP)
    /// - Instruction pointer (RIP)
    /// - Control registers (CR0, CR2, CR3, CR4, CR8)
    /// - Segment registers (CS, DS, SS, ES, FS, GS)
    /// - RFLAGS register
    ///
    /// # Examples
    /// ```text
    /// regs              # Show all CPU registers
    /// r                 # Short alias
    /// ```
    ///
    /// # Note
    /// Register values are captured at the time the monitor reads them.
    /// Some registers (like RIP) are approximate due to the reading process.
    pub(in crate::monitor) fn cmd_registers(&self) {
        self.writeln("CPU Registers:");

        macro_rules! read_gpr {
            ($reg:tt) => {{
                let value: u64;
                unsafe {
                    core::arch::asm!(
                        concat!("mov {0}, ", stringify!($reg)),
                        out(reg) value,
                        options(nomem, preserves_flags, nostack)
                    );
                }
                value
            }};
        }

        let rax = read_gpr!(rax);
        let rbx = read_gpr!(rbx);
        let rcx = read_gpr!(rcx);
        let rdx = read_gpr!(rdx);
        let rsi = read_gpr!(rsi);
        let rdi = read_gpr!(rdi);
        let rsp = read_gpr!(rsp);
        let rbp = read_gpr!(rbp);
        let r8 = read_gpr!(r8);
        let r9 = read_gpr!(r9);
        let r10 = read_gpr!(r10);
        let r11 = read_gpr!(r11);
        let r12 = read_gpr!(r12);
        let r13 = read_gpr!(r13);
        let r14 = read_gpr!(r14);
        let r15 = read_gpr!(r15);

        let rip: u64;
        unsafe {
            core::arch::asm!(
                "lea {0}, [rip]",
                out(reg) rip,
                options(nomem, preserves_flags)
            );
        }

        self.writeln(&format!("  RAX: 0x{:016X}  RBX: 0x{:016X}", rax, rbx));
        self.writeln(&format!("  RCX: 0x{:016X}  RDX: 0x{:016X}", rcx, rdx));
        self.writeln(&format!("  RSI: 0x{:016X}  RDI: 0x{:016X}", rsi, rdi));
        self.writeln(&format!("  RSP: 0x{:016X}  RBP: 0x{:016X}", rsp, rbp));
        self.writeln(&format!("  R8 : 0x{:016X}  R9 : 0x{:016X}", r8, r9));
        self.writeln(&format!("  R10: 0x{:016X}  R11: 0x{:016X}", r10, r11));
        self.writeln(&format!("  R12: 0x{:016X}  R13: 0x{:016X}", r12, r13));
        self.writeln(&format!("  R14: 0x{:016X}  R15: 0x{:016X}", r14, r15));
        self.writeln(&format!("  RIP: 0x{:016X}", rip));

        // Control registers
        let cr0: u64;
        let cr2: u64;
        let cr3: u64;
        let cr4: u64;
        let cr8: u64;

        unsafe {
            core::arch::asm!("mov {}, cr0", out(reg) cr0, options(nomem, nostack));
            core::arch::asm!("mov {}, cr2", out(reg) cr2, options(nomem, nostack));
            core::arch::asm!("mov {}, cr3", out(reg) cr3, options(nomem, nostack));
            core::arch::asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack));
            core::arch::asm!("mov {}, cr8", out(reg) cr8, options(nomem, nostack));
        }

        self.writeln("");
        self.writeln("Control Registers:");
        self.writeln(&format!("  CR0: 0x{:016X}  CR2: 0x{:016X}", cr0, cr2));
        self.writeln(&format!("  CR3: 0x{:016X}  CR4: 0x{:016X}", cr3, cr4));
        self.writeln(&format!("  CR8: 0x{:016X}", cr8));

        // Segment registers
        let cs: u16;
        let ds: u16;
        let ss: u16;
        let es: u16;
        let fs: u16;
        let gs: u16;

        unsafe {
            core::arch::asm!("mov {:x}, cs", out(reg) cs, options(nomem, nostack));
            core::arch::asm!("mov {:x}, ds", out(reg) ds, options(nomem, nostack));
            core::arch::asm!("mov {:x}, ss", out(reg) ss, options(nomem, nostack));
            core::arch::asm!("mov {:x}, es", out(reg) es, options(nomem, nostack));
            core::arch::asm!("mov {:x}, fs", out(reg) fs, options(nomem, nostack));
            core::arch::asm!("mov {:x}, gs", out(reg) gs, options(nomem, nostack));
        }

        self.writeln("");
        self.writeln("Segment Registers:");
        self.writeln(&format!(
            "  CS: 0x{:04X}  DS: 0x{:04X}  SS: 0x{:04X}  ES: 0x{:04X}",
            cs, ds, ss, es
        ));
        self.writeln(&format!("  FS: 0x{:04X}  GS: 0x{:04X}", fs, gs));

        let rflags = x86_64::registers::rflags::read().bits();
        self.writeln("");
        self.writeln(&format!("RFLAGS: 0x{:016X}", rflags));
    }

    /// Display ACPI information
    ///
    /// Shows ACPI (Advanced Configuration and Power Interface) platform information:
    /// - RSDP (Root System Description Pointer) location and details
    /// - RSDT/XSDT addresses
    /// - OEM ID and revision
    /// - Checksum validation
    /// - Platform summary (CPU count, APIC configuration, legacy PIC)
    /// - MADT (Multiple APIC Description Table) details
    ///
    /// # Examples
    /// ```text
    /// acpi              # Show ACPI information
    /// ```
    pub(in crate::monitor) fn cmd_acpi(&self) {
        self.writeln("ACPI Information:");

        let handoff = unsafe {
            &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff)
        };

        if handoff.acpi_rsdp == 0 {
            self.writeln("  RSDP: not present in handoff");
        } else {
            self.writeln(&format!(
                "  RSDP Physical Address: 0x{:016X}",
                handoff.acpi_rsdp
            ));

            match parse_rsdp_info(handoff.acpi_rsdp) {
                Ok(info) => {
                    let oem_str = match str::from_utf8(&info.oem_id) {
                        Ok(s) => s.trim_end_matches(char::from(0)),
                        Err(_) => "???",
                    };
                    self.writeln(&format!(
                        "  Signature: {}  Revision: {} ({})",
                        info.signature_string(),
                        info.revision,
                        info.revision_label()
                    ));
                    self.writeln(&format!("  OEM ID: {}", oem_str));
                    self.writeln(&format!("  RSDT Address: 0x{:016X}", info.rsdt_address));
                    if let Some(xsdt) = info.xsdt_address {
                        self.writeln(&format!("  XSDT Address: 0x{:016X}", xsdt));
                    }
                    self.writeln(&format!(
                        "  Checksum: {}",
                        if info.checksum_ok { "valid" } else { "INVALID" }
                    ));
                    if let Some(ok) = info.extended_checksum_ok {
                        self.writeln(&format!(
                            "  Extended Checksum: {}",
                            if ok { "valid" } else { "INVALID" }
                        ));
                    }
                }
                Err(err) => {
                    self.writeln(&format!("  RSDP decode failed: {}", err));
                }
            }
        }

        self.writeln("");
        self.writeln("Platform Summary:");
        if let Some(info) = acpi::cached_platform_info() {
            self.writeln(&format!("  CPUs reported: {}", info.cpu_count));
            self.writeln(&format!(
                "  IO APICs: {} (present: {})",
                info.io_apic_count,
                if info.has_io_apic { "yes" } else { "no" }
            ));
            self.writeln(&format!("  Local APIC: 0x{:016X}", info.local_apic_address));
            self.writeln(&format!(
                "  Legacy PIC present: {}",
                if info.has_legacy_pic { "yes" } else { "no" }
            ));

            if let Some(ref madt) = info.madt_info {
                let apic_ids: Vec<String> = madt
                    .cpu_apic_ids
                    .iter()
                    .map(|id| format!("0x{:02X}", id))
                    .collect();
                self.writeln("  MADT:");
                if apic_ids.is_empty() {
                    self.writeln("    CPU APIC IDs: (none)");
                } else {
                    self.writeln(&format!("    CPU APIC IDs: {}", apic_ids.join(", ")));
                }
                if madt.io_apics.is_empty() {
                    self.writeln("    IO APICs: (none)");
                } else {
                    for entry in &madt.io_apics {
                        self.writeln(&format!(
                            "    IO APIC id:{} addr:0x{:016X} gsi_base:{}",
                            entry.id, entry.address, entry.gsi_base
                        ));
                    }
                }
                self.writeln(&format!(
                    "    Has 8259 PIC: {}",
                    if madt.has_8259_pic { "yes" } else { "no" }
                ));
            } else {
                self.writeln("  MADT: not parsed (platform info missing details)");
            }
        } else {
            self.writeln("  (ACPI platform info cache is empty)");
            self.writeln("  Hint: ensure driver system initialization has completed.");
        }
    }

    /// Display stack backtrace
    ///
    /// Walks the stack frame pointers (RBP chain) to show the call stack.
    /// Displays up to 16 frames with return addresses and frame pointers.
    ///
    /// # Examples
    /// ```text
    /// stack             # Show stack backtrace
    /// bt                # Short alias (backtrace)
    /// ```
    ///
    /// # Safety
    /// Reads memory at frame pointer addresses. May show garbage if stack
    /// is corrupted or if frame pointers are not being used.
    pub(in crate::monitor) fn cmd_stack_trace(&self) {
        self.writeln("Stack trace:");

        unsafe {
            let mut rbp: u64;
            // Read current frame pointer
            core::arch::asm!("mov {}, rbp", out(reg) rbp, options(nostack));

            // Walk up to 16 frames
            for frame in 0..16 {
                // Validate RBP is in kernel space (high-half)
                // x86-64 kernel addresses start at 0xFFFF_8000_0000_0000
                if rbp == 0 || rbp < 0xFFFF_8000_0000_0000 {
                    break;
                }

                // Stack frame layout (x86-64 calling convention):
                // [RBP + 0]: Previous RBP (saved frame pointer)
                // [RBP + 8]: Return address (saved RIP)
                let ret_addr_ptr = (rbp + 8) as *const u64;
                let prev_rbp_ptr = rbp as *const u64;

                let ret_addr = core::ptr::read_volatile(ret_addr_ptr);
                let prev_rbp = core::ptr::read_volatile(prev_rbp_ptr);

                self.writeln(&format!(
                    "  Frame #{:2}: RIP=0x{:016X} RBP=0x{:016X}",
                    frame, ret_addr, rbp
                ));

                // Move to next (caller's) frame
                rbp = prev_rbp;

                // Prevent infinite loops (detect circular references)
                if rbp == prev_rbp {
                    break;
                }
            }
        }
    }
}

// Helper structures and functions for ACPI parsing

/// Information parsed from the RSDP (Root System Description Pointer)
///
/// This structure contains the parsed contents of an ACPI RSDP table,
/// which is the entry point for ACPI table discovery.
struct RsdpInfo {
    signature: [u8; 8],
    oem_id: [u8; 6],
    revision: u8,
    rsdt_address: u64,
    xsdt_address: Option<u64>,
    checksum_ok: bool,
    extended_checksum_ok: Option<bool>,
}

impl RsdpInfo {
    /// Convert the 8-byte signature to a displayable string
    fn signature_string(&self) -> String {
        self.signature.iter().map(|&b| b as char).collect()
    }

    /// Get a human-readable ACPI version label based on revision number
    fn revision_label(&self) -> &'static str {
        match self.revision {
            0 | 1 => "ACPI 1.0",
            2 | 3 => "ACPI 2.0+",
            _ => "ACPI",
        }
    }
}

/// Parse RSDP information from a physical address
///
/// Reads and validates an ACPI RSDP structure, including checksums.
/// Supports both ACPI 1.0 (20-byte RSDP) and ACPI 2.0+ (36-byte extended RSDP).
///
/// # Arguments
/// * `rsdp_phys` - Physical address of the RSDP
///
/// # Returns
/// * `Ok(RsdpInfo)` - Successfully parsed RSDP
/// * `Err(&str)` - Error message if parsing failed
///
/// # Safety
/// Requires PHYS_OFFSET mapping to be active to access the physical address.
fn parse_rsdp_info(rsdp_phys: u64) -> Result<RsdpInfo, &'static str> {
    if rsdp_phys == 0 {
        return Err("RSDP address is zero");
    }
    if !memory::phys_offset_is_active() {
        return Err("PHYS_OFFSET mapping inactive; RSDP inaccessible");
    }

    let rsdp_va = memory::phys_to_virt_pa(rsdp_phys);
    let ptr = rsdp_va as *const u8;

    unsafe {
        // Read and validate 8-byte signature (should be "RSD PTR ")
        let mut signature = [0u8; 8];
        for i in 0..8 {
            signature[i] = core::ptr::read_volatile(ptr.add(i));
        }
        if &signature != b"RSD PTR " {
            return Err("Invalid RSDP signature");
        }

        // Read 6-byte OEM ID (offset 9-14)
        let mut oem_id = [0u8; 6];
        for i in 0..6 {
            oem_id[i] = core::ptr::read_volatile(ptr.add(9 + i));
        }
        
        // Read revision (offset 15) - determines RSDP version
        let revision = core::ptr::read_volatile(ptr.add(15));
        
        // Read RSDT address (offset 16-19)
        let rsdt_address = core::ptr::read_unaligned(ptr.add(16) as *const u32) as u64;

        // Determine total length based on revision
        // ACPI 1.0 (rev 0): 20 bytes
        // ACPI 2.0+ (rev 2+): 36 bytes minimum
        let mut length = if revision >= 2 {
            core::ptr::read_unaligned(ptr.add(20) as *const u32)
        } else {
            20
        };
        // Sanity check: enforce minimum lengths
        if length < 20 {
            length = 20;
        }
        if revision >= 2 && length < 36 {
            length = 36;
        }

        // Read XSDT address (offset 24-31, only in ACPI 2.0+)
        let xsdt_address = if revision >= 2 {
            Some(core::ptr::read_unaligned(ptr.add(24) as *const u64))
        } else {
            None
        };

        // Verify checksums
        let checksum_ok = verify_checksum(ptr, 20);  // Basic checksum (first 20 bytes)
        let extended_checksum_ok = if revision >= 2 {
            Some(verify_checksum(ptr, length as usize))  // Extended checksum (all bytes)
        } else {
            None
        };

        Ok(RsdpInfo {
            signature,
            oem_id,
            revision,
            rsdt_address,
            xsdt_address,
            checksum_ok,
            extended_checksum_ok,
        })
    }
}

/// Verify ACPI table checksum
///
/// ACPI tables use a simple checksum: all bytes (including the checksum byte)
/// must sum to zero (mod 256).
///
/// # Arguments
/// * `ptr` - Pointer to the start of the ACPI structure
/// * `len` - Length in bytes to checksum
///
/// # Returns
/// * `true` - Checksum is valid
/// * `false` - Checksum verification failed
fn verify_checksum(ptr: *const u8, len: usize) -> bool {
    let mut sum: u8 = 0;
    for i in 0..len {
        unsafe {
            sum = sum.wrapping_add(core::ptr::read_volatile(ptr.add(i)));
        }
    }
    sum == 0
}

