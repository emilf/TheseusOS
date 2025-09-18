#![allow(dead_code)]
//! CPU setup and control module
//! 
//! This module provides functions to configure CPU features, control registers,
//! and set up the CPU for kernel operation.

/// Control register flags
pub const CR0_PE: u64 = 1 << 0;   // Protected Mode Enable
pub const CR0_MP: u64 = 1 << 1;   // Monitor Coprocessor
pub const CR0_EM: u64 = 1 << 2;   // Emulation
pub const CR0_TS: u64 = 1 << 3;   // Task Switched
pub const CR0_ET: u64 = 1 << 4;   // Extension Type
pub const CR0_NE: u64 = 1 << 5;   // Numeric Error
pub const CR0_WP: u64 = 1 << 16;  // Write Protect
pub const CR0_AM: u64 = 1 << 18;  // Alignment Mask
pub const CR0_NW: u64 = 1 << 29;  // Not Writethrough
pub const CR0_CD: u64 = 1 << 30;  // Cache Disable
pub const CR0_PG: u64 = 1 << 31;  // Paging Enable

pub const CR4_VME: u64 = 1 << 0;  // Virtual 8086 Mode Extensions
pub const CR4_PVI: u64 = 1 << 1;  // Protected Mode Virtual Interrupts
pub const CR4_TSD: u64 = 1 << 2;  // Time Stamp Disable
pub const CR4_DE: u64 = 1 << 3;   // Debugging Extensions
pub const CR4_PSE: u64 = 1 << 4;  // Page Size Extensions
pub const CR4_PAE: u64 = 1 << 5;  // Physical Address Extension
pub const CR4_MCE: u64 = 1 << 6;  // Machine Check Enable
pub const CR4_PGE: u64 = 1 << 7;  // Page Global Enable
pub const CR4_PCE: u64 = 1 << 8;  // Performance Counter Enable
pub const CR4_OSFXSR: u64 = 1 << 9; // OS FXSAVE/FXRSTOR Support
pub const CR4_OSXMMEXCPT: u64 = 1 << 10; // OS XMM Exception Support
pub const CR4_UMIP: u64 = 1 << 11; // User Mode Instruction Prevention
pub const CR4_VMXE: u64 = 1 << 13; // Virtual Machine Extensions Enable
pub const CR4_SMXE: u64 = 1 << 14; // Safer Mode Extensions Enable
pub const CR4_FSGSBASE: u64 = 1 << 16; // Enable RDFSBASE/RDGSBASE/WRFSBASE/WRGSBASE
pub const CR4_PCIDE: u64 = 1 << 17; // PCID Enable
pub const CR4_OSXSAVE: u64 = 1 << 18; // XSAVE and Processor Extended States Enable
pub const CR4_SMEP: u64 = 1 << 20; // Supervisor Mode Execution Prevention
pub const CR4_SMAP: u64 = 1 << 21; // Supervisor Mode Access Prevention

/// CPU feature flags
#[derive(Debug, Clone, Copy)]
pub struct CpuFeatures {
    pub sse: bool,
    pub sse2: bool,
    pub sse3: bool,
    pub ssse3: bool,
    pub sse4_1: bool,
    pub sse4_2: bool,
    pub avx: bool,
    pub avx2: bool,
    pub fma: bool,
    pub xsave: bool,
    pub osxsave: bool,
}

impl CpuFeatures {
    pub const fn new() -> Self {
        Self {
            sse: false,
            sse2: false,
            sse3: false,
            ssse3: false,
            sse4_1: false,
            sse4_2: false,
            avx: false,
            avx2: false,
            fma: false,
            xsave: false,
            osxsave: false,
        }
    }
}

/// Set up control registers for kernel mode
pub unsafe fn setup_control_registers() {
    // Configure CR0
    let mut cr0: u64;
    core::arch::asm!("mov {}, cr0", out(reg) cr0);
    
    // Enable protected mode, paging, write protection, alignment check
    cr0 |= CR0_PE | CR0_PG | CR0_WP | CR0_AM | CR0_NE;
    
    // Clear emulation bit (disable x87 emulation)
    cr0 &= !CR0_EM;
    
    // Set monitor coprocessor bit
    cr0 |= CR0_MP;
    
    core::arch::asm!("mov cr0, {}", in(reg) cr0);
    
    // Configure CR4
    let mut cr4: u64;
    core::arch::asm!("mov {}, cr4", out(reg) cr4);
    
    // Enable PAE, PGE, OSFXSR, OSXMMEXCPT, SMEP, SMAP
    cr4 |= CR4_PAE | CR4_PGE | CR4_OSFXSR | CR4_OSXMMEXCPT | CR4_SMEP | CR4_SMAP;
    
    // Enable FSGSBASE if available
    if has_fsgsbase() {
        cr4 |= CR4_FSGSBASE;
    }
    
    // Enable OSXSAVE if available
    if has_osxsave() {
        cr4 |= CR4_OSXSAVE;
    }
    
    core::arch::asm!("mov cr4, {}", in(reg) cr4);
}

/// Detect CPU features using CPUID
pub unsafe fn detect_cpu_features() -> CpuFeatures {
    let mut features = CpuFeatures::new();
    
    // Check standard features (CPUID.1:EDX)
    let (_, _, _, edx) = cpuid(1, 0);
    
    features.sse = (edx & (1 << 25)) != 0;  // SSE
    features.sse2 = (edx & (1 << 26)) != 0; // SSE2
    
    // Check extended features (CPUID.1:ECX)
    let (_, _, ecx, _) = cpuid(1, 0);
    
    features.sse3 = (ecx & (1 << 0)) != 0;   // SSE3
    features.ssse3 = (ecx & (1 << 9)) != 0;  // SSSE3
    features.sse4_1 = (ecx & (1 << 19)) != 0; // SSE4.1
    features.sse4_2 = (ecx & (1 << 20)) != 0; // SSE4.2
    features.avx = (ecx & (1 << 28)) != 0;    // AVX
    features.xsave = (ecx & (1 << 26)) != 0;  // XSAVE
    features.osxsave = (ecx & (1 << 27)) != 0; // OSXSAVE
    
    // Check extended features (CPUID.7:EBX)
    let (_, ebx, _, _) = cpuid(7, 0);
    
    features.avx2 = (ebx & (1 << 5)) != 0;   // AVX2
    features.fma = (ebx & (1 << 12)) != 0;   // FMA
    
    features
}

/// Enable SSE/AVX if available
pub unsafe fn setup_floating_point(features: &CpuFeatures) {
    // Enable SSE
    if features.sse {
        enable_sse();
    }
    
    // Enable AVX if available
    if features.avx && features.osxsave {
        enable_avx();
    }
}

/// Enable SSE
unsafe fn enable_sse() {
    // Set CR0.EM = 0 and CR0.MP = 1
    let mut cr0: u64;
    core::arch::asm!("mov {}, cr0", out(reg) cr0);
    cr0 &= !CR0_EM;
    cr0 |= CR0_MP;
    core::arch::asm!("mov cr0, {}", in(reg) cr0);
    
    // Set CR4.OSFXSR = 1 and CR4.OSXMMEXCPT = 1
    let mut cr4: u64;
    core::arch::asm!("mov {}, cr4", out(reg) cr4);
    cr4 |= CR4_OSFXSR | CR4_OSXMMEXCPT;
    core::arch::asm!("mov cr4, {}", in(reg) cr4);
}

/// Enable AVX
unsafe fn enable_avx() {
    // Set CR4.OSXSAVE = 1
    let mut cr4: u64;
    core::arch::asm!("mov {}, cr4", out(reg) cr4);
    cr4 |= CR4_OSXSAVE;
    core::arch::asm!("mov cr4, {}", in(reg) cr4);
    
    // Set XCR0 to enable AVX
    let mut eax: u32;
    let mut edx: u32;
    
    core::arch::asm!(
        "xgetbv",
        out("eax") eax,
        out("edx") edx,
        options(nomem, nostack, preserves_flags)
    );
    
    // Enable X87, SSE, and AVX state
    eax |= 0x7;
    
    core::arch::asm!(
        "xsetbv",
        in("eax") eax,
        in("edx") edx,
        options(nomem, nostack, preserves_flags)
    );
}

/// Execute CPUID instruction
unsafe fn cpuid(eax: u32, ecx: u32) -> (u32, u32, u32, u32) {
    let mut eax_out: u32;
    let mut ebx_out: u32;
    let mut ecx_out: u32;
    let mut edx_out: u32;
    
    core::arch::asm!(
        "cpuid",
        inout("eax") eax => eax_out,
        inout("rcx") ecx => ecx_out,
        inout("rdx") 0 => edx_out,
        options(nomem, nostack, preserves_flags)
    );
    
    // Get EBX using a separate instruction
    core::arch::asm!(
        "mov {0:e}, ebx",
        out(reg) ebx_out,
        options(nomem, nostack, preserves_flags)
    );
    
    (eax_out, ebx_out, ecx_out, edx_out)
}

/// Check if FSGSBASE is available
unsafe fn has_fsgsbase() -> bool {
    let (_, ebx, _, _) = cpuid(7, 0);
    (ebx & (1 << 0)) != 0
}

/// Check if OSXSAVE is available
unsafe fn has_osxsave() -> bool {
    let (_, _, ecx, _) = cpuid(1, 0);
    (ecx & (1 << 27)) != 0
}

/// Set up Model Specific Registers (MSRs)
pub unsafe fn setup_msrs() {
    // Set up EFER (Extended Feature Enable Register)
    let mut efer: u64;
    core::arch::asm!(
        "rdmsr",
        in("ecx") 0xC0000080u32,
        out("eax") efer,
        options(nomem, nostack, preserves_flags)
    );
    
    // Enable SYSCALL/SYSRET (if not already enabled)
    efer |= 0x1; // SCE (SYSCALL Enable)
    
    core::arch::asm!(
        "wrmsr",
        in("ecx") 0xC0000080u32,
        in("eax") efer,
        options(nomem, nostack, preserves_flags)
    );
}
