#![allow(dead_code)]
//! CPU setup and control module
//! 
//! This module provides functions to configure CPU features, control registers,
//! and set up the CPU for kernel operation.

use raw_cpuid::CpuId;
use spin::Once;
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

static CPU_FEATURES_ONCE: Once<CpuFeatures> = Once::new();

pub unsafe fn detect_cpu_features_once_lowhalf() {
    let _ = CPU_FEATURES_ONCE.call_once(|| detect_cpu_features());
}

// legacy path removed

pub fn get_cpu_features_once() -> Option<CpuFeatures> {
    CPU_FEATURES_ONCE.get().copied()
}

// single definition only; legacy removed

/// Set up control registers for kernel mode
pub unsafe fn setup_control_registers() {
    // Configure CR0
    {
        use x86_64::registers::control::{Cr0, Cr0Flags};
        crate::display::kernel_write_line("  [cr] CR0: begin");
        let mut f0 = Cr0::read();
        f0.insert(Cr0Flags::PROTECTED_MODE_ENABLE);
        f0.insert(Cr0Flags::PAGING);
        f0.insert(Cr0Flags::WRITE_PROTECT);
        f0.insert(Cr0Flags::ALIGNMENT_MASK);
        f0.insert(Cr0Flags::NUMERIC_ERROR);
        f0.remove(Cr0Flags::EMULATE_COPROCESSOR);
        f0.insert(Cr0Flags::MONITOR_COPROCESSOR);
        Cr0::write(f0);
        crate::display::kernel_write_line("  [cr] CR0: done");
    }

    // Configure CR4
    {
        use x86_64::registers::control::Cr4;
        crate::display::kernel_write_line("  [cr] CR4: begin");
        let _ = Cr4::read();
        crate::display::kernel_write_line("  [cr] CR4: skip modifications");
    }
}

/// Detect CPU features using CPUID
pub unsafe fn detect_cpu_features() -> CpuFeatures {
    let mut f = CpuFeatures::new();
    let cpuid = CpuId::new();
    if let Some(fi) = cpuid.get_feature_info() {
        f.sse = fi.has_sse();
        f.sse2 = fi.has_sse2();
        f.sse3 = fi.has_sse3();
        f.ssse3 = fi.has_ssse3();
        f.sse4_1 = fi.has_sse41();
        f.sse4_2 = fi.has_sse42();
        f.avx = fi.has_avx();
        f.xsave = fi.has_xsave();
        // Some CPUs report OSXSAVE only after CR4.OSXSAVE; treat as false at detect time
        f.osxsave = false;
        f.fma = fi.has_fma();
    }
    if let Some(efi) = cpuid.get_extended_feature_info() {
        f.avx2 = efi.has_avx2();
    }
    f
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
    {
        use x86_64::registers::control::{Cr0, Cr0Flags};
        let mut v = Cr0::read();
        v.remove(Cr0Flags::EMULATE_COPROCESSOR);
        v.insert(Cr0Flags::MONITOR_COPROCESSOR);
        Cr0::write(v);
    }

    // Set CR4.OSFXSR = 1 and CR4.OSXMMEXCPT = 1
    {
        use x86_64::registers::control::{Cr4, Cr4Flags};
        let mut v = Cr4::read();
        v.insert(Cr4Flags::OSFXSR);
        v.insert(Cr4Flags::OSXMMEXCPT_ENABLE);
        Cr4::write(v);
    }
}

/// Enable AVX
unsafe fn enable_avx() {
    // Set CR4.OSXSAVE = 1
    {
        use x86_64::registers::control::{Cr4, Cr4Flags};
        let mut v = Cr4::read();
        v.insert(Cr4Flags::OSXSAVE);
        Cr4::write(v);
        // Set XCR0: enable x87, SSE, AVX
        use x86_64::registers::xcontrol::{XCr0, XCr0Flags};
        let mut x = XCr0::read();
        x.insert(XCr0Flags::X87);
        x.insert(XCr0Flags::SSE);
        x.insert(XCr0Flags::AVX);
        XCr0::write(x);
    }
}

// raw_cpuid replaces inline cpuid

/// Check if FSGSBASE is available
pub unsafe fn has_fsgsbase() -> bool {
    if let Some(efi) = CpuId::new().get_extended_feature_info() {
        return efi.has_fsgsbase();
    }
    false
}

/// Check if OSXSAVE is available
unsafe fn has_osxsave() -> bool {
    // Querying OSXSAVE via CPUID bit 27 is unreliable before CR4.OSXSAVE; treat as false
    false
}

/// Set up Model Specific Registers (MSRs)
pub unsafe fn setup_msrs() {
    // Set up EFER (Extended Feature Enable Register)
    {
        use x86_64::registers::model_specific::{Efer, EferFlags};
        let mut e = Efer::read();
        e.insert(EferFlags::SYSTEM_CALL_EXTENSIONS);
        unsafe { Efer::write(e); }
    }
}
