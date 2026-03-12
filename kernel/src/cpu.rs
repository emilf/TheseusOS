//! Module: cpu
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/boot-flow.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A1:-The-kernel-is-x86_64-no_std-code-using-the-x86-interrupt-ABI
//! - docs/axioms/arch-x86_64.md#A3:-Interrupt-delivery-is-APIC-based-during-kernel-bring-up-with-legacy-PIC-masked
//!
//! INVARIANTS:
//! - This module owns CPU feature detection and the low-level control-register / MSR setup used during kernel bring-up.
//! - Feature detection results cached here are shared boot-time/runtime assumptions for later subsystem initialization.
//! - Control-register setup is sequenced by the environment/bring-up path; this module does not decide global boot order on its own.
//!
//! SAFETY:
//! - CR0/CR4/MSR mutations are architectural state changes and must only occur in the intended bring-up sequence.
//! - CPUID-derived capability detection is input to policy, not proof that every advertised feature is safe to use immediately in every context.
//! - Comments must not imply features are enabled when the current code only detects or partially configures them.
//!
//! PROGRESS:
//! - docs/plans/interrupts-and-platform.md
//! - docs/plans/boot-flow.md
//!
//! CPU feature detection and low-level control-register/MSR setup.
//!
//! This module gathers CPU capability information and applies the small set of
//! control-register/MSR changes used during bring-up.

use crate::log_debug;
use raw_cpuid::CpuId;
use spin::Once;

/// Control register flags for CR0
pub const CR0_PE: u64 = 1 << 0; // Protected Mode Enable
pub const CR0_MP: u64 = 1 << 1; // Monitor Coprocessor
pub const CR0_EM: u64 = 1 << 2; // Emulation
pub const CR0_TS: u64 = 1 << 3; // Task Switched
pub const CR0_ET: u64 = 1 << 4; // Extension Type
pub const CR0_NE: u64 = 1 << 5; // Numeric Error
pub const CR0_WP: u64 = 1 << 16; // Write Protect
pub const CR0_AM: u64 = 1 << 18; // Alignment Mask
pub const CR0_NW: u64 = 1 << 29; // Not Writethrough
pub const CR0_CD: u64 = 1 << 30; // Cache Disable
pub const CR0_PG: u64 = 1 << 31; // Paging Enable

/// Control register flags for CR4
pub const CR4_VME: u64 = 1 << 0; // Virtual 8086 Mode Extensions
pub const CR4_PVI: u64 = 1 << 1; // Protected Mode Virtual Interrupts
pub const CR4_TSD: u64 = 1 << 2; // Time Stamp Disable
pub const CR4_DE: u64 = 1 << 3; // Debugging Extensions
pub const CR4_PSE: u64 = 1 << 4; // Page Size Extensions
pub const CR4_PAE: u64 = 1 << 5; // Physical Address Extension
pub const CR4_MCE: u64 = 1 << 6; // Machine Check Enable
pub const CR4_PGE: u64 = 1 << 7; // Page Global Enable
pub const CR4_PCE: u64 = 1 << 8; // Performance Counter Enable
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

/// Cached CPU feature flags used by bring-up code.
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
    /// Create a `CpuFeatures` value with every feature disabled.
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

/// Detect CPU features once during early initialization.
///
/// This caches CPUID-derived feature information for later bring-up steps.
pub unsafe fn detect_cpu_features_once_lowhalf() {
    let _ = CPU_FEATURES_ONCE.call_once(|| detect_cpu_features());
}

/// Get the cached CPU feature snapshot, if it has been populated.
pub fn get_cpu_features_once() -> Option<CpuFeatures> {
    CPU_FEATURES_ONCE.get().copied()
}

/// Apply the current CR0/CR4 setup used by kernel bring-up.
///
/// This performs the limited control-register changes the current environment path
/// expects before higher-half paging and later CPU setup steps.
pub unsafe fn setup_control_registers() {
    // Configure CR0
    {
        use x86_64::registers::control::{Cr0, Cr0Flags};
        log_debug!("CR0: begin");
        let mut f0 = Cr0::read();
        f0.insert(Cr0Flags::PROTECTED_MODE_ENABLE);
        // Do NOT enable CR0.PAGING here; CR3 must be loaded and page tables prepared
        // before setting the PAGING bit. Paging will be enabled after the new PML4
        // is activated (CR3 load) in the environment setup.
        f0.insert(Cr0Flags::WRITE_PROTECT);
        f0.insert(Cr0Flags::ALIGNMENT_MASK);
        f0.insert(Cr0Flags::NUMERIC_ERROR);
        f0.remove(Cr0Flags::EMULATE_COPROCESSOR);
        f0.insert(Cr0Flags::MONITOR_COPROCESSOR);
        Cr0::write(f0);
        log_debug!("CR0: done");
    }

    // Configure CR4
    {
        use x86_64::registers::control::Cr4;
        log_debug!("CR4: begin");
        let _ = Cr4::read();
        log_debug!("CR4: skip modifications");
    }
}

/// Detect CPU features with CPUID.
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

/// Enable the floating-point/SIMD state the current feature set supports.
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

/// Enable SSE state through the required CR0/CR4 bits.
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

/// Enable AVX state by updating `CR4.OSXSAVE` and `XCR0`.
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

/// Return whether the CPU reports FSGSBASE support.
pub unsafe fn has_fsgsbase() -> bool {
    if let Some(efi) = CpuId::new().get_extended_feature_info() {
        return efi.has_fsgsbase();
    }
    false
}

/// Apply the current boot-time MSR configuration.
///
/// Today this enables `EFER.SYSTEM_CALL_EXTENSIONS`.
pub unsafe fn setup_msrs() {
    // Set up EFER (Extended Feature Enable Register)
    {
        use x86_64::registers::model_specific::{Efer, EferFlags};
        let mut e = Efer::read();
        e.insert(EferFlags::SYSTEM_CALL_EXTENSIONS);
        unsafe {
            Efer::write(e);
        }
    }
}
