//! Module: cpu_features
//!
//! SOURCE OF TRUTH:
//! - docs/plans/interrupts-and-platform.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/arch-x86_64.md#A1:-The-kernel-is-x86_64-no_std-code-using-the-x86-interrupt-ABI
//!
//! INVARIANTS:
//! - `CpuFeatures::detect()` populates a global once-cell with CPUID-derived feature flags.
//! - `CpuFeatures::get()` returns the cached result; panics if `detect()` was never called.
//! - SMEP/SMAP are enabled in CR4 when the CPU reports support.
//!
//! SAFETY:
//! - CPUID is observational. CR4 writes are architectural state changes.
//! - This module must be called exactly once during early boot before APIC init.
//!
//! Centralized CPU feature detection and CR4 hardening.

use raw_cpuid::CpuId;
use spin::Once;

/// Cached CPU feature flags detected via CPUID.
#[derive(Debug, Clone, Copy)]
pub struct CpuFeatures {
    // Standard features (leaf 1 ECX/EDX)
    pub sse: bool,
    pub sse2: bool,
    pub sse3: bool,
    pub ssse3: bool,
    pub sse4_1: bool,
    pub sse4_2: bool,
    pub avx: bool,
    pub fma: bool,
    pub xsave: bool,
    pub x2apic: bool,
    pub tsc_deadline: bool,
    pub apic: bool,

    // Extended features (leaf 7 EBX/ECX)
    pub avx2: bool,
    pub fsgsbase: bool,
    pub smep: bool,
    pub smap: bool,

    // Extended processor features (leaf 0x80000001)
    pub rdtscp: bool,
}

impl CpuFeatures {
    /// Detect CPU features by executing CPUID.
    pub fn detect() -> Self {
        let cpuid = CpuId::new();
        let mut f = Self {
            sse: false,
            sse2: false,
            sse3: false,
            ssse3: false,
            sse4_1: false,
            sse4_2: false,
            avx: false,
            fma: false,
            xsave: false,
            x2apic: false,
            tsc_deadline: false,
            apic: false,
            avx2: false,
            fsgsbase: false,
            smep: false,
            smap: false,
            rdtscp: false,
        };

        if let Some(fi) = cpuid.get_feature_info() {
            f.sse = fi.has_sse();
            f.sse2 = fi.has_sse2();
            f.sse3 = fi.has_sse3();
            f.ssse3 = fi.has_ssse3();
            f.sse4_1 = fi.has_sse41();
            f.sse4_2 = fi.has_sse42();
            f.avx = fi.has_avx();
            f.fma = fi.has_fma();
            f.xsave = fi.has_xsave();
            f.x2apic = fi.has_x2apic();
            f.tsc_deadline = fi.has_tsc_deadline();
            f.apic = fi.has_apic();
        }

        if let Some(efi) = cpuid.get_extended_feature_info() {
            f.avx2 = efi.has_avx2();
            f.fsgsbase = efi.has_fsgsbase();
            f.smep = efi.has_smep();
            f.smap = efi.has_smap();
        }

        if let Some(ext) = cpuid.get_extended_processor_and_feature_identifiers() {
            f.rdtscp = ext.has_rdtscp();
        }

        f
    }

    /// Store the detected features in the global once-cell and apply CR4 hardening.
    ///
    /// Call this once during early boot before APIC init.
    pub fn init() {
        let features = Self::detect();
        CPU_FEATURES.call_once(|| features);

        // Enable SMEP/SMAP in CR4 if CPU supports them
        unsafe {
            Self::apply_cr4_hardening(&features);
        }
    }

    /// Get the cached feature flags. Panics if `init()` was never called.
    pub fn get() -> &'static Self {
        CPU_FEATURES
            .get()
            .expect("CpuFeatures::init() must be called before CpuFeatures::get()")
    }

    /// Enable SMEP and SMAP in CR4 when available.
    ///
    /// # Safety
    /// Writes to CR4 are architectural state changes. Must be called during boot.
    unsafe fn apply_cr4_hardening(features: &CpuFeatures) {
        use x86_64::registers::control::{Cr4, Cr4Flags};

        let mut cr4 = Cr4::read();

        if features.smep {
            cr4.insert(Cr4Flags::SUPERVISOR_MODE_EXECUTION_PROTECTION);
            crate::log_debug!("CR4: enabling SMEP");
        }

        if features.smap {
            cr4.insert(Cr4Flags::SUPERVISOR_MODE_ACCESS_PREVENTION);
            crate::log_debug!("CR4: enabling SMAP");
        }

        if features.fsgsbase {
            cr4.insert(Cr4Flags::FSGSBASE);
            crate::log_debug!("CR4: enabling FSGSBASE");
        }

        Cr4::write(cr4);
    }
}

static CPU_FEATURES: Once<CpuFeatures> = Once::new();
