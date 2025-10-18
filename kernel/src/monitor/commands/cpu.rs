//! CPU inspection commands
//!
//! This module implements commands for inspecting CPU features and state:
//! - `cpuid`: Display CPUID information (vendor, brand, features)
//! - `msr`: Read/write Model-Specific Registers

use crate::monitor::parsing::parse_number;
use crate::monitor::Monitor;
use alloc::format;
use alloc::vec::Vec;

impl Monitor {
    /// Display CPUID information
    ///
    /// Shows detailed CPU information obtained via the CPUID instruction:
    /// - Vendor ID (Intel, AMD, etc.)
    /// - Brand string (full processor name)
    /// - Family, Model, and Stepping IDs
    /// - APIC ID and logical processor count
    /// - Cache line size
    /// - Standard CPU features (FPU, SSE, AVX, etc.)
    /// - Extended features (FSGSBASE, BMI, SMAP, SMEP, etc.)
    /// - 64-bit and architectural features (LM, NX, 1GiB pages, etc.)
    ///
    /// # Examples
    /// ```text
    /// cpuid             # Show all CPUID information
    /// ```
    ///
    /// # Note
    /// Uses the `raw_cpuid` crate to safely query CPUID leaves.
    pub(in crate::monitor) fn cmd_cpuid(&self) {
        self.writeln("CPUID Information:");

        use raw_cpuid::CpuId;
        let cpuid = CpuId::new();

        if let Some(vi) = cpuid.get_vendor_info() {
            self.writeln(&format!("  Vendor: {}", vi.as_str()));
        }

        if let Some(brand) = cpuid.get_processor_brand_string() {
            let text = brand.as_str().trim_end_matches(char::from(0));
            if !text.is_empty() {
                self.writeln(&format!("  Brand: {}", text));
            }
        }

        if let Some(fi) = cpuid.get_feature_info() {
            self.writeln(&format!(
                "  Family: 0x{:X}  Model: 0x{:X}  Stepping: 0x{:X}",
                fi.family_id(),
                fi.model_id(),
                fi.stepping_id()
            ));
            self.writeln(&format!(
                "  APIC ID: {}  Logical CPUs/package: {}",
                fi.initial_local_apic_id(),
                fi.max_logical_processor_ids()
            ));
            self.writeln(&format!(
                "  CLFLUSH line size: {} bytes",
                fi.cflush_cache_line_size() * 8
            ));

            // Build list of standard CPU features from CPUID leaf 1
            let mut standard = Vec::new();
            if fi.has_fpu() {
                standard.push("FPU");
            }
            if fi.has_vme() {
                standard.push("VME");
            }
            if fi.has_de() {
                standard.push("DE");
            }
            if fi.has_pse() {
                standard.push("PSE");
            }
            if fi.has_tsc() {
                standard.push("TSC");
            }
            if fi.has_msr() {
                standard.push("MSR");
            }
            if fi.has_pae() {
                standard.push("PAE");
            }
            if fi.has_mce() {
                standard.push("MCE");
            }
            if fi.has_cmpxchg8b() {
                standard.push("CMPXCHG8B");
            }
            if fi.has_apic() {
                standard.push("APIC");
            }
            if fi.has_sysenter_sysexit() {
                standard.push("SYSENTER/SYSEXIT");
            }
            if fi.has_mtrr() {
                standard.push("MTRR");
            }
            if fi.has_pge() {
                standard.push("PGE");
            }
            if fi.has_mca() {
                standard.push("MCA");
            }
            if fi.has_cmov() {
                standard.push("CMOV");
            }
            if fi.has_pat() {
                standard.push("PAT");
            }
            if fi.has_pse36() {
                standard.push("PSE36");
            }
            if fi.has_clflush() {
                standard.push("CLFLUSH");
            }
            if fi.has_mmx() {
                standard.push("MMX");
            }
            if fi.has_fxsave_fxstor() {
                standard.push("FXSAVE/FXRSTOR");
            }
            if fi.has_sse() {
                standard.push("SSE");
            }
            if fi.has_sse2() {
                standard.push("SSE2");
            }
            if fi.has_sse3() {
                standard.push("SSE3");
            }
            if fi.has_ssse3() {
                standard.push("SSSE3");
            }
            if fi.has_sse41() {
                standard.push("SSE4.1");
            }
            if fi.has_sse42() {
                standard.push("SSE4.2");
            }
            if fi.has_avx() {
                standard.push("AVX");
            }
            if fi.has_x2apic() {
                standard.push("x2APIC");
            }
            if fi.has_hypervisor() {
                standard.push("HYPERVISOR");
            }

            if standard.is_empty() {
                self.writeln("  Standard Features: (none)");
            } else {
                self.writeln(&format!("  Standard Features: {}", standard.join(", ")));
            }
        }

        // Extended features from CPUID leaf 7
        if let Some(ef) = cpuid.get_extended_feature_info() {
            let mut features = Vec::new();
            if ef.has_fsgsbase() {
                features.push("FSGSBASE");
            }
            if ef.has_bmi1() {
                features.push("BMI1");
            }
            if ef.has_bmi2() {
                features.push("BMI2");
            }
            if ef.has_avx2() {
                features.push("AVX2");
            }
            if ef.has_smap() {
                features.push("SMAP");
            }
            if ef.has_smep() {
                features.push("SMEP");
            }
            if ef.has_rep_movsb_stosb() {
                features.push("REP MOVSB/STOSB");
            }
            if ef.has_invpcid() {
                features.push("INVPCID");
            }
            if ef.has_rdseed() {
                features.push("RDSEED");
            }
            if ef.has_rtm() {
                features.push("RTM");
            }
            if !features.is_empty() {
                self.writeln(&format!(
                    "  Extended Features (leaf 7): {}",
                    features.join(", ")
                ));
            }
        }

        // Extended features from CPUID leaf 0x80000001 (AMD64 features)
        if let Some(ext) = cpuid.get_extended_processor_and_feature_identifiers() {
            let mut features = Vec::new();
            if ext.has_64bit_mode() {
                features.push("LM");
            }
            if ext.has_execute_disable() {
                features.push("NX");
            }
            if ext.has_1gib_pages() {
                features.push("1GiB pages");
            }
            if ext.has_rdtscp() {
                features.push("RDTSCP");
            }
            if ext.has_sse4a() {
                features.push("SSE4A");
            }
            if ext.has_prefetchw() {
                features.push("PREFETCHW");
            }
            if ext.has_lahf_sahf() {
                features.push("LAHF/SAHF");
            }
            if ext.has_syscall_sysret() {
                features.push("SYSCALL/SYSRET");
            }
            if ext.has_lzcnt() {
                features.push("LZCNT");
            }
            if ext.has_mmx_extensions() {
                features.push("MMXEXT");
            }
            if !features.is_empty() {
                self.writeln(&format!(
                    "  Extended Features (leaf 0x80000001): {}",
                    features.join(", ")
                ));
            }
        }
    }

    /// Read or write a model-specific register
    ///
    /// Accesses CPU Model-Specific Registers (MSRs) using RDMSR/WRMSR instructions.
    ///
    /// # Arguments
    /// * `args` - Command arguments:
    ///   - `r ADDRESS` or `ADDRESS`: Read MSR at address
    ///   - `w ADDRESS VALUE`: Write value to MSR at address
    ///
    /// # Examples
    /// ```text
    /// msr 0x1B                 # Read IA32_APIC_BASE MSR
    /// msr r 0x1B               # Explicit read
    /// msr w 0x1B 0xFEE00000    # Write to IA32_APIC_BASE (dangerous!)
    /// ```
    ///
    /// # Safety
    /// - Reading unknown MSRs may cause #GP fault
    /// - Writing MSRs can crash the system or corrupt hardware state
    /// - Some MSRs are read-only
    /// - Use with extreme caution!
    pub(in crate::monitor) fn cmd_msr(&self, args: &[&str]) {
        if args.is_empty() {
            self.writeln("Usage: msr [r|w] ADDRESS [VALUE]");
            self.writeln("  r ADDRESS          - Read MSR (default operation)");
            self.writeln("  w ADDRESS VALUE    - Write MSR");
            self.writeln("Examples:");
            self.writeln("  msr 0x1B                 # read IA32_APIC_BASE");
            self.writeln("  msr w 0x1B 0x00000000    # write new value");
            return;
        }

        enum MsrOp {
            Read(u32),
            Write(u32, u64),
        }

        let op = match args[0] {
            "r" | "read" => {
                if args.len() < 2 {
                    self.writeln("Missing MSR address");
                    return;
                }
                match parse_number(args[1]) {
                    Some(addr) => MsrOp::Read(addr as u32),
                    None => {
                        self.writeln("Invalid MSR address");
                        return;
                    }
                }
            }
            "w" | "write" => {
                if args.len() < 3 {
                    self.writeln("Usage: msr w ADDRESS VALUE");
                    return;
                }
                let addr = match parse_number(args[1]) {
                    Some(addr) => addr as u32,
                    None => {
                        self.writeln("Invalid MSR address");
                        return;
                    }
                };
                let value = match parse_number(args[2]) {
                    Some(val) => val,
                    None => {
                        self.writeln("Invalid MSR value");
                        return;
                    }
                };
                MsrOp::Write(addr, value)
            }
            _ => match parse_number(args[0]) {
                Some(addr) => MsrOp::Read(addr as u32),
                None => {
                    self.writeln("Invalid MSR address or operation");
                    return;
                }
            },
        };

        unsafe {
            match op {
                MsrOp::Read(addr) => {
                    let value = x86_64::registers::model_specific::Msr::new(addr).read();
                    self.writeln(&format!("MSR 0x{:X} = 0x{:016X}", addr, value));
                }
                MsrOp::Write(addr, value) => {
                    x86_64::registers::model_specific::Msr::new(addr).write(value);
                    self.writeln(&format!("MSR 0x{:X} <- 0x{:016X}", addr, value));
                }
            }
        }
    }
}
