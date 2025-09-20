//! Interrupt control module
//! 
//! This module provides functions to disable all interrupts including NMI,
//! and control various interrupt sources during kernel initialization.
//! 
//! The module provides:
//! - Interrupt Descriptor Table (IDT) setup
//! - Exception and interrupt handlers
//! - Interrupt disabling and masking
//! - APIC and PIC management
//! 
//! This is essential for kernel initialization to ensure a clean environment
//! before setting up proper interrupt handling.

/// Minimal IDT structures for exception handling (64-bit gate is 16 bytes)
/// 
/// This structure represents a single entry in the Interrupt Descriptor Table.
/// It contains the handler address, segment selector, and various flags.
#[repr(C, packed)]
#[derive(Copy, Clone)]
struct IdtEntry {
    offset_low: u16,
    selector: u16,
    ist: u8,
    type_attr: u8,
    offset_mid: u16,
    offset_high: u32,
    zero: u32,
}

impl IdtEntry {
    #[allow(dead_code)]
    const fn missing() -> Self {
        Self { offset_low: 0, selector: 0, ist: 0, type_attr: 0, offset_mid: 0, offset_high: 0, zero: 0 }
    }
}

// Switch to x86_64 crate IDT; retain minimal entry struct for diagnostics via SIDT
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode};
use x86_64::registers::control::Cr2;
use spin::Once as SpinOnce;

static IDT_X86: SpinOnce<InterruptDescriptorTable> = SpinOnce::new();

// Legacy inline assembly ISR stubs removed in favor of x86-interrupt handlers

// legacy ISR symbols removed

#[no_mangle]
#[allow(dead_code)]
extern "C" fn pf_report(ec: u64, rip: u64, cs: u64, rflags: u64, cr2: u64, rsp: u64) {
    unsafe {
        print_str_0xe9("PF EC="); print_hex_u64_0xe9(ec);
        print_str_0xe9(" CR2="); print_hex_u64_0xe9(cr2);
        print_str_0xe9(" RIP="); print_hex_u64_0xe9(rip);
        // Reconstruct potential high-half RIP if low
        let mut hrip = rip;
        if rip < crate::memory::KERNEL_VIRTUAL_BASE { hrip = rip.wrapping_add(crate::memory::KERNEL_VIRTUAL_BASE); }
        print_str_0xe9(" HRIP="); print_hex_u64_0xe9(hrip);
        print_str_0xe9(" CS=");  { out_char_0xe9(b'0'); out_char_0xe9(b'x'); for i in (0..4).rev() { let nib = ((cs >> (i * 4)) & 0xF) as u8; let ch = if nib < 10 { b'0'+nib } else { b'A'+(nib-10) }; out_char_0xe9(ch); } }
        print_str_0xe9(" F=");   print_hex_u64_0xe9(rflags);
        print_str_0xe9(" SP=");  print_hex_u64_0xe9(rsp);
        // Dump 8 bytes at RIP and at HRIP (guard HRIP against unmapped range)
        print_str_0xe9(" INS=");
        let q = core::ptr::read_volatile(rip as *const u64);
        print_hex_u64_0xe9(q);
        print_str_0xe9(" HINS=");
        let mut did_hins = false;
        // Only read HRIP when it lies within the loaded kernel image range
    let _hv_base = crate::memory::KERNEL_VIRTUAL_BASE;
        let h = &*(crate::handoff::handoff_phys_ptr() as *const theseus_shared::handoff::Handoff);
        let (img_base, img_size) = (h.kernel_virtual_base, h.kernel_image_size);
        let img_end = img_base.wrapping_add(img_size);
        if hrip >= img_base && hrip < img_end {
            let hq = core::ptr::read_volatile(hrip as *const u64);
            print_hex_u64_0xe9(hq);
            did_hins = true;
        }
        if !did_hins { print_str_0xe9("(skip)"); }
        // Dump top of fault stack (original RSP)
        print_str_0xe9(" STK:");
        for i in 0..6u64 {
            let val = core::ptr::read_volatile((rsp + i*8) as *const u64);
            out_char_0xe9(b' ');
            print_hex_u64_0xe9(val);
        }
        out_char_0xe9(b'\n');
    }
}

// legacy resolver helpers removed

#[inline(always)]
unsafe fn out_char_0xe9(byte: u8) {
    use x86_64::instructions::port::Port;
    let mut port: Port<u8> = Port::new(0xE9);
    unsafe { port.write(byte); }
}

unsafe fn print_str_0xe9(s: &str) {
    for b in s.bytes() { out_char_0xe9(b); }
}

unsafe fn print_hex_u64_0xe9(v: u64) {
    out_char_0xe9(b'0');
    out_char_0xe9(b'x');
    for i in (0..16).rev() {
        let nib = ((v >> (i * 4)) & 0xF) as u8;
        let ch = if nib < 10 { b'0' + nib } else { b'A' + (nib - 10) };
        out_char_0xe9(ch);
    }
}

/// Compute full 64-bit handler address from an `IdtEntry`
fn idt_entry_addr(e: &IdtEntry) -> u64 {
    (e.offset_low as u64) | ((e.offset_mid as u64) << 16) | ((e.offset_high as u64) << 32)
}

/// Print one IDT entry in hex: index, selector, type_attr, address
#[allow(dead_code)]
unsafe fn print_idt_entry(idx: usize, e: &IdtEntry) {
    print_str_0xe9("IDT[");
    // small decimal print
    let d = idx as u32;
    let mut buf = [0u8; 3];
    let mut n = d;
    let mut i = 0usize;
    if n == 0 { out_char_0xe9(b'0'); } else {
        while n > 0 && i < buf.len() { buf[i] = b'0' + (n % 10) as u8; n /= 10; i += 1; }
        while i > 0 { i -= 1; out_char_0xe9(buf[i]); }
    }
    print_str_0xe9("] sel=0x");
    let sel = e.selector as u64;
    // print 4 hex digits for selector
    for shift in (0..4).rev() { let nib = ((sel >> (shift * 4)) & 0xF) as u8; let ch = if nib < 10 { b'0'+nib } else { b'A'+(nib-10) }; out_char_0xe9(ch); }
    print_str_0xe9(" type=0x");
    let ty = e.type_attr as u64;
    for shift in (0..2).rev() { let nib = ((ty >> (shift * 4)) & 0xF) as u8; let ch = if nib < 10 { b'0'+nib } else { b'A'+(nib-10) }; out_char_0xe9(ch); }
    print_str_0xe9(" addr=");
    print_hex_u64_0xe9(idt_entry_addr(e));
    out_char_0xe9(b'\n');
}

/// Validate a small set of vectors and print results; returns true if all non-zero
#[allow(dead_code)]
pub unsafe fn validate_idt_basic() -> bool {
    let _indices = [0usize, 3, 6, 13, 14];
    let mut ok = true;
    {
        use x86_64::instructions::tables::sidt;
        let idtr = sidt();
        let base = idtr.base.as_u64();
        for &i in &_indices {
            let e = &*(base.wrapping_add((i * core::mem::size_of::<IdtEntry>()) as u64) as *const IdtEntry);
            print_idt_entry(i, e);
            if idt_entry_addr(e) == 0 { ok = false; }
        }
        if ok { print_str_0xe9("IDT OK\n"); } else { print_str_0xe9("IDT BAD\n"); }
        return ok;
    }
}

/// Set up a basic IDT with exception handlers
/// 
/// This function creates and loads the Interrupt Descriptor Table with handlers
/// for critical exceptions. It sets up handlers for divide error, breakpoint,
/// invalid opcode, general protection fault, page fault, double fault, NMI,
/// and machine check exceptions.
/// 
/// # Safety
/// 
/// This function modifies the IDT and should only be called during kernel
/// initialization when it's safe to set up interrupt handling.
pub unsafe fn setup_idt() {
    let idt = IDT_X86.call_once(|| {
        let mut idt = InterruptDescriptorTable::new();
        idt.divide_error.set_handler_fn(handler_de);
        idt.breakpoint.set_handler_fn(handler_bp);
        idt.invalid_opcode.set_handler_fn(handler_ud);
        idt.general_protection_fault.set_handler_fn(handler_gp);
        idt.page_fault.set_handler_fn(handler_pf);
        // Assign IST indices for critical exceptions
        {
            use crate::gdt::{IST_INDEX_DF, IST_INDEX_NMI, IST_INDEX_MC};
            unsafe {
                idt.double_fault.set_handler_fn(handler_df).set_stack_index(IST_INDEX_DF);
                idt.non_maskable_interrupt.set_handler_fn(handler_nmi).set_stack_index(IST_INDEX_NMI);
                idt.machine_check.set_handler_fn(handler_mc).set_stack_index(IST_INDEX_MC);
            }
        }
        idt
    });
    idt.load();
    return;

    /* legacy inline-assembly writer (disabled)
        // Helper: write descriptor at [rdi] using handler address in rax and selector in rsi
        // rdi: entry ptr, rax: handler addr, rsi: selector
        // --- Install #DE (vector 0) ---
    */
}

// x86-interrupt Rust handlers
extern "x86-interrupt" fn handler_de(_stack: InterruptStackFrame) {
    unsafe { print_str_0xe9("DE\n"); }
    loop { x86_64::instructions::hlt(); }
}

extern "x86-interrupt" fn handler_bp(_stack: InterruptStackFrame) {
    unsafe { print_str_0xe9("BP\n"); }
    loop { x86_64::instructions::hlt(); }
}

extern "x86-interrupt" fn handler_ud(_stack: InterruptStackFrame) {
    unsafe { print_str_0xe9("UD\n"); }
    loop { x86_64::instructions::hlt(); }
}

extern "x86-interrupt" fn handler_gp(_stack: InterruptStackFrame, code: u64) {
    unsafe {
        print_str_0xe9("GP EC=");
        print_hex_u64_0xe9(code);
        out_char_0xe9(b'\n');
    }
    loop { x86_64::instructions::hlt(); }
}

extern "x86-interrupt" fn handler_pf(_stack: InterruptStackFrame, code: PageFaultErrorCode) {
    let cr2 = Cr2::read().as_u64();
    let ec = code.bits() as u64;
    unsafe {
        print_str_0xe9("PF EC="); print_hex_u64_0xe9(ec);
        print_str_0xe9(" CR2="); print_hex_u64_0xe9(cr2);
        out_char_0xe9(b'\n');
    }
    loop { x86_64::instructions::hlt(); }
}

extern "x86-interrupt" fn handler_df(_stack: InterruptStackFrame, _code: u64) -> ! {
    unsafe { print_str_0xe9("DF\n"); }
    loop { x86_64::instructions::hlt(); }
}

extern "x86-interrupt" fn handler_nmi(_stack: InterruptStackFrame) {
    unsafe { print_str_0xe9("NMI\n"); }
    loop { x86_64::instructions::hlt(); }
}

extern "x86-interrupt" fn handler_mc(_stack: InterruptStackFrame) -> ! {
    unsafe { print_str_0xe9("MC\n"); }
    loop { x86_64::instructions::hlt(); }
}

/// Print a compact one-line summary of key IDT handler addresses
pub unsafe fn print_idt_summary_compact() {
    let indices = [0usize, 3, 6, 13, 14];
    {
        print_str_0xe9("IDT addrs: ");
        use x86_64::instructions::tables::sidt;
        let idtr = sidt();
        let base = idtr.base.as_u64();
        for (k, &idx) in indices.iter().enumerate() {
            if k > 0 { print_str_0xe9(" "); }
            let d = idx as u32; let mut buf = [0u8;3]; let mut n=d; let mut c=0usize;
            if n==0 { out_char_0xe9(b'0'); } else { while n>0 { buf[c]=b'0'+(n%10) as u8; n/=10; c+=1; } while c>0 { c-=1; out_char_0xe9(buf[c]); } }
            out_char_0xe9(b'=');
            let e = &*(base.wrapping_add((idx * core::mem::size_of::<IdtEntry>()) as u64) as *const IdtEntry);
            print_hex_u64_0xe9(idt_entry_addr(e));
        }
        out_char_0xe9(b'\n');
    }
}

/// Disable all interrupts including NMI
/// 
/// This function comprehensively disables all interrupt sources to ensure
/// a clean environment during kernel initialization. It disables regular
/// interrupts, NMI, local APIC interrupts, and masks all PIC interrupts.
/// 
/// # Safety
/// 
/// This function modifies interrupt control registers and should only be called
/// during kernel initialization when it's safe to disable all interrupts.
pub unsafe fn disable_all_interrupts() {
    // Disable regular interrupts (CLI)
    x86_64::instructions::interrupts::disable();
    
    // Disable NMI
    disable_nmi();
    
    // Disable local APIC interrupts
    disable_local_apic();
    
    // Mask all PIC interrupts
    mask_pic_interrupts();
}

/// Disable Non-Maskable Interrupts (NMI)
unsafe fn disable_nmi() {
    // Write to CMOS index port with NMI disable bit
    use x86_64::instructions::port::Port;
    let mut port: Port<u8> = Port::new(0x70);
    unsafe { port.write(0x80u8); }
}

/// Disable local APIC interrupts
/// 
/// Currently disabled as LAPIC MMIO region (0xFEE0_0000) needs to be mapped
/// before we can access it. This is a placeholder for future implementation.
unsafe fn disable_local_apic() {
    // Note: LAPIC MMIO region (0xFEE0_0000) needs to be mapped before accessing
    // For now, we skip APIC configuration during early kernel initialization
    return;
}

/// Mask all PIC interrupts
unsafe fn mask_pic_interrupts() {
    use x86_64::instructions::port::Port;
    let mut master: Port<u8> = Port::new(0x21);
    let mut slave: Port<u8> = Port::new(0xA1);
    unsafe {
        master.write(0xFFu8);
        slave.write(0xFFu8);
    }
}

/// Check if APIC is available
#[allow(dead_code)]
unsafe fn has_apic() -> bool {
    if let Some(fi) = raw_cpuid::CpuId::new().get_feature_info() {
        return fi.has_apic();
    }
    false
}

/// Get APIC base address from IA32_APIC_BASE MSR
/// Get the Local APIC base address from the IA32_APIC_BASE MSR
/// 
/// This function reads the Model Specific Register (MSR) that contains the
/// physical base address of the Local APIC (Advanced Programmable Interrupt Controller).
/// The APIC is used for interrupt handling in modern x86-64 systems.
/// 
/// # Assembly Details
/// - `rdmsr`: Read Model Specific Register instruction
/// - `ecx` = 0x1B: IA32_APIC_BASE MSR number
/// - `eax` and `edx`: 64-bit value returned in two 32-bit registers
/// 
/// # Returns
/// 
/// The 64-bit physical base address of the Local APIC
/// 
/// # Safety
/// 
/// This function is unsafe because it:
/// - Executes privileged MSR read instruction
/// - Assumes the CPU supports the APIC_BASE MSR
/// 
/// The caller must ensure:
/// - The CPU supports the IA32_APIC_BASE MSR
/// - We are running in kernel mode with sufficient privileges
/// - The MSR is accessible and not corrupted
#[allow(dead_code)]
unsafe fn get_apic_base() -> u64 {
    let mut eax: u32;
    let mut edx: u32;
    
    core::arch::asm!(
        "rdmsr",                    // Read Model Specific Register
        in("ecx") 0x1Bu32,         // IA32_APIC_BASE MSR number
        out("eax") eax,            // Lower 32 bits of result
        out("edx") edx,            // Upper 32 bits of result
        options(nomem, nostack, preserves_flags)
    );
    
    // Combine the two 32-bit values into a 64-bit address
    ((edx as u64) << 32) | (eax as u64)
}

/// Read APIC register
#[allow(dead_code)]
unsafe fn read_apic_register(apic_base: u64, offset: u32) -> u32 {
    let addr = (apic_base & 0xFFFFF000) | (offset as u64);
    // Use volatile read
    core::ptr::read_volatile(addr as *const u32)
}

/// Write APIC register
#[allow(dead_code)]
unsafe fn write_apic_register(apic_base: u64, offset: u32, value: u32) {
    let addr = (apic_base & 0xFFFFF000) | (offset as u64);
    core::ptr::write_volatile(addr as *mut u32, value);
}

/// Enable interrupts (for future use)
#[allow(dead_code)]
pub unsafe fn enable_interrupts() { x86_64::instructions::interrupts::enable(); }

/// Trigger a breakpoint exception (#BP)
#[inline(always)]
#[allow(dead_code)]
pub fn trigger_breakpoint() { x86_64::instructions::interrupts::int3(); }

/// Check if interrupts are currently enabled
/// 
/// This function reads the CPU flags register to determine if interrupts
/// are enabled. It's useful for debugging and ensuring proper interrupt state.
/// 
/// # Assembly Details
/// - `pushfq`: Push the RFLAGS register onto the stack
/// - `pop {flags}`: Pop the flags into a general-purpose register
/// - The Interrupt Flag (IF) is bit 9 (0x200) in the RFLAGS register
/// 
/// # Returns
/// 
/// * `true` - If interrupts are enabled (IF flag is set)
/// * `false` - If interrupts are disabled (IF flag is clear)
/// 
/// # Safety
/// 
/// This function is safe because it only reads the flags register without
/// modifying any system state. However, the result may be stale if interrupts
/// are enabled/disabled between the read and use of the result.
#[allow(dead_code)]
pub fn interrupts_enabled() -> bool {
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "pushfq",        // Push RFLAGS register onto stack
            "pop {}",        // Pop flags into general-purpose register
            out(reg) flags,
            options(nomem, nostack, preserves_flags)
        );
    }
    (flags & 0x200) != 0 // Check bit 9 (IF flag) in RFLAGS
}
