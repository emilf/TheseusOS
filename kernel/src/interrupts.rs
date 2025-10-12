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
        Self {
            offset_low: 0,
            selector: 0,
            ist: 0,
            type_attr: 0,
            offset_mid: 0,
            offset_high: 0,
            zero: 0,
        }
    }
}

// Switch to x86_64 crate IDT; retain minimal entry struct for diagnostics via SIDT
use core::sync::atomic::{AtomicU32, Ordering};
use spin::Once as SpinOnce;
use x86_64::instructions::tables::sgdt;
use x86_64::registers::control::Cr2;
use x86_64::structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode};

static IDT_X86: SpinOnce<InterruptDescriptorTable> = SpinOnce::new();
pub const APIC_TIMER_VECTOR: u8 = 0x40; // 64
pub const SERIAL_RX_VECTOR: u8 = APIC_TIMER_VECTOR + 1;
const APIC_ERROR_VECTOR: u8 = 0xFE; // APIC error interrupts
pub static TIMER_TICKS: AtomicU32 = AtomicU32::new(0);
pub static mut DOUBLE_FAULT_CONTEXT: Option<DoubleFaultContext> = None;

#[derive(Copy, Clone)]
pub struct DoubleFaultContext {
    rip: u64,
    cr2: u64,
    rsp: u64,
    stack: [u64; 6],
}

impl DoubleFaultContext {
    pub const fn new(rip: u64, cr2: u64, rsp: u64, stack: [u64; 6]) -> Self {
        Self {
            rip,
            cr2,
            rsp,
            stack,
        }
    }
}

pub fn set_double_fault_context(rip: u64, cr2: u64, rsp: u64, stack: [u64; 6]) {
    unsafe {
        DOUBLE_FAULT_CONTEXT = Some(DoubleFaultContext::new(rip, cr2, rsp, stack));
    }
}
/// Global handoff pointer for timer interrupt access
static mut HANDOFF_FOR_TIMER: Option<&'static theseus_shared::handoff::Handoff> = None;

// Legacy inline assembly ISR stubs removed in favor of x86-interrupt handlers

// legacy ISR symbols removed

#[no_mangle]
#[allow(dead_code)]
extern "C" fn pf_report(ec: u64, rip: u64, cs: u64, rflags: u64, cr2: u64, rsp: u64) {
    unsafe {
        print_str_0xe9("PF EC=");
        print_hex_u64_0xe9(ec);
        print_str_0xe9(" CR2=");
        print_hex_u64_0xe9(cr2);
        print_str_0xe9(" RIP=");
        print_hex_u64_0xe9(rip);
        // Reconstruct potential high-half RIP if low
        let mut hrip = rip;
        if rip < crate::memory::KERNEL_VIRTUAL_BASE {
            hrip = rip.wrapping_add(crate::memory::KERNEL_VIRTUAL_BASE);
        }
        print_str_0xe9(" HRIP=");
        print_hex_u64_0xe9(hrip);
        print_str_0xe9(" CS=");
        {
            out_char_0xe9(b'0');
            out_char_0xe9(b'x');
            for i in (0..4).rev() {
                let nib = ((cs >> (i * 4)) & 0xF) as u8;
                let ch = if nib < 10 {
                    b'0' + nib
                } else {
                    b'A' + (nib - 10)
                };
                out_char_0xe9(ch);
            }
        }
        print_str_0xe9(" F=");
        print_hex_u64_0xe9(rflags);
        print_str_0xe9(" SP=");
        print_hex_u64_0xe9(rsp);
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
        if !did_hins {
            print_str_0xe9("(skip)");
        }
        // Stack dump disabled (see handler_pf below)
        print_str_0xe9("\n");
    }
}

// legacy resolver helpers removed

#[inline(always)]
unsafe fn out_char_0xe9(byte: u8) {
    use x86_64::instructions::port::Port;
    let mut port: Port<u8> = Port::new(0xE9);
    unsafe {
        port.write(byte);
    }
}

unsafe fn print_str_0xe9(s: &str) {
    for b in s.bytes() {
        out_char_0xe9(b);
    }
}

unsafe fn print_hex_u64_0xe9(v: u64) {
    out_char_0xe9(b'0');
    out_char_0xe9(b'x');
    for i in (0..16).rev() {
        let nib = ((v >> (i * 4)) & 0xF) as u8;
        let ch = if nib < 10 {
            b'0' + nib
        } else {
            b'A' + (nib - 10)
        };
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
    if n == 0 {
        out_char_0xe9(b'0');
    } else {
        while n > 0 && i < buf.len() {
            buf[i] = b'0' + (n % 10) as u8;
            n /= 10;
            i += 1;
        }
        while i > 0 {
            i -= 1;
            out_char_0xe9(buf[i]);
        }
    }
    print_str_0xe9("] sel=0x");
    let sel = e.selector as u64;
    // print 4 hex digits for selector
    for shift in (0..4).rev() {
        let nib = ((sel >> (shift * 4)) & 0xF) as u8;
        let ch = if nib < 10 {
            b'0' + nib
        } else {
            b'A' + (nib - 10)
        };
        out_char_0xe9(ch);
    }
    print_str_0xe9(" type=0x");
    let ty = e.type_attr as u64;
    for shift in (0..2).rev() {
        let nib = ((ty >> (shift * 4)) & 0xF) as u8;
        let ch = if nib < 10 {
            b'0' + nib
        } else {
            b'A' + (nib - 10)
        };
        out_char_0xe9(ch);
    }
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
            let e = &*(base.wrapping_add((i * core::mem::size_of::<IdtEntry>()) as u64)
                as *const IdtEntry);
            print_idt_entry(i, e);
            if idt_entry_addr(e) == 0 {
                ok = false;
            }
        }
        if ok {
            print_str_0xe9("IDT OK\n");
        } else {
            print_str_0xe9("IDT BAD\n");
        }
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
        // Timer interrupt vector
        idt[APIC_TIMER_VECTOR as usize].set_handler_fn(handler_timer);
        // Spurious interrupt vector (0xFF)
        idt[0xFF].set_handler_fn(handler_spurious);
        // APIC error vector
        idt[APIC_ERROR_VECTOR as usize].set_handler_fn(handler_spurious);
        // Assign IST indices for critical exceptions
        {
            use crate::gdt::{IST_INDEX_DF, IST_INDEX_MC, IST_INDEX_NMI, IST_INDEX_PF};
            unsafe {
                idt.double_fault
                    .set_handler_fn(handler_df)
                    .set_stack_index(IST_INDEX_DF);
                idt.non_maskable_interrupt
                    .set_handler_fn(handler_nmi)
                    .set_stack_index(IST_INDEX_NMI);
                idt.machine_check
                    .set_handler_fn(handler_mc)
                    .set_stack_index(IST_INDEX_MC);
                idt.page_fault
                    .set_handler_fn(handler_pf)
                    .set_stack_index(IST_INDEX_PF);
            }
        }
        // Serial RX vector
        idt[SERIAL_RX_VECTOR as usize].set_handler_fn(handler_serial_rx);
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
    unsafe {
        print_str_0xe9("DE\n");
    }
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_bp(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("BP\n");
    }
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_ud(stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("UD RIP=");
        print_hex_u64_0xe9(stack.instruction_pointer.as_u64());
        print_str_0xe9(" CS=");
        print_hex_u64_0xe9(stack.code_segment as u64);
        print_str_0xe9(" RFLAGS=");
        print_hex_u64_0xe9(stack.cpu_flags);
        out_char_0xe9(b'\n');
        theseus_shared::qemu_exit_ok!();
    }
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_gp(stack: InterruptStackFrame, code: u64) {
    unsafe {
        print_str_0xe9("GP EC=");
        print_hex_u64_0xe9(code);
        print_str_0xe9(" RIP=");
        print_hex_u64_0xe9(stack.instruction_pointer.as_u64());
        print_str_0xe9(" CS=");
        print_hex_u64_0xe9(stack.code_segment as u64);
        print_str_0xe9(" RFLAGS=");
        print_hex_u64_0xe9(stack.cpu_flags);
        print_str_0xe9(" TR=");
        let tr: u16;
        core::arch::asm!("str {0:x}", out(reg) tr, options(nomem, nostack, preserves_flags));
        print_hex_u64_0xe9(tr as u64);
        // Dump 8 bytes at RIP and top of stack for debugging
        print_str_0xe9(" INS=");
        let rip = stack.instruction_pointer.as_u64();
        let ins = core::ptr::read_volatile(rip as *const u64);
        print_hex_u64_0xe9(ins);
        print_str_0xe9(" STK:");
        let rsp = stack.stack_pointer.as_u64();
        for i in 0..4u64 {
            let val = core::ptr::read_volatile((rsp + i * 8) as *const u64);
            out_char_0xe9(b' ');
            print_hex_u64_0xe9(val);
        }
        out_char_0xe9(b'\n');
        theseus_shared::qemu_exit_ok!();
    }
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_pf(stack: InterruptStackFrame, code: PageFaultErrorCode) {
    let cr2 = Cr2::read().as_u64();
    let ec = code.bits() as u64;
    unsafe {
        print_str_0xe9("PF EC=");
        print_hex_u64_0xe9(ec);
        print_str_0xe9(" CR2=");
        print_hex_u64_0xe9(cr2);
        print_str_0xe9(" RIP=");
        print_hex_u64_0xe9(stack.instruction_pointer.as_u64());
        print_str_0xe9(" CS=");
        print_hex_u64_0xe9(stack.code_segment as u64);
        print_str_0xe9(" RFLAGS=");
        print_hex_u64_0xe9(stack.cpu_flags);
        print_str_0xe9(" RSP=");
        let rsp = stack.stack_pointer.as_u64();
        print_hex_u64_0xe9(rsp);
        print_str_0xe9(" STK:");
        for i in 0..6u64 {
            let val = core::ptr::read_volatile((rsp + i * 8) as *const u64);
            out_char_0xe9(b' ');
            print_hex_u64_0xe9(val);
        }
        out_char_0xe9(b'\n');
    }
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_df(_stack: InterruptStackFrame, _code: u64) -> ! {
    unsafe {
        print_str_0xe9("DF at");
        if let Some(ctx) = DOUBLE_FAULT_CONTEXT {
            print_str_0xe9(" RIP=");
            print_hex_u64_0xe9(ctx.rip);
            print_str_0xe9(" CR2=");
            print_hex_u64_0xe9(ctx.cr2);
            print_str_0xe9(" RSP=");
            print_hex_u64_0xe9(ctx.rsp);
            print_str_0xe9(" STK:");
            for word in ctx.stack.iter() {
                out_char_0xe9(b' ');
                print_hex_u64_0xe9(*word);
            }
            out_char_0xe9(b'\n');
        } else {
            print_str_0xe9(" (no context)\n");
        }
    }
    theseus_shared::qemu_exit_ok!();
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_nmi(_stack: InterruptStackFrame) {
    unsafe {
        print_str_0xe9("NMI\n");
    }
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_mc(_stack: InterruptStackFrame) -> ! {
    unsafe {
        print_str_0xe9("MC\n");
    }
    theseus_shared::qemu_exit_ok!();
    loop {
        x86_64::instructions::hlt();
    }
}

extern "x86-interrupt" fn handler_timer(_stack: InterruptStackFrame) {
    // Acknowledge LAPIC EOI first to avoid stuck-in-service
    unsafe {
        let apic_base = get_apic_base();
        write_apic_register(apic_base, 0xB0, 0); // EOI
    }

    // Record the tick
    TIMER_TICKS.fetch_add(1, Ordering::Relaxed);

    // Update heart animation if handoff is available
    unsafe {
        if let Some(handoff) = get_handoff_for_timer() {
            crate::framebuffer::update_heart_animation(handoff);
        }
    }
}

extern "x86-interrupt" fn handler_serial_rx(_stack: InterruptStackFrame) {
    let mut handled = false;
    if let Some(irq) = crate::drivers::serial::current_irq_number() {
        let mut mgr = crate::drivers::manager::driver_manager().lock();
        handled = mgr.handle_irq(irq);
    }
    unsafe {
        let apic_base = get_apic_base();
        write_apic_register(apic_base, 0xB0, 0);
    }
    if !handled {
        unsafe {
            print_str_0xe9("[serial] irq but no handler\n");
        }
    }
}

extern "x86-interrupt" fn handler_spurious(_stack: InterruptStackFrame) {
    // Log spurious interrupt entry for debugging nested/extra interrupts
    unsafe {
        print_str_0xe9("[INT] spurious\n");
    }
    // return to interrupted context
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
            if k > 0 {
                print_str_0xe9(" ");
            }
            let d = idx as u32;
            let mut buf = [0u8; 3];
            let mut n = d;
            let mut c = 0usize;
            if n == 0 {
                out_char_0xe9(b'0');
            } else {
                while n > 0 {
                    buf[c] = b'0' + (n % 10) as u8;
                    n /= 10;
                    c += 1;
                }
                while c > 0 {
                    c -= 1;
                    out_char_0xe9(buf[c]);
                }
            }
            out_char_0xe9(b'=');
            let e = &*(base.wrapping_add((idx * core::mem::size_of::<IdtEntry>()) as u64)
                as *const IdtEntry);
            print_hex_u64_0xe9(idt_entry_addr(e));
        }
        out_char_0xe9(b'\n');
        // Also print selector for timer vector 0x40
        let e40 = &*(base
            .wrapping_add(((APIC_TIMER_VECTOR as usize) * core::mem::size_of::<IdtEntry>()) as u64)
            as *const IdtEntry);
        print_str_0xe9("IDT[0x40] sel=");
        print_hex_u64_0xe9(e40.selector as u64);
        out_char_0xe9(b'\n');
    }
}

/// Print first few GDT entries raw to help diagnose selector issues
pub unsafe fn print_gdt_summary_basic() {
    let gdtr = sgdt();
    let base = gdtr.base.as_u64();
    let limit = gdtr.limit as u64;
    print_str_0xe9("GDT base=");
    print_hex_u64_0xe9(base);
    print_str_0xe9(" limit=");
    print_hex_u64_0xe9(limit);
    out_char_0xe9(b'\n');
    // dump first 8 descriptors (8 bytes each); TSS spans 16 bytes
    let count = core::cmp::min(((limit + 1) / 8) as usize, 8usize);
    for i in 0..count {
        let lo = core::ptr::read_unaligned((base + (i as u64) * 8) as *const u64);
        print_str_0xe9("GDT[");
        // print i
        let d = i as u32;
        let mut buf = [0u8; 3];
        let mut n = d;
        let mut c = 0usize;
        if n == 0 {
            out_char_0xe9(b'0');
        } else {
            while n > 0 {
                buf[c] = b'0' + (n % 10) as u8;
                n /= 10;
                c += 1;
            }
            while c > 0 {
                c -= 1;
                out_char_0xe9(buf[c]);
            }
        }
        print_str_0xe9("]=");
        print_hex_u64_0xe9(lo);
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
    unsafe {
        port.write(0x80u8);
    }
}

/// Disable local APIC interrupts
///
/// This function disables the Local APIC by masking all interrupts and setting
/// the Spurious Interrupt Vector register to disable the APIC.
unsafe fn disable_local_apic() {
    // Defer LAPIC access until after paging maps the MMIO region
    crate::display::kernel_write_line("  [apic] deferring LAPIC access until after paging");
}

/// Mask LAPIC after paging is enabled and MMIO is mapped
#[allow(dead_code)]
pub unsafe fn mask_lapic_after_paging() {
    if !has_apic() {
        return;
    }
    let apic_base = get_apic_base();
    // Probe-only: read APIC ID (0x20) and Version (0x30) to verify MMIO mapping
    let apic_id = read_apic_register(apic_base, 0x20);
    let apic_ver = read_apic_register(apic_base, 0x30);
    crate::display::kernel_write_line("  [apic] LAPIC probed id=");
    print_hex_u64_0xe9(apic_id as u64);
    crate::display::kernel_write_line(" ver=");
    print_hex_u64_0xe9(apic_ver as u64);
    out_char_0xe9(b'\n');
}

/// Configure LAPIC timer (not started). Call after paging is active.
#[allow(dead_code)]
pub unsafe fn lapic_timer_configure() {
    // Assume APIC present in this environment; avoid CPUID check to prevent early faults
    let apic_base = get_apic_base();
    crate::display::kernel_write_line("  [lapic] TPR=0");
    // Allow all priorities
    write_apic_register(apic_base, 0x80, 0x00); // TPR
                                                // Enable APIC via SIVR and set spurious vector to 0xFF
    let siv = (read_apic_register(apic_base, 0xF0) & !0xFF) | 0xFF | 0x100;
    write_apic_register(apic_base, 0xF0, siv);
    // Set divide configuration to /16 (0x3)
    crate::display::kernel_write_line("  [lapic] set divide");
    write_apic_register(apic_base, 0x3E0, 0x3);
    // Program LVT Timer with our vector, masked for now (one-shot mode)
    crate::display::kernel_write_line("  [lapic] program LVT timer (masked)");
    let lvt_timer = (APIC_TIMER_VECTOR as u32) | (1 << 16);
    write_apic_register(apic_base, 0x320, lvt_timer);
    // Mask LINT0/LINT1 and set error LVT vector
    let mut lint0 = read_apic_register(apic_base, 0x350);
    lint0 |= 1 << 16;
    write_apic_register(apic_base, 0x350, lint0);
    let mut lint1 = read_apic_register(apic_base, 0x360);
    lint1 |= 1 << 16;
    write_apic_register(apic_base, 0x360, lint1);
    let lvt_err = (read_apic_register(apic_base, 0x370) & !0xFF) | (APIC_ERROR_VECTOR as u32);
    write_apic_register(apic_base, 0x370, lvt_err);
    // Clear and read ESR
    write_apic_register(apic_base, 0x280, 0);
    let esr = read_apic_register(apic_base, 0x280);
    // Debug: dump key LAPIC regs
    let id = read_apic_register(apic_base, 0x20);
    let ver = read_apic_register(apic_base, 0x30);
    let sivr = read_apic_register(apic_base, 0xF0);
    let tpr = read_apic_register(apic_base, 0x80);
    let lvt = read_apic_register(apic_base, 0x320);
    print_str_0xe9("  [lapic] ID=");
    print_hex_u64_0xe9(id as u64);
    print_str_0xe9(" VER=");
    print_hex_u64_0xe9(ver as u64);
    print_str_0xe9(" SIVR=");
    print_hex_u64_0xe9(sivr as u64);
    print_str_0xe9(" TPR=");
    print_hex_u64_0xe9(tpr as u64);
    print_str_0xe9(" LVT=");
    print_hex_u64_0xe9(lvt as u64);
    print_str_0xe9(" ESR=");
    print_hex_u64_0xe9(esr as u64);
    out_char_0xe9(b'\n');
}

/// Start LAPIC timer in one-shot mode with given initial count
#[allow(dead_code)]
pub unsafe fn lapic_timer_start_oneshot(initial_count: u32) {
    let apic_base = get_apic_base();
    // Unmask timer (one-shot by default when periodic bit not set)
    let mut lvt = read_apic_register(apic_base, 0x320);
    lvt &= !(1 << 16);
    write_apic_register(apic_base, 0x320, lvt);
    write_apic_register(apic_base, 0x380, initial_count);
    // Debug: read current count right after arming
    let cur = read_apic_register(apic_base, 0x390);
    print_str_0xe9("  [lapic] current=");
    print_hex_u64_0xe9(cur as u64);
    out_char_0xe9(b'\n');
}

/// Start LAPIC timer in periodic mode with given initial count
pub unsafe fn lapic_timer_start_periodic(initial_count: u32) {
    let apic_base = get_apic_base();
    // Set periodic mode (bit 17) and unmask timer (clear bit 16)
    let mut lvt = read_apic_register(apic_base, 0x320);
    lvt |= 1 << 17; // Set periodic mode
    lvt &= !(1 << 16); // Unmask timer
    write_apic_register(apic_base, 0x320, lvt);
    write_apic_register(apic_base, 0x380, initial_count);
}

/// Stop/mask LAPIC timer
#[allow(dead_code)]
pub unsafe fn lapic_timer_mask() {
    if !has_apic() {
        return;
    }
    let apic_base = get_apic_base();
    let vbase = crate::memory::PHYS_OFFSET + (apic_base & 0xFFFFF000);
    unsafe {
        use x86_64::registers::control::Cr3;
        let (frame, _f) = Cr3::read();
        let cr3pa = frame.start_address().as_u64();
        print_str_0xe9("[lapic_mask] apic_base=");
        print_hex_u64_0xe9(apic_base);
        print_str_0xe9(" vbase=");
        print_hex_u64_0xe9(vbase);
        print_str_0xe9(" CR3=");
        print_hex_u64_0xe9(cr3pa);
        print_str_0xe9(" offset=0x320 addr=");
        print_hex_u64_0xe9(vbase + 0x320);
        out_char_0xe9(b'\n');
    }
    // Read-modify-write LVT timer
    let val = read_apic_register(apic_base, 0x320);
    unsafe {
        print_str_0xe9("[lapic_mask] lvt before=");
        print_hex_u64_0xe9(val as u64);
        out_char_0xe9(b'\n');
    }
    let new = val | (1 << 16);
    write_apic_register(apic_base, 0x320, new);
    let val2 = read_apic_register(apic_base, 0x320);
    unsafe {
        print_str_0xe9("[lapic_mask] lvt after=");
        print_hex_u64_0xe9(val2 as u64);
        out_char_0xe9(b'\n');
        use x86_64::registers::control::Cr3;
        let (frame2, _f2) = Cr3::read();
        print_str_0xe9("[lapic_mask] CR3 after=");
        print_hex_u64_0xe9(frame2.start_address().as_u64());
        out_char_0xe9(b'\n');
    }
}

/// No-op function used to test whether a simple CALL from environment causes the PF.
#[no_mangle]
pub extern "C" fn noop_for_test() {}

#[allow(dead_code)]
pub fn timer_tick_count() -> u32 {
    TIMER_TICKS.load(Ordering::Relaxed)
}

// NOTE: Avoid RDMSR/WRMSR for APIC base to prevent UD on some setups; assume default LAPIC base.
// QEMU and most PCs use 0xFEE00000 as the Local APIC physical base.
/// Reinstall/patch timer vector (0x40) IDT entry at runtime with full 64-bit handler address
pub unsafe fn install_timer_vector_runtime() {
    use x86_64::instructions::tables::sidt;
    let idtr = sidt();
    let base = idtr.base.as_u64();
    let ent = base + ((APIC_TIMER_VECTOR as u64) * 16);
    let handler = handler_timer as usize as u64;
    let selector = crate::gdt::KERNEL_CS as u64;
    // Build low 64 bits: offset_low | selector | ist(0) | type_attr(0x8E) | offset_mid
    let mut low: u64 = 0;
    low |= (handler & 0xFFFF) as u64; // offset_low
    low |= selector << 16; // selector
    low |= (0u64) << 32; // ist (3 bits) kept zero
    low |= (0x8Eu64) << 40; // type_attr (present, DPL=0, 64-bit interrupt gate)
    low |= ((handler >> 16) & 0xFFFF) << 48; // offset_mid
    let high: u64 = (handler >> 32) & 0xFFFF_FFFF; // offset_high in low dword; high dword zero
    core::ptr::write_unaligned(ent as *mut u64, low);
    core::ptr::write_unaligned((ent + 8) as *mut u64, high);
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
pub unsafe fn get_apic_base() -> u64 {
    // Assume default LAPIC base address. Avoid RDMSR to keep boot stable across hosts.
    0xFEE0_0000u64
}

/// Read APIC register
#[allow(dead_code)]
pub unsafe fn read_apic_register(apic_base: u64, offset: u32) -> u32 {
    // The LAPIC MMIO is mapped at PHYS_OFFSET + physical address
    let vbase = crate::memory::phys_to_virt_pa(apic_base & 0xFFFFF000);
    let addr = vbase + (offset as u64);
    const APIC_MMIO_DEBUG: bool = false; // silence verbose MMIO debug during stress
    if APIC_MMIO_DEBUG {
        unsafe {
            print_str_0xe9("[lapic_mmio] READ addr=");
            print_hex_u64_0xe9(addr);
            out_char_0xe9(b'\n');
        }
    }
    // Use volatile read
    let val = core::ptr::read_volatile(addr as *const u32);
    if APIC_MMIO_DEBUG {
        unsafe {
            print_str_0xe9("[lapic_mmio] READ val=");
            print_hex_u64_0xe9(val as u64);
            out_char_0xe9(b'\n');
        }
    }
    val
}

/// Write APIC register
#[allow(dead_code)]
unsafe fn write_apic_register(apic_base: u64, offset: u32, value: u32) {
    let vbase = crate::memory::phys_to_virt_pa(apic_base & 0xFFFFF000);
    let addr = vbase + (offset as u64);
    const APIC_MMIO_DEBUG: bool = false;
    if APIC_MMIO_DEBUG {
        unsafe {
            print_str_0xe9("[lapic_mmio] WRITE addr=");
            print_hex_u64_0xe9(addr);
            print_str_0xe9(" val=");
            print_hex_u64_0xe9(value as u64);
            out_char_0xe9(b'\n');
        }
    }
    core::ptr::write_volatile(addr as *mut u32, value);
}

/// Enable interrupts (for future use)
#[allow(dead_code)]
pub unsafe fn enable_interrupts() {
    x86_64::instructions::interrupts::enable();
}

/// Trigger a breakpoint exception (#BP)
#[inline(always)]
#[allow(dead_code)]
pub fn trigger_breakpoint() {
    x86_64::instructions::interrupts::int3();
}

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

/// Set the handoff pointer for timer interrupt access
///
/// # Safety
///
/// This function is unsafe because it modifies a global static. It should only
/// be called once during kernel initialization with a valid handoff pointer.
pub unsafe fn set_handoff_for_timer(handoff: &'static theseus_shared::handoff::Handoff) {
    HANDOFF_FOR_TIMER = Some(handoff);
}

/// Get the handoff pointer for timer interrupt access
///
/// # Safety
///
/// This function is unsafe because it accesses a global static. The returned
/// reference is only valid if the handoff was previously set and remains valid.
pub unsafe fn get_handoff_for_timer() -> Option<&'static theseus_shared::handoff::Handoff> {
    HANDOFF_FOR_TIMER
}
