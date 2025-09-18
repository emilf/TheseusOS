//! Interrupt control module
//! 
//! This module provides functions to disable all interrupts including NMI,
//! and control various interrupt sources during kernel initialization.

/// Minimal IDT structures for exception handling (64-bit gate is 16 bytes)
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
    const fn missing() -> Self {
        Self { offset_low: 0, selector: 0, ist: 0, type_attr: 0, offset_mid: 0, offset_high: 0, zero: 0 }
    }

    fn set_handler_addr(&mut self, addr: u64) {
        self.offset_low = addr as u16;
        self.selector = super::gdt::KERNEL_CS as u16;
        self.ist = 0; // no IST
        self.type_attr = 0x8E; // present=1, DPL=0, type=0xE (interrupt gate)
        self.offset_mid = (addr >> 16) as u16;
        self.offset_high = (addr >> 32) as u32;
        self.zero = 0;
    }
}

#[repr(C, packed)]
struct IdtPointer {
    limit: u16,
    base: u64,
}

static mut IDT: [IdtEntry; 256] = [const { IdtEntry::missing() }; 256];

core::arch::global_asm!(r#"
.globl isr_de_stub
isr_de_stub:
    mov dx, 0xE9
    mov al, 'D'
    out dx, al
    mov al, 'E'
    out dx, al
    mov al, 0x0A
    out dx, al
    mov dx, 0xF4
    mov al, 0x01
    out dx, al
    cli
.Lhang_de:
    hlt
    jmp .Lhang_de

.globl isr_gp_stub
isr_gp_stub:
    mov dx, 0xE9
    mov al, 'G'
    out dx, al
    mov al, 'P'
    out dx, al
    mov al, ' '
    out dx, al
    mov al, 'E'
    out dx, al
    mov al, 'C'
    out dx, al
    mov al, '='
    out dx, al
    mov rax, [rsp]       // error code pushed by CPU
    mov rcx, 16
.Lgp_hex_loop:
    mov rbx, rax
    shr rbx, 60
    and bl, 0x0F
    cmp bl, 9
    jbe .Lgp_digit_num
    add bl, 'A' - 10
    jmp .Lgp_emit
.Lgp_digit_num:
    add bl, '0'
.Lgp_emit:
    mov al, bl
    out dx, al
    shl rax, 4
    loop .Lgp_hex_loop
    mov al, 0x0A
    out dx, al
    mov dx, 0xF4
    mov al, 0x01
    out dx, al
    cli
.Lhang_gp:
    hlt
    jmp .Lhang_gp

.globl isr_pf_stub
isr_pf_stub:
    mov dx, 0xE9
    mov al, 'P'
    out dx, al
    mov al, 'F'
    out dx, al
    mov al, 0x0A
    out dx, al
    cli
.Lhang_pf:
    hlt
    jmp .Lhang_pf

.globl isr_ud_stub
isr_ud_stub:
    mov dx, 0xE9
    mov al, 'U'
    out dx, al
    mov al, 'D'
    out dx, al
    mov al, 0x0A
    out dx, al
    cli
.Lhang_ud:
    hlt
    jmp .Lhang_ud
# breakpoint
.globl isr_bp_stub
isr_bp_stub:
    mov dx, 0xE9
    mov al, 'B'
    out dx, al
    mov al, 'P'
    out dx, al
    mov al, 0x0A
    out dx, al
    cli
.Lhang_bp:
    hlt
    jmp .Lhang_bp
"#);

extern "C" {
    fn isr_de_stub();
    fn isr_gp_stub();
    fn isr_pf_stub();
    fn isr_ud_stub();
    fn isr_bp_stub();
}

#[inline(always)]
unsafe fn out_char_0xe9(byte: u8) {
    core::arch::asm!(
        "out dx, al",
        in("dx") 0xE9u16,
        in("al") byte,
        options(nomem, nostack, preserves_flags)
    );
}

unsafe fn print_str_0xe9(s: &str) {
    for b in s.bytes() { out_char_0xe9(b); }
}

unsafe fn print_hex_u64_0xe9(mut v: u64) {
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
pub unsafe fn validate_idt_basic() -> bool {
    let indices = [0usize, 3, 6, 13, 14];
    let mut ok = true;
    for &i in &indices {
        let e = &*(&IDT[i] as *const IdtEntry);
        print_idt_entry(i, e);
        if idt_entry_addr(e) == 0 { ok = false; }
    }
    if ok { print_str_0xe9("IDT OK\n"); } else { print_str_0xe9("IDT BAD\n"); }
    ok
}

/// Set up a basic IDT with exception handlers
pub unsafe fn setup_idt() {
    // Install handlers by computing addresses of inline labels at runtime (no relocations)
    let ent0 = &IDT[0] as *const IdtEntry as u64 as *mut u8;
    let ent3 = &IDT[3] as *const IdtEntry as u64 as *mut u8;
    let ent6 = &IDT[6] as *const IdtEntry as u64 as *mut u8;
    let ent13 = &IDT[13] as *const IdtEntry as u64 as *mut u8;
    let ent14 = &IDT[14] as *const IdtEntry as u64 as *mut u8;
    let sel: u64 = super::gdt::KERNEL_CS as u64;

    unsafe { core::arch::asm!(
        // Helper: write descriptor at [rdi] using handler address in rax and selector in rsi
        // rdi: entry ptr, rax: handler addr, rsi: selector
        // --- Install #DE (vector 0) ---
        "lea rax, [rip + 2f]",
        "mov rdi, {ent0}",
        "mov rsi, {sel}",
        // write gate
        "mov dx, ax",
        "shr rax, 16",
        "mov cx, ax",
        "shr rax, 16",
        "mov r8d, eax",
        "mov word ptr [rdi + 0], dx",
        "mov word ptr [rdi + 2], si",
        "mov byte ptr [rdi + 4], 0",
        "mov byte ptr [rdi + 5], 0x8E",
        "mov word ptr [rdi + 6], cx",
        "mov dword ptr [rdi + 8], r8d",
        "mov dword ptr [rdi + 12], 0",
        "jmp 3f",
        // handler body for #DE
        "2:",
        "mov dx, 0xE9",
        "mov al, 'D'",
        "out dx, al",
        "mov al, 'E'",
        "out dx, al",
        "mov al, 0x0A",
        "out dx, al",
        // exit QEMU
        "mov dx, 0xF4",
        "mov al, 0x01",
        "out dx, al",
        "cli",
        "hlt",
        "jmp 2b",
        "3:",

        // --- Install #BP (vector 3) ---
        "lea rax, [rip + 4f]",
        "mov rdi, {ent3}",
        "mov rsi, {sel}",
        "mov dx, ax",
        "shr rax, 16",
        "mov cx, ax",
        "shr rax, 16",
        "mov r8d, eax",
        "mov word ptr [rdi + 0], dx",
        "mov word ptr [rdi + 2], si",
        "mov byte ptr [rdi + 4], 0",
        "mov byte ptr [rdi + 5], 0x8E",
        "mov word ptr [rdi + 6], cx",
        "mov dword ptr [rdi + 8], r8d",
        "mov dword ptr [rdi + 12], 0",
        "jmp 5f",
        // handler body for #BP
        "4:",
        "mov dx, 0xE9",
        "mov al, 'B'",
        "out dx, al",
        "mov al, 'P'",
        "out dx, al",
        "mov al, 0x0A",
        "out dx, al",
        // exit QEMU
        "mov dx, 0xF4",
        "mov al, 0x01",
        "out dx, al",
        "cli",
        "hlt",
        "jmp 4b",
        "5:",

        // --- Install #UD (vector 6) ---
        "lea rax, [rip + 6f]",
        "mov rdi, {ent6}",
        "mov rsi, {sel}",
        "mov dx, ax",
        "shr rax, 16",
        "mov cx, ax",
        "shr rax, 16",
        "mov r8d, eax",
        "mov word ptr [rdi + 0], dx",
        "mov word ptr [rdi + 2], si",
        "mov byte ptr [rdi + 4], 0",
        "mov byte ptr [rdi + 5], 0x8E",
        "mov word ptr [rdi + 6], cx",
        "mov dword ptr [rdi + 8], r8d",
        "mov dword ptr [rdi + 12], 0",
        "jmp 7f",
        // handler body for #UD
        "6:",
        "mov dx, 0xE9",
        "mov al, 'U'",
        "out dx, al",
        "mov al, 'D'",
        "out dx, al",
        "mov al, 0x0A",
        "out dx, al",
        // exit QEMU
        "mov dx, 0xF4",
        "mov al, 0x01",
        "out dx, al",
        "cli",
        "hlt",
        "jmp 6b",
        "7:",

        // --- Install #GP (vector 13) ---
        "lea rax, [rip + 8f]",
        "mov rdi, {ent13}",
        "mov rsi, {sel}",
        "mov dx, ax",
        "shr rax, 16",
        "mov cx, ax",
        "shr rax, 16",
        "mov r8d, eax",
        "mov word ptr [rdi + 0], dx",
        "mov word ptr [rdi + 2], si",
        "mov byte ptr [rdi + 4], 0",
        "mov byte ptr [rdi + 5], 0x8E",
        "mov word ptr [rdi + 6], cx",
        "mov dword ptr [rdi + 8], r8d",
        "mov dword ptr [rdi + 12], 0",
        "jmp 9f",
        // handler body for #GP
        "8:",
        "mov dx, 0xE9",
        "mov al, 'G'",
        "out dx, al",
        "mov al, 'P'",
        "out dx, al",
        "mov al, 0x0A",
        "out dx, al",
        // exit QEMU
        "mov dx, 0xF4",
        "mov al, 0x01",
        "out dx, al",
        "cli",
        "hlt",
        "jmp 8b",
        "9:",

        // --- Install #PF (vector 14) ---
        "lea rax, [rip + 12f]",
        "mov rdi, {ent14}",
        "mov rsi, {sel}",
        "mov dx, ax",
        "shr rax, 16",
        "mov cx, ax",
        "shr rax, 16",
        "mov r8d, eax",
        "mov word ptr [rdi + 0], dx",
        "mov word ptr [rdi + 2], si",
        "mov byte ptr [rdi + 4], 0",
        "mov byte ptr [rdi + 5], 0x8E",
        "mov word ptr [rdi + 6], cx",
        "mov dword ptr [rdi + 8], r8d",
        "mov dword ptr [rdi + 12], 0",
        "jmp 13f",
        // handler body for #PF
        "12:",
        "mov dx, 0xE9",
        "mov al, 'P'",
        "out dx, al",
        "mov al, 'F'",
        "out dx, al",
        "mov al, 0x0A",
        "out dx, al",
        // exit QEMU
        "mov dx, 0xF4",
        "mov al, 0x01",
        "out dx, al",
        "cli",
        "hlt",
        "jmp 12b",
        "13:",

        ent0 = in(reg) ent0,
        ent3 = in(reg) ent3,
        ent6 = in(reg) ent6,
        ent13 = in(reg) ent13,
        ent14 = in(reg) ent14,
        sel = in(reg) sel,
        out("rax") _, out("rdx") _, out("rcx") _, out("r8d") _, out("rdi") _, out("rsi") _,
        options()
    ); }

    let idt_ptr = IdtPointer {
        limit: (core::mem::size_of_val(&IDT) - 1) as u16,
        base: &IDT as *const _ as u64,
    };
    core::arch::asm!("lidt [{}]", in(reg) &idt_ptr, options(readonly, nostack, preserves_flags));

    // Print and validate a few entries before testing exceptions
    let _ = validate_idt_basic();
}

// ISR stubs implemented in global assembly above; no Rust bodies needed

/// Disable all interrupts including NMI
pub unsafe fn disable_all_interrupts() {
    // Disable regular interrupts (CLI)
    core::arch::asm!("cli", options(nomem, nostack, preserves_flags));
    
    // Disable NMI
    disable_nmi();
    
    // Disable local APIC interrupts
    disable_local_apic();
    
    // Mask all PIC interrupts
    mask_pic_interrupts();
}

/// Disable Non-Maskable Interrupts (NMI)
unsafe fn disable_nmi() {
    // Read from CMOS register to disable NMI
    core::arch::asm!(
        "out dx, al",
        in("dx") 0x70u16,
        in("al") 0x80u8,
        options(nomem, nostack, preserves_flags)
    );
}

/// Disable local APIC interrupts
unsafe fn disable_local_apic() {
    // Check if APIC is available
    if has_apic() {
        // Disable APIC by clearing the APIC enable bit
        let apic_base = get_apic_base();
        if apic_base != 0 {
            let mut apic_sivr = read_apic_register(apic_base, 0xF0); // SIVR register
            apic_sivr &= !0x100; // Clear APIC enable bit
            write_apic_register(apic_base, 0xF0, apic_sivr);
        }
    }
}

/// Mask all PIC interrupts
unsafe fn mask_pic_interrupts() {
    // Mask all interrupts on master PIC
    core::arch::asm!(
        "out dx, al",
        in("dx") 0x21u16,
        in("al") 0xFFu8,
        options(nomem, nostack, preserves_flags)
    );
    
    // Mask all interrupts on slave PIC
    core::arch::asm!(
        "out dx, al",
        in("dx") 0xA1u16,
        in("al") 0xFFu8,
        options(nomem, nostack, preserves_flags)
    );
}

/// Check if APIC is available
unsafe fn has_apic() -> bool {
    // Check CPUID feature flags for APIC
    let _eax: u32;
    let _ecx: u32;
    let edx: u32;
    
    core::arch::asm!(
        "cpuid",
        inout("eax") 1 => _eax,
        out("ecx") _ecx,
        out("edx") edx,
        options(nomem, nostack, preserves_flags)
    );
    
    // Check APIC bit in EDX
    (edx & (1 << 9)) != 0
}

/// Get APIC base address from IA32_APIC_BASE MSR
unsafe fn get_apic_base() -> u64 {
    let mut eax: u32;
    let mut edx: u32;
    
    core::arch::asm!(
        "rdmsr",
        in("ecx") 0x1Bu32, // IA32_APIC_BASE MSR
        out("eax") eax,
        out("edx") edx,
        options(nomem, nostack, preserves_flags)
    );
    
    ((edx as u64) << 32) | (eax as u64)
}

/// Read APIC register
unsafe fn read_apic_register(apic_base: u64, offset: u32) -> u32 {
    let addr = (apic_base & 0xFFFFF000) | (offset as u64);
    core::ptr::read_volatile(addr as *const u32)
}

/// Write APIC register
unsafe fn write_apic_register(apic_base: u64, offset: u32, value: u32) {
    let addr = (apic_base & 0xFFFFF000) | (offset as u64);
    core::ptr::write_volatile(addr as *mut u32, value);
}

/// Enable interrupts (for future use)
pub unsafe fn enable_interrupts() {
    core::arch::asm!("sti", options(nomem, nostack, preserves_flags));
}

/// Check if interrupts are enabled
pub fn interrupts_enabled() -> bool {
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "pushfq",
            "pop {}",
            out(reg) flags,
            options(nomem, nostack, preserves_flags)
        );
    }
    (flags & 0x200) != 0 // IF flag
}
