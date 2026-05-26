//! Ring 3 launch helpers and embedded user test binary.
//!
//! Provides `run_usermode_test()` which:
//!   1. Maps a user code page and user stack page (PTE_USER, no SMEP/SMAP for test)
//!   2. Copies the embedded test binary into the code page
//!   3. Uses `iretq` to jump to ring 3
//!
//! The test binary calls syscalls and halts. Output goes to debug serial (0xE9).

use crate::gdt::{USER_CS, USER_DS};
use crate::log_info;
use x86_64::structures::paging::PageTableFlags;

/// User code load address (must match `org` in user_test.asm).
/// Uses 4 GiB — above the identity-mapped low region (QEMU has 2 GiB RAM
/// with max physical ~0x8000_0000, plus ACPI regions). 4 GiB is safely above.
const USER_CODE_VA: u64 = 0x1_0000_0000;
/// User stack grows down from this address (stack page at USER_CODE_VA - 0x1000).
/// Initial RSP at top of the mapped page, aligned to 16 bytes for x86-64 ABI.
/// (0x100000000 is the address *after* the page; the last valid byte is 0xFFFFFFFFF.)
// Stack: page at 0x0FFFFFF000 (one below code at 0x100000000).
// RSP starts at page top minus 16, leaving room for interrupt frames.
const USER_STACK_TOP_VA: u64 = 0x0FFFFFFF0;

/// The pre-assembled user test binary (flat binary, position-dependent at USER_CODE_VA).
static USER_TEST_BIN: &[u8] = include_bytes!("user_test.bin");

/// Run the ring 3 syscall test.
///
/// Maps user pages, copies the test binary, then drops to ring 3 via iretq.
/// Does not return (the test binary loops in `hlt`).
///
/// Must be called after:
///   - syscall_init() (MSRs configured)
///   - Memory subsystem is up (kernel_mapper + physical allocator)
///   - Timer is running (so SYS_GET_TICKS returns nonzero)
///
/// # Safety
/// Maps pages, transitions CPU privilege level. Kernel code must be
/// protected by SMEP/SMAP (set up by cpu_features).
pub unsafe fn run_usermode_test() -> ! {
    log_info!("syscall-test: preparing ring 3 test");

    // Allocate physical frames for user code and stack.
    let code_pa = crate::physical_memory::alloc_frame()
        .expect("alloc user code frame");
    let stack_pa = crate::physical_memory::alloc_frame()
        .expect("alloc user stack frame");

    log_info!(
        "syscall-test: code_pa={:#x} stack_pa={:#x}",
        code_pa,
        stack_pa
    );

    let stack_va = USER_STACK_TOP_VA - 0x1000;

    // Map user pages as kernel-accessible first (SMAP prevents supervisor access
    // to user pages). After copying, we switch them to user-accessible via STAC/CLAC.
    {
        let mut mapper_guard = crate::memory::runtime_mapper::kernel_mapper().lock();
        let mapper = mapper_guard
            .as_mut()
            .expect("kernel_mapper must be initialized");

        mapper
            .map_page(USER_CODE_VA, code_pa, PageTableFlags::PRESENT | PageTableFlags::WRITABLE)
            .expect("map user code page");
        mapper
            .map_page(stack_va, stack_pa, PageTableFlags::PRESENT | PageTableFlags::WRITABLE)
            .expect("map user stack page");
    }

    log_info!(
        "syscall-test: mapped code at {:#x}, stack at {:#x}–{:#x}",
        USER_CODE_VA,
        stack_va,
        USER_STACK_TOP_VA
    );

    // Copy test binary into user code page.
    // SMAP is enabled, but the page is kernel-mapped for now — OK.
    let dst = USER_CODE_VA as *mut u8;
    unsafe {
        core::ptr::copy_nonoverlapping(USER_TEST_BIN.as_ptr(), dst, USER_TEST_BIN.len());
    }

    log_info!(
        "syscall-test: copied {} bytes of user code",
        USER_TEST_BIN.len()
    );

    // Change page permissions for user access using STAC/CLAC to bypass SMAP.
    // We need to write the user-accessible flag into existing page table entries.
    // Use the kernel mapper's `unmap_page` and `map_page` approach, but we need
    // to keep the same physical frame. Runtime mapper doesn't have a remap API,
    // so we do it directly via the x86_64 Mapper trait.
    {
        let mut mapper_guard = crate::memory::runtime_mapper::kernel_mapper().lock();
        let mapper = mapper_guard
            .as_mut()
            .expect("kernel_mapper must be initialized");

        // Unmap both pages to clear existing PTEs, then remap with user flags.
        let _code_pa_v = mapper
            .unmap_page(USER_CODE_VA)
            .expect("unmap code page for flag update");
        let _stack_pa_v = mapper
            .unmap_page(stack_va)
            .expect("unmap stack page for flag update");

        // Remap with user-accessible flags. Code: R+X (user). Stack: R+W (user), NX.
        mapper
            .map_page(
                USER_CODE_VA,
                code_pa,
                PageTableFlags::PRESENT | PageTableFlags::USER_ACCESSIBLE,
            )
            .expect("remap code page with user flags");
        mapper
            .map_page(
                stack_va,
                stack_pa,
                PageTableFlags::PRESENT
                    | PageTableFlags::WRITABLE
                    | PageTableFlags::USER_ACCESSIBLE
                    | PageTableFlags::NO_EXECUTE,
            )
            .expect("remap stack page with user flags");
    }

    log_info!("syscall-test: user page permissions updated");

    // Re-enable interrupts for the ring 3 test.
    x86_64::instructions::interrupts::enable();

    log_info!("syscall-test: jumping to ring 3 at {:#x}", USER_CODE_VA);

    // Transition to ring 3. This never returns.
    jump_to_usermode(USER_CODE_VA, USER_STACK_TOP_VA);
}

/// Transition from ring 0 to ring 3 using an iretq frame.
///
/// Builds a fake interrupt frame and executes `iretq` to jump to user code.
///
/// # Safety
/// Caller guarantees:
///   - `entry` is in a mapped, user-accessible, executable page.
///   - `stack_top` is in a mapped, user-accessible, writable region.
///   - SMEP prevents kernel from accidentally executing user pages.
unsafe fn jump_to_usermode(entry: u64, stack_top: u64) -> ! {
    // RPL bits (bits 0:1) must be set to 3 for ring 3.
    let user_cs: u64 = USER_CS as u64 | 0x3;
    let user_ss: u64 = USER_DS as u64 | 0x3;

    // iretq frame (pushed in reverse; CPU pops: RIP, CS, RFLAGS, RSP, SS).
    // RFLAGS = 0x202: IF (bit 9) + reserved bit 1 always set.
    core::arch::asm!(
        // Clear segment registers for a clean user state.
        "xor ax, ax",
        "mov ds, ax",
        "mov es, ax",
        "mov fs, ax",
        // GS is left at 0 (swapgs will swap on syscall entry).

        // Build iretq frame on kernel stack.
        "push {user_ss}",        // SS  (ring 3)
        "push {stack_top}",      // RSP (user stack top)
        "push 0x202",            // RFLAGS: IF=1, reserved bit 1
        "push {user_cs}",        // CS  (ring 3, L=1)
        "push {entry}",          // RIP (user entry point)

        // iretq pops: RIP, CS, RFLAGS, RSP, SS → transitions to ring 3.
        "iretq",

        user_ss = in(reg) user_ss,
        stack_top = in(reg) stack_top,
        user_cs = in(reg) user_cs,
        entry = in(reg) entry,
        options(noreturn)
    );
}
