//! Per-CPU data for syscall/interrupt paths.
//!
//! Pointed to by IA32_KERNEL_GS_BASE (swapped to GS on `swapgs`).
//! Each CPU gets one of these; BSP uses a static instance for now.

/// Per-CPU data accessed via GS segment after `swapgs`.
///
/// # Layout contract
/// This struct is `repr(C)` because assembly code accesses fields by fixed offsets.
/// Do NOT reorder fields or insert padding without updating `entry.rs` offsets.
#[repr(C)]
pub struct PerCpuData {
    /// Top of kernel stack for syscall path (16-byte aligned).
    pub kernel_stack_top: u64,
    /// User RSP saved on syscall entry; restored on return.
    pub user_rsp: u64,
    /// CPU index (0 = BSP).
    pub cpu_index: u32,
    _pad: u32,
}

impl PerCpuData {
    /// Create a zeroed per-CPU value. Fields must be initialized before use.
    pub const fn new() -> Self {
        Self {
            kernel_stack_top: 0,
            user_rsp: 0,
            cpu_index: 0,
            _pad: 0,
        }
    }
}

/// BSP per-CPU data. Initialized by `syscall_init()`.
///
/// Aligned to 16 bytes so SIMD/GS-relative accesses are efficient.
#[repr(C, align(16))]
pub struct AlignedPerCpu(pub PerCpuData);

static mut BSP_PER_CPU: AlignedPerCpu = AlignedPerCpu(PerCpuData::new());

/// Return the address of the BSP per-CPU data (for IA32_KERNEL_GS_BASE).
pub fn bsp_percpu_addr() -> u64 {
    core::ptr::addr_of!(BSP_PER_CPU) as u64
}

/// Get a mutable reference to BSP per-CPU data.
///
/// # Safety
/// Must only be called when we own the BSP per-CPU data (before any interrupt
/// or syscall path could race on it). In practice: during `syscall_init()`.
pub unsafe fn bsp_percpu_mut() -> &'static mut PerCpuData {
    unsafe { &mut (*core::ptr::addr_of_mut!(BSP_PER_CPU)).0 }
}

/// Dedicated syscall kernel stack (BSP).
///
/// 32 KiB, placed in `.bss.stack` so the linker and memory subsystem map it
/// alongside the IST stacks.
#[link_section = ".bss.stack"]
static mut SYSCALL_STACK: [u8; 32 * 1024] = [0; 32 * 1024];

/// Return the address and size of the BSP syscall stack (for mapping during bring-up).
pub fn syscall_stack_range() -> (u64, u64) {
    let base = core::ptr::addr_of!(SYSCALL_STACK) as u64;
    let size = core::mem::size_of::<[u8; 32 * 1024]>() as u64;
    (base, size)
}

/// Returns the top of the BSP syscall stack, 16-byte aligned.
pub fn syscall_stack_top() -> u64 {
    let (base, size) = syscall_stack_range();
    (base + size) & !0xFu64
}
