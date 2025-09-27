//! Test Environment Specifications
//!
//! This module defines the different environments that tests can run in,
//! and what services are available in each environment.

use core::fmt;

/// Different test execution environments
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestEnvironment {
    /// Bare-metal environment - runs immediately after bootloader handoff
    ///
    /// Available:
    /// - Basic CPU operations
    /// - Stack-based memory
    /// - Raw hardware access
    ///
    /// NOT Available:
    /// - Heap allocation
    /// - Kernel services
    /// - Interrupt handling
    /// - Memory mapping beyond bootloader setup
    BareMetal,

    /// Kernel-initialized environment - runs after full kernel setup
    ///
    /// Available:
    /// - All kernel services
    /// - Heap allocation
    /// - Memory mapping
    /// - Interrupt handling
    /// - Kernel modules
    ///
    /// NOT Available:
    /// - User space features (if any)
    KernelInitialized,

    /// User space environment - runs in user mode (future)
    ///
    /// Available:
    /// - User space APIs
    /// - System calls
    ///
    /// NOT Available:
    /// - Direct hardware access
    /// - Kernel memory
    UserSpace,
}

impl fmt::Display for TestEnvironment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TestEnvironment::BareMetal => write!(f, "BareMetal"),
            TestEnvironment::KernelInitialized => write!(f, "KernelInitialized"),
            TestEnvironment::UserSpace => write!(f, "UserSpace"),
        }
    }
}

/// Test environment requirements and capabilities
#[derive(Debug, Clone)]
pub struct TestEnvironmentSpec {
    pub environment: TestEnvironment,
    pub capabilities: TestCapabilities,
}

#[derive(Debug, Clone)]
pub struct TestCapabilities {
    pub heap_allocation: bool,
    pub kernel_services: bool,
    pub interrupt_handling: bool,
    pub memory_mapping: bool,
    pub raw_hardware_access: bool,
    pub output_available: bool,
}

impl TestEnvironmentSpec {
    pub fn bare_metal() -> Self {
        Self {
            environment: TestEnvironment::BareMetal,
            capabilities: TestCapabilities {
                heap_allocation: false,
                kernel_services: false,
                interrupt_handling: false,
                memory_mapping: false,
                raw_hardware_access: true,
                output_available: false, // No safe output in bare-metal
            },
        }
    }

    pub fn kernel_initialized() -> Self {
        Self {
            environment: TestEnvironment::KernelInitialized,
            capabilities: TestCapabilities {
                heap_allocation: true,
                kernel_services: true,
                interrupt_handling: true,
                memory_mapping: true,
                raw_hardware_access: true,
                output_available: true,
            },
        }
    }

    pub fn user_space() -> Self {
        Self {
            environment: TestEnvironment::UserSpace,
            capabilities: TestCapabilities {
                heap_allocation: true,
                kernel_services: false, // Only through system calls
                interrupt_handling: false,
                memory_mapping: false,
                raw_hardware_access: false,
                output_available: true,
            },
        }
    }
}

/// Macro to specify which environment a test should run in
///
/// # Examples
///
/// ```rust
/// #[test_case]
/// #[test_environment(BareMetal)]
/// fn test_basic_cpu_ops() {
///     // This test runs in bare-metal environment
///     assert_eq!(1 + 1, 2);
/// }
///
/// #[test_case]
/// #[test_environment(KernelInitialized)]
/// fn test_heap_allocation() {
///     // This test runs after kernel initialization
///     let vec = Vec::new();
///     vec.push(42);
/// }
/// ```
#[macro_export]
macro_rules! test_environment {
    (BareMetal) => {
        // Marker for bare-metal environment
        // Tests in this environment should not use heap allocation or kernel services
    };
    (KernelInitialized) => {
        // Marker for kernel-initialized environment
        // Tests in this environment can use all kernel services
    };
    (UserSpace) => {
        // Marker for user space environment
        // Tests in this environment run in user mode
    };
}

/// Validate that a test is compatible with its specified environment
pub fn validate_test_environment(
    _test_name: &str,
    environment: TestEnvironment,
) -> Result<(), &'static str> {
    match environment {
        TestEnvironment::BareMetal => {
            // Bare-metal tests should not use heap allocation or kernel services
            // This would need to be implemented with static analysis or runtime checks
            Ok(())
        }
        TestEnvironment::KernelInitialized => {
            // Kernel tests can use all available services
            Ok(())
        }
        TestEnvironment::UserSpace => {
            // User space tests should not use kernel services directly
            Ok(())
        }
    }
}
