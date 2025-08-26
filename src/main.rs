#![no_std]
#![no_main]

use uefi::prelude::*;
use core::arch::asm;


/// Main UEFI entry point
#[entry]
fn efi_main() -> Status {
    // Step 1: Initialize UEFI logger
    uefi::helpers::init().unwrap();

    loop {
        unsafe { asm!("hlt", options(nomem, nostack, preserves_flags)) }
    }
}


