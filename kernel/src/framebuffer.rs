//! Framebuffer drawing utilities
//!
//! This module provides functions for drawing to the framebuffer, including
//! pixel manipulation and simple graphics primitives.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use theseus_shared::handoff::Handoff;

/// Virtual address where the framebuffer is mapped
const FRAMEBUFFER_VIRTUAL_BASE: u64 = 0xFFFFFFFF90000000;

/// Global state for the heart animation
static HEART_VISIBLE: AtomicBool = AtomicBool::new(true);
static ANIMATION_TICKS: AtomicU32 = AtomicU32::new(0);

/// Heart pattern data (16x16 pixels) - 4x larger than before
/// Each row is 16 bits, where 1 = pixel on, 0 = pixel off
#[allow(dead_code)]
const HEART_PATTERN: [u16; 16] = [
    0b0000110000110000,
    0b0011111001111100,
    0b0111111111111110,
    0b1111111111111111,
    0b1111111111111111,
    0b1111111111111111,
    0b1111111111111111,
    0b0111111111111110,
    0b0011111111111100,
    0b0001111111111000,
    0b0000111111110000,
    0b0000011111100000,
    0b0000001111000000,
    0b0000000110000000,
    0b0000000000000000,
    0b0000000000000000,
];

/// Colors for 4-byte BGRA framebuffer
#[allow(dead_code)]
const WHITE_COLOR: u8 = 0xFF; // Full intensity white

/// Black color (background) - transparent/black
#[allow(dead_code)]
const BLACK_COLOR: u8 = 0x00; // No intensity

/// Red color for heart (bright red in BGRA format)
#[allow(dead_code)]
const RED_COLOR: u8 = 0xFF; // Will be used to create bright red BGRA value

/// Heart size in pixels
const HEART_SIZE: usize = 16;

/// Animation timing constants
const ANIMATION_TICK_INTERVAL: u32 = 1000; // Toggle every 1000 ticks for ~1 second intervals

/// Heart positioning constants
const HEART_MARGIN_RIGHT: usize = 5; // Pixels from right edge
const HEART_MARGIN_TOP: usize = 5; // Pixels from top edge

/// Color constants for simple color mapping
const COLOR_RED: u8 = 0x00;
const COLOR_BLACK: u8 = 0xFF;
const COLOR_GRAY: u8 = 0x80;

/// Get the framebuffer virtual address
fn get_framebuffer_ptr() -> *mut u8 {
    FRAMEBUFFER_VIRTUAL_BASE as *mut u8
}

/// Create a BGRA color value (Blue-Green-Red-Alpha)
fn create_bgra_color(b: u8, g: u8, r: u8, a: u8) -> u32 {
    (a as u32) << 24 | (r as u32) << 16 | (g as u32) << 8 | (b as u32)
}

/// Calculate pixel offset in framebuffer
fn get_pixel_offset(x: usize, y: usize, _width: usize, stride: usize) -> usize {
    // If stride equals width, it's pixels per scanline, not bytes
    // If stride > width, it's bytes per scanline
    if stride == 1280 {
        // Same as width, so it's pixels per scanline
        y * stride * 4 + (x * 4) // 4 bytes per pixel
    } else {
        y * stride + (x * 4) // Stride is already in bytes
    }
}

/// Draw a pixel at the specified coordinates with BGRA color
unsafe fn draw_pixel_bgra(
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    stride: usize,
    bgra_color: u32,
    verbose: bool,
) {
    if verbose
        && ((x == 100 && y == 50)
            || (x == 200 && y == 50)
            || (x == 300 && y == 50)
            || (x == 400 && y == 50))
    {
        theseus_shared::qemu_print!("Pixel (");
        theseus_shared::print_hex_u64_0xe9!(x as u64);
        theseus_shared::qemu_print!(",");
        theseus_shared::print_hex_u64_0xe9!(y as u64);
        theseus_shared::qemu_print!(") offset=");
        theseus_shared::print_hex_u64_0xe9!(get_pixel_offset(x, y, width, stride) as u64);
        theseus_shared::qemu_print!(" stride=");
        theseus_shared::print_hex_u64_0xe9!(stride as u64);
        theseus_shared::qemu_print!(" fb_ptr=");
        theseus_shared::print_hex_u64_0xe9!(get_framebuffer_ptr() as u64);
        theseus_shared::qemu_print!(" bgra_color=");
        theseus_shared::print_hex_u64_0xe9!(bgra_color as u64);
        theseus_shared::qemu_println!("");
    }

    if x >= width || y >= height {
        return; // Out of bounds
    }

    let offset = get_pixel_offset(x, y, width, stride);
    let fb_ptr = get_framebuffer_ptr();

    core::ptr::write_unaligned(fb_ptr.add(offset) as *mut u32, bgra_color);
}

/// Draw a pixel at the specified coordinates with simple color
unsafe fn draw_pixel(
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    stride: usize,
    color: u8,
    verbose: bool,
) {
    // Convert simple color to BGRA (Blue-Green-Red-Alpha)
    let bgra_color = match color {
        COLOR_BLACK => create_bgra_color(0x00, 0x00, 0x00, 0xFF), // Black (B=00, G=00, R=00, A=FF)
        COLOR_RED => create_bgra_color(0x00, 0x00, 0xFF, 0xFF),   // Red (B=00, G=00, R=FF, A=FF)
        COLOR_GRAY => create_bgra_color(0x80, 0x80, 0x80, 0xFF),  // Gray (B=80, G=80, R=80, A=FF)
        _ => create_bgra_color(color, color, color, 0xFF),        // Grayscale
    };
    draw_pixel_bgra(x, y, width, height, stride, bgra_color, verbose);
}

/// Clear a rectangular area
#[allow(dead_code)]
unsafe fn clear_area(
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    fb_width: usize,
    fb_height: usize,
    fb_stride: usize,
    color: u8,
    verbose: bool,
) {
    if verbose {
        theseus_shared::qemu_print!("clear_area called: (");
        theseus_shared::print_hex_u64_0xe9!(x as u64);
        theseus_shared::qemu_print!(",");
        theseus_shared::print_hex_u64_0xe9!(y as u64);
        theseus_shared::qemu_print!(") size ");
        theseus_shared::print_hex_u64_0xe9!(width as u64);
        theseus_shared::qemu_print!("x");
        theseus_shared::print_hex_u64_0xe9!(height as u64);
        theseus_shared::qemu_print!(" color=");
        theseus_shared::print_hex_u64_0xe9!(color as u64);
        theseus_shared::qemu_println!("");
    }

    for dy in 0..height {
        for dx in 0..width {
            draw_pixel(
                x + dx,
                y + dy,
                fb_width,
                fb_height,
                fb_stride,
                color,
                verbose,
            );
        }
    }
}

/// Draw the heart pattern at the specified position
#[allow(dead_code)]
unsafe fn draw_heart(
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    stride: usize,
    color: u8,
    verbose: bool,
) {
    for row in 0..HEART_SIZE {
        let pattern_row = HEART_PATTERN[row];
        for col in 0..HEART_SIZE {
            if (pattern_row & (1 << (15 - col))) != 0 {
                draw_pixel(x + col, y + row, width, height, stride, color, verbose);
            }
        }
    }
}

/// Update the heart animation
/// This function should be called from the timer interrupt handler
pub unsafe fn update_heart_animation(handoff: &Handoff) {
    if handoff.gop_fb_base == 0 || handoff.gop_width == 0 || handoff.gop_height == 0 {
        return;
    }

    let width = handoff.gop_width as usize;
    let height = handoff.gop_height as usize;
    let stride = handoff.gop_stride as usize;

    // Position heart in upper right corner using constants
    let heart_x = width - HEART_SIZE - HEART_MARGIN_RIGHT;
    let heart_y = HEART_MARGIN_TOP;

    // Increment animation tick
    let ticks = ANIMATION_TICKS.fetch_add(1, Ordering::Relaxed);

    // Toggle heart visibility every ANIMATION_TICK_INTERVAL ticks
    if ticks % ANIMATION_TICK_INTERVAL == 0 {
        let visible = HEART_VISIBLE.load(Ordering::Relaxed);
        HEART_VISIBLE.store(!visible, Ordering::Relaxed);

        // Clear the heart area first with black background
        clear_area(
            heart_x,
            heart_y,
            HEART_SIZE,
            HEART_SIZE,
            width,
            height,
            stride,
            COLOR_BLACK,
            false,
        );

        // Draw heart if visible
        if !visible {
            draw_heart(heart_x, heart_y, width, height, stride, COLOR_RED, false);
        }
    }
}

/// Initialize the framebuffer drawing system
pub fn init_framebuffer_drawing() {
    ANIMATION_TICKS.store(0, Ordering::Relaxed);
    HEART_VISIBLE.store(true, Ordering::Relaxed);
}

/// Draw an initial heart pattern (call this once during kernel init)
pub unsafe fn draw_initial_heart(handoff: &Handoff, verbose: bool) {
    if handoff.gop_fb_base == 0 || handoff.gop_width == 0 || handoff.gop_height == 0 {
        if verbose {
            theseus_shared::qemu_println!("No framebuffer available");
        }
        return;
    }

    // Get framebuffer info from handoff struct
    let width = handoff.gop_width as usize;
    let height = handoff.gop_height as usize;
    let stride = handoff.gop_stride as usize;
    let pixel_format = handoff.gop_pixel_format;
    let fb_base = handoff.gop_fb_base;
    let fb_size = handoff.gop_fb_size;

    if verbose {
        theseus_shared::qemu_print!("Framebuffer: ");
        theseus_shared::print_hex_u64_0xe9!(width as u64);
        theseus_shared::qemu_print!("x");
        theseus_shared::print_hex_u64_0xe9!(height as u64);
        theseus_shared::qemu_print!(", stride: ");
        theseus_shared::print_hex_u64_0xe9!(stride as u64);
        theseus_shared::qemu_print!(", format: ");
        theseus_shared::print_hex_u64_0xe9!(pixel_format as u64);
        theseus_shared::qemu_print!(", base: 0x");
        theseus_shared::print_hex_u64_0xe9!(fb_base);
        theseus_shared::qemu_print!(", size: ");
        theseus_shared::print_hex_u64_0xe9!(fb_size);
        theseus_shared::qemu_println!("");

        // Calculate bytes per pixel from handoff data
        let bytes_per_pixel = stride / width;
        theseus_shared::qemu_print!("Bytes per pixel: ");
        theseus_shared::print_hex_u64_0xe9!(bytes_per_pixel as u64);
        theseus_shared::qemu_println!("");
    }

    // Position heart in upper right corner using constants
    let heart_x = width - HEART_SIZE - HEART_MARGIN_RIGHT;
    let heart_y = HEART_MARGIN_TOP;

    if verbose {
        theseus_shared::qemu_print!("Heart at (");
        theseus_shared::print_hex_u64_0xe9!(heart_x as u64);
        theseus_shared::qemu_print!(",");
        theseus_shared::print_hex_u64_0xe9!(heart_y as u64);
        theseus_shared::qemu_println!(") size 16x16");
    }

    // Draw the initial heart in the upper right corner
    draw_heart(heart_x, heart_y, width, height, stride, COLOR_RED, verbose);
}
