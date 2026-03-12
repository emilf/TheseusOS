//! Module: framebuffer
//!
//! SOURCE OF TRUTH:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! DEPENDS ON AXIOMS:
//! - docs/axioms/debug.md#A3:-The-runtime-monitor-is-a-first-class-inspection-surface
//! - docs/axioms/boot.md#A2:-Boot-Services-are-exited-before-kernel-entry
//!
//! INVARIANTS:
//! - This module owns simple framebuffer-side drawing helpers used by current visual bring-up/debug behavior.
//! - Framebuffer access depends on the handoff-provided graphics information remaining valid and mapped by the documented boot path.
//! - The current animation/debug drawing behavior is observability/UI garnish, not core kernel correctness.
//!
//! SAFETY:
//! - Raw framebuffer writes assume the mapped framebuffer region and stride/pixel-format interpretation are correct enough for the current environment.
//! - Visual debug helpers must not silently become required for the rest of kernel bring-up to function.
//!
//! PROGRESS:
//! - docs/plans/observability.md
//! - docs/plans/boot-flow.md
//!
//! Framebuffer drawing helpers for visual bring-up/debug output.
//!
//! This module contains simple framebuffer-side drawing and animation helpers,
//! not a full graphics subsystem.

use crate::{log_debug, log_info, log_trace};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use theseus_shared::handoff::Handoff;

/// Virtual base address where the framebuffer is expected to be mapped.
const FRAMEBUFFER_VIRTUAL_BASE: u64 = 0xFFFFFFFF90000000;

/// Global state for the simple heart animation.
static HEART_VISIBLE: AtomicBool = AtomicBool::new(true);
static ANIMATION_TICKS: AtomicU32 = AtomicU32::new(0);

/// Heart pattern data (16×16 pixels).
///
/// Each row uses 16 bits where `1` means pixel-on.
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

/// Heart size in pixels.
const PIXEL_SIZE: usize = 16;

/// Animation timing constants.
const ANIMATION_TICK_INTERVAL: u32 = 1000; // Toggle every 1000 ticks for ~1 second intervals

/// Heart positioning constants.
const HEART_MARGIN_RIGHT: usize = 5; // Pixels from right edge
const HEART_MARGIN_TOP: usize = 5; // Pixels from top edge

/// Color constants for the simple drawing helpers.
const COLOR_RED: u8 = 0x00;
const COLOR_BLACK: u8 = 0xFF;
const COLOR_GRAY: u8 = 0x80;

/// Return the mapped framebuffer base pointer.
fn get_framebuffer_ptr() -> *mut u8 {
    FRAMEBUFFER_VIRTUAL_BASE as *mut u8
}

/// Pack a BGRA color value.
fn create_bgra_color(b: u8, g: u8, r: u8, a: u8) -> u32 {
    (a as u32) << 24 | (r as u32) << 16 | (g as u32) << 8 | (b as u32)
}

/// Calculate the framebuffer byte offset for one pixel.
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

/// Draw one pixel using a packed BGRA color.
unsafe fn draw_pixel_bgra(
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    stride: usize,
    bgra_color: u32,
) {
    const DEBUG_PIXELS: bool = false; // Set to true to trace specific pixels
    if DEBUG_PIXELS
        && ((x == 100 && y == 50)
            || (x == 200 && y == 50)
            || (x == 300 && y == 50)
            || (x == 400 && y == 50))
    {
        log_trace!(
            "Pixel ({},{}) offset={:#x} stride={:#x} fb_ptr={:#x} bgra_color={:#x}",
            x,
            y,
            get_pixel_offset(x, y, width, stride),
            stride,
            get_framebuffer_ptr() as u64,
            bgra_color
        );
    }

    if x >= width || y >= height {
        return; // Out of bounds
    }

    let offset = get_pixel_offset(x, y, width, stride);
    let fb_ptr = get_framebuffer_ptr();

    core::ptr::write_unaligned(fb_ptr.add(offset) as *mut u32, bgra_color);
}

/// Draw one pixel using the module's simple color constants.
unsafe fn draw_pixel(x: usize, y: usize, width: usize, height: usize, stride: usize, color: u8) {
    // Convert simple color to BGRA (Blue-Green-Red-Alpha)
    let bgra_color = match color {
        COLOR_BLACK => create_bgra_color(0x00, 0x00, 0x00, 0xFF), // Black (B=00, G=00, R=00, A=FF)
        COLOR_RED => create_bgra_color(0x00, 0x00, 0xFF, 0xFF),   // Red (B=00, G=00, R=FF, A=FF)
        COLOR_GRAY => create_bgra_color(0x80, 0x80, 0x80, 0xFF),  // Gray (B=80, G=80, R=80, A=FF)
        _ => create_bgra_color(color, color, color, 0xFF),        // Grayscale
    };
    draw_pixel_bgra(x, y, width, height, stride, bgra_color);
}

/// Fill a rectangular framebuffer area.
unsafe fn clear_area(
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    fb_width: usize,
    fb_height: usize,
    fb_stride: usize,
    color: u8,
) {
    log_trace!(
        "clear_area called: ({},{}) size {}x{} color={:#x}",
        x,
        y,
        width,
        height,
        color
    );

    for dy in 0..height {
        for dx in 0..width {
            draw_pixel(x + dx, y + dy, fb_width, fb_height, fb_stride, color);
        }
    }
}

/// Draw the heart pattern at the requested position.
unsafe fn draw_heart(x: usize, y: usize, width: usize, height: usize, stride: usize, color: u8) {
    log_trace!("Drawing heart at ({},{}) size 16x16", x, y);

    for row in 0..PIXEL_SIZE {
        let pattern_row = HEART_PATTERN[row];
        for col in 0..PIXEL_SIZE {
            if (pattern_row & (1 << (15 - col))) != 0 {
                draw_pixel(x + col, y + row, width, height, stride, color);
            }
        }
    }
}

/// Update the heart animation state.
///
/// The current visual-debug path expects this to be ticked from the timer path.
pub unsafe fn update_heart_animation(handoff: &Handoff) {
    if handoff.gop_fb_base == 0 || handoff.gop_width == 0 || handoff.gop_height == 0 {
        return;
    }

    let width = handoff.gop_width as usize;
    let height = handoff.gop_height as usize;
    let stride = handoff.gop_stride as usize;

    // Position heart in upper right corner using constants
    let heart_x = width - PIXEL_SIZE - HEART_MARGIN_RIGHT;
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
            PIXEL_SIZE,
            PIXEL_SIZE,
            width,
            height,
            stride,
            COLOR_BLACK,
        );

        // Draw heart if visible
        if !visible {
            draw_heart(heart_x, heart_y, width, height, stride, COLOR_RED);
        }
    }
}

/// Initialize framebuffer animation state.
pub fn init_framebuffer_drawing() {
    ANIMATION_TICKS.store(0, Ordering::Relaxed);
    HEART_VISIBLE.store(true, Ordering::Relaxed);
}

/// Draw the initial framebuffer heart pattern.
pub unsafe fn draw_initial_screen(handoff: &Handoff) {
    if handoff.gop_fb_base == 0 || handoff.gop_width == 0 || handoff.gop_height == 0 {
        log_debug!("No framebuffer available");
        return;
    }

    // Get framebuffer info from handoff struct
    let width = handoff.gop_width as usize;
    let height = handoff.gop_height as usize;
    let stride = handoff.gop_stride as usize;
    let pixel_format = handoff.gop_pixel_format;
    let fb_base = handoff.gop_fb_base;
    let fb_size = handoff.gop_fb_size;

    log_info!(
        "Framebuffer: {}x{}, stride: {}, format: {}, base: {:#x}, size: {:#x}",
        width,
        height,
        stride,
        pixel_format,
        fb_base,
        fb_size
    );

    // Calculate bytes per pixel from handoff data
    let bytes_per_pixel = stride / width;
    log_debug!("Bytes per pixel: {}", bytes_per_pixel);

    // Position heart in upper right corner using constants
    let heart_x = width - PIXEL_SIZE - HEART_MARGIN_RIGHT;
    let heart_y = HEART_MARGIN_TOP;

    log_debug!("Heart at ({},{}) size 16x16", heart_x, heart_y);

    // Draw the initial heart in the upper right corner
    draw_heart(heart_x, heart_y, width, height, stride, COLOR_RED);

    // Draw the boot logo
    draw_bootlogo(width - 190, 0, width, height, stride);
}

/// Draw the boot logo at the specified position
unsafe fn draw_bootlogo(x: usize, y: usize, width: usize, height: usize, stride: usize) {
    log_trace!(
        "Drawing boot logo at ({},{}) size {}x{}",
        x,
        y,
        width,
        height
    );
    use crate::bootlogo::BOOT_LOGO_ARRAY;

    for row in 0..255 {
        for col in 0..255 {
            draw_pixel_bgra(
                x + col,
                y + row,
                width,
                height,
                stride,
                BOOT_LOGO_ARRAY[row][col],
            );
        }
    }
}
