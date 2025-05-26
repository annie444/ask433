//! Timer and tick-loop utilities for ASK driver.
//!
//! Logic for setting up the RF timer/clock. This employs two approaches: an interrupt service
//! routine using `critical_section::with` (`timer-isr` feature), or a busy-loop delay timer
//! (`delayus-loop` feature).
//!
//! Contains helpers for polling- and ISR-based scheduling, including:
//! - `compute_ocr_value`: runtime OCR calculator
//! - `const_ocr_value`: compile-time OCR calculator
//! - `run_ask_tick_loop`: blocking driver loop for DelayUs (feature `delayus-loop`)
//! - `global_ask_timer_tick` and `tick_ask_timer!()`: interrupt-based tick callback wrapper
//! (feature `timer-isr`)
//!
//! Common prescalers: (For use with `compute_ocr_value` and `const_ocr_value`)
//!
//! | PRESCALER | TIMER_COUNTS | Overflow Interval |
//! |-----------|--------------|-------------------|
//! |        64 |          250 |              1 ms |
//! |       256 |          125 |              2 ms |
//! |       256 |          250 |              4 ms |
//! |      1024 |          125 |              8 ms |
//! |      1024 |          250 |             16 ms |

use libm::round;

#[cfg(feature = "delayus-loop")]
mod delay;
#[cfg_attr(feature = "delayus-loop", allow(unused_imports))]
#[cfg(feature = "delayus-loop")]
pub use delay::*;

#[cfg(feature = "timer-isr")]
mod isr;
#[cfg_attr(feature = "timer-isr", allow(unused_imports))]
#[cfg(feature = "timer-isr")]
pub use isr::*;

#[cfg(feature = "timer-isr")]
mod macros;
#[cfg_attr(feature = "timer-isr", allow(unused_imports))]
#[cfg(feature = "timer-isr")]
pub use macros::*;

/// 2 kilobits / second == 2000 bits / second
pub const BITS_PER_SECOND: u16 = 2_000;
/// (2 kilobits / second)^-1 == 0.0005 seconds / bit
pub const SECONDS_PER_BIT: f32 = 0.0005;
/// 2 kilobits / second == 2 * 10^-12 bits / picosecond
pub const BITS_PER_PICOSECOND: f32 = 2E-12_f32;
/// (2 * 10^-12 bits / picosecond)^-1 = 500,000,000,000 picoseconds / bit
pub const PICOSECONDS_PER_BIT: u64 = 500_000_000_000;
/// 1,000,000 picoseconds = 1 microsecond
pub const PICOSECONDS_PER_MICROSECOND: u32 = 1_000_000;

/// Computes the OCR value for an AVR timer (CTC mode)
///
/// # Arguments
/// - `f_cpu`: CPU frequency in Hz
/// - `prescaler`: timer prescaler (e.g., 8, 64, 256)
/// - `tick_us`: desired tick interval in microseconds (e.g., 62.5)
///
/// # Returns
/// - OCR value for OCRnA (rounds to nearest integer)
/// - Number of ticks per bit (for initializing the `AskDriver`)
pub fn compute_ocr_value(f_cpu: u32, prescaler: u32, tick_us: f32) -> (u16, u8) {
    let ticks_per_second: f32 = f_cpu as f32 / prescaler as f32;
    let ticks_per_bit: u8 = (SECONDS_PER_BIT * ticks_per_second) as u8;
    let ticks_per_tick: f32 = ticks_per_second * (tick_us / 1_000_000.0);
    (round(ticks_per_tick as f64) as u16, ticks_per_bit)
}

/// Compile-time OCR value calculator
///
/// # Arguments
/// - `f_cpu`: CPU frequency in Hz
/// - `prescaler`: timer prescaler (e.g., 8, 64, 256)
/// - `tick_us`: desired tick interval in microseconds (e.g., 62.5)
///
/// # Returns
/// - OCR value for OCRnA (rounds to nearest integer)
/// - Number of ticks per bit (for initializing the `AskDriver`)
pub const fn const_ocr_value(f_cpu: u32, prescaler: u32, tick_us: f32) -> (u16, u8) {
    let tick_ps = ((tick_us as f64) * (PICOSECONDS_PER_MICROSECOND as f64)) as u32; // convert µs to picoseconds to preserve precision
    let ticks_per_bit: u8 = (PICOSECONDS_PER_BIT * (tick_ps as u64)) as u8;
    let ticks_per_tick =
        (f_cpu / prescaler) as u64 * tick_ps as u64 / (PICOSECONDS_PER_MICROSECOND as u64);
    (ticks_per_tick as u16, ticks_per_bit)
}

/// Compute ticks per bit value
///
/// # Arguments
/// - `tick_us`: desired tick interval in microseconds (e.g., 62.5)
///
/// # Returns
/// - Number of ticks per bit (for initializing the `AskDriver`)
pub fn ticks_per_bit(tick_us: f32) -> u8 {
    let tick_ps = ((tick_us as f64) * (PICOSECONDS_PER_MICROSECOND as f64)) as u32;
    (PICOSECONDS_PER_BIT * (tick_ps as u64)) as u8
}

/// Compile-time ticks per bit value
///
/// # Arguments
/// - `tick_us`: desired tick interval in microseconds (e.g., 62.5)
///
/// # Returns
/// - Number of ticks per bit (for initializing the `AskDriver`)
pub const fn const_ticks_per_bit(tick_us: f32) -> u8 {
    let tick_ps = ((tick_us as f64) * (PICOSECONDS_PER_MICROSECOND as f64)) as u32;
    (PICOSECONDS_PER_BIT * (tick_ps as u64)) as u8
}
