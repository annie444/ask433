//! # ask433
//!
//! A portable, no_std Rust driver for Amplitude Shift Keying (ASK/OOK) 433 MHz RF transceivers,
//! compatible with cheap modules like the FS1000A and XY-MK-5V.
//!
//! This driver implements a software-based ASK modem using:
//! - `embedded-hal` traits for digital I/O and timing
//! - a software PLL for reception and demodulation
//! - interrupt-safe buffer access with `critical-section`
//! - optional tick sources using either timer interrupts or blocking delay
//!
//! ## Crate features
//! | Feature               | Description |
//! |-----------------------|-------------|
//! | `std`                 | Disables `#![no_std]` support and replaces `healpess::Vec`s with
//! `std::vec::Vec`s |
//! | `delayus-loop`        | Uses `embedded_hal::blocking::delay::DelayUs` for bit timing |
//! | `timer-isr` (default) | Uses `critical_section::with` for bit timing |
//! | `defmt`               | Uses `defmt` logging |
//! | `log`                 | Uses `log` logging |
//!
//! ## Software Features
//!
//! - **Transmitter and receiver** in pure software (no UART or DMA)
//! - Bit-level protocol compatible with [RadioHead RH_ASK](http://www.airspayce.com/mikem/arduino/RadioHead/)
//! - Supports **training preamble**, **4b6b encoding**, and **CRC16 validation**
//! - Fully portable across AVR (e.g., Arduino Uno) and ARM Cortex-M targets
//! - Feature flags for interrupt-driven or blocking tick scheduling
//!
//! ## Usage
//!
//! ```rust
//! use ask433::driver::AskDriver;
//!
//! let mut driver = AskDriver::new(tx_pin, rx_pin);
//! loop {
//!     driver.tick(); // Call at ~62.5 µs intervals
//! }
//! ```
//!
//! Or, use `run_tick_loop()` with a `DelayUs` implementation:
//!
//! ```rust
//! ask433::timer::run_tick_loop(&mut driver, &mut delay, 63);
//! ```
//!
//! ## Feature Flags
//!
//! - `timer-isr`: Use a hardware timer ISR to call `tick()` (requires platform-specific ISR setup)
//! - `delayus-loop`: Use a blocking loop to drive `tick()` with `embedded_hal::blocking::delay::DelayUs`
//!
//! ## Integration Notes
//!
//! - Transmit and receive timing are based on ~2 kbps bit rate (~62.5 µs per tick)
//! - Timing precision is critical; hardware timer configuration is recommended for reliability
//! - Only one driver instance should be active at a time in interrupt-driven mode
//!
//! ## Status
//!
//! This crate is in early development. Contributions welcome!
//!
//! --
//! Designed for `#![no_std]` use in resource-constrained embedded environments.

#![deny(
    bad_style,
    dead_code,
    improper_ctypes,
    non_shorthand_field_patterns,
    no_mangle_generic_items,
    overflowing_literals,
    path_statements,
    patterns_in_fns_without_body,
    unconditional_recursion,
    unused,
    while_true,
    missing_debug_implementations,
    missing_docs,
    trivial_casts,
    trivial_numeric_casts,
    unused_extern_crates,
    unused_import_braces,
    unused_qualifications,
    unused_results
)]
#![cfg_attr(not(feature = "std"), no_std)]

pub use critical_section;

#[cfg(not(feature = "std"))]
pub use heapless;

pub mod consts;
pub(crate) mod crc;
pub mod driver;
pub mod encoding;
pub mod pll;
pub mod timer;
