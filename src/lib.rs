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
//! | `delay-loop`        | Uses `embedded_hal::blocking::delay::DelayUs` for bit timing |
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
//! # use embedded_hal_mock::eh1::digital::{Mock as Pin, Transaction as PinTransaction, State as PinState};
//! # use embedded_hal::digital::OutputPin;
//! fn main() {
//!     // ...
//!     # let tx_pin = Pin::new(&[PinTransaction::set(PinState::Low)]);
//!     # let rx_pin = Pin::new(&[]);
//!     let mut driver: AskDriver<Pin, Pin, Pin> = AskDriver::new(tx_pin, rx_pin, None, 8, None, None);
//!                                                        // ^ this is the number of interrupts per bit
//!     loop {
//!         driver.tick(); // Call at ~62.5 µs intervals
//!         # break; // For testing purposes
//!     }
//!     # driver.tx.done(); // Mark the transmission complete
//!     # driver.rx.done(); // Mark the reception complete
//! }
//! ```
//!
//! Or, use `run_tick_loop()` with a `DelayUs` implementation:
//!
//! ```rust
//! use ask433::driver::AskDriver;
//! #[cfg(feature = "delay-loop")]
//! use ask433::timer::run_ask_tick_loop;
//! # use embedded_hal_mock::eh1::digital::{Mock as Pin, Transaction as PinTransaction, State as PinState};
//! # use embedded_hal_mock::eh1::delay::NoopDelay as Delay;
//! # use embedded_hal::digital::OutputPin;
//!
//! fn main() {
//!     // ...
//!     # let tx_pin = Pin::new(&[PinTransaction::set(PinState::Low)]);
//!     # let rx_pin = Pin::new(&[]);
//!     let mut driver: AskDriver<Pin, Pin, Pin> = AskDriver::new(tx_pin, rx_pin, None, 8, None, None);
//! # #[cfg(feature = "delay-loop")]                                                      // ^ this is the number of interrupts per bit
//!     # let mut delay = Delay::new();
//! # #[cfg(feature = "delay-loop")]
//!     run_ask_tick_loop(&mut driver, &mut delay, 63);
//!     # driver.tx.done();
//!     # driver.rx.done();
//! }
//! ```
//!
//! ## Feature Flags
//!
//! - `timer-isr`: Use a hardware timer ISR to call `tick()` (requires platform-specific ISR setup)
//! - `delay-loop`: Use a blocking loop to drive `tick()` with `embedded_hal::blocking::delay::DelayUs`
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

#[cfg(feature = "timer-isr")]
pub use critical_section;

#[cfg(all(feature = "timer-isr", not(feature = "std")))]
pub use heapless;

pub mod consts;
pub(crate) mod crc;
pub mod driver;
pub mod encoding;
pub mod pll;
pub mod timer;

#[cfg(test)]
mod tests {

    #[cfg(all(test, feature = "std"))]
    mod lib {
        use crate::driver::AskDriver;
        use core::fmt;
        use critical_section::RawRestoreState;
        use embedded_hal::digital;
        use std::collections::VecDeque;
        use std::sync::{Arc, Mutex};

        pub static CRIT: Mutex<bool> = Mutex::new(true);

        struct MyCriticalSection;
        critical_section::set_impl!(MyCriticalSection);

        unsafe impl critical_section::Impl for MyCriticalSection {
            unsafe fn acquire() -> RawRestoreState {
                let val = CRIT.lock().unwrap();
                *val
            }

            unsafe fn release(_token: RawRestoreState) {
                CRIT.clear_poison();
            }
        }

        #[derive(Clone, Debug)]
        pub struct Pin(Arc<Mutex<VecDeque<bool>>>);

        impl Pin {
            pub fn new() -> Self {
                Pin(Arc::new(Mutex::new(VecDeque::new())))
            }
        }

        pub struct PinError;

        impl fmt::Debug for PinError {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "PinError")
            }
        }
        impl fmt::Display for PinError {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "PinError")
            }
        }
        impl digital::Error for PinError {
            fn kind(&self) -> digital::ErrorKind {
                digital::ErrorKind::Other
            }
        }
        impl digital::ErrorType for &Pin {
            type Error = PinError;
        }

        impl digital::InputPin for &Pin {
            fn is_high(&mut self) -> Result<bool, Self::Error> {
                if self.0.is_poisoned() {
                    self.0.clear_poison();
                }
                if let Ok(mut state) = self.0.lock() {
                    if let Some(last) = state.pop_front() {
                        return Ok(last);
                    } else {
                        return Ok(false);
                    }
                } else {
                    return Err(PinError);
                }
            }

            fn is_low(&mut self) -> Result<bool, Self::Error> {
                if self.0.is_poisoned() {
                    self.0.clear_poison();
                }
                if let Ok(mut state) = self.0.lock() {
                    if let Some(last) = state.pop_front() {
                        return Ok(last == false);
                    } else {
                        return Ok(false);
                    }
                } else {
                    return Err(PinError);
                }
            }
        }

        impl digital::OutputPin for &Pin {
            fn set_high(&mut self) -> Result<(), Self::Error> {
                if self.0.is_poisoned() {
                    self.0.clear_poison();
                }
                if let Ok(mut state) = self.0.lock() {
                    state.extend(&[true; 8]);
                } else {
                    return Err(PinError);
                }
                Ok(())
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                if self.0.is_poisoned() {
                    self.0.clear_poison();
                }
                if let Ok(mut state) = self.0.lock() {
                    state.extend(&[false; 8]);
                } else {
                    return Err(PinError);
                }
                Ok(())
            }
        }

        #[test]
        fn test_simulated_radio_send_and_receive() {
            let pin = Pin::new();
            let mut driver: AskDriver<&Pin, &Pin, &Pin> =
                AskDriver::new(&pin, &pin, None, 8, None, None);
            let mut message = Vec::new();
            message.extend_from_slice(b"Hello, world!");
            let okay = driver.send(message.clone());
            assert!(okay, "Failed to send data");

            // 6 bits per byte * 8 ticks per bit + 1 bit just for good measure
            let ticks = (driver.tx_buf.len() * 48) + 8;

            // Simulate reception by pushing the sent bits into the pin state
            for _ in 0..ticks {
                driver.tick();
                std::thread::sleep(std::time::Duration::from_micros(63));
            }

            assert_eq!(
                driver.tx_good, 1,
                "Transmit buffer should be empty after sending"
            );

            // Put the driver in a state to receive
            driver.set_mode_rx();

            // Simulate halft the reception of the sent bits
            for _ in 0..(ticks / 2) {
                driver.tick();
                std::thread::sleep(std::time::Duration::from_micros(63));
            }

            assert!(driver.pll.active, "PLL should be active during sending");

            for _ in 0..ticks {
                driver.tick();
                std::thread::sleep(std::time::Duration::from_micros(63));
            }

            let _ = driver.availabile();

            // Check if the driver received the message correctly
            let received = driver.receive();
            assert!(received.is_some(), "No data received");
            let received_data = received.unwrap();
            assert_eq!(
                received_data, message,
                "Received data does not match sent data"
            );
        }
    }

    #[cfg(all(test, feature = "timer-isr"))]
    mod macros {
        use embedded_hal_mock::eh1::digital::{
            Mock as PinMock, State as PinState, Transaction as PinTransaction,
        };

        #[test]
        fn test_setup_macro_initializes_driver() {
            use crate::{init_ask_driver, setup_ask_driver};
            init_ask_driver!(PinMock, PinMock, PinMock);
            let tx = PinMock::new(&[PinTransaction::set(PinState::Low)]);
            let rx = PinMock::new(&[]);
            let ptt = PinMock::new(&[]);

            setup_ask_driver!(tx, rx, Some(ptt), 8, None, None);

            critical_section::with(|cs| {
                assert!(ASK_DRIVER.borrow(cs).borrow().is_some());
                let mut driver = ASK_DRIVER.take(cs).unwrap();
                driver.tx.done();
                driver.rx.done();
                let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
            });
        }

        #[test]
        fn test_tick_macro_runs_driver_tick() {
            use crate::{init_ask_driver, receive_from_ask, setup_ask_driver, tick_ask_timer};
            init_ask_driver!(PinMock, PinMock, PinMock);
            let tx = PinMock::new(&[
                PinTransaction::set(PinState::Low),
                PinTransaction::set(PinState::Low),
            ]);
            let rx = PinMock::new(&[PinTransaction::get(PinState::High)]);
            let ptt = PinMock::new(&[PinTransaction::set(PinState::Low)]);

            setup_ask_driver!(tx, rx, Some(ptt), 8, None, None);

            tick_ask_timer!();

            let message = receive_from_ask!();
            assert!(message.is_none());

            tick_ask_timer!();

            critical_section::with(|cs| {
                let mut driver = ASK_DRIVER.take(cs).unwrap();
                driver.tx.done();
                driver.rx.done();
                let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
            });
        }

        #[test]
        fn test_send_macro_formats_payload() {
            use crate::{init_ask_driver, send_from_ask, setup_ask_driver, tick_ask_timer};
            init_ask_driver!(PinMock, PinMock, PinMock);
            let tx = PinMock::new(&[PinTransaction::set(PinState::Low)]);
            let rx = PinMock::new(&[]);
            let ptt = PinMock::new(&[PinTransaction::set(PinState::High)]);

            setup_ask_driver!(tx, rx, Some(ptt), 8, Some(false), Some(false));

            let sent = send_from_ask![0x42, 0x43];
            assert!(sent);

            tick_ask_timer!();

            critical_section::with(|cs| {
                let mut driver = ASK_DRIVER.take(cs).unwrap();
                driver.tx.done();
                driver.rx.done();
                let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
            });
        }

        #[test]
        fn test_receive_macro_returns_none_by_default() {
            use crate::{init_ask_driver, receive_from_ask, setup_ask_driver};
            init_ask_driver!(PinMock, PinMock, PinMock);
            let tx = PinMock::new(&[
                PinTransaction::set(PinState::Low),
                PinTransaction::set(PinState::Low),
            ]);
            let rx = PinMock::new(&[]);
            let ptt = PinMock::new(&[PinTransaction::set(PinState::Low)]);

            setup_ask_driver!(tx, rx, Some(ptt), 8, Some(false), Some(false));

            let result = receive_from_ask!();
            assert!(result.is_none());

            critical_section::with(|cs| {
                let mut driver = ASK_DRIVER.take(cs).unwrap();
                driver.tx.done();
                driver.rx.done();
                let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
            });
        }
    }
}
