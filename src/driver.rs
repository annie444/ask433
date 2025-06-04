//! ASK/OOK modem driver for 433 MHz RF transceivers.
//!
//! This module provides the [`AskDriver`] struct, which implements a software-based
//! Amplitude Shift Keying (ASK) modem for use with low-cost 433 MHz RF modules.
//! It supports both transmission and reception using only digital I/O and a timing source.
//!
//! The driver operates independently of the target platform's oscillator speed,
//! provided that the [`tick()`](AskDriver::tick) method is called at regular intervals
//! (typically every 62.5 µs for 2 kbps bit rate).
//!
//! ## Features
//!
//! - Bit-level transmit and receive logic using On-Off Keying (OOK)
//! - Software-based phase-locked loop (PLL) demodulator
//! - Uses `embedded-hal` for pin abstraction and portability
//! - Designed for `no_std` environments with optional interrupt integration
//!
//! ## Example
//!
//! ```rust
//! # use embedded_hal_mock::eh1::digital::{Mock as Pin, State as PinState, Transaction as PinTransaction};
//! # use embedded_hal::digital::OutputPin;
//! use ask433::driver::AskDriver;
//!
//! fn main() {
//!     # let tx_pin = Pin::new(&[PinTransaction::set(PinState::Low)]);
//!     # let rx_pin = Pin::new(&[]);
//!     let mut driver: AskDriver<Pin, Pin, Pin> = AskDriver::new(tx_pin, rx_pin, None, 8, None, None);
//!
//!     loop {
//!         driver.tick(); // Called every 62.5 µs by a delay or timer interrupt
//!         # break; // For testing purposes
//!     }
//!     # driver.tx.done();
//!     # driver.rx.done();
//! }
//! ```
//!
//! ## Design Notes
//!
//! This module does **not** include packet framing, encoding, or CRC —
//! it focuses solely on low-level modulation and demodulation.
//! You may layer a protocol (e.g., RadioHead-compatible) on top.
//!
//! For demodulation internals, see [`crate::pll::SoftwarePLL`].
//!
//! For timer and tick scheduling helpers, see [`crate::timer`].

#[cfg(not(feature = "std"))]
use crate::consts::{ASK_MAX_BUF_LEN_USIZE, ASK_MAX_MESSAGE_LEN_USIZE};

use crate::consts::{ASK_HEADER_LEN, ASK_MAX_MESSAGE_LEN, ASK_PREAMBLE_LEN, BROADCAST_ADDRESS};
use crate::crc::crc_ccitt_update;
use crate::encoding::{SYMBOLS, encode_4b6b};
use crate::pll::SoftwarePLL;
use embedded_hal::digital::{InputPin, OutputPin};
use nb::block;

use core::convert::Infallible;
#[cfg(not(feature = "std"))]
use heapless::Vec;
#[cfg(feature = "std")]
use std::vec::Vec;

/// High-level state machine for the `AskDriver`, representing its current operational mode.
///
/// This enum allows `AskDriver` to track and transition between distinct stages
/// of operation such as sleeping, transmitting, or receiving. It is intended for
/// use in both internal driver control logic and for external inspection of driver state.
///
/// The state should be updated at key transitions: during setup, after transmission,
/// when entering receive mode, or when conserving power.
#[derive(PartialEq, Eq, Clone, Copy, Default, Debug)]
pub enum AskMode {
    ///   Low-power mode where no RF activity is occurring.
    ///   The driver disables all transmission and reception and may shut down peripherals.
    ///   Transition to [`Rx`](AskMode::Rx) or [`Tx`](AskMode::Tx) to resume operation.
    Sleep,
    ///   Ready state. The driver is powered and initialized but not actively sending or receiving.
    ///   This is a good default resting state between operations.
    #[default]
    Idle,
    ///   The driver is actively transmitting a message.
    ///   Transmission timing is handled by internal counters or scheduled `tick()` calls.
    Tx,
    ///   The driver is actively listening for incoming signals and demodulating them into bytes.
    ///   The software PLL is engaged, and decoded symbols are processed.
    Rx,
    ///   Channel Activity Detection mode. This is a temporary sniffing state used to detect
    ///   whether the channel is currently in use, before transmitting.
    ///   Typically a short-duration passive RX phase that ends in either [`Idle`](AskMode::Idle) or [`Tx`](AskMode::Tx).
    Cad,
}

/// A software-driven Amplitude Shift Keying (ASK) modem for 433 MHz RF transceivers.
///
/// `AskDriver` provides both transmission and reception support for low-cost OOK/ASK
/// RF modules such as the FS1000A (TX) and XY-MK-5V (RX). It uses `embedded-hal` traits
/// for digital pin access, allowing portability across a wide range of platforms.
///
/// ## Transmission
///
/// Transmission is handled by manually toggling a digital output pin (`TX`)
/// using On-Off Keying (OOK), where:
/// - `HIGH` = carrier on = logical `1`
/// - `LOW`  = carrier off = logical `0`
///
/// You must call [`tick()`](#method.tick) at regular intervals (typically every 62.5 µs)
/// to advance the internal state machine and emit bits with accurate timing.
///
/// ## Reception
///
/// Reception is performed via a sampled input pin (`RX`) using a software
/// phase-locked loop (PLL) to demodulate incoming ASK signals and reconstruct
/// the bitstream. The PLL samples the RX pin at the same tick rate and maintains
/// a phase ramp and integrator for majority-vote detection.
///
/// ## Type Parameters
///
/// - `TX`: A type implementing [`embedded_hal::digital::OutputPin`] used for RF transmission
/// - `RX`: A type implementing [`embedded_hal::digital::InputPin`] used for RF reception
///
/// ## Example
///
/// ```rust
/// # use embedded_hal_mock::eh1::digital::{Mock as Pin, State as PinState, Transaction as PinTransaction};
/// # use embedded_hal::digital::OutputPin;
/// use ask433::driver::AskDriver;
///
/// fn main() {
///     # let tx_pin = Pin::new(&[PinTransaction::set(PinState::Low)]);
///     # let rx_pin = Pin::new(&[]);
///     let mut driver: AskDriver<Pin, Pin, Pin> = AskDriver::new(tx_pin, rx_pin, None, 8, None, None);
///
///     loop {
///         driver.tick(); // Called every 62.5 µs by a delay or timer interrupt
///         # break; // For testing purposes
///     }
///     # driver.tx.done();
///     # driver.rx.done();
/// }
/// ```
///
/// ## Notes
///
/// - Only one `AskDriver` instance should be active if you're using interrupts.
/// - You are responsible for calling `tick()` at the correct interval using either
///   a hardware timer interrupt or a polling loop.
/// - Message encoding (4b6b) and decoding are not yet implemented.
///
/// ## See also
/// - [`SoftwarePLL`]: internal demodulation logic
#[derive(Debug)]
pub struct AskDriver<TX, RX, PTT>
where
    TX: OutputPin,
    RX: InputPin,
    PTT: OutputPin,
{
    /// The current mode of the RF module
    pub mode: AskMode,
    /// TX pin
    pub tx: TX,
    /// RX pin
    pub rx: RX,
    /// Push To Talk (PTT) pin
    pub ptt: Option<PTT>,
    /// [`SoftwarePLL`] instance
    pub pll: SoftwarePLL,
    ticks_per_bit: u8,
    tick_counter: u8,
    /// Holds the transmission buffer
    #[cfg(feature = "std")]
    pub tx_buf: Vec<u8>,
    /// Holds the transmission buffer
    #[cfg(not(feature = "std"))]
    pub tx_buf: Vec<u8, ASK_MAX_BUF_LEN_USIZE>,
    this_address: u8,
    promiscuous: bool,

    /// Destination address for the outgoing message header.
    /// This field is used when preparing the `to` header byte during transmission.
    pub tx_header_to: u8,

    /// Source address for the outgoing message header.
    /// Identifies the sender of the message (i.e., this device).
    pub tx_header_from: u8,

    /// Packet ID for the outgoing message.
    /// Typically incremented per message to help receivers detect duplicates.
    pub tx_header_id: u8,

    /// Custom user-defined flags for the outgoing message.
    /// Useful for acknowledgments, control bits, or application-specific use.
    pub tx_header_flags: u8,

    /// Destination address from the last received message.
    /// Parsed from the incoming packet header.
    pub rx_header_to: u8,

    /// Source address from the last received message.
    /// Indicates who sent the last valid message.
    pub rx_header_from: u8,

    /// Packet ID from the last received message.
    /// Can be used to detect duplicate packets or correlate replies.
    pub rx_header_id: u8,

    /// Custom flags from the last received message.
    /// Application-specific semantics (e.g., ACK bit, message type).
    pub rx_header_flags: u8,
    ptt_inverted: bool,

    /// Index into the transmission buffer, pointing to the current symbol being transmitted.
    /// Used by the internal state machine to track symbol progress.
    pub(crate) tx_index: u8,

    /// Current bit position within the current 6-bit symbol being transmitted (0–5).
    /// After 6 bits, the driver moves to the next symbol.
    pub(crate) tx_bit: u8,

    tx_sample: u8,
    tx_buf_len: u8,

    /// Counter of successfully completed transmissions.
    /// Incremented when the transmit buffer is sent in full and the driver returns to idle.
    pub tx_good: u16,

    /// Counter of failed or invalid received messages.
    /// Typically incremented when a CRC check fails or an invalid header is detected.
    pub rx_bad: u16,

    /// Counter of successfully received and validated messages.
    /// Incremented after a complete and CRC-valid packet is accepted.
    pub rx_good: u16,
    rx_buf_valid: bool,
}

impl<TX, RX, PTT> AskDriver<TX, RX, PTT>
where
    TX: OutputPin,
    RX: InputPin,
    PTT: OutputPin,
{
    const PREAMBLE: [u8; ASK_PREAMBLE_LEN as usize] =
        [0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c];

    /// Creates a new `AskDriver` instance with the given TX and RX pins.
    ///
    /// # Arguments
    /// - `tx`: The output pin used to drive the 433 MHz transmitter (carrier on/off).
    /// - `rx`: The input pin used to sample the receiver output (OOK signal).
    /// - `ptt`: The optional push to talk pin output singnal.
    /// - `ticks_per_bit`: Number of `tick()` calls per bit period (e.g., 8 for 2 kbps).
    /// - `ptt_inverted`: Whether the ptt signals should be inverted (On = LOW, Off = HIGH)
    ///
    /// # Returns
    /// A fully initialized `AskDriver` ready to use with scheduled `tick()` calls.
    ///
    /// # Notes
    /// TX is driven `LOW` initially (carrier off).
    pub fn new(
        tx: TX,
        rx: RX,
        ptt: Option<PTT>,
        ticks_per_bit: u8,
        ptt_inverted: Option<bool>,
        rx_inverted: Option<bool>,
    ) -> Self {
        let ptt_invert = match ptt_inverted {
            Some(ptt) => ptt,
            None => false,
        };
        #[allow(unused_mut)]
        let mut tx = tx;
        let _ = tx.set_low(); // Ensure idle
        let mut tx_buf = Vec::new();
        let _ = tx_buf.extend_from_slice(&Self::PREAMBLE);
        let rx_invert = match rx_inverted {
            Some(rxi) => rxi,
            None => false,
        };
        let mut cls = Self {
            mode: AskMode::Idle,
            tx,
            rx,
            pll: SoftwarePLL::new(ticks_per_bit, rx_invert),
            ticks_per_bit,
            tick_counter: 0,
            tx_buf,
            tx_header_to: BROADCAST_ADDRESS,
            tx_header_from: BROADCAST_ADDRESS,
            tx_header_id: 0,
            tx_header_flags: 0,
            rx_header_to: 0,
            rx_header_from: 0,
            rx_header_id: 0,
            rx_header_flags: 0,
            this_address: BROADCAST_ADDRESS,
            promiscuous: false,
            tx_good: 0,
            ptt,
            ptt_inverted: ptt_invert,
            tx_index: 0,
            tx_bit: 0,
            tx_sample: 0,
            tx_buf_len: 0,
            rx_good: 0,
            rx_bad: 0,
            rx_buf_valid: false,
        };
        cls.set_mode_idle();
        cls
    }

    /// Sets the address for this RF module.
    /// Defaults to u8::MAX.
    pub fn set_address(&mut self, addr: u8) {
        self.this_address = addr;
    }

    fn write_tx(&mut self, mode: bool) {
        if mode {
            self.tx.set_high().unwrap();
        } else {
            self.tx.set_low().unwrap();
        }
    }

    fn write_ptt(&mut self, mode: bool) {
        let state = if self.ptt_inverted { !mode } else { mode };
        if let Some(ref mut ptt) = self.ptt {
            if state {
                let _ = ptt.set_high();
            } else {
                let _ = ptt.set_low();
            }
        }
    }

    /// Sets the driver into sleep mode.
    pub fn set_mode_idle(&mut self) {
        if self.mode != AskMode::Idle {
            self.write_ptt(false);
            self.write_tx(false);
            self.mode = AskMode::Idle;
        }
    }

    /// Sets the driver into receive mode.
    pub fn set_mode_rx(&mut self) {
        if self.mode != AskMode::Rx {
            self.write_ptt(false);
            self.write_tx(false);
            self.mode = AskMode::Rx;
        }
    }

    /// Sets the driver into transmit mode.
    pub fn set_mode_tx(&mut self) {
        if self.mode != AskMode::Tx {
            self.tx_index = 0;
            self.tx_bit = 0;
            self.tx_sample = 0;

            self.write_ptt(true);
            self.mode = AskMode::Tx;
        }
    }

    /// Checks whether a valid, complete message is available to be received.
    ///
    /// This method transitions the driver into receive mode (if not already there),
    /// checks whether the internal demodulation buffer is full, and if so,
    /// runs validation (via `validate_rx_buf()`) to determine whether the message is valid.
    ///
    /// # Behavior
    /// - If the driver is currently transmitting ([`AskMode::Tx`]), returns `false`
    /// - If the software PLL has filled its buffer (`AskDriver.pll.full == true`), calls
    ///   [`validate_rx_buf()`](AskDriver::validate_rx_buf) to perform a CRC check and extract headers
    /// - Marks the buffer as no longer full after validation
    /// - Returns the value of `self.rx_buf_valid`, indicating whether a valid message is now present
    ///
    /// # Returns
    /// - `true`: A valid, fully received message is available in the RX buffer
    /// - `false`: No valid message is available, or the driver is currently transmitting
    ///
    /// # Notes
    /// - Calling this function does **not** consume the message; use [`receive()`](AskDriver::receive)
    ///   to access the payload after this returns `true`.
    /// - This function may trigger a transition from [`AskMode::Idle`] or [`AskMode::Sleep`]
    ///   to [`AskMode::Rx`] if receive mode is not already active.
    ///
    /// # Side Effects
    /// - May cause internal state transition via `set_mode_rx()`
    /// - Clears the `pll.full` flag after validation
    ///
    /// # See also
    /// - [`AskDriver::receive()`]
    /// - [`AskDriver::validate_rx_buf()`]
    pub fn availabile(&mut self) -> bool {
        if self.mode == AskMode::Tx {
            return false;
        }
        self.set_mode_rx();
        if self.pll.full {
            self.validate_rx_buf();
            self.pll.full = false;
        }
        return self.rx_buf_valid;
    }

    /// Validates the received message buffer using CRC-CCITT and extracts header metadata.
    ///
    /// This method is called after a full message has been received and decoded into
    /// `rx_buf`. It performs a CRC check using the CRC-CCITT (XModem) algorithm
    /// to verify message integrity, and if valid, extracts the protocol headers
    /// (to, from, id, flags) from the payload.
    ///
    /// # Behavior
    /// - Computes the CRC over the entire `rx_buf`
    /// - Compares the result against the expected terminal CRC value `0xF0B8`
    /// - If the CRC is invalid:
    ///   - Increments `rx_bad`
    ///   - Marks the message as invalid (`rx_buf_valid = false`)
    /// - If the CRC is valid:
    ///   - Extracts the four header fields from the buffer:
    ///     - `to` (at index 1)
    ///     - `from` (at index 2)
    ///     - `id` (at index 3)
    ///     - `flags` (at index 4)
    ///   - Increments `rx_good`
    ///   - Marks the buffer as valid (`rx_buf_valid = true`) if:
    ///     - The message is broadcast, or
    ///     - The receiver is in promiscuous mode, or
    ///     - The `to` address matches `this_address`
    ///
    /// # Notes
    /// - Assumes the message format aligns with the RadioHead convention, where
    ///   the first byte is the length and the next four are protocol headers.
    /// - This method also transitions the driver to [`AskMode::Idle`] as part of
    ///   RX completion handling.
    pub fn validate_rx_buf(&mut self) {
        let mut crc: u16 = 0xffff;
        for b in &self.pll.buf {
            crc = crc_ccitt_update(crc, b);
        }
        if crc != 0xf0b8 {
            // CRC when buffer and expected CRC are CRC'd
            // Reject and drop the message
            self.rx_bad += 1;
            self.rx_buf_valid = false;
            return;
        }

        // Extract the 4 headers that follow the message length
        self.rx_header_to = self.pll.buf[1];
        self.rx_header_from = self.pll.buf[2];
        self.rx_header_id = self.pll.buf[3];
        self.rx_header_flags = self.pll.buf[4];
        if self.promiscuous
            || self.rx_header_to == self.this_address
            || self.rx_header_to == BROADCAST_ADDRESS
        {
            self.rx_good += 1;
            self.rx_buf_valid = true;
        }
    }

    /// Returns a slice of the received message payload, if a valid message is available.
    ///
    /// This method checks whether a complete and validated message is available
    /// in the internal receive buffer (via [`AskDriver::availabile()`]), and if so,
    /// returns a slice containing just the message payload, excluding the protocol headers
    /// and CRC trailer.
    ///
    /// # Returns
    /// - `Some(Vec<u8>)`: The message payload
    /// - `None`: If no valid message has been received or the buffer is incomplete
    ///
    /// # Message Layout
    /// The internal buffer (`self.pll.buf`) is assumed to follow the structure:
    /// `[len, to, from, id, flags, ...payload..., crc_hi, crc_lo]`
    ///
    /// This function:
    /// - Reads the total message length from `self.pll.buf_len`
    /// - Calculates the offset of the payload by skipping the 4-byte header and 2-byte CRC
    /// - Returns a slice over just the user data portion of the message
    ///
    /// # Notes
    /// - This does **not** clear the receive buffer; call `self.set_mode_idle()` or
    ///   a specific buffer-reset method after processing.
    /// - The returned slice is valid until the next driver state change that modifies the RX buffer.
    ///
    /// # Panics
    /// May panic if `buf_len` is smaller than `ASK_HEADER_LEN + 1`, but this should never
    /// happen in a valid message.
    #[cfg(not(feature = "std"))]
    pub fn receive(&mut self) -> Option<Vec<u8, ASK_MAX_MESSAGE_LEN_USIZE>> {
        if !self.availabile() {
            return None;
        }

        let message_len: u8 = self.pll.buf_len - 2;
        self.rx_buf_valid = false;
        let message =
            Vec::from_slice(&self.pll.buf[((ASK_HEADER_LEN + 1) as usize)..(message_len as usize)])
                .unwrap();
        Some(message)
    }

    /// Returns a slice of the received message payload, if a valid message is available.
    ///
    /// This method checks whether a complete and validated message is available
    /// in the internal receive buffer (via [`AskDriver::availabile()`]), and if so,
    /// returns a slice containing just the message payload, excluding the protocol headers
    /// and CRC trailer.
    ///
    /// # Returns
    /// - `Some(Vec<u8>)`: The message payload slice
    /// - `None`: If no valid message has been received or the buffer is incomplete
    ///
    /// # Message Layout
    /// The internal buffer (`self.pll.buf`) is assumed to follow the structure:
    /// `[len, to, from, id, flags, ...payload..., crc_hi, crc_lo]`
    ///
    /// This function:
    /// - Reads the total message length from `self.pll.buf_len`
    /// - Calculates the offset of the payload by skipping the 4-byte header and 2-byte CRC
    /// - Returns a slice over just the user data portion of the message
    ///
    /// # Notes
    /// - This does **not** clear the receive buffer; call `self.set_mode_idle()` or
    ///   a specific buffer-reset method after processing.
    /// - The returned slice is valid until the next driver state change that modifies the RX buffer.
    ///
    /// # Panics
    /// May panic if `buf_len` is smaller than `ASK_HEADER_LEN + 1`, but this should never
    /// happen in a valid message.
    #[cfg(feature = "std")]
    pub fn receive(&mut self) -> Option<Vec<u8>> {
        if !self.availabile() {
            return None;
        }

        let message_len: u8 = self.pll.buf_len - 2;
        self.rx_buf_valid = false;
        let message =
            Vec::from(&self.pll.buf[(ASK_HEADER_LEN + 1) as usize..(message_len as usize)]);
        Some(message)
    }

    /// Advances the internal transmit/receive state machine by one timing tick.
    ///
    /// This function must be called at fixed intervals (e.g. every 62.5 µs).
    /// It handles either advancing transmit state or sampling the input during reception.
    ///
    /// # Timing
    /// Must be called precisely and regularly—ideally via timer interrupt or delay loop.
    ///
    /// # See also
    /// - [`SoftwarePLL`]
    pub fn tick(&mut self) {
        if self.mode == AskMode::Rx {
            // RX always sampled every tick
            self.pll.update(&mut self.rx);

            // Buffer the received byte or flag availability
            if self.pll.full && !self.pll.active {
                self.validate_rx_buf();
            }
        } else if self.mode == AskMode::Tx {
            // TX advances only every `ticks_per_bit` ticks
            self.tick_counter += 1;
            if self.tick_counter >= self.ticks_per_bit {
                self.tick_counter = 0;
                self.transmit_bit(); // Move to next TX bit
            }
        }
    }

    fn wait_packet_sent(&self) -> nb::Result<(), Infallible> {
        if self.mode == AskMode::Tx {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    /// Queues a single byte for transmission over the ASK RF link.
    ///
    /// This method should encode the byte (e.g. with 4b6b encoding) and load it
    /// into an internal transmit buffer for use by `tick()`.
    ///
    /// # Note
    /// Actual bit-by-bit transmission is handled incrementally in `tick()`.
    #[cfg(feature = "std")]
    pub fn send(&mut self, bytes: Vec<u8>) -> bool {
        let mut crc: u16 = 0xffff;
        let count: u8 = bytes.len() as u8 + 3 + ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

        if bytes.len() as u8 > ASK_MAX_MESSAGE_LEN {
            return false;
        }

        // Wait for transmitter to become available
        let _ = block!(self.wait_packet_sent());

        // Encode the message length
        crc = crc_ccitt_update(crc, &count);
        let _ = self.tx_buf.extend(encode_4b6b(count));

        // Encode the headers
        crc = crc_ccitt_update(crc, &self.tx_header_to);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_to));
        crc = crc_ccitt_update(crc, &self.tx_header_from);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_from));
        crc = crc_ccitt_update(crc, &self.tx_header_id);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_id));
        crc = crc_ccitt_update(crc, &self.tx_header_flags);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_flags));

        // Encode the message into 6 bit symbols. Each byte is converted into
        // 2 6-bit symbols, high nybble first, low nybble second
        for b in bytes {
            crc = crc_ccitt_update(crc, &b);
            let _ = self.tx_buf.extend(encode_4b6b(b));
        }

        // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
        // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
        // VW sends FCS as low byte then hi byte
        crc = !crc;
        let _ = self.tx_buf.push(SYMBOLS[((crc >> 4) & 0xf) as usize]);
        let _ = self.tx_buf.push(SYMBOLS[(crc & 0xf) as usize]);
        let _ = self.tx_buf.push(SYMBOLS[((crc >> 12) & 0xf) as usize]);
        let _ = self.tx_buf.push(SYMBOLS[((crc >> 8) & 0xf) as usize]);

        // Total number of 6-bit symbols to send
        self.tx_buf_len = self.tx_buf.len() as u8;

        // Start the low level interrupt handler sending symbols
        self.set_mode_tx();

        return true;
    }

    /// Queues a single byte for transmission over the ASK RF link.
    ///
    /// This method should encode the byte (e.g. with 4b6b encoding) and load it
    /// into an internal transmit buffer for use by `tick()`.
    ///
    /// # Note
    /// Actual bit-by-bit transmission is handled incrementally in `tick()`.
    #[cfg(not(feature = "std"))]
    pub fn send(&mut self, bytes: Vec<u8, ASK_MAX_MESSAGE_LEN_USIZE>) -> bool {
        let mut crc: u16 = 0xffff;
        let count: u8 = bytes.len() as u8 + 3 + ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

        if bytes.len() as u8 > ASK_MAX_MESSAGE_LEN {
            return false;
        }

        // Wait for transmitter to become available
        let _ = block!(self.wait_packet_sent());

        // Encode the message length
        crc = crc_ccitt_update(crc, &count);
        let _ = self.tx_buf.extend(encode_4b6b(count));

        // Encode the headers
        crc = crc_ccitt_update(crc, &self.tx_header_to);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_to));
        crc = crc_ccitt_update(crc, &self.tx_header_from);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_from));
        crc = crc_ccitt_update(crc, &self.tx_header_id);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_id));
        crc = crc_ccitt_update(crc, &self.tx_header_flags);
        let _ = self.tx_buf.extend(encode_4b6b(self.tx_header_flags));

        // Encode the message into 6 bit symbols. Each byte is converted into
        // 2 6-bit symbols, high nybble first, low nybble second
        for b in bytes {
            crc = crc_ccitt_update(crc, &b);
            let _ = self.tx_buf.extend(encode_4b6b(b));
        }

        // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
        // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
        // VW sends FCS as low byte then hi byte
        crc = !crc;
        let _ = self.tx_buf.push(SYMBOLS[((crc >> 4) & 0xf) as usize]);
        let _ = self.tx_buf.push(SYMBOLS[(crc & 0xf) as usize]);
        let _ = self.tx_buf.push(SYMBOLS[((crc >> 12) & 0xf) as usize]);
        let _ = self.tx_buf.push(SYMBOLS[((crc >> 8) & 0xf) as usize]);

        // Total number of 6-bit symbols to send
        self.tx_buf_len = self.tx_buf.len() as u8;

        // Start the low level interrupt handler sending symbols
        self.set_mode_tx();

        return true;
    }

    /// Advances to the next encoded bit in the transmission sequence.
    ///
    /// This should be called after every full bit interval, not every `tick()`.
    /// Used internally by the driver to separate fine-grained timing from bit-level modulation.
    ///
    /// # Note
    /// Not intended for direct user invocation.
    fn transmit_bit(&mut self) {
        // Send next bit
        // Symbols are sent LSB first
        // Finished sending the whole message? (after waiting one bit period
        // since the last bit)
        if self.tx_index >= self.tx_buf_len {
            self.tx_good += 1;
            self.set_mode_idle();
        } else {
            // bit = bit_to_send (Bitwise AND) (1 (Bitwise shift Left) tx_bit)
            // e.g. for bit_to_send = 4 = 00000100
            // tx_bit = 6
            // 0000001 (1) << 6 = 1000000 (64)
            // 0000100 & 1000000 = 0000000 (0 = false)
            // but for tx_bit = 3
            // 0000001 (1) << 3 = 0001000 (8)
            // 0001000 & 0001000 = 0001000 (8 = true)
            let bit = self.tx_buf[self.tx_index as usize] & (1 << self.tx_bit);
            self.tx_bit += 1;
            self.write_tx(bit != 0);
            if self.tx_bit >= 6 {
                self.tx_bit = 0;
                self.tx_index += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };

    #[test]
    fn test_driver_initialization() {
        let tx = PinMock::new(&[PinTransaction::set(PinState::Low)]);
        let rx = PinMock::new(&[]);
        let ptt = PinMock::new(&[]);

        let mut driver = AskDriver::new(tx, rx, Some(ptt), 8, Some(false), Some(false));

        assert_eq!(driver.mode, AskMode::Idle);
        driver.tx.done();
        driver.rx.done();
        let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
    }

    #[test]
    fn test_set_address() {
        let tx = PinMock::new(&[PinTransaction::set(PinState::Low)]);
        let rx = PinMock::new(&[]);
        let ptt = PinMock::new(&[]);
        let mut driver = AskDriver::new(tx, rx, Some(ptt), 8, Some(false), Some(false));

        driver.set_address(0x42);
        assert_eq!(driver.this_address, 0x42);
        driver.tx.done();
        driver.rx.done();
        let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
    }

    #[test]
    fn test_send_message_starts_transmission() {
        #[cfg(not(feature = "std"))]
        use heapless::Vec;
        #[cfg(feature = "std")]
        use std::vec::Vec;

        let tx = PinMock::new(&[PinTransaction::set(PinState::Low)]);
        let rx = PinMock::new(&[]);
        let ptt = PinMock::new(&[PinTransaction::set(PinState::High)]);

        let mut driver = AskDriver::new(tx, rx, Some(ptt), 8, Some(false), Some(false));
        let mut message = Vec::new();
        #[cfg(feature = "std")]
        message.extend_from_slice(b"Hi");
        #[cfg(not(feature = "std"))]
        let _ = message.extend_from_slice(b"Hi");

        assert!(driver.send(message));
        assert_eq!(driver.mode, AskMode::Tx);
        assert_eq!(driver.tx_buf.len(), 26); // (2 bytes + 4 headers + 1 count + 2 CRC bytes) * 2 (4b6b encoding) + 8 (preamble)
        driver.tx.done();
        driver.rx.done();
        let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_tick_transmit_advances_tx_state() {
        let tx_states = vec![
            0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
            0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1,
            0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0,
            1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0,
            1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0,
            0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0,
        ];
        let tx = PinMock::new(
            &tx_states
                .iter()
                .map(|&s| {
                    PinTransaction::set(if s == 0 {
                        PinState::Low
                    } else {
                        PinState::High
                    })
                })
                .collect::<Vec<_>>(),
        );
        let rx = PinMock::new(&[]);
        let ptt = PinMock::new(&[
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
        ]);

        let mut driver = AskDriver::new(tx, rx, Some(ptt), 2, Some(false), Some(false));
        let mut message: Vec<u8> = Vec::new();
        message.extend_from_slice(b"AB");
        assert!(driver.send(message));

        assert_eq!(
            driver.tx_buf,
            vec![
                42, 42, 42, 42, 42, 42, 56, 44, 13, 37, 52, 52, 52, 52, 13, 13, 13, 13, 22, 14, 22,
                19, 37, 14, 52, 41
            ]
        );

        assert!(driver.mode == AskMode::Tx);

        for _ in 0..(tx_states.len() * 3) {
            // 2 ticks per bit
            driver.tick();
        }

        assert_eq!(driver.tx_buf_len, 26);
        assert_eq!(driver.tx_good, 1);
        assert_eq!(driver.tx_index, 26);
        driver.tx.done();
        driver.rx.done();
        let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
    }

    #[test]
    fn test_receive_no_data_returns_none() {
        let tx = PinMock::new(&[
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::Low),
        ]);
        let rx = PinMock::new(&[]);
        let ptt = PinMock::new(&[PinTransaction::set(PinState::Low)]);

        let mut driver = AskDriver::new(tx, rx, Some(ptt), 8, Some(false), Some(false));
        assert!(!driver.availabile());
        assert!(driver.receive().is_none());
        driver.tx.done();
        driver.rx.done();
        let _ = driver.ptt.as_mut().map(|ptt| ptt.done());
    }
}
