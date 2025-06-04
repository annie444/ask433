//! Software PLL for ASK/OOK signal demodulation.
//!
//! This module implements a software phase-locked loop used to reconstruct
//! digital data from 433 MHz ASK-modulated signals. It works by sampling the
//! receiver output at high frequency, aligning to bit transitions, and
//! recovering bits using a majority-vote integrator.

use embedded_hal::digital::InputPin;

#[cfg(not(feature = "std"))]
use heapless::Vec;
#[cfg(feature = "std")]
use std::vec::Vec;

#[cfg(not(feature = "std"))]
use crate::consts::ASK_MAX_BUF_LEN_USIZE;
use crate::consts::{ASK_MAX_PAYLOAD_LEN, ASK_START_SYMBOL};
use crate::encoding::decode_6b4b;

#[derive(Debug)]
/// A simple digital phase-locked loop for demodulating ASK signals.
///
/// This PLL samples the input pin at a fixed frequency (e.g., 8x per bit),
/// detects transitions to adjust phase, and integrates high samples to decide bits.
///
// - `ramp`: tracks phase position in the current bit interval
// - `integrator`: counts HIGH samples
pub struct SoftwarePLL {
    /// Last 12 bits received, so we can look for the start symbol
    bits: u16,

    /// Phase accumulator used to track position within a single bit interval.
    ///
    /// This is incremented on each `tick()` and wraps around at `ramp_len`.
    /// Signal edges (transitions) adjust how fast it is incremented to maintain phase lock.
    ramp: u16,

    /// Counts how many of the samples in the current bit interval were high (signal present).
    ///
    /// Reset to 0 at the beginning of each bit period. If the count exceeds
    /// a threshold (e.g., 5 of 8), the current bit is decoded as a 1.
    integrator: u8,

    /// Tracks the previous sample value to detect rising or falling edges.
    ///
    /// Transitions are used to adjust ramp incrementing and keep the PLL in sync.
    last_sample: bool,

    /// Tracks how many bits have been shifted into `bitstream`.
    ///
    /// Once this reaches 8, a complete byte is available and can be
    /// consumed by higher-level protocol code.
    pub bit_count: u8,

    /// The number of phase units per bit.
    ///
    /// This defines when the ramp wraps and thus determines the full duration of one bit.
    /// Set to `ticks_per_bit * 20`.
    ramp_len: u16,

    /// Default amount to increment the ramp per sample tick.
    ///
    /// Typically `ramp_len / ticks_per_bit`. Represents no timing correction.
    ramp_inc: u16,

    /// Ramp increment used when a transition is detected earlier than expected.
    ///
    /// Slightly smaller than `ramp_inc` to slow down the PLL clock ("retard" it).
    ramp_retard: u16,

    /// Ramp increment used when a transition is detected later than expected.
    ///
    /// Slightly larger than `ramp_inc` to speed up the PLL clock ("advance" it).
    ramp_advance: u16,

    /// Whether the read bits should be inverted.
    /// e.g. HIGH => LOW
    inverted: bool,

    /// Flag indicates if we have seen the start symbol of a new message and are
    /// in the processes of reading and decoding it
    pub active: bool,

    /// The current length of the reveiving buffer
    pub buf_len: u8,

    /// The bit count of 4-bit chunks expected to be received
    /// as sent in the message header.
    count: u8,

    /// The number of bad messages received
    pub bad: u16,

    /// Tracks whether the bitstream has accumulated a full byte.
    /// This is the same as when the `bit_count == 6`
    pub full: bool,

    /// Accumulates decoded bits until a full byte is received.
    ///
    /// This is built bit-by-bit using majority vote sampling of each full bit period.
    #[cfg(not(feature = "std"))]
    pub buf: Vec<u8, ASK_MAX_BUF_LEN_USIZE>,

    /// Accumulates decoded bits until a full byte is received.
    ///
    /// This is built bit-by-bit using majority vote sampling of each full bit period.
    #[cfg(feature = "std")]
    pub buf: Vec<u8>,
}
impl SoftwarePLL {
    /// Creates a new, zeroed software PLL instance.
    ///
    /// Resets ramp, integrator, and bit reconstruction state.
    pub fn new(ticks_per_bit: u8, inverted: bool) -> Self {
        let ramp_len = (ticks_per_bit as u16) * 20;
        let ramp_inc = ramp_len / ticks_per_bit as u16;
        let ramp_adjust = 9;
        let ramp_retard = ramp_inc - ramp_adjust;
        let ramp_advance = ramp_inc + ramp_adjust;
        Self {
            ramp: 0,
            integrator: 0,
            last_sample: false,
            bit_count: 0,
            active: false,
            bits: 0,
            buf_len: 0,
            count: 0,
            full: false,
            buf: Vec::new(),
            ramp_len,
            ramp_inc,
            ramp_retard,
            ramp_advance,
            inverted,
            bad: 0,
        }
    }

    /// Updates the PLL state using the current RX input sample.
    ///
    /// Should be called once per timing tick. This reads the RX pin, counts high
    /// samples, adjusts timing on transitions, and pushes decoded bits into `bitstream`.
    ///
    /// # Arguments
    /// - `rx`: Input pin to sample
    pub fn update<RX: InputPin>(&mut self, rx: &mut RX) {
        // Poll the current state
        let sample = if self.inverted {
            !rx.is_high().unwrap_or(false)
        } else {
            rx.is_high().unwrap_or(false)
        };
        // Ensure we're integrating each sample
        if sample {
            self.integrator += 1;
        }

        // If the current state is different than previous
        if sample != self.last_sample {
            // Transition, advance if ramp > 80, retard if < 80
            self.ramp += if self.ramp < self.ramp_len / 2 {
                self.ramp_retard
            } else {
                self.ramp_advance
            };
            // Keep track of the current state.
            self.last_sample = sample;
        } else {
            // No transition
            // Advance ramp by standard 20 (== 160/8 samples)
            self.ramp += self.ramp_inc;
        }

        if self.ramp >= self.ramp_len {
            // Add this to the 12th bit of _rxBits, LSB first
            // The last 12 bits are kept
            self.bits >>= 1;

            // Check the integrator to see how many samples in this cycle were high.
            // If < 5 out of 8, then its declared a 0 bit, else a 1;
            if self.integrator >= 5 {
                self.bits |= 0x800;
            }

            self.ramp -= self.ramp_len;
            self.integrator = 0;

            if self.active {
                // We have the start symbol and now we are collecting message bits,
                // 6 per symbol, each which has to be decoded to 4 bits
                self.bit_count += 1;
                if self.bit_count >= 12 {
                    // Have 12 bits of encoded message == 1 byte encoded
                    // Decode as 2 lots of 6 bits into 2 lots of 4 bits
                    // The 6 lsbits are the high nybble
                    let this_byte =
                        decode_6b4b(&((self.bits & 0x3f) as u8), &((self.bits >> 6) as u8));
                    // The first decoded byte is the byte count of the following message
                    // the count includes the byte count and the 2 trailing FCS bytes
                    // REVISIT: may also include the ACK flag at 0x40
                    if self.buf_len == 0 {
                        // The first byte is the byte count
                        // Check it for sensibility. It cant be less than 7, since it
                        // includes the byte count itself, the 4 byte header and the 2 byte FCS
                        self.count = this_byte;
                        if self.count < 7 || self.count > ASK_MAX_PAYLOAD_LEN {
                            // Stupid message length, drop the whole thing
                            self.active = false;
                            self.bad += 1;
                            return;
                        }
                    }
                    self.buf.push(this_byte);
                    self.buf_len += 1;

                    if self.buf_len >= self.count {
                        // Got all the bytes now
                        self.active = false;
                        self.full = true;
                    }
                    self.bit_count = 0;
                }
            } else if self.bits == ASK_START_SYMBOL {
                self.active = true;
                self.bit_count = 0;
                self.buf_len = 0;
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
    fn test_pll_initialization_defaults() {
        let pll = SoftwarePLL::new(8, false);
        assert_eq!(pll.ramp, 0);
        assert_eq!(pll.integrator, 0);
        assert_eq!(pll.bit_count, 0);
        assert_eq!(pll.active, false);
        assert_eq!(pll.full, false);
        assert_eq!(pll.buf_len, 0);
    }

    #[test]
    fn test_pll_updates_on_tick_with_high_signal() {
        let expectations = [PinTransaction::get(PinState::High)];
        let mut rx = PinMock::new(&expectations);

        let mut pll = SoftwarePLL::new(8, false);
        pll.update(&mut rx);

        // Should increment integrator if sample was high
        assert!(pll.integrator > 0);
        rx.done();
    }

    #[test]
    fn test_pll_detects_start_symbol() {
        let mut pll = SoftwarePLL::new(8, false);
        pll.bits = ASK_START_SYMBOL << 1; // Shifted to simulate previous bits
        pll.ramp = pll.ramp_len;

        // Should detect start symbol and set active
        let expectations = [PinTransaction::get(PinState::High)];
        let mut rx = PinMock::new(&expectations);
        pll.update(&mut rx);

        assert!(pll.active);
        rx.done();
    }

    #[test]
    fn test_pll_inverts_signal_when_flagged() {
        let expectations = [PinTransaction::get(PinState::High)];
        let mut rx = PinMock::new(&expectations);

        let mut pll = SoftwarePLL::new(8, true);
        pll.update(&mut rx);

        // Inverted high is low: integrator should remain zero
        assert_eq!(pll.integrator, 0);
        rx.done();
    }
}
