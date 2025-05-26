//! 4b6b symbol encoding and decoding for ASK/OOK message framing.
//!
//! This module implements a DC-balanced 4-to-6 bit encoding scheme used
//! in the RadioHead-style ASK protocol. It provides functions to encode
//! 8-bit bytes into 6-bit symbols and decode them back, as well as
//! buffer-level conversions for message transmission and reception.
//!
//! ## Purpose
//!
//! ASK/OOK RF communication is susceptible to long runs of 0s or 1s,
//! which can confuse receiver timing recovery. 4b6b encoding:
//!
//! - Ensures **DC balance**: each 6-bit symbol has 3 ones and 3 zeroes
//! - Prevents long sequences of the same bit
//! - Facilitates reliable PLL synchronization on the receiver side
//!
//! ## Symbol Table
//!
//! The encoder maps each 4-bit nibble to a unique 6-bit balanced symbol,
//! taken from a fixed lookup table. The decoder uses a reverse-lookup table
//! to reconstruct the original nibbles. Only 16 of the 64 possible 6-bit
//! values are valid in this scheme.
//!
//! ## Functions
//!
//! - [`encode_4b6b`]: Converts a single byte into two 6-bit symbols
//! - [`decode_6b4b`]: Recovers a byte from two 6-bit symbols
//! - [`encode_buffer`]: Encodes a full byte slice to symbol slice
//! - [`decode_buffer`]: Decodes a symbol slice back into original bytes
//!
//! ## Usage
//!
//! These functions are used as part of the transmit and receive pipeline,
//! transforming messages to/from their encoded representation just before
//! modulation and just after demodulation, respectively.
//!
//! ## Limitations
//!
//! - Only valid encoded 6-bit symbols are supported; decoding invalid values returns `None`
//! - Input to `decode_buffer` must be even-length; symbol pairs are required

static SYMBOLS: [u8; 16] = [
    0xd, 0xe, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34,
];
static REV_SYMBOLS: [Option<u8>; 64] = [
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    Some(0),
    Some(1),
    None,
    None,
    None,
    None,
    Some(2),
    None,
    Some(3),
    Some(4),
    None,
    None,
    Some(5),
    Some(6),
    None,
    Some(7),
    None,
    None,
    None,
    None,
    None,
    None,
    Some(8),
    None,
    Some(9),
    Some(10),
    None,
    None,
    Some(11),
    Some(12),
    None,
    Some(13),
    None,
    None,
    None,
    None,
    None,
    Some(14),
    None,
    Some(15),
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
];

/// Encodes an 8-bit byte into two 6-bit symbols using 4b6b encoding.
pub fn encode_4b6b(byte: u8) -> [u8; 2] {
    let high = (byte >> 4) & 0x0F;
    let low = byte & 0x0F;
    [SYMBOLS[high as usize], SYMBOLS[low as usize]]
}

/// Decodes two 6-bit symbols back into the original byte using the reverse symbol table.
///
/// Returns `None` if either symbol is invalid (not part of the encoding table).
pub fn decode_6b4b(sym_hi: u8, sym_lo: u8) -> Option<u8> {
    let high = REV_SYMBOLS.get(sym_hi as usize)?.clone()?;
    let low = REV_SYMBOLS.get(sym_lo as usize)?.clone()?;
    Some((high << 4) | low)
}

/// Encodes an array of 8-bit bytes into the array `output` as 6-bit symbols using 4b6b encoding.
///
/// # Arguments
/// - `&[u8]` : The input buffer slice
/// - `&mut [u8]` : The output buffer
///
/// # Returns
/// The length of the encoded array
pub fn encode_buffer(input: &[u8], output: &mut [u8]) -> usize {
    let mut i = 0;
    for &byte in input {
        let [hi, lo] = encode_4b6b(byte);
        output[i] = hi;
        output[i + 1] = lo;
        i += 2;
    }
    i
}

/// Decodes an array of 6-bit symbol pairs back into the original byte using the reverse symbol table.
///
/// # Arguments
/// - `&[u8]` : The input buffer slice
/// - `&mut [u8]` : The output buffer
///
/// # Returns
/// The optional length of the output buffer.
/// Returns `None` if the input buffer is an uneven length. (As the 6-bit encoded data comes in
/// pairs.)
pub fn decode_buffer(input: &[u8], output: &mut [u8]) -> Option<usize> {
    let mut i = 0;
    if input.len() % 2 != 0 {
        return None;
    }
    for chunk in input.chunks(2) {
        if chunk.len() < 2 {
            return None;
        }
        let byte = decode_6b4b(chunk[0], chunk[1])?;
        output[i] = byte;
        i += 1;
    }
    Some(i)
}
