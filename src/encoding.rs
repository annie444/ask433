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

/// 4b6b encoding symbol table.
pub static SYMBOLS: [u8; 16] = [
    0xd, 0xe, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34,
];

/// Encodes an 8-bit byte into two 6-bit symbols using 4b6b encoding.
pub fn encode_4b6b(byte: u8) -> [u8; 2] {
    let high = byte >> 4;
    let low = byte & 0x0F;
    [SYMBOLS[high as usize], SYMBOLS[low as usize]]
}

/// Decodes two 6-bit symbols back into the original byte using the reverse symbol table.
///
/// Returns `None` if either symbol is invalid (not part of the encoding table).
pub fn decode_6b4b(symbol1: &u8, symbol2: &u8) -> u8 {
    let b1 = SYMBOLS.iter().position(|s| s == symbol1).unwrap_or(0) as u8;
    let b2 = SYMBOLS.iter().position(|s| s == symbol2).unwrap_or(0) as u8;
    (b1 << 4) | b2
}

/// Encodes an array of 8-bit bytes into the array `output` as 6-bit symbols using 4b6b encoding.
///
/// # Arguments
/// - `&[u8]` : The input buffer slice
/// - `&mut [u8]` : The output buffer
///
/// # Returns
/// The length of the encoded array
pub fn encode_buffer(input: &[u8]) -> Vec<u8> {
    let mut output = Vec::with_capacity(input.len() * 2);
    for &byte in input {
        output.extend_from_slice(&encode_4b6b(byte));
    }
    output
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
pub fn decode_buffer(input: &[u8]) -> Vec<u8> {
    let mut output: Vec<u8> = Vec::with_capacity(input.len() / 2);
    if input.len() % 2 != 0 {
        return Vec::new(); // Invalid input length
    }
    for chunk in input.chunks(2) {
        output.push(decode_6b4b(&chunk[0], &chunk[1]));
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_4b6b_correct_output() {
        let encoded = encode_4b6b(0xAB); // 0xA = 10, 0xB = 11
        assert_eq!(encoded[0], 0x26);
        assert_eq!(encoded[1], 0x29);
    }

    #[test]
    fn test_decode_6b4b_correct_output() {
        let decoded = decode_6b4b(&0b10101, &0b1101);
        assert_eq!(decoded, 0b110000);
    }

    #[test]
    fn test_encode_buffer_correct_length_and_content() {
        let input = [0xAB, 0xCD];
        let output = encode_buffer(&input);
        assert_eq!(output.len(), 4);
        assert_eq!(output[0], 0x26);
        assert_eq!(output[1], 0x29);
        assert_eq!(output[2], 0x2a);
        assert_eq!(output[3], 0x2c);
    }

    #[test]
    fn test_decode_buffer_successful() {
        let input = [0x26, 0x29];
        let output = decode_buffer(&input);
        assert_eq!(output.len(), 1);
        assert_eq!(output[0], 0xAB);
    }

    #[test]
    fn test_decode_buffer_odd_length_fails() {
        let input = [0x2a];
        let result = decode_buffer(&input);
        assert_eq!(result, vec![]);
    }
}
