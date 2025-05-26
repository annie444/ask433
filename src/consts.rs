//! Constants used across the ASK protocol implementation.
//!
//! This module defines various protocol-wide constants used for
//! buffer sizing, header layout, preamble control, and flag handling.
//!
//! These constants are based on the RadioHead ASK/OOK framing conventions,
//! adapted for embedded use in constrained environments.
//!
//! ## Key Concepts
//!
//! - **Flags**: Split into protocol-reserved and application-specific bits.
//! - **Headers**: Fixed 4-byte format used to identify sender, receiver, and metadata.
//! - **Payload Limits**: Derived from maximum packet size with allowance for headers and CRC.
//! - **Preamble**: Byte pattern used to help synchronize the software PLL in the receiver.
//! - **Buffer Sizing**: Calculated based on encoded message length and preamble requirements.
//!
//! These values should be used wherever framing or buffer logic is implemented to ensure
//! consistent message boundaries and timing alignment.

use core::u8;

/// Bitmask for protocol-level flags reserved for future use.
///
/// These upper 4 bits of the flags byte are reserved by the protocol
/// and should not be modified by application code.
pub const ASK_FLAGS_RESERVED: i16 = 0xf0;

/// Bitmask for application-specific flags.
///
/// These lower 4 bits of the flags byte can be freely used by
/// application-layer logic for metadata, priority markers, etc.
pub const ASK_FLAGS_APPLICATION_SPECIFIC: i16 = 0x0f;

/// Constant for a "no flags" state (all bits zero).
///
/// Used when a message has no control or application flags set.
pub const ASK_FLAGS_NONE: i16 = 0;

/// Length (in bytes) of the fixed-length packet header.
///
/// This typically includes fields like `to`, `from`, `id`, and `flags`.
pub const ASK_HEADER_LEN: u8 = 4;

/// Maximum total length (in bytes) of the raw RF message payload,
/// including header, data, and trailer (e.g., checksum).
///
/// This includes everything before symbol encoding (4b6b).
pub const ASK_MAX_PAYLOAD_LEN: u8 = 67;

/// Length (in bytes) of the transmitted preamble used to train the receiver PLL.
///
/// Each byte is encoded as two 6-bit symbols during transmission.
/// This is typically filled with `0x55` (alternating bits) to help
/// the receiver synchronize its sampling phase.
pub const ASK_PREAMBLE_LEN: u8 = 8;

/// Maximum size (in bytes) of user message content.
///
/// This is derived from the maximum payload size minus header and trailer bytes (e.g., CRC).
pub const ASK_MAX_MESSAGE_LEN: u8 = ASK_MAX_PAYLOAD_LEN - ASK_HEADER_LEN - 3;

/// See [`ASK_MAX_MESSAGE_LEN`](crate::consts::ASK_MAX_MESSAGE_LEN)
pub const ASK_MAX_MESSAGE_LEN_USIZE: usize = ASK_MAX_MESSAGE_LEN as usize;

/// Maximum size (in bytes) of the full transmission buffer, after encoding and preamble.
///
/// Includes space for the 4b6b-encoded payload and the preamble prefix.
/// Each payload byte becomes two encoded symbols, hence the `* 2`.
pub const ASK_MAX_BUF_LEN: u8 = (ASK_MAX_PAYLOAD_LEN * 2) + ASK_PREAMBLE_LEN;

/// See [ASK_MAX_BUF_LEN](crate::consts::ASK_MAX_BUF_LEN)
pub const ASK_MAX_BUF_LEN_USIZE: usize = ASK_MAX_BUF_LEN as usize;

/// The default address for the RF module.
pub const BROADCAST_ADDRESS: u8 = u8::MAX;

/// The START symbol prepended to all messages
pub const ASK_START_SYMBOL: u16 = 0xb38;
