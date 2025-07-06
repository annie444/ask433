use crate::crc::crc_ccitt_update;
use crate::driver::AskDriver;

pub struct RadioHead {
    pub driver: AskDriver,
    pub rx_header_to: u8,
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
}

impl RadioHead {
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
}
