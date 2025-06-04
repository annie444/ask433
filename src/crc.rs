pub(crate) fn crc_ccitt_update(crc: u16, data: &u8) -> u16 {
    let mut d = *data as u16;
    d = d ^ lo8(crc);
    d = d ^ (d << 4);
    d = (d as u8) as u16; // Ensure d is treated as a u8 for the next operations

    ((d << 8) | hi8(crc)) ^ (((d >> 4) as u8) as u16) ^ (d << 3)
}

pub(crate) fn lo8(x: u16) -> u16 {
    x & 0xff
}

pub(crate) fn hi8(x: u16) -> u16 {
    x >> 8
}
