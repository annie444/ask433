# ask433

[![Crates.io](https://img.shields.io/crates/v/ask433.svg)](https://crates.io/crates/ask433)
[![Docs.rs](https://img.shields.io/docsrs/ask433)](https://docs.rs/ask433)

**A portable, no_std Amplitude Shift Keying (ASK/OOK) modem driver for 433 MHz RF transceivers.**

This crate implements a full software modem compatible with cheap 433 MHz ASK/OOK transceivers (like FS1000A and XY-MK-5V), designed for use on both AVR (e.g. Arduino Uno) and ARM Cortex-M microcontrollers.

---

## Features

- 🛠 Built with [`embedded-hal`] traits for maximum portability
- 📡 Software-based ASK modem with transmit & receive support
- 🧠 Demodulation via a software phase-locked loop (PLL)
- 🧾 RadioHead-compatible 4b6b symbol encoding
- 📦 Message framing with headers, preamble, and CRC validation
- 💤 Sleep, idle, transmit, receive, and CAD modes via `AskMode`
- 💥 Interrupt-safe using `critical-section`
- 🔒 `no_std` and embedded-friendly

---

## How It Works

Each `AskDriver` instance manages the RF link using a timer-driven `tick()` interface. The driver samples the RX pin and modulates the TX pin at a fixed interval (usually every 62.5 µs for 2 kbps). All state — including framing, CRC, and symbol decoding — is handled in software.

```rust
use ask433::driver::AskDriver;
use embedded_hal::blocking::delay::DelayUs;
use heapless::Vec;

let mut driver = AskDriver::new(tx, rx, None, 8, None, None); // 8 ticks per bit (2kbps)
let mut delay = ...; // Your DelayUs provider

let msg = Vec::from_slice("Hello, World!".as_bytes()).unwrap();
driver.send(msg);

loop {
    driver.tick();
    delay.delay_us(63).ok();
    if driver.available() {
        if let Some(payload) = driver.receive() {
            // Handle message payload
        }
    }
}
```

---

## Platform Support

| Platform          | Status | Notes                                               |
| ----------------- | ------ | --------------------------------------------------- |
| Arduino Uno (AVR) | ✅     | Tested with `arduino-hal`                           |
| STM32, nRF, etc.  | ✅     | Works with HALs supporting `DelayUs` or `CountDown` |
| RTIC              | ✅     | Compatible via tick() ISR                           |

You can use either:
• `delayus-loop`: a blocking loop to call `tick()`
• `timer-isr`: an interrupt handler that calls `tick()`

See `src/timer.rs` for examples.

---

## Buffer Sizes and Limits

- `ASK_MAX_PAYLOAD_LEN`: 67 bytes total (RadioHead compatible)
- `ASK_HEADER_LEN`: 4 bytes (to, from, id, flags)
- `ASK_PREAMBLE_LEN`: 8 bytes (0x55 training pattern)
- `ASK_MAX_MESSAGE_LEN`: 60 bytes usable application data
- `ASK_MAX_BUF_LEN`: Total transmission buffer (preamble + encoded)

---

## Safety & Performance

- Fully interrupt-safe using `critical-section`
- No heap allocation or dynamic dispatch
- Efficient symbol encoding using lookup tables
- Overflow-safe `millis()`-based timers for duration tracking

---

## License

Licensed under either of:
• MIT License (LICENSE-MIT or http://opensource.org/licenses/MIT)
• Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)

---

## Contributing

Pull requests and issue reports are welcome! This project is in early development and feedback is appreciated — especially around portability, signal reliability, and architecture improvements.

---

## Acknowledgements

• Inspired by [RadioHead](https://github.com/epsilonrt/RadioHead) and [VirtualWire](https://github.com/lsongdev/VirtualWire)
• Uses `4b6b` encoding patterns compatible with FS1000A-style radios
• Built with <3 for embedded systems and open hardware
