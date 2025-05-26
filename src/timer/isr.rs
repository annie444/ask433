#[cfg(not(feature = "std"))]
use crate::consts::ASK_MAX_MESSAGE_LEN_USIZE;
use crate::driver::AskDriver;
use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal::digital::{InputPin, OutputPin};
#[cfg(not(feature = "std"))]
use heapless::Vec;

/// Used to initialize the global static `AskDriver` for use with
/// `critical_section`.
///
/// # Returns
/// * An empty mutable ref-cell
///
/// # Example
/// ```rust
/// use ask433::driver::AskDriver;
/// use core::cell::RefCell;
/// use critical_section::Mutex;
/// use embedded_hal::digital::{InputPin, OutputPin};
/// use some_hal::{PD1, PD2};
///
/// static ASK_DRIVER: Mutex<RefCell<Option<AskDriver<PD1, PD2>>>> =
///     global_ask_driver_init::<PD1, PD2>()
/// ```
pub const fn global_ask_driver_init<TX: OutputPin, RX: InputPin, PTT: OutputPin>()
-> Mutex<RefCell<Option<AskDriver<TX, RX, PTT>>>> {
    Mutex::new(RefCell::new(None))
}

/// Sets up the `critical_section::with` callback.
///
/// # Arguments
/// * The global static `AskDriver`
/// * The tx pin
/// * The rx pin
/// * The number of ticks per bit such that:
///     `interrupt frequency / ticks per bit = 2000 bits per second`
///     e.g. For the Atmega328P with an interrupt frequency of `~62.5µs`:
///         ```rust
///         // 8 ticks / bit = 1 second / 2000 bits * 1 tick / 6.25e-5 seconds
///         const TICKS_PER_BIT: u8 = (1/2000)*(1/625e-7_f32) as u8
///         ```
///# Example
/// ```rust
/// main() {
///     global_ask_driver_setup(ASK_DRIVER, tx, rx, 8);
/// }
/// ```
pub fn global_ask_driver_setup<TX: OutputPin, RX: InputPin, PTT: OutputPin>(
    global_driver: &'static Mutex<RefCell<Option<AskDriver<TX, RX, PTT>>>>,
    tx: TX,
    rx: RX,
    ptt: Option<PTT>,
    ticks_per_bit: &'static u8,
    ptt_inverted: Option<bool>,
    rx_inverted: Option<bool>,
) {
    critical_section::with(|cs| {
        let _ = global_driver.borrow(cs).replace(Some(AskDriver::new(
            tx,
            rx,
            ptt,
            *ticks_per_bit,
            ptt_inverted,
            rx_inverted,
        )));
    });
}

/// Runs the tick at each interrupt
///
/// # Arguments
/// * The global static `AskDriver`
///# Example
/// ```rust
/// #[interrupt]
/// fn TIM2() {
///     global_ask_driver_tick(ASK_DRIVER);
/// }
/// ```
pub fn global_ask_timer_tick<TX: OutputPin, RX: InputPin, PTT: OutputPin>(
    global_driver: &'static Mutex<RefCell<Option<AskDriver<TX, RX, PTT>>>>,
) {
    critical_section::with(|cs| {
        if let Some(driver) = global_driver.borrow(cs).borrow_mut().as_mut() {
            driver.tick();
        }
    });
}

/// Attempts to receive a message from a global `AskDriver` instance wrapped in a `Mutex`.
///
/// This function checks whether a valid and complete message is available using the
/// driver's `availabile()` method. If a message is present, it returns a heapless `Vec`
/// containing the decoded payload. If the buffer is invalid or incomplete, `None` is returned.
///
/// # Type Parameters
/// - `TX`: Type of the TX pin (must implement `OutputPin`)
/// - `RX`: Type of the RX pin (must implement `InputPin`)
/// - `PTT`: Type of the Push-To-Talk (PTT) control pin (must implement `OutputPin`)
///
/// # Arguments
/// - `global_driver`: A reference to a global `Mutex<RefCell<Option<AskDriver>>>`
///   that wraps the ASK driver state, typically created using `critical_section`.
///
/// # Returns
/// - `Some(Vec<u8>)`: A heapless vector containing the decoded message payload
/// - `None`: If no valid message is currently available
///
/// # Safety
/// - This function must be called from a context where `critical_section` access is safe,
///   such as inside `main()` or an interrupt-free routine.
///
/// # Example
/// ```rust
/// if let Some(msg) = receive_from_global_ask(&ASK_DRIVER) {
///     // Use the received message
/// }
/// ```
///
/// # See also
/// - [`AskDriver::availabile()`]
/// - [`AskDriver::receive()`]
#[cfg(not(feature = "std"))]
pub fn receive_from_global_ask<TX: OutputPin, RX: InputPin, PTT: OutputPin>(
    global_driver: &'static Mutex<RefCell<Option<AskDriver<TX, RX, PTT>>>>,
) -> Option<Vec<u8, ASK_MAX_MESSAGE_LEN_USIZE>> {
    critical_section::with(|cs| {
        let mut guard = global_driver.borrow(cs).borrow_mut();
        let driver = guard.as_mut()?;
        if driver.availabile() {
            driver.receive()
        } else {
            None
        }
    })
}

/// Attempts to receive a message from a global `AskDriver` instance wrapped in a `Mutex`.
///
/// This function checks whether a valid and complete message is available using the
/// driver's `availabile()` method. If a message is present, it returns a heapless `Vec`
/// containing the decoded payload. If the buffer is invalid or incomplete, `None` is returned.
///
/// # Type Parameters
/// - `TX`: Type of the TX pin (must implement `OutputPin`)
/// - `RX`: Type of the RX pin (must implement `InputPin`)
/// - `PTT`: Type of the Push-To-Talk (PTT) control pin (must implement `OutputPin`)
///
/// # Arguments
/// - `global_driver`: A reference to a global `Mutex<RefCell<Option<AskDriver>>>`
///   that wraps the ASK driver state, typically created using `critical_section`.
///
/// # Returns
/// - `Some(Vec<u8>)`: A vector containing the decoded message payload
/// - `None`: If no valid message is currently available
///
/// # Safety
/// - This function must be called from a context where `critical_section` access is safe,
///   such as inside `main()` or an interrupt-free routine.
///
/// # Example
/// ```rust
/// if let Some(msg) = receive_from_global_ask(&ASK_DRIVER) {
///     // Use the received message
/// }
/// ```
///
/// # See also
/// - [`AskDriver::availabile()`]
/// - [`AskDriver::receive()`]
#[cfg(feature = "std")]
pub fn receive_from_global_ask<TX: OutputPin, RX: InputPin, PTT: OutputPin>(
    global_driver: &'static Mutex<RefCell<Option<AskDriver<TX, RX, PTT>>>>,
) -> Option<Vec<u8>> {
    critical_section::with(|cs| {
        let mut guard = global_driver.borrow(cs).borrow_mut();
        let driver = guard.as_mut()?;
        if driver.availabile() {
            driver.receive()
        } else {
            None
        }
    })
}

/// Sends a message using a global `AskDriver` instance wrapped in a `Mutex`.
///
/// This function attempts to send the provided message buffer using the global
/// ASK driver. If the driver is currently idle and the message length is valid,
/// it transitions to transmit mode and begins sending the encoded message
/// (including headers, preamble, and CRC).
///
/// # Type Parameters
/// - `TX`: The TX pin type (must implement `OutputPin`)
/// - `RX`: The RX pin type (must implement `InputPin`)
/// - `PTT`: The Push-To-Talk (PTT) control pin type (must implement `OutputPin`)
///
/// # Arguments
/// - `global_driver`: A reference to a global `Mutex<RefCell<Option<AskDriver>>>`,
///   typically declared using [`init_global_ask_driver!`] and initialized via [`setup_global_ask_driver!`].
/// - `vec`: A `heapless::Vec<u8>` containing the payload to send. Must not exceed [`ASK_MAX_MESSAGE_LEN`].
///
/// # Returns
/// - `true` if the message was successfully queued for transmission
/// - `false` if the driver was uninitialized or busy
///
/// # Example
/// ```rust
/// let mut vec: Vec<u8, ASK_MAX_MESSAGE_LEN_USIZE> = Vec::new();
/// vec.extend_from_slice(b"Hello").unwrap();
/// let sent = send_from_global_ask(&ASK_DRIVER, vec);
/// if sent {
///     // Message is being transmitted
/// }
/// ```
///
/// # Notes
/// - This function does not block. Transmission occurs incrementally via repeated `tick()` calls.
/// - Returns `false` if the driver is uninitialized or already transmitting a message.
/// - The PTT pin is asserted automatically during transmission.
///
/// # See also
/// - [`AskDriver::send()`]
#[cfg(not(feature = "std"))]
pub fn send_from_global_ask<TX: OutputPin, RX: InputPin, PTT: OutputPin>(
    global_driver: &'static Mutex<RefCell<Option<AskDriver<TX, RX, PTT>>>>,
    vec: Vec<u8, ASK_MAX_MESSAGE_LEN_USIZE>,
) -> bool {
    critical_section::with(|cs| {
        if let Some(driver) = global_driver.borrow(cs).borrow_mut().as_mut() {
            driver.send(vec)
        } else {
            false
        }
    })
}

/// Sends a message using a global `AskDriver` instance wrapped in a `Mutex`.
///
/// This function attempts to send the provided message buffer using the global
/// ASK driver. If the driver is currently idle and the message length is valid,
/// it transitions to transmit mode and begins sending the encoded message
/// (including headers, preamble, and CRC).
///
/// # Type Parameters
/// - `TX`: The TX pin type (must implement `OutputPin`)
/// - `RX`: The RX pin type (must implement `InputPin`)
/// - `PTT`: The Push-To-Talk (PTT) control pin type (must implement `OutputPin`)
///
/// # Arguments
/// - `global_driver`: A reference to a global `Mutex<RefCell<Option<AskDriver>>>`,
///   typically declared using [`init_global_ask_driver!`] and initialized via [`setup_global_ask_driver!`].
/// - `vec`: A `Vec<u8>` containing the payload to send.
///
/// # Returns
/// - `true` if the message was successfully queued for transmission
/// - `false` if the driver was uninitialized or busy
///
/// # Example
/// ```rust
/// let mut vec: Vec<u8> = Vec::new();
/// vec.extend(b"Hello");
/// let sent = send_from_global_ask(&ASK_DRIVER, vec);
/// if sent {
///     // Message is being transmitted
/// }
/// ```
///
/// # Notes
/// - This function does not block. Transmission occurs incrementally via repeated `tick()` calls.
/// - Returns `false` if the driver is uninitialized or already transmitting a message.
/// - The PTT pin is asserted automatically during transmission.
///
/// # See also
/// - [`AskDriver::send()`]
#[cfg(feature = "std")]
pub fn send_from_global_ask<TX: OutputPin, RX: InputPin, PTT: OutputPin>(
    global_driver: &'static Mutex<RefCell<Option<AskDriver<TX, RX, PTT>>>>,
    vec: Vec<u8>,
) -> bool {
    critical_section::with(|cs| {
        if let Some(driver) = global_driver.borrow(cs).borrow_mut().as_mut() {
            driver.send(vec)
        } else {
            false
        }
    })
}
