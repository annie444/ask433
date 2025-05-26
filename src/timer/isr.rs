use crate::driver::AskDriver;
use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal::digital::{InputPin, OutputPin};

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
