use crate::driver::AskDriver;
use embedded_hal::delay::DelayNs;

/// Runs a blocking loop that repeatedly calls `tick()` on the provided ASK driver.
///
/// This is a simple timing loop for use in environments where interrupts are unavailable
/// or undesired. It drives the ASK modem's timing using a delay provider implementing
/// `embedded_hal::blocking::delay::DelayUs`.
///
/// # Arguments
/// - `driver`: A mutable reference to an `AskDriver` instance.
/// - `delay`: A delay provider implementing `DelayUs<u16>`, typically from the HAL.
/// - `tick_us`: The delay between each tick call, in microseconds (e.g. 63 for ~2 kbps).
///
/// # Example
/// ```rust
/// use ask433::run_ask_tick_loop;
/// let mut driver = AskDriver::new(tx, rx, 8);
/// run_ask_tick_loop(&mut driver, &mut delay, 63);
/// ```
///
/// # Notes
/// - This loop will never return; it is intended for single-purpose polling firmware.
/// - For more efficient or concurrent applications, prefer interrupt-driven tick scheduling.
/// - `delay.delay_us()` errors are ignored, which is acceptable in typical HALs where
///   the only error case is an uninitialized peripheral or transient underrun.
pub fn run_ask_tick_loop<D: DelayNs, TX, RX, PTT>(
    driver: &mut AskDriver<TX, RX, PTT>,
    delay: &mut D,
    tick_us: u32,
) where
    TX: embedded_hal::digital::OutputPin,
    RX: embedded_hal::digital::InputPin,
    PTT: embedded_hal::digital::OutputPin,
{
    loop {
        driver.tick();
        delay.delay_us(tick_us);
    }
}
