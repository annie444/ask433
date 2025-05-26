/// Declares a static global `ASK_DRIVER` instance protected by a `critical_section` mutex.
///
/// This macro creates a `static` singleton `ASK_DRIVER` suitable for use in
/// interrupt-based environments, where both the main thread and an ISR need
/// to safely access the shared driver state.
///
/// # Arguments
/// - `$tx`: The concrete type of the TX pin (must implement `OutputPin`)
/// - `$rx`: The concrete type of the RX pin (must implement `InputPin`)
///
/// # Example
/// ```rust
/// init_ask_driver!(MyTxPinType, MyRxPinType);
/// ```
#[macro_export]
macro_rules! init_ask_driver {
    ( $tx:ty, $rx:ty, $ptt:ty ) => {
        pub static ASK_DRIVER: $crate::critical_section::Mutex<
            core::cell::RefCell<Option<$crate::driver::AskDriver<$tx, $rx, $ptt>>>,
        > = $crate::critical_section::Mutex::new(core::cell::RefCell::new(None));
    };
}

/// Initializes the global `ASK_DRIVER` singleton with a new driver instance.
///
/// This macro wraps construction of the `AskDriver` and stores it inside the
/// globally declared `ASK_DRIVER` created by `init_ask_driver!`.
///
/// # Arguments
/// - `$tx`: The TX pin variable (must implement `OutputPin`)
/// - `$rx`: The RX pin variable (must implement `InputPin`)
/// - `$ptt`: The optional PTT pin variable or literal (must implement `OutputPin`)
/// - `$tpb`: Either a variable or literal specifying ticks per bit (e.g., 8 for 2 kbps)
/// - `$ptt_inverted`: Either a variable or literal specifying whether the PTT pin should be
/// inverted
/// - `$rx_inverted`: Either a variable or literal specifying whether the RX pin should be
/// inverted
///
/// # Example
/// ```rust
/// main() {
///     setup_ask_driver!(tx, rx, None, 8, None, None);
/// }
/// ```
///
/// # Notes
/// - Must be called inside a critical section-aware context (safe in `main()`).
/// - Requires `init_ask_driver!` to have been used earlier.
#[macro_export]
macro_rules! setup_ask_driver {
    ( $tx:ident, $rx:ident, $ptt:ident, $tpb:ident, $ptt_inverted:ident, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:literal, $tpb:ident, $ptt_inverted:ident, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:literal, $tpb:literal, $ptt_inverted:ident, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:literal, $tpb:literal, $ptt_inverted:literal, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:literal, $tpb:literal, $ptt_inverted:literal, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:ident, $tpb:ident, $ptt_inverted:ident, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:ident, $tpb:ident, $ptt_inverted:literal, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:ident, $tpb:literal, $ptt_inverted:literal, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:ident, $tpb:literal, $ptt_inverted:ident, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:literal, $tpb:ident, $ptt_inverted:literal, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:literal, $tpb:ident, $ptt_inverted:ident, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    ( $tx:ident, $rx:ident, $ptt:ident, $tpb:literal, $ptt_inverted:literal, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $tbp,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
}

/// Calls `tick()` on the global `ASK_DRIVER` if it has been initialized.
///
/// This macro is intended to be invoked from a timer ISR or scheduler to
/// advance the ASK state machine at regular intervals (e.g., every 62.5 µs).
///
/// # Example
/// ```rust
/// #[interrupt]
/// fn TIM2() {
///     tick_ask_timer!();
/// }
/// ```
///
/// # Notes
/// - This macro assumes `ASK_DRIVER` was declared with `init_ask_driver!`
///   and initialized via `setup_ask_driver!`.
/// - Safe to call repeatedly — will silently do nothing if the driver hasn't been set up yet.
#[macro_export]
macro_rules! tick_ask_timer {
    () => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                driver.tick();
            }
        });
    };
}
