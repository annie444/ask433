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
    // The arrays below signify the pattern for [$ptt, $ticks_per_bit, $ptt_inverted, $rx_inverted]
    // such that `:ident` == 0 and `:literal` == 1
    // [0, 0, 0, 0]
    ( $tx:ident, $rx:ident, $ptt:ident, $ticks_per_bit:ident, $ptt_inverted:ident, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 0, 0, 0]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:ident, $ptt_inverted:ident, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 1, 0, 0]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:literal, $ptt_inverted:ident, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 1, 1, 0]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:literal, $ptt_inverted:literal, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 1, 1, 1]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:literal, $ptt_inverted:literal, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [0, 0, 0, 1]
    ( $tx:ident, $rx:ident, $ptt:ident, $ticks_per_bit:ident, $ptt_inverted:ident, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [0, 0, 1, 1]
    ( $tx:ident, $rx:ident, $ptt:ident, $ticks_per_bit:ident, $ptt_inverted:literal, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [0, 1, 1, 1]
    ( $tx:ident, $rx:ident, $ptt:ident, $ticks_per_bit:literal, $ptt_inverted:literal, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [0, 1, 0, 1]
    ( $tx:ident, $rx:ident, $ptt:ident, $ticks_per_bit:literal, $ptt_inverted:ident, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 0, 1, 0]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:ident, $ptt_inverted:literal, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 0, 0, 1]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:ident, $ptt_inverted:ident, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [0, 1, 1, 0]
    ( $tx:ident, $rx:ident, $ptt:ident, $ticks_per_bit:literal, $ptt_inverted:literal, $rx_inverted:ident ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 0, 1, 1]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:ident, $ptt_inverted:literal, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
                    $ptt_inverted,
                    $rx_inverted,
                )));
        });
    };
    // [1, 1, 0, 1]
    ( $tx:ident, $rx:ident, $ptt:literal, $ticks_per_bit:literal, $ptt_inverted:ident, $rx_inverted:literal ) => {
        $crate::critical_section::with(|cs| {
            ASK_DRIVER
                .borrow(cs)
                .replace(Some($crate::driver::AskDriver::new(
                    $tx,
                    $rx,
                    $ptt,
                    $ticks_per_bit,
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

/// Attempts to receive a completed message from the global ASK driver instance.
///
/// This macro checks whether a valid and complete message is available using
/// `driver.availabile()`, and if so, returns a reference to the decoded message
/// payload slice. If no message is available, or the driver is currently transmitting,
/// it returns `None`.
///
/// # Requirements
/// - The global `ASK_DRIVER` instance must have been initialized using
///   [`init_global_ask_driver!`] and [`setup_global_ask_driver!`].
/// - Must be called in a context where `critical_section` is available and safe to use.
///
/// # Returns
/// - `Some(&[u8])`: A reference to the message payload if available
/// - `None`: If no message is currently available
///
/// # Example
/// ```rust
/// if let Some(msg) = receive_from_ask!() {
///     // Process message payload
/// }
/// ```
#[macro_export]
macro_rules! receive_from_ask {
    () => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                if driver.availabile() {
                    driver.receive()
                } else {
                    None
                }
            } else {
                None
            }
        })
    };
}

/// Sends a message from the global ASK driver using a heapless `Vec<u8>`.
///
/// This macro supports two usage patterns:
/// 1. Repeating a single element: `send_from_ask!(0xAA; 10)`
/// 2. Sending an explicit sequence of bytes: `send_from_ask!(0x01, 0x02, 0x03)`
///
/// The message is internally collected into a `heapless::Vec<u8, ASK_MAX_MESSAGE_LEN_USIZE>`
/// and passed to the `send()` method on the global driver.
///
/// # Requirements
/// - The driver must be initialized via `init_global_ask_driver!` and `setup_global_ask_driver!`.
///
/// # Panics
/// - Will panic at runtime if the message exceeds `ASK_MAX_MESSAGE_LEN`
///
/// # Example
/// ```rust
/// send_from_ask!(0xAB; 4);         // send [0xAB, 0xAB, 0xAB, 0xAB]
/// send_from_ask!(1, 2, 3, 4, 5);   // send [1, 2, 3, 4, 5]
/// send_from_ask!("Hello, World!")  // send [0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21]
/// ```
#[cfg(not(feature = "std"))]
#[macro_export]
macro_rules! send_from_ask {
    ($elem:expr) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let message: $crate::heapless::Vec<u8, $crate::consts::ASK_MAX_MESSAGE_LEN_USIZE> = $crate::heapless::Vec::from_slice($elem.as_bytes()).unwrap();
                driver.send(vec)
            } else {
                false
            }
        })
    };
    ($elem:expr; $n:expr) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let mut vec = $crate::heapless::Vec::new();
                vec.extend([$elem; $n]);
                driver.send(vec)
            } else {
                false
            }
        })
    };
    ($($x:expr),+ $(,)?) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let mut vec: $crate::heapless::Vec<u8, $crate::consts::ASK_MAX_MESSAGE_LEN_USIZE> = $crate::heapless::Vec::new();
                for item in [$($x),+] {
                    let _ = vec.push(item);
                }
                driver.send(vec)
            } else {
                false
            }
        })
    };
}

/// Sends a message from the global ASK driver using a heapless `Vec<u8>`.
///
/// This macro supports two usage patterns:
/// 1. Repeating a single element: `send_from_ask!(0xAA; 10)`
/// 2. Sending an explicit sequence of bytes: `send_from_ask!(0x01, 0x02, 0x03)`
///
/// The message is internally collected into a `heapless::Vec<u8, ASK_MAX_MESSAGE_LEN_USIZE>`
/// and passed to the `send()` method on the global driver.
///
/// # Requirements
/// - The driver must be initialized via `init_global_ask_driver!` and `setup_global_ask_driver!`.
///
/// # Panics
/// - Will panic at runtime if the message exceeds `ASK_MAX_MESSAGE_LEN`
///
/// # Example
/// ```rust
/// send_from_ask!(0xAB; 4);         // send [0xAB, 0xAB, 0xAB, 0xAB]
/// send_from_ask!(1, 2, 3, 4, 5);   // send [1, 2, 3, 4, 5]
/// send_from_ask!("Hello, World!")  // send [0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21]
/// ```
#[cfg(feature = "std")]
#[macro_export]
macro_rules! send_from_ask {
    ($elem:expr) => {
        $crate::critical_section::with(|cs| {
            let mut guard = ASK_DRIVER.borrow(cs).borrow_mut();
            let driver = guard.as_mut()?;
            let message: Vec<u8> = Vec::from($elem.as_bytes());
            driver.send(vec)
        })
    };
    ($elem:expr; $n:expr) => {
        $crate::critical_section::with(|cs| {
            let mut guard = ASK_DRIVER.borrow(cs).borrow_mut();
            let driver = guard.as_mut()?;
            let mut vec = Vec::new();
            vec.extend([$elem; $n]);
            driver.send(vec)
        })
    };
    ($($x:expr),+ $(,)?) => {
        $crate::critical_section::with(|cs| {
            let mut guard = ASK_DRIVER.borrow(cs).borrow_mut();
            let driver = guard.as_mut()?;
            let mut vec: Vec<u8> = Vec::new();
            for item in [$($x),+] {
                vec.push(item);
            }
            driver.send(vec)
        })
    };
}
