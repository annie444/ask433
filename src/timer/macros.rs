/// Declares a static global `ASK_DRIVER` instance protected by a `critical_section` mutex.
///
/// This macro creates a `static` singleton `ASK_DRIVER` suitable for use in
/// interrupt-based environments, where both the main thread and an ISR need
/// to safely access the shared driver state.
///
/// # Arguments
/// - `$tx`: The concrete type of the TX pin (must implement `OutputPin`)
/// - `$rx`: The concrete type of the RX pin (must implement `InputPin`)
/// - `$ptt`: The concrete type of the PRR pin (must implement `OutputPin`). **NOTE**: While you
/// might not have a PTT pin, it is imperative that you pass a type here.
///
/// # Example
/// ```rust
/// use ask433::init_ask_driver;
/// use embedded_hal_mock::eh1::digital::{Mock as Pin};
/// init_ask_driver!(Pin, Pin, Pin);
/// ```
#[cfg_attr(feature = "timer-isr", macro_export)]
macro_rules! init_ask_driver {
    ($tx:ty, $rx:ty, $ptt:ty) => {
        #[allow(unused)]
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
/// - `$tx`: The TX pin (must implement `OutputPin`)
/// - `$rx`: The RX pin (must implement `InputPin`)
/// - `$ptt`: The optional PTT pin (must implement `OutputPin`)
/// - `$tpb`: The number of ticks per bit (e.g., 8 for a radio at 2 kbps and an interrupt at ~2MHz)
/// - `$ptt_inverted`: The optional boolean specifying whether the PTT pin should be inverted
/// - `$rx_inverted`: The optional boolean specifying whether the RX pin should be inverted
///
/// # Example
/// ```rust
/// use embedded_hal_mock::eh1::digital::{Mock as Pin, Transaction as Tx, State as St};
/// use ask433::{init_ask_driver, setup_ask_driver};
///
/// init_ask_driver!(Pin, Pin, Pin);
///
/// fn main() {
///     # let tx = Pin::new(&[Tx::set(St::Low)]);
///     # let rx = Pin::new(&[]);
///     setup_ask_driver!(tx, rx, None, 8, None, None);
///     # critical_section::with(|cs| {
///     #    if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
///     #       driver.tx.done();
///     #       driver.rx.done();
///     #   }
///     # });
/// }
/// ```
///
/// # Notes
/// - Must be called inside a critical section-aware context (safe in `main()`).
/// - Requires `init_ask_driver!` to have been used earlier.
#[cfg_attr(feature = "timer-isr", macro_export)]
macro_rules! setup_ask_driver {
    ( $tx:expr, $rx:expr, $ptt:expr, $ticks_per_bit:expr, $ptt_inverted:expr, $rx_inverted:expr ) => {
        $crate::critical_section::with(|cs| {
            let _ = ASK_DRIVER
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
/// ```rust,ignore
/// use ask433::tick_ask_timer;
///
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
#[cfg_attr(feature = "timer-isr", macro_export)]
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
/// use ask433::{receive_from_ask, init_ask_driver};
/// # use embedded_hal_mock::eh1::digital::{Mock as Pin};
///
/// init_ask_driver!(Pin, Pin, Pin);
///
/// fn main() {
///     // ...
///     if let Some(msg) = receive_from_ask!() {
///         // Process message payload
///     }
/// }
/// ```
#[cfg_attr(feature = "timer-isr", macro_export)]
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
/// ```rust,ignore
/// use ask433::{send_from_ask, init_ask_driver};
/// # use embedded_hal_mock::eh1::digital::{Mock as Pin};
///
/// init_ask_driver!(Pin, Pin, Pin);
///
/// fn main() {
///
///     // ...
///
///     let sent1 = send_from_ask!(0xAB; 4);         // send [0xAB, 0xAB, 0xAB, 0xAB]
///     let sent2 = send_from_ask!(1, 2, 3, 4, 5);   // send [1, 2, 3, 4, 5]
///     let sent3 = send_from_ask!("Hello, World!");  // send [0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21]
/// }
/// ```
#[cfg(not(feature = "std"))]
#[cfg_attr(all(feature = "timer-isr", not(feature = "std")), macro_export)]
macro_rules! send_from_ask {
    ($elem:expr) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let message = $crate::heapless::Vec::from_slice($elem.as_bytes()).unwrap();
                driver.send(message)
            } else {
                false
            }
        })
    };
    ($elem:expr; $n:expr) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let mut message = $crate::heapless::Vec::new();
                message.extend([$elem; $n]);
                driver.send(message)
            } else {
                false
            }
        })
    };
    ($($x:expr),+ $(,)?) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let mut message = $crate::heapless::Vec::new();
                for item in [$($x),+] {
                    let _ = message.push(item);
                }
                driver.send(message)
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
/// - The driver must be initialized via `init_ask_driver!` and `setup_ask_driver!`.
///
/// # Panics
/// - Will panic at runtime if the message exceeds `ASK_MAX_MESSAGE_LEN`
///
/// # Example
/// ```rust
/// use ask433::{send_from_ask, init_ask_driver};
/// # use embedded_hal_mock::eh1::digital::{Mock as Pin};
///
/// init_ask_driver!(Pin, Pin, Pin);
///
/// fn main() {
///
///     // ...
///
///     let sent1: bool = send_from_ask![0xAB; 4];         // send [0xAB, 0xAB, 0xAB, 0xAB]
///     let sent2: bool = send_from_ask![1, 2, 3, 4, 5];   // send [1, 2, 3, 4, 5]
///     let sent3: bool = send_from_ask!("Hello, World!");  // send [0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x57, 0x6f, 0x72, 0x6c, 0x64, 0x21]
///
/// }
/// ```
#[cfg(feature = "std")]
#[cfg_attr(all(feature = "timer-isr", feature = "std"), macro_export)]
macro_rules! send_from_ask {
    ($elem:expr) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let message = ::std::vec::Vec::from($elem.as_bytes());
                driver.send(message)
            } else {
                false
            }
        })
    };
    ($elem:expr; $n:expr) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let mut message = ::std::vec::Vec::new();
                message.extend([$elem; $n]);
                driver.send(message)
            } else {
                false
            }
        })
    };
    ($($x:expr),+ $(,)?) => {
        $crate::critical_section::with(|cs| {
            if let Some(driver) = ASK_DRIVER.borrow(cs).borrow_mut().as_mut() {
                let mut message = ::std::vec::Vec::new();
                for item in [$($x),+] {
                    message.push(item);
                }
                driver.send(message)
            } else {
                false
            }
        })
    };
}

// see src/lib.rs for tests
