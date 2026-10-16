//! Allen-Bradley ControlLogix / PLC-5 EtherNet/IP + CIP driver.
//!
//! A port of `epics-modules/ether_ip` (`ether_ip.c`, `drvEtherIP.c`,
//! `devEtherIP.c`). The layers, bottom up:
//!
//! - [`cip`] -- the CIP message-router PDU: tag-path encoding, ReadData /
//!   WriteData / MultiRequest / CM_Unconnected_Send, and the typed value
//!   accessors. Pure functions, unit-tested against byte fixtures produced by
//!   the C encoders themselves.
//! - [`encap`] -- the EtherNet/IP encapsulation header and its commands
//!   (`ListServices`, `RegisterSession`, `SendRRData`). Also pure.
//! - [`connection`] -- one TCP session with one PLC. The only socket code.
//! - [`driver`] -- PLCs, scan lists, tags, and the per-PLC scan thread that
//!   packs tags into MultiRequests.
//! - [`device`] -- the EPICS device support and the `@PLC tag flags` link.

pub mod cip;
pub mod connection;
pub mod device;
pub mod driver;
pub mod encap;

use std::io::Write;
use std::sync::atomic::{AtomicI32, Ordering};

/// `EIP_verbosity` (`ether_ip.c:364`): a line prints when its level is at or
/// below this.
pub static EIP_VERBOSITY: AtomicI32 = AtomicI32::new(4);

/// `EIP_printf`: the text goes to stderr as given, newline included.
pub fn eip_printf(level: i32, args: std::fmt::Arguments<'_>) {
    if level > EIP_VERBOSITY.load(Ordering::Relaxed) {
        return;
    }
    let _ = std::io::stderr().write_fmt(args);
}

/// `EIP_printf_time`: [`eip_printf`] behind a local `%Y/%m/%d %H:%M:%S.%04f`
/// stamp.
pub fn eip_printf_time(level: i32, args: std::fmt::Arguments<'_>) {
    if level > EIP_VERBOSITY.load(Ordering::Relaxed) {
        return;
    }
    let now = chrono::Local::now();
    let mut err = std::io::stderr().lock();
    let _ = write!(
        err,
        "{}.{:04} ",
        now.format("%Y/%m/%d %H:%M:%S"),
        now.timestamp_subsec_nanos() / 100_000
    );
    let _ = err.write_fmt(args);
}

#[macro_export]
macro_rules! eip_printf_time {
    ($level:expr, $($arg:tt)*) => {
        $crate::eip_printf_time($level, format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! eip_printf {
    ($level:expr, $($arg:tt)*) => {
        $crate::eip_printf($level, format_args!($($arg)*))
    };
}
