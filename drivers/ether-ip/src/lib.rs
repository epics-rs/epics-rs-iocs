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

use epics_libcom_rs::runtime::log::StagedStderr;

/// `EIP_verbosity` (`ether_ip.c:364`): a line prints when its level is at or
/// below this.
pub static EIP_VERBOSITY: AtomicI32 = AtomicI32::new(4);

/// `EIP_printf`: the text goes to stderr as given, newline included.
pub fn eip_printf(level: i32, args: std::fmt::Arguments<'_>) {
    if level > EIP_VERBOSITY.load(Ordering::Relaxed) {
        return;
    }
    let _ = StagedStderr.write_fmt(args);
}

/// `EIP_printf_time`: [`fn@eip_printf`] behind a local `%Y/%m/%d %H:%M:%S.%04f`
/// stamp. C writes the stamp and the message in two `fprintf`s
/// (`ether_ip.c:390-393`), so another thread's line can land between them;
/// here they go out as one write.
pub fn eip_printf_time(level: i32, args: std::fmt::Arguments<'_>) {
    if level > EIP_VERBOSITY.load(Ordering::Relaxed) {
        return;
    }
    let now = chrono::Local::now();
    let _ = write!(
        StagedStderr,
        "{}.{:04} {args}",
        now.format("%Y/%m/%d %H:%M:%S"),
        now.timestamp_subsec_nanos() / 100_000
    );
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

#[cfg(all(test, unix))]
mod tests {
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};

    /// What `f` writes to fd 2, one entry per `write`: fd 2 is a datagram
    /// socket for the call, so each `write` is one datagram.
    pub(crate) fn stderr_writes(f: impl FnOnce()) -> Vec<String> {
        let mut fds = [0; 2];
        // SAFETY: `fds` has room for the pair, and each fd is owned once.
        let (rx, tx) = unsafe {
            assert_eq!(
                libc::socketpair(libc::AF_UNIX, libc::SOCK_DGRAM, 0, fds.as_mut_ptr()),
                0
            );
            (OwnedFd::from_raw_fd(fds[0]), OwnedFd::from_raw_fd(fds[1]))
        };
        // SAFETY: fd 2 is swapped for the call and put back after it.
        unsafe {
            let saved = OwnedFd::from_raw_fd(libc::dup(2));
            libc::dup2(tx.as_raw_fd(), 2);
            f();
            libc::dup2(saved.as_raw_fd(), 2);
        }
        let mut writes = Vec::new();
        let mut buf = [0u8; 2048];
        loop {
            // SAFETY: `buf` is writable for its length.
            let got = unsafe {
                libc::recv(
                    rx.as_raw_fd(),
                    buf.as_mut_ptr().cast(),
                    buf.len(),
                    libc::MSG_DONTWAIT,
                )
            };
            if got <= 0 {
                break;
            }
            writes.push(String::from_utf8_lossy(&buf[..got as usize]).into_owned());
        }
        writes
    }

    /// An `EIP_printf` line is one write, and an `EIP_printf_time` line with
    /// its stamp. glibc writes an unbuffered `vfprintf` in 128-byte chunks:
    /// one write at any length is a deliberate deviation from C.
    #[test]
    fn an_eip_line_is_one_write() {
        let name = "plc1";
        let writes = stderr_writes(|| crate::eip_printf!(0, "EIP describe {name} {}\n", 7));
        assert!(
            writes.contains(&"EIP describe plc1 7\n".to_string()),
            "{writes:?}"
        );

        let writes = stderr_writes(|| crate::eip_printf_time!(0, "EIP connecting {name}\n"));
        // `%Y/%m/%d %H:%M:%S.%04f ` is 25 bytes.
        assert!(
            writes
                .iter()
                .any(|w| w.len() == 25 + 20 && w.ends_with(" EIP connecting plc1\n")),
            "{writes:?}"
        );
    }
}
