//! TCP connect shared by the UR transports.

use std::io;
use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

/// The deadline of ur_rtde's timed connects: `DashboardClient::connect` and
/// `RobotiqGripper::connect` default `timeout_ms = 2000`
/// (dashboard_client.h:37, robotiq_gripper.h:165).
pub(crate) const CONNECT_TIMEOUT: Duration = Duration::from_millis(2000);

/// Connect to `host:port` as `boost::asio::connect` does over the resolver
/// results: each address in turn, failing with the last error.
///
/// `Some(deadline)` bounds the whole attempt, as the timed `async_connect`
/// loop does. `None` is the plain blocking connect of `RTDE::connect` and
/// `ScriptClient::connect` (rtde.cpp:75, script_client.cpp:42), which waits
/// for the kernel's verdict — "No route to host", not a timeout.
pub(crate) fn connect_tcp(
    host: &str,
    port: u16,
    deadline: Option<Duration>,
) -> io::Result<TcpStream> {
    let Some(deadline) = deadline else {
        return TcpStream::connect((host, port));
    };
    let until = Instant::now() + deadline;
    let mut last = io::Error::new(io::ErrorKind::NotFound, "Host not found");
    for addr in (host, port).to_socket_addrs()? {
        let left = until.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        match TcpStream::connect_timeout(&addr, left) {
            Ok(sock) => return Ok(sock),
            Err(e) => last = e,
        }
    }
    Err(last)
}

/// What a failed blocking connect's `boost::system::system_error::what()`
/// says, e.g. `connect: No route to host [system:113]` — C's text minus the
/// boost header path asio appends after the code.
pub(crate) fn asio_connect_what(e: &io::Error) -> String {
    match e.raw_os_error() {
        Some(code) => {
            let text = e.to_string();
            let reason = text
                .strip_suffix(&format!(" (os error {code})"))
                .unwrap_or(&text);
            format!("connect: {reason} [system:{code}]")
        }
        None => format!("connect: {e}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Privileged port 1: no test can bind it, so a connect is always refused.
    /// A freed ephemeral port can be taken by a concurrent test's listener.
    const DEAD_PORT: u16 = 1;

    #[test]
    fn a_refused_connect_reads_as_asio_what() {
        let e = connect_tcp("127.0.0.1", DEAD_PORT, None).expect_err("nothing listens");
        let code = e.raw_os_error().expect("an OS error");
        let what = asio_connect_what(&e);
        assert!(what.starts_with("connect: "), "{what}");
        assert!(what.ends_with(&format!(" [system:{code}]")), "{what}");
        assert!(!what.contains("os error"), "{what}");
    }
}
