//! TCP connect shared by the UR transports.

use std::io;
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

/// Resolve `host:port` and connect to the first address within `timeout`.
pub(crate) fn connect_tcp(host: &str, port: u16, timeout: Duration) -> io::Result<TcpStream> {
    let addr = (host, port)
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Host not found"))?;
    TcpStream::connect_timeout(&addr, timeout)
}
