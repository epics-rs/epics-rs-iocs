//! The XPS_C8_drivers.cpp socket calls the motorNewport SNL programs
//! share: `TCP_ConnectToServer` makes an asyn IP port in the IOC `seq`
//! started the program in, and each call runs on that port's
//! [`XpsSocket`].

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_seq_rs::c::CSrc;
use epics_seq_rs::{ProgramVars, StateSet};

use crate::ioc::connect_to_server;
use crate::xps::{SocketMode, XpsResult, XpsSocket};

/// asynOctetSocket.cpp's `socketStructs`: the socket each
/// `TCP_ConnectToServer` index names.
pub(super) static SOCKETS: Mutex<BTreeMap<i32, Arc<XpsSocket>>> = Mutex::new(BTreeMap::new());

/// C `TCP_ConnectToServer`: the new socket's index, or -1. A positive
/// `timeout` is C's writeRead socket, a negative one its write-only socket.
pub(super) fn tcp_connect_to_server<V: ProgramVars>(
    env: &StateSet<V>,
    ip: impl CSrc,
    ip_port: i32,
    timeout: f64,
) -> i32 {
    let Some((index, handle)) = connect_to_server(env.database(), &text(ip), ip_port) else {
        return -1;
    };
    let mode = if timeout > 0.0 {
        SocketMode::Query
    } else {
        SocketMode::Fire
    };
    let port = handle.port_name().to_string();
    let socket = XpsSocket::new(
        handle,
        index,
        &port,
        mode,
        Duration::from_secs_f64(timeout.abs()),
    );
    SOCKETS.lock().unwrap().insert(index, Arc::new(socket));
    index
}

/// The socket `index` names; -1 is C's invalid socket, which answers -22.
pub(super) fn socket(index: i32) -> Arc<XpsSocket> {
    SOCKETS
        .lock()
        .unwrap()
        .get(&index)
        .cloned()
        .unwrap_or_else(|| Arc::new(XpsSocket::invalid("", SocketMode::Query)))
}

/// The `int` an XPS_C8_drivers.cpp call returns.
pub(super) fn c_int(result: XpsResult<()>) -> i32 {
    result.map_or_else(|e| e.code(), |()| 0)
}

pub(super) fn text(s: impl CSrc) -> String {
    s.with_c_bytes(|b| String::from_utf8_lossy(b).into_owned())
}
