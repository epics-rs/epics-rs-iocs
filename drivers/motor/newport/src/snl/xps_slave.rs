//! `xpsSlave.st`, compiled by snc-rs from `build.rs` and run on
//! `epics-seq-rs`.
//!
//! Its escaped C is XPS_C8_drivers.cpp: `TCP_ConnectToServer` makes the
//! program's asyn IP port in the IOC `seq` started it in, and the
//! SingleAxisSlave* calls run on that port's [`XpsSocket`]. The `%%`
//! includes need no Rust.

use std::collections::BTreeMap;
use std::ffi::c_char;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_seq_rs::c::*;
use epics_seq_rs::{ProgramVars, SeqVar, StateSet};

use crate::ioc::connect_to_server;
use crate::xps::{SocketMode, XpsResult, XpsSocket};

macro_rules! seqg_escaped {
    (r#"#define epicsExportSharedSymbols"#) => {};
    (r#"#include <shareLib.h>"#) => {};
    (r#"#include "XPS_C8_drivers.h""#) => {};
    ($env:ident, r#"pVar->xps_socket = TCP_ConnectToServer(pVar->ip, 5001, 100.0);"#) => {
        $env.var()
            .xps_socket
            .set(tcp_connect_to_server($env, &$env.var().ip, 5001, 100.0))
    };
    ($env:ident, r#"pVar->val = SingleAxisSlaveParametersGet(pVar->xps_socket, pVar->slave, pVar->masterRdbk, &pVar->ratioRdbk);"#) => {
        $env.var().val.set(single_axis_slave_parameters_get(
            $env.var().xps_socket.get(),
            &$env.var().slave,
            &$env.var().masterRdbk,
            &$env.var().ratioRdbk,
        ))
    };
    ($env:ident, r#"pVar->val = SingleAxisSlaveModeEnable(pVar->xps_socket, pVar->slave);"#) => {
        $env.var().val.set(c_int(
            socket($env.var().xps_socket.get())
                .single_axis_slave_mode_enable(&text(&$env.var().slave)),
        ))
    };
    ($env:ident, r#"pVar->val = SingleAxisSlaveModeDisable(pVar->xps_socket, pVar->slave);"#) => {
        $env.var().val.set(c_int(
            socket($env.var().xps_socket.get())
                .single_axis_slave_mode_disable(&text(&$env.var().slave)),
        ))
    };
    ($env:ident, r#"pVar->val = SingleAxisSlaveParametersSet(pVar->xps_socket, pVar->slave, pVar->master, pVar->ratio_pv);"#) => {
        $env.var().val.set(c_int(
            socket($env.var().xps_socket.get()).single_axis_slave_parameters_set(
                &text(&$env.var().slave),
                &text(&$env.var().master),
                $env.var().ratio_pv.get(),
            ),
        ))
    };
}

epics_seq_rs::include_program!("xpsSlave");

/// asynOctetSocket.cpp's `socketStructs`: the socket each
/// `TCP_ConnectToServer` index names.
static SOCKETS: Mutex<BTreeMap<i32, Arc<XpsSocket>>> = Mutex::new(BTreeMap::new());

/// C `TCP_ConnectToServer`: the new socket's index, or -1. A positive
/// `timeout` is C's writeRead socket, a negative one its write-only socket.
fn tcp_connect_to_server<V: ProgramVars>(
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
fn socket(index: i32) -> Arc<XpsSocket> {
    SOCKETS
        .lock()
        .unwrap()
        .get(&index)
        .cloned()
        .unwrap_or_else(|| Arc::new(XpsSocket::invalid("", SocketMode::Query)))
}

/// The `int` an XPS_C8_drivers.cpp call returns.
fn c_int(result: XpsResult<()>) -> i32 {
    result.map_or_else(|e| e.code(), |()| 0)
}

fn text(s: impl CSrc) -> String {
    s.with_c_bytes(|b| String::from_utf8_lossy(b).into_owned())
}

/// C `SingleAxisSlaveParametersGet(socket, group, master, &ratio)`.
fn single_axis_slave_parameters_get<const N: usize>(
    index: i32,
    group: impl CSrc,
    master: &SeqVar<[c_char; N]>,
    ratio: &SeqVar<f64>,
) -> i32 {
    let (mut name, mut value) = (text(master), ratio.get());
    let code =
        c_int(socket(index).single_axis_slave_parameters_get(&text(group), &mut name, &mut value));
    strcpy(master, name.as_str());
    ratio.set(value);
    code
}

#[cfg(test)]
mod tests {
    use epics_seq_rs::{ProgOptions, VarModel};

    use super::*;
    use crate::xps::fake::FakeLine;

    /// xpsSlave.st:1, with `option +d; option +r;` (:4, :6).
    #[test]
    fn the_program_is_the_st_one() {
        assert_eq!(xpsSlave.validate(), Ok(()));
        assert_eq!(
            xpsSlave.params,
            "name=xpsSlave,P=xxx:,R=xps:slave,IP=192.168.1.11,M=GROUP1.POSITIONER,S=GROUP2"
        );
        assert_eq!(
            xpsSlave.options,
            ProgOptions::new(ProgOptions::CONN | ProgOptions::DEBUG | ProgOptions::NEWEF)
        );
        assert!(matches!(xpsSlave.vars, VarModel::Reentrant));
    }

    /// asynOctetSocket.cpp:137-141: after a failed TCP_ConnectToServer the
    /// program's socket -1 answers every call -22.
    #[test]
    fn socket_minus_one_answers_minus_22() {
        assert_eq!(c_int(socket(-1).single_axis_slave_mode_enable("G2")), -22);
        let (master, ratio) = (SeqVar::new([0; 1024]), SeqVar::new(1.0));
        assert_eq!(
            single_axis_slave_parameters_get(-1, "G2", &master, &ratio),
            -22
        );
        assert_eq!((text(&master), ratio.get()), (String::new(), 1.0));
    }

    /// SingleAxisSlaveParametersGet fills masterRdbk and ratioRdbk; a failed
    /// one keeps them.
    #[test]
    fn parameters_get_fills_the_readbacks() {
        let line = FakeLine::new("xps_slave_snl", "");
        SOCKETS
            .lock()
            .unwrap()
            .insert(900, Arc::new(line.socket(SocketMode::Query)));
        let (master, ratio) = (SeqVar::new([0; 1024]), SeqVar::new(1.0));
        assert_eq!(
            single_axis_slave_parameters_get(900, "G2", &master, &ratio),
            0
        );
        assert_eq!((text(&master), ratio.get()), ("G1.P1".to_string(), 2.5));
        *line.fail.lock().unwrap() = "SingleAxisSlaveParametersGet";
        assert_eq!(
            single_axis_slave_parameters_get(900, "G2", &master, &ratio),
            -3
        );
        assert_eq!((text(&master), ratio.get()), ("G1.P1".to_string(), 2.5));
    }
}
