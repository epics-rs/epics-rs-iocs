//! `xpsSlave.st`, compiled by snc-rs from `build.rs` and run on
//! `epics-seq-rs`.
//!
//! Its escaped C is XPS_C8_drivers.cpp, through `xps_c8`. The
//! `%%` includes need no Rust.

use std::ffi::c_char;

use epics_seq_rs::SeqVar;
use epics_seq_rs::c::*;

use super::xps_c8::{c_int, socket, tcp_connect_to_server, text};

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

    use std::sync::Arc;

    use super::super::xps_c8::SOCKETS;
    use super::*;
    use crate::xps::SocketMode;
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
