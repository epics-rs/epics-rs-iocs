//! C `phytronController::writeOctet` (phytronAxisMotor.cpp:170-174,
//! 550-578): the DIRECT_COMMAND passthrough.

#![cfg(target_os = "linux")]

use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::task::sleep;
use epics_rs::base::server::iocsh::registry::ArgValue;
use motor_common::testing::{LineRig, port_report, run_command};

/// An MCC at address 0: `XY` answers ACK `hello`, `BAD` NAK `x`, `GONE` a
/// frame without STX; every other frame is ACKed.
fn mcc_reply(frame: &str) -> String {
    let answer = match &frame[2..frame.len() - 1] {
        "XY" => "\x06hello",
        "BAD" => "\x15x",
        "GONE" => return "garbage\x03".into(),
        "SUI" => "\x06I=00000000",
        c if c.ends_with("=H") => "\x06E",
        c if c.ends_with("=E") || c.ends_with("P20R") || c.ends_with("P22R") => "\x060",
        _ => "\x06",
    };
    format!("\x02{answer}\x03")
}

/// MCC `name` with axis 1, polled once, and its line drained.
async fn mcc(name: &str) -> (LineRig, SyncIOHandle, PortHandle) {
    let io_port = format!("{name}_IO");
    let (line, io) = LineRig::new(&io_port, mcc_reply);
    let mut commands = motor_phytron::ioc::phytron_commands().into_iter();
    let (_phymotion, create_mcc, create_axis) = (
        commands.next().unwrap(),
        commands.next().unwrap(),
        commands.next().unwrap(),
    );
    let s = |v: &str| ArgValue::String(v.into());
    let mcc_args = vec![
        s(name),
        s(&io_port),
        ArgValue::Int(0),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ];
    run_command(create_mcc, mcc_args).await;
    run_command(
        create_axis,
        vec![s(name), ArgValue::Int(0), ArgValue::Int(1)],
    )
    .await;
    line.sent();
    let port = PortManager::global().find_port_handle(name).unwrap();
    (line, io, port)
}

/// Run `f` on a thread of its own, as a record's port thread does.
async fn off_runtime<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let thread = std::thread::spawn(f);
    while !thread.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    thread.join().unwrap()
}

fn reason(port: &PortHandle, name: &str) -> usize {
    port.drv_user_create_blocking(&DrvUserRequest::new(name, 0))
        .unwrap()
        .reason
}

/// A stringout's write of `text` to `param` at address 0.
fn write(port: &PortHandle, param: &str, text: &str) -> bool {
    let op = RequestOp::OctetWrite {
        data: text.as_bytes().to_vec(),
    };
    let user = AsynUser::new(reason(port, param)).with_addr(0);
    port.submit_blocking(op, user).is_ok()
}

fn string(port: &PortHandle, param: &str) -> String {
    let value = port.get_string_param_blocking(reason(port, param), 0, 100);
    String::from_utf8(value.unwrap()).unwrap()
}

/// DIRECT_STATUS's value in the port's report.
fn status(port: &str) -> String {
    let report = port_report(port, 2);
    let line = report
        .lines()
        .find(|l| l.contains("name=DIRECT_STATUS, value="));
    line.unwrap().split("value=").nth(1).unwrap().to_owned()
}

#[epics_rs::base::epics_test]
async fn a_direct_command_s_reply_and_status_are_c_s() {
    let (line, _io, port) = mcc("phyDirect").await;
    let (before, ack, nak) = off_runtime(move || {
        let before = (string(&port, "DIRECT_REPLY"), status("phyDirect"));
        assert!(write(&port, "DIRECT_COMMAND", "XY"));
        let ack = (string(&port, "DIRECT_REPLY"), status("phyDirect"));
        assert!(write(&port, "DIRECT_COMMAND", "BAD"));
        let nak = (
            string(&port, "DIRECT_REPLY"),
            status("phyDirect"),
            string(&port, "DIRECT_COMMAND"),
        );
        (before, ack, nak)
    })
    .await;
    assert_eq!(before, ("".into(), "0, status=0".into()));
    assert_eq!(ack, ("hello".into(), "1, status=0".into()));
    assert_eq!(nak, ("x".into(), "2, status=0".into(), "BAD".into()));
    // The frames as C frames them, between the poller's.
    let wire: Vec<String> = line
        .sent()
        .into_iter()
        .filter(|f| f.contains("XY") || f.contains("BAD"))
        .collect();
    assert_eq!(wire, ["\x020XY\x03", "\x020BAD\x03"]);
}

/// A failed exchange returns its error and writes nothing: C calls the
/// base only on success.
#[epics_rs::base::epics_test]
async fn a_failed_direct_command_leaves_every_parameter() {
    let (_line, _io, port) = mcc("phyDirectFail").await;
    let after = off_runtime(move || {
        assert!(write(&port, "DIRECT_COMMAND", "XY"));
        assert!(!write(&port, "DIRECT_COMMAND", "GONE"));
        (
            string(&port, "DIRECT_REPLY"),
            status("phyDirectFail"),
            string(&port, "DIRECT_COMMAND"),
        )
    })
    .await;
    assert_eq!(after, ("hello".into(), "1, status=0".into(), "XY".into()));
}

#[epics_rs::base::epics_test]
async fn direct_reply_and_status_refuse_a_write() {
    let (line, _io, port) = mcc("phyDirectRo").await;
    let (refused, reply) = off_runtime(move || {
        let refused = !write(&port, "DIRECT_REPLY", "XY") && !write(&port, "DIRECT_STATUS", "1");
        (refused, string(&port, "DIRECT_REPLY"))
    })
    .await;
    assert!(refused);
    assert_eq!(reply, "");
    assert!(!line.sent().iter().any(|f| f.contains("XY")));
}
