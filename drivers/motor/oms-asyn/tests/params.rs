//! C `omsBaseController::writeInt32` / `writeFloat64` on a MAXnet built by
//! `omsMAXnetConfig`, its axis X at address 0.

#![cfg(target_os = "linux")]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::{TraceFile, TraceMask};
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use epics_rs::motor::controller_port::index;
use motor_common::testing::LineRig;

/// A MAXnet with firmware 1.33.4 and one idle encoder axis.
fn maxnet_reply(cmd: &str) -> String {
    match cmd.trim_end() {
        "WY" => "MAXnet ver:1.33.4, s/n:1",
        "AA RP;" => "0",
        "AX PS?" => "=E",
        "AX LT?" => "=h",
        "AM;RI;" => "PDNN",
        "AM PP;" => "100",
        "AM PE;" => "7",
        "AM;CL?;" => "on",
        "AM;RV;" => "0",
        "AM;EA;" => "EN",
        "AM;QL;" => "0",
        "#WS" => "=0",
        "AX;RE;" => "7",
        _ => "",
    }
    .into()
}

/// A MAXnet with firmware 1.29.0, older than the 1.30 MAXnet's newer
/// command forms need.
fn old_maxnet_reply(cmd: &str) -> String {
    match cmd.trim_end() {
        "WY" => "MAXnet ver:1.29.0, s/n:1".into(),
        "AX ?PS" => "=E".into(),
        "AX ?LS" => "=h".into(),
        "AX ?PM" => "=on".into(),
        "AX PS?" | "AX LT?" | "AM;CL?;" => String::new(),
        other => maxnet_reply(other),
    }
}

/// The poller's own reads, which a test of what a write sends leaves out.
fn polled(cmd: &str) -> bool {
    cmd.starts_with("AM") || cmd == "#WS"
}

/// `omsMAXnetConfig(port, 1, line, 100, 1000, "")`, then `body` on the
/// shell thread with the motor port and the line.
async fn on_maxnet<T: Send + 'static>(
    port: &str,
    body: impl FnOnce(PortHandle, &Mutex<LineRig>) -> T + Send + 'static,
) -> T {
    on_line(port, maxnet_reply, move |port, rig| {
        rig.lock().unwrap().sent();
        body(port, rig)
    })
    .await
}

/// [`on_maxnet`] on a line answering with `reply`, what config sent left
/// on the line.
async fn on_line<T: Send + 'static>(
    port: &str,
    reply: fn(&str) -> String,
    body: impl FnOnce(PortHandle, &Mutex<LineRig>) -> T + Send + 'static,
) -> T {
    let line = format!("{port}Line");
    let (rig, _io) = LineRig::new(&line, reply);
    let config = motor_oms_asyn::ioc::oms_commands()
        .into_iter()
        .next()
        .unwrap();
    let args = vec![
        ArgValue::String(port.into()),
        ArgValue::Int(1),
        ArgValue::String(line.into()),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::String("".into()),
    ];
    let bridge = BlockingBridge::capture();
    let port = port.to_owned();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        config.handler.call(&args, &ctx).unwrap();
        let rig = Mutex::new(rig);
        body(PortManager::global().find_port_handle(&port).unwrap(), &rig)
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap()
}

/// What the line was sent within `wait`, the poller's reads left out.
fn sent_within(line: &Mutex<LineRig>, wait: Duration) -> Vec<String> {
    let deadline = Instant::now() + wait;
    let mut sent = Vec::new();
    while Instant::now() < deadline {
        sent.extend(
            line.lock()
                .unwrap()
                .sent()
                .into_iter()
                .map(|c| c.trim_end().to_owned())
                .filter(|c| !polled(c)),
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    sent
}

/// Closed loop, encoder position and gains are each sent as C's
/// writeInt32 / writeFloat64 send them (omsBaseController.cpp:260-293,
/// 333-406); a gain outside 0..32768 is refused.
#[epics_rs::base::epics_test]
async fn axis_writes_are_sent_as_c_sends_them() {
    let sent = on_maxnet("omsAxisWrites", |port, line| {
        port.write_int32_blocking(index::CLOSED_LOOP, 0, 1).unwrap();
        port.write_int32_blocking(index::CLOSED_LOOP, 0, 0).unwrap();
        port.write_float64_blocking(index::ENCODER_POSITION, 0, 12.7)
            .unwrap();
        port.write_float64_blocking(index::PGAIN, 0, 0.5).unwrap();
        port.write_float64_blocking(index::IGAIN, 0, 40000.0)
            .unwrap();
        port.write_float64_blocking(index::DGAIN, 0, 0.25).unwrap();
        sent_within(line, Duration::from_millis(300))
    })
    .await;
    assert_eq!(
        sent,
        [
            "AX CL1",
            "AX CL0",
            "AX LPE12;",
            "AX KP0.500000;",
            "AX KD0.250000;"
        ]
    );
}

/// C does not implement deferred moves and says so
/// (omsBaseController.cpp:254-259).
#[epics_rs::base::epics_test]
async fn a_deferred_move_write_says_it_is_not_implemented() {
    use epics_rs::asyn::trace::{TraceFile, TraceMask};
    let name = "omsDeferMoves";
    let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
    let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
    on_maxnet(name, move |port, _| {
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_device_trace_mask(name, 0, TraceMask::ERROR);
        trace.set_device_trace_file(name, 0, TraceFile::File(file));
        port.write_int32_blocking(index::DEFER_MOVES, 0, 1).unwrap();
        std::thread::sleep(Duration::from_millis(200));
    })
    .await;
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let want = format!("omsBaseDriver:writeInt32:{name} Deferred Move: not yet implemented");
    assert!(text.lines().any(|l| l.ends_with(&want)), "{text}");
}

/// Before firmware 1.30 a MAXnet takes the older forms: `?PS` and `?LS` at
/// init, `HN` / `HF` for closed loop and a `?PM` per axis for its status
/// (omsBaseController.cpp:279-290, 528-556, 871-885).
#[epics_rs::base::epics_test]
async fn old_firmware_takes_the_older_command_forms() {
    let (init, sent) = on_line("omsOldFw", old_maxnet_reply, |port, line| {
        let init = line.lock().unwrap().sent();
        port.write_int32_blocking(index::CLOSED_LOOP, 0, 1).unwrap();
        port.write_int32_blocking(index::CLOSED_LOOP, 0, 0).unwrap();
        (init, sent_within(line, Duration::from_millis(1500)))
    })
    .await;
    let init: Vec<_> = init.iter().map(|c| c.trim_end()).collect();
    assert!(
        init.contains(&"AX ?PS") && init.contains(&"AX ?LS"),
        "{init:?}"
    );
    assert!(
        !init.contains(&"AX PS?") && !init.contains(&"AX LT?"),
        "{init:?}"
    );
    let (status, writes): (Vec<_>, Vec<_>) = sent.iter().partition(|c| *c == "AX ?PM");
    assert_eq!(writes, ["AX HN", "AX HF"], "{sent:?}");
    assert!(!status.is_empty(), "{sent:?}");
}

/// C's closing line ends every writeFloat64 and the writeInt32s it
/// handles itself; one it returns to the base prints none
/// (omsBaseController.cpp:266-313, 409-416).
#[epics_rs::base::epics_test]
async fn writes_print_c_s_closing_line() {
    let name = "omsClosingTrace";
    let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
    let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
    let text = on_maxnet(name, move |port, _| {
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_device_trace_mask(name, 0, TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_device_trace_file(name, 0, TraceFile::File(file));
        port.write_int32_blocking(index::STOP, 0, 1).unwrap();
        port.write_int32_blocking(index::CLOSED_LOOP, 0, 1).unwrap();
        port.write_int32_blocking(index::DEFER_MOVES, 0, 0).unwrap();
        port.write_float64_blocking(index::ENCODER_POSITION, 0, 12.7)
            .unwrap();
        port.write_float64_blocking(index::VELOCITY, 0, 2.0)
            .unwrap();
        // The axis's writes print on its poller.
        std::thread::sleep(Duration::from_millis(300));
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        text
    })
    .await;
    let mut lines: Vec<&str> = text
        .lines()
        .filter_map(|l| l.split_once("omsBaseDriver:write").map(|(_, tail)| tail))
        .filter(|tail| tail.starts_with("Int32: ") || tail.starts_with("Float64: "))
        .collect();
    lines.sort_unstable();
    let mut expected = [
        format!("Int32: function={}, value=1", index::CLOSED_LOOP),
        format!("Int32: function={}, value=0", index::DEFER_MOVES),
        format!(
            "Float64: function={}, value=12.700000",
            index::ENCODER_POSITION
        ),
        format!("Float64: function={}, value=2.000000", index::VELOCITY),
    ];
    expected.sort_unstable();
    assert_eq!(lines, expected, "{text}");
}
