//! C `phytronBrakeOutput`, `setBrakeOutput` and the brake parameters
//! (phytronAxisMotor.cpp:322, 455-456, 493-537, 1287-1330, 1510-1585).

#![cfg(target_os = "linux")]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::runtime::task::sleep;
use epics_rs::base::server::iocsh::registry::ArgValue;
use epics_rs::motor::controller_port::index;
use motor_common::testing::{LineRig, port_report, run_command};

/// An MCC at address 0 that ACKs each command of a frame, with idle,
/// fault-free axes.
fn mcc_reply(frame: &str) -> String {
    let answers: String = frame[2..frame.len() - 1]
        .split(' ')
        .map(|c| match c {
            "SUI" => "\x06I=00000000",
            c if c.ends_with("=H") => "\x06E",
            c if c.ends_with("=E") || c.ends_with("P20R") || c.ends_with("P22R") => "\x060",
            _ => "\x06",
        })
        .collect();
    format!("\x02{answers}\x03")
}

/// MCC `name`, its axis 1 polled once, then `phytronBrakeOutput(name,
/// 0.1, 0.3, 1, 0, 0)`: the MCC's axis 1, output 3, the idle motor
/// disabled.
async fn mcc(name: &str) -> (Arc<Mutex<LineRig>>, SyncIOHandle, PortHandle) {
    let io_port = format!("{name}_IO");
    let (line, io) = LineRig::new(&io_port, mcc_reply);
    let mut commands = motor_phytron::ioc::phytron_commands().into_iter();
    let (_phymotion, create_mcc, create_axis, brake) = (
        commands.next().unwrap(),
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
    let brake_args = vec![
        s(name),
        ArgValue::Double(0.1),
        ArgValue::Double(0.3),
        ArgValue::Int(1),
        ArgValue::Double(0.0),
        ArgValue::Double(0.0),
    ];
    run_command(brake, brake_args).await;
    let port = PortManager::global().find_port_handle(name).unwrap();
    (Arc::new(Mutex::new(line)), io, port)
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
    port.drv_user_create_blocking(&DrvUserRequest::new(name, 1))
        .unwrap()
        .reason
}

/// The commands on `line` since the last call, one per frame command.
fn wire(line: &Mutex<LineRig>) -> Vec<String> {
    let sent = line.lock().unwrap().sent();
    sent.iter()
        .flat_map(|f| {
            f[2..f.len() - 1]
                .split(' ')
                .map(str::to_owned)
                .collect::<Vec<_>>()
        })
        .collect()
}

/// `line`'s commands until one is `want`, for up to 5 s.
fn wait_for(line: &Mutex<LineRig>, want: &str) -> Vec<String> {
    let mut sent = Vec::new();
    for _ in 0..500 {
        sent.extend(wire(line));
        if sent.iter().any(|c| c == want) {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    sent
}

/// configureBrake on an idle axis engages the brake and then disables the
/// motor, each its own exchange (:1520, :1563-1583).
#[epics_rs::base::epics_test]
async fn phytron_brake_output_engages_the_brake_of_the_idle_axis() {
    let (line, _io, _port) = mcc("phyBrakeCfg").await;
    assert_eq!(wire(&line), ["A3R", "1MD"]);
}

/// A move enables the motor and releases the brake ahead of it; the poll
/// that reads it stopped engages the brake and disables the motor
/// (:1540-1557, :2022-2023).
#[epics_rs::base::epics_test]
async fn a_move_releases_the_brake_and_its_stop_engages_it() {
    let (line, _io, port) = mcc("phyBrakeMove").await;
    wire(&line);
    let rig = line.clone();
    let sent = off_runtime(move || {
        port.write_float64_blocking(index::MOVE_ABS, 1, 100.0)
            .unwrap();
        wait_for(&rig, "1MD")
    })
    .await;
    let at = |c: &str| sent.iter().position(|s| s == c);
    let (ma, release, mv) = (at("1MA"), at("A3S"), at("1A100"));
    assert!(ma.is_some() && ma < release && release < mv, "{sent:?}");
    let (engage, md) = (at("A3R"), at("1MD"));
    assert!(mv < engage && engage < md, "{sent:?}");
}

/// The brake records read the axis's members, in C's float; a time is
/// clamped to 10 s, which the base stores and posts (:493-537).
#[epics_rs::base::epics_test]
async fn the_brake_parameters_read_the_axis_s_brake() {
    let (line, _io, port) = mcc("phyBrakeParams").await;
    wire(&line);
    let (output, engage, disabled, report) = off_runtime(move || {
        let engage = reason(&port, "AXIS_BRAKE_ENGAGE_TIME");
        port.write_float64_blocking(engage, 1, 20.0).unwrap();
        let output = port
            .read_float64_blocking(reason(&port, "AXIS_BRAKE_OUTPUT"), 1)
            .unwrap();
        let engage = port.read_float64_blocking(engage, 1).unwrap();
        let disable = reason(&port, "AXIS_DISABLE_MOTOR");
        let disabled = port.read_int32_blocking(disable, 1).unwrap();
        (output, engage, disabled, port_report("phyBrakeParams", 2))
    })
    .await;
    assert_eq!(output, f64::from(0.3f32));
    assert_eq!(engage, 10.0);
    assert_eq!(disabled, 1);
    assert!(
        report.contains("name=AXIS_BRAKE_ENGAGE_TIME, value=10, status=0"),
        "{report}"
    );
}

/// Clearing AXIS_DISABLE_MOTOR re-runs setBrakeOutput(NULL, 0): the brake
/// engages again and the motor stays enabled (:455-456).
#[epics_rs::base::epics_test]
async fn clearing_disable_motor_engages_without_disabling() {
    let (line, _io, port) = mcc("phyBrakeDisable").await;
    wire(&line);
    let disabled = off_runtime(move || {
        let disable = reason(&port, "AXIS_DISABLE_MOTOR");
        port.write_int32_blocking(disable, 1, 0).unwrap();
        port.read_int32_blocking(disable, 1).unwrap()
    })
    .await;
    assert_eq!(disabled, 0);
    assert_eq!(wire(&line), ["A3R"]);
}
