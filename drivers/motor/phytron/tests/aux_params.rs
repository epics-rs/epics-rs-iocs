//! C `phytronController::readInt32`/`writeInt32`/`readFloat64`
//! (phytronAxisMotor.cpp:285-357, 365-466, 472-512): an axis's controller
//! parameters go to its P registers.

#![cfg(target_os = "linux")]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use motor_common::testing::LineRig;

/// An MCC at address 0 whose axis 1 holds `P41 = 45` (10 mA units),
/// `P54 = 235` (0.1 °C) and `P36 = 1` and sits on its positive limit; a
/// `P34` read and a status reset are refused, the controller status is 42.
fn mcc_reply(frame: &str) -> String {
    let payload = match &frame[2..frame.len() - 1] {
        "SUI" => "I=+0000000",
        "ST" => "42",
        "1P34R" | "STC" => return "\x02\x15\x03".into(),
        "1P41R" => "45",
        "1P54R" => "235",
        "1P36R" => "1",
        c if c.ends_with("=H") => "E",
        c if c.ends_with("=E") || c.ends_with("P20R") || c.ends_with("P22R") => "0",
        _ => "",
    };
    format!("\x02\x06{payload}\x03")
}

/// `phytronCreateMCC(name, ...)` and `phytronCreateAxis(name, 0, 1)` on a
/// line answering `mcc_reply`, then `body` on the shell thread with the
/// controller port.
async fn on_mcc<T: Send + 'static>(
    name: &str,
    body: impl FnOnce(PortHandle) -> T + Send + 'static,
) -> (T, Vec<String>) {
    let io_port = format!("{name}_IO");
    let (line, _io) = LineRig::new(&io_port, mcc_reply);
    let mut commands = motor_phytron::ioc::phytron_commands().into_iter();
    let (_phymotion, create_mcc, create_axis) = (
        commands.next().unwrap(),
        commands.next().unwrap(),
        commands.next().unwrap(),
    );
    let mcc_args = vec![
        ArgValue::String(name.into()),
        ArgValue::String(io_port.into()),
        ArgValue::Int(0),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ];
    let axis_args = vec![
        ArgValue::String(name.into()),
        ArgValue::Int(0),
        ArgValue::Int(1),
    ];
    let bridge = BlockingBridge::capture();
    let port = name.to_owned();
    let line = Arc::new(Mutex::new(line));
    let rig = line.clone();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        create_mcc.handler.call(&mcc_args, &ctx).unwrap();
        create_axis.handler.call(&axis_args, &ctx).unwrap();
        rig.lock().unwrap().sent();
        body(PortManager::global().find_port_handle(&port).unwrap())
    });
    // An axis joining the poller waits on a task this runtime runs.
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    let out = shell.join().unwrap();
    let sent = line.lock().unwrap().sent();
    (out, sent)
}

fn reason(port: &PortHandle, name: &str) -> usize {
    port.drv_user_create_blocking(&DrvUserRequest::new(name, 1))
        .unwrap()
        .reason
}

#[epics_rs::base::epics_test]
async fn an_axis_parameter_write_sets_its_p_register_as_c() {
    let (_, sent) = on_mcc("phyAuxWrite", |port| {
        port.write_int32_blocking(reason(&port, "RUN_CURRENT"), 1, 1239)
            .unwrap();
        port.write_int32_blocking(reason(&port, "ENC_FUNCTION"), 1, 5)
            .unwrap();
        port.write_int32_blocking(reason(&port, "AXIS_MODE"), 1, 2)
            .unwrap();
        assert_eq!(
            port.read_int32_blocking(reason(&port, "AXIS_MODE"), 1)
                .unwrap(),
            0,
            "a read asks the controller, whose P01 here is empty"
        );
    })
    .await;
    // RUN_CURRENT's EGU is mA, the register's 10 mA; P36 is 1 for any
    // positive ENC_FUNCTION.
    let wire: Vec<&str> = sent.iter().map(|f| &f[2..f.len() - 1]).collect();
    // A write's register is two digits, a read's `to_string` (:380, :353).
    for want in ["1P41S123", "1P36S1", "1P01S2", "1P1R"] {
        assert!(wire.contains(&want), "{want}: {wire:?}");
    }
}

#[epics_rs::base::epics_test]
async fn an_axis_parameter_read_asks_its_p_register_as_c() {
    let ((current, func, temp, refused), _) = on_mcc("phyAuxRead", |port| {
        (
            port.read_int32_blocking(reason(&port, "RUN_CURRENT"), 1)
                .unwrap(),
            port.read_int32_blocking(reason(&port, "ENC_FUNCTION"), 1)
                .unwrap(),
            port.read_float64_blocking(reason(&port, "MOTOR_TEMP"), 1)
                .unwrap(),
            port.read_int32_blocking(reason(&port, "ENCODER_TYP"), 1)
                .is_err(),
        )
    })
    .await;
    assert_eq!(current, 450);
    assert_eq!(func, 1);
    assert_eq!(temp, 23.5);
    assert!(refused, "a NAK fails the read");
}

/// C `CHECK_AXIS` prints a failure once until the controller's last
/// status changes (phytronAxisMotor.cpp:60-70).
#[epics_rs::base::epics_test]
async fn a_failed_parameter_read_prints_check_axis_once() {
    use epics_rs::asyn::trace::TraceFile;
    let dir = std::env::temp_dir().join(format!("phyAuxTrace-{}", std::process::id()));
    let file = std::fs::File::create(&dir).unwrap();
    on_mcc("phyAuxTrace", move |port| {
        PortManager::global().trace_manager().set_trace_file(
            Some("phyAuxTrace"),
            TraceFile::File(Arc::new(Mutex::new(file))),
        );
        for _ in 0..2 {
            assert!(
                port.read_int32_blocking(reason(&port, "ENCODER_TYP"), 1)
                    .is_err()
            );
        }
    })
    .await;
    let text = std::fs::read_to_string(&dir).unwrap();
    let _ = std::fs::remove_file(&dir);
    let line = "phytronAxis::readInt32: (phyAuxTrace) Reading parameter for axis 1 \
                failed with error code: 6\n";
    assert_eq!(text.matches(line).count(), 1, "{text}");
}

/// C `phytronController::writeInt32`/`writeFloat64` print the missing axis
/// for any reason (phytronAxisMotor.cpp:386-391, 521-525).
#[epics_rs::base::epics_test]
async fn a_write_with_no_axis_prints_axis_not_found() {
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::motor::controller_port::index;
    let path = std::env::temp_dir().join(format!("phyNoAxis-{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    on_mcc("phyNoAxis", move |port| {
        let file = Arc::new(Mutex::new(file));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_file(Some("phyNoAxis"), TraceFile::File(file.clone()));
        // pasynUserSelf is address 0, whose device trace has its own file.
        trace.set_device_trace_file("phyNoAxis", 0, TraceFile::File(file));
        assert!(
            port.write_int32_blocking(index::REC_DIRECTION, 2, 1)
                .is_err()
        );
        assert!(
            port.write_int32_blocking(reason(&port, "RUN_CURRENT"), 2, 100)
                .is_err()
        );
        assert!(
            port.write_float64_blocking(index::VELOCITY, 2, 1.0)
                .is_err()
        );
    })
    .await;
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let int = "phytronAxis::writeInt32: Axis not found on the controller phyNoAxis\n";
    let float = "phytronAxis::writeFloat64: Axis not found on the controller phyNoAxis\n";
    assert_eq!(text.matches(int).count(), 2, "{text}");
    assert_eq!(text.matches(float).count(), 1, "{text}");
}

/// C `phytronController::writeInt32`/`readInt32` answer the controller
/// reasons at any address (phytronAxisMotor.cpp:293-303, 375-386).
#[epics_rs::base::epics_test]
async fn a_controller_reset_sends_cr_and_sets_every_encoder_ratio_to_1() {
    use epics_rs::asyn::param::ParamValue;
    use epics_rs::asyn::request::ParamSetValue;
    use epics_rs::motor::controller_port::index;
    let ((ratio, status), sent) = on_mcc("phyCtrlReset", |port| {
        // The ratio alone, without the axis write that would store it again.
        let ratio = ParamSetValue::new(index::ENCODER_RATIO, 1, ParamValue::Float64(0.5));
        port.set_params_and_notify_blocking(1, vec![ratio]).unwrap();
        port.write_int32_blocking(reason(&port, "CONTROLLER_RESET"), 0, 1)
            .unwrap();
        (
            port.read_float64_blocking(index::ENCODER_RATIO, 1).unwrap(),
            port.read_int32_blocking(reason(&port, "CONTROLLER_STATUS"), 0)
                .unwrap(),
        )
    })
    .await;
    assert_eq!(ratio, 1.0);
    assert_eq!(status, 42);
    let wire: Vec<&str> = sent.iter().map(|f| &f[2..f.len() - 1]).collect();
    for want in ["CR", "ST"] {
        assert!(wire.contains(&want), "{want}: {wire:?}");
    }
}

/// C `CHECK_CTRL` prints a failed controller command once until the
/// status changes (phytronAxisMotor.cpp:48-58).
#[epics_rs::base::epics_test]
async fn a_failed_status_reset_prints_check_ctrl_once() {
    use epics_rs::asyn::trace::TraceFile;
    let path = std::env::temp_dir().join(format!("phyCtrlTrace-{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    let (status_reset, _) = on_mcc("phyCtrlTrace", move |port| {
        let file = Arc::new(Mutex::new(file));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_file(Some("phyCtrlTrace"), TraceFile::File(file.clone()));
        trace.set_device_trace_file("phyCtrlTrace", 0, TraceFile::File(file));
        let status_reset = reason(&port, "CONTROLLER_STATUS_RESET");
        for _ in 0..2 {
            assert!(port.write_int32_blocking(status_reset, 0, 1).is_err());
        }
        status_reset
    })
    .await;
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let line = format!(
        "phytronController::writeInt32: (phyCtrlTrace) Reseting status failed with error \
         code: 6, reason: {status_reset}\n"
    );
    assert_eq!(text.matches(&line).count(), 1, "{text}");
}

/// C sends `{m}C` for AXIS_RESET; AXIS_STATUS_RESET is phyMOTION's only
/// (phytronAxisMotor.cpp:399-404).
#[epics_rs::base::epics_test]
async fn an_axis_reset_sends_c_and_an_mcc_refuses_a_status_reset() {
    let (refused, sent) = on_mcc("phyAxisReset", |port| {
        port.write_int32_blocking(reason(&port, "AXIS_RESET"), 1, 1)
            .unwrap();
        port.write_int32_blocking(reason(&port, "AXIS_STATUS_RESET"), 1, 1)
            .is_err()
    })
    .await;
    assert!(refused);
    let wire: Vec<&str> = sent.iter().map(|f| &f[2..f.len() - 1]).collect();
    assert!(wire.contains(&"1C"), "{wire:?}");
    assert!(!wire.iter().any(|c| c.starts_with("SEC")), "{wire:?}");
}

/// C `phytronAxis::poll` sets AXIS_STATUS to the axis status word
/// (phytronAxisMotor.cpp:2084).
#[epics_rs::base::epics_test]
async fn the_poll_sets_axis_status() {
    let (status, _) = on_mcc("phyAxisStatus", |port| {
        let axis_status = reason(&port, "AXIS_STATUS");
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            let status = port.read_int32_blocking(axis_status, 1).unwrap();
            if status != 0 || std::time::Instant::now() > deadline {
                return status;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    })
    .await;
    assert_eq!(status, 0x10, "the positive limit switch");
}

/// C `phytronController::writeOption`/`readOption` "pollMethod"
/// (phytronAxisMotor.cpp:597-606, 668-725): an MCC polls serially only,
/// and an axis must exist.
#[epics_rs::base::epics_test]
async fn the_poll_method_option_is_the_controllers_and_an_mcc_refuses_parallel() {
    use epics_rs::asyn::user::AsynUser;
    let ((refused, missing, axis, shown), _) = on_mcc("phyPollOption", |port| {
        let at = |addr| AsynUser::default().with_addr(addr);
        let refused = port
            .set_option_blocking(at(0), "pollMethod", "parallel")
            .is_err();
        let missing = port
            .set_option_blocking(at(2), "POLLMETHOD", "serial")
            .is_err();
        let axis = port.set_option_blocking(at(1), "pollMethod", "default");
        port.set_option_blocking(at(0), "pollMethod", " old ")
            .unwrap();
        let shown = port.get_option_blocking(at(0), "pollmethod").unwrap();
        (refused, missing, axis.is_ok(), shown)
    })
    .await;
    assert!(refused, "an MCC has no parallel poll");
    assert!(missing, "no axis 2");
    assert!(axis);
    assert_eq!(shown, "0/serial");
}
