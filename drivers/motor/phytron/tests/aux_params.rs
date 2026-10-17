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
/// `P54 = 235` (0.1 °C) and `P36 = 1`; a `P34` read is refused.
fn mcc_reply(frame: &str) -> String {
    let payload = match &frame[2..frame.len() - 1] {
        "SUI" => "I=00000000",
        "1P41R" => "45",
        "1P54R" => "235",
        "1P36R" => "1",
        "1P34R" => return "\x02\x15\x03".into(),
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
