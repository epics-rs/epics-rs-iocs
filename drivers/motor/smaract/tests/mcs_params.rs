//! C `SmarActMCSController::writeInt32` and `SmarActMCSAxis::poll`
//! (smarActMCSMotorDriver.cpp:274-329, 443-510): the positioner type,
//! calibration, max clock frequency, hold time and auto-zero of an axis,
//! here axis 1 on channel 0.

#![cfg(target_os = "linux")]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::{TraceFile, TraceMask};
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use epics_rs::motor::controller_port::index;
use motor_common::testing::LineRig;

/// An idle linear channel 0 with sensor type 300; `:SST0,999` is refused.
fn mcs_reply(cmd: &str) -> String {
    let reply = match cmd.trim_end() {
        ":GCLS0" => ":CLS0,0",
        ":GS0" => ":S0,0",
        ":GP0" => ":P0,0",
        ":GST0" => ":ST0,300",
        ":GPPK0" => ":PPK0,1",
        ":SST0,999" => ":E0,5",
        // Nothing is waiting on the line before the first command.
        "" => return String::new(),
        c if c.starts_with(":G") => "",
        _ => ":E0,0",
    };
    format!("{reply}\n")
}

/// `smarActMCSCreateController(name, ..., 2 axes)` and
/// `smarActMCSCreateAxis(name, 1, 0)` on a line answering `mcs_reply`, then
/// `body` on the shell thread with the controller port and the line.
async fn on_mcs<T: Send + 'static>(
    name: &str,
    body: impl FnOnce(PortHandle, &Mutex<LineRig>) -> T + Send + 'static,
) -> T {
    on_line(name, mcs_reply, body).await
}

/// [`on_mcs`] on a line answering `reply`.
async fn on_line<T: Send + 'static>(
    name: &str,
    reply: impl Fn(&str) -> String + Send + Sync + 'static,
    body: impl FnOnce(PortHandle, &Mutex<LineRig>) -> T + Send + 'static,
) -> T {
    let io_port = format!("{name}_IO");
    let (line, _io) = LineRig::new(&io_port, reply);
    let controller_args = vec![
        ArgValue::String(name.into()),
        ArgValue::String(io_port.into()),
        ArgValue::Int(2),
        ArgValue::Double(0.05),
        ArgValue::Double(0.05),
        ArgValue::Int(0),
    ];
    let axis_args = vec![
        ArgValue::String(name.into()),
        ArgValue::Int(1),
        ArgValue::Int(0),
    ];
    let create = motor_smaract::ioc::mcs_create_controller_command();
    let axis = motor_smaract::ioc::mcs_create_axis_command();
    let bridge = BlockingBridge::capture();
    let port = name.to_owned();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        create.handler.call(&controller_args, &ctx).unwrap();
        axis.handler.call(&axis_args, &ctx).unwrap();
        motor_common::testing::await_first_polls();
        let line = Mutex::new(line);
        line.lock().unwrap().sent();
        body(
            PortManager::global().find_port_handle(&port).unwrap(),
            &line,
        )
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap()
}

fn reason(port: &PortHandle, name: &str) -> usize {
    port.drv_user_create_blocking(&DrvUserRequest::new(name, 0))
        .unwrap()
        .reason
}

/// Wait for `line` to have sent everything in `want`.
fn wait_sent(line: &Mutex<LineRig>, want: &[&str]) -> Vec<String> {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut sent = Vec::new();
    loop {
        sent.extend(line.lock().unwrap().sent());
        let trimmed: Vec<&str> = sent.iter().map(|c| c.trim_end()).collect();
        if want.iter().all(|w| trimmed.contains(w)) || Instant::now() > deadline {
            return sent;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[epics_rs::base::epics_test]
async fn a_channel_parameter_write_is_sent_on_the_axis_s_channel() {
    let (sent, stored) = on_mcs("mcsAuxWrite", |port, line| {
        for (name, value) in [("PTYP", 301), ("CAL", 1), ("MCLF", 4000)] {
            port.write_int32_blocking(reason(&port, name), 1, value)
                .unwrap();
        }
        let sent = wait_sent(line, &[]);
        (
            sent,
            port.read_int32_blocking(reason(&port, "PTYP"), 1).unwrap(),
        )
    })
    .await;
    let sent: Vec<&str> = sent.iter().map(|c| c.trim_end()).collect();
    for want in [":SST0,301", ":CS0", ":SCLF0,4000"] {
        assert!(sent.contains(&want), "{want}: {sent:?}");
    }
    assert_eq!(stored, 301);
}

/// A PTYP write runs C's `checkType`, so a stage that becomes rotary
/// moves by angle (smarActMCSMotorDriver.cpp:293-299, 331-344).
#[epics_rs::base::epics_test]
async fn a_positioner_type_write_rechecks_rotation() {
    let rotary = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let set = rotary.clone();
    let reply = move |cmd: &str| {
        let cmd = cmd.trim_end();
        if cmd.starts_with(":SST0,") {
            set.store(true, std::sync::atomic::Ordering::SeqCst);
        }
        match cmd {
            ":GP0" if set.load(std::sync::atomic::Ordering::SeqCst) => ":E0,140\n".into(),
            ":GA0" => ":A0,0,0\n".into(),
            _ => mcs_reply(cmd),
        }
    };
    let sent = on_line("mcsAuxType", reply, |port, line| {
        port.write_int32_blocking(reason(&port, "PTYP"), 1, 301)
            .unwrap();
        port.write_float64_blocking(index::VELOCITY, 1, 1000.0)
            .unwrap();
        port.write_float64_blocking(index::MOVE_ABS, 1, 1234.0)
            .unwrap();
        wait_sent(line, &[":MAA0,1234,0,0"])
    })
    .await;
    assert!(rotary.load(std::sync::atomic::Ordering::SeqCst));
    let sent: Vec<&str> = sent.iter().map(|c| c.trim_end()).collect();
    assert!(sent.contains(&":MAA0,1234,0,0"), "{sent:?}");
}

/// C's move and home read HOLD and AUTO_ZERO back at the axis number
/// (smarActMCSMotorDriver.cpp:585, 618-621).
#[epics_rs::base::epics_test]
async fn moves_and_homes_send_the_axis_s_hold_time_and_auto_zero() {
    let sent = on_mcs("mcsAuxMove", |port, line| {
        port.write_int32_blocking(reason(&port, "HOLD"), 1, 50)
            .unwrap();
        port.write_int32_blocking(reason(&port, "AUTO_ZERO"), 1, 0)
            .unwrap();
        port.write_float64_blocking(index::VELOCITY, 1, 1000.0)
            .unwrap();
        port.write_float64_blocking(index::MOVE_ABS, 1, 1234.0)
            .unwrap();
        port.write_float64_blocking(index::HOME, 1, 1.0).unwrap();
        wait_sent(line, &[":MPA0,1234,50", ":FRM0,0,50,0"])
    })
    .await;
    let sent: Vec<&str> = sent.iter().map(|c| c.trim_end()).collect();
    for want in [":MPA0,1234,50", ":FRM0,0,50,0"] {
        assert!(sent.contains(&want), "{want}: {sent:?}");
    }
}

/// The constructor sets AUTO_ZERO 1 and HOLD 0, and the poll reads the
/// sensor type into PTYP_RB (smarActMCSMotorDriver.cpp:367-368, 495-498).
#[epics_rs::base::epics_test]
async fn the_axis_posts_its_defaults_and_the_poll_its_sensor_type() {
    let values = on_mcs("mcsAuxPoll", |port, _| {
        let reasons: Vec<usize> = ["PTYP_RB", "AUTO_ZERO", "HOLD"]
            .iter()
            .map(|n| reason(&port, n))
            .collect();
        reasons
            .iter()
            .map(|&r| port.read_int32_blocking(r, 1).unwrap())
            .collect::<Vec<i32>>()
    })
    .await;
    assert_eq!(values, [300, 1, 0]);
}

/// C traces a write at its end, the base's included, but returns before
/// it when the controller refuses, and refuses an address with no axis
/// with its own line (smarActMCSMotorDriver.cpp:282-286, 293-298, 318-327).
#[epics_rs::base::epics_test]
async fn writes_print_c_s_lines() {
    let name = "mcsAuxTrace";
    let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    let functions = on_mcs(name, move |port, _| {
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        let file = Arc::new(Mutex::new(file));
        trace.set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_trace_file(Some(name), TraceFile::File(file.clone()));
        for addr in 0..2 {
            trace.set_device_trace_mask(name, addr, TraceMask::ERROR | TraceMask::IO_DRIVER);
            trace.set_device_trace_file(name, addr, TraceFile::File(file.clone()));
        }
        let ptyp = reason(&port, "PTYP");
        let hold = reason(&port, "HOLD");
        port.write_int32_blocking(ptyp, 1, 301).unwrap();
        assert!(port.write_int32_blocking(ptyp, 1, 999).is_err());
        port.write_int32_blocking(hold, 1, 7).unwrap();
        assert!(port.write_int32_blocking(hold, 0, 7).is_err());
        (ptyp, hold)
    })
    .await;
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let (ptyp, hold) = functions;
    let lines: Vec<&str> = text
        .lines()
        .filter(|l| l.contains("SmarActMCSController"))
        .collect();
    let expected = [
        format!("SmarActMCSController:writeInt32: function={ptyp}, value=301"),
        format!("SmarActMCSController:writeInt32: function={hold}, value=7"),
        format!("SmarActMCSController:writeInt32: error, function: {hold}. Invalid axis number."),
    ];
    assert_eq!(lines.len(), expected.len(), "{text}");
    for (line, want) in lines.iter().zip(&expected) {
        assert!(line.ends_with(want.as_str()), "{line:?} vs {want:?}");
    }
}

/// A refused move raises CommsError until the next poll rewrites it from
/// its own status (smarActMCSMotorDriver.cpp:504-505, 599-604); the rig
/// holds that poll back so the raised bit can be read.
#[epics_rs::base::epics_test]
async fn a_refused_move_raises_comms_error() {
    let refused = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let set = refused.clone();
    let reply = move |cmd: &str| {
        let cmd = cmd.trim_end();
        if cmd.starts_with(":MPA0,") {
            set.store(true, std::sync::atomic::Ordering::SeqCst);
            return ":E0,5\n".into();
        }
        if cmd == ":GS0" && set.load(std::sync::atomic::Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(500));
        }
        mcs_reply(cmd)
    };
    let raised = on_line("mcsCommsError", reply, |port, _| {
        port.write_float64_blocking(index::VELOCITY, 1, 1000.0)
            .unwrap();
        let _ = port.write_float64_blocking(index::MOVE_ABS, 1, 1234.0);
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let error = port.read_int32_blocking(index::STATUS_COMMS_ERROR, 1).ok();
            if error == Some(1) || Instant::now() > deadline {
                return error;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    })
    .await;
    assert!(refused.load(std::sync::atomic::Ordering::SeqCst));
    assert_eq!(raised, Some(1));
}
