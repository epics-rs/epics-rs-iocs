//! C `MCS2Controller::writeInt32` and `MCS2Axis::poll`
//! (smarActMCS2MotorDriver.cpp:209-260, 459-562): the positioner type,
//! max clock frequency, hold time and calibration of a channel.

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
use motor_common::testing::LineRig;

/// An idle channel 0 with a sensor, calibrated (`0x60`), positioner type
/// 300, MCLF 5000 and hold time 60.
fn mcs2_reply(cmd: &str) -> String {
    let reply = match cmd.trim_end() {
        ":SYST:ERR:COUN?" => "0",
        ":DEV:SNUM?" => "MCS2-00001",
        ":CHAN0:STAT?" => "96",
        ":CHAN0:POS?" | ":CHAN0:POS:TARG?" => "0",
        ":CHAN0:AMPL?" => "1",
        ":CHAN0:PTYP?" => "300",
        ":CHAN0:MCLF?" => "5000",
        ":CHAN0:HOLD?" => "60",
        _ => "",
    };
    format!("{reply}\r\n")
}

/// `MCS2CreateController(name, ..., 1 axis)` on a line answering
/// `mcs2_reply`, then `body` on the shell thread with the controller port.
async fn on_mcs2<T: Send + 'static>(
    name: &str,
    body: impl FnOnce(PortHandle) -> T + Send + 'static,
) -> (T, Vec<String>) {
    let io_port = format!("{name}_IO");
    let (line, _io) = LineRig::new(&io_port, mcs2_reply);
    let args = vec![
        ArgValue::String(name.into()),
        ArgValue::String(io_port.into()),
        ArgValue::Int(1),
        ArgValue::Int(100),
        ArgValue::Int(100),
        ArgValue::Int(0),
    ];
    let cmd = motor_smaract::ioc::mcs2_create_controller_command();
    let bridge = BlockingBridge::capture();
    let port = name.to_owned();
    let line = Arc::new(Mutex::new(line));
    let rig = line.clone();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        cmd.handler.call(&args, &ctx).unwrap();
        rig.lock().unwrap().sent();
        body(PortManager::global().find_port_handle(&port).unwrap())
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    let out = shell.join().unwrap();
    let sent = line.lock().unwrap().sent();
    (out, sent)
}

fn reason(port: &PortHandle, name: &str) -> usize {
    port.drv_user_create_blocking(&DrvUserRequest::new(name, 0))
        .unwrap()
        .reason
}

#[epics_rs::base::epics_test]
async fn a_channel_parameter_write_is_sent_and_stored_as_c() {
    let (stored, sent) = on_mcs2("mcs2AuxWrite", |port| {
        let writes = [("PTYP", 301), ("MCLF", 4000), ("CAL", 1), ("HOLD", 50)];
        for (name, value) in writes {
            port.write_int32_blocking(reason(&port, name), 0, value)
                .unwrap();
        }
        // The poll reads the type back into PTYP_RB, leaving PTYP alone.
        port.read_int32_blocking(reason(&port, "PTYP"), 0).unwrap()
    })
    .await;
    let sent: Vec<&str> = sent.iter().map(|c| c.trim_end()).collect();
    for want in [
        ":CHAN0:PTYP 301",
        ":CHAN0:MCLF:CURR 4000",
        ":CAL0",
        ":CHAN0:HOLD 50",
    ] {
        assert!(sent.contains(&want), "{want}: {sent:?}");
    }
    assert_eq!(stored, 301);
}

#[epics_rs::base::epics_test]
async fn the_poll_reads_the_channel_parameters_back() {
    let (values, _) = on_mcs2("mcs2AuxPoll", |port| {
        let names = ["PTYP_RB", "PSTAT", "CAL", "MCLF", "HOLD"];
        let reasons: Vec<usize> = names.iter().map(|n| reason(&port, n)).collect();
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            // Undefined until the poller thread's startup pass sets it.
            let values: Vec<i32> = reasons
                .iter()
                .map(|&r| port.read_int32_blocking(r, 0).unwrap_or(0))
                .collect();
            if values.iter().all(|&v| v != 0) || Instant::now() > deadline {
                return values;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    })
    .await;
    assert_eq!(values, [300, 96, 1, 5000, 60]);
}

/// C traces every channel write at its end, the base's included
/// (smarActMCS2MotorDriver.cpp:250-257).
#[epics_rs::base::epics_test]
async fn every_write_prints_the_driver_s_line() {
    let path = std::env::temp_dir().join(format!("mcs2AuxTrace-{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    let (functions, _) = on_mcs2("mcs2AuxTrace", move |port| {
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        let file = Arc::new(Mutex::new(file));
        trace.set_trace_mask(
            Some("mcs2AuxTrace"),
            TraceMask::ERROR | TraceMask::IO_DRIVER,
        );
        trace.set_trace_file(Some("mcs2AuxTrace"), TraceFile::File(file.clone()));
        trace.set_device_trace_file("mcs2AuxTrace", 0, TraceFile::File(file));
        let hold = reason(&port, "HOLD");
        // MOTOR_CLOSED_LOOP, which the base handles.
        let closed_loop = reason(&port, "MOTOR_CLOSED_LOOP");
        port.write_int32_blocking(hold, 0, 50).unwrap();
        port.write_int32_blocking(closed_loop, 0, 1).unwrap();
        (hold, closed_loop)
    })
    .await;
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let (hold, closed_loop) = functions;
    for line in [
        format!("SmarActMCS2MotorDriver:writeInt32: function={hold}, value=50\n"),
        format!("SmarActMCS2MotorDriver:writeInt32: function={closed_loop}, value=1\n"),
    ] {
        assert_eq!(text.matches(&line).count(), 1, "{line}{text}");
    }
}

/// A user at address -1 writes channel 0, getAddress's axis for it
/// (asynPortDriver.cpp:1901-1914).
#[epics_rs::base::epics_test]
async fn a_write_at_address_minus_one_reaches_channel_0() {
    let (_, sent) = on_mcs2("mcs2AddrM1", |port| {
        port.write_int32_blocking(reason(&port, "PTYP"), -1, 302)
            .unwrap();
    })
    .await;
    let sent: Vec<&str> = sent.iter().map(|c| c.trim_end()).collect();
    assert!(sent.contains(&":CHAN0:PTYP 302"), "{sent:?}");
}
