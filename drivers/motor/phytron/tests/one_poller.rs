//! C `phytronController` starts one `asynMotorPoller` in its constructor
//! (phytronAxisMotor.cpp:240), `phytronCreateAxis` axes join it, and its
//! `poll` reads an MCC's `SUI` once per pass for all of them.

#![cfg(target_os = "linux")]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext, CommandDef};
use motor_common::testing::{pollers, pollers_after};

/// An MCC at address 0 that ACKs every frame, with idle, fault-free axes;
/// records each command payload.
struct Mcc {
    base: PortDriverBase,
    sent: Arc<Mutex<Vec<String>>>,
    reply: Vec<u8>,
}

impl PortDriver for Mcc {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let cmd = String::from_utf8_lossy(&data[2..data.len() - 1]).into_owned();
        let payload = match cmd.as_str() {
            "SUI" => "I=00000000",
            c if c.ends_with("=H") => "E",
            c if c.ends_with("=E") || c.ends_with("P20R") || c.ends_with("P22R") => "0",
            _ => "",
        };
        self.reply = format!("\x02\x06{payload}\x03").into_bytes();
        self.sent.lock().unwrap().push(cmd);
        Ok(data.len())
    }
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let reply = std::mem::take(&mut self.reply);
        buf[..reply.len()].copy_from_slice(&reply);
        Ok(reply.len())
    }
}

/// Port `name` registered as `drvAsynSerialPortConfigure` does; the
/// commands written to it.
fn mcc_port(name: &str) -> Arc<Mutex<Vec<String>>> {
    let sent = Arc::new(Mutex::new(Vec::new()));
    let mcc = Mcc {
        base: PortDriverBase::new(name, 1, PortFlags::default()),
        sent: sent.clone(),
        reply: Vec::new(),
    };
    let claim = claim_port(name).unwrap();
    let (rt, _actor) = create_port_runtime(&claim, mcc, RuntimeConfig::default()).unwrap();
    claim
        .publish(rt.port_handle().clone(), &open_owner())
        .unwrap();
    std::mem::forget(rt);
    sent
}

/// `phytronCreateMCC` on a fresh line, then four `phytronCreateAxis` axes;
/// what the line was sent and the poller count after the controller command.
async fn mcc_with_four_axes(name: &str) -> (Arc<Mutex<Vec<String>>>, usize) {
    let io_port = format!("{name}_IO");
    let sent = mcc_port(&io_port);
    let mut commands = motor_phytron::ioc::phytron_commands().into_iter();
    let (_phymotion, create_mcc, create_axis) = (
        commands.next().unwrap(),
        commands.next().unwrap(),
        commands.next().unwrap(),
    );
    let args = vec![
        ArgValue::String(name.into()),
        ArgValue::String(io_port.into()),
        ArgValue::Int(0),
        ArgValue::Int(20),
        ArgValue::Int(20),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ];
    let started = pollers_after(create_mcc, args).await;
    run(create_axis, name, 4);
    (sent, started)
}

/// `phytronCreateAxis(name, 0, index)` for index 1..=n, on a thread of its
/// own as the shell runs it.
fn run(create_axis: CommandDef, name: &str, n: i64) {
    let bridge = BlockingBridge::capture();
    let name = name.to_owned();
    std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        for index in 1..=n {
            let args = vec![
                ArgValue::String(name.clone().into()),
                ArgValue::Int(0),
                ArgValue::Int(index),
            ];
            create_axis.handler.call(&args, &ctx).unwrap();
        }
    })
    .join()
    .unwrap();
}

#[epics_rs::base::epics_test]
async fn four_mcc_axes_poll_on_the_one_poller_the_controller_starts() {
    let (_sent, started) = mcc_with_four_axes("PHY_ONE").await;
    assert_eq!(started, 1);
    sleep(Duration::from_millis(200)).await;
    assert_eq!(pollers(), 1);
}

#[epics_rs::base::epics_test]
async fn an_mcc_poll_pass_reads_sui_once_for_all_its_axes() {
    let (line, _) = mcc_with_four_axes("PHY_SUI").await;
    sleep(Duration::from_millis(300)).await;
    line.lock().unwrap().clear();
    sleep(Duration::from_millis(300)).await;
    let sent = line.lock().unwrap().clone();
    let passes: Vec<&[String]> = sent.split(|c| c == "SUI").collect();
    assert!(passes.len() > 2, "{sent:?}");
    // The first and last slices are cut by the window.
    for pass in &passes[1..passes.len() - 1] {
        let axes: Vec<&str> = pass.iter().filter_map(|c| c.strip_suffix("=H")).collect();
        assert_eq!(axes, ["1", "2", "3", "4"], "{sent:?}");
    }
}
