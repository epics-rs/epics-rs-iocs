//! C `phytronController`'s constructor asks the controller's type before
//! its poller starts (phytronAxisMotor.cpp:214-240).

#![cfg(target_os = "linux")]

use std::sync::Arc;
use std::time::Duration;

use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use motor_common::testing::LineRig;

/// A controller that ACKs every frame: an MCC-2 with a chopper stage, or a
/// phyMOTION whose frames carry the CRC both ways.
fn reply(frame: &str) -> String {
    match frame.strip_suffix('\x03').and_then(|f| f.rsplit_once(':')) {
        Some(_) => "\x02\x06phyMOTION:XX\x03".into(),
        None => {
            let payload = match &frame[2..frame.len() - 1] {
                "IAR" => "2",
                "1P48R" => "0",
                "SUI" => "I=00000000",
                _ => "",
            };
            format!("\x02\x06{payload}\x03")
        }
    }
}

/// The commands `create` (0 phyMOTION, 1 MCC) sends before it returns.
async fn created(name: &str, create: usize) -> Vec<String> {
    let io_port = format!("{name}_IO");
    let (line, _io) = LineRig::new(&io_port, reply);
    let command = motor_phytron::ioc::phytron_commands().swap_remove(create);
    let s = |v: &str| ArgValue::String(v.into());
    let mut args = vec![s(name), s(&io_port)];
    if create == 1 {
        args.push(ArgValue::Int(0));
    }
    args.extend([
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ]);
    let bridge = BlockingBridge::capture();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        command.handler.call(&args, &ctx).unwrap();
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap();
    line.sent()
}

#[epics_rs::base::epics_test]
async fn an_mcc_asks_iar_then_1p48r_before_its_first_poll() {
    let sent = created("phyTypeMcc", 1).await;
    let wire: Vec<&str> = sent.iter().map(|f| &f[2..f.len() - 1]).collect();
    assert_eq!(wire[..2], ["IAR", "1P48R"], "{wire:?}");
}

#[epics_rs::base::epics_test]
async fn a_phymotion_asks_im0() {
    let sent = created("phyTypeMcm", 0).await;
    let wire: Vec<&str> = sent.iter().map(|f| &f[2..f.len() - 4]).collect();
    assert_eq!(wire, ["IM0"]);
}
