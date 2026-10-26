//! C `phytronController::epicsInithookFunction` (phytronAxisMotor.cpp:
//! 1003-1024) holds iocInit at initHookAfterInitDevSup until the controller
//! has been read, so motorRecord init_record pass 1 sees polled values.

#![cfg(target_os = "linux")]

use std::sync::Arc;
use std::time::Duration;

use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_app::{InitHookState, init_hook_announce};
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};
use motor_common::testing::LineRig;

/// An MCC at address 0 that ACKs every frame, with idle, fault-free axes.
fn mcc_reply(frame: &str) -> String {
    let payload = match &frame[2..frame.len() - 1] {
        "SUI" => "I=00000000",
        c if c.ends_with("=H") => "E",
        c if c.ends_with("=E") || c.ends_with("P20R") || c.ends_with("P22R") => "0",
        _ => "",
    };
    format!("\x02\x06{payload}\x03")
}

/// Run `f` on a thread of its own, as the shell and iocInit do.
async fn shell<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let thread = std::thread::spawn(f);
    while !thread.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    thread.join().unwrap()
}

/// An axis created after the controller's startup pass is read before
/// initHookAfterInitDevSup returns, not a poll period later.
#[epics_rs::base::epics_test]
async fn after_init_dev_sup_waits_for_the_axis_first_read() {
    let name = "PHY_HOOK";
    let io_port = format!("{name}_IO");
    let (line, _io) = LineRig::new(&io_port, mcc_reply);
    let mut commands = motor_phytron::ioc::phytron_commands().into_iter();
    let (_phymotion, create_mcc, create_axis) = (
        commands.next().unwrap(),
        commands.next().unwrap(),
        commands.next().unwrap(),
    );
    let bridge = BlockingBridge::capture();
    shell(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        let s = |v: &str| ArgValue::String(v.into());
        let mcc = vec![
            s(name),
            s(&io_port),
            ArgValue::Int(0),
            ArgValue::Int(1000),
            ArgValue::Int(1000),
            ArgValue::Double(1000.0),
            ArgValue::Int(1),
        ];
        create_mcc.handler.call(&mcc, &ctx).unwrap();
        // Past the startup pass and its forced fast polls' first.
        std::thread::sleep(Duration::from_millis(150));
        let axis = vec![s(name), ArgValue::Int(0), ArgValue::Int(1)];
        create_axis.handler.call(&axis, &ctx).unwrap();
    })
    .await;
    // No axis existed at the startup pass, so any `1=H` is the axis's read:
    // motor-rs polls a joined axis before its add returns, else the hook
    // waits for the next pass.
    shell(|| init_hook_announce(InitHookState::AfterInitDevSup)).await;
    let sent: Vec<String> = line
        .sent()
        .iter()
        .map(|f| f[2..f.len() - 1].to_owned())
        .collect();
    assert!(sent.iter().any(|c| c == "1=H"), "{sent:?}");
}
