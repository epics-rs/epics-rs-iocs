//! C `MCS2Controller` starts one `asynMotorPoller` for all its axes
//! (smarActMCS2MotorDriver.cpp:95).

#![cfg(target_os = "linux")]

use epics_rs::base::server::iocsh::registry::ArgValue;
use motor_common::testing::pollers_after;

#[epics_rs::base::epics_test]
async fn four_mcs2_axes_poll_on_one_poller() {
    let args = vec![
        ArgValue::String("MCS2_ONE".into()),
        ArgValue::String("MCS2_ONE_SERIAL".into()),
        ArgValue::Int(4),
        ArgValue::Int(100),
        ArgValue::Int(1000),
        ArgValue::Int(0),
    ];
    let cmd = motor_smaract::ioc::mcs2_create_controller_command();
    assert_eq!(pollers_after(cmd, args).await, 1);
}
