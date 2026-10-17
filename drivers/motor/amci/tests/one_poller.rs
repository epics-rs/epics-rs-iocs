//! C `ANG1Controller` starts one `asynMotorPoller` for all its axes
//! (ANG1Driver.cpp:98).

#![cfg(target_os = "linux")]

use epics_rs::base::server::iocsh::registry::ArgValue;
use motor_common::testing::pollers_after;

#[epics_rs::base::epics_test]
async fn four_ang1_axes_poll_on_one_poller() {
    let args = vec![
        ArgValue::String("ANG1_ONE".into()),
        ArgValue::String("ANG1_ONE_IN".into()),
        ArgValue::String("ANG1_ONE_OUT".into()),
        ArgValue::Int(4),
        ArgValue::Int(100),
        ArgValue::Int(1000),
    ];
    let cmd = motor_amci::ioc::ang1_create_controller_command();
    assert_eq!(pollers_after(cmd, args).await, 1);
}
