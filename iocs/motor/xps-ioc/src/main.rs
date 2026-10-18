//! Newport XPS motion controller IOC.
//!
//! Assembles: an XPS controller (`XPSCreateController`, which opens its TCP
//! sockets from an IP address and port) with its axes (`XPSCreateAxis`), the motor record
//! type, and the CA + PVA (QSRV) server.
//!
//! Usage:
//!   cargo run -p xps-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_newport::ioc::{
    tclcall_command, xps_aux_config_command, xps_build_profile_command, xps_create_axis_command,
    xps_create_controller_command, xps_create_profile_command, xps_define_profile_command,
    xps_disable_auto_enable_command, xps_enable_moving_mode_command, xps_execute_profile_command,
    xps_no_disable_error_command, xps_position_compare_command, xps_readback_profile_command,
    xps_tcl_script_execute_command,
};

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: xps-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("XPS_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // Standard asyn iocsh commands — provides `drvAsynIPPortConfigure`.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // XPS controller + per-axis create commands + motor device support.
    app = app.register_startup_command(xps_create_controller_command());
    app = app.register_startup_command(xps_create_axis_command());
    app = app.register_startup_command(epics_rs::motor::ioc::enable_move_to_home_command());
    app = app.register_startup_command(xps_create_profile_command());
    app = app.register_startup_command(xps_disable_auto_enable_command());
    app = app.register_startup_command(xps_no_disable_error_command());
    app = app.register_startup_command(xps_enable_moving_mode_command());
    app = app.register_startup_command(tclcall_command());
    app = app.register_startup_command(xps_aux_config_command());
    app = app.register_startup_command(xps_tcl_script_execute_command());
    app = app.register_startup_command(xps_position_compare_command());
    app = app.register_startup_command(xps_define_profile_command());
    app = app.register_startup_command(xps_build_profile_command());
    app = app.register_startup_command(xps_execute_profile_command());
    app = app.register_startup_command(xps_readback_profile_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
