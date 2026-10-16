//! Simulated motor IOC.
//!
//! Assembles: the simulated motor controller (`motorSimCreateController` /
//! `motorSimConfigAxis`), the motor record type, and the CA + PVA (QSRV)
//! server. No hardware — the axes integrate their own trajectories, so this is
//! a self-contained motor-record test/demo target.
//!
//! Usage:
//!   cargo run -p motorsim-ioc -- st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::ca::server::ioc_app::IocApplication;

use motor_motorsim::ioc::MotorSimHolder;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: motorsim-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MOTORSIM_IOC", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // Motor record type.
    let (motor_name, motor_factory) = epics_rs::motor::motor_record_factory();
    app = app.register_record_type(motor_name, motor_factory);

    // motorSimCreateController / motorSimConfigAxis commands, and the
    // asynMotor device support their axes serve. No octet port is needed —
    // the simulator has no hardware.
    let holder = MotorSimHolder::new();
    app = app.register_startup_command(holder.create_controller_command());
    app = app.register_startup_command(holder.config_axis_command());
    app = app.register_dynamic_device_support(
        &epics_rs::motor::devices::devices(),
        epics_rs::motor::devices::device_support_factory(),
    );

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
