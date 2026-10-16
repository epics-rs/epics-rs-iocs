//! Vendor-independent support for the motor-controller driver crates ported
//! from `epics-modules/motor`.
//!
//! The per-vendor crates (`motor-newport`, `motor-micronix`, …) all share the
//! same shape: a C-named create command connects an octet port, builds one
//! [`epics_rs::asyn::interfaces::motor::AsynMotor`] per axis, and registers
//! each where a record's OUT reaches it — `@asyn(PORT,ADDR)` for an asyn
//! driver, `#C card S signal` for a model-1 one — as motor-rs's
//! [`devices`] binds them at `init_record`. This crate factors out everything
//! that is not protocol-specific:
//!
//! - [`install`] — wires one axis through motor-rs's [`MotorBuilder`],
//!   registers it and starts its poller.
//! - [`model1::Model1`] — a model-1 driver's `Setup` and card table.
//! - [`iocsh`] — argument parsing helpers for the create commands.
//! - [`connect`] — octet-port connect helpers ([`connect_serial`],
//!   [`connect_ip`]).
//! - [`util`] — the C-runtime numeric helpers (`atof`/`atoi`/`NINT`/…) the
//!   protocols share.
//!
//! [`MotorBuilder`]: epics_rs::motor::builder::MotorBuilder
//! [`devices`]: epics_rs::motor::devices
//! [`connect_serial`]: connect::connect_serial
//! [`connect_ip`]: connect::connect_ip

pub mod connect;
pub mod iocsh;
pub mod model1;
pub mod util;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::server::iocsh::registry::CommandContext;
use epics_rs::motor::builder::{AxisSetup, MotorBuilder};
pub use epics_rs::motor::devices::{
    PortDriver, device_support_factory, devices, model1_device, register_model1_device,
    register_port,
};
pub use epics_rs::motor::trace::TraceUser;

/// The trace user of an axis a C asyn driver serves: its port and address.
pub fn asyn_user(port: &str, addr: i32) -> TraceUser {
    TraceUser::Asyn {
        port: port.into(),
        addr,
    }
}

/// Make `motor` the axis at `user` — [`asyn_user`] on a port the create
/// command registered with [`register_port`], or [`TraceUser::Model1`] for
/// a card of a model-1 driver — and start its poller, which polls from
/// here on as C's poller thread does from controller creation.
pub fn install(
    ctx: &CommandContext,
    user: TraceUser,
    motor: Arc<Mutex<dyn AsynMotor>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
) -> Result<(), String> {
    let AxisSetup {
        axis,
        poll_loop,
        poll_cmd_tx: _,
    } = MotorBuilder::new(motor)
        .trace_user(user)
        .moving_poll_interval(Duration::from_millis(moving_poll_ms))
        .idle_poll_interval(Duration::from_millis(idle_poll_ms))
        .build_axis();
    epics_rs::motor::devices::register_axis(axis)?;
    ctx.bridge().spawn(poll_loop.run());
    Ok(())
}
