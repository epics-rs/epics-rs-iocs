//! Vendor-independent support for the motor-controller driver crates ported
//! from `epics-modules/motor`.
//!
//! The per-vendor crates (`motor-newport`, `motor-micronix`, …) all share the
//! same shape: a C-named create command connects an octet port, builds one
//! [`epics_rs::asyn::interfaces::motor::AsynMotor`] per axis, and registers
//! each where a record's OUT reaches it — `@asyn(PORT,ADDR)` for an asyn
//! or model-2 driver, `#C card S signal` for a model-1 one — as motor-rs's
//! [`devices`] binds them at `init_record`. This crate factors out everything
//! that is not protocol-specific:
//!
//! - [`Controller`] — an asyn controller's axes and the one poller that
//!   polls them in turn; [`install`] is a controller of one axis.
//! - [`model1::Model1`] — a model-1 driver's `Setup` and card table.
//! - [`model2::Model2`] — a model-2 driver's cards, opened by
//!   `drvAsynMotorConfigure`.
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
pub mod model2;
#[cfg(feature = "testing")]
pub mod testing;
pub mod util;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::base::server::iocsh::registry::CommandContext;
use epics_rs::motor::builder::{AxisSetup, ControllerAxis, ControllerBuilder, MotorBuilder};
pub use epics_rs::motor::devices::{
    device_support_factory, devices, model1_device, register_controller_port,
    register_drv_motor_asyn_port, register_model1_device,
};
use epics_rs::motor::poll_loop::PollCommand;
pub use epics_rs::motor::trace::TraceUser;

/// Create C `asynMotorController` port `port` with axes `0..num_axes` for a
/// driver with no parameters of its own, gated on the IOC's interruptAccept.
pub fn register_port(ctx: &CommandContext, port: &str, num_axes: i32) -> Result<(), String> {
    let owner = ctx.db().scan_control().interrupt_gate();
    register_controller_port(owner, port, num_axes, ()).map(drop)
}

/// The trace user of an axis a C asyn driver serves: its port and address.
pub fn asyn_user(port: &str, addr: i32) -> TraceUser {
    TraceUser::Asyn {
        port: port.into(),
        addr,
    }
}

/// A C `asynMotorController`'s axes and its one poller thread
/// (asynMotorController.cpp:567-575, :615-695): each pass runs the
/// controller poll, then every axis in the order [`Self::install`] added
/// them, so a slow axis delays the rest as in C.
#[derive(Default)]
pub struct Controller {
    builder: ControllerBuilder,
}

impl Controller {
    pub fn new() -> Self {
        Self::default()
    }

    /// The driver's `asynMotorController::poll` override.
    pub fn controller_poll(mut self, poll: impl FnMut() + Send + 'static) -> Self {
        self.builder = self.builder.controller_poll(poll);
        self
    }

    /// Register `motor` as the next axis, under `user`; the sender reaches
    /// its poller, for a driver thread's [`PollCommand::Post`] or wakeup.
    pub fn install(
        &mut self,
        user: TraceUser,
        motor: Arc<Mutex<dyn AsynMotor>>,
    ) -> Result<mpsc::Sender<PollCommand>, String> {
        let ControllerAxis { axis, poll_cmd_tx } = self.builder.axis(motor, user);
        epics_rs::motor::devices::register_axis(axis)?;
        Ok(poll_cmd_tx)
    }

    /// C `startPoller(movingPollPeriod, idlePollPeriod, forcedFastPolls)`:
    /// the startup pass over the axes, then the poller thread.
    /// `forced_fast_polls` is the driver's own, C has no default
    /// (asynMotorController.cpp:567). A constructor that returns before it
    /// leaves its axes unpolled, as C's does.
    pub fn start(
        self,
        ctx: &CommandContext,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
        forced_fast_polls: u32,
    ) {
        let poller = self.builder.build(
            Duration::from_millis(moving_poll_ms),
            Duration::from_millis(idle_poll_ms),
            forced_fast_polls,
        );
        ctx.bridge().spawn(poller.run());
    }
}

/// Make `motor` the axis at [`asyn_user`] on a port the create command
/// registered with [`register_port`], and start its poller, which polls from
/// here on as C's poller thread does from controller creation. A model-1
/// card goes through [`model1::Model1::install`] instead.
/// `forced_fast_polls` is the driver's own: C's `startPoller` has no default
/// (asynMotorController.cpp:567).
pub fn install(
    ctx: &CommandContext,
    user: TraceUser,
    motor: Arc<Mutex<dyn AsynMotor>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
    forced_fast_polls: u32,
) -> Result<(), String> {
    let AxisSetup {
        axis,
        poll_loop,
        poll_cmd_tx: _,
    } = MotorBuilder::new(motor)
        .trace_user(user)
        .moving_poll_interval(Duration::from_millis(moving_poll_ms))
        .idle_poll_interval(Duration::from_millis(idle_poll_ms))
        .forced_fast_polls(forced_fast_polls)
        .build_axis();
    epics_rs::motor::devices::register_axis(axis)?;
    ctx.bridge().spawn(poll_loop.run());
    Ok(())
}
