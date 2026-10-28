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
pub mod drv_motor_asyn;
pub mod iocsh;
pub mod model1;
pub mod model2;
pub mod param_lib;
#[cfg(feature = "testing")]
pub mod testing;
pub mod util;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::{AsynMotor, PortSets};
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::base::server::driver_support::{DriverSupport, register_driver_support};
use epics_rs::base::server::iocsh::registry::CommandContext;
use epics_rs::motor::builder::{
    AxisSetup, ControllerAxes, ControllerAxis, ControllerBuilder, MotorBuilder,
};
pub use epics_rs::motor::controller_port::{ControllerParams, ControllerReport};
pub use epics_rs::motor::device_support::AxisReport;
use epics_rs::motor::device_support::MotorAxis;
pub use epics_rs::motor::devices::{
    device_support_factory, devices, model1_device, register_controller_port,
    register_drv_motor_asyn_port, register_model1_device,
};
use epics_rs::motor::poll_loop::PollCommand;
pub use epics_rs::motor::trace::TraceUser;

/// asynMotorController's `DEFAULT_CONTROLLER_TIMEOUT` (asynMotorController.h:16):
/// what `writeController`, `writeReadController` and `readController` pass.
pub const DEFAULT_CONTROLLER_TIMEOUT: Duration = Duration::from_secs(2);

/// Create C `asynMotorController` port `port` with axes `0..num_axes` for a
/// driver with no parameters of its own, gated on the IOC's interruptAccept.
pub fn register_port(ctx: &CommandContext, port: &str, num_axes: i32) -> Result<(), String> {
    register_port_with(ctx, port, num_axes, ())
}

/// [`register_port`] for a driver whose subclass adds to the base port, such
/// as its `report(fp, level)` override.
pub fn register_port_with(
    ctx: &CommandContext,
    port: &str,
    num_axes: i32,
    params: impl ControllerParams,
) -> Result<(), String> {
    let owner = ctx.db().scan_control().interrupt_gate();
    register_controller_port(owner, port, num_axes, params).map(drop)
}

/// A `.dbd` `driver(name)` line with its drvet `report`, which `dbior` runs
/// with its level and prints after `Driver: name`.
pub fn register_driver_report(
    name: &'static str,
    report: impl Fn(i32) -> String + Send + Sync + 'static,
) {
    struct Drvet<F>(F);
    impl<F: Fn(i32) -> String + Send + Sync + 'static> DriverSupport for Drvet<F> {
        fn report(&self, level: i32) -> Option<Vec<u8>> {
            Some((self.0)(level).into_bytes())
        }
    }
    register_driver_support(name, Arc::new(Drvet(report)));
}

/// The trace user of an axis a C asyn driver serves: its port and address.
pub fn asyn_user(port: &str, addr: i32) -> TraceUser {
    TraceUser::Asyn {
        port: port.into(),
        addr,
    }
}

/// What a [`Controller::install_built`] build returns: C's constructed
/// axis, and its `asynMotorAxis::report` override if it has one.
pub type AxisBuilt = Result<(Arc<Mutex<dyn AsynMotor>>, Option<Arc<dyn AxisReport>>), String>;

/// A C `asynMotorController`'s axes and its one poller thread
/// (asynMotorController.cpp:567-575, :615-695): each pass runs the
/// controller poll, then every axis in address order, so a slow axis delays
/// the rest as in C. An axis installed after [`Self::start`] joins the
/// running poller, as C's reads the axis list on every pass.
pub struct Controller {
    /// Until the poller starts.
    builder: Mutex<Option<ControllerBuilder>>,
    axes: ControllerAxes,
}

impl Default for Controller {
    fn default() -> Self {
        let builder = ControllerBuilder::new();
        Self {
            axes: builder.axes(),
            builder: Mutex::new(Some(builder)),
        }
    }
}

impl Controller {
    pub fn new() -> Self {
        Self::default()
    }

    /// The driver's `asynMotorController::poll` override; what it returns
    /// is what it set on its own port.
    pub fn controller_poll<R: Into<PortSets>>(
        self,
        poll: impl FnMut() -> R + Send + 'static,
    ) -> Self {
        let builder = self.builder.into_inner().unwrap_or_else(|e| e.into_inner());
        Self {
            builder: Mutex::new(builder.map(|b| b.controller_poll(poll))),
            axes: self.axes,
        }
    }

    /// Register `motor` as the axis at `user`; the sender reaches its
    /// poller, for a driver thread's [`PollCommand::Post`] or wakeup.
    pub fn install(
        &self,
        user: TraceUser,
        motor: Arc<Mutex<dyn AsynMotor>>,
    ) -> Result<mpsc::Sender<PollCommand>, String> {
        self.add(user, motor, None)
    }

    /// [`Self::install`] for an axis with C's `asynMotorAxis::report`
    /// override, which its controller's report runs.
    pub fn install_with_report(
        &self,
        user: TraceUser,
        motor: Arc<Mutex<dyn AsynMotor>>,
        report: Arc<dyn AxisReport>,
    ) -> Result<mpsc::Sender<PollCommand>, String> {
        self.add(user, motor, Some(report))
    }

    /// [`Self::install`] for a driver whose C constructs the axis under
    /// `pC->lock()` (MVP2001Driver.cpp:222-224): `build` runs under the
    /// controller lock, so a pass due meanwhile waits for it and then polls
    /// the axis, before iocInit as in C.
    pub fn install_built(
        &self,
        user: TraceUser,
        build: impl FnOnce() -> AxisBuilt,
    ) -> Result<mpsc::Sender<PollCommand>, String> {
        let (ControllerAxis { axis, poll_cmd_tx }, report) = self.axes.add_built(user, build)?;
        epics_rs::motor::devices::register_axis(with_report(axis, report))?;
        Ok(poll_cmd_tx)
    }

    /// [`Self::install_built`] for every `(user, build)` under one hold of
    /// the controller lock, as pmacCreateAxes constructs its axes
    /// (pmacController.cpp:974-979). A build's error ends the batch; the
    /// axes built before it stay installed.
    pub fn install_built_all<F>(
        &self,
        builds: impl IntoIterator<Item = (TraceUser, F)>,
    ) -> Result<Vec<mpsc::Sender<PollCommand>>, String>
    where
        F: FnOnce() -> AxisBuilt,
    {
        let (added, done) = self.axes.add_built_all(builds);
        let mut senders = Vec::new();
        for (ControllerAxis { axis, poll_cmd_tx }, report) in added {
            epics_rs::motor::devices::register_axis(with_report(axis, report))?;
            senders.push(poll_cmd_tx);
        }
        done.map(|()| senders)
    }

    fn add(
        &self,
        user: TraceUser,
        motor: Arc<Mutex<dyn AsynMotor>>,
        report: Option<Arc<dyn AxisReport>>,
    ) -> Result<mpsc::Sender<PollCommand>, String> {
        let ControllerAxis { axis, poll_cmd_tx } = self.axes.add(motor, user)?;
        epics_rs::motor::devices::register_axis(with_report(axis, report))?;
        Ok(poll_cmd_tx)
    }

    /// [`Self::install`]'s poller half: the axis polls, but no record reaches
    /// it until its [`MotorAxis`] is registered, which a model-2 port's axes
    /// wait for until the port exists.
    pub(crate) fn open(
        &self,
        user: TraceUser,
        motor: Arc<Mutex<dyn AsynMotor>>,
    ) -> Result<ControllerAxis, String> {
        self.axes.add(motor, user)
    }

    /// C `startMoveToHomeThread` (asynMotorController.cpp:703-710).
    pub fn start_move_to_home_thread(&self) -> std::io::Result<()> {
        self.axes.start_move_to_home_thread()
    }

    /// Count a first poll pending on this controller until the guard drops,
    /// for a driver whose own thread polls (omsBaseController's omsPoller).
    pub fn first_poll_pending(&self) -> epics_rs::motor::poll_loop::FirstPollPending {
        self.axes.first_poll_pending()
    }

    /// C `startPoller(movingPollPeriod, idlePollPeriod, forcedFastPolls)`
    /// on controller port `port`: the startup pass over the axes so far,
    /// then the poller thread, which setMovingPollPeriod and
    /// setIdlePollPeriod find by `port` whether or not an axis has joined.
    /// `forced_fast_polls` is the driver's own, C has no default
    /// (asynMotorController.cpp:567). A constructor that returns before it
    /// leaves its axes unpolled, as C's does. The poller starts once.
    pub fn start(
        &self,
        port: &str,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
        forced_fast_polls: u32,
    ) {
        self.spawn(Some(port), moving_poll_ms, idle_poll_ms, forced_fast_polls);
    }

    /// A model-2 driver's own thread, which no port names until
    /// drvAsynMotorConfigure opens its axes (drvMotorSim's motorSimThread,
    /// drvMotorSim.c:788-800).
    pub fn start_driver_thread(
        &self,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
        forced_fast_polls: u32,
    ) {
        self.spawn(None, moving_poll_ms, idle_poll_ms, forced_fast_polls);
    }

    /// C `setMovingPollPeriod` / `setIdlePollPeriod` on the running
    /// [`Self::start_driver_thread`] poller, which then runs a pass.
    pub fn set_poll_periods(&self, moving_poll_ms: Option<u64>, idle_poll_ms: Option<u64>) {
        self.axes.set_poll_periods(
            moving_poll_ms.map(Duration::from_millis),
            idle_poll_ms.map(Duration::from_millis),
        );
    }

    fn spawn(
        &self,
        port: Option<&str>,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
        forced_fast_polls: u32,
    ) {
        let builder = self
            .builder
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        let Some(mut builder) = builder else {
            return;
        };
        if let Some(port) = port {
            builder = builder.port(port);
        }
        let poller = builder.build(
            Duration::from_millis(moving_poll_ms),
            Duration::from_millis(idle_poll_ms),
            forced_fast_polls,
        );
        poller.start();
    }
}

/// Make `motor` the axis at [`asyn_user`] on a port the create command
/// registered with [`register_port`], and start its poller, which polls from
/// here on as C's poller thread does from controller creation. A model-1
/// card goes through [`model1::Model1::install`] instead.
/// `forced_fast_polls` is the driver's own: C's `startPoller` has no default
/// (asynMotorController.cpp:567).
pub fn install(
    user: TraceUser,
    motor: Arc<Mutex<dyn AsynMotor>>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
    forced_fast_polls: u32,
) -> Result<(), String> {
    install_axis(
        user,
        motor,
        None,
        moving_poll_ms,
        idle_poll_ms,
        forced_fast_polls,
    )
}

/// [`install`] for an axis with C's `asynMotorAxis::report` override.
pub fn install_with_report(
    user: TraceUser,
    motor: Arc<Mutex<dyn AsynMotor>>,
    report: Arc<dyn AxisReport>,
    moving_poll_ms: u64,
    idle_poll_ms: u64,
    forced_fast_polls: u32,
) -> Result<(), String> {
    install_axis(
        user,
        motor,
        Some(report),
        moving_poll_ms,
        idle_poll_ms,
        forced_fast_polls,
    )
}

fn install_axis(
    user: TraceUser,
    motor: Arc<Mutex<dyn AsynMotor>>,
    report: Option<Arc<dyn AxisReport>>,
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
    epics_rs::motor::devices::register_axis(with_report(axis, report))?;
    poll_loop.start();
    Ok(())
}

fn with_report(axis: MotorAxis, report: Option<Arc<dyn AxisReport>>) -> MotorAxis {
    match report {
        Some(report) => axis.with_report(report),
        None => axis,
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Instant;

    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::interfaces::motor::{MotorStatus, PollFailure};
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::base::runtime::task::BlockingBridge;
    use epics_rs::base::server::database::PvDatabase;
    use epics_rs::base::server::iocsh::registry::ArgValue;

    use super::*;
    use epics_rs::asyn::port::InterfaceMask;

    type Log = Arc<Mutex<Vec<i32>>>;

    struct Polled {
        addr: i32,
        log: Log,
    }

    impl AsynMotor for Polled {
        fn move_absolute(
            &mut self,
            _: &AsynUser,
            _: f64,
            _: f64,
            _: f64,
            _: f64,
        ) -> AsynResult<()> {
            Ok(())
        }
        fn move_relative(
            &mut self,
            _: &AsynUser,
            _: f64,
            _: f64,
            _: f64,
            _: f64,
        ) -> AsynResult<()> {
            Ok(())
        }
        fn home(&mut self, _: &AsynUser, _: f64, _: f64, _: f64, _: bool) -> AsynResult<()> {
            Ok(())
        }
        fn stop(&mut self, _: &AsynUser, _: f64) -> AsynResult<()> {
            Ok(())
        }
        fn set_position(&mut self, _: &AsynUser, _: f64) -> AsynResult<()> {
            Ok(())
        }
        fn poll_failure(&self) -> PollFailure {
            PollFailure::PROBLEM_AND_COMMS_ERROR
        }
        fn poll(&mut self, _: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
            self.log.lock().unwrap().push(self.addr);
            *status = MotorStatus::default();
            Ok(())
        }
    }

    /// SmarAct's constructor starts the poller and `*CreateAxis` adds the
    /// axes later; ANF2StartPoller starts it after iocInit.
    #[test]
    fn an_axis_installed_after_start_joins_the_poller() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let _entered = rt.enter();
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        let log = Log::default();
        let motor = |addr| -> Arc<Mutex<dyn AsynMotor>> {
            Arc::new(Mutex::new(Polled {
                addr,
                log: log.clone(),
            }))
        };
        register_port(&ctx, "joinAfterStart", 2).unwrap();
        // -1 marks each pass: its controller poll comes first.
        let pass = log.clone();
        let controller = Controller::new().controller_poll(move || pass.lock().unwrap().push(-1));
        let _a1 = controller
            .install(asyn_user("joinAfterStart", 1), motor(1))
            .unwrap();
        controller.start("joinAfterStart", 5, 5, 0);
        let _a0 = controller
            .install(asyn_user("joinAfterStart", 0), motor(0))
            .unwrap();
        // The startup pass polls A0 or not by when the add lands, as C's
        // may; every pass that starts after A0's first poll polls both.
        let deadline = Instant::now() + Duration::from_secs(5);
        while epics_rs::motor::devices::first_polls_pending("joinAfterStart") != Some(0) {
            assert!(Instant::now() < deadline, "a first poll never ran");
            std::thread::sleep(Duration::from_millis(1));
        }
        let joined = log.lock().unwrap().len();
        let passes_after = || {
            let log = log.lock().unwrap();
            let start = log[joined..]
                .iter()
                .position(|&a| a == -1)
                .map(|p| joined + p);
            start.map_or_else(Vec::new, |start| log[start..].to_vec())
        };
        let deadline = Instant::now() + Duration::from_secs(5);
        while passes_after().len() < 6 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        let passes = passes_after();
        assert_eq!(passes[..6], [-1, 0, 1, -1, 0, 1], "{passes:?}");
    }

    /// C setIdlePollPeriod finds the controller by its port, so a poller
    /// started before any axis joins (SmarAct's, phytron's) takes it.
    #[test]
    fn the_poll_periods_reach_a_controller_with_no_axes_yet() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let _entered = rt.enter();
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        register_port(&ctx, "periodNoAxes", 1).unwrap();
        let passes = Arc::new(AtomicUsize::new(0));
        let counted = passes.clone();
        let controller = Controller::new().controller_poll(move || {
            counted.fetch_add(1, Ordering::Relaxed);
            PortSets::default()
        });
        controller.start("periodNoAxes", 3_600_000, 3_600_000, 0);
        std::thread::sleep(Duration::from_millis(100));
        for set in [
            epics_rs::motor::ioc::set_moving_poll_period_command(),
            epics_rs::motor::ioc::set_idle_poll_period_command(),
        ] {
            let args = [
                ArgValue::String("periodNoAxes".into()),
                ArgValue::Double(0.01),
            ];
            set.handler.call(&args, &ctx).unwrap();
        }
        let before = passes.load(Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(300));
        let passes = passes.load(Ordering::Relaxed) - before;
        assert!(passes >= 10, "{passes} passes");
    }

    struct Reporting;

    impl ControllerParams for Reporting {
        const INTERFACE_MASK: InterfaceMask = InterfaceMask::NONE;
        const INTERRUPT_MASK: InterfaceMask = InterfaceMask::NONE;

        fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
            let _ = writeln!(out, "driver {} numAxes={}", ctx.port_name(), ctx.num_axes());
            ctx.report(out, level);
        }
    }

    struct AxisLine(i32);

    impl AxisReport for AxisLine {
        fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
            if level > 0 {
                let _ = writeln!(out, "  axis {}", self.0);
            }
        }
    }

    /// C `asynReport`: the controller's `report` override, then each axis's.
    #[test]
    fn the_port_report_reaches_the_controller_and_axis_overrides() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let _entered = rt.enter();
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), BlockingBridge::capture());
        register_port_with(&ctx, "reportHooks", 2, Reporting).unwrap();
        let controller = Controller::new();
        for addr in 0..2 {
            let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(Polled {
                addr,
                log: Log::default(),
            }));
            controller
                .install_with_report(
                    asyn_user("reportHooks", addr),
                    motor,
                    Arc::new(AxisLine(addr)),
                )
                .unwrap();
        }
        let port = epics_rs::asyn::manager::PortManager::global()
            .find_port_handle("reportHooks")
            .unwrap();
        let text = port.report_blocking(1).unwrap();
        let want = "driver reportHooks numAxes=2\n  axis 0\n  axis 1\n";
        assert!(text.contains(want), "{text}");
    }
}

#[cfg(all(test, feature = "testing", target_os = "linux"))]
mod poller_count {
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::interfaces::motor::{MotorStatus, PollFailure};
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::base::server::iocsh::registry::{ArgValue, CommandDef, CommandOutcome};

    use super::*;
    use crate::testing::pollers_after;

    struct Idle;

    impl AsynMotor for Idle {
        fn move_absolute(
            &mut self,
            _: &AsynUser,
            _: f64,
            _: f64,
            _: f64,
            _: f64,
        ) -> AsynResult<()> {
            Ok(())
        }
        fn move_relative(
            &mut self,
            _: &AsynUser,
            _: f64,
            _: f64,
            _: f64,
            _: f64,
        ) -> AsynResult<()> {
            Ok(())
        }
        fn home(&mut self, _: &AsynUser, _: f64, _: f64, _: f64, _: bool) -> AsynResult<()> {
            Ok(())
        }
        fn stop(&mut self, _: &AsynUser, _: f64) -> AsynResult<()> {
            Ok(())
        }
        fn set_position(&mut self, _: &AsynUser, _: f64) -> AsynResult<()> {
            Ok(())
        }
        fn poll_failure(&self) -> PollFailure {
            PollFailure::PROBLEM_AND_COMMS_ERROR
        }
        fn poll(&mut self, _: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
            *status = MotorStatus::default();
            Ok(())
        }
    }

    #[epics_rs::base::epics_test]
    async fn a_controller_s_axes_share_one_motor_poller() {
        let cmd = CommandDef::new(
            "countPollers",
            vec![],
            "countPollers",
            |_: &[ArgValue], ctx: &CommandContext| {
                register_port(ctx, "countPollers", 3)?;
                let controller = Controller::new();
                for addr in 0..3 {
                    let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(Idle));
                    controller.install(asyn_user("countPollers", addr), motor)?;
                }
                controller.start("countPollers", 10, 10, 0);
                Ok(CommandOutcome::Continue)
            },
        );
        assert_eq!(pollers_after(cmd, vec![]).await, 1);
    }
}
