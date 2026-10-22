//! Micos SMC Taurus controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorMicos/micosApp/src/SMCTaurusAxis.cpp` +
//! `SMCTaurusDriver.cpp` (a model-3 `asynMotorController`/`asynMotorAxis` pair).
//! Connects over a `drvAsynIPPort` or `drvAsynSerialPort`; commands are ASCII,
//! CR/LF-terminated (the startup script sets both EOS, as C needs; the driver
//! sends bare commands).
//!
//! The Taurus command set is the per-axis SMC dialect shared with the hydra
//! sibling (`<value> <axis> <verb>` setters, `<axis> <verb>` queries). Compared
//! to the hydra it is simpler: no motor-form probe (the resolution is always
//! `clPeriod`), `stop` is a bare `nabort`, and `set_closed_loop(true)` sends the
//! closed-loop flag directly as the regulator mode. It additionally supports
//! pushing the record soft limits to the controller (`setnlimit`).
//!
//! ## Units
//!
//! As C, the record's steps are scaled by `axisRes = clPeriod` (read with
//! `getclperiod`) into the controller's units and divided back on poll;
//! `SMCTaurusChangeResolution` overrides it.
//!
//! ## Deviations (documented)
//!
//! - The C poll computes an e-stop-switch problem flag from status bit `0x200`
//!   but then unconditionally overwrites the problem flag to `0` a few lines
//!   later (a latent bug that discards it). This port honours the intent and
//!   reports bit `0x200` as a problem.
//! - C `setHighLimit`/`setLowLimit` store the record's raw-step limit unscaled
//!   into the travel-limit cache that `getnlimit` filled in controller units,
//!   and send it as such through `setnlimit` and the jog `nm`. This port scales
//!   the record limit by `axisRes`, so the cache stays in controller units.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, StatusWrite,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use epics_libcom_rs::cfmt::CFloat;

use motor_common::util::{atof, atoi};

use crate::smc::{SmcMembers, SmcState, trace_axis_status, trace_move_velocity, travel_limits};

/// Response buffer size.
const READ_BUF: usize = 128;

/// `nst` status bits.
const STATUS_MOVING: i32 = 0x1; // axis in motion
const STATUS_POWER_OFF: i32 = 0x100; // drive power disabled
const STATUS_ESTOP_SWITCH: i32 = 0x200; // e-stop switch active

/// Delay required by the controller after an `init` command.
const INIT_DELAY: Duration = Duration::from_millis(200);

/// Shared Taurus controller endpoint owning the asyn octet handle.
pub struct TaurusController {
    handle: SyncIOHandle,
}

impl TaurusController {
    /// Wrap a connected octet handle.
    pub fn new(handle: SyncIOHandle) -> Self {
        Self { handle }
    }

    /// Write a command with no reply expected.
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// C's run of `status = writeController()`: every command is sent,
    /// whatever the one before returned, and the last one's status is the
    /// caller's.
    fn write_all<const N: usize>(&self, cmds: [String; N]) -> AsynResult<()> {
        let mut status = Ok(());
        for cmd in &cmds {
            status = self.write_only(cmd);
        }
        status
    }

    /// C `SMCTaurusAxis::setLimitSwitches` for 1-based `addr`: both
    /// switches take `value`; C ignores both writes' status.
    pub fn set_limit_switches(&self, addr: usize, value: i32) {
        for switch in 0..2 {
            let _ = self.write_only(&format!("{value} {switch} {addr} setsw"));
        }
    }

    /// C `SMCTaurusAxis::getLimitSwitches` for 1-based `addr`: switch
    /// `index`'s type from `getsw`.
    pub fn limit_switches(&self, addr: usize, index: usize) -> AsynResult<i32> {
        // C parses inString_ whatever writeReadController returned
        // (SMCTaurusAxis.cpp:186-187).
        let (reply, _) = self.exchange(&format!("{addr} getsw"));
        let types: Vec<i32> = reply
            .split_whitespace()
            .map_while(|tok| tok.parse::<i32>().ok())
            .take(2)
            .collect();
        if types.len() != 2 {
            println!("Could not parse getsw response.");
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("could not parse getsw response {reply:?}"),
            });
        }
        Ok(types[index])
    }

    /// Write a query and read one reply line.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let (reply, status) = self.exchange(cmd);
        status.map(|()| reply)
    }

    /// C `writeReadController`: `inString_` beside the status. The EOS layer
    /// NUL-terminates the buffer on every read, so a failed read leaves the
    /// bytes it delivered, which the callers that ignore the status parse.
    fn exchange(&self, cmd: &str) -> (String, AsynResult<()>) {
        let (raw, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        let reply = String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string();
        (reply, status)
    }
}

/// One Taurus axis sharing a controller. Implements [`AsynMotor`].
pub struct TaurusAxis {
    controller: Arc<Mutex<TaurusController>>,
    /// The travel limits are cached in controller units, as `setnlimit`
    /// takes them as a pair; `axisRes` starts as `clPeriod`.
    members: Arc<SmcMembers>,
    /// What the constructor set on the axis's status.
    status_writes: Vec<StatusWrite>,
    /// C's `pasynUser_`, which the axis's lines print on.
    trace: MotorTrace,
}

impl TaurusAxis {
    /// Construct axis `axis_no` (0-based) and read its `axisRes` and travel
    /// limits. C ignores both queries' status and `sscanf`s whatever is in the
    /// reply (SMCTaurusAxis.cpp:14-27); a value the reply does not carry
    /// stays 0.
    pub fn new(controller: Arc<Mutex<TaurusController>>, axis_no: usize, port: &str) -> Self {
        let addr = axis_no + 1;
        let (cl_period, reply) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let cl_period = atof(&ctrl.exchange(&format!("{addr} getclperiod")).0);
            (cl_period, ctrl.exchange(&format!("{addr} getnlimit")).0)
        };
        let state = SmcState {
            axis_res: cl_period,
            travel_limits: travel_limits(&reply),
            ..SmcState::default()
        };
        Self {
            controller,
            members: Arc::new(SmcMembers {
                axis_no,
                motor_form: None,
                pitch: None,
                pole_pairs: None,
                cl_period,
                state: Mutex::new(state),
            }),
            trace: MotorTrace::new(port, axis_no as i32),
            // SMCTaurusAxis.cpp:23.
            status_writes: vec![StatusWrite::GainSupport(true)],
        }
    }

    /// What C's `report` prints of the axis.
    pub fn members(&self) -> Arc<SmcMembers> {
        self.members.clone()
    }

    /// C `changeResolution`: replace `axisRes`.
    pub fn change_resolution(&mut self, new_resolution: f64) {
        self.members.state().axis_res = new_resolution;
    }

    fn addr(&self) -> usize {
        self.members.axis_no + 1
    }

    fn axis_res(&self) -> f64 {
        self.members.axis_res()
    }

    fn lock(&self) -> MutexGuard<'_, TaurusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `sendAccelAndVelocity`'s `snv`/`sna`, then `cmd`, as one
    /// [`TaurusController::write_all`].
    fn write_after_accel_velocity(
        &self,
        ctrl: &TaurusController,
        velocity: f64,
        acceleration: f64,
        cmd: String,
    ) -> AsynResult<()> {
        let addr = self.addr();
        ctrl.write_all([
            format!("{} {addr} snv", CFloat((velocity * self.axis_res()).abs())),
            format!(
                "{} {addr} sna",
                CFloat((acceleration * self.axis_res()).abs())
            ),
            cmd,
        ])
    }

    /// Push the current travel-limit pair to the controller (`setnlimit`).
    fn send_travel_limits(&self, ctrl: &TaurusController) -> AsynResult<()> {
        let (neg, pos) = self.members.state().travel_limits;
        ctrl.write_only(&format!(
            "{} {} {} setnlimit",
            CFloat(neg),
            CFloat(pos),
            self.addr()
        ))
    }
}

impl AsynMotor for TaurusAxis {
    /// C `SMCTaurusAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (SMCTaurusAxis.cpp).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let cmd = format!("{} {} nm", CFloat(position * self.axis_res()), self.addr());
        self.write_after_accel_velocity(&self.lock(), velocity, acceleration, cmd)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let cmd = format!("{} {} nr", CFloat(distance * self.axis_res()), self.addr());
        self.write_after_accel_velocity(&self.lock(), velocity, acceleration, cmd)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        trace_move_velocity(
            &self.trace,
            "SMCTaurusAxis::moveVelocity",
            min_velocity,
            velocity,
            acceleration,
        );
        // No jog command — move to the cached axis travel limit.
        let (neg, pos) = self.members.state().travel_limits;
        let target = if velocity > 0.0 { pos } else { neg };
        let cmd = format!("{} {} nm", CFloat(target), self.addr());
        self.write_after_accel_velocity(&self.lock(), velocity, acceleration, cmd)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let verb = if forward { "nrm" } else { "ncal" };
        let cmd = format!("{} {verb}", self.addr());
        self.write_after_accel_velocity(&self.lock(), velocity, acceleration, cmd)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!("{} nabort", self.addr()))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // setnpos takes the distance from the current position to the desired
        // origin, hence the negation (C multiplies by -1).
        ctrl.write_only(&format!(
            "{} {} setnpos",
            CFloat(-position * self.axis_res()),
            self.addr()
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        let addr = self.addr();
        if enable {
            // C sends the closed-loop flag itself (1) as the regulator mode,
            // then reinits so it takes effect (powers the motor); the
            // controller needs a delay after init.
            let status = ctrl.write_all([format!("1 {addr} setcloop"), format!("{addr} init")]);
            std::thread::sleep(INIT_DELAY);
            status
        } else {
            ctrl.write_only(&format!("{addr} motoroff"))
        }
    }

    // C stores and sends the raw-step limit unscaled into the cache
    // getnlimit filled in controller units (SMCTaurusAxis.cpp:157-171); not
    // copied.
    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let pos = position * self.axis_res();
        self.members.state().travel_limits.1 = pos;
        let ctrl = self.lock();
        self.send_travel_limits(&ctrl)
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let neg = position * self.axis_res();
        self.members.state().travel_limits.0 = neg;
        let ctrl = self.lock();
        self.send_travel_limits(&ctrl)
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID-gain support in the C Taurus driver.
        Ok(())
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }

    /// C `SMCTaurusAxis::poll`: each field is set as C sets its parameter,
    /// and a failed exchange skips the rest (SMCTaurusAxis.cpp:215-343).
    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let ctrl = self.lock();
        let addr = self.addr();

        let position = atof(&ctrl.query(&format!("{addr} np"))?) / self.axis_res();
        status.position = position;
        status.encoder_position = position;

        let axis_status = atoi(&ctrl.query(&format!("{addr} nst"))?);
        let moving = (axis_status & STATUS_MOVING) != 0;
        status.done = !moving;
        status.moving = moving;
        // SMCTaurusAxis.cpp:263 motorStatusMoving_ = !done.
        status.msta_moving = moving;

        // SMCTaurusAxis.cpp:267-271 reads the commanded velocity and
        // acceleration, ignoring the replies and their status.
        let _ = ctrl.query(&format!("{addr} gnv"));
        let _ = ctrl.query(&format!("{addr} gna"));
        trace_axis_status(
            &self.trace,
            "SMCTaurusAxis::poll",
            self.members.axis_no,
            axis_status,
        );
        // C sets problem from 0x200 here, ahead of the getsw and getswst
        // skips (SMCTaurusAxis.cpp:287-294); its overwrite with 0 at :337
        // makes that a dead store on a good poll, not copied.
        status.problem = (axis_status & STATUS_ESTOP_SWITCH) != 0;

        // Switch config: bit 1 (0x2) masks (disables) the switch.
        let sw = ctrl.query(&format!("{addr} getsw"))?;
        let (ignore_low, ignore_high) = self.members.read_limit_config(&sw);

        // Switch state: "low high", 0=inactive 1=active.
        let st = ctrl.query(&format!("{addr} getswst"))?;
        let state: Vec<i32> = st
            .split_whitespace()
            .filter_map(|tok| tok.parse::<i32>().ok())
            .collect();
        let (mut low_limit, mut high_limit) = (false, false);
        if let [low, high, ..] = state.as_slice() {
            low_limit = !ignore_low && *low != 0;
            high_limit = !ignore_high && *high != 0;
        }
        status.low_limit = low_limit;
        status.high_limit = high_limit;

        status.powered = (axis_status & STATUS_POWER_OFF) == 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use motor_common::testing::Polled;

    /// SMCTaurusAxis.cpp:14-28 and :186-188 parse `inString_` whatever
    /// `writeReadController` returned, so replies cut by a timeout still set
    /// `axisRes`, the travel limits and the limit-switch types.
    #[test]
    fn replies_cut_by_a_timeout_are_still_parsed() {
        use motor_common::testing::LineRig;
        let (_line, handle) = LineRig::cut(
            "taurusCutReplies",
            |cmd| match cmd.trim_end() {
                "1 getclperiod" => "0.5".into(),
                "1 getnlimit" => "-10 20".into(),
                "1 getsw" => "2 3".into(),
                _ => String::new(),
            },
            |_| true,
        );
        let controller = Arc::new(Mutex::new(TaurusController::new(handle)));
        let axis = TaurusAxis::new(controller.clone(), 0, "");
        let state = axis.members.state();
        assert_eq!((state.axis_res, state.travel_limits), (0.5, (-10.0, 20.0)));
        let ctrl = controller.lock().unwrap();
        assert_eq!(ctrl.limit_switches(1, 1).unwrap(), 3);
    }

    /// C's poll prints the `nst` limit, e-stop and busy bits, and
    /// `moveVelocity` its FLOW line, on the axis's `pasynUser_`.
    #[test]
    fn the_status_bits_and_a_velocity_move_are_traced() {
        use epics_rs::asyn::trace::TraceMask;
        use motor_common::testing::LineRig;
        let (_line, handle) = LineRig::new("taurusTraceLine", |cmd| match cmd.trim_end() {
            "1 getmotor" => "1".into(),
            "1 getclperiod" => "0.5".into(),
            "1 nst" => "1728".into(),
            _ => String::new(),
        });
        let controller = Arc::new(Mutex::new(TaurusController::new(handle)));
        let mut axis = TaurusAxis::new(controller, 0, "taurusTrace");
        let mask = TraceMask::ERROR | TraceMask::IO_DRIVER | TraceMask::FLOW;
        let lines = crate::smc::trace_lines("taurusTrace", mask, || {
            let user = AsynUser::default();
            axis.polled(&user).unwrap();
            axis.move_velocity(&user, 1.0, 2.0, 4.0).unwrap();
        });
        assert_eq!(
            lines,
            [
                "SMCTaurusAxis::poll: axis 1 limit indicator active.",
                "SMCTaurusAxis::poll: axis 1 emergency stopped.",
                "SMCTaurusAxis::poll: axis 1 emergency stop switch active.",
                "SMCTaurusAxis::poll: axis 1 device is busy - move commands discarded.",
                "SMCTaurusAxis::moveVelocity: baseVelocity=1.000000, slewVelocity=2.000000, \
                 acceleration=4.000000",
            ]
        );
    }

    /// A failed `nst` skips the rest of C's poll, but the position it read
    /// first is still posted, with Problem (SMCTaurusAxis.cpp:243-256).
    #[test]
    fn a_failed_status_read_still_posts_the_position() {
        use motor_common::testing::LineRig;
        let (_line, handle) = LineRig::failing(
            "taurusNstFails",
            |cmd| match cmd.trim_end() {
                "1 getclperiod" => "0.5".into(),
                "1 np" => "2.5".into(),
                _ => String::new(),
            },
            |cmd| cmd.trim_end() == "1 nst",
        );
        let controller = Arc::new(Mutex::new(TaurusController::new(handle)));
        let mut axis = TaurusAxis::new(controller, 0, "");
        let res = axis.axis_res();
        let mut status = axis.constructed();
        assert!(axis.poll(&AsynUser::default(), &mut status).is_err());
        assert_eq!(status.position, 2.5 / res);
        assert_eq!(status.encoder_position, 2.5 / res);
        assert!(status.gain_support);
    }
    /// SMCTaurusAxis.cpp:216-343 sets only what it read before a failed
    /// read's `skip`, over the params as they stood, the Done=0 a move start
    /// set included (asynMotorController.cpp:301).
    #[test]
    fn a_failed_read_leaves_the_done_a_move_start_set() {
        use motor_common::testing::LineRig;
        let refused = Arc::new(Mutex::new(String::new()));
        let pick = refused.clone();
        let (_line, handle) = LineRig::failing(
            "taurusMoveStartDone",
            |cmd| match cmd.trim_end() {
                "1 getclperiod" => "0.5".into(),
                "1 np" => "2.5".into(),
                "1 nst" => "0".into(),
                "1 getsw" | "1 getswst" => "0 0".into(),
                _ => String::new(),
            },
            move |cmd| *pick.lock().unwrap() == cmd.trim_end(),
        );
        let controller = Arc::new(Mutex::new(TaurusController::new(handle)));
        let mut axis = TaurusAxis::new(controller, 0, "");
        let user = AsynUser::default();
        // The PROBLEM the poller raised over an earlier failure; `skip` clears it.
        let mut status = MotorStatus {
            problem: true,
            ..MotorStatus::default()
        };
        axis.poll(&user, &mut status).unwrap();
        assert!(status.done && !status.problem, "{status:?}");
        status.done = false;
        *refused.lock().unwrap() = "1 np".into();
        assert!(axis.poll(&user, &mut status).is_err());
        assert!(!status.done, "{status:?}");
    }

    /// C sets Problem from the e-stop switch bit before the `getsw` and
    /// `getswst` skips (SMCTaurusAxis.cpp:287-294), so a failure there clears
    /// the Problem an earlier failure left and `skip` sets it again: a
    /// change the poller calls back on every failing poll.
    #[test]
    fn a_failed_switch_read_clears_the_problem_before_its_skip() {
        use motor_common::testing::LineRig;
        for (port, refused) in [
            ("taurusGetswFails", "1 getsw"),
            ("taurusGetswstFails", "1 getswst"),
        ] {
            let (_line, handle) = LineRig::failing(
                port,
                |cmd| match cmd.trim_end() {
                    "1 getclperiod" => "0.5".into(),
                    "1 np" => "2.5".into(),
                    "1 nst" => "0".into(),
                    "1 getsw" | "1 getswst" => "0 0".into(),
                    _ => String::new(),
                },
                move |cmd| cmd.trim_end() == refused,
            );
            let controller = Arc::new(Mutex::new(TaurusController::new(handle)));
            let mut axis = TaurusAxis::new(controller, 0, "");
            let mut status = MotorStatus {
                problem: true,
                ..MotorStatus::default()
            };
            assert!(axis.poll(&AsynUser::default(), &mut status).is_err());
            assert!(!status.problem, "{refused}: {status:?}");
        }
    }

    #[test]
    fn an_axis_of_an_unanswering_controller_is_still_built() {
        let (handle, _) =
            SyncIOHandle::connect_port("TAURUS_NONE", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(TaurusController::new(handle)));
        let axis = TaurusAxis::new(controller, 0, "");
        assert_eq!(axis.members.state().travel_limits, (0.0, 0.0));
    }

    /// C's status_ starts at 0 plus GainSupport (SMCTaurusAxis.cpp:23); only
    /// a good poll sets Done (:262).
    #[test]
    fn an_unanswering_controller_s_axis_fails_its_poll_without_done() {
        let (handle, _) =
            SyncIOHandle::connect_port("TAURUS_DEAD", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(TaurusController::new(handle)));
        let mut axis = TaurusAxis::new(controller, 0, "");
        let mut status = axis.constructed();
        assert!(axis.poll(&AsynUser::default(), &mut status).is_err());
        assert!(status.gain_support && !status.done, "{status:?}");
    }

    /// Refuses every setting write (`snv`, `sna`, `setcloop`); records
    /// every write.
    struct SettingsRefused {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
    }

    const REFUSED: [&str; 3] = ["snv", "sna", "setcloop"];

    impl epics_rs::asyn::port::PortDriver for SettingsRefused {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let line = String::from_utf8_lossy(data).trim_end().to_string();
            let refused = REFUSED.iter().any(|verb| line.ends_with(verb));
            self.written.lock().unwrap().push(line);
            if refused {
                Err(epics_rs::asyn::error::AsynError::Status {
                    status: epics_rs::asyn::error::AsynStatus::Error,
                    message: "refused".into(),
                })
            } else {
                Ok(data.len())
            }
        }
    }

    /// C overwrites each write's status with the next one's
    /// (SMCTaurusAxis.cpp:52-111, :134-154), so a refused setting still
    /// sends the command after it, and that command's status is returned.
    #[test]
    fn a_refused_setting_still_sends_the_command_after_it() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = SettingsRefused {
            base: PortDriverBase::new(
                "taurusRefused",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: written.clone(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("taurusRefused").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let mut axis = TaurusAxis {
            controller: Arc::new(Mutex::new(TaurusController::new(handle))),
            members: Arc::new(SmcMembers {
                axis_no: 0,
                motor_form: None,
                pitch: None,
                pole_pairs: None,
                cl_period: 0.0,
                state: Mutex::new(SmcState {
                    axis_res: 0.5,
                    travel_limits: (-100.0, 100.0),
                    ..SmcState::default()
                }),
            }),
            status_writes: Vec::new(),
            trace: MotorTrace::default(),
        };
        let user = AsynUser::default();
        axis.move_absolute(&user, 2.0, 0.0, 2.0, 4.0).unwrap();
        axis.move_relative(&user, -2.0, 0.0, 2.0, 4.0).unwrap();
        axis.move_velocity(&user, 0.0, 2.0, 4.0).unwrap();
        axis.home(&user, 0.0, 2.0, 4.0, false).unwrap();
        axis.set_closed_loop(&user, true).unwrap();
        let written = written.lock().unwrap();
        let tails: Vec<&String> = written
            .iter()
            .filter(|w| !REFUSED.iter().any(|v| w.ends_with(v)))
            .collect();
        assert_eq!(
            tails,
            [
                "1.000000 1 nm",
                "-1.000000 1 nr",
                "100.000000 1 nm",
                "1 ncal",
                "1 init"
            ]
        );
        // Four moves of three writes, then closed-loop's two.
        assert_eq!(written.len(), 4 * 3 + 2);
    }

    #[test]
    fn status_bits() {
        let moving = |s: i32| (s & STATUS_MOVING) != 0;
        let powered = |s: i32| (s & STATUS_POWER_OFF) == 0;
        let problem = |s: i32| (s & STATUS_ESTOP_SWITCH) != 0;
        assert!(moving(0x1));
        assert!(!powered(0x100));
        assert!(powered(0x1));
        assert!(problem(0x200));
        assert!(!problem(0x1));
    }
}
