//! Micos SMC corvus controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorMicos/micosApp/src/SMCcorvusDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` pair). Connects over a `drvAsynIPPort`
//! or `drvAsynSerialPort`; commands are ASCII, CR/LF-terminated (the startup
//! script sets both EOS, as the C example does; the driver sends bare commands).
//!
//! ## Shared controller
//!
//! The corvus exposes only whole-controller operations: `pos` returns every
//! axis' position on one line, `move`/`rmove` take a target for every axis at
//! once, `st` is a global moving status, and `abort`/`sv`/`sa` act on all axes.
//! A single-axis command is therefore a read-modify-write of the full target
//! vector: read all positions, replace the entry for this axis, and send the
//! whole line. All axes share one [`CorvusController`] behind a mutex so that
//! read-modify-write is atomic.
//!
//! ## Units
//!
//! The corvus works in physical units; as C, the record's steps are scaled by
//! `axisRes = pitch / (4 · polePairs)` on the way out and divided back on
//! poll. `SMCcorvusChangeResolution` overrides `axisRes` at run time.
//!
//! ## Deviations (documented)
//!
//! - Relative moves send `0` for the non-moving axes. C defect, not copied: C
//!   (SMCcorvusDriver.cpp:253-272) fills those `rmove` slots with each idle
//!   axis' absolute position from `pos`, so a relative move of one axis moves
//!   the others by their own position; this port neither sends that `pos`
//!   query nor moves the idle axes.
//! - Limit and drive-power status are read only when the axis is done (matching
//!   C, which avoids polling switches mid-move to keep the interpreter
//!   responsive); while moving they are reported as not-triggered / powered.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use epics_libcom_rs::cfmt::CFloat;

use motor_common::util::{atof, atoi};

use crate::smc::{SmcMembers, SmcState, trace_move_velocity, travel_limits};

/// Response buffer size.
const READ_BUF: usize = 128;

/// `st` status-word bit set while any axis is moving.
const STATUS_MOVING: i32 = 0x1;

fn corvus_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared corvus controller endpoint owning the asyn octet handle.
pub struct CorvusController {
    handle: SyncIOHandle,
    num_axes: usize,
}

impl CorvusController {
    /// Wrap a connected octet handle for a controller with `num_axes` axes.
    pub fn new(handle: SyncIOHandle, num_axes: usize) -> Self {
        Self { handle, num_axes }
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

    /// Write a query and read one reply line.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        // C writeReadController: pasynOctetSyncIO->writeRead.
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }

    /// Read every axis position (`pos`) as a vector of exactly `num_axes`
    /// engineering-unit values.
    fn read_positions(&self) -> AsynResult<Vec<f64>> {
        let reply = self.query("pos")?;
        let values: Vec<f64> = reply
            .split_whitespace()
            .filter_map(|tok| tok.parse::<f64>().ok())
            .collect();
        if values.len() < self.num_axes {
            return Err(corvus_err(format!(
                "corvus: pos returned {} values, expected {}",
                values.len(),
                self.num_axes
            )));
        }
        Ok(values[..self.num_axes].to_vec())
    }

    /// Join a target vector and append `verb` (`move`, `rmove`, or `setpos`).
    fn send_target_vector(&self, targets: &[f64], verb: &str) -> AsynResult<()> {
        let mut cmd = String::new();
        for t in targets {
            cmd.push_str(&format!("{} ", CFloat(*t)));
        }
        cmd.push_str(verb);
        self.write_only(&cmd)
    }
}

/// One corvus axis sharing a controller. Implements [`AsynMotor`].
pub struct CorvusAxis {
    controller: Arc<Mutex<CorvusController>>,
    members: Arc<SmcMembers>,
    num_axes: usize,
    /// What the last poll left: C keeps every parameter a poll did not
    /// write (asynMotorAxis params persist across polls).
    last_status: MotorStatus,
    /// The record's soft limits in steps (C `motorHighLimit_`/`motorLowLimit_`),
    /// the jog targets.
    high_limit: f64,
    low_limit: f64,
    /// C's `pasynUser_`, which the axis's lines print on.
    trace: MotorTrace,
}

impl CorvusAxis {
    /// Construct axis `axis_no` (0-based) and read its pitch, pole pairs,
    /// closed-loop period and travel limits as C's constructor does.
    pub fn new(
        controller: Arc<Mutex<CorvusController>>,
        axis_no: usize,
        port: &str,
    ) -> AsynResult<Self> {
        let (num_axes, members) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            if axis_no >= ctrl.num_axes {
                return Err(corvus_err(format!(
                    "corvus: axis {axis_no} out of range (num_axes {})",
                    ctrl.num_axes
                )));
            }
            // C ignores each exchange's status and parses what it got.
            let read = |verb: &str| {
                ctrl.query(&format!("{} {verb}", axis_no + 1))
                    .unwrap_or_default()
            };
            let pitch = atof(&read("getpitch"));
            let pole_pairs = atoi(&read("getpolepairs"));
            let cl_period = atof(&read("getclperiod"));
            let state = SmcState {
                axis_res: pitch / (4.0 * f64::from(pole_pairs)),
                travel_limits: travel_limits(&read("getnlimit")),
                ..SmcState::default()
            };
            let members = SmcMembers {
                axis_no,
                motor_form: None,
                pitch: Some(pitch),
                pole_pairs: Some(pole_pairs),
                cl_period,
                state: Mutex::new(state),
            };
            (ctrl.num_axes, members)
        };
        Ok(Self {
            controller,
            members: Arc::new(members),
            num_axes,
            trace: MotorTrace::new(port, axis_no as i32),
            last_status: motor_common::status_before_poll(),
            high_limit: 0.0,
            low_limit: 0.0,
        })
    }

    /// What C's `report` prints of the axis.
    pub fn members(&self) -> Arc<SmcMembers> {
        self.members.clone()
    }

    /// C `changeResolution`: replace `axisRes`.
    pub fn change_resolution(&mut self, new_resolution: f64) {
        self.members.state().axis_res = new_resolution;
    }

    fn axis_no(&self) -> usize {
        self.members.axis_no
    }

    fn axis_res(&self) -> f64 {
        self.members.axis_res()
    }

    fn lock(&self) -> MutexGuard<'_, CorvusController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Absolute move of this axis (others held at their current position).
    fn absolute_move(
        &self,
        ctrl: &CorvusController,
        target: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C overwrites sv/sa's status with the pos read's.
        let _ = ctrl.write_all(self.accel_velocity(velocity, acceleration));
        // C sends the move with pos1..3 uninitialized when pos fails
        // (SMCcorvusDriver.cpp:257-264); refusing it is a C defect not copied.
        let mut targets = ctrl.read_positions()?;
        targets[self.axis_no()] = target * self.axis_res();
        ctrl.send_target_vector(&targets, "move")
    }

    /// C `sendAccelAndVelocity`'s `sv`/`sa` for all axes, in controller
    /// units.
    fn accel_velocity(&self, velocity: f64, acceleration: f64) -> [String; 2] {
        [
            format!("{} sv", CFloat((velocity * self.axis_res()).abs())),
            format!("{} sa", CFloat((acceleration * self.axis_res()).abs())),
        ]
    }
}

impl AsynMotor for CorvusAxis {
    /// C `SMCcorvusAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (SMCcorvusDriver.cpp).
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
        let ctrl = self.lock();
        self.absolute_move(&ctrl, position, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        let _ = ctrl.write_all(self.accel_velocity(velocity, acceleration));
        // Zero delta for the idle axes; C's absolute position there is a C
        // defect (see module deviations).
        let mut targets = vec![0.0; self.num_axes];
        targets[self.axis_no()] = distance * self.axis_res();
        ctrl.send_target_vector(&targets, "rmove")
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
            "SMCcorvusAxis::moveVelocity",
            min_velocity,
            velocity,
            acceleration,
        );
        let ctrl = self.lock();
        // No reliable jog on the corvus — move to the record's soft limit.
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        };
        self.absolute_move(&ctrl, target, velocity, acceleration)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // Forward homes to the range-measure switch (nrm); reverse calibrates
        // to the low switch (ncal).
        let verb = if forward { "nrm" } else { "ncal" };
        let [sv, sa] = self.accel_velocity(velocity, acceleration);
        self.lock()
            .write_all([sv, sa, format!("{} {verb}", self.axis_no() + 1)])
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // The corvus can only abort all axes at once.
        let ctrl = self.lock();
        ctrl.write_only("abort")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // As in absolute_move: C sends setpos with pos1..3 uninitialized when
        // pos fails (SMCcorvusDriver.cpp:339-346); not copied.
        let mut targets = ctrl.read_positions()?;
        // setpos takes the distance from the current position to the desired
        // origin, hence the negation (C multiplies by -1).
        targets[self.axis_no()] = -position * self.axis_res();
        ctrl.send_target_vector(&targets, "setpos")
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!(
            "{} {} setcloop",
            if enable { 1 } else { 0 },
            self.axis_no() + 1
        ))
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.high_limit = position;
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.low_limit = position;
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID-gain support in the C corvus driver.
        Ok(())
    }

    /// C `SMCcorvusAxis::poll`: a failed exchange skips the rest and sets
    /// only Problem; C's poller ignores the returned error, so no COMM_ERR.
    /// Problem is set from the comm status alone, overwriting the drive-off
    /// problem (SMCcorvusDriver.cpp:473-477).
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut status = self.last_status.clone();
        status.problem = self.poll_into(&mut status).is_err();
        self.last_status = status.clone();
        Ok(status)
    }
}

impl CorvusAxis {
    /// Write each field as C sets its parameter, in C's order.
    fn poll_into(&self, status: &mut MotorStatus) -> AsynResult<()> {
        let ctrl = self.lock();

        // C sscanf's into locals: a short reply leaves the position unset.
        let reply = ctrl.query("pos")?;
        if let Some(position) = reply
            .split_whitespace()
            .filter_map(|tok| tok.parse::<f64>().ok())
            .nth(self.axis_no())
        {
            status.position = position / self.axis_res();
            status.encoder_position = position / self.axis_res();
        }

        let moving = (atoi(&ctrl.query("st")?) & STATUS_MOVING) != 0;
        status.done = !moving;
        status.moving = moving;
        // SMCcorvusDriver.cpp:432 motorStatusMoving_ = !done.
        status.msta_moving = moving;

        // Limits and drive current are not read while moving (it would block
        // the controller's interpreter); they keep the last poll's values.
        if !moving {
            // Switch config: bit 1 (0x2) masks (disables) the switch.
            let sw = ctrl.query(&format!("{} getsw", self.axis_no() + 1))?;
            let (ignore_low, ignore_high) = self.members.read_limit_config(&sw);

            // Switch state: "low high", 0=inactive 1=active.
            let st = ctrl.query(&format!("{} getswst", self.axis_no() + 1))?;
            let state: Vec<i32> = st
                .split_whitespace()
                .filter_map(|tok| tok.parse::<i32>().ok())
                .collect();
            if let [low, high, ..] = state.as_slice() {
                status.low_limit = !ignore_low && *low != 0;
                status.high_limit = !ignore_high && *high != 0;
            }

            let drive_on = atoi(&ctrl.query(&format!("{} getmp", self.axis_no() + 1))?);
            status.powered = drive_on != 0;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// C's status_ starts at 0 and the constructor sets no status bit; only
    /// a good poll sets Done (SMCcorvusDriver.cpp:431).
    #[test]
    fn an_unanswering_controller_s_axis_polls_problem_without_done() {
        use epics_rs::asyn::interfaces::InterfaceType;
        let (handle, _) = SyncIOHandle::connect_port(
            "CORVUS_DEAD",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let controller = Arc::new(Mutex::new(CorvusController::new(handle, 1)));
        let mut axis = CorvusAxis::new(controller, 0, "").unwrap();
        let status = axis.poll(&AsynUser::default()).unwrap();
        assert!(status.problem && !status.done, "{status:?}");
    }

    /// C `SMCcorvusAxis::moveVelocity`'s FLOW line, on the axis's
    /// `pasynUser_` (SMCcorvusDriver.cpp:304-306).
    #[test]
    fn a_velocity_move_is_traced() {
        use epics_rs::asyn::trace::TraceMask;
        use motor_common::testing::LineRig;
        let (_line, handle) = LineRig::new("corvusTraceLine", |_| "0".into());
        let controller = Arc::new(Mutex::new(CorvusController::new(handle, 1)));
        let mut axis = CorvusAxis::new(controller, 0, "corvusTrace").unwrap();
        let lines = crate::smc::trace_lines("corvusTrace", TraceMask::FLOW, || {
            axis.move_velocity(&AsynUser::default(), 1.0, -2.0, 4.0)
                .unwrap();
        });
        assert_eq!(
            lines,
            [
                "SMCcorvusAxis::moveVelocity: baseVelocity=1.000000, slewVelocity=-2.000000, \
              acceleration=4.000000"
            ]
        );
    }

    /// Refuses `sv` and `sa`, answers `pos`; records every write.
    struct SettingsRefused {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for SettingsRefused {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let line = String::from_utf8_lossy(data).into_owned();
            let refused = line.ends_with(" sv") || line.ends_with(" sa");
            self.written.lock().unwrap().push(line);
            if refused {
                Err(corvus_err("refused"))
            } else {
                Ok(data.len())
            }
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = b"1.0 2.0";
            buf[..reply.len()].copy_from_slice(reply);
            Ok(reply.len())
        }
    }

    /// C overwrites sv/sa's status with what follows
    /// (SMCcorvusDriver.cpp:220-292), so a refused setting still
    /// moves or homes, and that command's status is returned.
    #[test]
    fn a_refused_setting_still_sends_the_command_after_it() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = SettingsRefused {
            base: PortDriverBase::new(
                "corvusRefused",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: written.clone(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("corvusRefused").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_secs(1),
        );
        let mut axis = CorvusAxis {
            controller: Arc::new(Mutex::new(CorvusController::new(handle, 2))),
            members: Arc::new(SmcMembers {
                axis_no: 1,
                motor_form: None,
                pitch: Some(0.0),
                pole_pairs: Some(0),
                cl_period: 0.0,
                state: Mutex::new(SmcState {
                    axis_res: 0.5,
                    ..SmcState::default()
                }),
            }),
            num_axes: 2,
            trace: MotorTrace::default(),
            last_status: MotorStatus::default(),
            high_limit: 10.0,
            low_limit: -10.0,
        };
        let user = AsynUser::default();
        axis.move_absolute(&user, 6.0, 0.0, 2.0, 4.0).unwrap();
        axis.move_relative(&user, -2.0, 0.0, 2.0, 4.0).unwrap();
        axis.move_velocity(&user, 0.0, 2.0, 4.0).unwrap();
        axis.home(&user, 0.0, 2.0, 4.0, true).unwrap();
        let written = written.lock().unwrap();
        let tails: Vec<&String> = written
            .iter()
            .filter(|w| !w.ends_with(" sv") && !w.ends_with(" sa"))
            .collect();
        assert_eq!(
            tails,
            [
                "pos",
                "1.000000 3.000000 move",
                "0.000000 -1.000000 rmove",
                "pos",
                "1.000000 5.000000 move",
                "2 nrm"
            ]
        );
    }

    #[test]
    fn status_moving_bit() {
        let moving = |s: i32| (s & STATUS_MOVING) != 0;
        assert!(moving(0x1));
        assert!(moving(0x3));
        assert!(!moving(0x0));
        assert!(!moving(0x2));
    }
}
