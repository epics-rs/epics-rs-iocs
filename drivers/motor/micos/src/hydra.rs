//! Micos SMC hydra controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorMicos/micosApp/src/SMChydraAxis.cpp` +
//! `SMChydraDriver.cpp` (a model-3 `asynMotorController`/`asynMotorAxis` pair).
//! Connects over a `drvAsynIPPort` or `drvAsynSerialPort`; commands are ASCII,
//! CR/LF-terminated (the startup script sets both EOS, as the C example does;
//! the driver sends bare commands).
//!
//! ## Per-axis addressing
//!
//! Unlike the corvus sibling, the hydra addresses each axis independently: every
//! command carries the 1-based axis number (`<value> <axis> <verb>` for setters,
//! `<axis> <verb>` for queries), so no shared target vector is needed.
//!
//! ## Units
//!
//! As C, the record's steps are scaled by `axisRes` (`pitch/(4·polePairs)` for
//! a stepper motor form, `clPeriod` otherwise) into the controller's units and
//! divided back on poll; `SMChydraChangeResolution` overrides it. The motor
//! form also selects the `set_closed_loop` behaviour.
//!
//! ## Deviations (documented)
//!
//! - The C poll computes an e-stop-switch problem flag from status bit `0x200`
//!   but then unconditionally overwrites the problem flag to `0` a few lines
//!   later (a latent bug that discards it). This port honours the intent and
//!   reports bit `0x200` as a problem.

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
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

/// Shared hydra controller endpoint owning the asyn octet handle.
pub struct HydraController {
    handle: SyncIOHandle,
}

impl HydraController {
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

    /// Write a query and read one reply line.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        // C writeReadController: pasynOctetSyncIO->writeRead.
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }
}

/// C `axisRes_` by motor form: a stepper's `pitch/(4·polePairs)`, else
/// `clPeriod`.
fn axis_res(motor_form: i32, pitch: f64, pole_pairs: i32, cl_period: f64) -> f64 {
    if motor_form == 0 {
        pitch / (4.0 * f64::from(pole_pairs))
    } else {
        cl_period
    }
}

/// One hydra axis sharing a controller. Implements [`AsynMotor`].
pub struct HydraAxis {
    controller: Arc<Mutex<HydraController>>,
    members: Arc<SmcMembers>,
    /// Motor form (`getmotor`): 0 stepper, 1 linear/torque, other. Selects the
    /// `set_closed_loop` behaviour.
    motor_form: i32,
    /// The axis's `SMCHYDRA_REGULATOR_MODE`, which only the controller
    /// port's write sets. C reads it uninitialised before the first write;
    /// it is 0 here.
    regulator_mode: Arc<AtomicI32>,
    /// What the last poll left: a failed exchange keeps every parameter
    /// the poll did not reach.
    last_status: MotorStatus,
    /// C's `pasynUser_`, which the axis's lines print on.
    trace: MotorTrace,
}

impl HydraAxis {
    /// Construct axis `axis_no` (0-based) and read its motor form, pitch,
    /// pole pairs and closed-loop period for `axisRes`, then its travel
    /// limits, as C's constructor.
    /// C ignores the queries' status (SMChydraAxis.cpp:15-17), so a controller
    /// that does not answer leaves each at `atoi("")`/`atof("")`, 0.
    pub fn new(
        controller: Arc<Mutex<HydraController>>,
        axis_no: usize,
        regulator_mode: Arc<AtomicI32>,
        port: &str,
    ) -> Self {
        let addr = axis_no + 1;
        let (motor_form, members) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let read = |verb: &str| ctrl.query(&format!("{addr} {verb}")).unwrap_or_default();
            let motor_form = atoi(&read("getmotor"));
            let pitch = atof(&read("getpitch"));
            let pole_pairs = atoi(&read("getpolepairs"));
            let cl_period = atof(&read("getclperiod"));
            let state = SmcState {
                axis_res: axis_res(motor_form, pitch, pole_pairs, cl_period),
                travel_limits: travel_limits(&read("getnlimit")),
                ..SmcState::default()
            };
            let members = SmcMembers {
                axis_no,
                motor_form: Some(motor_form),
                pitch: Some(pitch),
                pole_pairs: Some(pole_pairs),
                cl_period,
                state: Mutex::new(state),
            };
            (motor_form, members)
        };
        Self {
            controller,
            members: Arc::new(members),
            motor_form,
            regulator_mode,
            trace: MotorTrace::new(port, axis_no as i32),
            last_status: MotorStatus {
                gain_support: true,
                ..MotorStatus::default()
            },
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

    fn lock(&self) -> MutexGuard<'_, HydraController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `sendAccelAndVelocity`'s `snv`/`sna`, then `cmd`, as one
    /// [`HydraController::write_all`].
    fn write_after_accel_velocity(
        &self,
        ctrl: &HydraController,
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
}

impl AsynMotor for HydraAxis {
    /// C `SMChydraAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (SMChydraAxis.cpp).
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
            "SMChydraAxis::moveVelocity",
            min_velocity,
            velocity,
            acceleration,
        );
        // No jog command: move to the travel limit the constructor read.
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

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        let addr = self.addr();
        // Set stop deceleration, then abort this axis.
        ctrl.write_all([
            format!(
                "{} {addr} ssd",
                CFloat((acceleration * self.axis_res()).abs())
            ),
            format!("{addr} nabort"),
        ])
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
        // Only the stepper (0) and linear/torque (1) forms support this.
        if !matches!(self.motor_form, 0 | 1) {
            return Ok(());
        }
        let ctrl = self.lock();
        let addr = self.addr();
        if enable {
            // Reinit so the closed-loop setting takes effect (powers the motor);
            // the controller needs a delay after init.
            let status = ctrl.write_all([
                format!(
                    "{} {addr} setcloop",
                    self.regulator_mode.load(Ordering::Relaxed)
                ),
                format!("{addr} init"),
            ]);
            std::thread::sleep(INIT_DELAY);
            status
        } else {
            ctrl.write_only(&format!("{addr} motoroff"))
        }
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID-gain support in the C hydra driver.
        Ok(())
    }

    /// C `SMChydraAxis::poll`: a failed exchange skips the rest and sets
    /// Problem, and the parameters set before it are still posted
    /// (SMChydraAxis.cpp:247-362).
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut status = self.last_status.clone();
        if self.poll_into(&mut status).is_err() {
            status.problem = true;
        }
        self.last_status = status.clone();
        Ok(status)
    }
}

impl HydraAxis {
    /// Write each field as C sets its parameter, in C's order.
    fn poll_into(&self, status: &mut MotorStatus) -> AsynResult<()> {
        let ctrl = self.lock();
        let addr = self.addr();

        let position = atof(&ctrl.query(&format!("{addr} np"))?) / self.axis_res();
        status.position = position;
        status.encoder_position = position;

        let axis_status = atoi(&ctrl.query(&format!("{addr} nst"))?);
        let moving = (axis_status & STATUS_MOVING) != 0;
        status.done = !moving;
        status.moving = moving;
        // SMChydraAxis.cpp:273 motorStatusMoving_ = !done.
        status.msta_moving = moving;

        // SMChydraAxis.cpp:276-281 reads the commanded velocity and
        // acceleration, ignoring the replies and their status.
        let _ = ctrl.query(&format!("{addr} gnv"));
        let _ = ctrl.query(&format!("{addr} gna"));
        trace_axis_status(
            &self.trace,
            "SMChydraAxis::poll",
            self.members.axis_no,
            axis_status,
        );

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
        // C sets problem from 0x200 (SMChydraAxis.cpp:302-311), then
        // overwrites it at :357 and :360, a dead store; not copied.
        status.problem = (axis_status & STATUS_ESTOP_SWITCH) != 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;

    /// C's poll prints the `nst` limit, e-stop and busy bits, and
    /// `moveVelocity` its FLOW line, on the axis's `pasynUser_`.
    #[test]
    fn the_status_bits_and_a_velocity_move_are_traced() {
        use epics_rs::asyn::trace::TraceMask;
        use motor_common::testing::LineRig;
        let (_line, handle) = LineRig::new("hydraTraceLine", |cmd| match cmd.trim_end() {
            "1 getmotor" => "1".into(),
            "1 getclperiod" => "0.5".into(),
            "1 nst" => "1728".into(),
            _ => String::new(),
        });
        let controller = Arc::new(Mutex::new(HydraController::new(handle)));
        let mut axis = HydraAxis::new(controller, 0, Arc::default(), "hydraTrace");
        let mask = TraceMask::ERROR | TraceMask::IO_DRIVER | TraceMask::FLOW;
        let lines = crate::smc::trace_lines("hydraTrace", mask, || {
            let user = AsynUser::default();
            axis.poll(&user).unwrap();
            axis.move_velocity(&user, 1.0, 2.0, 4.0).unwrap();
        });
        assert_eq!(
            lines,
            [
                "SMChydraAxis::poll: axis 1 limit indicator active.",
                "SMChydraAxis::poll: axis 1 emergency stopped.",
                "SMChydraAxis::poll: axis 1 emergency stop switch active.",
                "SMChydraAxis::poll: axis 1 device is busy - move commands discarded.",
                "SMChydraAxis::moveVelocity: baseVelocity=1.000000, slewVelocity=2.000000, \
                 acceleration=4.000000",
            ]
        );
    }

    /// A failed `nst` skips the rest of C's poll, but the position it read
    /// first is still posted, with Problem (SMChydraAxis.cpp:255-266).
    #[test]
    fn a_failed_status_read_still_posts_the_position() {
        use motor_common::testing::LineRig;
        let (_line, handle) = LineRig::failing(
            "hydraNstFails",
            |cmd| match cmd.trim_end() {
                "1 getmotor" => "1".into(),
                "1 getclperiod" => "0.5".into(),
                "1 np" => "2.5".into(),
                _ => String::new(),
            },
            |cmd| cmd.trim_end() == "1 nst",
        );
        let controller = Arc::new(Mutex::new(HydraController::new(handle)));
        let mut axis = HydraAxis::new(controller, 0, Arc::default(), "");
        let res = axis.axis_res();
        let status = axis.poll(&AsynUser::default()).unwrap();
        assert_eq!(status.position, 2.5 / res);
        assert_eq!(status.encoder_position, 2.5 / res);
        assert!(status.problem);
        assert!(status.gain_support);
    }
    #[test]
    fn an_axis_of_an_unanswering_controller_is_still_built() {
        let (handle, _) =
            SyncIOHandle::connect_port("HYDRA_NONE", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(HydraController::new(handle)));
        assert_eq!(
            HydraAxis::new(controller, 0, Arc::default(), "").motor_form,
            0
        );
    }

    /// C `sprintf("%f %i nm")` of a NaN target sends glibc's `nan`.
    #[test]
    fn a_nan_target_goes_out_as_c_printf_spells_it() {
        use motor_common::testing::LineRig;
        let (line, handle) = LineRig::new("hydraNan", |_| String::new());
        let mut axis = HydraAxis {
            controller: Arc::new(Mutex::new(HydraController::new(handle))),
            members: Arc::new(SmcMembers {
                axis_no: 0,
                motor_form: Some(0),
                pitch: Some(0.0),
                pole_pairs: Some(0),
                cl_period: 0.0,
                state: Mutex::new(SmcState {
                    axis_res: 0.5,
                    ..SmcState::default()
                }),
            }),
            motor_form: 0,
            regulator_mode: Arc::default(),
            last_status: MotorStatus::default(),
            trace: MotorTrace::default(),
        };
        line.sent();
        axis.move_absolute(&AsynUser::default(), f64::NAN, 0.0, 2.0, 4.0)
            .unwrap();
        let sent = line.sent();
        assert!(sent.iter().any(|s| s.trim_end() == "nan 1 nm"), "{sent:?}");
    }

    #[test]
    fn status_bits() {
        let moving = |s: i32| (s & STATUS_MOVING) != 0;
        let powered = |s: i32| (s & STATUS_POWER_OFF) == 0;
        let problem = |s: i32| (s & STATUS_ESTOP_SWITCH) != 0;
        assert!(moving(0x1));
        assert!(!moving(0x100));
        assert!(!powered(0x100));
        assert!(powered(0x1));
        assert!(problem(0x200));
        assert!(!problem(0x1));
    }

    /// Refuses every setting write (`snv`, `sna`, `ssd`, `setcloop`);
    /// records every write.
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
            let refused = ["snv", "sna", "ssd", "setcloop"]
                .iter()
                .any(|verb| line.ends_with(verb));
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
    /// (SMChydraAxis.cpp:95-177, :207-213), so a refused setting still
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
                "hydraRefused",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: written.clone(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("hydraRefused").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let mut axis = HydraAxis {
            controller: Arc::new(Mutex::new(HydraController::new(handle))),
            members: Arc::new(SmcMembers {
                axis_no: 0,
                motor_form: Some(0),
                pitch: Some(0.0),
                pole_pairs: Some(0),
                cl_period: 0.0,
                state: Mutex::new(SmcState {
                    axis_res: 0.5,
                    travel_limits: (-100.0, 100.0),
                    ..SmcState::default()
                }),
            }),
            motor_form: 0,
            regulator_mode: Arc::default(),
            last_status: MotorStatus::default(),
            trace: MotorTrace::default(),
        };
        let user = AsynUser::default();
        axis.move_absolute(&user, 2.0, 0.0, 2.0, 4.0).unwrap();
        axis.move_relative(&user, -2.0, 0.0, 2.0, 4.0).unwrap();
        axis.move_velocity(&user, 0.0, 2.0, 4.0).unwrap();
        axis.home(&user, 0.0, 2.0, 4.0, false).unwrap();
        axis.stop(&user, 4.0).unwrap();
        axis.set_closed_loop(&user, true).unwrap();
        let tails: Vec<String> = written
            .lock()
            .unwrap()
            .iter()
            .filter(|w| {
                !["snv", "sna", "ssd", "setcloop"]
                    .iter()
                    .any(|v| w.ends_with(v))
            })
            .cloned()
            .collect();
        assert_eq!(
            tails,
            [
                "1.000000 1 nm",
                "-1.000000 1 nr",
                "100.000000 1 nm",
                "1 ncal",
                "1 nabort",
                "1 init"
            ]
        );
        // Four moves of three writes, then stop's and closed-loop's two.
        assert_eq!(written.lock().unwrap().len(), 4 * 3 + 2 + 2);
    }

    #[test]
    fn axis_res_follows_the_motor_form() {
        assert_eq!(axis_res(0, 1.0, 50, 0.0001), 0.005);
        assert_eq!(axis_res(1, 1.0, 50, 0.0001), 0.0001);
    }
}
