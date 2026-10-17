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
//! - The regulator-mode aux parameter (`SMCHYDRA_REGULATOR_MODE`, driven by a
//!   separate PV in C) is not modeled; `set_closed_loop(true)` uses the C startup
//!   default of `0`. Configuring a non-default regulator mode would require
//!   modeling that PV.
//! - The C poll computes an e-stop-switch problem flag from status bit `0x200`
//!   but then unconditionally overwrites the problem flag to `0` a few lines
//!   later (a latent bug that discards it). This port honours the intent and
//!   reports bit `0x200` as a problem.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size.
const READ_BUF: usize = 128;

/// `nst` status bits.
const STATUS_MOVING: i32 = 0x1; // axis in motion
const STATUS_POWER_OFF: i32 = 0x100; // drive power disabled
const STATUS_ESTOP_SWITCH: i32 = 0x200; // e-stop switch active

/// Switch-config bit that disables (masks) a limit switch.
const SWITCH_IGNORE: i32 = 0x2;

/// Regulator mode used when enabling closed loop (C startup default, unmodeled
/// aux parameter `SMCHYDRA_REGULATOR_MODE`).
const REGULATOR_MODE_DEFAULT: i32 = 0;

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

/// `sscanf(reply, "%lf %lf", &neg, &pos)` (SMChydraAxis.cpp:57). A value
/// sscanf does not convert is 0 here; C leaves the member uninitialized, a
/// defect not copied.
fn travel_limits(reply: &str) -> (f64, f64) {
    let mut values = reply
        .split_whitespace()
        .map_while(|tok| tok.parse::<f64>().ok());
    let neg = values.next().unwrap_or(0.0);
    (neg, values.next().unwrap_or(0.0))
}

/// One hydra axis sharing a controller. Implements [`AsynMotor`].
pub struct HydraAxis {
    controller: Arc<Mutex<HydraController>>,
    /// 0-based axis index; commands use `axis_no + 1`.
    axis_no: usize,
    /// Motor form (`getmotor`): 0 stepper, 1 linear/torque, other. Selects the
    /// `set_closed_loop` behaviour.
    motor_form: i32,
    /// C `axisRes_`: controller units per record step.
    axis_res: f64,
    /// C `negTravelLimit_`/`posTravelLimit_`, the `getnlimit` the
    /// constructor read; `move_velocity` moves to one (no jog command).
    travel_limits: (f64, f64),
}

impl HydraAxis {
    /// Construct axis `axis_no` (0-based) and read its motor form, pitch,
    /// pole pairs and closed-loop period for `axisRes`, then its travel
    /// limits, as C's constructor.
    /// C ignores the queries' status (SMChydraAxis.cpp:15-17), so a controller
    /// that does not answer leaves each at `atoi("")`/`atof("")`, 0.
    pub fn new(controller: Arc<Mutex<HydraController>>, axis_no: usize) -> Self {
        let addr = axis_no + 1;
        let (motor_form, axis_res, travel_limits) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let read = |verb: &str| ctrl.query(&format!("{addr} {verb}")).unwrap_or_default();
            let motor_form = atoi(&read("getmotor"));
            let pitch = atof(&read("getpitch"));
            let pole_pairs = atoi(&read("getpolepairs"));
            let cl_period = atof(&read("getclperiod"));
            let axis_res = axis_res(motor_form, pitch, pole_pairs, cl_period);
            (motor_form, axis_res, travel_limits(&read("getnlimit")))
        };
        Self {
            controller,
            axis_no,
            motor_form,
            axis_res,
            travel_limits,
        }
    }

    /// C `changeResolution`: replace `axisRes`.
    pub fn change_resolution(&mut self, new_resolution: f64) {
        self.axis_res = new_resolution;
    }

    fn addr(&self) -> usize {
        self.axis_no + 1
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
            format!("{:.6} {addr} snv", (velocity * self.axis_res).abs()),
            format!("{:.6} {addr} sna", (acceleration * self.axis_res).abs()),
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
        let cmd = format!("{:.6} {} nm", position * self.axis_res, self.addr());
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
        let cmd = format!("{:.6} {} nr", distance * self.axis_res, self.addr());
        self.write_after_accel_velocity(&self.lock(), velocity, acceleration, cmd)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // No jog command: move to the travel limit the constructor read.
        let (neg, pos) = self.travel_limits;
        let target = if velocity > 0.0 { pos } else { neg };
        let cmd = format!("{target:.6} {} nm", self.addr());
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
            format!("{:.6} {addr} ssd", (acceleration * self.axis_res).abs()),
            format!("{addr} nabort"),
        ])
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        // setnpos takes the distance from the current position to the desired
        // origin, hence the negation (C multiplies by -1).
        ctrl.write_only(&format!(
            "{:.6} {} setnpos",
            -position * self.axis_res,
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
                format!("{REGULATOR_MODE_DEFAULT} {addr} setcloop"),
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

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let addr = self.addr();

        let position = atof(&ctrl.query(&format!("{addr} np"))?);

        let status = atoi(&ctrl.query(&format!("{addr} nst"))?);
        let moving = (status & STATUS_MOVING) != 0;
        let powered = (status & STATUS_POWER_OFF) == 0;
        let problem = (status & STATUS_ESTOP_SWITCH) != 0;

        // SMChydraAxis.cpp:276-281 reads the commanded velocity and
        // acceleration, ignoring the replies and their status.
        let _ = ctrl.query(&format!("{addr} gnv"));
        let _ = ctrl.query(&format!("{addr} gna"));

        // Switch config: bit 1 (0x2) masks (disables) the switch.
        let sw = ctrl.query(&format!("{addr} getsw"))?;
        let cfg: Vec<i32> = sw
            .split_whitespace()
            .filter_map(|tok| tok.parse::<i32>().ok())
            .collect();
        let (ignore_low, ignore_high) = match cfg.as_slice() {
            [low, high, ..] => (low & SWITCH_IGNORE, high & SWITCH_IGNORE),
            _ => (0, 0),
        };

        // Switch state: "low high", 0=inactive 1=active.
        let st = ctrl.query(&format!("{addr} getswst"))?;
        let state: Vec<i32> = st
            .split_whitespace()
            .filter_map(|tok| tok.parse::<i32>().ok())
            .collect();
        let (mut low_limit, mut high_limit) = (false, false);
        if let [low, high, ..] = state.as_slice() {
            low_limit = ignore_low == 0 && *low != 0;
            high_limit = ignore_high == 0 && *high != 0;
        }
        drop(ctrl);

        Ok(MotorStatus {
            position: position / self.axis_res,
            encoder_position: position / self.axis_res,
            velocity: 0.0,
            done: !moving,
            moving,
            // SMChydraAxis.cpp:273 motorStatusMoving_ = !done.
            msta_moving: moving,
            gain_support: true,
            high_limit,
            low_limit,
            powered,
            problem,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;

    #[test]
    fn an_axis_of_an_unanswering_controller_is_still_built() {
        let (handle, _) =
            SyncIOHandle::connect_port("HYDRA_NONE", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(HydraController::new(handle)));
        assert_eq!(HydraAxis::new(controller, 0).motor_form, 0);
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
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = SettingsRefused {
            base: PortDriverBase::new("hydraRefused", 1, PortFlags::default()),
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
            axis_no: 0,
            motor_form: 0,
            axis_res: 0.5,
            travel_limits: (-100.0, 100.0),
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
    fn travel_limits_scan_as_c_sscanf() {
        assert_eq!(travel_limits("-100.0 100.0"), (-100.0, 100.0));
        assert_eq!(travel_limits("-5 x"), (-5.0, 0.0));
        assert_eq!(travel_limits(""), (0.0, 0.0));
    }

    #[test]
    fn axis_res_follows_the_motor_form() {
        assert_eq!(axis_res(0, 1.0, 50, 0.0001), 0.005);
        assert_eq!(axis_res(1, 1.0, 50, 0.0001), 0.0001);
    }

    #[test]
    fn switch_ignore_mask() {
        let ignored = |cfg: i32| (cfg & SWITCH_IGNORE) != 0;
        assert!(!ignored(0x0));
        assert!(!ignored(0x1));
        assert!(ignored(0x2));
        assert!(ignored(0x3));
    }
}
