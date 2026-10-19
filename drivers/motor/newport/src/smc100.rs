//! Newport SMC100 single-axis motor controller driver.
//!
//! Ported from `motorNewport/newportApp/src/SMC100Driver.cpp`
//! (`SMC100Axis`). The C `SMC100Controller`/`SMC100Axis` pair subclasses
//! `asynMotorController`/`asynMotorAxis`; here the axis implements asyn-rs
//! [`AsynMotor`], and the controller/serial-port/record wiring is assembled by
//! the IOC binary (the analogue of `SMC100CreateController`).
//!
//! The controller speaks a serial ASCII protocol over a dedicated asyn serial
//! port; this driver holds a [`SyncIOHandle`] to that port and issues
//! [`crate::protocol`] commands. `MAX_SMC100_AXES` is 1 in the C driver, so a
//! [`Smc100Axis`] owns exactly one axis.
//!
//! ## Units
//!
//! The record boundary is raw steps, as in SMC100Driver.cpp: every value is
//! `steps * stepSize_` (the `eguPerStep` of `SMC100CreateController`) and the
//! `TP` readback is `position / stepSize_`, so the record's `MRES` must equal
//! `eguPerStep`. The jog targets (`SR?`/`SL?`) are controller units already.
//!
//! C creates the controller with an `eguPerStep` of 0 and divides by it at
//! every poll; `SMC100CreateController` refuses it here.

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, StatusWrite};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use crate::protocol;

/// Response buffer size for a single controller reply (positions and status
/// strings are short; 256 bytes is generous).
const READ_BUF: usize = 256;

/// A Newport SMC100 axis driver implementing [`AsynMotor`].
pub struct Smc100Axis {
    /// Synchronous octet handle to the dedicated serial-transport asyn port.
    handle: SyncIOHandle,
    /// 1-based controller axis number sent as the command prefix (C
    /// `axisNo_ + 1`). The SMC100 supports a single axis, addressed as `1`.
    axis: u8,
    /// Controller units per record step (C `stepSize_`, `eguPerStep`).
    step_size: f64,
    /// C `pasynUser_`.
    trace: MotorTrace,
    /// What the last write set, for [`AsynMotor::take_status_writes`].
    status_writes: Vec<StatusWrite>,
}

impl Smc100Axis {
    /// Create a driver for `axis` (1-based) over the given serial-port handle.
    /// `step_size` is C's `eguPerStep` (module Units note).
    pub fn new(handle: SyncIOHandle, axis: u8, step_size: f64, trace: MotorTrace) -> Self {
        Self {
            handle,
            axis,
            step_size,
            trace,
            status_writes: Vec::new(),
        }
    }

    /// Write a command (C `writeController`). C SMC100Driver sets no EOS: the
    /// CR/LF framing is the port's, from st.cmd.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Write a command and read the reply (C `writeReadController`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        let reply = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        Ok(String::from_utf8_lossy(&reply).into_owned())
    }

    /// C `SMC100Axis::sendAccelAndVelocity`: send `VA` then `AC`, in steps
    /// times `stepSize_`.
    fn send_accel_and_velocity(&self, acceleration: f64, velocity: f64) -> AsynResult<()> {
        self.write(&protocol::cmd_set_velocity(
            self.axis,
            velocity * self.step_size,
        ))?;
        self.write(&protocol::cmd_set_acceleration(
            self.axis,
            acceleration * self.step_size,
        ))?;
        Ok(())
    }

    /// C SMC100Axis::moveVelocity: the SMC100 has no free jog, so the driver
    /// queries the configured travel limits (SR?/SL?, already in controller
    /// EGU) and moves to the limit in the jog direction.
    fn jog_to_limit(&self, velocity: f64, acceleration: f64) -> AsynResult<()> {
        self.send_accel_and_velocity(acceleration, velocity)?;
        // A reply under 3 bytes is refused: C's `atof(&inString_[3])` reads
        // past it into the previous reply.
        let high =
            protocol::parse_value(&self.write_read(&protocol::cmd_query_high_limit(self.axis))?)
                .ok_or_else(|| parse_error("SR?"))?;
        let low =
            protocol::parse_value(&self.write_read(&protocol::cmd_query_low_limit(self.axis))?)
                .ok_or_else(|| parse_error("SL?"))?;
        // C: `maxVelocity > 0.`, so a zero jog goes to the low limit.
        let target = if velocity > 0.0 { high } else { low };
        // The limits are already controller EGU — sent directly, not scaled.
        self.write(&protocol::cmd_move_absolute(self.axis, target))
    }
}

fn parse_error(what: &str) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!("SMC100: could not parse {what} response"),
    }
}

impl AsynMotor for Smc100Axis {
    /// C `SMC100Axis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (SMC100Driver.cpp).
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
        self.send_accel_and_velocity(acceleration, velocity)?;
        self.write(&protocol::cmd_move_absolute(
            self.axis,
            position * self.step_size,
        ))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C SMC100 has a native relative move (PR); override the trait's
        // poll-then-absolute default to match.
        self.send_accel_and_velocity(acceleration, velocity)?;
        self.write(&protocol::cmd_move_relative(
            self.axis,
            distance * self.step_size,
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.trace.print(
            TraceMask::FLOW,
            0,
            format_args!(
                "SMC100Axis::moveVelocity: minVelocity={:.6}, maxVelocity={:.6}, acceleration={:.6}\n",
                CFloat(min_velocity),
                CFloat(velocity),
                CFloat(acceleration)
            ),
        );
        let result = self.jog_to_limit(velocity, acceleration);
        // C's `skip` label: PROBLEM is the command status (SMC100Driver.cpp:254).
        self.status_writes
            .push(StatusWrite::Problem(result.is_err()));
        result
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C SMC100Axis::home issues OR (home search); direction is not
        // parameterized on this controller.
        self.write(&protocol::cmd_home(self.axis))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.write(&protocol::cmd_stop(self.axis))
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C SMC100Axis::setPosition has no functional body — the command
        // sprintf is commented out and no "redefine position" command is wired
        // for the SMC100. This is intentionally a no-op.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Position (TP): controller units to record steps.
        let position_units =
            protocol::parse_value(&self.write_read(&protocol::cmd_query_position(self.axis))?)
                .ok_or_else(|| parse_error("TP"))?;
        let position = position_units / self.step_size;

        // Status (TS): moving / limits / at-home.
        let status =
            protocol::parse_status(&self.write_read(&protocol::cmd_query_status(self.axis))?)
                .ok_or_else(|| parse_error("TS"))?;

        Ok(MotorStatus {
            position,
            encoder_position: position,
            done: !status.moving,
            moving: status.moving,
            high_limit: status.high_limit,
            low_limit: status.low_limit,
            home: status.at_home,
            // The SMC100 exposes no separate encoder and ignores VBAS; the
            // record then treats base velocity as 0 in its acceleration math.
            has_encoder: false,
            vbas_supported: false,
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::LineRig;

    /// SMC100Driver.cpp: values go out as steps * stepSize_, TP comes back
    /// / stepSize_, and the jog targets SR?/SL? are sent unscaled.
    #[test]
    fn record_steps_scale_by_egu_per_step() {
        let (line, io) = LineRig::new("smc100_steps", |last| {
            match last {
                "1TP" => "1TP2.5",
                "1TS" => "1TS000033",
                "1SR?" => "1SR25",
                "1SL?" => "1SL-5",
                _ => "",
            }
            .to_string()
        });
        let mut axis = Smc100Axis::new(io, 1, 0.00005, MotorTrace::new("SMC100_T", 0));
        let user = AsynUser::default();
        axis.move_absolute(&user, 20000.0, 0.0, 2000.0, 4000.0)
            .unwrap();
        axis.move_relative(&user, -100.0, 0.0, 2000.0, 4000.0)
            .unwrap();
        axis.move_velocity(&user, 0.0, 2000.0, 4000.0).unwrap();
        assert_eq!(
            line.sent(),
            [
                "1VA0.100000",
                "1AC0.200000",
                "1PA1.000000",
                "1VA0.100000",
                "1AC0.200000",
                "1PR-0.005000",
                "1VA0.100000",
                "1AC0.200000",
                "1SR?",
                "1SL?",
                "1PA25.000000"
            ]
        );
        assert_eq!(axis.poll(&user).unwrap().position, 50000.0);
    }

    /// SMC100Driver.cpp:243-249: only a positive jog velocity goes to the
    /// high limit.
    #[test]
    fn a_zero_jog_goes_to_the_low_limit() {
        let (line, io) = LineRig::new("smc100_zero_jog", |last| {
            match last {
                "1SR?" => "1SR25",
                "1SL?" => "1SL-5",
                _ => "",
            }
            .to_string()
        });
        let mut axis = Smc100Axis::new(io, 1, 1.0, MotorTrace::new("SMC100_T", 0));
        axis.move_velocity(&AsynUser::default(), 0.0, 0.0, 0.0)
            .unwrap();
        assert_eq!(line.sent().last().unwrap(), "1PA-5.000000");
    }

    /// SMC100Driver.cpp:253-255: moveVelocity's `skip` label posts PROBLEM
    /// as its command status, raised on a failed one and cleared otherwise.
    #[test]
    fn move_velocity_posts_problem_as_its_status() {
        let (_line, io) = LineRig::failing(
            "smc100_jog_problem",
            |last| match last {
                "1SR?" => "1SR25".to_string(),
                _ => "1SL-5".to_string(),
            },
            |cmd| cmd == "1SL?",
        );
        let mut axis = Smc100Axis::new(io, 1, 1.0, MotorTrace::new("SMC100_T", 0));
        let user = AsynUser::default();
        assert!(axis.move_velocity(&user, 0.0, 1.0, 0.0).is_err());
        assert_eq!(axis.take_status_writes(), [StatusWrite::Problem(true)]);
        let (_line, io) = LineRig::new("smc100_jog_ok", |_| "1SR25".to_string());
        let mut axis = Smc100Axis::new(io, 1, 1.0, MotorTrace::new("SMC100_T", 0));
        axis.move_velocity(&user, 0.0, 1.0, 0.0).unwrap();
        assert_eq!(axis.take_status_writes(), [StatusWrite::Problem(false)]);
    }

    /// SMC100Driver.cpp:218: moveVelocity's FLOW line, on the axis's user
    /// and before any command.
    #[test]
    fn move_velocity_traces_as_c() {
        use std::sync::{Arc, Mutex};

        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};

        let (_line, io) = LineRig::new("smc100_flow_ip", |_| String::new());
        let mut axis = Smc100Axis::new(io, 1, 1.0, MotorTrace::new("smc100_flow", 0));
        let path = std::env::temp_dir().join(format!("smc100-flow-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_device_trace_mask("smc100_flow", 0, TraceMask::FLOW);
        trace.set_device_trace_info_mask("smc100_flow", 0, TraceInfoMask::empty());
        trace.set_device_trace_file("smc100_flow", 0, TraceFile::File(file));
        let _ = axis.move_velocity(&AsynUser::default(), 0.5, 100.0, 250.0);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            text,
            "SMC100Axis::moveVelocity: minVelocity=0.500000, maxVelocity=100.000000, acceleration=250.000000\n"
        );
    }
}
