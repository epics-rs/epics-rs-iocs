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
//! The asyn-rs motor boundary is dial-frame EGU in both directions (the
//! record converts EGU ↔ raw counts itself), and SMC100 commands/readbacks
//! are already in physical units (mm) — so the correct wire scale is 1.0 in
//! the normal record-EGU-equals-controller-units configuration. `step_size`
//! is kept as a record-EGU → controller-units conversion factor for records
//! deliberately using a different unit (e.g. µm records on an mm
//! controller). C's `stepSize_` (`eguPerStep`) converted its raw-step record
//! boundary instead; passing that C value here would scale moves wrongly.

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::protocol;

/// Response buffer size for a single controller reply (positions and status
/// strings are short; 256 bytes is generous).
const READ_BUF: usize = 256;

/// Command line terminator. The SMC100 speaks CR/LF-terminated ASCII. The C
/// driver relies on the asyn serial port's output EOS to append it
/// (`asynOctetSetInputEos`/`OutputEos` set in st.cmd); asyn-rs exposes no
/// iocsh EOS command yet, so this driver owns framing explicitly and appends
/// the terminator itself. `crate::protocol` keeps the bare command text (as C
/// `outString_` does), so the wire format stays unit-testable.
const TERMINATOR: &[u8] = b"\r\n";

/// A Newport SMC100 axis driver implementing [`AsynMotor`].
pub struct Smc100Axis {
    /// Synchronous octet handle to the dedicated serial-transport asyn port.
    handle: SyncIOHandle,
    /// 1-based controller axis number sent as the command prefix (C
    /// `axisNo_ + 1`). The SMC100 supports a single axis, addressed as `1`.
    axis: u8,
    /// Controller units per record EGU (wire scale; 1.0 in the normal
    /// configuration — see the module Units note).
    step_size: f64,
}

impl Smc100Axis {
    /// Create a driver for `axis` (1-based) over the given serial-port handle.
    /// `step_size` is the controller-units-per-record-EGU wire scale
    /// (normally 1.0; see the module Units note).
    pub fn new(handle: SyncIOHandle, axis: u8, step_size: f64) -> Self {
        Self {
            handle,
            axis,
            step_size,
        }
    }

    /// Frame a command with the CR/LF [`TERMINATOR`] (see its docs for why the
    /// driver appends this itself rather than relying on port EOS).
    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command (C `writeController`); the terminator is appended here.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Write a command and read the reply (C `writeReadController`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let reply = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&reply).into_owned())
    }

    /// C `SMC100Axis::sendAccelAndVelocity`: send `VA` then `AC`, through the
    /// wire scale (identity in the normal configuration).
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
}

fn parse_error(what: &str) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!("SMC100: could not parse {what} response"),
    }
}

impl AsynMotor for Smc100Axis {
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
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C SMC100Axis::moveVelocity: the SMC100 has no free jog, so the driver
        // queries the configured travel limits (SR?/SL?, already in controller
        // EGU) and moves to the limit in the jog direction.
        self.send_accel_and_velocity(acceleration, velocity)?;
        let high =
            protocol::parse_value(&self.write_read(&protocol::cmd_query_high_limit(self.axis))?)
                .ok_or_else(|| parse_error("SR?"))?;
        let low =
            protocol::parse_value(&self.write_read(&protocol::cmd_query_low_limit(self.axis))?)
                .ok_or_else(|| parse_error("SL?"))?;
        let target = if velocity >= 0.0 { high } else { low };
        // The limits are already controller EGU — sent directly, not scaled.
        self.write(&protocol::cmd_move_absolute(self.axis, target))
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
        // Position (TP): controller units -> record EGU via the wire scale
        // (identity in the normal configuration).
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
