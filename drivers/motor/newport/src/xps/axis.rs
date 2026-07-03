//! XPS axis: one positioner exposed as an asyn [`AsynMotor`].
//!
//! Faithful port of `XPSAxis` (`XPSAxis.cpp`). Each axis holds a clone of the
//! shared [`XpsController`] (for the poll socket + group registry) plus its own
//! `Fire`-mode **move socket**. Reads (`poll`, PID, status) go through the
//! locked controller's poll socket; the actual `GroupMove*`/`GroupHomeSearch`
//! go out the axis's own move socket without waiting for completion — the poll
//! observes when the move finishes via `GroupStatusGet`.
//!
//! Positions are scaled by `step_size` (device units per motor step, C
//! `stepSize_`): commands send `value * step_size`, readbacks divide by it.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::user::AsynUser;

use super::controller::XpsController;
use super::corrector;
use super::rpc::{XpsError, XpsSocket};

/// Velocity below which the axis is considered stopped (C
/// `XPS_VELOCITY_DEADBAND`).
const XPS_VELOCITY_DEADBAND: f64 = 0.0000001;

/// `PositionerErrorGet` bit for the positive hardware end-of-run limit
/// (C `XPSC8_END_OF_RUN_PLUS`).
const XPSC8_END_OF_RUN_PLUS: u32 = 0x8000_0200;
/// `PositionerErrorGet` bit for the negative hardware end-of-run limit
/// (C `XPSC8_END_OF_RUN_MINUS`).
const XPSC8_END_OF_RUN_MINUS: u32 = 0x8000_0100;

/// The XPS group status code (`GroupStatusGet`) that means "home switch
/// active" for the encoder-home (ATHM) signal.
const XPS_STATUS_HOMING_DONE: i32 = 11;

/// One XPS positioner as an asyn motor axis.
pub struct XpsAxis {
    controller: Arc<Mutex<XpsController>>,
    move_sock: XpsSocket,
    positioner_name: String,
    group_name: String,
    step_size: f64,
    /// Cached S-gamma jerk times read at construction, resent on every move
    /// (C caches these in the `XPS_MIN_JERK`/`XPS_MAX_JERK` record params).
    min_jerk: f64,
    max_jerk: f64,
    /// Last group status code seen by [`poll`](AsynMotor::poll); `move` reads it
    /// to decide whether to auto-enable a disabled axis.
    axis_status: i32,
}

impl XpsAxis {
    /// Build an axis for `positioner_name` (`group.positioner`) over its own
    /// `Fire`-mode move socket, registering it with the controller and reading
    /// its S-gamma jerk times (C `XPSAxis` constructor).
    pub fn new(
        controller: Arc<Mutex<XpsController>>,
        move_sock: XpsSocket,
        positioner_name: &str,
        step_size: f64,
    ) -> AsynResult<Self> {
        let group_name = group_of(positioner_name).to_string();
        let (min_jerk, max_jerk) = {
            let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let (_vel, _accel, min_jerk, max_jerk) = ctrl
                .poll_socket()
                .positioner_sgamma_parameters_get(positioner_name)?;
            ctrl.register_axis(&group_name);
            (min_jerk, max_jerk)
        };
        Ok(Self {
            controller,
            move_sock,
            positioner_name: positioner_name.to_string(),
            group_name,
            step_size,
            min_jerk,
            max_jerk,
            axis_status: 0,
        })
    }

    fn lock_controller(&self) -> MutexGuard<'_, XpsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Shared move preamble (C `XPSAxis::move`, before the `GroupMove*` call):
    /// auto-enable a disabled axis, then set the S-gamma velocity/acceleration
    /// profile — both on the poll socket.
    fn prepare_move(
        &self,
        ctrl: &XpsController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let sock = ctrl.poll_socket();
        // Disabled state (20..=36): auto-enable, or refuse the move.
        if (20..=36).contains(&self.axis_status) {
            if ctrl.auto_enable() {
                sock.group_motion_enable(&self.group_name)?;
            } else {
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!(
                        "XPS axis {} is disabled and auto-enable is off",
                        self.positioner_name
                    ),
                });
            }
        }
        sock.positioner_sgamma_parameters_set(
            &self.positioner_name,
            velocity * self.step_size,
            acceleration * self.step_size,
            self.min_jerk,
            self.max_jerk,
        )?;
        Ok(())
    }
}

/// The group name is the prefix of `group.positioner` before the first `.`
/// (C `XPSAxis` constructor terminates `groupName_` at the `.`).
fn group_of(positioner: &str) -> &str {
    positioner.split('.').next().unwrap_or(positioner)
}

/// Decode the hardware end-of-run limits from a `PositionerErrorGet` bitmask
/// (`XPSAxis::poll`): `(high_limit, low_limit)`. Faithful to C's `error & mask`
/// test — because both masks include bit 31, a code with only bit 31 set (or
/// either real end-of-run code) reports *both* limits.
fn limits_from_error(error: u32) -> (bool, bool) {
    (
        error & XPSC8_END_OF_RUN_PLUS != 0,
        error & XPSC8_END_OF_RUN_MINUS != 0,
    )
}

/// Motor status bits derived purely from an XPS group status code, factored out
/// for testing against the C boundary values (`XPSAxis::poll`). Assumes
/// `referencingMode == 0` (the standard-home case; the move-to-home referencing
/// mode is not modeled in the core layer).
#[derive(Debug, PartialEq, Eq)]
struct StatusFlags {
    /// Group is moving/homing/jogging (43..=48) → `!done`.
    group_moving: bool,
    /// Encoder home switch active (ATHM).
    encoder_home: bool,
    /// Axis has been homed/referenced.
    homed: bool,
    /// Power is on / closed loop active.
    powered: bool,
    /// Axis cannot move (disabled / uninitialised / not referenced).
    problem: bool,
}

impl StatusFlags {
    fn from_status(status: i32) -> Self {
        StatusFlags {
            group_moving: (43..=48).contains(&status),
            encoder_home: status == XPS_STATUS_HOMING_DONE,
            homed: (10..=21).contains(&status) || status == 44 || status == 45 || status == 47,
            powered: (10..=19).contains(&status)
                || (43..=49).contains(&status)
                || matches!(status, 56 | 64 | 68 | 70 | 77 | 79),
            problem: status < 10 || (20..=42).contains(&status) || status == 50 || status == 64,
        }
    }
}

impl AsynMotor for XpsAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let device_units = position * self.step_size;
        {
            let ctrl = self.lock_controller();
            self.prepare_move(&ctrl, velocity, acceleration)?;
        }
        tolerate_dir_change(
            self.move_sock
                .group_move_absolute(&self.positioner_name, device_units),
        )
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C `move()` has a native relative branch (GroupMoveRelative); override
        // the trait's poll-then-absolute default.
        let device_units = distance * self.step_size;
        {
            let ctrl = self.lock_controller();
            self.prepare_move(&ctrl, velocity, acceleration)?;
        }
        tolerate_dir_change(
            self.move_sock
                .group_move_relative(&self.positioner_name, device_units),
        )
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        {
            let ctrl = self.lock_controller();
            ctrl.poll_socket().group_jog_mode_enable(&self.group_name)?;
        }
        self.move_sock.group_jog_parameters_set(
            &self.positioner_name,
            velocity * self.step_size,
            acceleration * self.step_size,
        )?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // The XPS home search takes no direction; C ignores `forwards`.
        {
            let ctrl = self.lock_controller();
            let sock = ctrl.poll_socket();
            // A Ready group (10..=18) will refuse home; kill it first.
            let status = sock.group_status_get(&self.group_name)?;
            if (10..=18).contains(&status) {
                sock.group_kill(&self.group_name)?;
            }
            // If not initialized, initialize it.
            let status = sock.group_status_get(&self.group_name)?;
            if (0..=9).contains(&status) || status == 50 || status == 63 {
                sock.group_initialize(&self.group_name)?;
            }
        }
        self.move_sock.group_home_search(&self.group_name)?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // A jog is stopped differently from a move, so read the status first.
        let status = {
            let ctrl = self.lock_controller();
            ctrl.poll_socket().group_status_get(&self.group_name)?
        };
        self.axis_status = status;
        if status == 44 || status == 45 || status == 47 {
            self.move_sock.group_move_abort(&self.group_name)?;
        }
        if status == 43 {
            self.move_sock.group_kill(&self.group_name)?;
        }
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        if !ctrl.enable_set_position() {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "XPS set position is disabled (enableSetPosition=0)".into(),
            });
        }
        if ctrl.axes_in_group(&self.group_name) > 1 {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!(
                    "XPS set position for multi-axis group '{}' is not supported",
                    self.group_name
                ),
            });
        }
        ctrl.set_position(
            &self.positioner_name,
            &self.group_name,
            position * self.step_size,
        )?;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        let sock = ctrl.poll_socket();
        if enable {
            sock.group_motion_enable(&self.group_name)?;
        } else {
            sock.group_motion_disable(&self.group_name)?;
        }
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        corrector::set_pid(ctrl.poll_socket(), &self.positioner_name, kind, gain)?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Clone the Arc into a local so the lock guard does not borrow `self`
        // (we mutate `self.axis_status` after the reads).
        let controller = self.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let sock = ctrl.poll_socket();

        let axis_status = sock.group_status_get(&self.group_name)?;
        let encoder = sock.group_position_current_get(&self.positioner_name)?;
        let setpoint = sock.group_position_setpoint_get(&self.positioner_name)?;
        let positioner_error = sock.positioner_error_get(&self.positioner_name)?;
        let velocity = sock.group_velocity_current_get(&self.positioner_name)?;
        drop(ctrl);

        self.axis_status = axis_status;

        let flags = StatusFlags::from_status(axis_status);
        let moving = velocity.abs() > XPS_VELOCITY_DEADBAND;
        let (high_limit, low_limit) = limits_from_error(positioner_error);

        Ok(MotorStatus {
            position: setpoint / self.step_size,
            encoder_position: encoder / self.step_size,
            velocity: velocity / self.step_size,
            // Motion-done is the group-status view; the separate `moving` flag
            // is velocity-derived (C `motorStatusDone_` vs `motorStatusMoving_`).
            done: !flags.group_moving,
            moving,
            high_limit,
            low_limit,
            encoder_home: flags.encoder_home,
            homed: flags.homed,
            powered: flags.powered,
            problem: flags.problem,
            direction: velocity > XPS_VELOCITY_DEADBAND,
            gain_support: true,
            has_encoder: true,
            // The XPS move ignores base velocity (min_velocity); VBAS unused.
            vbas_supported: false,
            ..Default::default()
        })
    }
}

/// C `move()` tolerates error `-27` (raised when the motor record reverses
/// direction, aborting an in-flight move); every other error propagates.
fn tolerate_dir_change(result: Result<(), XpsError>) -> AsynResult<()> {
    match result {
        Ok(()) => Ok(()),
        Err(XpsError::Api(-27)) => Ok(()),
        Err(e) => Err(e.into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn group_name_is_prefix_before_dot() {
        assert_eq!(group_of("GROUP1.POSITIONER"), "GROUP1");
        assert_eq!(group_of("XY.X"), "XY");
        // No dot: whole string is the group (C leaves it unterminated).
        assert_eq!(group_of("BARE"), "BARE");
    }

    #[test]
    fn status_flags_moving_range() {
        // 43..=48 are moving/homing/jogging.
        for s in 43..=48 {
            assert!(StatusFlags::from_status(s).group_moving, "status {s}");
        }
        assert!(!StatusFlags::from_status(42).group_moving);
        assert!(!StatusFlags::from_status(49).group_moving);
    }

    #[test]
    fn status_flags_homed_and_home() {
        // Ready/enabled range and the specific moving-but-referenced codes.
        assert!(StatusFlags::from_status(10).homed);
        assert!(StatusFlags::from_status(21).homed);
        assert!(StatusFlags::from_status(44).homed);
        assert!(StatusFlags::from_status(47).homed);
        assert!(!StatusFlags::from_status(9).homed);
        assert!(!StatusFlags::from_status(22).homed);
        assert!(!StatusFlags::from_status(46).homed);
        // Encoder home (ATHM) only at status 11.
        assert!(StatusFlags::from_status(11).encoder_home);
        assert!(!StatusFlags::from_status(10).encoder_home);
    }

    #[test]
    fn status_flags_powered() {
        assert!(StatusFlags::from_status(10).powered);
        assert!(StatusFlags::from_status(19).powered);
        assert!(StatusFlags::from_status(43).powered);
        assert!(StatusFlags::from_status(49).powered);
        assert!(StatusFlags::from_status(56).powered);
        assert!(StatusFlags::from_status(79).powered);
        assert!(!StatusFlags::from_status(20).powered);
        assert!(!StatusFlags::from_status(50).powered);
    }

    #[test]
    fn status_flags_problem() {
        // Uninitialised / disabled / not-referenced states.
        assert!(StatusFlags::from_status(0).problem);
        assert!(StatusFlags::from_status(9).problem);
        assert!(StatusFlags::from_status(20).problem);
        assert!(StatusFlags::from_status(42).problem);
        assert!(StatusFlags::from_status(50).problem);
        assert!(StatusFlags::from_status(64).problem);
        // Ready and moving states are not a problem.
        assert!(!StatusFlags::from_status(10).problem);
        assert!(!StatusFlags::from_status(43).problem);
    }

    #[test]
    fn limits_from_error_masks() {
        // No error → neither limit.
        assert_eq!(limits_from_error(0), (false, false));
        // A real positive/negative end-of-run code matches its own mask; both
        // masks share bit 31, so C's `err & mask` reports BOTH limits for a real
        // end-of-run code (or a bit-31-only code). We reproduce that exactly.
        assert_eq!(limits_from_error(0x8000_0200), (true, true));
        assert_eq!(limits_from_error(0x8000_0100), (true, true));
        assert_eq!(limits_from_error(0x8000_0000), (true, true));
        // An unrelated low-bit error touches neither limit.
        assert_eq!(limits_from_error(0x0000_0004), (false, false));
    }
}
