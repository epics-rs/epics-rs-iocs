//! HXP axis: one hexapod coordinate (X/Y/Z/U/V/W) exposed as an asyn
//! [`AsynMotor`].
//!
//! Port of `HXPAxis` (`HXPDriver.cpp`). Every "axis" move is really a
//! six-coordinate hexapod move: the absolute branch reads all six current
//! positions (poll socket), replaces this axis's target, and sends
//! `HexapodMoveAbsolute`; the relative branch sends `HexapodMoveIncremental`
//! with only this axis's delta. Moves go out the shared `Fire`-mode move
//! socket (C gives each axis its own move socket with a `-0.1` timeout);
//! status comes from the controller's cached group poll.
//!
//! The hexapod plans its own trajectory, so velocity/acceleration arguments
//! are ignored (C never sends them).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::user::AsynUser;

use super::controller::{HXP_GROUP, HXP_MRES, HxpController, MoveCoordSys};
use crate::xps::rpc::{XpsError, XpsSocket};

/// One hexapod coordinate as an asyn motor axis.
pub struct HxpAxis {
    controller: Arc<Mutex<HxpController>>,
    move_sock: XpsSocket,
    /// 0..=5 → X, Y, Z, U, V, W.
    axis_no: usize,
}

impl HxpAxis {
    pub fn new(
        controller: Arc<Mutex<HxpController>>,
        move_sock: XpsSocket,
        axis_no: usize,
    ) -> Self {
        Self {
            controller,
            move_sock,
            axis_no,
        }
    }

    fn lock_controller(&self) -> MutexGuard<'_, HxpController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// C `move()` tolerates error `-27` (raised when the motor record reverses
/// direction, aborting an in-flight move); every other error propagates.
fn tolerate_dir_change(result: Result<(), XpsError>) -> AsynResult<()> {
    match result {
        Ok(()) | Err(XpsError::Api(-27)) => Ok(()),
        Err(e) => Err(e.into()),
    }
}

impl AsynMotor for HxpAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let end_pos = position * HXP_MRES;
        let (coord_sys, current) = {
            let ctrl = self.lock_controller();
            (ctrl.move_coord_sys(), ctrl.current_positions()?)
        };
        match coord_sys {
            MoveCoordSys::Work => {
                let mut targets = current;
                targets[self.axis_no] = end_pos;
                tolerate_dir_change(
                    self.move_sock
                        .hexapod_move_absolute(HXP_GROUP, "Work", &targets),
                )
            }
            // Tool coordinates have no absolute form: C converts to a
            // relative move from the current position.
            MoveCoordSys::Tool => {
                let mut deltas = [0.0; 6];
                deltas[self.axis_no] = end_pos - current[self.axis_no];
                tolerate_dir_change(
                    self.move_sock
                        .hexapod_move_incremental(HXP_GROUP, "Tool", &deltas),
                )
            }
        }
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let coord_sys = self.lock_controller().move_coord_sys();
        let cs_name = match coord_sys {
            MoveCoordSys::Work => "Work",
            MoveCoordSys::Tool => "Tool",
        };
        let mut deltas = [0.0; 6];
        deltas[self.axis_no] = distance * HXP_MRES;
        tolerate_dir_change(
            self.move_sock
                .hexapod_move_incremental(HXP_GROUP, cs_name, &deltas),
        )
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `home()`: kill, initialize, home search — return codes ignored.
        let _ = self.move_sock.group_kill(HXP_GROUP);
        let _ = self.move_sock.group_initialize(HXP_GROUP);
        let _ = self.move_sock.group_home_search(HXP_GROUP);
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C `stop()`: abort the group move, return code ignored.
        let _ = self.move_sock.group_move_abort(HXP_GROUP);
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C `HXPAxis` does not override setPosition; a hexapod coordinate
        // cannot be redefined per-axis.
        Err(AsynError::Status {
            status: AsynStatus::Error,
            message: "HXP set position is not supported".into(),
        })
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        let sock = ctrl.poll_socket();
        if enable {
            sock.group_motion_enable(HXP_GROUP)?;
        } else {
            sock.group_motion_disable(HXP_GROUP)?;
        }
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let data = self.lock_controller().poll_data();
        Ok(MotorStatus {
            position: data.setpoint[self.axis_no] / HXP_MRES,
            encoder_position: data.encoder[self.axis_no] / HXP_MRES,
            done: !data.moving,
            moving: data.moving,
            problem: data.problem,
            comms_error: data.comms_error,
            powered: data.powered,
            homed: data.homed,
            // CNEN enables/disables the hexapod; it reads stage encoders.
            gain_support: true,
            has_encoder: true,
            // The hexapod plans its own trajectory; base velocity is ignored.
            vbas_supported: false,
            ..Default::default()
        })
    }
}
