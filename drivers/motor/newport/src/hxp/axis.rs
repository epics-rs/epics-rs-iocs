//! HXP axis: one hexapod coordinate (X/Y/Z/U/V/W) exposed as an asyn
//! [`AsynMotor`].
//!
//! Port of `HXPAxis` (`HXPDriver.cpp`). Every "axis" move is really a
//! six-coordinate hexapod move: the absolute branch reads all six current
//! positions (poll socket), replaces this axis's target, and sends
//! `HexapodMoveAbsolute`; the relative branch sends `HexapodMoveIncremental`
//! with only this axis's delta. Moves go out the axis's own `Fire`-mode move
//! socket (C's `-0.1` timeout); status comes from the controller's group
//! poll.
//!
//! The hexapod plans its own trajectory, so velocity/acceleration arguments
//! are ignored (C never sends them).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PortSets};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::user::AsynUser;

use super::controller::{HXP_GROUP, HxpController, MRES, MoveCoordSys};
use crate::xps::rpc::{XpsError, XpsResult, XpsSocket};

/// C's longest `errorDesc_`, the `HXP_ERROR_DESC` a record shows.
const ERROR_DESC_LEN: usize = 39;

/// The `int` a C HXP wrapper returns for `result`.
pub(super) fn xps_status(result: &XpsResult<()>) -> i32 {
    result.as_ref().err().map_or(0, XpsError::code)
}

/// An axis's move socket and C `errorDescFull_`, which its moves and the
/// controller's parameter writes on its address share.
pub struct MoveLine {
    sock: XpsSocket,
    desc_full: Mutex<String>,
}

impl MoveLine {
    pub fn new(sock: XpsSocket) -> Self {
        Self {
            sock,
            desc_full: Mutex::default(),
        }
    }

    pub fn sock(&self) -> &XpsSocket {
        &self.sock
    }

    /// `HXP_ERROR` and `HXP_ERROR_DESC` for `status`: a failure's
    /// ErrorStringGet text on this socket cut to 39 bytes, or both cleared
    /// (HXPDriver.cpp:193-215, :350-371).
    pub fn error_values(&self, status: i32) -> (i32, Vec<u8>) {
        if status >= 0 {
            return (0, Vec::new());
        }
        let mut full = self.desc_full.lock().unwrap_or_else(|e| e.into_inner());
        self.sock.error_string_get_into(status, &mut full);
        let bytes = full.as_bytes();
        (status, bytes[..bytes.len().min(ERROR_DESC_LEN)].to_vec())
    }
}

/// One hexapod coordinate as an asyn motor axis.
pub struct HxpAxis {
    controller: Arc<Mutex<HxpController>>,
    line: Arc<MoveLine>,
    /// 0..=5 → X, Y, Z, U, V, W.
    axis_no: usize,
    /// The controller-port steps of the last move, for the owner to apply.
    sets: PortSets,
}

impl HxpAxis {
    pub fn new(controller: Arc<Mutex<HxpController>>, line: Arc<MoveLine>, axis_no: usize) -> Self {
        Self {
            controller,
            line,
            axis_no,
            sets: PortSets::default(),
        }
    }

    fn lock_controller(&self) -> MutexGuard<'_, HxpController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `move()`'s close: the error at the controller's address 0, flushed
    /// at this axis's, so another axis's shows at the next controller poll.
    /// `-27` (the record reversing, which aborts the move) is posted but
    /// not failed.
    fn finish_move(&mut self, result: XpsResult<()>) -> AsynResult<()> {
        let status = xps_status(&result);
        let (code, desc) = self.line.error_values(status);
        let params = self.lock_controller().params;
        if let Some(p) = params {
            self.sets = HxpController::post(
                self.axis_no as i32,
                vec![
                    (p.error, ParamValue::Int32(code)),
                    (p.error_desc, ParamValue::Octet(desc.into())),
                ],
            );
        }
        match result {
            Ok(()) | Err(XpsError::Api(-27)) => Ok(()),
            Err(e) => Err(e.into()),
        }
    }
}

impl AsynMotor for HxpAxis {
    /// C `HXPController::poll`'s `done` label sets `motorStatusProblem_` and
    /// `motorStatusCommsError_` on every axis.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM_AND_COMMS_ERROR
    }

    fn take_port_sets(&mut self) -> PortSets {
        std::mem::take(&mut self.sets)
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let end_pos = position * MRES;
        // C ignores HXPGroupPositionCurrentGet's status and moves from its
        // uninitialised `cur_pos` (HXPDriver.cpp:626); not copied, a failed
        // read refuses the move.
        let (coord_sys, current) = {
            let ctrl = self.lock_controller();
            (ctrl.move_coord_sys(), ctrl.current_positions()?)
        };
        match coord_sys {
            MoveCoordSys::Work => {
                let mut targets = current;
                targets[self.axis_no] = end_pos;
                self.finish_move(
                    self.line
                        .sock
                        .hexapod_move_absolute(HXP_GROUP, "Work", &targets),
                )
            }
            // Tool coordinates have no absolute form: C converts to a
            // relative move from the current position.
            MoveCoordSys::Tool => {
                let mut deltas = [0.0; 6];
                deltas[self.axis_no] = end_pos - current[self.axis_no];
                self.finish_move(
                    self.line
                        .sock
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
        deltas[self.axis_no] = distance * MRES;
        self.finish_move(
            self.line
                .sock
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
        let _ = self.line.sock.group_kill(HXP_GROUP);
        let _ = self.line.sock.group_initialize(HXP_GROUP);
        let _ = self.line.sock.group_home_search(HXP_GROUP);
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C `stop()`: abort the group move, return code ignored.
        let _ = self.line.sock.group_move_abort(HXP_GROUP);
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C `HXPAxis` inherits asynMotorAxis::setPosition: no-op, asynSuccess.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock_controller();
        let sock = ctrl.poll_socket();
        if enable {
            sock.group_motion_enable(HXP_GROUP)
        } else {
            sock.group_motion_disable(HXP_GROUP)
        }
        .map_err(XpsError::into_c_status)?;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // C `HXPAxis::poll` reads what the controller poll before it read.
        let data = self.lock_controller().data().clone();
        Ok(MotorStatus {
            position: data.setpoint[self.axis_no] / MRES,
            encoder_position: data.encoder[self.axis_no] / MRES,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xps::fake::FakeLine;
    use crate::xps::rpc::SocketMode;

    /// HXPAxis inherits asynMotorAxis::setPosition, which sends nothing and
    /// returns asynSuccess (asynMotorAxis.cpp:130).
    #[test]
    fn set_position_is_a_successful_no_op() {
        let line = FakeLine::new("hxp_setpos", "");
        let controller = Arc::new(Mutex::new(HxpController::new(
            line.socket(SocketMode::Query),
        )));
        let mut axis = HxpAxis::new(
            controller,
            Arc::new(MoveLine::new(line.socket(SocketMode::Fire))),
            0,
        );
        line.log.lock().unwrap().clear();
        axis.set_position(&AsynUser::new(0), 5.0).unwrap();
        assert!(line.log.lock().unwrap().is_empty());
    }

    /// setClosedLoop returns the XPS code as its asynStatus
    /// (HXPDriver.cpp:742).
    #[test]
    fn a_closed_loop_failure_returns_the_xps_code() {
        for (call, enable) in [("GroupMotionEnable", true), ("GroupMotionDisable", false)] {
            let line = FakeLine::new(&format!("hxp_code_{call}"), call);
            let controller = Arc::new(Mutex::new(HxpController::new(
                line.socket(SocketMode::Query),
            )));
            let mut axis = HxpAxis::new(
                controller,
                Arc::new(MoveLine::new(line.socket(SocketMode::Fire))),
                0,
            );
            let e = axis.set_closed_loop(&AsynUser::new(0), enable).unwrap_err();
            assert_eq!(e.c_status_code(), -3, "{call}");
        }
    }

    /// HXPDriver.cpp:605, 631, 766, 769: record steps times MRES go out,
    /// and the readbacks come back divided by it.
    #[test]
    fn record_steps_scale_by_mres() {
        let line = FakeLine::new("hxp_steps", "");
        let controller = Arc::new(Mutex::new(HxpController::new(
            line.socket(SocketMode::Query),
        )));
        let mut axis = HxpAxis::new(
            controller,
            Arc::new(MoveLine::new(line.socket(SocketMode::Fire))),
            0,
        );
        let user = AsynUser::new(0);
        axis.move_absolute(&user, 2e5, 0.0, 1.0, 1.0).unwrap();
        axis.move_relative(&user, -1e5, 0.0, 1.0, 1.0).unwrap();
        let log = std::mem::take(&mut *line.log.lock().unwrap());
        assert!(
            log.iter()
                .any(|c| c == "HexapodMoveAbsolute (HEXAPOD,Work,2,0,0,0,0,0)"),
            "{log:?}"
        );
        assert!(
            log.iter()
                .any(|c| c == "HexapodMoveIncremental (HEXAPOD,Work,-1,0,0,0,0,0)"),
            "{log:?}"
        );
        // The fake reports 1.5 for the current position.
        axis.lock_controller().poll();
        let status = axis.poll(&user).unwrap();
        assert_eq!(
            (status.position, status.encoder_position),
            (150000.0, 150000.0)
        );
    }
}
