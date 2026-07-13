//! Coordinate-system axes, ported from `pmacApp/pmacAsynCoordSrc/pmacAsynCoord.c`.
//!
//! A PMAC coordinate system runs kinematics: the nine CS axes A, B, C, U, V, W,
//! X, Y, Z are the *kinematic* coordinates, and the PMAC's own motion program
//! resolves them onto the real motors. The IOC drives one CS axis per motor
//! record:
//!
//! - the demand goes into `Q7{n}` (n = 1..9, A = 1),
//! - a position-reporting PLC on the controller publishes the readbacks in
//!   `Q8{n}`,
//! - `&{cs}??` returns the three CS status words,
//! - a move is started by running the motion program (`B{prog}R`) in the CS,
//!   after an abort (`&{cs}A`) that re-closes the loops.
//!
//! The C file is a *model-2* driver (`motorAxisDrvSET_t`, wired up by
//! `drvAsynMotorConfigure`) with its own polling thread; this port is model-3
//! ([`AsynMotor`]) like the real-axis driver, so the poll loop comes from
//! motor-rs. Everything on the wire is unchanged.
//!
//! ## Units (deviation, deliberate)
//!
//! C multiplies every demand by `stepSize` (default 1e-4, i.e. 10000 steps per
//! EGU) and divides every readback by it, with `MRES` set to the reciprocal —
//! the same raw-steps round trip the real-axis driver does with `scale_`, and it
//! cancels at the record boundary for the same reason (see [`crate::axis`]).
//! This port speaks the PLC's EGU directly, so `pmacSetCoordStepsPerUnit` /
//! `pmacSetDefaultCoordSteps` are not provided and the records take `MRES = 1`.
//!
//! ## Not supported by the hardware path
//!
//! `home` and `move_velocity` return an error, as they do in C: a CS axis is a
//! kinematic coordinate, and neither a limit-switch seek nor an open-ended jog
//! is defined for one.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::controller::{octet_write_read, pmac_err};
use crate::protocol::{
    CS_STATUS1_RUNNING_PROG, CS_STATUS2_AMP_FAULT, CS_STATUS2_FOLLOW_ERR, CS_STATUS2_IN_POSITION,
    CS_STATUS2_RUNTIME_ERR, CS_STATUS3_LIMIT, parse_cs_positions, parse_cs_status,
};

/// The nine kinematic axes of a coordinate system (C `NAXES`).
pub const CS_AXES: usize = 9;

/// C `axisName[]`: the CS axis letters, indexed 1..=9.
const AXIS_NAMES: [char; CS_AXES] = ['A', 'B', 'C', 'U', 'V', 'W', 'X', 'Y', 'Z'];

/// The letter of a 1-based CS axis number, for messages.
pub fn axis_name(axis: usize) -> Option<char> {
    AXIS_NAMES.get(axis.checked_sub(1)?).copied()
}

/// One poll of the whole coordinate system: the three status words and the nine
/// `Q8{n}` readbacks. C polls all nine axes in one thread pass; a model-3 driver
/// polls per axis, so the first axis of a cycle reads the system and the other
/// eight reuse that read while it is fresh — same traffic on the wire.
#[derive(Debug, Clone)]
struct CsPoll {
    status: (u32, u32, u32),
    positions: Vec<f64>,
    read_at: Instant,
}

/// The coordinate system itself: C `drvPmac_t`, one per
/// `pmacAsynCoordCreate`.
pub struct PmacCoordSystem {
    handle: SyncIOHandle,
    /// C `cs`, the coordinate-system number (1..=16).
    cs: i32,
    /// C `program`: the motion program a move runs. Zero means "an external
    /// process starts the motion" — the demand is written and nothing is run.
    program: i32,
    moves_deferred: bool,
    /// Which CS axes have a demand written but not yet run (C
    /// `motorAxis::deferred_move`).
    deferred: [bool; CS_AXES],
    poll: Option<CsPoll>,
    poll_ttl: Duration,
}

impl PmacCoordSystem {
    pub fn new(handle: SyncIOHandle, cs: i32, program: i32, poll_ttl: Duration) -> Self {
        Self {
            handle,
            cs,
            program,
            moves_deferred: false,
            deferred: [false; CS_AXES],
            poll: None,
            poll_ttl,
        }
    }

    pub fn coord_system(&self) -> i32 {
        self.cs
    }

    fn command(&self, command: &str) -> AsynResult<()> {
        octet_write_read(&self.handle, command).map(|_| ())
    }

    /// C `drvPmacGetAxesStatus`: `&{cs}??` for the status words, then one
    /// command carrying all nine `Q8{n}` readbacks.
    fn refresh(&mut self) -> AsynResult<CsPoll> {
        let cs = self.cs;
        let response = octet_write_read(&self.handle, &format!("&{cs}??"))?;
        let status = parse_cs_status(&response)
            .ok_or_else(|| pmac_err(format!("could not parse CS {cs} status: {response:?}")))?;

        let mut query = format!("&{cs}");
        for axis in 1..=CS_AXES {
            query.push_str(&format!("Q8{axis}"));
        }
        let response = octet_write_read(&self.handle, &query)?;
        let positions = parse_cs_positions(&response, CS_AXES);
        if positions.len() != CS_AXES {
            return Err(pmac_err(format!(
                "CS {cs} returned {} of {CS_AXES} readbacks: {response:?}",
                positions.len()
            )));
        }

        let poll = CsPoll {
            status,
            positions,
            read_at: Instant::now(),
        };
        self.poll = Some(poll.clone());
        Ok(poll)
    }

    fn poll_system(&mut self) -> AsynResult<CsPoll> {
        match &self.poll {
            Some(poll) if poll.read_at.elapsed() < self.poll_ttl => Ok(poll.clone()),
            _ => self.refresh(),
        }
    }

    /// C `processDeferredMoves`: abort (to re-enable the motors), then run the
    /// motion program once for the whole coordinate system.
    fn process_deferred_moves(&mut self) -> AsynResult<()> {
        let cs = self.cs;
        let program = self.program;
        self.deferred = [false; CS_AXES];
        self.command(&format!("&{cs}A"))?;
        self.command(&format!("&{cs}AB{program}R"))
    }
}

pub struct PmacCsAxis {
    system: Arc<Mutex<PmacCoordSystem>>,
    /// 1-based index into the coordinate system: 1 = A … 9 = Z.
    axis: usize,
    previous_position: f64,
    previous_direction: bool,
}

fn lock(system: &Arc<Mutex<PmacCoordSystem>>) -> std::sync::MutexGuard<'_, PmacCoordSystem> {
    system
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

impl PmacCsAxis {
    /// `axis` is 1-based (1 = A, 9 = Z).
    pub fn new(system: Arc<Mutex<PmacCoordSystem>>, axis: usize) -> Result<Self, String> {
        if !(1..=CS_AXES).contains(&axis) {
            return Err(format!(
                "coordinate-system axis must be 1..={CS_AXES} (A..Z), got {axis}"
            ));
        }
        Ok(Self {
            system,
            axis,
            previous_position: 0.0,
            previous_direction: false,
        })
    }
}

impl AsynMotor for PmacCsAxis {
    /// C `motorAxisMove`: set the CS feed rate (`I{cs+50}89`, EGU/s) and
    /// acceleration time (`I{cs+50}87`, msec), write the demand into `Q7{n}`,
    /// and — unless moves are deferred or the CS has no program — abort and run
    /// the motion program.
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let axis = self.axis;
        let mut system = lock(&self.system);
        let cs = system.cs;
        let csvar = cs + 50;

        let mut command = format!("&{cs}");
        if velocity != 0.0 {
            command.push_str(&format!("I{csvar}89={:.6} ", velocity.abs()));
            if acceleration != 0.0 {
                command.push_str(&format!(
                    "I{csvar}87={:.6} ",
                    (velocity / acceleration).abs() * 1000.0
                ));
            }
        }
        command.push_str(&format!("Q7{axis}={position:.12}"));

        if system.moves_deferred {
            system.deferred[axis - 1] = true;
        } else if system.program != 0 {
            // Abort first so the motors are back in closed loop; then append the
            // program run to the demand, so the demand and its start are one
            // command.
            system.command(&format!("&{cs}A"))?;
            command.push_str(&format!("B{}R", system.program));
        }
        system.command(&command)
    }

    /// **Upstream fix.** C `motorAxisMove` takes a `relative` argument and never
    /// looks at it: a relative demand was written into `Q7{n}` as if it were
    /// absolute, so a REL move on a CS axis drove to the wrong place (from the
    /// origin instead of from the current position). The trait's default
    /// `move_relative` polls the readback and adds the distance, which is what
    /// the CS demand register needs; spelling it out here so the fix is visible
    /// rather than inherited.
    fn move_relative(
        &mut self,
        user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let status = self.poll(user)?;
        self.move_absolute(
            user,
            status.position + distance,
            min_velocity,
            velocity,
            acceleration,
        )
    }

    /// C `motorAxisHome`: a kinematic coordinate has no home switch.
    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        Err(pmac_err(
            "home is not implemented for coordinate-system axes",
        ))
    }

    /// C `motorAxisVelocityMove`: likewise, no open-ended jog in a CS.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        Err(pmac_err(
            "jog (velocity move) is not implemented for coordinate-system axes",
        ))
    }

    /// C `motorAxisStop`: abort the coordinate system and park the demand on the
    /// readback, so the CS does not resume the interrupted move.
    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let axis = self.axis;
        let mut system = lock(&self.system);
        let cs = system.cs;
        system.deferred[axis - 1] = false;
        system.command(&format!("&{cs}A Q7{axis}=Q8{axis}"))
    }

    /// C `motorAxisSetDouble`, `motorAxisPosition`: there is nothing to redefine
    /// on a kinematic axis — the readback comes from the CS PLC — so C only
    /// updates the demand register, keeping it consistent with the readback.
    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let axis = self.axis;
        let system = lock(&self.system);
        let cs = system.cs;
        system.command(&format!("&{cs}Q7{axis}={position:.6}"))
    }

    /// C `motorAxisSetDouble`, `motorAxisDeferMoves`: releasing the deferral runs
    /// the motion program once for the whole coordinate system, starting every
    /// axis whose demand was written while it was armed.
    fn set_deferred_moves(&mut self, _user: &AsynUser, defer: bool) -> AsynResult<()> {
        let mut system = lock(&self.system);
        let release = !defer && system.moves_deferred;
        system.moves_deferred = defer;
        if release {
            system.process_deferred_moves()?;
        }
        Ok(())
    }

    /// C `drvPmacGetAxesStatus`, for this axis's share of the system poll.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let axis = self.axis;
        let mut system = lock(&self.system);
        let poll = system.poll_system()?;
        let (status1, status2, status3) = poll.status;
        let position = poll.positions[axis - 1];

        let direction = if position > self.previous_position {
            true
        } else if position == self.previous_position {
            self.previous_direction
        } else {
            false
        };
        self.previous_position = position;
        self.previous_direction = direction;

        let flags = decode_cs_status((status1, status2, status3), system.deferred[axis - 1]);

        Ok(MotorStatus {
            position,
            encoder_position: position,
            done: flags.done,
            moving: flags.moving,
            high_limit: flags.limit,
            low_limit: flags.limit,
            direction,
            slip_stall: flags.following_error,
            problem: flags.problem,
            has_encoder: true,
            ..MotorStatus::default()
        })
    }
}

/// What the three coordinate-system status words mean for one axis
/// (C `drvPmacGetAxesStatus`, the `motorParam->setInteger` block).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CsFlags {
    done: bool,
    moving: bool,
    /// The CS status carries one limit bit for the whole system and does not say
    /// which end it is; C reports it as both the high and the low hard limit,
    /// and so does this.
    limit: bool,
    following_error: bool,
    problem: bool,
}

fn decode_cs_status(status: (u32, u32, u32), deferred: bool) -> CsFlags {
    let (status1, status2, status3) = status;
    let in_position = status2 & CS_STATUS2_IN_POSITION != 0;
    CsFlags {
        // A deferred axis has a demand written but no program running yet.
        done: !deferred && status1 & CS_STATUS1_RUNNING_PROG == 0 && in_position,
        moving: !in_position,
        limit: status3 & CS_STATUS3_LIMIT != 0,
        following_error: status2 & CS_STATUS2_FOLLOW_ERR != 0,
        // **Upstream fix.** C sets motorAxisProblem twice in a row — first from
        // CS_STATUS2_AMP_FAULT, then from CS_STATUS2_RUNTIME_ERR — so the
        // amplifier-fault bit was overwritten every poll and an amp fault never
        // reached the record. Both are faults; report either.
        problem: status2 & (CS_STATUS2_AMP_FAULT | CS_STATUS2_RUNTIME_ERR) != 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_names_are_the_kinematic_letters() {
        assert_eq!(axis_name(1), Some('A'));
        assert_eq!(axis_name(4), Some('U'));
        assert_eq!(axis_name(9), Some('Z'));
        assert_eq!(axis_name(0), None);
        assert_eq!(axis_name(10), None);
    }

    #[test]
    fn in_position_with_no_program_running_is_done() {
        let flags = decode_cs_status((0, CS_STATUS2_IN_POSITION, 0), false);
        assert!(flags.done);
        assert!(!flags.moving);
    }

    #[test]
    fn a_running_program_is_not_done_even_in_position() {
        // The kinematics can pass through the demand and keep going: the program
        // is the authority on whether the move has finished.
        let flags = decode_cs_status((CS_STATUS1_RUNNING_PROG, CS_STATUS2_IN_POSITION, 0), false);
        assert!(!flags.done);
    }

    #[test]
    fn out_of_position_is_moving_and_not_done() {
        let flags = decode_cs_status((CS_STATUS1_RUNNING_PROG, 0, 0), false);
        assert!(!flags.done);
        assert!(flags.moving);
    }

    #[test]
    fn a_deferred_axis_is_never_done() {
        let flags = decode_cs_status((0, CS_STATUS2_IN_POSITION, 0), true);
        assert!(!flags.done);
    }

    #[test]
    fn the_single_limit_bit_lands_on_both_ends() {
        let flags = decode_cs_status((0, CS_STATUS2_IN_POSITION, CS_STATUS3_LIMIT), false);
        assert!(flags.limit);
    }

    #[test]
    fn both_amp_fault_and_runtime_error_raise_problem() {
        // The upstream defect: C's second write overwrote the first, so only the
        // runtime-error bit ever reached the record.
        assert!(decode_cs_status((0, CS_STATUS2_AMP_FAULT, 0), false).problem);
        assert!(decode_cs_status((0, CS_STATUS2_RUNTIME_ERR, 0), false).problem);
        assert!(!decode_cs_status((0, CS_STATUS2_FOLLOW_ERR, 0), false).problem);
        assert!(decode_cs_status((0, CS_STATUS2_FOLLOW_ERR, 0), false).following_error);
    }
}
