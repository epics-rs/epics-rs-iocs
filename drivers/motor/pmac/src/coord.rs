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
//! ## Units
//!
//! The record boundary is raw steps, as in C: every demand and feed rate is
//! `steps * stepSize` (default 1e-4, i.e. 10000 steps per EGU, set by
//! `pmacSetDefaultCoordSteps` / `pmacSetCoordStepsPerUnit`) and every readback
//! is `Q8{n} / stepSize`, so the record's `MRES` is `stepSize`.
//!
//! C's direction test compares the new readback in steps with the previous one
//! scaled back to EGU (pmacAsynCoord.c:649-655); this port compares steps with
//! steps.
//!
//! ## Not supported by the hardware path
//!
//! `home` and `move_velocity` return an error, as they do in C: a CS axis is a
//! kinematic coordinate, and neither a limit-switch seek nor an open-ended jog
//! is defined for one.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;
use motor_common::model2::AxisLog;

use crate::controller::{PMAC_MAXBUF, checked_reply, pmac_err};
use crate::protocol::{
    CS_STATUS1_RUNNING_PROG, CS_STATUS2_AMP_FAULT, CS_STATUS2_FOLLOW_ERR, CS_STATUS2_IN_POSITION,
    CS_STATUS2_RUNTIME_ERR, CS_STATUS3_LIMIT, parse_cs_positions, scan_cs_status,
};

/// The nine kinematic axes of a coordinate system (C `NAXES`).
pub const CS_AXES: usize = 9;

/// C `axisName[]`: the CS axis letters, indexed 1..=9.
const AXIS_NAMES: [char; CS_AXES] = ['A', 'B', 'C', 'U', 'V', 'W', 'X', 'Y', 'Z'];

/// C `drvPmacCreate`'s `stepSize` for every axis: 10000 steps per EGU.
const DEFAULT_STEP_SIZE: f64 = 0.0001;

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
    /// C `drvPmacTask`'s `status[3]`: the words a failed read leaves.
    status: [u32; 3],
    poll_ttl: Duration,
    /// C `motorAxis::stepSize` of each CS axis: EGU per record step.
    step_size: [f64; CS_AXES],
    /// C `pDrv->pasynUser`'s trace, which every axis's `pasynUser` shares
    /// (pmacAsynCoord.c:812).
    trace: MotorTrace,
    /// C `pDrv->ref`, the `pmacAsynCoordCreate` reference.
    reference: i32,
    /// C `motorAxis::logParam` of each CS axis.
    logs: [Arc<AxisLog>; CS_AXES],
}

impl PmacCoordSystem {
    pub fn new(
        handle: SyncIOHandle,
        trace: MotorTrace,
        reference: i32,
        cs: i32,
        program: i32,
        poll_ttl: Duration,
    ) -> Self {
        Self {
            handle,
            trace,
            reference,
            cs,
            program,
            moves_deferred: false,
            deferred: [false; CS_AXES],
            poll: None,
            status: [0; 3],
            poll_ttl,
            step_size: [DEFAULT_STEP_SIZE; CS_AXES],
            logs: Default::default(),
        }
    }

    /// The axes' logs, A first, for `Model2::set_axis_logs`.
    pub fn logs(&self) -> Vec<Arc<AxisLog>> {
        self.logs.to_vec()
    }

    /// C `asynPrint(pAxis->pasynUser, ASYN_TRACE_ERROR, line)`, then the
    /// error the C function returns.
    fn refuse(&self, line: &str) -> AsynError {
        self.trace
            .print(TraceMask::ERROR, 0, format_args!("{line}\n"));
        pmac_err(line)
    }

    /// C `pAxis->print(pAxis->logParam, TRACE_FLOW, ...)` for 1-based `axis`.
    fn flow(&self, axis: usize, args: std::fmt::Arguments<'_>) {
        self.logs[axis - 1].print(TraceMask::FLOW, args);
    }

    /// C `motorAxisSetDouble` for a value the CS cannot set: its FLOW line,
    /// then success.
    fn ignore(&self, axis: usize, verb: &str, what: &str, value: f64) -> AsynResult<()> {
        self.flow(
            axis,
            format_args!(
                "{verb} PMAC ref {}, cs {}, axis {} {what} ({:.6})\n",
                self.reference,
                self.cs,
                axis_name(axis).unwrap_or(' '),
                CFloat(value)
            ),
        );
        Ok(())
    }

    /// C `pAxis->stepSize = 1.0 / stepsPerUnit` for 0-based `axis`, which
    /// the iocsh commands have range-checked and kept positive.
    pub fn set_steps_per_unit(&mut self, axis: usize, steps_per_unit: f64) {
        self.step_size[axis] = 1.0 / steps_per_unit;
    }

    #[cfg(test)]
    pub(crate) fn step_size(&self, axis: usize) -> f64 {
        self.step_size[axis]
    }

    pub fn coord_system(&self) -> i32 {
        self.cs
    }

    /// The poll snapshot predates the command, so the next poll reads the
    /// system again (C signals its poller after a move).
    fn command(&mut self, axis: usize, command: &str) -> AsynResult<()> {
        self.poll = None;
        self.write_read(axis, command).map(|_| ())
    }

    /// C `motorAxisWriteRead` for 1-based `axis`: a failed transfer prints
    /// its line. C returns MOTOR_AXIS_OK on a refused (BELL) reply
    /// (pmacAsynCoord.c:345); that C defect is not copied (see the crate doc).
    fn write_read(&self, axis: usize, command: &str) -> AsynResult<String> {
        checked_reply(command, self.transfer(axis, command, false)?)
    }

    /// [`Self::write_read`] without the BELL check: the reply as C sees it.
    /// Only a `log_global` read leaves the axis's log quiet.
    fn transfer(&self, axis: usize, command: &str, log_global: bool) -> AsynResult<Vec<u8>> {
        let log = (!log_global).then(|| &self.logs[axis - 1]);
        let reference = self.reference;
        if let Some(log) = log {
            log.print(
                TraceMask::IO_DRIVER,
                format_args!("Sending to PMAC C.S. ref {reference} command : {command}\n"),
            );
        }
        let reply = self.handle.write_read(0, command.as_bytes(), PMAC_MAXBUF);
        if let (Some(log), Ok(raw)) = (log, &reply)
            && !raw.is_empty()
        {
            let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
            log.print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "Recvd from PMAC C.S. ref {reference} response: {}\n",
                    String::from_utf8_lossy(&raw[..end])
                ),
            );
        }
        reply.inspect_err(|e| {
            self.trace.print(
                TraceMask::ERROR,
                0,
                format_args!(
                    "Read/write error to PMAC C.S. ref {}, axis {} command {command}. \
                         Status={}, Error={}\n",
                    self.reference,
                    axis_name(axis).unwrap_or(' '),
                    e.c_status_code(),
                    e.message_display()
                ),
            );
        })
    }

    /// C `drvPmacGetAxesStatus`: `&{cs}??` for the status words, then one
    /// command carrying all nine `Q8{n}` readbacks.
    fn refresh(&mut self) -> AsynResult<CsPoll> {
        let cs = self.cs;
        // C `drvPmacGetCoordStatus(first_axis, ...)`.
        let command = format!("&{cs}??");
        let (cmd_status, response) = match self.transfer(1, &command, true) {
            // C `MOTOR_AXIS_ERROR`, over the empty response buffer.
            Err(_) => (-1, String::new()),
            Ok(raw) => (
                0,
                String::from_utf8_lossy(&raw).trim_matches('\0').to_string(),
            ),
        };
        // C goes on to the positions whatever the status read gave.
        let nvals = scan_cs_status(&response, &mut self.status);
        if cmd_status != 0 || nvals != 3 {
            self.trace.print(
                TraceMask::ERROR,
                0,
                format_args!(
                    "drvPmacAxisGetStatus: not all status values returned. Status: \
                     {cmd_status}\nCommand :{command}\nResponse:{response}"
                ),
            );
        }

        let mut query = format!("&{cs}");
        for axis in 1..=CS_AXES {
            query.push_str(&format!("Q8{axis}"));
        }
        // A failed read leaves C's position buffer uninitialised; that
        // garbage is not posted here.
        let raw = self.transfer(1, &query, true)?;
        let response = String::from_utf8_lossy(&raw);
        // strtod reads 0 past the end of a short reply, which C posts.
        let mut positions = parse_cs_positions(response.trim_matches('\0'), CS_AXES);
        positions.resize(CS_AXES, 0.0);

        let [s0, s1, s2] = self.status;
        let poll = CsPoll {
            status: (s0, s1, s2),
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
    /// motion program once for the whole coordinate system. Its caller
    /// ignores the status, and the abort's is overwritten (pmacAsynCoord.c:356-361),
    /// so a failure only prints.
    fn process_deferred_moves(&mut self) {
        let cs = self.cs;
        let program = self.program;
        self.deferred = [false; CS_AXES];
        // C sends both on `&pDrv->axis[0]`.
        let _ = self.command(1, &format!("&{cs}A"));
        let _ = self.command(1, &format!("&{cs}AB{program}R"));
    }
}

pub struct PmacCsAxis {
    system: Arc<Mutex<PmacCoordSystem>>,
    /// 1-based index into the coordinate system: 1 = A … 9 = Z.
    axis: usize,
    previous_position: f64,
    previous_direction: bool,
    /// A move was sent since the last poll: C `motorAxisMove` clears
    /// `motorAxisDone` so the record's next status update is not done.
    motion_sent: bool,
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
            motion_sent: false,
        })
    }
}

impl AsynMotor for PmacCsAxis {
    /// C `motorAxisWriteRead` sets only `motorAxisCommError`
    /// (pmacAsynCoord.c).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::COMMS_ERROR
    }

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
        let (cs, step) = (system.cs, system.step_size[axis - 1]);
        let csvar = cs + 50;
        let position = position * step;

        let mut command = format!("&{cs}");
        if velocity != 0.0 {
            command.push_str(&format!("I{csvar}89={:.6} ", CFloat(velocity * step)));
            if acceleration != 0.0 {
                command.push_str(&format!(
                    "I{csvar}87={:.6} ",
                    CFloat((velocity / acceleration).abs() * 1000.0)
                ));
            }
        }
        command.push_str(&format!(
            "Q7{axis}={position:.12}",
            position = CFloat(position)
        ));

        if system.moves_deferred {
            system.deferred[axis - 1] = true;
        } else if system.program != 0 {
            // Abort first so the motors are back in closed loop; then append the
            // program run to the demand, so the demand and its start are one
            // command.
            system.command(axis, &format!("&{cs}A"))?;
            command.push_str(&format!("B{}R", system.program));
        }
        let sent = system.command(axis, &command);
        self.motion_sent = true;
        sent
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
        Err(lock(&self.system).refuse("motorAxisHome: not implemented for CS axes"))
    }

    /// C `motorAxisVelocityMove`: likewise, no open-ended jog in a CS.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        Err(lock(&self.system).refuse("motorAxisVelocityMove: not implemented for CS axes"))
    }

    /// C `motorAxisStop`: abort the coordinate system and park the demand on the
    /// readback, so the CS does not resume the interrupted move.
    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let axis = self.axis;
        let mut system = lock(&self.system);
        let cs = system.cs;
        system.deferred[axis - 1] = false;
        system.command(axis, &format!("&{cs}A Q7{axis}=Q8{axis}"))
    }

    /// C `motorAxisSetDouble`, `motorAxisPosition`: there is nothing to redefine
    /// on a kinematic axis — the readback comes from the CS PLC — so C only
    /// updates the demand register, keeping it consistent with the readback.
    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let axis = self.axis;
        let mut system = lock(&self.system);
        let (cs, step) = (system.cs, system.step_size[axis - 1]);
        system.flow(
            axis,
            format_args!(
                "Set ref {}, axis {} to position {:.6}\n",
                system.reference,
                axis_name(axis).unwrap_or(' '),
                CFloat(position)
            ),
        );
        // C rounds the record's steps to a whole step first.
        let position = (position + 0.5).floor() * step;
        system.command(
            axis,
            &format!("&{cs}Q7{axis}={position:.6}", position = CFloat(position)),
        )
    }

    /// C `motorAxisSetDouble`, `motorAxisDeferMoves`: releasing the deferral runs
    /// the motion program once for the whole coordinate system, starting every
    /// axis whose demand was written while it was armed.
    fn set_deferred_moves(&mut self, _user: &AsynUser, defer: bool) -> AsynResult<()> {
        let mut system = lock(&self.system);
        system.flow(
            self.axis,
            format_args!(
                "{}ing Deferred Move flag on PMAC ref {}, cs {}\n",
                if defer { "Sett" } else { "Clear" },
                system.reference,
                system.cs
            ),
        );
        let release = !defer && system.moves_deferred;
        system.moves_deferred = defer;
        if release {
            system.process_deferred_moves();
        }
        Ok(())
    }

    fn set_encoder_ratio(&mut self, _user: &AsynUser, ratio: f64) -> AsynResult<()> {
        lock(&self.system).ignore(self.axis, "Cannot set", "encoder ratio", ratio)
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        lock(&self.system).ignore(self.axis, "Ignoring", "low limit", position)
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        lock(&self.system).ignore(self.axis, "Ignoring", "high limit", position)
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let what = match kind {
            PidGainKind::Proportional => "pgain",
            PidGainKind::Integral => "igain",
            PidGainKind::Derivative => "dgain",
        };
        lock(&self.system).ignore(self.axis, "Ignoring", what, gain)
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let system = lock(&self.system);
        system.flow(
            self.axis,
            format_args!(
                "Cannot set PMAC ref {}, axis {} closed loop ({})\n",
                system.reference,
                axis_name(self.axis).unwrap_or(' '),
                if enable { "ON" } else { "OFF" }
            ),
        );
        Ok(())
    }

    /// C `motorAxisforceCallback`; the poll that follows is the poller's.
    fn force_callback(&mut self, _user: &AsynUser) {
        let system = lock(&self.system);
        system.flow(
            self.axis,
            format_args!(
                "motorAxisforceCallback: request ref {}, cs {}, axis {} status update\n",
                system.reference,
                system.cs,
                axis_name(self.axis).unwrap_or(' ')
            ),
        );
    }

    /// C `drvPmacGetAxesStatus`, for this axis's share of the system poll.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let axis = self.axis;
        let mut system = lock(&self.system);
        let poll = system.poll_system()?;
        let (status1, status2, status3) = poll.status;
        let position = poll.positions[axis - 1] / system.step_size[axis - 1];

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

        let done = flags.done && !std::mem::take(&mut self.motion_sent);
        Ok(MotorStatus {
            position,
            encoder_position: position,
            done,
            // C paces on motorAxisDone (pmacAsynCoord.c:719-724) and sets
            // motorAxisMoving from in-position alone (659).
            moving: !done,
            msta_moving: flags.moving,
            high_limit: flags.limit,
            low_limit: flags.limit,
            direction,
            slip_stall: flags.following_error,
            problem: flags.problem,
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

    /// A coordinate system in position with no program running.
    struct Idle {
        base: epics_rs::asyn::port::PortDriverBase,
        reply: Vec<u8>,
        sent: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for Idle {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            self.reply = match cmd.as_str() {
                "&1??" => b"000000\r020000\r000000\r".to_vec(),
                c if c.starts_with("&1Q81") => b"1.5 0 0 0 0 0 0 0 0\r".to_vec(),
                _ => b"\r".to_vec(),
            };
            self.sent.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let n = self.reply.len();
            buf[..n].copy_from_slice(&self.reply);
            Ok(n)
        }
    }

    type Rig = (
        PmacCsAxis,
        Arc<Mutex<PmacCoordSystem>>,
        Arc<Mutex<Vec<String>>>,
    );

    /// CS 1 axis A on an [`Idle`] line, its system and the wire log.
    fn idle_axis(name: &str, poll_ttl: Duration) -> Rig {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Idle {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            reply: Vec::new(),
            sent: sent.clone(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let system = Arc::new(Mutex::new(PmacCoordSystem::new(
            io,
            MotorTrace::new(name, 0),
            1,
            1,
            0,
            poll_ttl,
        )));
        let axis = PmacCsAxis::new(system.clone(), 1).unwrap();
        (axis, system, sent)
    }

    /// pmacAsynCoord.c: demand and feed rate are steps * stepSize, the
    /// readback Q8n / stepSize, a set position rounds to a whole step first.
    #[test]
    fn record_steps_scale_by_step_size() {
        let (mut axis, system, sent) = idle_axis("pmac_cs_steps", Duration::ZERO);
        let user = AsynUser::default();
        assert_eq!(axis.poll(&user).unwrap().position, 15000.0);
        sent.lock().unwrap().clear();
        axis.move_absolute(&user, 20000.0, 0.0, 5000.0, 10000.0)
            .unwrap();
        axis.set_position(&user, 1234.6).unwrap();
        lock(&system).set_steps_per_unit(0, 100.0);
        axis.move_absolute(&user, 250.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(axis.poll(&user).unwrap().position, 150.0);
        let wire = sent.lock().unwrap().clone();
        assert_eq!(
            wire[..3],
            [
                "&1I5189=0.500000 I5187=500.000000 Q71=2.000000000000",
                "&1Q71=0.123500",
                "&1Q71=2.500000000000"
            ]
        );
    }

    /// pmacAsynCoord.c motorAxisMove clears motorAxisDone and wakes the
    /// poller: the poll after a move reads the system and is not done, the
    /// one after it is served from that read again.
    #[test]
    fn the_poll_after_a_move_is_fresh_and_not_done() {
        let (mut axis, _, sent) = idle_axis("pmac_cs_moved", Duration::from_secs(3600));
        let user = AsynUser::default();
        assert!(axis.poll(&user).unwrap().done);
        sent.lock().unwrap().clear();
        assert!(axis.poll(&user).unwrap().done);
        assert!(sent.lock().unwrap().is_empty(), "inside the TTL");

        axis.move_absolute(&user, 2.0, 0.0, 0.0, 0.0).unwrap();
        let after = axis.poll(&user).unwrap();
        let wire = sent.lock().unwrap().clone();
        assert_eq!(wire.len(), 3, "{wire:?}");
        assert!(!after.done);
        // In position yet not done: C paces fast, motorAxisMoving stays 0.
        assert!(after.moving && !after.msta_moving);

        sent.lock().unwrap().clear();
        assert!(axis.poll(&user).unwrap().done);
        assert!(sent.lock().unwrap().is_empty());
    }

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
