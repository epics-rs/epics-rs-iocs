//! Simulated motor axis, ported from
//! `motorMotorSim/motorSimApp/src/motorSimDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` driver).
//!
//! Each axis owns a [`Route`] trajectory ([`crate::route`]) and integrates its
//! motion forward in real time. The C driver runs a dedicated 0.1 s background
//! thread (`motorSimTask`) that calls `process(delta)`; the asyn-rs port folds
//! that into `MotorSimAxis::advance`, which every driver entry
//! point calls first: it integrates the wall time elapsed since the last call
//! in `DELTA` steps, so a command acts on the position the axis has at that
//! instant and the hard-limit check runs at C's step size however sparse the
//! polls are. There is no hardware and no unit scaling — positions and
//! velocities are dimensionless record EGU.
//!
//! ## Deviations from C (documented)
//!
//! - C drops a task period outside `[DELTA/4, 4·DELTA]` as a clock step; here
//!   there is no task to stall, so every elapsed interval is integrated.
//! - `movesDeferred` is controller-wide in C (`writeInt32(motorDeferMoves)`
//!   applies every axis's deferred move); here it is per-axis, which is
//!   equivalent for the simulator (its axes have no inter-axis coupling).
//! - The post-move delay (`motorPostMoveDelay_`) is owned by the record layer
//!   in this stack, so the axis reports `done` as soon as it stops.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, StatusWrite,
};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use motor_common::drv_motor_asyn::{DrvGet, DrvSet, function};
use motor_common::model2::AxisLog;
use motor_common::param_lib::Params;

use crate::route::{Demand, Reroute, Route};

/// C `DEFAULT_LOW_LIMIT` / `DEFAULT_HI_LIMIT`.
pub const DEFAULT_LOW_LIMIT: f64 = -10000.0;
pub const DEFAULT_HI_LIMIT: f64 = 10000.0;
/// C `DEFAULT_HOME` / `DEFAULT_START`.
pub const DEFAULT_HOME: f64 = 0.0;
pub const DEFAULT_START: f64 = 0.0;
/// C `DELTA`: the `motorSimTask` integration step, in seconds.
const DELTA: f64 = 0.1;

fn sim_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// A single simulated axis.
pub struct MotorSimAxis {
    route: Route,
    endpoint: Demand,
    nextpoint: Demand,
    reroute: Reroute,
    low_hard_limit: f64,
    hi_hard_limit: f64,
    home: f64,
    homing: bool,
    homed: bool,
    enc_offset: f64,
    deferred_move: bool,
    deferred_position: f64,
    moves_deferred: bool,
    /// The instant the trajectory has been integrated up to.
    clock: Instant,
    last_status: MotorStatus,
}

impl MotorSimAxis {
    /// Construct an axis at `start`, with the C constructor defaults
    /// (`Amax = Vmax = 1`). Hard limits, home and start match
    /// `motorSimController`'s `new motorSimAxis(...)`.
    pub fn new(low_hard_limit: f64, hi_hard_limit: f64, home: f64, start: f64) -> Self {
        let demand = Demand {
            t: 0.0,
            p: start,
            v: 0.0,
        };
        // Amax = Vmax = 1 with zero initial velocity always satisfies routeNew.
        let route = Route::new(demand, 1.0, 1.0).expect("Amax=Vmax=1 at rest is valid");
        let mut axis = Self {
            route,
            endpoint: demand,
            nextpoint: demand,
            reroute: Reroute::Calc,
            low_hard_limit,
            hi_hard_limit,
            home,
            homing: false,
            homed: false,
            enc_offset: 0.0,
            deferred_move: false,
            deferred_position: 0.0,
            moves_deferred: false,
            clock: Instant::now(),
            last_status: MotorStatus::default(),
        };
        axis.last_status = axis.snapshot(true);
        axis
    }

    /// C `motorSimAxis::config`: reset the hard limits, home, and start
    /// (start becomes the encoder offset), clearing the homed flag.
    pub fn config(&mut self, hi_hard_limit: f64, low_hard_limit: f64, home: f64, start: f64) {
        self.hi_hard_limit = hi_hard_limit;
        self.low_hard_limit = low_hard_limit;
        self.home = home;
        self.enc_offset = start;
        self.homed = false;
    }

    /// The level-1 lines C `motorSimController::report` prints for this
    /// axis (motorSimDriver.cpp:122-141), with the port's soft limits.
    pub fn report(&self, out: &mut dyn std::fmt::Write, (low_soft, hi_soft): (f64, f64)) {
        let (next, end) = (self.nextpoint, self.endpoint);
        let _ = writeln!(
            out,
            "  Current position = {:.6}, velocity = {:.6} at current time: {:.6}",
            next.p, next.v, next.t
        );
        let _ = writeln!(
            out,
            "  Destination posn = {:.6}, velocity = {:.6} at desination time:  {:.6}",
            end.p, end.v, end.t
        );
        let _ = writeln!(
            out,
            "    Hard limits: {:.6}, {:.6}",
            self.low_hard_limit, self.hi_hard_limit
        );
        let _ = writeln!(out, "           Home: {:.6}", self.home);
        let _ = writeln!(out, "    Enc. offset: {:.6}", self.enc_offset);
        let _ = writeln!(out, "    Soft limits: {low_soft:.6}, {hi_soft:.6}");
        if self.homing {
            let _ = writeln!(out, "    Currently homing axis");
        }
    }

    /// Shared body of the absolute/relative move (C `motorSimAxis::move`).
    fn do_move(
        &mut self,
        position_in: f64,
        relative: bool,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let mut position = position_in;
        if relative {
            position += self.endpoint.p + self.enc_offset;
        }
        // Reject a move that pushes further past a hard limit.
        if (self.nextpoint.p >= self.hi_hard_limit && position > self.nextpoint.p)
            || (self.nextpoint.p <= self.low_hard_limit && position < self.nextpoint.p)
        {
            return Err(sim_err("motorSim: move beyond hard limit"));
        }
        if !self.moves_deferred {
            self.endpoint.p = position - self.enc_offset;
            self.endpoint.v = 0.0;
        } else {
            self.deferred_position = position;
            self.deferred_move = true;
        }
        let mut amax = self.route.amax();
        let mut vmax = self.route.vmax();
        if velocity != 0.0 {
            vmax = velocity.abs();
        }
        if acceleration != 0.0 {
            amax = acceleration.abs();
        }
        self.route.set_params(amax, vmax);
        Ok(())
    }

    /// C `motorSimAxis::setVelocity`: move at a constant velocity (used by jog,
    /// home, and stop).
    fn set_velocity(&mut self, velocity: f64, acceleration: f64) -> AsynResult<()> {
        let delta_v = velocity - self.nextpoint.v;
        if (self.nextpoint.p > self.hi_hard_limit && velocity > 0.0)
            || (self.nextpoint.p < self.low_hard_limit && velocity < 0.0)
        {
            return Err(sim_err("motorSim: velocity beyond hard limit"));
        }
        let mut amax = self.route.amax();
        if acceleration != 0.0 {
            amax = acceleration.abs();
        }
        self.route.set_params(amax, self.route.vmax());

        let time = (delta_v / amax).abs();
        self.endpoint.v = velocity;
        self.endpoint.p = self.nextpoint.p + time * (self.nextpoint.v + 0.5 * delta_v);
        self.reroute = Reroute::New;
        Ok(())
    }

    /// Integrate the trajectory up to `now` in `DELTA` steps (C
    /// `motorSimTask`), so commands and polls see the current position.
    fn advance(&mut self, now: Instant) {
        let mut elapsed = now.saturating_duration_since(self.clock).as_secs_f64();
        self.clock = now;
        while elapsed > 0.0 {
            let step = elapsed.min(DELTA);
            self.process(step);
            elapsed -= step;
        }
    }

    /// C `motorSimAxis::process`: propagate the motion forward by `delta`
    /// seconds and refresh the reported status.
    fn process(&mut self, delta: f64) {
        let lastpos = self.nextpoint.p;
        self.nextpoint.t += delta;

        let mut endp = self.endpoint;
        self.route
            .find(self.reroute, &mut endp, &mut self.nextpoint);
        self.endpoint = endp;
        self.reroute = Reroute::Calc;

        // Homing: stop and pin to home once the home sensor is crossed.
        if self.homing && (lastpos - self.home) * (self.nextpoint.p - self.home) <= 0.0 {
            self.homing = false;
            self.homed = true;
            self.reroute = Reroute::New;
            self.endpoint.p = self.home;
            self.endpoint.v = 0.0;
        }

        // Hard-limit reflection.
        if self.nextpoint.p > self.hi_hard_limit && self.nextpoint.v > 0.0 {
            if self.homing {
                let _ = self.set_velocity(-self.endpoint.v, 0.0);
            } else {
                self.reroute = Reroute::New;
                self.endpoint.p = self.hi_hard_limit;
                self.endpoint.v = 0.0;
            }
        } else if self.nextpoint.p < self.low_hard_limit && self.nextpoint.v < 0.0 {
            if self.homing {
                let _ = self.set_velocity(-self.endpoint.v, 0.0);
            } else {
                self.reroute = Reroute::New;
                self.endpoint.p = self.low_hard_limit;
                self.endpoint.v = 0.0;
            }
        }

        let done = self.nextpoint.v == 0.0 && !self.deferred_move;
        self.last_status = self.snapshot(done);
    }

    /// processDeferredMoves for this axis: take up its deferred move unless
    /// that pushes past a hard limit; whether it did.
    fn release_deferred(&mut self) -> bool {
        let position = self.deferred_position;
        let blocked = (self.nextpoint.p >= self.hi_hard_limit && position > self.nextpoint.p)
            || (self.nextpoint.p <= self.low_hard_limit && position < self.nextpoint.p);
        if !self.deferred_move || blocked {
            return false;
        }
        self.endpoint.p = position - self.enc_offset;
        self.endpoint.v = 0.0;
        self.deferred_move = false;
        true
    }

    /// Build the reported [`MotorStatus`] from the current point (C's
    /// `setDoubleParam`/`setIntegerParam` block at the end of `process`).
    fn snapshot(&self, done: bool) -> MotorStatus {
        let pos = self.nextpoint.p + self.enc_offset;
        MotorStatus {
            position: pos,
            encoder_position: pos,
            velocity: 0.0,
            done,
            moving: !done,
            msta_moving: !done,
            direction: self.nextpoint.v > 0.0,
            high_limit: self.nextpoint.p >= self.hi_hard_limit,
            low_limit: self.nextpoint.p <= self.low_hard_limit,
            // C motorStatusHome_ (EA_HOME), not motorStatusAtHome_.
            encoder_home: self.nextpoint.p == self.home,
            homed: self.homed,
            ..MotorStatus::default()
        }
    }
}

impl AsynMotor for MotorSimAxis {
    /// C `motorSimAxis::poll` has no failure path.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.advance(Instant::now());
        self.do_move(position, false, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.advance(Instant::now());
        self.do_move(distance, true, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.advance(Instant::now());
        self.set_velocity(velocity, acceleration)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        self.advance(Instant::now());
        let status = self.set_velocity(if forward { velocity } else { -velocity }, acceleration);
        self.homing = true;
        self.homed = false;
        status
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        self.advance(Instant::now());
        let _ = self.set_velocity(0.0, acceleration);
        self.deferred_move = false;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.advance(Instant::now());
        self.enc_offset = position - self.nextpoint.p;
        Ok(())
    }

    fn set_deferred_moves(&mut self, _user: &AsynUser, defer: bool) -> AsynResult<()> {
        self.advance(Instant::now());
        // C processDeferredMoves, reduced to this axis (module Deviations).
        if !defer && self.moves_deferred {
            self.release_deferred();
        }
        self.moves_deferred = defer;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        self.advance(Instant::now());
        Ok(self.last_status.clone())
    }
}

/// C `motorSimLogMsg`, drvMotorSim's print until drvAsynMotorConfigure
/// sets another: stdout and a newline, whatever the mask.
pub fn sim_log(_: TraceMask, args: std::fmt::Arguments<'_>) {
    println!("{args}");
}

/// drvMotorSim's `drv.movesDeferred`, one flag for every axis, and the
/// number of times it was cleared with moves deferred, each of which runs
/// processDeferredMoves over the axes. An axis takes up a release on its
/// next call, which the poller makes within `DELTA`, as C's motorProcTask
/// only moves it then.
#[derive(Debug, Default)]
pub struct Deferral(Mutex<(bool, u64)>);

impl Deferral {
    fn get(&self) -> (bool, u64) {
        *self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn set(&self, defer: bool) {
        let mut state = self.0.lock().unwrap_or_else(|e| e.into_inner());
        if !defer && state.0 {
            state.1 += 1;
        }
        state.0 = defer;
    }
}

/// drvMotorSim's `motorAxisGetInteger`/`motorAxisGetDouble`: the axis'
/// store, but the driver's deferral for motorAxisDeferMoves
/// (drvMotorSim.c:221-248).
pub struct SimGet {
    params: Params,
    deferral: Arc<Deferral>,
}

impl DrvGet for SimGet {
    fn params(&self) -> &Params {
        &self.params
    }

    fn get_integer(&self, function: i32) -> Option<i32> {
        match function {
            function::DEFER_MOVES => Some(i32::from(self.deferral.get().0)),
            _ => self.params.get_integer(function).ok().flatten(),
        }
    }

    fn get_double(&self, function: i32) -> Option<f64> {
        match function {
            function::DEFER_MOVES => Some(f64::from(u8::from(self.deferral.get().0))),
            _ => self.params.get_double(function).ok().flatten(),
        }
    }
}

/// An axis of drvMotorSim.c, the model-2 simulator `motorSimCreate` builds
/// for `DTYP "Motor Simulation"`: the same trajectory as [`MotorSimAxis`],
/// reported through its motorAxis status bits, where the home sensor is
/// `motorAxisHomeSignal` (MSTA RA_HOME) and nothing tracks homed.
pub struct DrvMotorSimAxis {
    axis: MotorSimAxis,
    writes: Vec<StatusWrite>,
    card: i32,
    signal: i32,
    log: Arc<AxisLog>,
    deferral: Arc<Deferral>,
    /// The releases of `deferral` this axis has taken up.
    releases: u64,
    /// `pAxis->params` and the getters over it.
    get: Arc<SimGet>,
}

impl DrvMotorSimAxis {
    /// `motorSimCreateAxis(card, axis, lowLimit, hiLimit, home, start)`,
    /// which overwrites `start` with 0 before using it and sets it as the
    /// paramLib's Position (drvMotorSim.c:739).
    pub fn new(
        card: i32,
        signal: i32,
        low_hard_limit: f64,
        hi_hard_limit: f64,
        home: f64,
        deferral: Arc<Deferral>,
    ) -> Self {
        let releases = deferral.get().1;
        let get = Arc::new(SimGet {
            params: Params::default(),
            deferral: deferral.clone(),
        });
        // drvMotorSim.c:739.
        let _ = get.params.set_double(function::POSITION, DEFAULT_START);
        Self {
            axis: MotorSimAxis::new(low_hard_limit, hi_hard_limit, home, DEFAULT_START),
            writes: Vec::new(),
            card,
            signal,
            log: Arc::new(AxisLog::with_own(sim_log)),
            deferral,
            releases,
            get,
        }
    }

    /// The lines C `motorAxisReportAxis` prints for this axis at level 1
    /// (drvMotorSim.c:136-158), its soft limits and dump from its paramLib.
    pub fn report(&self, out: &mut dyn std::fmt::Write) {
        let axis = &self.axis;
        let (next, end) = (axis.nextpoint, axis.endpoint);
        let _ = writeln!(
            out,
            "Current position = {:.6}, velocity = {:.6} at current time: {:.6}",
            next.p, next.v, next.t
        );
        let _ = writeln!(
            out,
            "Destination posn = {:.6}, velocity = {:.6} at desination time:  {:.6}",
            end.p, end.v, end.t
        );
        let _ = writeln!(
            out,
            "Hard limits: {:.6}, {:.6}",
            axis.low_hard_limit, axis.hi_hard_limit
        );
        // C's locals stay 0.0 when getDouble finds no value.
        let soft = |f| self.params().get_double(f).ok().flatten().unwrap_or(0.0);
        let (low_soft, hi_soft) = (soft(function::LOW_LIMIT), soft(function::HIGH_LIMIT));
        let _ = writeln!(out, "Soft limits: {low_soft:.6}, {hi_soft:.6}");
        if axis.homing {
            let _ = writeln!(out, "Currently homing axis");
        }
        self.params().dump(out);
    }

    /// The axis' getters, which drvAsynMotorConfigure reads and calls back
    /// through.
    pub fn getter(&self) -> Arc<SimGet> {
        self.get.clone()
    }

    fn params(&self) -> &Params {
        &self.get.params
    }

    /// The axis' `print`/`logParam`, which drvAsynMotorConfigure sets.
    pub fn log(&self) -> Arc<AxisLog> {
        self.log.clone()
    }

    fn flow(&self, args: std::fmt::Arguments<'_>) {
        self.log.print(TraceMask::FLOW, args);
    }

    /// `motorParam->setInteger(motorAxisDone, 0)` + `callCallback`, which
    /// drvMotorSim.c's move, home and velocity move post before returning.
    fn started(&mut self) {
        self.writes.push(StatusWrite::Done(false));
        let _ = self.params().set_integer(function::DONE, 0);
        self.params().call_callback();
    }

    /// The driver's deferral as of now: a release since the last call
    /// first moves the axis to its deferred position, setting Done 0 as
    /// processDeferredMoves does. C returns at the first axis a hard limit
    /// blocks, leaving every later axis deferred (drvMotorSim.c:274); here
    /// each axis is released on its own.
    fn sync(&mut self) {
        let (deferred, releases) = self.deferral.get();
        if releases != self.releases {
            self.releases = releases;
            self.axis.advance(Instant::now());
            if self.axis.release_deferred() {
                self.started();
            }
        }
        self.axis.moves_deferred = deferred;
    }

    /// C motorAxisDeferMoves in setDouble/setInteger (drvMotorSim.c:351-357,
    /// :409-416). C stores `(int)value`, so a fraction clears the flag
    /// without releasing the moves (:357); here any nonzero value defers.
    fn defer(&mut self, defer: bool) {
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "{}ing Deferred Move flag on PMAC card {}\n",
                if defer { "Sett" } else { "Clear" },
                self.card
            ),
        );
        self.deferral.set(defer);
        self.sync();
    }

    fn move_to(
        &mut self,
        position: f64,
        relative: bool,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.sync();
        self.axis.advance(Instant::now());
        // C prints the absolute target (drvMotorSim.c:440).
        let target = if relative {
            position + self.axis.endpoint.p + self.axis.enc_offset
        } else {
            position
        };
        self.axis
            .do_move(position, relative, velocity, acceleration)?;
        self.started();
        self.flow(format_args!(
            "Set card {}, axis {} move to {}, min vel={}, max_vel={}, accel={}",
            self.card,
            self.signal,
            CFloat(target),
            CFloat(min_velocity),
            CFloat(velocity),
            CFloat(acceleration)
        ));
        Ok(())
    }
}

impl AsynMotor for DrvMotorSimAxis {
    /// drvMotorSim.c's `query_axis` has no failure path.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.move_to(position, false, min_velocity, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.move_to(distance, true, min_velocity, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.sync();
        let status = self
            .axis
            .move_velocity(user, min_velocity, velocity, acceleration);
        self.started();
        self.flow(format_args!(
            "Set card {}, axis {} move with velocity of {}, accel={}",
            self.card,
            self.signal,
            CFloat(velocity),
            CFloat(acceleration)
        ));
        status
    }

    fn home(
        &mut self,
        user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        self.sync();
        let status = self
            .axis
            .home(user, min_velocity, velocity, acceleration, forward);
        self.started();
        self.flow(format_args!(
            "Set card {}, axis {} to home {}, min vel={}, max_vel={}, accel={}",
            self.card,
            self.signal,
            if forward { "FORWARDS" } else { "REVERSE" },
            CFloat(min_velocity),
            CFloat(velocity),
            CFloat(acceleration)
        ));
        status
    }

    fn stop(&mut self, user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        self.sync();
        self.axis.stop(user, acceleration)?;
        self.flow(format_args!(
            "Set card {}, axis {} to stop with accel={}",
            self.card,
            self.signal,
            CFloat(acceleration)
        ));
        Ok(())
    }

    fn set_position(&mut self, user: &AsynUser, position: f64) -> AsynResult<()> {
        self.sync();
        self.axis.set_position(user, position)?;
        self.flow(format_args!(
            "Set card {}, axis {} to position {}",
            self.card,
            self.signal,
            CFloat(position)
        ));
        Ok(())
    }

    fn set_encoder_ratio(&mut self, _user: &AsynUser, ratio: f64) -> AsynResult<()> {
        self.flow(format_args!(
            "Set card {}, axis {} to enc. ratio to {}",
            self.card,
            self.signal,
            CFloat(ratio)
        ));
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.flow(format_args!(
            "Set card {}, axis {} low limit to {}",
            self.card,
            self.signal,
            CFloat(position)
        ));
        Ok(())
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.flow(format_args!(
            "Set card {}, axis {} high limit to {}",
            self.card,
            self.signal,
            CFloat(position)
        ));
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let which = match kind {
            PidGainKind::Proportional => "p",
            PidGainKind::Integral => "i",
            PidGainKind::Derivative => "d",
        };
        self.flow(format_args!(
            "Set card {}, axis {} {which}gain to {}",
            self.card,
            self.signal,
            CFloat(gain)
        ));
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.flow(format_args!(
            "Set card {}, axis {} closed loop to {}",
            self.card,
            self.signal,
            if enable { "ON" } else { "OFF" }
        ));
        Ok(())
    }

    /// motorProcTask's pass over the axis: motorSimProcess's parameters
    /// (drvMotorSim.c:646-653), then the callback (:677).
    fn poll(&mut self, user: &AsynUser) -> AsynResult<MotorStatus> {
        self.sync();
        let status = self.axis.poll(user)?;
        let params = self.params();
        for (f, v) in [
            (function::POSITION, status.position),
            (function::ENCODER_POSN, status.encoder_position),
        ] {
            let _ = params.set_double(f, v);
        }
        for (f, v) in [
            (function::DIRECTION, status.direction),
            (function::DONE, status.done),
            (function::HIGH_HARD_LIMIT, status.high_limit),
            (function::HOME_SIGNAL, status.encoder_home),
            (function::MOVING, !status.done),
            (function::LOW_HARD_LIMIT, status.low_limit),
        ] {
            let _ = params.set_integer(f, i32::from(v));
        }
        params.call_callback();
        Ok(params.motor_status())
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.writes)
    }

    /// C `motorAxisforceCallback` (drvMotorSim.c:571-579): the next
    /// callback goes out with nothing changed; the poll the port then
    /// forces is C's motorProcTask.
    fn force_callback(&mut self, _user: &AsynUser) {
        self.params().force_callback();
    }
}

/// C `motorAxisSetDouble`/`motorAxisSetInteger` (drvMotorSim.c:292-432):
/// each function they know prints its FLOW line, goes into the store and
/// runs the callback; any other fails with none.
impl DrvSet for DrvMotorSimAxis {
    fn set_double(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()> {
        self.set_double_only(user, function, value)?;
        let _ = self.params().set_double(function, value);
        self.params().call_callback();
        Ok(())
    }

    fn set_integer(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()> {
        self.set_integer_only(user, function, value)?;
        let _ = self.params().set_integer(function, value);
        self.params().call_callback();
        Ok(())
    }
}

impl DrvMotorSimAxis {
    fn set_double_only(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()> {
        match function {
            function::POSITION => self.set_position(user, value),
            function::RESOLUTION => {
                self.flow(format_args!(
                    "Set card {}, axis {} resolution to {}",
                    self.card,
                    self.signal,
                    CFloat(value)
                ));
                Ok(())
            }
            function::ENCODER_RATIO => self.set_encoder_ratio(user, value),
            function::LOW_LIMIT => self.set_low_limit(user, value),
            function::HIGH_LIMIT => self.set_high_limit(user, value),
            function::P_GAIN => self.set_pid_gain(user, PidGainKind::Proportional, value),
            function::I_GAIN => self.set_pid_gain(user, PidGainKind::Integral, value),
            function::D_GAIN => self.set_pid_gain(user, PidGainKind::Derivative, value),
            function::CLOSED_LOOP => self.set_closed_loop(user, value != 0.0),
            function::DEFER_MOVES => {
                self.defer(value != 0.0);
                Ok(())
            }
            _ => Err(sim_err(format!("motorSim: no double function {function}"))),
        }
    }

    fn set_integer_only(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()> {
        let what = match function {
            function::POSITION => {
                self.sync();
                self.axis.set_position(user, f64::from(value))?;
                "to position"
            }
            function::LOW_LIMIT => "low limit to",
            function::HIGH_LIMIT => "high limit to",
            function::CLOSED_LOOP => return self.set_closed_loop(user, value != 0),
            function::DEFER_MOVES => {
                self.defer(value != 0);
                return Ok(());
            }
            _ => return Err(sim_err(format!("motorSim: no integer function {function}"))),
        };
        self.flow(format_args!(
            "Set card {}, axis {} {what} {value}",
            self.card, self.signal
        ));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn user() -> AsynUser {
        AsynUser::default()
    }

    /// Drive the axis with fixed `delta` steps until it reports done (or the
    /// step budget runs out), returning the reported position.
    fn run_to_done(axis: &mut MotorSimAxis, dt: f64, max_steps: usize) -> (f64, bool) {
        let mut done = false;
        for _ in 0..max_steps {
            axis.process(dt);
            done = axis.last_status.done;
            if done {
                break;
            }
        }
        (axis.last_status.position, done)
    }

    #[test]
    fn absolute_move_reaches_target() {
        let mut axis = MotorSimAxis::new(DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, 0.0, 0.0);
        axis.move_absolute(&user(), 5.0, 0.0, 2.0, 1.0).unwrap();
        let (pos, done) = run_to_done(&mut axis, 0.05, 1000);
        assert!(done, "move did not finish");
        assert!((pos - 5.0).abs() < 1e-3, "final position {pos}");
    }

    /// C motorSimDriver.cpp:476 / drvMotorSim.c:652: MSTA RA_MOVING is
    /// `!done`.
    #[test]
    fn msta_moving_follows_not_done() {
        let mut axis = MotorSimAxis::new(DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, 0.0, 0.0);
        axis.move_absolute(&user(), 5.0, 0.0, 2.0, 1.0).unwrap();
        axis.process(0.05);
        assert!(axis.last_status.msta_moving);
        run_to_done(&mut axis, 0.05, 1000);
        assert!(!axis.last_status.msta_moving);
    }

    #[test]
    fn relative_move_accumulates() {
        let mut axis = MotorSimAxis::new(DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, 0.0, 0.0);
        axis.move_absolute(&user(), 3.0, 0.0, 2.0, 1.0).unwrap();
        run_to_done(&mut axis, 0.05, 1000);
        axis.move_relative(&user(), 2.0, 0.0, 2.0, 1.0).unwrap();
        let (pos, done) = run_to_done(&mut axis, 0.05, 1000);
        assert!(done);
        assert!((pos - 5.0).abs() < 1e-3, "final position {pos}");
    }

    #[test]
    fn hard_limit_stops_motion() {
        let mut axis = MotorSimAxis::new(-5.0, 5.0, 0.0, 0.0);
        // Jog positive forever; the axis must stop at the high hard limit.
        axis.move_velocity(&user(), 0.0, 3.0, 1.0).unwrap();
        for _ in 0..1000 {
            axis.process(0.05);
        }
        assert!(
            axis.last_status.position <= 5.0 + 1e-6,
            "position {} exceeded hard limit",
            axis.last_status.position
        );
        assert!(axis.last_status.high_limit, "high-limit flag not set");
    }

    #[test]
    fn sparse_poll_still_stops_at_hard_limit() {
        // One poll 30 s after a move past the limit: the step-wise
        // integration must clamp at the limit, not land on the target.
        let mut axis = MotorSimAxis::new(-100.0, 100.0, 0.0, 0.0);
        axis.move_absolute(&user(), 150.0, 0.0, 5.0, 10.0).unwrap();
        let later = axis.clock + std::time::Duration::from_secs(30);
        axis.advance(later);
        assert!(
            axis.last_status.position <= 100.0 + 1e-6,
            "position {} passed the hard limit",
            axis.last_status.position
        );
        assert!(axis.last_status.high_limit);
    }

    #[test]
    fn set_position_applies_encoder_offset() {
        let mut axis = MotorSimAxis::new(DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, 0.0, 0.0);
        axis.set_position(&user(), 100.0).unwrap();
        axis.process(0.05);
        assert!(
            (axis.last_status.position - 100.0).abs() < 1e-9,
            "offset position {}",
            axis.last_status.position
        );
    }

    #[test]
    fn drv_motor_sim_reports_home_signal_as_at_home() {
        let mut axis = DrvMotorSimAxis::new(
            0,
            0,
            DEFAULT_LOW_LIMIT,
            DEFAULT_HI_LIMIT,
            0.0,
            Arc::default(),
        );
        axis.axis.process(0.05);
        let status = axis.poll(&user()).unwrap();
        assert!(status.home, "RA_HOME not set at the home position");
        assert!(!status.encoder_home && !status.homed);
    }

    /// drvMotorSim's paramLib sets: Position at create (drvMotorSim.c:739),
    /// then the eight of each motorSimProcess (:646-653), into the axis'
    /// store and nowhere else.
    #[test]
    fn drv_motor_sim_sets_its_parameters_at_create_and_each_update() {
        let mut axis = DrvMotorSimAxis::new(0, 0, -10.0, 10.0, 0.0, Arc::default());
        let get = axis.getter();
        assert_eq!(get.get_double(function::POSITION), Some(0.0));
        assert_eq!(get.get_integer(function::DONE), None);
        axis.axis.process(0.05);
        axis.poll(&user()).unwrap();
        assert!(axis.take_status_writes().is_empty());
        for (f, v) in [(function::POSITION, 0.0), (function::ENCODER_POSN, 0.0)] {
            assert_eq!(get.get_double(f), Some(v), "{f}");
        }
        for (f, v) in [
            (function::DIRECTION, 0),
            (function::DONE, 1),
            (function::HIGH_HARD_LIMIT, 0),
            (function::HOME_SIGNAL, 1),
            (function::MOVING, 0),
            (function::LOW_HARD_LIMIT, 0),
        ] {
            assert_eq!(get.get_integer(f), Some(v), "{f}");
        }
    }

    /// drvMotorSim's record sees only `pAxis->params` (drvMotorSim.c:646-677).
    #[test]
    fn drv_motor_sim_reports_its_parameters() {
        let mut axis = DrvMotorSimAxis::new(0, 0, -10.0, 10.0, 0.0, Arc::default());
        axis.axis.process(0.05);
        let polled = axis.poll(&user()).unwrap();
        assert_eq!(polled, axis.params().motor_status());
        axis.move_absolute(&user(), 5.0, 0.0, 1.0, 1.0).unwrap();
        axis.axis.process(0.5);
        let polled = axis.poll(&user()).unwrap();
        assert!(!polled.done);
        assert_eq!(polled, axis.params().motor_status());
    }

    #[test]
    fn drv_motor_sim_posts_not_done_when_motion_starts() {
        let mut axis = DrvMotorSimAxis::new(
            0,
            0,
            DEFAULT_LOW_LIMIT,
            DEFAULT_HI_LIMIT,
            0.0,
            Arc::default(),
        );
        axis.move_velocity(&user(), 0.0, 1.0, 1.0).unwrap();
        assert_eq!(axis.take_status_writes(), vec![StatusWrite::Done(false)]);
        axis.stop(&user(), 1.0).unwrap();
        assert!(axis.take_status_writes().is_empty());
    }

    /// drvMotorSim.c:259-287, 351-357: the deferral is the driver's: a move
    /// on one axis waits until a write to another clears it, and a function
    /// the driver does not set fails.
    #[test]
    fn drv_motor_sim_deferral_is_driver_wide() {
        let deferral = Arc::new(Deferral::default());
        let new = |signal| {
            DrvMotorSimAxis::new(
                0,
                signal,
                DEFAULT_LOW_LIMIT,
                DEFAULT_HI_LIMIT,
                0.0,
                deferral.clone(),
            )
        };
        let (mut a, mut b) = (new(0), new(1));
        a.set_integer(&user(), function::DEFER_MOVES, 1).unwrap();
        b.move_absolute(&user(), 4.0, 0.0, 2.0, 1.0).unwrap();
        for _ in 0..20 {
            b.axis.process(0.05);
        }
        assert!(b.axis.last_status.position.abs() < 1e-6);
        a.set_double(&user(), function::DEFER_MOVES, 0.0).unwrap();
        b.poll(&user()).unwrap();
        assert_eq!(
            b.take_status_writes(),
            [StatusWrite::Done(false), StatusWrite::Done(false)]
        );
        let (pos, done) = run_to_done(&mut b.axis, 0.05, 1000);
        assert!(done);
        assert!((pos - 4.0).abs() < 1e-3, "final position {pos}");
        assert!(a.set_integer(&user(), 12, 1).is_err());
        assert!(a.set_double(&user(), function::MOVE_TO_HOME, 1.0).is_err());
    }

    /// drvMotorSim.c:366-367, 426-427, 646-653, 677: sets and polls go into
    /// the store and call back what changed; DEFER reads the driver's flag.
    #[test]
    fn drv_motor_sim_calls_back_its_parameters() {
        let mut axis = DrvMotorSimAxis::new(
            0,
            0,
            DEFAULT_LOW_LIMIT,
            DEFAULT_HI_LIMIT,
            5.0,
            Arc::default(),
        );
        let get = axis.getter();
        let calls = Arc::new(Mutex::new(Vec::new()));
        let sink = calls.clone();
        get.params().add_callback(Arc::new(move |changed: &[i32]| {
            sink.lock().unwrap().push(changed.to_vec());
        }));
        axis.set_double(&user(), function::P_GAIN, 0.5).unwrap();
        axis.set_integer(&user(), function::DEFER_MOVES, 1).unwrap();
        assert_eq!(get.get_double(function::P_GAIN), Some(0.5));
        assert_eq!(get.get_integer(function::DEFER_MOVES), Some(1));
        axis.poll(&user()).unwrap();
        axis.poll(&user()).unwrap();
        axis.force_callback(&user());
        axis.poll(&user()).unwrap();
        assert_eq!(
            *calls.lock().unwrap(),
            [
                vec![function::POSITION, function::P_GAIN],
                vec![function::DEFER_MOVES],
                vec![
                    function::ENCODER_POSN,
                    function::DIRECTION,
                    function::DONE,
                    function::HIGH_HARD_LIMIT,
                    function::HOME_SIGNAL,
                    function::MOVING,
                    function::LOW_HARD_LIMIT,
                ],
                vec![],
            ]
        );
        assert_eq!(get.get_integer(function::DONE), Some(1));
        axis.set_integer(&user(), function::DEFER_MOVES, 0).unwrap();
        axis.move_velocity(&user(), 0.0, 1.0, 1.0).unwrap();
        assert_eq!(get.get_integer(function::DONE), Some(0));
    }

    #[test]
    fn deferred_move_waits_for_release() {
        let mut axis = MotorSimAxis::new(DEFAULT_LOW_LIMIT, DEFAULT_HI_LIMIT, 0.0, 0.0);
        axis.set_deferred_moves(&user(), true).unwrap();
        axis.move_absolute(&user(), 4.0, 0.0, 2.0, 1.0).unwrap();
        // While deferred, the axis must not have adopted the new endpoint.
        for _ in 0..20 {
            axis.process(0.05);
        }
        assert!(
            axis.last_status.position.abs() < 1e-6,
            "moved before release: {}",
            axis.last_status.position
        );
        axis.set_deferred_moves(&user(), false).unwrap();
        let (pos, done) = run_to_done(&mut axis, 0.05, 1000);
        assert!(done);
        assert!((pos - 4.0).abs() < 1e-3, "final position {pos}");
    }
}
