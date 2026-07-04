//! Simulated motor axis, ported from
//! `motorMotorSim/motorSimApp/src/motorSimDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` driver).
//!
//! Each axis owns a [`Route`] trajectory ([`crate::route`]) and integrates its
//! motion forward in real time. The C driver runs a dedicated 0.1 s background
//! thread (`motorSimTask`) that calls `process(delta)`; the asyn-rs port folds
//! that into [`poll`](AsynMotor::poll), whose cadence (the record's
//! moving/idle poll interval) supplies the elapsed `delta` from a monotonic
//! clock. There is no hardware and no unit scaling — positions and velocities
//! are dimensionless record EGU.
//!
//! ## Deviations from C (documented)
//!
//! - The 0.1 s task + its `[DELTA/4, 4·DELTA]` clock-sanity window is replaced
//!   by integrating the true elapsed time each poll (the poll cadence *is* the
//!   integration step), so no fixed-step window is imposed.
//! - `movesDeferred` is controller-wide in C (`writeInt32(motorDeferMoves)`
//!   applies every axis's deferred move); here it is per-axis, which is
//!   equivalent for the simulator (its axes have no inter-axis coupling).
//! - The post-move delay (`motorPostMoveDelay_`) is owned by the record layer
//!   in this stack, so the axis reports `done` as soon as it stops.

use std::time::Instant;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::user::AsynUser;

use crate::route::{Demand, Reroute, Route};

/// C `DEFAULT_LOW_LIMIT` / `DEFAULT_HI_LIMIT`.
pub const DEFAULT_LOW_LIMIT: f64 = -10000.0;
pub const DEFAULT_HI_LIMIT: f64 = 10000.0;
/// C `DEFAULT_HOME` / `DEFAULT_START`.
pub const DEFAULT_HOME: f64 = 0.0;
pub const DEFAULT_START: f64 = 0.0;

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
    last_poll: Option<Instant>,
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
            last_poll: None,
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
            direction: self.nextpoint.v > 0.0,
            high_limit: self.nextpoint.p >= self.hi_hard_limit,
            low_limit: self.nextpoint.p <= self.low_hard_limit,
            home: self.nextpoint.p == self.home,
            homed: self.homed,
            ..MotorStatus::default()
        }
    }
}

impl AsynMotor for MotorSimAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
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
        self.do_move(distance, true, velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
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
        let status = self.set_velocity(if forward { velocity } else { -velocity }, acceleration);
        self.homing = true;
        self.homed = false;
        status
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let _ = self.set_velocity(0.0, acceleration);
        self.deferred_move = false;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.enc_offset = position - self.nextpoint.p;
        Ok(())
    }

    fn set_deferred_moves(&mut self, _user: &AsynUser, defer: bool) -> AsynResult<()> {
        // C processDeferredMoves, reduced to this axis (module Deviations).
        if !defer && self.moves_deferred && self.deferred_move {
            let position = self.deferred_position;
            let blocked = (self.nextpoint.p >= self.hi_hard_limit && position > self.nextpoint.p)
                || (self.nextpoint.p <= self.low_hard_limit && position < self.nextpoint.p);
            if !blocked {
                self.endpoint.p = position - self.enc_offset;
                self.endpoint.v = 0.0;
                self.deferred_move = false;
            }
        }
        self.moves_deferred = defer;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let now = Instant::now();
        if let Some(prev) = self.last_poll {
            let delta = now.duration_since(prev).as_secs_f64();
            if delta > 0.0 {
                self.process(delta);
            }
        }
        self.last_poll = Some(now);
        Ok(self.last_status.clone())
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
