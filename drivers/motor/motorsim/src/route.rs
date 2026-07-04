//! Single-axis trapezoidal trajectory solver, ported from
//! `motorMotorSim/motorSimApp/src/route.c` (Nick Rees, JCMT TCS design note
//! TCS/DN/10).
//!
//! The C library routes up to `NUM_AXES` axes with optional inter-axis
//! synchronization (`Tsync`) and end coast (`Tcoast`). The simulator drives it
//! with exactly one routed axis and `Tsync == Tcoast == 0`
//! (`motorSimAxis` constructor), so this port is the single-axis reduction:
//! the multi-axis `long_path`/`Tsync` machinery collapses (there is one axis,
//! it is always the long path, and no resync occurs). The four-phase path math
//! ([`find_path`], [`find_which_v2_sqrt`], [`route_demand`]) is transcribed
//! verbatim, including a latent C quirk noted at its site.
//!
//! Path phases (C `routeFindPath`): (1) accelerate at ±`Amax` for `t1`,
//! (2) coast at `v2` for `t2`, (3) decelerate at `Amax` for `t3`, (4) coast at
//! the final velocity `vf` for `t4`. Positions/velocities are dimensionless
//! (record EGU) — the simulator has no unit scaling.

/// C `IS_ZERO(a, scale)`: `|a| <= |2 * DBL_EPSILON * scale|`.
fn is_zero(a: f64, scale: f64) -> bool {
    a.abs() <= (2.0 * f64::EPSILON * scale).abs()
}

/// Unknowns bit mask for [`find_path`] (C `route_unknown_t`).
const V2: i32 = 1;
const T: i32 = 2;
const T2: i32 = 4;
const T4: i32 = 8;

/// Solver status (C `route_status_t`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Ok,
    BadParam,
    NegTime,
}

/// One four-phase path (C `path_t`).
#[derive(Clone, Copy, Debug, Default)]
pub struct Path {
    pub dist: f64,
    pub vi: f64,
    pub vf: f64,
    pub v2: f64,
    pub t1: f64,
    pub t2: f64,
    pub t3: f64,
    pub t4: f64,
    /// Total path time (C `path->T`).
    pub big_t: f64,
}

/// One axis demand: position and velocity at a time `t` (C `route_demand_t`
/// reduced to a single axis, with the valid time carried alongside).
#[derive(Clone, Copy, Debug, Default)]
pub struct Demand {
    pub t: f64,
    pub p: f64,
    pub v: f64,
}

/// C `routeDemand`: position and velocity at time `t` on `path`, integrating
/// backward from the endpoint (`t` is measured relative to the endpoint time,
/// so it is `<= 0` on the path). Returns `(position, velocity)`.
fn route_demand(path: &Path, t_in: f64) -> (f64, f64) {
    let accel1 = if path.t1 != 0.0 {
        (path.v2 - path.vi) / path.t1
    } else {
        0.0
    };
    let accel2 = if path.t3 != 0.0 {
        (path.vf - path.v2) / path.t3
    } else {
        0.0
    };

    let mut t = t_in;
    if t >= -path.t4 {
        return (path.vf * t, path.vf);
    }
    t += path.t4;
    let mut p = -path.vf * path.t4;
    if t >= -path.t3 {
        let v = path.vf + accel2 * t;
        p += 0.5 * (v + path.vf) * t;
        return (p, v);
    }
    t += path.t3;
    p -= 0.5 * (path.v2 + path.vf) * path.t3;
    if t >= -path.t2 {
        let v = path.v2;
        p += path.v2 * t;
        return (p, v);
    }
    t += path.t2;
    p -= path.v2 * path.t2;
    if t >= -path.t1 {
        let v = path.v2 + accel1 * t;
        p += 0.5 * (v + path.v2) * t;
        (p, v)
    } else {
        let v = path.vi;
        // Verbatim from C: this pre-motion branch (queried time before the
        // path started) uses `path->t2`, not `t1`, in the accel-phase term.
        // It is effectively unreachable in forward integration; kept
        // bug-for-bug.
        p += 0.5 * (path.vi + path.v2) * path.t2 + (t + path.t1) * path.vi;
        (p, v)
    }
}

/// C `routeFindWhichV2Sqrt`: pick the `v2` root of the path quadratic.
fn find_which_v2_sqrt(path: &mut Path, ai: f64, lin_term: f64, sqrt_term_in: f64, unknown: i32) {
    let mut sqrt_term = sqrt_term_in;
    if sqrt_term < 0.0 {
        sqrt_term = 0.0;
    }
    sqrt_term = sqrt_term.sqrt();

    if unknown == T2 {
        path.v2 = if ai > 0.0 {
            lin_term - sqrt_term
        } else {
            lin_term + sqrt_term
        };
    } else {
        path.v2 = if ai > 0.0 {
            lin_term + sqrt_term
        } else {
            lin_term - sqrt_term
        };
    }
}

/// C `routeFindPath`: solve the four-phase path given `accel` (= `Amax`) and a
/// bit mask of the two unknowns. Mutates `path` (`v2`, `t1..t4`, `big_t`).
fn find_path(path: &mut Path, accel: f64, unknowns: i32) -> Status {
    if accel <= 0.0 {
        return Status::BadParam;
    }

    let mut status = Status::Ok;

    if unknowns == (V2 | T) || unknowns == (V2 | T4) || unknowns == (V2 | T2) {
        let min_accel_time = ((path.vi - path.vf) / accel).abs();
        let mut max_t2_time = path.t2;
        let known = unknowns & !V2;

        let min_not_t2_dist = match known {
            T => 0.5 * (path.vi + path.vf) * min_accel_time + path.vf * path.t4,
            T4 => {
                0.5 * (path.vi + path.vf) * min_accel_time
                    + path.vf * (path.big_t - path.t2 - min_accel_time)
            }
            T2 => {
                let d = 0.5 * (path.vi + path.vf) * min_accel_time + path.vf * path.t4;
                max_t2_time = path.big_t - path.t4 - min_accel_time;
                d
            }
            _ => return Status::BadParam,
        };

        let vi_dist = min_not_t2_dist + path.vi * max_t2_time;
        let vf_dist = min_not_t2_dist + path.vf * max_t2_time;

        if path.dist == vi_dist {
            path.v2 = path.vi;
        } else if path.dist == vf_dist {
            path.v2 = path.vf;
        } else if (path.dist < vi_dist && path.dist > vf_dist)
            || (path.dist > vi_dist && path.dist < vf_dist)
        {
            // v2 intermediate between vi and vf.
            let ai = if path.vf > path.vi { accel } else { -accel };
            path.v2 = match known {
                T => {
                    (path.dist + 0.5 * (path.vi * path.vi - path.vf * path.vf) / ai
                        - path.vf * path.t4)
                        / max_t2_time
                }
                T4 => {
                    (path.dist + 0.5 * (path.vi - path.vf) * (path.vi - path.vf) / ai
                        - path.vf * (path.big_t - path.t2))
                        / max_t2_time
                }
                T2 => {
                    (path.dist + 0.5 * (path.vi * path.vi - path.vf * path.vf) / ai
                        - path.vf * path.t4)
                        / max_t2_time
                }
                _ => path.v2,
            };
        } else {
            // dist outside [vi_dist, vf_dist]: solve the quadratic for v2.
            let ai = if path.dist < vi_dist && path.dist < vf_dist {
                -accel
            } else {
                accel
            };
            let (lin_term, sqrt_term) = match known {
                T => {
                    let lin = -0.5 * ai * path.t2;
                    let s = 0.5 * path.t2 * ai;
                    let sq = s * s
                        + 0.5 * (path.vi * path.vi + path.vf * path.vf)
                        + ai * (path.dist - path.vf * path.t4);
                    (lin, sq)
                }
                T4 => {
                    let lin = path.vf - 0.5 * ai * path.t2;
                    let s = 0.5 * ai * path.t2;
                    let sq = s * s
                        + 0.5 * (path.vi - path.vf) * (path.vi - path.vf)
                        + ai * (path.dist - path.vf * path.big_t);
                    (lin, sq)
                }
                T2 => {
                    let lin = 0.5 * (ai * (path.big_t - path.t4) + path.vi + path.vf);
                    let sq = lin * lin
                        - 0.5 * (path.vi * path.vi + path.vf * path.vf)
                        - ai * (path.dist - path.vf * path.t4);
                    (lin, sq)
                }
                _ => return Status::BadParam,
            };
            find_which_v2_sqrt(path, ai, lin_term, sqrt_term, known);
        }

        if status == Status::Ok {
            path.t1 = ((path.v2 - path.vi) / accel).abs();
            path.t3 = ((path.vf - path.v2) / accel).abs();
            if unknowns & T != 0 {
                path.big_t = path.t1 + path.t2 + path.t3 + path.t4;
            }
            if unknowns & T4 != 0 {
                path.t4 = path.big_t - (path.t1 + path.t2 + path.t3);
            }
            if unknowns & T2 != 0 {
                path.t2 = path.big_t - (path.t1 + path.t3 + path.t4);
            }
        }
    } else if unknowns == (T | T4) || unknowns == (T | T2) || unknowns == (T2 | T4) {
        // v2 is known: t1 and t3 are trivial.
        path.t1 = ((path.v2 - path.vi) / accel).abs();
        path.t3 = ((path.vf - path.v2) / accel).abs();
        let dist =
            path.dist - 0.5 * ((path.vi + path.v2) * path.t1 + (path.v2 + path.vf) * path.t3);

        if unknowns & T != 0 {
            if unknowns & T4 != 0 && path.vf != 0.0 {
                path.t4 = (dist - path.v2 * path.t2) / path.vf;
            } else if unknowns & T2 != 0 && path.v2 != 0.0 {
                path.t2 = (dist - path.vf * path.t4) / path.v2;
            } else {
                status = Status::BadParam;
            }
            path.big_t = path.t1 + path.t2 + path.t3 + path.t4;
        } else if path.v2 != path.vf {
            path.t2 = (dist - path.vf * (path.big_t - path.t1 - path.t3)) / (path.v2 - path.vf);
            path.t4 = path.big_t - path.t1 - path.t2 - path.t3;
        } else {
            status = Status::BadParam;
        }
    } else {
        status = Status::BadParam;
    }

    // Test for positive times (C's negative-time cleanup).
    if status == Status::Ok
        && (path.t1 < 0.0 || path.t2 < 0.0 || path.t3 < 0.0 || path.t4 < 0.0 || path.big_t < 0.0)
    {
        if is_zero(path.t1, path.big_t) {
            path.t1 = 0.0;
        }
        if is_zero(path.t2, path.big_t) {
            path.t2 = 0.0;
        }
        if is_zero(path.t3, path.big_t) {
            path.t3 = 0.0;
        }
        if is_zero(path.t4, path.big_t) {
            path.t4 = 0.0;
        }
        if path.t1 < 0.0 || path.t2 < 0.0 || path.t3 < 0.0 || path.t4 < 0.0 || path.big_t < 0.0 {
            status = Status::NegTime;
            if path.t1 < 0.0 {
                path.t1 = 0.0;
            }
            if path.t2 < 0.0 {
                path.t2 = 0.0;
            }
            if path.t3 < 0.0 {
                path.t3 = 0.0;
            }
            if path.t4 < 0.0 {
                path.t4 = 0.0;
            }
            if path.big_t < 0.0 {
                path.big_t = 0.0;
            }
        }
    }

    if status == Status::Ok
        && !is_zero(
            path.big_t - (path.t1 + path.t2 + path.t3 + path.t4),
            path.big_t,
        )
    {
        status = Status::NegTime;
    }

    status
}

/// C `routeFindPathWithVmax`: solve for the path, then if the coast velocity
/// exceeds `Vmax`, clamp it and re-solve with `t2` as the unknown.
fn find_path_with_vmax(path: &mut Path, amax: f64, vmax: f64, unknown: i32) -> Status {
    path.t2 = 0.0;
    let mut status = find_path(path, amax, V2 | unknown);
    if status == Status::Ok && path.v2.abs() > vmax {
        path.v2 = if path.v2 >= 0.0 { vmax } else { -vmax };
        status = find_path(path, amax, T2 | unknown);
    }
    status
}

/// Reroute mode (C `route_reroute_t`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reroute {
    /// Force a full recalculation (C `ROUTE_NEW_ROUTE`).
    New,
    /// Turn routing off: coast straight to the endpoint (C `ROUTE_NO_NEW_ROUTE`).
    Off,
    /// Calculate an acceptable route to the next point (C `ROUTE_CALC_ROUTE`).
    Calc,
}

/// Single-axis route state (C `route_t` reduced to one routed axis with
/// `Tsync == Tcoast == 0`).
pub struct Route {
    amax: f64,
    vmax: f64,
    demand: Demand,
    endp: Demand,
    path: Path,
}

impl Route {
    /// C `routeNew`: initialise from the starting demand. `Vmax`/`Amax` must be
    /// positive and `Vmax` must exceed the initial speed. Returns `None` on bad
    /// parameters (C returns `NULL`).
    pub fn new(demand: Demand, amax: f64, vmax: f64) -> Option<Self> {
        if !(amax > 0.0 && vmax > 0.0 && vmax > demand.v.abs()) {
            return None;
        }
        let path = Path {
            vi: demand.v,
            v2: demand.v,
            vf: demand.v,
            ..Path::default()
        };
        Some(Self {
            amax,
            vmax,
            demand,
            endp: demand,
            path,
        })
    }

    /// C `routeSetParams` for the single axis: update `Amax`/`Vmax` (rejecting
    /// values that violate the same checks as construction).
    pub fn set_params(&mut self, amax: f64, vmax: f64) -> Status {
        if amax > 0.0 && vmax > 0.0 && vmax > self.demand.v.abs() {
            self.amax = amax;
            self.vmax = vmax;
            Status::Ok
        } else {
            Status::BadParam
        }
    }

    /// Current maximum acceleration.
    pub fn amax(&self) -> f64 {
        self.amax
    }

    /// Current maximum velocity.
    pub fn vmax(&self) -> f64 {
        self.vmax
    }

    /// C `routeFind` (single axis, `Tsync == Tcoast == 0`): compute the demand
    /// at time `nextp.t`, updating `nextp.{p,v}`. `endp.t` is set to the time
    /// the endpoint will actually be reached.
    pub fn find(&mut self, reroute: Reroute, endp: &mut Demand, nextp: &mut Demand) -> Status {
        let mut ret_status = Status::Ok;

        if reroute == Reroute::Off {
            self.demand.v = endp.v;
            self.demand.p = endp.p - (nextp.t - self.demand.t) * endp.v;
        }

        // Is the previous path still usable?
        let mut old_path_ok = reroute == Reroute::Calc && is_zero(endp.t - self.endp.t, endp.t);
        if old_path_ok {
            old_path_ok = is_zero(endp.p - self.endp.p, 40.0)
                && (endp.v - self.endp.v).abs() < (self.vmax * 1.0e-10);
        }

        if !old_path_ok {
            self.path.dist = endp.p - self.demand.p;
            self.path.vi = self.demand.v;
            self.path.vf = endp.v;
            self.path.t2 = 0.0;
            self.path.t4 = 0.0; // Tcoast == 0
            self.path.big_t = endp.t - self.demand.t;

            // Will the route complete within the next coast period? (Tcoast 0.)
            let short_path = reroute != Reroute::New && nextp.t >= endp.t;
            let mut status = Status::Ok;
            if short_path {
                status = find_path_with_vmax(&mut self.path, self.amax, self.vmax, T4);
            }

            if !short_path || status != Status::Ok {
                self.path.t4 = 0.0;
                status = find_path_with_vmax(&mut self.path, self.amax, self.vmax, T);
                match status {
                    Status::Ok | Status::NegTime => {}
                    Status::BadParam => return status,
                }
                // Single axis: it is always the long path; Tsync == 0.
                endp.t = self.demand.t + self.path.big_t;
                // Recalculation loop is skipped (j == long_path, Tsync == 0).
                if status != Status::Ok {
                    ret_status = status;
                }
            }
        }

        // Evaluate the demand at the next time.
        let (p, v) = route_demand(&self.path, nextp.t - endp.t);
        nextp.p = p + endp.p;
        nextp.v = v;

        self.demand = *nextp;
        self.endp = *endp;
        ret_status
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Integrate a route forward in fixed steps and return the sampled
    /// (time, position, velocity) triples, mirroring `motorSimAxis::process`.
    fn integrate(route: &mut Route, mut endp: Demand, steps: usize, dt: f64) -> Vec<Demand> {
        let mut next = route.endp; // start at the route's current demand
        let mut out = Vec::new();
        let mut reroute = Reroute::New;
        for _ in 0..steps {
            next.t += dt;
            route.find(reroute, &mut endp, &mut next);
            reroute = Reroute::Calc;
            out.push(next);
        }
        out
    }

    #[test]
    fn move_from_rest_reaches_endpoint() {
        // Start at 0, move to 10, Vmax=2, Amax=1.
        let start = Demand {
            t: 0.0,
            p: 0.0,
            v: 0.0,
        };
        let mut route = Route::new(start, 1.0, 2.0).unwrap();
        let endp = Demand {
            t: 0.0,
            p: 10.0,
            v: 0.0,
        };
        let samples = integrate(&mut route, endp, 400, 0.05);

        let last = samples.last().unwrap();
        assert!((last.p - 10.0).abs() < 1e-6, "final pos {}", last.p);
        assert!(last.v.abs() < 1e-6, "final vel {}", last.v);
        // Peak speed never exceeds Vmax (allow rounding).
        let peak = samples.iter().map(|d| d.v.abs()).fold(0.0, f64::max);
        assert!(peak <= 2.0 + 1e-6, "peak speed {peak}");
        assert!(peak > 1.0, "should reach coast speed, got {peak}");
    }

    #[test]
    fn short_move_is_triangular_under_vmax() {
        // Distance so small the profile never reaches Vmax (triangular).
        let start = Demand {
            t: 0.0,
            p: 0.0,
            v: 0.0,
        };
        let mut route = Route::new(start, 1.0, 100.0).unwrap();
        let endp = Demand {
            t: 0.0,
            p: 1.0,
            v: 0.0,
        };
        let samples = integrate(&mut route, endp, 200, 0.02);
        let last = samples.last().unwrap();
        assert!((last.p - 1.0).abs() < 1e-6, "final pos {}", last.p);
        let peak = samples.iter().map(|d| d.v.abs()).fold(0.0, f64::max);
        // Triangular peak = sqrt(dist*Amax) = 1.0 for dist=1, Amax=1.
        assert!(peak < 1.5, "triangular peak should be ~1, got {peak}");
    }

    #[test]
    fn monotonic_forward_progress() {
        let start = Demand {
            t: 0.0,
            p: 0.0,
            v: 0.0,
        };
        let mut route = Route::new(start, 1.0, 2.0).unwrap();
        let endp = Demand {
            t: 0.0,
            p: 5.0,
            v: 0.0,
        };
        let samples = integrate(&mut route, endp, 300, 0.05);
        for pair in samples.windows(2) {
            assert!(
                pair[1].p >= pair[0].p - 1e-9,
                "position went backward: {} -> {}",
                pair[0].p,
                pair[1].p
            );
        }
    }

    #[test]
    fn rejects_bad_parameters() {
        let start = Demand {
            t: 0.0,
            p: 0.0,
            v: 5.0,
        };
        // Vmax must exceed the initial speed.
        assert!(Route::new(start, 1.0, 2.0).is_none());
        assert!(Route::new(start, 0.0, 10.0).is_none());
        assert!(Route::new(start, 1.0, 10.0).is_some());
    }
}
