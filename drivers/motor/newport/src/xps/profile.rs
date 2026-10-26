//! XPS PVT (position-velocity-time) trajectory profiles.
//!
//! The I/O-free core of C `buildProfile` (XPSController.cpp:578-693): the
//! [`Profile`] model and [`Profile::generate`], which builds the XPS
//! trajectory-file text exactly as C does. [`super::profile_move`] reads the
//! profile from the motor-rs profile parameters, then uploads, verifies and
//! runs it.

use epics_libcom_rs::cfmt::CFloat;

/// Minimum acceleration/deceleration ramp time, seconds (C
/// `XPS_MIN_PROFILE_ACCEL_TIME`). Small scan velocities would otherwise ask the
/// XPS to accelerate almost instantly and trip a roundoff "acceleration too
/// high" error.
const XPS_MIN_PROFILE_ACCEL_TIME: f64 = 0.25;

/// The XPS reads the controller acceleration back over an ASCII link; C reduces
/// it 10% before computing ramp times to leave headroom for that roundoff
/// (XPSController.cpp:611-614).
const XPS_ACCEL_ROUNDOFF_MARGIN: f64 = 0.9;

/// One positioner's contribution to a profile: its name and the per-point
/// position array (device/engineering units, `positions.len() == num_points`).
#[derive(Clone, Debug)]
pub struct ProfileAxis {
    pub positioner: String,
    pub positions: Vec<f64>,
}

/// A multi-axis PVT profile for one XPS group.
#[derive(Clone, Debug)]
pub struct Profile {
    pub group: String,
    /// Per-point time array (seconds), `num_points` entries. Indexed exactly as
    /// C `profileTimes_`: entry `i` times point `i`.
    pub times: Vec<f64>,
    /// Participating positioners; every axis has `num_points` positions.
    pub axes: Vec<ProfileAxis>,
}

/// Result of [`Profile::generate`]: the trajectory-file text plus the pre/post
/// ramp displacements per axis (needed to place the motors at the true start
/// before execution — C `profilePreDistance_` / `profilePostDistance_`).
#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryFile {
    pub text: String,
    pub pre_distance: Vec<f64>,
    pub post_distance: Vec<f64>,
}

/// Why a profile could not be turned into a trajectory file.
#[derive(Clone, Debug, PartialEq)]
pub enum ProfileError {
    /// Fewer than two points, or a mismatched axis position-array length.
    Shape(String),
    /// An axis's controller max acceleration was non-positive.
    Acceleration(String),
    /// A profile time was non-positive (would make a segment velocity blow up).
    Time(String),
}

impl std::fmt::Display for ProfileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProfileError::Shape(m) => write!(f, "profile shape error: {m}"),
            ProfileError::Acceleration(m) => write!(f, "profile acceleration error: {m}"),
            ProfileError::Time(m) => write!(f, "profile time error: {m}"),
        }
    }
}

impl std::error::Error for ProfileError {}

impl Profile {
    /// Number of profile points.
    pub fn num_points(&self) -> usize {
        self.times.len()
    }

    /// Build the XPS trajectory-file text (C `buildProfile`, XPSController.cpp:578-693).
    ///
    /// `max_acceleration` gives each axis's controller S-gamma max acceleration
    /// (same order as `self.axes`), read via `PositionerSGammaParametersGet`.
    /// Each is reduced by `XPS_ACCEL_ROUNDOFF_MARGIN` before use, exactly as C.
    /// `None` is an axis C's `useAxis` turns off: it has no ramp, and every
    /// element moves it by zero at zero velocity.
    ///
    /// The file is: one leading acceleration element (ramp from rest to the
    /// first segment velocity), `num_points - 1` trajectory elements (each a
    /// relative displacement + averaged end velocity per axis), and one trailing
    /// deceleration element (ramp to rest). Each numeric field is formatted with
    /// C `%f` (six fraction digits).
    pub fn generate(
        &self,
        max_acceleration: &[Option<f64>],
    ) -> Result<TrajectoryFile, ProfileError> {
        let n = self.num_points();
        if n < 2 {
            return Err(ProfileError::Shape(format!(
                "need at least 2 points, got {n}"
            )));
        }
        if self.axes.is_empty() {
            return Err(ProfileError::Shape("no axes in profile".into()));
        }
        if max_acceleration.len() != self.axes.len() {
            return Err(ProfileError::Shape(format!(
                "max_acceleration has {} entries, expected {}",
                max_acceleration.len(),
                self.axes.len()
            )));
        }
        for (a, axis) in self.axes.iter().enumerate() {
            if axis.positions.len() != n {
                return Err(ProfileError::Shape(format!(
                    "axis {} ({}) has {} positions, expected {n}",
                    a,
                    axis.positioner,
                    axis.positions.len()
                )));
            }
        }
        // Times used as segment durations must be positive (C would divide by
        // them; the XPS also rejects "negative or null delta time").
        for (i, &t) in self.times.iter().enumerate() {
            if t <= 0.0 {
                return Err(ProfileError::Time(format!(
                    "time[{i}] = {} is not positive",
                    CFloat(t).g()
                )));
            }
        }

        // Per-axis ramp velocities into the first and out of the last segment.
        let mut pre_velocity = vec![0.0_f64; self.axes.len()];
        let mut post_velocity = vec![0.0_f64; self.axes.len()];
        let mut pre_time_max = 0.0_f64;
        let mut post_time_max = 0.0_f64;
        for (j, axis) in self.axes.iter().enumerate() {
            let Some(controller_accel) = max_acceleration[j] else {
                continue;
            };
            let max_accel = controller_accel * XPS_ACCEL_ROUNDOFF_MARGIN;
            if max_accel <= 0.0 {
                return Err(ProfileError::Acceleration(format!(
                    "axis {} ({}) max acceleration {} is not positive",
                    j,
                    axis.positioner,
                    CFloat(controller_accel).g()
                )));
            }
            pre_velocity[j] = (axis.positions[1] - axis.positions[0]) / self.times[0];
            post_velocity[j] = (axis.positions[n - 1] - axis.positions[n - 2]) / self.times[n - 1];
            pre_time_max = pre_time_max.max(pre_velocity[j].abs() / max_accel);
            post_time_max = post_time_max.max(post_velocity[j].abs() / max_accel);
        }
        pre_time_max = pre_time_max.max(XPS_MIN_PROFILE_ACCEL_TIME);
        post_time_max = post_time_max.max(XPS_MIN_PROFILE_ACCEL_TIME);

        let pre_distance: Vec<f64> = pre_velocity
            .iter()
            .map(|v| 0.5 * v * pre_time_max)
            .collect();
        let post_distance: Vec<f64> = post_velocity
            .iter()
            .map(|v| 0.5 * v * post_time_max)
            .collect();

        let mut text = String::new();

        // Leading acceleration element.
        text.push_str(&format!(
            "{pre_time_max:.6}",
            pre_time_max = CFloat(pre_time_max)
        ));
        for (pd, pv) in pre_distance.iter().zip(&pre_velocity) {
            text.push_str(&format!(
                ", {pd:.6}, {pv:.6}",
                pd = CFloat(*pd),
                pv = CFloat(*pv)
            ));
        }
        text.push('\n');

        // Trajectory elements (numElements = num_points - 1).
        let num_elements = n - 1;
        for i in 0..num_elements {
            let t0 = self.times[i];
            let t1 = if i < num_elements - 1 {
                self.times[i + 1]
            } else {
                t0
            };
            text.push_str(&format!("{:.6}", CFloat(self.times[i])));
            for (axis, used) in self.axes.iter().zip(max_acceleration) {
                let mut d0 = axis.positions[i + 1] - axis.positions[i];
                let d1 = if i < num_elements - 1 {
                    axis.positions[i + 2] - axis.positions[i + 1]
                } else {
                    d0
                };
                // Velocity is the displacement averaged either side of the point.
                let mut traj_vel = (d0 + d1) / (t0 + t1);
                if used.is_none() {
                    (d0, traj_vel) = (0.0, 0.0);
                }
                text.push_str(&format!(
                    ", {d0:.6}, {traj_vel:.6}",
                    d0 = CFloat(d0),
                    traj_vel = CFloat(traj_vel)
                ));
            }
            text.push('\n');
        }

        // Trailing deceleration element; final velocity is zero.
        text.push_str(&format!(
            "{post_time_max:.6}",
            post_time_max = CFloat(post_time_max)
        ));
        for pd in &post_distance {
            text.push_str(&format!(", {pd:.6}, {:.6}", CFloat(0.0), pd = CFloat(*pd)));
        }
        text.push('\n');

        Ok(TrajectoryFile {
            text,
            pre_distance,
            post_distance,
        })
    }
}

/// Resolve the PVT pulse-output window for `MultipleAxesPVTPulseOutputSet`
/// (C `executeProfile`, XPSController.cpp:975-1013). `times` is the profile's
/// per-point time array; `start_pulses`/`end_pulses` are 1-based user elements
/// (`numElements = times.len() - 1` real elements, `numElements + 1` addresses
/// the deceleration element); `num_pulses` is the number of pulses to spread
/// over that window. Returns the wire arguments
/// `(start_element, end_element, pulse_period)` — the element numbers are the
/// user values plus one to skip the acceleration element, and the period is
/// the window's total profile time divided by `num_pulses` (`0` pulses → `0`
/// period, as C).
pub fn pulse_output_window(
    times: &[f64],
    start_pulses: i32,
    end_pulses: i32,
    num_pulses: i32,
) -> Result<(i32, i32, f64), String> {
    let num_elements = times.len().saturating_sub(1) as i32;
    if start_pulses < 1
        || start_pulses > num_elements + 1
        || end_pulses < start_pulses
        || end_pulses > num_elements + 1
    {
        return Err("Error: start or end pulses outside valid range".into());
    }
    let last_time = end_pulses.min(num_elements);
    let time: f64 = (start_pulses..=last_time)
        .map(|i| times[(i - 1) as usize])
        .sum();
    let pulse_period = if num_pulses != 0 {
        time / f64::from(num_pulses)
    } else {
        0.0
    };
    Ok((start_pulses + 1, end_pulses + 1, pulse_period))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_axis(positions: &[f64]) -> Vec<ProfileAxis> {
        vec![ProfileAxis {
            positioner: "GROUP1.POS".into(),
            positions: positions.to_vec(),
        }]
    }

    #[test]
    fn rejects_too_few_points() {
        let p = Profile {
            group: "GROUP1".into(),
            times: vec![1.0],
            axes: one_axis(&[0.0]),
        };
        assert!(matches!(
            p.generate(&[Some(10.0)]),
            Err(ProfileError::Shape(_))
        ));
    }

    #[test]
    fn rejects_mismatched_position_len() {
        let p = Profile {
            group: "GROUP1".into(),
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 1.0]),
        };
        assert!(matches!(
            p.generate(&[Some(10.0)]),
            Err(ProfileError::Shape(_))
        ));
    }

    #[test]
    fn rejects_nonpositive_time_and_accel() {
        let p = Profile {
            group: "GROUP1".into(),
            times: vec![1.0, 0.0, 1.0],
            axes: one_axis(&[0.0, 1.0, 2.0]),
        };
        assert!(matches!(
            p.generate(&[Some(10.0)]),
            Err(ProfileError::Time(_))
        ));

        let p = Profile {
            group: "GROUP1".into(),
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 1.0, 2.0]),
        };
        assert!(matches!(
            p.generate(&[Some(0.0)]),
            Err(ProfileError::Acceleration(_))
        ));
    }

    #[test]
    fn generates_single_axis_uniform_trajectory() {
        // 3 points, unit spacing, unit times. numElements = 2.
        // maxAccel = 10 * 0.9 = 9.
        // preVelocity = (1-0)/1 = 1; preTime = 1/9 = 0.111.. < 0.25 -> preTimeMax = 0.25.
        // postVelocity = (2-1)/1 = 1; postTime likewise -> postTimeMax = 0.25.
        // preDistance = 0.5*1*0.25 = 0.125; postDistance = 0.5*1*0.25 = 0.125.
        let p = Profile {
            group: "GROUP1".into(),
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 1.0, 2.0]),
        };
        let out = p.generate(&[Some(10.0)]).unwrap();
        assert_eq!(out.pre_distance, vec![0.125]);
        assert_eq!(out.post_distance, vec![0.125]);

        // Element 0: t=1, D0=1, D1=1, trajVel=(1+1)/(1+1)=1.
        // Element 1 (last): t=1, D0=1, D1=D0=1, T1=T0=1, trajVel=1.
        let expected = "\
0.250000, 0.125000, 1.000000
1.000000, 1.000000, 1.000000
1.000000, 1.000000, 1.000000
0.250000, 0.125000, 0.000000
";
        assert_eq!(out.text, expected);
    }

    #[test]
    fn averages_velocity_across_uneven_segments() {
        // 3 points at 0, 2, 3; times all 1. Element 0: D0=2, D1=1,
        // trajVel=(2+1)/(1+1)=1.5. Element 1: D0=1, D1=1 -> trajVel=1.
        let p = Profile {
            group: "GROUP1".into(),
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 2.0, 3.0]),
        };
        let out = p.generate(&[Some(100.0)]).unwrap();
        let lines: Vec<&str> = out.text.lines().collect();
        // Middle element 0 carries the averaged 1.5 velocity.
        assert_eq!(lines[1], "1.000000, 2.000000, 1.500000");
        assert_eq!(lines[2], "1.000000, 1.000000, 1.000000");
    }

    #[test]
    fn two_axes_emit_paired_columns() {
        let p = Profile {
            group: "XY".into(),
            times: vec![1.0, 1.0],
            axes: vec![
                ProfileAxis {
                    positioner: "XY.X".into(),
                    positions: vec![0.0, 1.0],
                },
                ProfileAxis {
                    positioner: "XY.Y".into(),
                    positions: vec![0.0, 2.0],
                },
            ],
        };
        let out = p.generate(&[Some(100.0), Some(100.0)]).unwrap();
        // numElements = 1: a single trajectory element between the ramps.
        let lines: Vec<&str> = out.text.lines().collect();
        assert_eq!(lines.len(), 3);
        // Element line has one (D, V) pair per axis: X moves 1, Y moves 2.
        assert_eq!(lines[1], "1.000000, 1.000000, 1.000000, 2.000000, 2.000000");
        assert_eq!(out.pre_distance.len(), 2);
    }

    /// XPSController.cpp:592-686: an axis turned off by useAxis keeps its
    /// columns, all zero, and sizes no ramp.
    #[test]
    fn an_unused_axis_writes_zero_columns() {
        let p = Profile {
            group: "XY".into(),
            times: vec![1.0, 1.0, 1.0],
            axes: vec![
                ProfileAxis {
                    positioner: "XY.X".into(),
                    positions: vec![0.0, 1.0, 2.0],
                },
                ProfileAxis {
                    positioner: "XY.Y".into(),
                    positions: vec![0.0, 50.0, 100.0],
                },
            ],
        };
        let out = p.generate(&[Some(100.0), None]).unwrap();
        assert_eq!(
            out.text,
            "0.250000, 0.125000, 1.000000, 0.000000, 0.000000\n\
             1.000000, 1.000000, 1.000000, 0.000000, 0.000000\n\
             1.000000, 1.000000, 1.000000, 0.000000, 0.000000\n\
             0.250000, 0.125000, 0.000000, 0.000000, 0.000000\n"
        );
        assert_eq!((out.pre_distance[1], out.post_distance[1]), (0.0, 0.0));
    }

    #[test]
    fn pulse_window_matches_c_math() {
        // 5 points -> 4 elements timed by times[0..4].
        let times = [0.5, 0.5, 1.0, 2.0, 3.0];
        // Full window, one pulse per element: period = (0.5+0.5+1.0+2.0)/4.
        let (start, end, period) = pulse_output_window(&times, 1, 4, 4).unwrap();
        assert_eq!((start, end), (2, 5)); // +1 skips the acceleration element
        assert!((period - 1.0).abs() < 1e-12);
        // Sub-window elements 2..3: time = 0.5 + 1.0 over 3 pulses.
        let (start, end, period) = pulse_output_window(&times, 2, 3, 3).unwrap();
        assert_eq!((start, end), (3, 4));
        assert!((period - 0.5).abs() < 1e-12);
        // endPulses may address the deceleration element (numElements + 1);
        // its time is not part of the window sum (C caps lastTime).
        let (start, end, period) = pulse_output_window(&times, 1, 5, 4).unwrap();
        assert_eq!((start, end), (2, 6));
        assert!((period - 1.0).abs() < 1e-12);
        // Zero pulses -> zero period, as C.
        let (_, _, period) = pulse_output_window(&times, 1, 4, 0).unwrap();
        assert_eq!(period, 0.0);
    }

    #[test]
    fn pulse_window_rejects_out_of_range() {
        let times = [0.5, 0.5, 1.0];
        assert!(pulse_output_window(&times, 0, 2, 1).is_err()); // start < 1
        assert!(pulse_output_window(&times, 1, 4, 1).is_err()); // end > numElements+1
        assert!(pulse_output_window(&times, 2, 1, 1).is_err()); // end < start
    }
}
