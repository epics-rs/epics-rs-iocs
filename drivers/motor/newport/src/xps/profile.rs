//! XPS PVT (position-velocity-time) trajectory profiles.
//!
//! Driver-private port of the profile half of `XPSController` (`buildProfile` /
//! `runProfile`, XPSController.cpp:532-1090). The epics-rs motor framework has
//! no controller-level profile-array subsystem (the C `asynMotorController`
//! base plus the motor module's `profileMove` record database), so the whole
//! feature lives in the driver and is driven by iocsh commands rather than the
//! C record interface.
//!
//! This module holds the parity-critical, I/O-free core: the [`Profile`] model
//! and [`Profile::generate`], which builds the XPS trajectory-file text exactly
//! as C `buildProfile` does. FTP upload, verification and execution live in the
//! controller/ioc layers on top of this.
//!
//! # Simplification vs. C
//!
//! C distinguishes `useAxis` (an axis in the group but turned off for this
//! scan, written as zero displacement/velocity) from in-group active axes.
//! Here every axis in a [`Profile`] is active and in the group — the caller
//! defines exactly the participating positioners and their position arrays — so
//! the `useAxis==0` zero-fill path is not modelled.

/// Minimum acceleration/deceleration ramp time, seconds (C
/// `XPS_MIN_PROFILE_ACCEL_TIME`). Small scan velocities would otherwise ask the
/// XPS to accelerate almost instantly and trip a roundoff "acceleration too
/// high" error.
const XPS_MIN_PROFILE_ACCEL_TIME: f64 = 0.25;

/// The XPS reads the controller acceleration back over an ASCII link; C reduces
/// it 10% before computing ramp times to leave headroom for that roundoff
/// (XPSController.cpp:611-614).
const XPS_ACCEL_ROUNDOFF_MARGIN: f64 = 0.9;

/// Whether execute-time moves to the trajectory start are absolute or relative
/// (C `PROFILE_MOVE_MODE_ABSOLUTE` / `_RELATIVE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveMode {
    Absolute,
    Relative,
}

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
    pub move_mode: MoveMode,
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

    /// Parse a profile from CSV text for the group's `positioners` (given in the
    /// order the trajectory columns map to — the controller's registration
    /// order). Each non-empty, non-`#`-comment line is one point:
    /// `time, pos_0, pos_1, ...` with one position column per positioner, in
    /// device (positioner) units and seconds. Blank lines and `#` comments are
    /// skipped. At least two points are required.
    pub fn from_csv(
        group: &str,
        move_mode: MoveMode,
        positioners: &[String],
        csv: &str,
    ) -> Result<Profile, String> {
        if positioners.is_empty() {
            return Err(format!("group '{group}' has no positioners"));
        }
        let expected_cols = positioners.len() + 1;
        let mut times = Vec::new();
        let mut columns: Vec<Vec<f64>> = vec![Vec::new(); positioners.len()];
        for (lineno, raw) in csv.lines().enumerate() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let fields: Vec<f64> = line
                .split(',')
                .map(|f| {
                    f.trim()
                        .parse::<f64>()
                        .map_err(|_| format!("line {}: '{}' is not a number", lineno + 1, f.trim()))
                })
                .collect::<Result<_, _>>()?;
            if fields.len() != expected_cols {
                return Err(format!(
                    "line {}: expected {expected_cols} columns (time + {} positioners), got {}",
                    lineno + 1,
                    positioners.len(),
                    fields.len()
                ));
            }
            times.push(fields[0]);
            for (col, value) in columns.iter_mut().zip(&fields[1..]) {
                col.push(*value);
            }
        }
        if times.len() < 2 {
            return Err(format!(
                "profile needs at least 2 points, got {}",
                times.len()
            ));
        }
        let axes = positioners
            .iter()
            .zip(columns)
            .map(|(positioner, positions)| ProfileAxis {
                positioner: positioner.clone(),
                positions,
            })
            .collect();
        Ok(Profile {
            group: group.to_string(),
            move_mode,
            times,
            axes,
        })
    }

    /// Build the XPS trajectory-file text (C `buildProfile`, XPSController.cpp:578-693).
    ///
    /// `max_acceleration` gives each axis's controller S-gamma max acceleration
    /// (same order as `self.axes`), read via `PositionerSGammaParametersGet`.
    /// Each is reduced by [`XPS_ACCEL_ROUNDOFF_MARGIN`] before use, exactly as C.
    ///
    /// The file is: one leading acceleration element (ramp from rest to the
    /// first segment velocity), `num_points - 1` trajectory elements (each a
    /// relative displacement + averaged end velocity per axis), and one trailing
    /// deceleration element (ramp to rest). Each numeric field is formatted with
    /// C `%f` (six fraction digits).
    pub fn generate(&self, max_acceleration: &[f64]) -> Result<TrajectoryFile, ProfileError> {
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
                    "time[{i}] = {t} is not positive"
                )));
            }
        }

        // Per-axis ramp velocities into the first and out of the last segment.
        let mut pre_velocity = vec![0.0_f64; self.axes.len()];
        let mut post_velocity = vec![0.0_f64; self.axes.len()];
        let mut pre_time_max = 0.0_f64;
        let mut post_time_max = 0.0_f64;
        for (j, axis) in self.axes.iter().enumerate() {
            let max_accel = max_acceleration[j] * XPS_ACCEL_ROUNDOFF_MARGIN;
            if max_accel <= 0.0 {
                return Err(ProfileError::Acceleration(format!(
                    "axis {} ({}) max acceleration {} is not positive",
                    j, axis.positioner, max_acceleration[j]
                )));
            }
            pre_velocity[j] = (axis.positions[1] - axis.positions[0]) / self.times[0];
            pre_time_max = pre_time_max.max(pre_velocity[j].abs() / max_accel);
            post_velocity[j] = (axis.positions[n - 1] - axis.positions[n - 2]) / self.times[n - 1];
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
        text.push_str(&format!("{pre_time_max:.6}"));
        for (pd, pv) in pre_distance.iter().zip(&pre_velocity) {
            text.push_str(&format!(", {pd:.6}, {pv:.6}"));
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
            text.push_str(&format!("{:.6}", self.times[i]));
            for axis in &self.axes {
                let d0 = axis.positions[i + 1] - axis.positions[i];
                let d1 = if i < num_elements - 1 {
                    axis.positions[i + 2] - axis.positions[i + 1]
                } else {
                    d0
                };
                // Velocity is the displacement averaged either side of the point.
                let traj_vel = (d0 + d1) / (t0 + t1);
                text.push_str(&format!(", {d0:.6}, {traj_vel:.6}"));
            }
            text.push('\n');
        }

        // Trailing deceleration element; final velocity is zero.
        text.push_str(&format!("{post_time_max:.6}"));
        for pd in &post_distance {
            text.push_str(&format!(", {pd:.6}, {:.6}", 0.0));
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
        return Err("start or end pulses outside valid range".into());
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
            move_mode: MoveMode::Absolute,
            times: vec![1.0],
            axes: one_axis(&[0.0]),
        };
        assert!(matches!(p.generate(&[10.0]), Err(ProfileError::Shape(_))));
    }

    #[test]
    fn rejects_mismatched_position_len() {
        let p = Profile {
            group: "GROUP1".into(),
            move_mode: MoveMode::Absolute,
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 1.0]),
        };
        assert!(matches!(p.generate(&[10.0]), Err(ProfileError::Shape(_))));
    }

    #[test]
    fn rejects_nonpositive_time_and_accel() {
        let p = Profile {
            group: "GROUP1".into(),
            move_mode: MoveMode::Absolute,
            times: vec![1.0, 0.0, 1.0],
            axes: one_axis(&[0.0, 1.0, 2.0]),
        };
        assert!(matches!(p.generate(&[10.0]), Err(ProfileError::Time(_))));

        let p = Profile {
            group: "GROUP1".into(),
            move_mode: MoveMode::Absolute,
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 1.0, 2.0]),
        };
        assert!(matches!(
            p.generate(&[0.0]),
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
            move_mode: MoveMode::Absolute,
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 1.0, 2.0]),
        };
        let out = p.generate(&[10.0]).unwrap();
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
            move_mode: MoveMode::Relative,
            times: vec![1.0, 1.0, 1.0],
            axes: one_axis(&[0.0, 2.0, 3.0]),
        };
        let out = p.generate(&[100.0]).unwrap();
        let lines: Vec<&str> = out.text.lines().collect();
        // Middle element 0 carries the averaged 1.5 velocity.
        assert_eq!(lines[1], "1.000000, 2.000000, 1.500000");
        assert_eq!(lines[2], "1.000000, 1.000000, 1.000000");
    }

    #[test]
    fn parses_csv_skipping_comments_and_blanks() {
        let csv = "\
# a two-axis scan
0.5, 0.0, 10.0

0.5, 1.0, 12.0
0.5, 2.0, 14.0
";
        let positioners = vec!["XY.X".to_string(), "XY.Y".to_string()];
        let p = Profile::from_csv("XY", MoveMode::Absolute, &positioners, csv).unwrap();
        assert_eq!(p.num_points(), 3);
        assert_eq!(p.times, vec![0.5, 0.5, 0.5]);
        assert_eq!(p.axes[0].positioner, "XY.X");
        assert_eq!(p.axes[0].positions, vec![0.0, 1.0, 2.0]);
        assert_eq!(p.axes[1].positions, vec![10.0, 12.0, 14.0]);
    }

    #[test]
    fn csv_rejects_wrong_column_count_and_too_few_points() {
        let positioners = vec!["XY.X".to_string()];
        // Two columns for a one-positioner group (expects time + 1 = 2)? That is
        // correct; make it wrong with three columns.
        let bad = "0.5, 1.0, 2.0\n0.5, 3.0, 4.0\n";
        assert!(Profile::from_csv("XY", MoveMode::Absolute, &positioners, bad).is_err());

        let short = "0.5, 1.0\n";
        let err = Profile::from_csv("XY", MoveMode::Absolute, &positioners, short).unwrap_err();
        assert!(err.contains("at least 2 points"), "got: {err}");
    }

    #[test]
    fn csv_rejects_non_numeric() {
        let positioners = vec!["XY.X".to_string()];
        let bad = "0.5, foo\n0.5, 1.0\n";
        assert!(Profile::from_csv("XY", MoveMode::Absolute, &positioners, bad).is_err());
    }

    #[test]
    fn two_axes_emit_paired_columns() {
        let p = Profile {
            group: "XY".into(),
            move_mode: MoveMode::Absolute,
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
        let out = p.generate(&[100.0, 100.0]).unwrap();
        // numElements = 1: a single trajectory element between the ramps.
        let lines: Vec<&str> = out.text.lines().collect();
        assert_eq!(lines.len(), 3);
        // Element line has one (D, V) pair per axis: X moves 1, Y moves 2.
        assert_eq!(lines[1], "1.000000, 1.000000, 1.000000, 2.000000, 2.000000");
        assert_eq!(out.pre_distance.len(), 2);
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
