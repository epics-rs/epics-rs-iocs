//! SPiiPlus profile moves: C `SPiiPlusController::initializeProfile`,
//! `buildProfile` and its helpers, `writeFloat64Array`, and
//! `SPiiPlusAxis::defineProfile` / `correctProfile`.
//!
//! Positions are in controller units: an axis's profile positions are its
//! steps times `resolution_`, as C's `defineProfile` leaves them.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{
    ControllerCtx, PROFILE_BUILD_BUSY, PROFILE_BUILD_DONE, PROFILE_EXECUTE_MOVE_START,
    PROFILE_MOVE_MODE_ABSOLUTE, PROFILE_MOVE_MODE_RELATIVE, PROFILE_STATUS_FAILURE,
    PROFILE_STATUS_SUCCESS, PROFILE_STATUS_UNDEFINED, index,
};

use crate::bincomm::Span;
use crate::run::RunPlan;
use crate::spiiplus::{AcsMotionParams, spiiplus_err};

/// C `MAX_ACCEL_SEGMENTS`.
const MAX_ACCEL_SEGMENTS: usize = 20;
/// C `SPIIPLUS_MAX_DC_AXES`: the `DC_DATA_n` arrays data collection fills.
const MAX_DC_AXES: usize = 8;

/// C `SPiiPlusController`'s pulse-output parameters (SPiiPlusDriver.cpp:
/// 120-131).
#[derive(Clone, Copy, Debug)]
pub(crate) struct PulseParams {
    pub(crate) mode: usize,
    pub(crate) pos: usize,
    pub(crate) axis: usize,
    pub(crate) peg_eng_enc_code: usize,
    pub(crate) peg_out_assign_code: usize,
    pub(crate) pouts_output_index: usize,
    pub(crate) pouts_bit_code: usize,
    pub(crate) pulse_width: usize,
}

/// One axis's part of C's profile: the `SPiiPlusAxis` arrays the base
/// class does not have.
#[derive(Clone, Debug, Default)]
struct AxisPath {
    /// C `profilePositionsUser_`: the positions as written, in user units.
    positions_user: Vec<f64>,
    /// C `fullProfilePositions_`: acceleration, profile and deceleration.
    full_positions: Vec<f64>,
    accel_positions: [f64; MAX_ACCEL_SEGMENTS],
    decel_positions: [f64; MAX_ACCEL_SEGMENTS],
    pre_distance: f64,
    post_distance: f64,
    start_pos: f64,
    flyback_pos: f64,
}

/// C `SPiiPlusController`'s profile state beyond the base's arrays.
#[derive(Debug, Default)]
pub(crate) struct SpiiProfile {
    /// C `profilePulsesUser_`, as written, in user units.
    pulses_user: Vec<f64>,
    /// C `profilePulses_`, in controller units.
    pulses: Vec<f64>,
    /// C `numPulses_`: how many pulse positions the last write carried.
    num_pulses_written: usize,
    axes: Vec<AxisPath>,
    /// C `fullProfileTimes_`.
    full_times: Vec<f64>,
    accel_times: [f64; MAX_ACCEL_SEGMENTS],
    decel_times: [f64; MAX_ACCEL_SEGMENTS],
    num_accel: usize,
    num_decel: usize,
    /// C `fullProfileSize_`.
    full_size: usize,
    /// C `profileAxes_`.
    pub(crate) profile_axes: Vec<usize>,
    pulse_start_pos: f64,
    pulse_end_pos: f64,
    pulse_spacing: f64,
    data_collection_interval: f64,
}

/// C `axesToString`: an axis, or a parenthesized list of them, as ACSPL+
/// takes axes.
pub(crate) fn axes_to_string(axes: &[usize]) -> String {
    let list = axes
        .iter()
        .map(usize::to_string)
        .collect::<Vec<_>>()
        .join(",");
    if axes.len() > 1 {
        format!("({list})")
    } else {
        list
    }
}

/// C `motorsToString`: 1-based motor numbers for the user's messages.
fn motors_to_string(axes: &[usize]) -> String {
    axes.iter()
        .map(|a| (a + 1).to_string())
        .collect::<Vec<_>>()
        .join(", ")
}

/// C `getNumAccelSegments`: one segment per 10 ms, at most
/// `MAX_ACCEL_SEGMENTS`.
fn num_accel_segments(time: f64) -> usize {
    ((time / 0.01).round() as i64).clamp(0, MAX_ACCEL_SEGMENTS as i64) as usize
}

fn int(ctx: &ControllerCtx<'_>, index: usize, addr: i32) -> i32 {
    ctx.base.get_int32_param(index, addr).unwrap_or_default()
}

fn float(ctx: &ControllerCtx<'_>, index: usize, addr: i32) -> f64 {
    ctx.base.get_float64_param(index, addr).unwrap_or_default()
}

fn set_int(ctx: &mut ControllerCtx<'_>, index: usize, value: i32) {
    let _ = ctx.base.set_int32_param(index, 0, value);
}

fn set_message(ctx: &mut ControllerCtx<'_>, index: usize, message: &str) {
    let _ = ctx.base.set_string_param(index, 0, message);
}

/// `asynPrint(pasynUserSelf, mask, "SPiiPlusController:func: ...")`.
fn print(ctx: &ControllerCtx<'_>, mask: TraceMask, func: &str, args: std::fmt::Arguments<'_>) {
    ctx.base
        .trace_print(mask, format_args!("SPiiPlusController:{func}:{args}"));
}

/// The acceleration and deceleration every profile axis shares.
struct AccelPlan {
    pre_time_max: f64,
    post_time_max: f64,
    /// Each profile axis's velocity into and out of the profile.
    velocities: Vec<(f64, f64)>,
}

/// What a build reads from the parameters (C `buildProfile`'s locals).
#[derive(Clone, Copy, Debug)]
struct BuildParams {
    move_mode: i32,
    num_points: i32,
    time_mode: i32,
    time_per_point: f64,
    accel_time: f64,
    pulse_mode: i32,
    pulse_axis: i32,
    num_pulses: i32,
    start_pulses: i32,
    end_pulses: i32,
}

impl AcsMotionParams {
    fn pulse_params(&self) -> Option<PulseParams> {
        self.pulse
    }

    /// C `SPiiPlusController::initializeProfile(maxProfilePoints,
    /// maxProfilePulses)`: the base's arrays, the full-profile arrays with
    /// room for the acceleration and deceleration, the pulse arrays, and
    /// the controller's `DC_DATA_n` arrays data collection records into.
    pub fn initialize_profile_pulses(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        max_points: usize,
        max_pulses: usize,
    ) -> AsynResult<()> {
        let full = max_points + 2 * MAX_ACCEL_SEGMENTS - 1;
        let num_axes = self.lock().num_axes();
        self.profile.full_times = vec![0.0; full];
        self.profile.axes = vec![
            AxisPath {
                positions_user: vec![0.0; max_points],
                full_positions: vec![0.0; full],
                ..AxisPath::default()
            };
            num_axes
        ];
        let status = ctx.initialize_profile(max_points);
        self.profile.pulses = vec![0.0; max_pulses];
        self.profile.pulses_user = vec![0.0; max_pulses];
        let ctrl = self.lock();
        for i in 1..=MAX_DC_AXES {
            // Delete the array first, in case maxProfilePoints changed.
            let _ = ctrl.comm().write_read_ack(&format!("#VGV DC_DATA_{i}"));
            let _ = ctrl
                .comm()
                .write_read_ack(&format!("GLOBAL REAL DC_DATA_{i} (3)({max_points})"));
        }
        status
    }

    /// C `SPiiPlusController::writeFloat64Array`: SPIIPLUS_PULSE_POS is
    /// kept as written until a build converts it. C copies past the array
    /// `maxProfilePulses` sized; the excess is dropped here.
    pub(crate) fn write_pulse_positions(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        user: &AsynUser,
        value: &[f64],
    ) -> AsynResult<()> {
        let addr = ctx.base.get_address(user.addr, user.reason)?;
        if !ctx.axes().any(|a| a == addr) {
            return Err(spiiplus_err(format!(
                "{} has no axis {addr}",
                ctx.base.port_name
            )));
        }
        let n = value.len().min(self.profile.pulses_user.len());
        self.profile.pulses_user[..n].copy_from_slice(&value[..n]);
        self.profile.num_pulses_written = value.len();
        Ok(())
    }

    /// Whether `reason` is SPIIPLUS_PULSE_POS.
    pub(crate) fn is_pulse_positions(&self, reason: usize) -> bool {
        self.pulse_params().is_some_and(|p| p.pos == reason)
    }

    /// C `SPiiPlusAxis::defineProfile`: the user positions kept for a
    /// relative build, then the base's conversion to steps, then steps to
    /// controller units.
    pub(crate) fn define_axis_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        addr: i32,
        positions: &[f64],
    ) -> AsynResult<()> {
        if let Some(path) = self.profile.axes.get_mut(addr as usize) {
            let n = positions.len().min(path.positions_user.len());
            path.positions_user[..n].copy_from_slice(&positions[..n]);
        }
        ctx.define_profile(addr, positions)?;
        let resolution = self.lock().resolution(addr as usize);
        if let Some(axis) = ctx.profile.axes.get_mut(addr as usize) {
            let n = positions.len().min(axis.positions.len());
            for p in &mut axis.positions[..n] {
                *p *= resolution;
            }
        }
        Ok(())
    }

    /// C `SPiiPlusAxis::correctProfile`: a relative profile's displacements
    /// reconverted from the kept user positions, without the offset the
    /// base's conversion applied.
    fn correct_profile(
        &self,
        ctx: &mut ControllerCtx<'_>,
        addr: usize,
        num_points: usize,
    ) -> AsynResult<()> {
        if num_points > ctx.profile.max_points {
            return Err(spiiplus_err(""));
        }
        let resolution = ctx
            .base
            .get_float64_param(index::REC_RESOLUTION, addr as i32);
        let direction = ctx.base.get_int32_param(index::REC_DIRECTION, addr as i32);
        let (Ok(resolution), Ok(direction)) = (resolution, direction) else {
            return Err(spiiplus_err(""));
        };
        if resolution == 0.0 {
            return Err(spiiplus_err(""));
        }
        let mut scale = 1.0 / resolution;
        if direction != 0 {
            scale = -scale;
        }
        let axis_resolution = self.lock().resolution(addr);
        let (Some(axis), Some(path)) =
            (ctx.profile.axes.get_mut(addr), self.profile.axes.get(addr))
        else {
            return Err(spiiplus_err(""));
        };
        for (p, u) in axis.positions[..num_points]
            .iter_mut()
            .zip(&path.positions_user)
        {
            *p = u * scale * axis_resolution;
        }
        Ok(())
    }

    /// C `SPiiPlusController::definePulses`: the first `num_pulses` pulse
    /// positions from user to controller units of `pulse_axis`.
    fn define_pulses(
        &mut self,
        ctx: &ControllerCtx<'_>,
        pulse_axis: usize,
        move_mode: i32,
        num_pulses: i32,
    ) -> AsynResult<()> {
        let first = self
            .profile
            .pulses_user
            .first()
            .copied()
            .unwrap_or_default();
        print(
            ctx,
            TraceMask::FLOW,
            "definePulses",
            format_args!(
                " pulseAxis={pulse_axis}, numPulses={num_pulses}, profilePulsesUser_[0]={first:.6}"
            ),
        );
        let num_pulses = usize::try_from(num_pulses).unwrap_or(usize::MAX);
        if num_pulses > self.profile.pulses.len() {
            return Err(spiiplus_err(""));
        }
        let addr = pulse_axis as i32;
        let resolution = ctx.base.get_float64_param(index::REC_RESOLUTION, addr);
        let offset = ctx.base.get_float64_param(index::REC_OFFSET, addr);
        let direction = ctx.base.get_int32_param(index::REC_DIRECTION, addr);
        let status = [
            resolution.as_ref().err(),
            offset.as_ref().err(),
            direction.as_ref().err(),
        ]
        .into_iter()
        .flatten()
        .fold(0, |status, e| status | e.status() as i32);
        let (resolution, offset, direction) = (
            resolution.unwrap_or_default(),
            offset.unwrap_or_default(),
            direction.unwrap_or_default(),
        );
        print(
            ctx,
            TraceMask::FLOW,
            "definePulses",
            format_args!(
                " status={status}, offset={offset:.6} direction={direction}, resolution={resolution:.6}"
            ),
        );
        if status != 0 || resolution == 0.0 {
            return Err(spiiplus_err(""));
        }
        let axis_resolution = self.lock().resolution(pulse_axis);
        let mut scale = 1.0 / resolution;
        if direction != 0 {
            scale = -scale;
        }
        // A relative profile's pulse positions are displacements: no offset.
        let offset = if move_mode == PROFILE_MOVE_MODE_ABSOLUTE {
            offset
        } else {
            0.0
        };
        let profile = &mut self.profile;
        for (p, u) in profile.pulses[..num_pulses]
            .iter_mut()
            .zip(&profile.pulses_user)
        {
            *p = (u - offset) * scale * axis_resolution;
        }
        print(
            ctx,
            TraceMask::FLOW,
            "definePulses",
            format_args!(
                " scale={scale:.6}, offset={offset:.6}, resolution_={axis_resolution:.6}, \
                 profilePulsesUser_[0]={first:.6}, profilePulses_[0]={:.6}",
                profile.pulses.first().copied().unwrap_or_default()
            ),
        );
        Ok(())
    }

    /// C `SPiiPlusController::buildProfile`: the user's profile with
    /// acceleration and deceleration segments added, and the pulse output
    /// it will drive. Always succeeds; the outcome is in
    /// PROFILE_BUILD_STATUS and PROFILE_BUILD_MESSAGE.
    pub(crate) fn build(&mut self, ctx: &mut ControllerCtx<'_>) -> AsynResult<()> {
        print(ctx, TraceMask::FLOW, "buildProfile", format_args!(" entry"));
        // C drops the base's status.
        let _ = ctx.build_profile();
        set_message(ctx, index::PROFILE_BUILD_MESSAGE, "");
        set_int(ctx, index::PROFILE_BUILD_STATE, PROFILE_BUILD_BUSY);
        set_int(ctx, index::PROFILE_BUILD_STATUS, PROFILE_STATUS_UNDEFINED);
        let _ = ctx.base.call_param_callbacks(0);

        let (ok, message) = match self.build_path(ctx) {
            Ok(message) => (true, message),
            Err(message) => (false, message),
        };

        let status = if ok {
            PROFILE_STATUS_SUCCESS
        } else {
            PROFILE_STATUS_FAILURE
        };
        set_int(ctx, index::PROFILE_BUILD_STATUS, status);
        set_message(ctx, index::PROFILE_BUILD_MESSAGE, &message);
        if ok {
            // Execute and readback have not happened since this build.
            set_int(ctx, index::PROFILE_EXECUTE_STATUS, PROFILE_STATUS_UNDEFINED);
            set_int(
                ctx,
                index::PROFILE_READBACK_STATUS,
                PROFILE_STATUS_UNDEFINED,
            );
        } else {
            print(
                ctx,
                TraceMask::ERROR,
                "buildProfile",
                format_args!(" {message}"),
            );
        }
        // PROFILE_BUILD is a busy record: cleared only now the build is done.
        set_int(ctx, index::PROFILE_BUILD, 0);
        set_int(ctx, index::PROFILE_BUILD_STATE, PROFILE_BUILD_DONE);
        let _ = ctx.base.call_param_callbacks(0);
        Ok(())
    }

    /// The body of C `buildProfile` up to its `done:` label: the message it
    /// leaves, `Err` where C sets `buildOK = false`.
    fn build_path(&mut self, ctx: &mut ControllerCtx<'_>) -> Result<String, String> {
        let p = BuildParams {
            move_mode: int(ctx, index::PROFILE_MOVE_MODE, 0),
            num_points: int(ctx, index::PROFILE_NUM_POINTS, 0),
            time_mode: int(ctx, index::PROFILE_TIME_MODE, 0),
            time_per_point: float(ctx, index::PROFILE_FIXED_TIME, 0),
            accel_time: float(ctx, index::PROFILE_ACCELERATION, 0),
            pulse_mode: self.pulse_params().map_or(0, |pp| int(ctx, pp.mode, 0)),
            pulse_axis: self.pulse_params().map_or(0, |pp| int(ctx, pp.axis, 0)),
            num_pulses: int(ctx, index::PROFILE_NUM_PULSES, 0),
            start_pulses: int(ctx, index::PROFILE_START_PULSES, 0),
            end_pulses: int(ctx, index::PROFILE_END_PULSES, 0),
        };
        let pulse_axis_raw_pos = float(ctx, index::POSITION, p.pulse_axis);

        let num_axes = self.lock().num_axes();
        let profile = &mut self.profile;
        profile.profile_axes.clear();
        profile.accel_times = [0.0; MAX_ACCEL_SEGMENTS];
        profile.decel_times = [0.0; MAX_ACCEL_SEGMENTS];
        for path in &mut profile.axes {
            path.accel_positions = [0.0; MAX_ACCEL_SEGMENTS];
            path.decel_positions = [0.0; MAX_ACCEL_SEGMENTS];
        }
        for i in 0..num_axes {
            let use_axis = int(ctx, index::PROFILE_USE_AXIS, i as i32);
            print(
                ctx,
                TraceMask::IO_DRIVER,
                "buildProfile",
                format_args!(" {i} axis will be used: {use_axis}"),
            );
            if use_axis != 0 {
                self.profile.profile_axes.push(i);
                if p.move_mode == PROFILE_MOVE_MODE_RELATIVE {
                    // defineProfile's conversion applied the offset, wrong
                    // for displacements. C drops the status.
                    let _ = self.correct_profile(ctx, i, p.num_points.max(0) as usize);
                }
            }
        }
        if self.profile.profile_axes.is_empty() {
            return Err("No axes selected".into());
        }
        print(
            ctx,
            TraceMask::IO_DRIVER,
            "buildProfile",
            format_args!(" axisList = {}", axes_to_string(&self.profile.profile_axes)),
        );
        let message = format!(
            "Selected axes: {}",
            motors_to_string(&self.profile.profile_axes)
        );
        set_message(ctx, index::PROFILE_BUILD_MESSAGE, &message);
        let _ = ctx.base.call_param_callbacks(0);

        // C's debugging lines, printed as errors.
        print(ctx, TraceMask::ERROR, "buildProfile", format_args!(""));
        for (name, value) in [
            ("\tnumPoints", p.num_points.to_string()),
            ("\tnumPulses", p.num_pulses.to_string()),
            ("\tstartPulses", p.start_pulses.to_string()),
            ("\tendPulses", p.end_pulses.to_string()),
            ("\ttimeMode", p.time_mode.to_string()),
            ("\ttimePerPoint", format!("{:.6}", p.time_per_point)),
            ("\taccelerationTime", format!("{:.6}", p.accel_time)),
            ("\tmoveMode", p.move_mode.to_string()),
            ("\tpulseMode", p.pulse_mode.to_string()),
            ("\tpulseAxis", p.pulse_axis.to_string()),
            (
                "\tnumPulses",
                format!(
                    "{}, numPulses_ = {}",
                    p.num_pulses, self.profile.num_pulses_written
                ),
            ),
        ] {
            print(
                ctx,
                TraceMask::ERROR,
                "buildProfile",
                format_args!("{name} = {value}"),
            );
        }

        // C indexes the profile, times and full-profile arrays with
        // NUM_POINTS unchecked.
        let n = usize::try_from(p.num_points).unwrap_or(0);
        if n < 2
            || n > ctx.profile.max_points
            || n + 2 * MAX_ACCEL_SEGMENTS - 1 > self.profile.full_times.len()
        {
            return Err("Error: number of points outside valid range".into());
        }
        // C dereferences a null axis for a pulse axis the controller lacks.
        let pulse_axis = usize::try_from(p.pulse_axis)
            .ok()
            .filter(|&a| a < num_axes)
            .ok_or_else(|| "Error: pulse axis outside valid range".to_string())?;

        let AccelPlan {
            pre_time_max,
            post_time_max,
            velocities,
        } = self.accel_plan(ctx, &p, n)?;
        let profile = &mut self.profile;
        profile.num_accel = num_accel_segments(pre_time_max);
        profile.num_decel = num_accel_segments(post_time_max);
        // C createAccDecTimes: equal segments.
        for t in &mut profile.accel_times[..profile.num_accel] {
            *t = pre_time_max / profile.num_accel as f64;
        }
        for t in &mut profile.decel_times[..profile.num_decel] {
            *t = post_time_max / profile.num_decel as f64;
        }

        for (j, &(pre_velocity, post_velocity)) in velocities.iter().enumerate() {
            let idx = self.profile.profile_axes[j];
            let positions = &ctx.profile.axes[idx].positions;
            let (num_accel, num_decel) = (self.profile.num_accel, self.profile.num_decel);
            let path = &mut self.profile.axes[idx];
            path.pre_distance = 0.5 * pre_velocity * pre_time_max;
            path.post_distance = 0.5 * post_velocity * post_time_max;
            if p.move_mode == PROFILE_MOVE_MODE_ABSOLUTE {
                path.start_pos = positions[0] - path.pre_distance;
                path.flyback_pos = positions[n - 1];
            } else {
                path.start_pos = -path.pre_distance;
                path.flyback_pos = -path.post_distance;
            }
            // C createAccDecPositions: constant acceleration from rest to
            // the first segment's velocity, and from the last to rest.
            let pre_accel = pre_velocity / pre_time_max;
            let post_accel = post_velocity / post_time_max;
            for i in 0..num_accel {
                let t2 = pre_time_max * (i + 1) as f64 / num_accel as f64;
                let t1 = pre_time_max * i as f64 / num_accel as f64;
                path.accel_positions[i] = if p.move_mode == PROFILE_MOVE_MODE_ABSOLUTE {
                    positions[0] - path.pre_distance + 0.5 * pre_accel * t2.powi(2)
                } else {
                    0.5 * pre_accel * (t2.powi(2) - t1.powi(2))
                };
            }
            for i in 0..num_decel {
                let t2 = post_time_max * (i + 1) as f64 / num_decel as f64;
                let t1 = post_time_max * i as f64 / num_decel as f64;
                path.decel_positions[i] = if p.move_mode == PROFILE_MOVE_MODE_ABSOLUTE {
                    positions[n - 1] + post_velocity * t2 - 0.5 * post_accel * t2.powi(2)
                } else {
                    post_velocity * (t2 - t1) - 0.5 * post_accel * (t2.powi(2) - t1.powi(2))
                };
            }
            let path = path.clone();
            print(
                ctx,
                TraceMask::ERROR,
                "buildProfile",
                format_args!("\taxis = {idx}"),
            );
            for (name, value) in [
                ("\t\tpreVelocity", pre_velocity),
                ("\t\tprofilePreDistance_", path.pre_distance),
                ("\t\tprofileStartPos_", path.start_pos),
                ("\t\tpostVelocity", post_velocity),
                ("\t\tprofilePostDistance_", path.post_distance),
                ("\t\tprofileFlybackPos_", path.flyback_pos),
            ] {
                print(
                    ctx,
                    TraceMask::ERROR,
                    "buildProfile",
                    format_args!("{name} = {value:.6}"),
                );
            }
        }

        self.assemble_full_profile(ctx, n);

        let pulse_axis_resolution = self.lock().resolution(pulse_axis);
        // C drops definePulses' status.
        let _ = self.define_pulses(ctx, pulse_axis, p.move_mode, p.num_pulses);
        let pulse_axis_pos = pulse_axis_raw_pos * pulse_axis_resolution;
        let pulse_positions = &ctx.profile.axes[pulse_axis].positions;
        let absolute = p.move_mode == PROFILE_MOVE_MODE_ABSOLUTE;
        let profile = &mut self.profile;
        match p.pulse_mode {
            // Fixed: numPulses evenly over the profile, by PEG_I.
            0 => {
                let total = if absolute {
                    profile.pulse_start_pos = pulse_positions[0];
                    profile.pulse_end_pos = pulse_positions[n - 1];
                    pulse_positions[n - 1] - pulse_positions[0]
                } else {
                    let total: f64 = pulse_positions[..n - 1].iter().sum();
                    profile.pulse_start_pos = pulse_axis_pos;
                    profile.pulse_end_pos = pulse_axis_pos + total;
                    total
                };
                profile.pulse_spacing = total / (f64::from(p.num_pulses) - 1.0);
            }
            // Array: the pulse positions, by PEG_R, sent now so the execute
            // does not wait on them.
            1 => {
                if !absolute {
                    let mut current = pulse_axis_pos;
                    let count = (p.num_pulses.max(0) as usize).min(profile.pulses.len());
                    for pulse in &mut profile.pulses[..count] {
                        *pulse += current;
                        current = *pulse;
                    }
                }
                // All maxProfilePulses: pulsePos may not exist yet, and the
                // user may raise numPulses after the build.
                let span = Span::one(0, profile.pulses.len() as i32 - 1);
                let sent =
                    self.lock()
                        .comm()
                        .put_double_array(&self.profile.pulses, "pulsePos", span);
                if let Err(e) = sent {
                    return Err(format!(
                        "Error writing pulse positions, status={}\n",
                        e.status() as i32
                    ));
                }
            }
            // TrajPts: numPulses evenly from point startPulses to point
            // endPulses, by PEG_I.
            2 => {
                // C indexes the profile with them unchecked.
                let range = usize::try_from(p.start_pulses)
                    .ok()
                    .zip(usize::try_from(p.end_pulses).ok())
                    .filter(|&(s, e)| s < pulse_positions.len() && e < pulse_positions.len());
                let Some((start, end)) = range else {
                    return Err("Error: start or end pulses outside valid range".into());
                };
                if absolute {
                    profile.pulse_start_pos = pulse_positions[start];
                    profile.pulse_end_pos = pulse_positions[end];
                } else {
                    profile.pulse_start_pos =
                        pulse_axis_pos + pulse_positions[..start].iter().sum::<f64>();
                    profile.pulse_end_pos = profile.pulse_start_pos
                        + pulse_positions
                            .get(start..end)
                            .unwrap_or(&[])
                            .iter()
                            .sum::<f64>();
                }
                profile.pulse_spacing = (profile.pulse_end_pos - profile.pulse_start_pos)
                    / (f64::from(p.num_pulses) - 1.0);
            }
            // None, and anything else.
            _ => {}
        }

        // C calculateDataCollectionInterval: the whole profile over every
        // point data collection has.
        let profile = &mut self.profile;
        let time: f64 = profile.full_times[..profile.full_size].iter().sum();
        profile.data_collection_interval = time / ctx.profile.max_points as f64;
        let (size, interval) = (profile.full_size, profile.data_collection_interval);
        print(
            ctx,
            TraceMask::ERROR,
            "buildProfile",
            format_args!(
                "\tfullProfileSize_ = {size}, maxProfilePoints_ = {}, dataCollectionInterval_ = {interval:.6}",
                ctx.profile.max_points
            ),
        );
        Ok(message)
    }

    /// C `runProfile` to its first `unlock()`: what the run reads, with
    /// the execute state at the move to the start. `Ok` carries C's
    /// `message` as the run starts; `Err` is the message of a run that
    /// cannot start.
    pub(crate) fn start_run(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
    ) -> Result<(RunPlan, String), String> {
        let profile = &self.profile;
        let axes = profile.profile_axes.clone();
        if axes.is_empty() {
            return Err("No axes selected".into());
        }
        // C dereferences the arrays initializeProfile never made.
        if axes.iter().any(|&a| a >= profile.axes.len()) {
            return Err("Error: profile not initialized".into());
        }
        print(
            ctx,
            TraceMask::IO_DRIVER,
            "runProfile",
            format_args!(" axisList = {}", axes_to_string(&axes)),
        );
        let string = |ctx: &ControllerCtx<'_>, index: usize| {
            let bytes = ctx.base.get_string_param(index, 0).unwrap_or_default();
            String::from_utf8_lossy(bytes).into_owned()
        };
        let pp = self.pulse_params();
        let pulse_int = |ctx: &ControllerCtx<'_>, f: fn(&PulseParams) -> usize| {
            pp.map_or(0, |pp| int(ctx, f(&pp), 0))
        };
        let full_size = profile.full_size;
        let ctrl = self.lock();
        let recorded = axes
            .iter()
            .take(MAX_DC_AXES)
            .map(|&a| ctrl.recorded_position(a))
            .collect();
        drop(ctrl);
        let plan = RunPlan {
            start: axes.iter().map(|&a| profile.axes[a].start_pos).collect(),
            flyback: axes.iter().map(|&a| profile.axes[a].flyback_pos).collect(),
            full_positions: axes
                .iter()
                .map(|&a| profile.axes[a].full_positions[..full_size].to_vec())
                .collect(),
            full_times: profile.full_times[..full_size].to_vec(),
            num_accel: profile.num_accel,
            max_points: ctx.profile.max_points,
            data_collection_interval: profile.data_collection_interval,
            pulse_start_pos: profile.pulse_start_pos,
            pulse_end_pos: profile.pulse_end_pos,
            pulse_spacing: profile.pulse_spacing,
            recorded,
            start_pulses: int(ctx, index::PROFILE_START_PULSES, 0),
            end_pulses: int(ctx, index::PROFILE_END_PULSES, 0),
            num_points: int(ctx, index::PROFILE_NUM_POINTS, 0),
            num_pulses: int(ctx, index::PROFILE_NUM_PULSES, 0),
            pulse_mode: pulse_int(ctx, |pp| pp.mode),
            pulse_axis: pulse_int(ctx, |pp| pp.axis),
            peg_eng_enc_code: pp.map_or_else(String::new, |pp| string(ctx, pp.peg_eng_enc_code)),
            peg_out_assign_code: pp
                .map_or_else(String::new, |pp| string(ctx, pp.peg_out_assign_code)),
            pouts_output_index: pulse_int(ctx, |pp| pp.pouts_output_index),
            pouts_bit_code: pp.map_or_else(String::new, |pp| string(ctx, pp.pouts_bit_code)),
            pulse_width: pp.map_or(0.0, |pp| float(ctx, pp.pulse_width, 0)),
            move_mode: int(ctx, index::PROFILE_MOVE_MODE, 0),
            axes,
        };
        let message = format!("Selected axes: {}", motors_to_string(&plan.axes));
        set_message(ctx, index::PROFILE_EXECUTE_MESSAGE, &message);
        set_int(
            ctx,
            index::PROFILE_EXECUTE_STATE,
            PROFILE_EXECUTE_MOVE_START,
        );
        set_int(ctx, index::PROFILE_EXECUTE_STATUS, PROFILE_STATUS_UNDEFINED);
        // A readback now reads the run this starts.
        set_int(
            ctx,
            index::PROFILE_READBACK_STATUS,
            PROFILE_STATUS_UNDEFINED,
        );
        let _ = ctx.base.call_param_callbacks(0);
        Ok((plan, message))
    }

    /// C `buildProfile`'s loop over the profile axes: the longest
    /// acceleration and deceleration times any axis needs, at 90% of its
    /// XACC or the user's PROFILE_ACCELERATION if longer, and each axis's
    /// velocity into and out of the profile.
    fn accel_plan(
        &self,
        ctx: &ControllerCtx<'_>,
        p: &BuildParams,
        n: usize,
    ) -> Result<AccelPlan, String> {
        let ctrl = self.lock();
        let times = &ctx.profile.times;
        let (mut pre_time_max, mut post_time_max) = (0.0f64, 0.0f64);
        let mut velocities = Vec::new();
        for &idx in &self.profile.profile_axes {
            let query = |var: &str| {
                ctrl.comm()
                    .write_read_double(&format!("?{var}({idx})"))
                    .map_err(|e| format!("Error getting {var}, status={}\n", e.status() as i32))
            };
            let _max_velocity = query("XVEL")?;
            // ASCII round-off: 10% under the controller's maximum.
            let max_acceleration = query("XACC")? * 0.9;
            let positions = &ctx.profile.axes[idx].positions;
            let (pre_distance, post_distance) = if p.move_mode == PROFILE_MOVE_MODE_ABSOLUTE {
                (
                    positions[1] - positions[0],
                    positions[n - 1] - positions[n - 2],
                )
            } else {
                (positions[0], positions[n - 1])
            };
            // times[0] is the move to the start, so the first segment is 1.
            let pre_velocity = pre_distance / times[1];
            pre_time_max = pre_time_max
                .max(pre_velocity.abs() / max_acceleration)
                .max(p.accel_time);
            let post_velocity = post_distance / times[n - 1];
            post_time_max = post_time_max
                .max(post_velocity.abs() / max_acceleration)
                .max(p.accel_time);
            velocities.push((pre_velocity, post_velocity));
            print(
                ctx,
                TraceMask::FLOW,
                "buildProfile",
                format_args!(
                    " axis {idx} profilePositions[0]={:.6}, profilePositions[{}]={:.6}, \
                     maxAcceleration={max_acceleration:.6}, preTimeMax={pre_time_max:.6}, \
                     postTimeMax={post_time_max:.6}",
                    positions[0],
                    n - 1,
                    positions[n - 1]
                ),
            );
        }
        Ok(AccelPlan {
            pre_time_max,
            post_time_max,
            velocities,
        })
    }

    /// C `assembleFullProfile`: acceleration segments, the profile from its
    /// second point (the first is where the acceleration ends), then
    /// deceleration segments.
    fn assemble_full_profile(&mut self, ctx: &ControllerCtx<'_>, n: usize) {
        let profile = &mut self.profile;
        let (num_accel, num_decel) = (profile.num_accel, profile.num_decel);
        profile.full_times[..num_accel].copy_from_slice(&profile.accel_times[..num_accel]);
        profile.full_times[num_accel..num_accel + n - 1].copy_from_slice(&ctx.profile.times[1..n]);
        profile.full_times[num_accel + n - 1..num_accel + n - 1 + num_decel]
            .copy_from_slice(&profile.decel_times[..num_decel]);
        for &idx in &profile.profile_axes {
            let path = &mut profile.axes[idx];
            let positions = &ctx.profile.axes[idx].positions;
            let full = &mut path.full_positions;
            full[..num_accel].copy_from_slice(&path.accel_positions[..num_accel]);
            full[num_accel..num_accel + n - 1].copy_from_slice(&positions[1..n]);
            full[num_accel + n - 1..num_accel + n - 1 + num_decel]
                .copy_from_slice(&path.decel_positions[..num_decel]);
        }
        profile.full_size = num_accel + n - 1 + num_decel;
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use std::collections::HashSet;
    use std::sync::{Arc, Mutex};

    use epics_rs::asyn::interfaces::motor::AsynMotor;
    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::port_handle::PortHandle;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::motor::builder::MotorBuilder;
    use epics_rs::motor::controller_port::ControllerPort;
    use epics_rs::motor::devices::{register_axis, register_controller_port};
    use epics_rs::motor::trace::TraceUser;

    use super::*;
    use crate::comm::SpiiPlusComm;
    use crate::spiiplus::{SpiiPlusAxis, SpiiPlusController};

    /// A controller port of `axes` axes on a fake line, every axis
    /// registered, the record units 0.1 per step with no offset, and the
    /// profile initialized for `max_points` points and `max_pulses` pulses.
    pub(crate) struct Rig {
        pub(crate) port: PortHandle,
        pub(crate) line: Arc<Mutex<crate::fake::Controller>>,
        pub(crate) ctrl: Arc<Mutex<SpiiPlusController>>,
    }

    pub(crate) fn rig(name: &str, axes: usize, setup: &[(&str, &[f64])]) -> Rig {
        let line = crate::fake::line(name);
        for (var, values) in setup {
            line.lock().unwrap().set(var, values);
        }
        let comm = SpiiPlusComm::connect(&format!("{name}Comm"), name).unwrap();
        let port_name = format!("{name}Ctrl");
        let ctrl = SpiiPlusController::new(comm, &port_name, axes, &HashSet::new());
        let ctrl = Arc::new(Mutex::new(ctrl));
        let (params, thread) = AcsMotionParams::new(ctrl.clone());
        let port =
            register_controller_port(&open_owner(), &port_name, axes as i32, params).unwrap();
        thread.spawn(port.clone()).unwrap();
        for addr in 0..axes {
            let motor: Arc<Mutex<dyn AsynMotor>> =
                Arc::new(Mutex::new(SpiiPlusAxis::new(ctrl.clone(), addr)));
            let user = TraceUser::Asyn {
                port: port_name.clone(),
                addr: addr as i32,
            };
            register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
            let addr = addr as i32;
            port.write_float64_blocking(index::REC_RESOLUTION, addr, 0.1)
                .unwrap();
            port.write_float64_blocking(index::REC_OFFSET, addr, 0.0)
                .unwrap();
            port.write_int32_blocking(index::REC_DIRECTION, addr, 0)
                .unwrap();
        }
        let rig = Rig { port, line, ctrl };
        rig.with(|params, ctx| params.initialize_profile_pulses(ctx, 8, 3))
            .unwrap();
        rig.line.lock().unwrap().sent.clear();
        rig
    }

    impl Rig {
        pub(crate) fn with<R: Send + 'static>(
            &self,
            f: impl FnOnce(&mut AcsMotionParams, &mut ControllerCtx<'_>) -> R + Send + 'static,
        ) -> R {
            self.port
                .with_driver_blocking::<ControllerPort<AcsMotionParams>, _>(|d| d.with_params(f))
                .unwrap()
        }

        pub(crate) fn int(&self, index: usize, addr: i32, value: i32) {
            self.port.write_int32_blocking(index, addr, value).unwrap();
        }

        pub(crate) fn float(&self, index: usize, addr: i32, value: f64) {
            self.port
                .write_float64_blocking(index, addr, value)
                .unwrap();
        }

        pub(crate) fn array(&self, reason: usize, addr: i32, data: &[f64]) -> AsynResult<()> {
            let data = data.to_vec();
            self.port
                .with_driver_blocking::<ControllerPort<AcsMotionParams>, _>(move |d| {
                    let user = AsynUser::new(reason).with_addr(addr);
                    d.write_float64_array(&user, &data)
                })
                .unwrap()
        }

        pub(crate) fn param(&self, name: &'static str) -> usize {
            self.with(move |_, ctx| ctx.base.find_param(name).unwrap())
        }

        pub(crate) fn message(&self, index: usize) -> String {
            let bytes = self.port.get_string_param_blocking(index, 0, 256).unwrap();
            String::from_utf8(bytes).unwrap()
        }

        /// A fixed-time profile of `positions` for each axis in `axes`,
        /// 0.5 s a point and 0.1 s to accelerate.
        pub(crate) fn define(&self, axes: &[i32], positions: &[f64], move_mode: i32) {
            self.int(index::PROFILE_NUM_POINTS, 0, positions.len() as i32);
            self.int(index::PROFILE_TIME_MODE, 0, 0);
            self.float(index::PROFILE_FIXED_TIME, 0, 0.5);
            self.float(index::PROFILE_ACCELERATION, 0, 0.1);
            self.int(index::PROFILE_MOVE_MODE, 0, move_mode);
            for &addr in axes {
                self.int(index::PROFILE_USE_AXIS, addr, 1);
                self.array(index::PROFILE_POSITIONS, addr, positions)
                    .unwrap();
            }
        }

        pub(crate) fn sent(&self) -> Vec<String> {
            std::mem::take(&mut self.line.lock().unwrap().sent)
        }
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    /// EFAC 0.5 and the controller's XVEL/XACC for each axis.
    pub(crate) const SERVOS: &[(&str, &[f64])] = &[
        ("EFAC", &[0.5, 0.5]),
        ("XVEL", &[100.0, 100.0]),
        ("XACC", &[1000.0, 1000.0]),
    ];

    #[test]
    fn initialize_makes_the_data_collection_arrays_in_the_controller() {
        let line = crate::fake::line("spiiPInit");
        let comm = SpiiPlusComm::connect("spiiPInitComm", "spiiPInit").unwrap();
        let ctrl = SpiiPlusController::new(comm, "spiiPInitCtrl", 1, &HashSet::new());
        let (params, _) = AcsMotionParams::new(Arc::new(Mutex::new(ctrl)));
        let port = register_controller_port(&open_owner(), "spiiPInitCtrl", 1, params).unwrap();
        line.lock().unwrap().sent.clear();
        port.with_driver_blocking::<ControllerPort<AcsMotionParams>, _>(|d| {
            d.with_params(|params, ctx| params.initialize_profile_pulses(ctx, 5, 2))
        })
        .unwrap()
        .unwrap();
        let want: Vec<String> = (1..=8)
            .flat_map(|i| {
                [
                    format!("#VGV DC_DATA_{i}"),
                    format!("GLOBAL REAL DC_DATA_{i} (3)(5)"),
                ]
            })
            .collect();
        assert_eq!(line.lock().unwrap().sent, want);
    }

    #[test]
    fn an_absolute_build_adds_acceleration_and_deceleration_in_controller_units() {
        let rig = rig("spiiPAbs", 1, SERVOS);
        // 0.1 per step, 0.5 controller units per step: 5 units a point.
        rig.define(&[0], &[0.0, 1.0, 2.0, 3.0], PROFILE_MOVE_MODE_ABSOLUTE);
        rig.int(index::PROFILE_BUILD, 0, 1);
        assert_eq!(rig.sent(), ["?XVEL(0)", "?XACC(0)"]);
        assert_eq!(
            rig.port
                .read_int32_blocking(index::PROFILE_BUILD_STATUS, 0)
                .unwrap(),
            PROFILE_STATUS_SUCCESS
        );
        assert_eq!(
            rig.message(index::PROFILE_BUILD_MESSAGE),
            "Selected axes: 1"
        );
        assert_eq!(
            rig.port
                .read_int32_blocking(index::PROFILE_BUILD, 0)
                .unwrap(),
            0
        );
        let (profile, path) = rig.with(|params, _| {
            let p = &params.profile;
            (
                (
                    p.num_accel,
                    p.num_decel,
                    p.full_size,
                    p.data_collection_interval,
                ),
                p.axes[0].clone(),
            )
        });
        // 10 units/s into the profile over the user's 0.1 s, 900 units/s²
        // allowed: ten 10 ms segments each way, half the 0.5 s point's
        // distance spent accelerating.
        assert_eq!((profile.0, profile.1, profile.2), (10, 10, 23));
        assert!(close(profile.3, 1.7 / 8.0));
        assert!(close(path.start_pos, -0.5));
        assert!(close(path.flyback_pos, 15.0));
        let full = &path.full_positions;
        assert!(close(full[0], -0.5 + 50.0 * 0.01f64.powi(2)));
        assert!(close(full[9], 0.0));
        assert_eq!(full[10..13], [5.0, 10.0, 15.0]);
        assert!(close(full[22], 15.5));
    }

    #[test]
    fn a_relative_build_reconverts_the_displacements_without_the_offset() {
        let rig = rig("spiiPRel", 1, SERVOS);
        rig.float(index::REC_OFFSET, 0, 100.0);
        rig.define(&[0], &[1.0; 4], PROFILE_MOVE_MODE_RELATIVE);
        // defineProfile subtracted the offset, as for positions.
        let defined = rig.with(|_, ctx| ctx.profile.axes[0].positions[..4].to_vec());
        assert_eq!(defined, [-495.0; 4]);
        rig.int(index::PROFILE_BUILD, 0, 1);
        let (positions, path) = rig.with(|params, ctx| {
            (
                ctx.profile.axes[0].positions[..4].to_vec(),
                params.profile.axes[0].clone(),
            )
        });
        assert_eq!(positions, [5.0; 4]);
        assert!(close(path.start_pos, -0.5));
        assert!(close(path.flyback_pos, -0.5));
        // Each acceleration segment is its own displacement.
        let accel: f64 = path.accel_positions.iter().sum();
        assert!(close(accel, 0.5));
    }

    #[test]
    fn a_build_with_no_axis_selected_fails() {
        let rig = rig("spiiPNone", 1, SERVOS);
        rig.int(index::PROFILE_NUM_POINTS, 0, 4);
        rig.int(index::PROFILE_BUILD, 0, 1);
        assert_eq!(
            rig.port
                .read_int32_blocking(index::PROFILE_BUILD_STATUS, 0)
                .unwrap(),
            PROFILE_STATUS_FAILURE
        );
        assert_eq!(
            rig.message(index::PROFILE_BUILD_MESSAGE),
            "No axes selected"
        );
        assert_eq!(
            rig.port
                .read_int32_blocking(index::PROFILE_BUILD_STATE, 0)
                .unwrap(),
            PROFILE_BUILD_DONE
        );
        assert!(rig.sent().is_empty());
    }

    #[test]
    fn a_refused_xacc_fails_the_build_with_cs_message() {
        let rig = rig("spiiPXacc", 1, SERVOS);
        rig.line.lock().unwrap().refused.insert("?XACC(0)".into());
        rig.define(&[0], &[0.0, 1.0, 2.0], PROFILE_MOVE_MODE_ABSOLUTE);
        rig.int(index::PROFILE_BUILD, 0, 1);
        assert_eq!(
            rig.message(index::PROFILE_BUILD_MESSAGE),
            "Error getting XACC, status=3\n"
        );
    }

    #[test]
    fn a_build_past_the_profile_arrays_fails_where_c_overruns_them() {
        let rig = rig("spiiPSize", 1, SERVOS);
        rig.define(&[0], &[0.0, 1.0], PROFILE_MOVE_MODE_ABSOLUTE);
        for num_points in [1, 9] {
            rig.int(index::PROFILE_NUM_POINTS, 0, num_points);
            rig.int(index::PROFILE_BUILD, 0, 1);
            assert_eq!(
                rig.message(index::PROFILE_BUILD_MESSAGE),
                "Error: number of points outside valid range"
            );
        }
    }

    #[test]
    fn array_pulses_are_converted_and_sent_whole_at_build() {
        let rig = rig("spiiPArr", 1, SERVOS);
        let pulse_pos = rig.param("SPIIPLUS_PULSE_POS");
        // More than the 3 initialized: C copies past its array.
        rig.array(pulse_pos, 0, &[1.0, 2.0, 3.0, 4.0]).unwrap();
        rig.int(rig.param("SPIIPLUS_PULSE_MODE"), 0, 1);
        rig.int(index::PROFILE_NUM_PULSES, 0, 2);
        rig.define(&[0], &[0.0, 1.0, 2.0], PROFILE_MOVE_MODE_ABSOLUTE);
        rig.sent();
        rig.int(index::PROFILE_BUILD, 0, 1);
        assert_eq!(
            rig.sent(),
            [
                "?XVEL(0)",
                "?XACC(0)",
                // putDoubleArray's check that pulsePos is large enough.
                "?pulsePos(2)",
                "BIN >>pulsePos(0,2)(0,0)"
            ]
        );
        let line = rig.line.lock().unwrap();
        let sent = &line.vars["pulsePos"];
        // NUM_PULSES of them converted, the rest as they were.
        assert_eq!(
            [sent[&(0, 0)], sent[&(1, 0)], sent[&(2, 0)]],
            [5.0, 10.0, 0.0]
        );
    }

    #[test]
    fn pulse_positions_need_an_axis_at_the_address() {
        let rig = rig("spiiPAddr", 1, SERVOS);
        let pulse_pos = rig.param("SPIIPLUS_PULSE_POS");
        assert!(rig.array(pulse_pos, 0, &[1.0]).is_ok());
        assert!(rig.array(pulse_pos, 1, &[1.0]).is_err());
    }

    #[test]
    fn fixed_pulses_space_num_pulses_over_the_profile() {
        let rig = rig("spiiPFixed", 1, SERVOS);
        rig.int(index::PROFILE_NUM_PULSES, 0, 4);
        rig.define(&[0], &[0.0, 1.0, 2.0, 3.0], PROFILE_MOVE_MODE_ABSOLUTE);
        rig.int(index::PROFILE_BUILD, 0, 1);
        let pulses = rig.with(|params, _| {
            let p = &params.profile;
            (p.pulse_start_pos, p.pulse_end_pos, p.pulse_spacing)
        });
        assert_eq!(pulses, (0.0, 15.0, 5.0));
    }

    #[test]
    fn axes_print_as_acspl_takes_them() {
        assert_eq!(axes_to_string(&[2]), "2");
        assert_eq!(axes_to_string(&[0, 1, 3]), "(0,1,3)");
        assert_eq!(motors_to_string(&[0, 1, 3]), "1, 2, 4");
    }
}
