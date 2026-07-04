//! Shared XPS controller state: the poll socket and cross-axis registry.
//!
//! Every axis on one XPS shares a single **poll socket** (a `Query`-mode
//! [`XpsSocket`]) for status/position/limit reads; the controller owns it
//! behind an `Arc<Mutex<..>>` so each axis operation locks it for an atomic
//! exchange (the analogue of the C `pasynOctetSyncIO->writeRead` + socket
//! mutex in `asynOctetSocket.cpp`). The controller also holds the identity of
//! every registered axis so group-wide operations (`setPosition`, `isInGroup`)
//! can enumerate siblings sharing an XPS group — the single owner of that
//! cross-axis state.

use std::collections::HashSet;
use std::time::Duration;

use super::profile::{MoveMode, Profile, TrajectoryFile};
use super::rpc::{XpsResult, XpsSocket};

/// A registered axis's identity, kept by the controller for group enumeration.
struct AxisRef {
    positioner: String,
    group: String,
}

/// A verified, ready-to-execute PVT trajectory. Produced by
/// [`XpsController::verify_profile`] and consumed by the execute command.
struct BuiltTrajectory {
    /// Bare trajectory file name on the controller (RPC `file` argument).
    file_name: String,
    group: String,
    /// Per-axis move to the true trajectory start, applied before execution:
    /// `(positioner, target)`, already resolved to absolute or relative units
    /// per the profile's move mode.
    start_moves: Vec<(String, f64)>,
    move_mode: MoveMode,
}

/// A snapshot of a built trajectory for the execute command to run on its own
/// socket without holding the controller lock.
#[derive(Clone, Debug)]
pub struct ExecutionPlan {
    pub file_name: String,
    pub group: String,
    pub start_moves: Vec<(String, f64)>,
    pub move_mode: MoveMode,
    /// The profile's per-point times, for the pulse-output window
    /// (C `profileTimes_`).
    pub times: Vec<f64>,
    /// Every registered positioner in registration order — the gathering
    /// samples `SetpointPosition`+`CurrentPosition` for each (C
    /// `executeProfile` builds the gathering list over all `numAxes_` axes,
    /// not just the profile's).
    pub gathering_positioners: Vec<String>,
}

/// Shared controller state for one XPS, owning the poll socket.
pub struct XpsController {
    poll: XpsSocket,
    firmware: String,
    /// `enableSetPosition` — gate on `setPosition` (C `XPSCreateController`).
    enable_set_position: bool,
    /// `setPositionSettlingTime` — sleep after group init during set-position.
    set_position_settling: Duration,
    /// `autoEnable` — re-enable a disabled axis on move (default on).
    auto_enable: bool,
    /// Each registered axis's `(positioner, group)`, in registration order —
    /// for group-membership counts (C `XPSAxis::isInGroup`) and for enumerating
    /// a group's positioners when building/executing a PVT profile.
    axes: Vec<AxisRef>,
    /// Groups currently in referencing (move-to-home) mode. C tracks this
    /// per-axis as `referencingMode_`, but sets it for every axis in the group
    /// at once (`doMoveToHome`/`home`), so it is group-scoped state. While a
    /// group is here, poll reports its axes as not-homed/not-at-home.
    referencing_groups: HashSet<String>,
    /// The PVT profile currently defined (C `pAxes_`/profile arrays), if any.
    profile: Option<Profile>,
    /// The last successfully built+verified trajectory, ready to execute.
    built_trajectory: Option<BuiltTrajectory>,
}

impl XpsController {
    /// Build the controller over a connected `Query`-mode poll socket, reading
    /// the firmware version (C `XPSController` constructor). A firmware-read
    /// failure is tolerated (left blank), matching the C constructor which
    /// ignores the return value.
    pub fn new(
        poll: XpsSocket,
        enable_set_position: bool,
        set_position_settling: Duration,
    ) -> XpsResult<Self> {
        let firmware = poll.firmware_version_get().unwrap_or_default();
        Ok(Self {
            poll,
            firmware,
            enable_set_position,
            set_position_settling,
            auto_enable: true,
            axes: Vec::new(),
            referencing_groups: HashSet::new(),
            profile: None,
            built_trajectory: None,
        })
    }

    /// The shared poll socket (`Query` mode).
    pub fn poll_socket(&self) -> &XpsSocket {
        &self.poll
    }

    /// Firmware version string read at construction.
    pub fn firmware(&self) -> &str {
        &self.firmware
    }

    /// Whether disabled axes are auto-enabled on move (C `autoEnable_`).
    pub fn auto_enable(&self) -> bool {
        self.auto_enable
    }

    /// Set `autoEnable` (C `XPSDisableAutoEnable` clears it).
    pub fn set_auto_enable(&mut self, enable: bool) {
        self.auto_enable = enable;
    }

    /// Record an axis's positioner and group so group-membership counts and
    /// group positioner enumeration stay accurate.
    pub fn register_axis(&mut self, positioner: &str, group: &str) {
        self.axes.push(AxisRef {
            positioner: positioner.to_string(),
            group: group.to_string(),
        });
    }

    /// Number of registered axes in `group` (C `XPSAxis::isInGroup`).
    pub fn axes_in_group(&self, group: &str) -> usize {
        self.axes.iter().filter(|a| a.group == group).count()
    }

    /// Positioner names registered in `group`, in registration order. Used to
    /// map PVT profile columns and to move each axis to the trajectory start.
    pub fn positioners_in_group(&self, group: &str) -> Vec<String> {
        self.axes
            .iter()
            .filter(|a| a.group == group)
            .map(|a| a.positioner.clone())
            .collect()
    }

    /// Set or clear referencing (move-to-home) mode for `group`. `doMoveToHome`
    /// sets it on success; the normal `home` seek clears it.
    pub fn set_group_referencing(&mut self, group: &str, on: bool) {
        if on {
            self.referencing_groups.insert(group.to_string());
        } else {
            self.referencing_groups.remove(group);
        }
    }

    /// Whether `group` is in referencing mode (poll suppresses home/homed).
    pub fn is_group_referencing(&self, group: &str) -> bool {
        self.referencing_groups.contains(group)
    }

    /// Redefine the current position of `positioner` to `position` device
    /// units, via the XPS referencing mode (C `XPSAxis::setPosition`, single-
    /// positioner branch: `GroupKill` → `GroupInitialize` → settle →
    /// `GroupReferencingStart` → `GroupReferencingActionExecute(SetPosition)` →
    /// `GroupReferencingStop`). Requires `enableSetPosition`.
    ///
    /// The multi-positioner group branch (`isInGroup > 1`, coordinated XY/XYZ
    /// referencing of siblings) is not modeled here; callers detect it via
    /// [`axes_in_group`] and report it rather than issuing a partial sequence.
    ///
    /// [`axes_in_group`]: XpsController::axes_in_group
    pub fn set_position(&self, positioner: &str, group: &str, position: f64) -> XpsResult<()> {
        // C: GroupKill's status is not checked before GroupInitialize.
        let _ = self.poll.group_kill(group);
        self.poll.group_initialize(group)?;

        // Settle after initialization so the stage does not oscillate.
        std::thread::sleep(self.set_position_settling);

        self.poll.group_referencing_start(group)?;
        self.poll
            .group_referencing_action_execute(positioner, "SetPosition", "None", position)?;
        self.poll.group_referencing_stop(group)
    }

    /// Whether `setPosition` is enabled (C `enableSetPosition_`).
    pub fn enable_set_position(&self) -> bool {
        self.enable_set_position
    }

    // --- PVT trajectory profiles ------------------------------------------

    /// Store a PVT profile for `group`, replacing any previous one and
    /// invalidating a previously built trajectory. Every profile axis must be a
    /// positioner registered in the group.
    pub fn define_profile(&mut self, profile: Profile) -> Result<(), String> {
        let group_positioners = self.positioners_in_group(&profile.group);
        if group_positioners.is_empty() {
            return Err(format!(
                "no axes registered in group '{}' (create the axes first)",
                profile.group
            ));
        }
        for ax in &profile.axes {
            if !group_positioners.contains(&ax.positioner) {
                return Err(format!(
                    "positioner '{}' is not a registered axis in group '{}'",
                    ax.positioner, profile.group
                ));
            }
        }
        self.built_trajectory = None;
        self.profile = Some(profile);
        Ok(())
    }

    /// Generate the trajectory-file text for the defined profile, reading each
    /// axis's controller max acceleration via `PositionerSGammaParametersGet`
    /// (C `buildProfile` uses these for the ramp times).
    pub fn build_profile_text(&self) -> Result<TrajectoryFile, String> {
        let profile = self
            .profile
            .as_ref()
            .ok_or("no profile defined (call XPSDefineProfileFromFile first)")?;
        let mut max_accels = Vec::with_capacity(profile.axes.len());
        for ax in &profile.axes {
            // SGamma returns (velocity, acceleration, minJerk, maxJerk); field 1
            // is the max acceleration used to size the ramp elements.
            let (_vel, accel, _min_jerk, _max_jerk) = self
                .poll
                .positioner_sgamma_parameters_get(&ax.positioner)
                .map_err(|e| format!("SGamma read for {}: {e}", ax.positioner))?;
            max_accels.push(accel);
        }
        profile.generate(&max_accels).map_err(|e| e.to_string())
    }

    /// Verify an uploaded trajectory against the group's dynamics and each
    /// axis's software travel limits, then latch it for execution.
    ///
    /// `file_name` is the bare trajectory file name already uploaded to the
    /// controller; `built` is the [`TrajectoryFile`] from [`build_profile_text`]
    /// (its pre-ramp distances resolve the execute-time start positions). C
    /// `buildProfile` runs `MultipleAxesPVTVerification` then, per axis,
    /// `MultipleAxesPVTVerificationResultGet` and checks the resulting
    /// min/max travel against the soft limits.
    ///
    /// [`build_profile_text`]: XpsController::build_profile_text
    pub fn verify_profile(
        &mut self,
        file_name: &str,
        built: &TrajectoryFile,
    ) -> Result<(), String> {
        let profile = self
            .profile
            .as_ref()
            .ok_or("no profile defined (call XPSDefineProfileFromFile first)")?;
        if built.pre_distance.len() != profile.axes.len() {
            return Err("built trajectory does not match the defined profile".into());
        }
        let group = profile.group.clone();
        let move_mode = profile.move_mode;

        self.poll
            .multiple_axes_pvt_verification(&group, file_name)
            .map_err(|e| format!("PVT verification failed: {e}"))?;

        let mut start_moves = Vec::with_capacity(profile.axes.len());
        for (j, ax) in profile.axes.iter().enumerate() {
            let (min_pos, max_pos, _max_vel, _max_accel) = self
                .poll
                .multiple_axes_pvt_verification_result_get(&ax.positioner, file_name)
                .map_err(|e| format!("PVT result get for {}: {e}", ax.positioner))?;
            let (low, high) = self
                .poll
                .positioner_user_travel_limits_get(&ax.positioner)
                .map_err(|e| format!("travel limits read for {}: {e}", ax.positioner))?;
            // The verified min/max are relative to the trajectory start point.
            let start = ax.positions[0];
            if start + min_pos < low {
                return Err(format!(
                    "{}: trajectory low {:.6} violates soft limit {:.6}",
                    ax.positioner,
                    start + min_pos,
                    low
                ));
            }
            if start + max_pos > high {
                return Err(format!(
                    "{}: trajectory high {:.6} violates soft limit {:.6}",
                    ax.positioner,
                    start + max_pos,
                    high
                ));
            }
            // The execute-time move places the motor one pre-ramp distance
            // before the first point so it reaches full velocity at point 0.
            let target = match move_mode {
                MoveMode::Absolute => start - built.pre_distance[j],
                MoveMode::Relative => -built.pre_distance[j],
            };
            start_moves.push((ax.positioner.clone(), target));
        }

        self.built_trajectory = Some(BuiltTrajectory {
            file_name: file_name.to_string(),
            group,
            start_moves,
            move_mode,
        });
        Ok(())
    }

    /// A snapshot of the latched trajectory for the execute command to run on
    /// its own socket. `None` if nothing has been built+verified since the last
    /// [`define_profile`] (which also guarantees `profile` still matches the
    /// built trajectory: defining a new profile clears it).
    pub fn execution_plan(&self) -> Option<ExecutionPlan> {
        let built = self.built_trajectory.as_ref()?;
        let profile = self.profile.as_ref()?;
        Some(ExecutionPlan {
            file_name: built.file_name.clone(),
            group: built.group.clone(),
            start_moves: built.start_moves.clone(),
            move_mode: built.move_mode,
            times: profile.times.clone(),
            gathering_positioners: self.registered_positioners(),
        })
    }

    /// The number of registered axes and their positioner names in registration
    /// order — the shape of the gathering data (one
    /// `SetpointPosition;CurrentPosition` pair per registered axis per sample).
    pub fn registered_positioners(&self) -> Vec<String> {
        self.axes.iter().map(|a| a.positioner.clone()).collect()
    }
}
