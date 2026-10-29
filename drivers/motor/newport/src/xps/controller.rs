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
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::trace::TraceMask;
use epics_rs::motor::trace::MotorTrace;

use super::pco::PcoParams;
use super::rpc::{XpsResult, XpsSocket};

/// A registered axis's identity, kept by the controller for group enumeration.
struct AxisRef {
    axis_no: i32,
    positioner: String,
    group: String,
    move_sock: Arc<XpsSocket>,
    state: Arc<Mutex<MoveState>>,
    /// C `stepSize_`.
    step_size: f64,
}

/// What a profile reads of a registered axis (C `pAxes_[j]`).
pub struct RegisteredAxis {
    pub axis_no: i32,
    pub positioner: String,
    pub group: String,
    pub move_sock: Arc<XpsSocket>,
    pub step_size: f64,
}

/// The part of an axis a deferred group move reads and clears; the axis and
/// the controller share it, locked after the controller.
#[derive(Default)]
pub struct MoveState {
    /// C `setpointPosition_`, device units, from the last poll.
    pub setpoint: f64,
    /// C `deferredMove_`, with `deferredPosition_` and `deferredRelative_`.
    pub deferred: Option<DeferredMove>,
}

/// A move held until the deferral is released, in device units.
#[derive(Clone, Copy)]
pub struct DeferredMove {
    pub position: f64,
    pub relative: bool,
}

/// The `XPSController` parameters its axes and writes use
/// (XPSController.cpp:159-176).
#[derive(Clone, Copy, Debug)]
pub struct XpsParams {
    /// `XPS_MIN_JERK`, `XPS_MAX_JERK`.
    pub min_jerk: usize,
    pub max_jerk: usize,
    pub pco: PcoParams,
    /// `XPS_STATUS`, `XPS_STATUS_STRING`.
    pub status: usize,
    pub status_string: usize,
    /// `XPS_TCL_SCRIPT`, `XPS_TCL_SCRIPT_EXECUTE`.
    pub tcl_script: usize,
    pub tcl_script_execute: usize,
}

/// The [`XpsController::set_position`] step that failed, with its XPS code.
#[derive(Debug, PartialEq, Eq)]
pub enum SetPositionFailure {
    /// The grouped branch's `GroupPositionCurrentGet` of the last axis: the
    /// position was not set.
    CurrentGet(i32),
    /// `GroupKill`/`GroupInitialize`: the position was not set. `grouped`
    /// picks C's line, which differs between the two branches.
    Initialize { code: i32, grouped: bool },
    /// `GroupReferencingStop`: C reports it and returns success. `grouped`
    /// picks C's line: only the grouped branch's ends in a newline.
    ReferencingStop { code: i32, grouped: bool },
}

/// C `buildProfile`'s message for `MultipleAxesPVTVerification`'s status.
pub(crate) fn verify_message(status: i32) -> String {
    match -status {
        0 => " ".into(),
        69 => "Acceleration Too High".into(),
        68 => "Velocity Too High".into(),
        70 => "Final Velocity Non Zero".into(),
        75 => "Negative or Null Delta Time".into(),
        _ => format!("Unknown trajectory verify error={status}"),
    }
}

/// C `XPSController::processDeferredMovesInGroup`: move every axis of
/// `group`, in axis order, to its deferred target or else its setpoint, on
/// the last axis's move socket, and clear the group's deferred moves.
///
/// C sends `GroupMoveRelative` when any axis in the group was ever deferred
/// relative, with the same absolute targets it builds for
/// `GroupMoveAbsolute`; here the targets always go out absolute.
fn process_deferred_moves_in_group(
    axes: &[&AxisRef],
    group: &str,
    trace: &MotorTrace,
) -> XpsResult<()> {
    let mut targets = Vec::new();
    for a in axes {
        trace.print_self(
            TraceMask::FLOW,
            format_args!(
                "Executing deferred move on XPS: {}, Group: {group}\n",
                trace.port()
            ),
        );
        if a.group == group {
            let mut state = a.state.lock().unwrap_or_else(|e| e.into_inner());
            targets.push(match state.deferred.take() {
                Some(DeferredMove {
                    position,
                    relative: true,
                }) => state.setpoint + position,
                Some(DeferredMove { position, .. }) => position,
                None => state.setpoint,
            });
        }
    }
    let Some(last) = axes.last() else {
        return Ok(());
    };
    last.move_sock
        .group_move_absolute(group, &targets)
        .inspect_err(|e| {
            trace.print_self(
                TraceMask::ERROR,
                format_args!(
                    "Error peforming GroupMoveAbsolute/Relative in processDeferredMovesInGroup. XPS Return code: {}\n",
                    e.code()
                ),
            )
        })
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
    /// `noDisableError` — a disabled axis (state 20) is no MSTA problem.
    no_disable_error: bool,
    /// `enableMovingMode_`: an axis's move is done when its move socket
    /// answers, not when its group stops moving.
    moving_mode: bool,
    /// C `movesDeferred_`.
    moves_deferred: bool,
    /// C `useSFTP_`: set by the first XPS-D profile build.
    use_sftp: bool,
    /// Each registered axis's `(positioner, group)`, in registration order —
    /// for group-membership counts (C `XPSAxis::isInGroup`) and for enumerating
    /// a group's positioners when building/executing a PVT profile.
    axes: Vec<AxisRef>,
    /// Groups currently in referencing (move-to-home) mode. C tracks this
    /// per-axis as `referencingMode_`, but sets it for every axis in the group
    /// at once (`doMoveToHome`/`home`), so it is group-scoped state. While a
    /// group is here, poll reports its axes as not-homed/not-at-home.
    referencing_groups: HashSet<String>,
    /// The port's parameters the axes set.
    params: XpsParams,
}

impl XpsController {
    /// Build the controller over a connected `Query`-mode poll socket, reading
    /// the firmware version (C `XPSController` constructor). A firmware-read
    /// failure is tolerated (left blank), matching the C constructor which
    /// ignores the return value. `params` are the port's, created before.
    pub fn new(
        poll: XpsSocket,
        enable_set_position: bool,
        set_position_settling: Duration,
        params: XpsParams,
    ) -> XpsResult<Self> {
        let firmware = poll.firmware_version_get().unwrap_or_default();
        Ok(Self {
            poll,
            firmware,
            enable_set_position,
            set_position_settling,
            auto_enable: true,
            no_disable_error: false,
            moving_mode: false,
            moves_deferred: false,
            use_sftp: false,
            axes: Vec::new(),
            referencing_groups: HashSet::new(),
            params,
        })
    }

    /// The port's parameters the axes set.
    pub fn params(&self) -> XpsParams {
        self.params
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

    /// Whether a disabled axis leaves the problem bit clear (C
    /// `noDisableError_`).
    pub fn no_disable_error(&self) -> bool {
        self.no_disable_error
    }

    /// C `XPSNoDisableError`.
    pub fn set_no_disable_error(&mut self) {
        self.no_disable_error = true;
    }

    /// Whether moving mode is on (C `enableMovingMode_`).
    pub fn moving_mode(&self) -> bool {
        self.moving_mode
    }

    /// C `XPSEnableMovingMode`.
    pub fn set_moving_mode(&mut self) {
        self.moving_mode = true;
    }

    /// Record an axis's positioner and group so group-membership counts and
    /// group positioner enumeration stay accurate, and its move socket and
    /// move state for the deferred group move.
    pub fn register_axis(
        &mut self,
        axis_no: i32,
        positioner: &str,
        group: &str,
        move_sock: Arc<XpsSocket>,
        state: Arc<Mutex<MoveState>>,
        step_size: f64,
    ) {
        self.axes.push(AxisRef {
            axis_no,
            positioner: positioner.to_string(),
            group: group.to_string(),
            move_sock,
            state,
            step_size,
        });
    }

    /// The registered axes in axis order, as C's `for (j=0; j<numAxes_;
    /// j++)` profile loops visit them.
    pub fn registered_axes(&self) -> Vec<RegisteredAxis> {
        let mut axes: Vec<_> = self
            .axes
            .iter()
            .map(|a| RegisteredAxis {
                axis_no: a.axis_no,
                positioner: a.positioner.clone(),
                group: a.group.clone(),
                move_sock: a.move_sock.clone(),
                step_size: a.step_size,
            })
            .collect();
        axes.sort_by_key(|a| a.axis_no);
        axes
    }

    /// Whether profile builds upload over SFTP (C `useSFTP_`).
    pub fn use_sftp(&self) -> bool {
        self.use_sftp
    }

    /// An XPS-D profile build sets `useSFTP_`; nothing clears it.
    pub fn set_use_sftp(&mut self) {
        self.use_sftp = true;
    }

    /// Whether moves are held for a group move (C `movesDeferred_`).
    pub fn moves_deferred(&self) -> bool {
        self.moves_deferred
    }

    /// C `XPSController::setDeferredMoves`, printing on `trace`'s port.
    ///
    /// C sets `movesDeferred_` on any call that is not a release of deferred
    /// moves and never clears it, so a release with none deferred arms
    /// deferral and every move after a release stays deferred; here the
    /// flag follows `defer`.
    pub fn set_deferred_moves(&mut self, trace: &MotorTrace, defer: bool) -> XpsResult<()> {
        let release = self.moves_deferred && !defer;
        self.moves_deferred = defer;
        if !release {
            return Ok(());
        }
        let mut axes: Vec<&AxisRef> = self.axes.iter().collect();
        axes.sort_by_key(|a| a.axis_no);
        let mut done: Vec<&str> = Vec::new();
        let mut result = Ok(());
        for a in &axes {
            trace.print_self(
                TraceMask::FLOW,
                format_args!("Processing deferred moves on XPS: {}\n", trace.port()),
            );
            // C's groupNames cache (XPSController.cpp:459-471) skips only a
            // group equal to the previous axis's, so a group split by another
            // moves twice, the second time to its setpoints; here each group
            // moves once.
            if !done.contains(&a.group.as_str()) {
                done.push(&a.group);
                result = process_deferred_moves_in_group(&axes, &a.group, trace);
            }
        }
        result
    }

    /// Number of registered axes in `group` (C `XPSAxis::isInGroup`).
    pub fn axes_in_group(&self, group: &str) -> usize {
        self.axes.iter().filter(|a| a.group == group).count()
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
    /// units, via the XPS referencing mode (C `XPSAxis::setPosition`):
    /// `GroupKill` → `GroupInitialize` → settle → `GroupReferencingStart` →
    /// `GroupReferencingActionExecute(SetPosition)` → `GroupReferencingStop`.
    /// When the group holds more than one axis, every axis's current position
    /// is read first and its siblings are set back to it.
    ///
    /// C checks only the last `GroupPositionCurrentGet` and `GroupInitialize`,
    /// which abort the sequence, and `GroupReferencingStop`, which it reports
    /// and then returns success for; the caller prints each line.
    pub fn set_position(
        &self,
        positioner: &str,
        group: &str,
        position: f64,
    ) -> Result<(), SetPositionFailure> {
        let grouped = self.axes_in_group(group) > 1;
        let mut current = Vec::new();
        if grouped {
            let mut axes: Vec<&AxisRef> = self.axes.iter().collect();
            axes.sort_by_key(|a| a.axis_no);
            let mut last = Ok(());
            for a in axes {
                let read = self.poll.group_position_current_get(&a.positioner);
                // C leaves a failed read's position unset; here it is 0.
                current.push((a, *read.as_ref().unwrap_or(&0.0)));
                last = read.map(|_| ());
            }
            last.map_err(|e| SetPositionFailure::CurrentGet(e.code()))?;
        }
        let _ = self.poll.group_kill(group);
        self.poll
            .group_initialize(group)
            .map_err(|e| SetPositionFailure::Initialize {
                code: e.code(),
                grouped,
            })?;

        // Settle after initialization so the stage does not oscillate.
        std::thread::sleep(self.set_position_settling);

        let _ = self.poll.group_referencing_start(group);
        // C indexes the cached positions by the in-group counter rather than
        // by axis number; each sibling here gets the position read for it.
        for (a, p) in &current {
            if a.group == group && a.positioner != positioner {
                let _ = self.poll.group_referencing_action_execute(
                    &a.positioner,
                    "SetPosition",
                    "None",
                    *p,
                );
            }
        }
        let _ =
            self.poll
                .group_referencing_action_execute(positioner, "SetPosition", "None", position);
        if grouped {
            // Some axis types (e.g. spindle) stay in referencing mode without it.
            std::thread::sleep(Duration::from_millis(50));
        }
        self.poll
            .group_referencing_stop(group)
            .map_err(|e| SetPositionFailure::ReferencingStop {
                code: e.code(),
                grouped,
            })
    }

    /// Whether `setPosition` is enabled (C `enableSetPosition_`).
    pub fn enable_set_position(&self) -> bool {
        self.enable_set_position
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// C `buildProfile`'s `switch (-status)` (XPSController.cpp:756-774).
    #[test]
    fn verify_message_matches_c_switch() {
        assert_eq!(verify_message(0), " ");
        assert_eq!(verify_message(-69), "Acceleration Too High");
        assert_eq!(verify_message(-68), "Velocity Too High");
        assert_eq!(verify_message(-70), "Final Velocity Non Zero");
        assert_eq!(verify_message(-75), "Negative or Null Delta Time");
        assert_eq!(verify_message(-1), "Unknown trajectory verify error=-1");
    }
}
