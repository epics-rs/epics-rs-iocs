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

use super::rpc::{XpsResult, XpsSocket};

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
    /// XPS group name of each registered axis, for group-membership counts
    /// (C `XPSAxis::isInGroup`).
    axis_groups: Vec<String>,
    /// Groups currently in referencing (move-to-home) mode. C tracks this
    /// per-axis as `referencingMode_`, but sets it for every axis in the group
    /// at once (`doMoveToHome`/`home`), so it is group-scoped state. While a
    /// group is here, poll reports its axes as not-homed/not-at-home.
    referencing_groups: HashSet<String>,
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
            axis_groups: Vec::new(),
            referencing_groups: HashSet::new(),
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

    /// Record an axis's group so group-membership counts stay accurate.
    pub fn register_axis(&mut self, group: &str) {
        self.axis_groups.push(group.to_string());
    }

    /// Number of registered axes in `group` (C `XPSAxis::isInGroup`).
    pub fn axes_in_group(&self, group: &str) -> usize {
        self.axis_groups.iter().filter(|g| *g == group).count()
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
}
