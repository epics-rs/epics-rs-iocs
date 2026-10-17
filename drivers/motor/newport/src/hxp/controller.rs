//! Shared HXP controller state: the poll socket and the group-wide poll cache.
//!
//! Port of the controller half of `HXPDriver.cpp`. The hexapod is a single
//! six-axis group, so C polls at the *controller* level (`HXPController::poll`:
//! firmware probe, `GroupStatusGet`, two position-array reads) and each axis
//! just reads its slice. [`HxpController::poll`] is that controller poll, run
//! once per pass of the axes' one poller ahead of the
//! [`HxpAxis`](super::HxpAxis) polls.

use std::sync::Arc;

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use super::axis::MoveLine;
use super::params::HxpParams;
use crate::xps::rpc::{XpsResult, XpsSocket};

/// The hexapod's fixed group name (C `GROUP`).
pub const HXP_GROUP: &str = "HEXAPOD";

/// Number of hexapod axes: X, Y, Z, U, V, W (C `NUM_AXES`).
pub const NUM_HXP_AXES: usize = 6;

/// The record steps per hexapod mm/deg (C HXPDriver.cpp's `MRES`): moves
/// send `steps * MRES` and polls return `position / MRES`, so the record's
/// MRES must be this value.
pub const MRES: f64 = 0.00001;

/// Coordinate system used for motor-record moves (C `HXP_MOVE_COORD_SYS`
/// param: 0=Work, 1=Tool).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveCoordSys {
    Work,
    Tool,
}

/// One group-wide poll result, served to all six axes.
#[derive(Clone, Debug, Default)]
pub struct HxpPollData {
    /// `GroupStatusGet` code (C `groupStatus_`, posted as `HXP_STATUS`).
    pub group_status: i32,
    /// Current encoder positions, device units (C `encoderPosition_`).
    pub encoder: [f64; 6],
    /// "Setpoint" positions, device units (C `setpointPosition_`; see the
    /// upstream quirk note in [`HxpController::poll`]).
    pub setpoint: [f64; 6],
    pub moving: bool,
    pub problem: bool,
    pub powered: bool,
    pub homed: bool,
    pub comms_error: bool,
}

/// Motor status bits derived from an HXP group status code
/// (`HXPController::poll`), with the classic-firmware and HXP-D branches.
#[derive(Debug, PartialEq, Eq)]
struct HxpStatusFlags {
    moving: bool,
    problem: bool,
    powered: bool,
    homed: bool,
}

impl HxpStatusFlags {
    fn from_status(status: i32, hxpd: bool) -> Self {
        if hxpd {
            // HXP-D branch: power is always on unless explicitly disabled.
            let mut f = HxpStatusFlags {
                moving: false,
                problem: false,
                powered: true,
                homed: false,
            };
            if status < 10 {
                f.problem = true;
            } else if status < 20 {
                f.homed = true;
            } else if status < 40 {
                // All good but disabled.
                f.powered = false;
                f.homed = true;
            } else if status <= 41 {
                f.problem = true; // 40=emergency braking, 41=motor init
            } else if status == 42 {
                f.problem = true; // not referenced
            } else if status == 43 {
                f.moving = true; // homing
            } else if status <= 48 {
                f.moving = true; // moving
                f.homed = true;
            } else {
                f.problem = true; // 49=encoder calibrating; assume problem
            }
            f
        } else {
            // Classic firmware branch. C never sets homed here.
            let moving = (43..=48).contains(&status);
            let cannot_move =
                status < 10 || (20..=42).contains(&status) || status == 50 || status == 64;
            HxpStatusFlags {
                moving,
                // Status 20 is a normal disabled state, not a problem.
                problem: cannot_move && status != 20,
                powered: !cannot_move,
                homed: false,
            }
        }
    }
}

/// Shared controller state for one HXP, owning the poll socket.
pub struct HxpController {
    poll: XpsSocket,
    /// Coordinate system for motor-record moves (C `HXPMoveCoordSys_` param,
    /// default Work).
    move_coord_sys: MoveCoordSys,
    /// Firmware version; empty until a probe returns a plausible (`HXP`)
    /// string — C re-probes on every poll until then.
    firmware: String,
    /// Firmware is HXP-D (C `is_firmware_hxpd_`): different status decoding
    /// and position RPC name.
    is_hxpd: bool,
    /// What the last controller poll read, for the axis polls after it.
    data: HxpPollData,
    /// Each axis's move line, for the parameter writes C runs on it.
    lines: [Option<Arc<MoveLine>>; NUM_HXP_AXES],
    /// The controller port's parameters and, once published, its handle.
    pub(super) params: Option<HxpParams>,
    port: Option<PortHandle>,
}

impl HxpController {
    /// Build the controller over a connected `Query`-mode poll socket. The
    /// firmware probe happens lazily on the first poll (C probes in `poll()`,
    /// not the constructor).
    pub fn new(poll: XpsSocket) -> Self {
        Self {
            poll,
            move_coord_sys: MoveCoordSys::Work,
            firmware: String::new(),
            is_hxpd: false,
            data: HxpPollData::default(),
            lines: Default::default(),
            params: None,
            port: None,
        }
    }

    /// Post what the poll reads to the controller port from here on.
    pub fn attach_port(&mut self, port: PortHandle) {
        self.port = Some(port);
    }

    /// Axis `axis_no`'s move line, which the parameter writes use too.
    pub fn add_line(&mut self, axis_no: usize, line: Arc<MoveLine>) {
        self.lines[axis_no] = Some(line);
    }

    pub(super) fn line(&self, addr: i32) -> Option<Arc<MoveLine>> {
        let axis_no = usize::try_from(addr).ok()?;
        self.lines.get(axis_no)?.clone()
    }

    /// Set address-0 `values` and run the callbacks at `addr`: C sets
    /// `pC_`'s parameters and flushes the caller's address.
    pub(super) fn post(&self, addr: i32, values: Vec<(usize, ParamValue)>) {
        let Some(port) = &self.port else {
            return;
        };
        let updates = values
            .into_iter()
            .map(|(reason, value)| ParamSetValue::new(reason, 0, value))
            .collect();
        let _ = port.set_params_and_notify_blocking(addr, updates);
    }

    /// The shared poll socket (`Query` mode).
    pub fn poll_socket(&self) -> &XpsSocket {
        &self.poll
    }

    /// Firmware version string (empty until the first successful probe).
    pub fn firmware(&self) -> &str {
        &self.firmware
    }

    /// Coordinate system for motor-record moves.
    pub fn move_coord_sys(&self) -> MoveCoordSys {
        self.move_coord_sys
    }

    /// Select the coordinate system for motor-record moves
    /// (C `HXP_MOVE_COORD_SYS`).
    pub fn set_move_coord_sys(&mut self, cs: MoveCoordSys) {
        self.move_coord_sys = cs;
    }

    /// Read all six current positions over the poll socket (used by the
    /// absolute move to fill the untouched axes' targets).
    pub fn current_positions(&self) -> XpsResult<[f64; 6]> {
        self.poll.hexapod_positions_get(HXP_GROUP, self.is_hxpd)
    }

    /// What the last [`Self::poll`] read.
    pub fn data(&self) -> &HxpPollData {
        &self.data
    }

    /// C `HXPController::poll`. On an RPC failure the firmware probe is
    /// reset and every axis reports problem + comms error until a poll
    /// succeeds, exactly as C's `done:` path.
    pub fn poll(&mut self) {
        let mut status = None;
        let polled = self.poll_once(&mut status);
        // C sets HXP_STATUS once GroupStatusGet answers, and every poll ends
        // in callParamCallbacks() at address 0.
        let values = match (self.params, status) {
            (Some(p), Some(status)) => vec![(p.status, ParamValue::Int32(status))],
            _ => Vec::new(),
        };
        self.post(0, values);
        match polled {
            Ok(data) => self.data = data,
            Err(_) => {
                // C: status error → firmware re-probed next poll; all axes get
                // problem=1, commsError=1, homed/powerOn=0; positions and the
                // moving flag keep their last values.
                self.firmware.clear();
                self.data.problem = true;
                self.data.comms_error = true;
                self.data.powered = false;
                self.data.homed = false;
            }
        }
    }

    fn poll_once(&mut self, status: &mut Option<i32>) -> XpsResult<HxpPollData> {
        if self.firmware.is_empty() {
            let fw = self.poll.firmware_version_get()?;
            if fw.contains("HXP") {
                self.is_hxpd = fw.contains("HXP-D ");
                self.firmware = fw;
            }
            // Junk that does not mention HXP (socket noise, C's comment) is
            // discarded and re-probed on the next poll; the poll continues.
        }
        let group_status = self.poll.group_status_get(HXP_GROUP)?;
        *status = Some(group_status);
        let flags = HxpStatusFlags::from_status(group_status, self.is_hxpd);
        let encoder = self.poll.hexapod_positions_get(HXP_GROUP, self.is_hxpd)?;
        // Upstream quirk kept for wire parity: C `HXPGroupPositionSetpointGet`
        // sends the *current*-position function name (`str_XPositionCurrentGet`
        // in hxp_drivers.cpp), so the "setpoint" array is really a second
        // current-position sample.
        let setpoint = self.poll.hexapod_positions_get(HXP_GROUP, self.is_hxpd)?;
        Ok(HxpPollData {
            group_status,
            encoder,
            setpoint,
            moving: flags.moving,
            problem: flags.problem,
            powered: flags.powered,
            homed: flags.homed,
            comms_error: false,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classic_moving_range() {
        for s in 43..=48 {
            assert!(HxpStatusFlags::from_status(s, false).moving, "status {s}");
        }
        assert!(!HxpStatusFlags::from_status(42, false).moving);
        assert!(!HxpStatusFlags::from_status(49, false).moving);
    }

    #[test]
    fn classic_problem_and_power() {
        // Uninitialised / not referenced / disabled ranges are problems...
        for s in [0, 9, 21, 42, 50, 64] {
            let f = HxpStatusFlags::from_status(s, false);
            assert!(f.problem, "status {s}");
            assert!(!f.powered, "status {s}");
        }
        // ...except 20, the normal disabled state (not a problem, still
        // unpowered — C leaves powerOn 0 on that branch).
        let f = HxpStatusFlags::from_status(20, false);
        assert!(!f.problem);
        assert!(!f.powered);
        // Ready / moving states are powered, no problem.
        for s in [10, 19, 43, 48, 49] {
            let f = HxpStatusFlags::from_status(s, false);
            assert!(!f.problem, "status {s}");
            assert!(f.powered, "status {s}");
        }
    }

    #[test]
    fn classic_never_homed() {
        // C's classic branch never sets polled_motorStatusHomed.
        for s in [0, 10, 20, 43, 44, 48] {
            assert!(!HxpStatusFlags::from_status(s, false).homed, "status {s}");
        }
    }

    #[test]
    fn hxpd_branches() {
        // < 10: problem.
        assert!(HxpStatusFlags::from_status(9, true).problem);
        // 10..19: homed, powered, no problem.
        let f = HxpStatusFlags::from_status(11, true);
        assert!(f.homed && f.powered && !f.problem && !f.moving);
        // 20..39: disabled — homed but unpowered.
        let f = HxpStatusFlags::from_status(20, true);
        assert!(f.homed && !f.powered && !f.problem);
        let f = HxpStatusFlags::from_status(39, true);
        assert!(f.homed && !f.powered);
        // 40/41 emergency braking / motor init, 42 not referenced: problem.
        for s in [40, 41, 42] {
            assert!(HxpStatusFlags::from_status(s, true).problem, "status {s}");
        }
        // 43 homing: moving, not homed.
        let f = HxpStatusFlags::from_status(43, true);
        assert!(f.moving && !f.homed && !f.problem);
        // 44..48 moving: moving + homed.
        for s in 44..=48 {
            let f = HxpStatusFlags::from_status(s, true);
            assert!(f.moving && f.homed, "status {s}");
        }
        // 49+: assume problem (encoder calibrating).
        assert!(HxpStatusFlags::from_status(49, true).problem);
        // HXP-D power is on except in the explicit disabled range.
        assert!(HxpStatusFlags::from_status(9, true).powered);
        assert!(HxpStatusFlags::from_status(44, true).powered);
    }
}
