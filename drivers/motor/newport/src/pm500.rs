//! Newport PM500 precision motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvPM500.cc` + `devPM500.cc`
//! (a model-1 dev/drv pair, itself derived from drvMM4000). Axes are named
//! by letter — `X Y Z A B C D E F G H I` — and commands are `{letter}CC` with
//! values at 4 decimals (C `res_decpts` from the fixed 0.01 drive
//! resolution). The controller is put in `SCUM 1` mode with `SENAINT $AF` at
//! startup: responses are CR-terminated with the axis letter prefixed and no
//! command acknowledgement suppression, and the C driver reads one reply
//! after *every* command message (`cmnd_response = true`) — this port does
//! the same via [`Pm500Controller::command`].
//!
//! The C example st.cmd sets no EOS (GPIB-era, EOI-terminated); for RS-232
//! this driver owns framing like its siblings (appends `\r`) and expects
//! `asynOctetSetInputEos("\r")` in st.cmd.
//!
//! ## Units
//!
//! The PM500 wire speaks microns/arc-sec for positions but mm/sec and
//! karc-sec/sec for velocities (and /sec² for accelerations) — C divides
//! velocity and acceleration by 1000 after its raw-step conversion. The
//! asyn-rs motor boundary is dial-frame EGU (= wire position units, µm or
//! arc-sec), so positions pass through unscaled and velocities/accelerations
//! keep only the genuine ÷1000 unit conversion. C's `drive_resolution`
//! (0.01 for every firmware type it recognizes) existed only for the
//! raw-step record boundary; it survives here solely as the 4-decimal
//! command precision.
//!
//! ## Deviations from C (documented)
//!
//! - C leaves `drive_resolution` at 0.0 for an unrecognized `CONFIG?`
//!   firmware string, which divides by zero in `set_status` (positions read
//!   ±inf); this port errors at axis creation instead, naming the firmware.
//! - C's axis-discovery and status parsing index fixed reply offsets into a
//!   buffer that may hold stale bytes after a read failure; this port treats
//!   a too-short reply like an empty one (comm-retry machine).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::util::{atof, max_digits};

/// Response buffer size for a single controller reply (C `BUFF_SIZE` 100).
const READ_BUF: usize = 256;

/// Command line terminator (`ENAINT` bit 7: CR-only framing).
const TERMINATOR: &[u8] = b"\r";

/// Axis letters, in controller channel order (C `PM500_axis_names`,
/// `PM500_NUM_CHANNELS` 12).
const PM500_AXIS_NAMES: [char; 12] = ['X', 'Y', 'Z', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'];

fn pm_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

/// Controller communication state (C `cntrl->status`): one empty status
/// reply is retried silently; a second consecutive one is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// C `motor_init` firmware lookup: the drive resolution comes from the axis
/// firmware reported by `CONFIG?` — 302 (50 nm translator), 309 (25 nm
/// translator), 300 (unknown translator), and the XXX rotator placeholder
/// all map to 0.01. Unknown firmware is `None` (C leaves 0.0 and later
/// divides by it — see module Deviations).
fn resolution_for_firmware(firmware: &str) -> Option<f64> {
    match firmware {
        "302" | "309" | "300" | "XXX" => Some(0.01),
        _ => None,
    }
}

/// Shared controller endpoint: owns the serial handle and the cross-axis
/// communication state. The caller holds the `Arc<Mutex<..>>` lock.
pub struct Pm500Controller {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
    comm_state: CommState,
}

impl Pm500Controller {
    /// Connect and identify a PM500 (C `motor_init`): put the controller in
    /// `SCUM 1` mode, configure `SENAINT $AF` framing, probe `SVN?` for the
    /// identity, then discover axes by querying `{letter}STAT?` for each of
    /// the 12 channel letters until one answers with an error (`E` at reply
    /// byte 1). Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        // C sends SCUM 1 and SENAINT $AF and reads (ignoring) one reply each.
        ctrl.write("SCUM 1")?;
        let _ = ctrl.read_once();
        ctrl.write("SENAINT $AF")?;
        let _ = ctrl.read_once();

        // Identity probe (C sends SVN? twice: existence check, then read).
        let probe = ctrl.command("SVN?").unwrap_or_default();
        if probe.is_empty() {
            return Err(pm_err(
                "PM500: no response to SVN? identity query".to_string(),
            ));
        }
        let ident = ctrl.command("SVN?").unwrap_or_default();
        ctrl.ident = ident.get(2..).unwrap_or("").to_string(); // skip "XD"

        // Axis discovery: STAT? each channel letter in order; an 'E' at
        // reply byte 1 ends the scan (C checks buff[1]).
        let mut num_axes = 0;
        for name in PM500_AXIS_NAMES {
            let reply = ctrl.command(&format!("{name}STAT?")).unwrap_or_default();
            match reply.as_bytes().get(1) {
                Some(b'E') | None => break,
                Some(_) => num_axes += 1,
            }
        }
        ctrl.num_axes = num_axes;
        Ok(ctrl)
    }

    /// Identity string from `SVN?` (the 2-byte echo prefix stripped).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of axes discovered at construction.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command message (C `send_mess`); the terminator is appended
    /// here.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    fn read_once(&self) -> AsynResult<String> {
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw).into_owned())
    }

    /// Write one command message and read its one reply — C's motor task
    /// pairs every `send_mess` with a `recv_mess` (`cmnd_response = true`),
    /// so even pure motion messages consume a response. C `recv_mess` logs
    /// a "system error" on an `SE` reply but still returns it.
    fn command(&self, cmd: &str) -> AsynResult<String> {
        self.write(cmd)?;
        let reply = self.read_once()?;
        if reply.starts_with("SE") {
            eprintln!("PM500 system error: {reply}");
        }
        Ok(reply)
    }
}

/// One PM500 axis sharing a controller. Implements [`AsynMotor`].
pub struct Pm500Axis {
    controller: Arc<Mutex<Pm500Controller>>,
    /// Controller channel letter (C `PM500_axis_names[signal]`).
    name: char,
    /// Command decimal precision (C `res_decpts`, from the firmware-table
    /// drive resolution — 4 for every recognized type).
    digits: usize,
    /// Last polled status, reused on the comm-retry early exit where C
    /// leaves the record's bits stale.
    last_status: MotorStatus,
}

impl Pm500Axis {
    /// Construct the axis at 0-based channel `index`, running the C
    /// per-motor init: `{letter}CONFIG?` → firmware string at reply offset 8
    /// → drive resolution by lookup (only the command precision survives the
    /// EGU boundary — module Units note). Performs blocking serial I/O under
    /// the controller lock.
    pub fn new(controller: Arc<Mutex<Pm500Controller>>, index: usize) -> AsynResult<Self> {
        let name = *PM500_AXIS_NAMES.get(index).ok_or_else(|| {
            pm_err(format!(
                "PM500: axis index {index} out of range 0..{}",
                PM500_AXIS_NAMES.len()
            ))
        })?;
        let digits = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let config = ctrl.command(&format!("{name}CONFIG?"))?;
            let firmware = config.get(8..).unwrap_or("");
            let resolution = resolution_for_firmware(firmware).ok_or_else(|| {
                pm_err(format!(
                    "PM500 axis {name}: unrecognized CONFIG? firmware \"{firmware}\" \
                     (no drive resolution known)"
                ))
            })?;
            max_digits(resolution)
        };
        Ok(Self {
            controller,
            name,
            digits,
            last_status: MotorStatus::default(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Pm500Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for Pm500Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Record move transaction: SET_VELOCITY;SET_ACCEL;MOVE_ABS (VEL_BASE
        // and GO build nothing). Wire velocity/acceleration are mm/sec(²) —
        // the ÷1000 from position units is C's, kept (module Units note).
        let (c, d) = (self.name, self.digits);
        let ctrl = self.lock();
        ctrl.command(&format!(
            "{c}V{:.d$};{c}ACCEL{:.d$};{c}G{position:.d$};",
            velocity / 1000.0,
            acceleration / 1000.0
        ))?;
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let (c, d) = (self.name, self.digits);
        let ctrl = self.lock();
        ctrl.command(&format!(
            "{c}V{:.d$};{c}ACCEL{:.d$};{c}R{distance:.d$};",
            velocity / 1000.0,
            acceleration / 1000.0
        ))?;
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // Record JOG transaction: SET_ACCEL then the signed slew command
        // (C JOG case, `%f` — 6 decimals, unlike the other values).
        let (c, d) = (self.name, self.digits);
        let ctrl = self.lock();
        ctrl.command(&format!(
            "{c}ACCEL{:.d$};{c}S{:.6};",
            acceleration / 1000.0,
            velocity / 1000.0
        ))?;
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C builds the same `F0` for HOME_FOR and HOME_REV, after the
        // record's velocity/acceleration transaction parts.
        let (c, d) = (self.name, self.digits);
        let ctrl = self.lock();
        ctrl.command(&format!(
            "{c}V{:.d$};{c}ACCEL{:.d$};{c}F0;",
            velocity / 1000.0,
            acceleration / 1000.0
        ))?;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS: T (stop) then an R status query in the same message.
        let c = self.name;
        let ctrl = self.lock();
        ctrl.command(&format!("{c}T;{c}R"))?;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // C LOAD_POS builds no command (the PM500 cannot redefine its
        // position) and reports success; kept as a quiet no-op.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C ENABLE_TORQUE → `T;M?`, DISABL_TORQUE → `M;M?` (the M? readback
        // is consumed as the message's reply).
        let c = self.name;
        let ctrl = self.lock();
        if enable {
            ctrl.command(&format!("{c}T;{c}M?"))?;
        } else {
            ctrl.command(&format!("{c}M;{c}M?"))?;
        }
        Ok(())
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let (c, d) = (self.name, self.digits);
        let ctrl = self.lock();
        ctrl.command(&format!("{c}PSLIM{position:.d$};"))?;
        Ok(())
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let (c, d) = (self.name, self.digits);
        let ctrl = self.lock();
        ctrl.command(&format!("{c}NSLIM{position:.d$};"))?;
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C SET_[PID]GAIN builds no command but reports success (the record
        // sees GAIN_SUPPORT set); kept as a quiet no-op.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Port of C `set_status`: one `{letter}R` status/position query, then
        // `{letter}M?` for the servo state. Reply format
        // `[letter][status][sign+digits...]`, e.g. `XB+5.012`.
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        let c = self.name;

        // C turns any read failure into an empty reply and runs the retry
        // machine on it; a too-short reply is treated the same (module
        // Deviations).
        let reply = ctrl.command(&format!("{c}R")).unwrap_or_default();
        if reply.len() < 3 {
            if ctrl.comm_state == CommState::Normal {
                // One empty reply: retry silently next poll (C RETRY state),
                // leaving the record's bits stale.
                ctrl.comm_state = CommState::Retry;
                return Ok(self.last_status.clone());
            }
            ctrl.comm_state = CommState::CommErr;
            self.last_status.comms_error = true;
            self.last_status.problem = true;
            return Ok(self.last_status.clone());
        }
        ctrl.comm_state = CommState::Normal;

        let status_char = reply.as_bytes()[1];
        let dir_char = reply.as_bytes()[2];
        // Position parses from the direction sign onward; wire units are the
        // record EGU (module Units note).
        let position = atof(&reply[2..]);

        let moving = status_char == b'B';
        let direction = dir_char == b'+';
        let on_limit = status_char == b'L';

        // Servo on/off (C maps the M? readback to EA_POSITION).
        let servo = ctrl.command(&format!("{c}M?")).unwrap_or_default();
        let powered = atof(servo.get(2..).unwrap_or("")) as i32 != 0;
        drop(ctrl);

        self.last_status = MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0, // C: "Parse motor velocity? NEEDS WORK"
            done: !moving,
            moving,
            direction,
            high_limit: on_limit && direction,
            low_limit: on_limit && !direction,
            home: false,
            powered,
            // C sets RA_PROBLEM from an 'E' status but unconditionally
            // clears it a few lines later (upstream dead store) — net
            // effect: never a problem outside the comm-error path. Kept
            // bug-for-bug.
            problem: false,
            comms_error: false,
            // C: "PM500 only supports DC motors" — EA_PRESENT and
            // GAIN_SUPPORT are hard-wired on.
            gain_support: true,
            has_encoder: true,
            // C SET_VEL_BASE: "PM500 does not use base velocity".
            vbas_supported: false,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn firmware_lookup_matches_c_table() {
        assert_eq!(resolution_for_firmware("302"), Some(0.01));
        assert_eq!(resolution_for_firmware("309"), Some(0.01));
        assert_eq!(resolution_for_firmware("300"), Some(0.01));
        assert_eq!(resolution_for_firmware("XXX"), Some(0.01));
        assert_eq!(resolution_for_firmware("310"), None);
        assert_eq!(resolution_for_firmware(""), None);
    }

    #[test]
    fn axis_letters_match_c_channel_order() {
        assert_eq!(PM500_AXIS_NAMES[0], 'X');
        assert_eq!(PM500_AXIS_NAMES[3], 'A');
        assert_eq!(PM500_AXIS_NAMES[11], 'I');
    }
}
