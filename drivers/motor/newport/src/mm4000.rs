//! Newport MM4000/MM4005/MM4006 motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvMM4000Asyn.c` (the model-3
//! asyn driver, the maintained reference). Commands are `{axis}CC{value}`
//! with a 1-based unpadded axis prefix; values are formatted with the
//! per-axis precision derived from the `TU` drive resolution (C
//! `maxDigits`). The C example st.cmd sets input and output EOS `"\r"`; this
//! driver owns framing (appends `\r`) and expects the port's input EOS to
//! frame replies (`asynOctetSetInputEos("\r")` in st.cmd).
//!
//! ## Units
//!
//! The MM4000 wire speaks physical units (mm/deg): C multiplies its raw-step
//! record boundary by the `TU` step size to reach them. The asyn-rs motor
//! boundary is dial-frame EGU — already the wire units — so positions,
//! velocities, and accelerations pass through unscaled. The `TU` step size
//! is still read at startup because C derives the command decimal precision
//! from it (`maxDigits = -log10(stepSize) + 2`).
//!
//! ## Controller-wide poll
//!
//! C polls per controller, not per axis: one `MS;` returns every axis's
//! status byte (`1MSx,2MSy,...`) and one `TP;` every position
//! (`1TP5.012,2TP1.123,...`). The per-axis [`AsynMotor::poll`] calls share
//! one cached exchange with a TTL of half the moving poll period, like the
//! HXP driver.
//!
//! ## Deviations from C (documented)
//!
//! - C queries `TE;` once per axis inside its poll loop even though the
//!   answer is controller-global (and a `TE` comm failure would mark the
//!   *next* axis in error); this port queries it once per poll cycle.
//! - C's MM4005 torque-off path (`MF` then busy-wait until the `MS` power
//!   bit reads ON) loops forever if the bit never clears; this port bounds
//!   the wait at 50 tries (5 s) and errors.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::util::{atof, atoi};

/// Response buffer size for a single controller reply (C `BUFFER_SIZE` 160).
const READ_BUF: usize = 256;

/// Command line terminator (C example st.cmd output EOS `"\r"`).
const TERMINATOR: &[u8] = b"\r";

/// C status-byte masks (same layout as the MM3000).
const MM4000_MOVING: u8 = 0x01;
const MM4000_POWER_OFF: u8 = 0x02;
const MM4000_DIRECTION: u8 = 0x04;
const MM4000_HIGH_LIMIT: u8 = 0x08;
const MM4000_LOW_LIMIT: u8 = 0x10;
const MM4000_HOME: u8 = 0x20;

/// Bounded replacement for C's unbounded MM4005 torque-off busy-wait.
const TORQUE_OFF_MAX_TRIES: usize = 50;

fn mm_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

/// Motion Master model from the firmware version (C `MM_model`): MM4005
/// covers the 4005/4006.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mm4000Model {
    Mm4000,
    Mm4005,
}

/// C model detection: `strstr(firmware, "MM")` then `atoi`; 4000 → MM4000,
/// 4005/4006 → MM4005, anything else is an error.
fn detect_model(firmware: &str) -> Option<Mm4000Model> {
    let idx = firmware.find("MM")?;
    match atoi(&firmware[idx + 2..]) {
        4000 => Some(Mm4000Model::Mm4000),
        4005 | 4006 => Some(Mm4000Model::Mm4005),
        _ => None,
    }
}

/// C `maxDigits`: command decimal precision from the `TU` drive resolution,
/// `(int)(-log10(stepSize)) + 2`, floored at 1.
fn max_digits(step_size: f64) -> usize {
    let digits = (-step_size.log10()) as i32 + 2;
    digits.max(1) as usize
}

/// Extract axis `axis` (0-based) status byte from the `MS;` reply
/// (`1MSx,2MSy,...` — C indexes `axis*5 + 3` directly).
fn parse_axis_status(status_all: &str, axis: usize) -> Option<u8> {
    status_all.as_bytes().get(axis * 5 + 3).copied()
}

/// Extract axis `axis` (0-based) position from the `TP;` reply
/// (`1TP5.012,2TP1.123,...` — C takes the nth comma field and `atof`s past
/// the 3-char `nTP` prefix).
fn parse_axis_position(position_all: &str, axis: usize) -> Option<f64> {
    let field = position_all.split(',').nth(axis)?;
    Some(atof(field.get(3..)?))
}

/// C controller-error check: `TE;` reply byte 2 is `@` when error-free.
fn te_ok(te_reply: &str) -> bool {
    te_reply.as_bytes().get(2) == Some(&b'@')
}

/// One controller-wide poll snapshot (`MS;` + `TP;` + `TE;`).
#[derive(Clone, Debug, Default)]
struct Mm4000PollData {
    status_bytes: Vec<Option<u8>>,
    positions: Vec<Option<f64>>,
    problem: bool,
    comms_error: bool,
}

/// Shared controller endpoint: owns the serial handle and the cached
/// controller-wide poll. The caller holds the `Arc<Mutex<..>>` lock.
pub struct Mm4000Controller {
    handle: SyncIOHandle,
    firmware: String,
    model: Mm4000Model,
    num_axes: usize,
    cache: Mm4000PollData,
    fetched_at: Option<Instant>,
    cache_ttl: Duration,
}

impl Mm4000Controller {
    /// Connect and identify an MM4000/4005/4006 (C `MM4000AsynConfig`):
    /// `VE;` up to 3 tries → firmware/model, then `TP;` — the number of
    /// comma-separated fields is the axis count on the wire, which must
    /// cover `num_axes`. Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle, num_axes: usize, cache_ttl: Duration) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            firmware: String::new(),
            model: Mm4000Model::Mm4000,
            num_axes,
            cache: Mm4000PollData::default(),
            fetched_at: None,
            cache_ttl,
        };
        let mut ve = String::new();
        for _ in 0..3 {
            if let Ok(reply) = ctrl.write_read("VE;") {
                ve = reply;
                break;
            }
        }
        if ve.is_empty() {
            return Err(mm_err("MM4000: no response to VE identity query".into()));
        }
        ctrl.firmware = ve.get(2..).unwrap_or("").to_string(); // skip "VE"
        ctrl.model = detect_model(&ctrl.firmware)
            .ok_or_else(|| mm_err(format!("MM4000: invalid model = {}", ctrl.firmware)))?;

        let tp = ctrl.write_read("TP;")?;
        let total_axes = tp.split(',').count();
        if total_axes < num_axes {
            return Err(mm_err(format!(
                "MM4000: actual number of axes={total_axes} < numAxes={num_axes}"
            )));
        }
        Ok(ctrl)
    }

    /// Firmware version from `VE;` (the `VE` prefix stripped).
    pub fn firmware(&self) -> &str {
        &self.firmware
    }

    /// Detected model.
    pub fn model(&self) -> Mm4000Model {
        self.model
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command (C `sendOnly`); the terminator is appended here.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        Ok(())
    }

    /// Write a command and read the reply (C `sendAndReceive`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw).into_owned())
    }

    /// Controller-wide poll with a TTL cache: `MS;` + `TP;` + `TE;` at most
    /// once per TTL; the six per-axis polls inside one record scan share one
    /// exchange (C polls once per controller cycle).
    fn poll_data(&mut self) -> Mm4000PollData {
        if let Some(at) = self.fetched_at
            && at.elapsed() < self.cache_ttl
        {
            return self.cache.clone();
        }
        let mut data = Mm4000PollData {
            status_bytes: vec![None; self.num_axes],
            positions: vec![None; self.num_axes],
            problem: false,
            comms_error: false,
        };
        match (self.write_read("MS;"), self.write_read("TP;")) {
            (Ok(status_all), Ok(position_all)) => {
                for axis in 0..self.num_axes {
                    data.status_bytes[axis] = parse_axis_status(&status_all, axis);
                    data.positions[axis] = parse_axis_position(&position_all, axis);
                }
                // C checks TE inside the per-axis loop; the answer is
                // controller-global, so one query per cycle (see module
                // Deviations).
                match self.write_read("TE;") {
                    Ok(te) => {
                        if !te_ok(&te) {
                            eprintln!("MM4000: controller error {te}");
                            data.problem = true;
                        }
                    }
                    Err(_) => data.comms_error = true,
                }
            }
            _ => data.comms_error = true,
        }
        self.cache = data.clone();
        self.fetched_at = Some(Instant::now());
        data
    }
}

/// One MM4000 axis sharing a controller. Implements [`AsynMotor`].
pub struct Mm4000Axis {
    controller: Arc<Mutex<Mm4000Controller>>,
    /// 1-based controller axis number, sent unpadded (`%d`).
    axis: usize,
    model: Mm4000Model,
    /// Command decimal precision (C `maxDigits`, from the `TU` resolution).
    digits: usize,
    /// Home preset position (`XH`, controller units), restored after a
    /// set-position `SH;DH` sequence.
    home_preset: f64,
    /// Controller travel limits (`TL`/`TR`, controller units) captured at
    /// init. C jogs to these and never refreshes them after `SL`/`SR`
    /// writes — kept bug-for-bug.
    low_limit: f64,
    high_limit: f64,
}

impl Mm4000Axis {
    /// Construct axis `axis` (1-based), running the C per-axis init queries:
    /// `TC` (closed-loop state — read for wire parity, C stores it unused),
    /// `TU` (drive resolution → command precision), `XH` (home preset),
    /// `TL`/`TR` (travel limits). Performs blocking serial I/O under the
    /// controller lock.
    pub fn new(controller: Arc<Mutex<Mm4000Controller>>, axis: usize) -> AsynResult<Self> {
        let (model, digits, home_preset, low_limit, high_limit) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            let _ = ctrl.write_read(&format!("{axis}TC"))?;
            // C `atof(&inputBuff[3])`: skip the echoed `nTU` prefix.
            let step_size = atof(
                ctrl.write_read(&format!("{axis}TU"))?
                    .get(3..)
                    .unwrap_or(""),
            );
            if step_size <= 0.0 || !step_size.is_finite() {
                return Err(mm_err(format!(
                    "MM4000 axis {axis}: invalid TU resolution {step_size}"
                )));
            }
            let home_preset = atof(
                ctrl.write_read(&format!("{axis}XH"))?
                    .get(3..)
                    .unwrap_or(""),
            );
            let low_limit = atof(
                ctrl.write_read(&format!("{axis}TL"))?
                    .get(3..)
                    .unwrap_or(""),
            );
            let high_limit = atof(
                ctrl.write_read(&format!("{axis}TR"))?
                    .get(3..)
                    .unwrap_or(""),
            );
            (
                ctrl.model(),
                max_digits(step_size),
                home_preset,
                low_limit,
                high_limit,
            )
        };
        Ok(Self {
            controller,
            axis,
            model,
            digits,
            home_preset,
            low_limit,
            high_limit,
        })
    }

    fn lock(&self) -> MutexGuard<'_, Mm4000Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `motorAxisMove` transaction: `AC;VA;PA|PR;` (trailing `;` as C
    /// sends it), values at this axis's precision.
    fn move_command(&self, cmd: &str, value: f64, velocity: f64, acceleration: f64) -> String {
        let (a, d) = (self.axis, self.digits);
        format!("{a}AC{acceleration:.d$};{a}VA{velocity:.d$};{a}{cmd}{value:.d$};")
    }
}

impl AsynMotor for Mm4000Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&self.move_command("PA", position, velocity, acceleration))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write(&self.move_command("PR", distance, velocity, acceleration))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C `motorAxisVelocityMove`: no jog command — move absolute to the
        // controller travel limit captured at init, in the jog direction.
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        };
        let ctrl = self.lock();
        ctrl.write(&self.move_command("PA", target, velocity.abs(), acceleration))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `motorAxisHome` — direction is not parameterized; the stray
        // space after the first `;` is C's format string, kept for wire
        // parity.
        let (a, d) = (self.axis, self.digits);
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{a}AC{acceleration:.d$}; {a}VA{velocity:.d$};{a}OR;"
        ))
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let (a, d) = (self.axis, self.digits);
        let ctrl = self.lock();
        ctrl.write(&format!("{a}AC{acceleration:.d$};{a}ST;"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `motorAxisPosition`: set home to the value, define home, then
        // restore the home preset (`SH;DH;SH`).
        let (a, d) = (self.axis, self.digits);
        let ctrl = self.lock();
        ctrl.write(&format!(
            "{a}SH{position:.d$};{a}DH;{a}SH{:.d$}",
            self.home_preset
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        // C `motorAxisClosedLoop`: the MM4000 can only switch ALL motors
        // (MO/MF), so it is a no-op there; the MM4005 addresses one axis.
        if self.model == Mm4000Model::Mm4000 {
            return Ok(());
        }
        let a = self.axis;
        let ctrl = self.lock();
        if enable {
            return ctrl.write(&format!("{a}MO"));
        }
        ctrl.write(&format!("{a}MF"))?;
        // C busy-waits (unbounded) until the axis power bit reads ON after
        // MF; bounded here (see module Deviations).
        for _ in 0..TORQUE_OFF_MAX_TRIES {
            let status_all = ctrl.write_read("MS;")?;
            if let Some(byte) = parse_axis_status(&status_all, self.axis - 1)
                && byte & MM4000_POWER_OFF == 0
            {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(100));
        }
        Err(mm_err(format!(
            "MM4000 axis {a}: power did not return after MF"
        )))
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let (a, d) = (self.axis, self.digits);
        let ctrl = self.lock();
        ctrl.write(&format!("{a}SR{position:.d$}"))
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let (a, d) = (self.axis, self.digits);
        let ctrl = self.lock();
        ctrl.write(&format!("{a}SL{position:.d$}"))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        Err(mm_err(format!(
            "MM4000 does not support setting {kind:?} gain"
        )))
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let data = self.lock().poll_data();
        let idx = self.axis - 1;
        if data.comms_error {
            return Ok(MotorStatus {
                comms_error: true,
                problem: true,
                ..MotorStatus::default()
            });
        }
        let byte = data.status_bytes[idx].ok_or_else(|| {
            mm_err(format!(
                "MM4000 axis {}: missing MS status field",
                self.axis
            ))
        })?;
        let position = data.positions[idx].ok_or_else(|| {
            mm_err(format!(
                "MM4000 axis {}: missing TP position field",
                self.axis
            ))
        })?;
        Ok(MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0, // C: no way to query the actual velocity
            done: byte & MM4000_MOVING == 0,
            moving: byte & MM4000_MOVING != 0,
            direction: byte & MM4000_DIRECTION != 0,
            high_limit: byte & MM4000_HIGH_LIMIT != 0,
            low_limit: byte & MM4000_LOW_LIMIT != 0,
            home: byte & MM4000_HOME != 0,
            powered: byte & MM4000_POWER_OFF == 0,
            problem: data.problem,
            comms_error: false,
            // C motorAxisInit defaults motorAxisHasClosedLoop on for every
            // axis; the TC closed-loop probe result is stored but unused.
            gain_support: true,
            has_encoder: true,
            // The MM4000 move transaction has no base-velocity (VB) command.
            vbas_supported: false,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_detection_via_mm_substring() {
        assert_eq!(detect_model("MM4000 2.04"), Some(Mm4000Model::Mm4000));
        assert_eq!(detect_model(" MM4005 f/w 1.0"), Some(Mm4000Model::Mm4005));
        assert_eq!(detect_model("MM4006"), Some(Mm4000Model::Mm4005));
        assert_eq!(detect_model("MM3000"), None);
        assert_eq!(detect_model("ESP300"), None);
    }

    #[test]
    fn max_digits_matches_c_truncation() {
        // C: (int)(-log10(step)) + 2, floored at 1.
        assert_eq!(max_digits(0.001), 5);
        assert_eq!(max_digits(0.0005), 5); // -log10 = 3.30 → (int) 3 → 5
        assert_eq!(max_digits(0.1), 3);
        assert_eq!(max_digits(1.0), 2);
        assert_eq!(max_digits(100.0), 1); // -2 + 2 = 0 → floored at 1
    }

    #[test]
    fn status_string_indexes_fixed_offsets() {
        // "1MSx,2MSy,..." — byte at axis*5 + 3.
        let ms = "1MSP,2MS@,3MS\x21";
        assert_eq!(parse_axis_status(ms, 0), Some(b'P'));
        assert_eq!(parse_axis_status(ms, 1), Some(b'@'));
        assert_eq!(parse_axis_status(ms, 2), Some(0x21));
        assert_eq!(parse_axis_status(ms, 3), None);
    }

    #[test]
    fn position_string_takes_nth_comma_field() {
        let tp = "1TP5.012,2TP1.123,3TP-100.567";
        assert_eq!(parse_axis_position(tp, 0), Some(5.012));
        assert_eq!(parse_axis_position(tp, 1), Some(1.123));
        assert_eq!(parse_axis_position(tp, 2), Some(-100.567));
        assert_eq!(parse_axis_position(tp, 3), None);
    }

    #[test]
    fn te_reply_ok_only_on_at_sign() {
        assert!(te_ok("TE@"));
        assert!(!te_ok("TEA"));
        assert!(!te_ok(""));
    }
}
