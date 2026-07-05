//! PI GCS2 generic stage-controller protocol (ASCII over asyn octet).
//!
//! Ported from `motorPIGCS2/pigcs2App/src/PIGCSController.cpp` (protocol
//! core, base class) + `PIGCSMotorController.cpp` (the C-863/C-867/C-663/
//! C-884/E-861/E-871/E-873 motor specialization) — the two classes the task
//! scopes as "generic GCS2 stage-controller path". Commands are plain ASCII
//! text terminated by `\n`; the driver owns output framing and the startup
//! script sets only the input EOS (`\n`), as in the C module's
//! `PIasynController` constructor (`pasynOctetSyncIO->setInputEos(...,"\n",1)`
//! plus `setOutputEos(...,"",0)`). Axes are addressed by the GCS *name
//! string* returned by `SAI?` (e.g. `"1"`), not a bare index.
//!
//! ## Wire shape
//!
//! Two distinct exchange patterns, both confirmed from `PIInterface.cpp`:
//!
//! - **Set commands** (`MOV`, `VEL`, `SPA` (set), `SVO` (set), `RON`, `POS`,
//!   `HLT`, `FRF`/`FPL`/`FNL`) use `sendOnly`: a bare write, **no reply at
//!   all**. Any error is retrieved separately via a following `ERR?` query —
//!   and only some set commands actually check it (see below).
//! - **Queries** (`POS?`, `VEL?`, `TMN?`/`TMX?`, `LIM?`, `HAR?`, `TRS?`,
//!   `SVO?`, `FRF?`, `SPA?`, `ERR?`) use `sendAndReceive`: write then read one
//!   reply line. Value replies are `{axis}={value}`; `ERR?` alone replies
//!   with a bare decimal (no `=`).
//!
//! `ERR?`-after-set is **not** applied uniformly — this port preserves the
//! exact per-command gating found in the C source:
//!
//! | command       | error check                          |
//! |----------------|--------------------------------------|
//! | `MOV`          | `ERR? == 0`                           |
//! | `RON`/`POS`/`RON` (setPosition) | one `ERR? == 0` after all three |
//! | `HLT`          | `ERR? == PI_CNTR_STOP` (10)            |
//! | `SVO` (set)    | `ERR? == 0`                           |
//! | `FRF`/`FPL`/`FNL` (referenceAxis) | `ERR? == 0`         |
//! | `SPA 0x50` (reference velocity) | `ERR? == 0` (checked inline in `referenceVelCts`, unlike plain `SPA` below) |
//! | `VEL` (set)    | none (`setVelocityCts` is fire-and-forget) |
//! | `SPA` (accel/decel set) | none (`setGCSParameter` is fire-and-forget) |
//!
//! `SAI?` (axis discovery) is the one multi-line reply: PI GCS marks a
//! non-final line with a trailing space before the EOS (`PIInterface::
//! sendAndReceive`'s `while(inputBuff[strlen(inputBuff)-1]==' ')` continuation
//! loop) — [`PIGCS2Controller::find_connected_axes`] replicates that instead
//! of assuming one line per reply.
//!
//! ## Units
//!
//! `POS`/`MOV`/`TMN`/`TMX` are already physical EGU on the controller (unlike
//! e.g. the SPiiPlus port). The C driver additionally carries a
//! counts-per-unit (CPU) integer fraction (`PI_PARA_MOT_CPU_Z`/`_N`,
//! `getResolution`) used only to convert between the record's raw-step frame
//! and the controller's EGU — at the asyn-rs motor boundary (already EGU) that
//! conversion cancels, so it is dropped here: no CPU query, `MRES` = 1.
//!
//! ## Config (single-step, matching upstream — not a two-step registry)
//!
//! `PIasynController`'s C++ constructor is single-step:
//! `PI_GCS2_CreateController(portName, asynPort, numAxes, priority, stackSize,
//! movingPollingRate, idlePollingRate)` connects, reads `*IDN?`, probes
//! `VEL?` (sets `m_KnowsVELcommand`), auto-discovers every connected axis name
//! via `SAI?` (`PIGCSController::findConnectedAxes`), and creates
//! `PIasynAxis` objects for the *first* `numAxes` of them, in discovery order
//! — there is no explicit per-axis create command anywhere in this module.
//! [`PIGCS2Controller::new`] mirrors that probe sequence exactly, and
//! `ioc::pigcs2_config_command` mirrors the axis-count check (error if fewer
//! axes were found than requested) and the first-N-in-order axis creation.
//! `priority`/`stackSize` (OS thread-scheduling knobs) are dropped, matching
//! every other port in this workspace (there is no OS thread to schedule).
//!
//! ## Homing
//!
//! `PIGCSMotorController::referenceAxis`: `setServo(1)`, then `FRF` if the
//! axis has a reference sensor (`TRS?`), else `FPL`/`FNL` by direction if it
//! has limit switches (`LIM?`, falling back to `HAR?` on `PI_CNTR_UNKNOWN_
//! COMMAND`), else an error. No homing-method config argument exists or is
//! needed — the method is fully determined by the two capability probes
//! (done once, at axis construction) plus the record's home direction.
//!
//! ## Status
//!
//! `PIGCSMotorController::getStatus` sends the single raw byte `0x04` (no
//! output framing — this is a binary control code, not ASCII text) and
//! receives a hex bitmask string; the 4 hex chars for axis `n` sit at
//! `[2+4n .. 2+4n+4)`. This is documented in the C source itself as
//! "TODO this is for C-863/867 controllers!!!! TODO support other controllers
//! which do not understand #4 or have different bit masks" — i.e. even
//! upstream models this as the one implemented status transport, with other
//! models needing the base class's `getMoving`/`getBusy` (`char(5)`/`char(7)`)
//! fallback. Only the `char(4)` path is ported; `getMoving`/`getBusy` are not
//! (see "Not modeled" below).
//!
//! ## Not modeled (documented deviations from the full C module)
//!
//! - **Hexapod**: `PIHexapodController`, `PIGCS2_HexapodController`,
//!   `PICoordinateSystem` — coordinate systems, pivot points, tool/work
//!   offsets.
//! - **Piezo**: `PIGCSPiezoController`, `PIGCS2PiezoCL` — closed-loop piezo
//!   gain/notch-filter parameters and the ~20 aux PVs `PIasynController` wires
//!   for them (`KP`/`KI`/`KFF`/`NTCHFR*`/`RBONT`/`RBOVF`/…).
//! - **Per-model controllers**: `PIE517Controller`, `PIE727Controller`,
//!   `PIE755Controller`, `PIC702Controller`, `PIC885Controller` (each has its
//!   own init quirks/enable-after-homing behavior), and
//!   `PIGCSMotorControllerNoRefVel` (the `E-873.3QTU` variant that skips the
//!   reference-velocity `SPA 0x50` write).
//! - **`TranslatePIError`**: the 2172-line errcode→string table
//!   (`TranslatePIError.cpp`/`picontrollererrors.h`) is not ported; only the
//!   numeric `ERR?` code (`atoi`) is surfaced, in error messages.
//! - **Deferred/coordinated moves**: `PIasynController::processDeferredMoves`
//!   / `PIGCSController::moveCts(array)` (multi-axis `MOV` in one command) —
//!   controller-wide, not part of the single-axis `AsynMotor` interface.
//! - **Axis enable** (`EAX`/`EAX?`, `setEnableAxis`/`getEnableAxis`): present
//!   in the base class but only ever *called* from `PIasynAxis::poll`'s
//!   post-homing branch gated on `m_bEnableAxisAfterHoming`/
//!   `m_bSetServoAfterHoming` — flags that default `false` and are set `true`
//!   only by `PIC885Controller` (out of scope). Dead code in the generic
//!   motor path; not ported.
//! - **`SVO?` (`getServo`)**: implemented here as a protocol primitive (the
//!   task spec calls it out explicitly), but — matching upstream — the
//!   generic motor axis's `poll` never calls it; servo state comes from the
//!   `char(4)` status bitmask instead. Upstream's own `getServo` is only
//!   called from `PICoordinateSystem`/`PIGCSPiezoController` (both out of
//!   scope).
//! - **`getAxisOnt`/`getAxisOvf`**: base-class no-ops (on-target/overflow
//!   status meaningful only for the piezo closed-loop path); not ported.
//! - **`getMoving`/`getBusy`** (`char(5)`/`char(7)`): the base class's
//!   fallback status transport for controllers that don't understand
//!   `char(4)`; unreachable once `PIGCSMotorController::getStatus` overrides
//!   it, so not ported.
//! - **Max-acceleration clamp micro-optimization**: `setAccelerationCts`'s
//!   "skip if unchanged from last set" cache is not replicated (harmless
//!   extra wire traffic on repeated identical values, no behavioral
//!   difference); the max-acceleration *value* itself is still queried and
//!   clamped to, lazily cached per axis exactly as upstream does.
//! - **Velocity readback**: `getAxisVelocity` (`VEL?`) seeds a field
//!   (`m_velocity`) that upstream itself never feeds back to the motor
//!   record (not read anywhere in `PIasynAxis::poll`), so `MotorStatus::
//!   velocity` is reported as `0.0` rather than adding an unused round trip.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, leading_hex};

/// Response buffer size for a single reply line.
const READ_BUF: usize = 256;

/// Output command terminator (C sets input EOS `"\n"` and appends a bare
/// `"\n"` write after every command in `PIInterface::sendOnly`/
/// `sendAndReceive`); the driver owns this framing.
const TERMINATOR: &[u8] = b"\n";

/// `PI_CNTR_STOP` (`picontrollererrors.h`): the GCS error code a controller
/// sets after `HLT`, checked instead of 0.
const PI_CNTR_STOP: i32 = 10;

/// `PI_CNTR_UNKNOWN_COMMAND`: returned by `ERR?` when `HAR?` itself is
/// unsupported (used to distinguish "no limit switches" from a real fault).
const PI_CNTR_UNKNOWN_COMMAND: i32 = 2;

/// `PIGCSMotorController` GCS parameter IDs (`SPA`/`SPA?`), formatted as a
/// plain decimal (`%d` in the C `sprintf`) — NOT hex, despite the hex-valued
/// enum literals in the C header.
const PARA_MOT_CURR_ACCEL: i32 = 0x0B;
const PARA_MOT_CURR_DECEL: i32 = 0x0C;
const PARA_MOT_MAX_ACCEL: i32 = 0x4A;
const PARA_MOT_MAX_DECEL: i32 = 0x4B;

/// Reference-velocity `SPA` parameter ID, written as the *literal hex text*
/// `"0x50"` — the C `referenceVelCts` hardcodes `"SPA %s 0x50 %f"` directly in
/// the format string, unlike the plain-decimal `%d` used for
/// [`PARA_MOT_CURR_ACCEL`] and friends.
const REF_VEL_PARAM_HEX: &str = "0x50";

/// Status bitmask fields (`PIGCSController::getStatusFromBitMask`).
const STATUS_NEG_LIMIT: u32 = 0x0001;
const STATUS_POS_LIMIT: u32 = 0x0004;
const STATUS_SERVO: u32 = 0x1000;
const STATUS_MOVING: u32 = 0x2000;
const STATUS_HOMING: u32 = 0x4000;

fn pigcs2_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

fn framed(cmd: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
    out.extend_from_slice(cmd.as_bytes());
    out.extend_from_slice(TERMINATOR);
    out
}

/// `PIGCSController::getValue`: return the substring after the first `=`, or
/// `None` if there is no `=` at all (an empty tail, e.g. `"1="`, is `Some("")`
/// — `atof`/`atoi` on an empty string parse as `0`, matching C `atof("")`).
fn value_str(reply: &str) -> Option<&str> {
    reply.find('=').map(|pos| &reply[pos + 1..])
}

fn get_value_f64(reply: &str) -> AsynResult<f64> {
    value_str(reply)
        .map(atof)
        .ok_or_else(|| pigcs2_err(format!("no '=' in reply {reply:?}")))
}

fn get_value_bool(reply: &str) -> AsynResult<bool> {
    value_str(reply)
        .map(|v| atoi(v) != 0)
        .ok_or_else(|| pigcs2_err(format!("no '=' in reply {reply:?}")))
}

/// `PIGCSController::getStatusFromBitMask`.
fn status_from_bitmask(mask: u32) -> (bool, bool, bool, bool, bool) {
    let homing = mask & STATUS_HOMING != 0;
    let moving = mask & STATUS_MOVING != 0;
    let neg_limit = mask & STATUS_NEG_LIMIT != 0;
    let pos_limit = mask & STATUS_POS_LIMIT != 0;
    let servo = mask & STATUS_SERVO != 0;
    (homing, moving, neg_limit, pos_limit, servo)
}

/// Extract axis `axis_no`'s 4 hex chars from a `char(4)` status reply
/// (`PIGCSMotorController::getStatus`: `idx = 2 + axisNo*4`).
fn status_mask_at(reply: &str, axis_no: usize) -> Option<u32> {
    let idx = 2 + axis_no * 4;
    let mask_str = reply.get(idx..idx + 4)?;
    leading_hex(mask_str)
}

/// Shared controller endpoint: owns the octet handle, identification,
/// `VEL`-command support flag, and the discovered axis name list.
pub struct PIGCS2Controller {
    handle: SyncIOHandle,
    ident: String,
    knows_vel: bool,
    axis_names: Vec<String>,
}

impl PIGCS2Controller {
    /// Connect and initialize a GCS2 controller (C `PIasynController`
    /// constructor's probe sequence): read `*IDN?`, probe `VEL?` support, and
    /// auto-discover connected axis names via `SAI?`. Performs blocking octet
    /// I/O.
    pub fn new(handle: SyncIOHandle) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            knows_vel: false,
            axis_names: Vec::new(),
        };

        ctrl.ident = ctrl.query("*IDN?")?;

        // C only checks the transport status of the VEL? probe, not its
        // content (any real controller reply — even an error message — means
        // the round trip completed).
        ctrl.knows_vel = match ctrl.query("VEL?") {
            Ok(_) => true,
            Err(_) => {
                let _ = ctrl.get_gcs_error();
                false
            }
        };

        ctrl.axis_names = ctrl.find_connected_axes()?;
        Ok(ctrl)
    }

    /// The controller identification string (`*IDN?`).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Every axis name discovered via `SAI?`, in controller-reported order.
    pub fn axis_names(&self) -> &[String] {
        &self.axis_names
    }

    fn knows_vel(&self) -> bool {
        self.knows_vel
    }

    /// Write a framed command with no reply expected (C `sendOnly`).
    fn send_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &framed(cmd))?;
        Ok(())
    }

    /// Write a framed command and read one reply line (C `sendAndReceive`),
    /// trimmed of any stray CR/NUL.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &framed(cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_end_matches(['\r', '\0'])
            .to_string())
    }

    /// `SAI?`: read every connected axis name. PI GCS marks a non-final reply
    /// line with a trailing space (before the EOS strip); keep issuing plain
    /// reads (no further write) while the previous line ended in a space,
    /// then trim that marker off each line (C `findConnectedAxes`).
    fn find_connected_axes(&self) -> AsynResult<Vec<String>> {
        self.handle.write_octet(0, &framed("SAI?"))?;
        let mut names = Vec::new();
        loop {
            let raw = self.handle.read_octet(0, READ_BUF)?;
            let line = String::from_utf8_lossy(&raw)
                .trim_end_matches(['\r', '\0'])
                .to_string();
            let continues = line.ends_with(' ');
            let name = line.trim_end_matches(' ').to_string();
            if !name.is_empty() {
                names.push(name);
            }
            if !continues {
                break;
            }
        }
        Ok(names)
    }

    fn get_gcs_error(&self) -> AsynResult<i32> {
        let reply = self.query("ERR?")?;
        Ok(atoi(&reply))
    }

    fn check_ok(&self) -> AsynResult<()> {
        let err = self.get_gcs_error()?;
        if err == 0 {
            Ok(())
        } else {
            Err(pigcs2_err(format!("GCS error {err}")))
        }
    }

    fn check_stopped(&self) -> AsynResult<()> {
        let err = self.get_gcs_error()?;
        if err == PI_CNTR_STOP {
            Ok(())
        } else {
            Err(pigcs2_err(format!("HLT failed, GCS error {err}")))
        }
    }

    /// `MOV {axis} {position}` + `ERR? == 0`.
    fn mov(&self, axis: &str, position: f64) -> AsynResult<()> {
        self.send_only(&format!("MOV {axis} {position:.6}"))?;
        self.check_ok()
    }

    /// `VEL {axis} {|velocity|}`, gated on [`Self::knows_vel`]; no error
    /// check (C `setVelocityCts` is fire-and-forget).
    fn set_velocity(&self, axis: &str, velocity: f64) -> AsynResult<()> {
        if !self.knows_vel {
            return Ok(());
        }
        self.send_only(&format!("VEL {axis} {:.6}", velocity.abs()))
    }

    /// `SPA {axis} {param_id} {value}`; no error check (C `setGCSParameter`).
    fn set_gcs_parameter(&self, axis: &str, param_id: i32, value: f64) -> AsynResult<()> {
        self.send_only(&format!("SPA {axis} {param_id} {value:.6}"))
    }

    /// `SPA? {axis} {param_id}`.
    fn get_gcs_parameter(&self, axis: &str, param_id: i32) -> AsynResult<f64> {
        let reply = self.query(&format!("SPA? {axis} {param_id}"))?;
        get_value_f64(&reply)
    }

    /// `HLT {axis}` + `ERR? == PI_CNTR_STOP`.
    fn halt(&self, axis: &str) -> AsynResult<()> {
        self.send_only(&format!("HLT {axis}"))?;
        self.check_stopped()
    }

    /// `RON {axis} 0` / `POS {axis} {position}` / `RON {axis} 1`, one
    /// `ERR? == 0` after all three (C `setAxisPosition`).
    fn set_axis_position(&self, axis: &str, position: f64) -> AsynResult<()> {
        self.send_only(&format!("RON {axis} 0"))?;
        self.send_only(&format!("POS {axis} {position:.6}"))?;
        self.send_only(&format!("RON {axis} 1"))?;
        self.check_ok()
    }

    /// `POS? {axis}`.
    fn get_axis_position(&self, axis: &str) -> AsynResult<f64> {
        get_value_f64(&self.query(&format!("POS? {axis}"))?)
    }

    /// `TMN? {axis}` / `TMX? {axis}` -> `(negLimit, posLimit)`.
    fn get_travel_limits(&self, axis: &str) -> AsynResult<(f64, f64)> {
        let neg = get_value_f64(&self.query(&format!("TMN? {axis}"))?)?;
        let pos = get_value_f64(&self.query(&format!("TMX? {axis}"))?)?;
        Ok((neg, pos))
    }

    /// `LIM? {axis}`, falling back to `HAR? {axis}` when `LIM?` is false; a
    /// `HAR?` timeout with `ERR? == PI_CNTR_UNKNOWN_COMMAND` means "no limit
    /// switches" rather than a real fault (C `hasLimitSwitches`).
    fn has_limit_switches(&self, axis: &str) -> AsynResult<bool> {
        if get_value_bool(&self.query(&format!("LIM? {axis}"))?)? {
            return Ok(true);
        }
        match self.query(&format!("HAR? {axis}")) {
            Ok(reply) => get_value_bool(&reply),
            Err(AsynError::Status {
                status: AsynStatus::Timeout,
                ..
            }) => {
                let err = self.get_gcs_error()?;
                if err == PI_CNTR_UNKNOWN_COMMAND {
                    Ok(false)
                } else {
                    Err(pigcs2_err(format!("HAR? failed, GCS error {err}")))
                }
            }
            Err(e) => Err(e),
        }
    }

    /// `TRS? {axis}`.
    fn has_reference_sensor(&self, axis: &str) -> AsynResult<bool> {
        get_value_bool(&self.query(&format!("TRS? {axis}"))?)
    }

    /// `FRF? {axis}`.
    fn get_referenced_state(&self, axis: &str) -> AsynResult<bool> {
        get_value_bool(&self.query(&format!("FRF? {axis}"))?)
    }

    /// `SVO? {axis}` (see "Not modeled": unused by the generic axis's poll,
    /// kept as a protocol primitive matching the task spec).
    #[allow(dead_code)]
    fn get_servo(&self, axis: &str) -> AsynResult<bool> {
        get_value_bool(&self.query(&format!("SVO? {axis}"))?)
    }

    /// `SVO {axis} {0|1}` + `ERR? == 0`.
    fn set_servo(&self, axis: &str, enable: bool) -> AsynResult<()> {
        self.send_only(&format!("SVO {axis} {}", enable as i32))?;
        self.check_ok()
    }

    /// `SPA {axis} 0x50 {|velocity|}` + `ERR? == 0` (C `referenceVelCts`'s
    /// own inline error check, distinct from the unchecked
    /// [`Self::set_gcs_parameter`]).
    fn set_reference_velocity(&self, axis: &str, velocity: f64) -> AsynResult<()> {
        self.send_only(&format!(
            "SPA {axis} {REF_VEL_PARAM_HEX} {:.6}",
            velocity.abs()
        ))?;
        self.check_ok()
    }

    /// `FRF`/`FPL`/`FNL {axis}` + `ERR? == 0` (C `referenceAxis`).
    fn reference_axis(
        &self,
        axis: &str,
        forward: bool,
        has_reference: bool,
        has_limit_switches: bool,
    ) -> AsynResult<()> {
        if has_reference {
            self.send_only(&format!("FRF {axis}"))?;
        } else if has_limit_switches {
            let cmd = if forward { "FPL" } else { "FNL" };
            self.send_only(&format!("{cmd} {axis}"))?;
        } else {
            return Err(pigcs2_err(format!(
                "axis {axis} has no reference sensor or limit switch to home against"
            )));
        }
        self.check_ok()
    }

    /// `CST? {axis}` (C `initAxis`'s stage-configuration probe, logged only —
    /// C keeps it purely to surface mis/non-configured controllers in the
    /// log).
    fn stage_id(&self, axis: &str) -> AsynResult<String> {
        self.query(&format!("CST? {axis}"))
    }

    /// `char(4)` -> `(homing, moving, negLimit, posLimit, servoControl)` for
    /// `axis_no` (C `PIGCSMotorController::getStatus`).
    fn get_status(&self, axis_no: usize) -> AsynResult<(bool, bool, bool, bool, bool)> {
        self.handle.write_octet(0, &[0x04])?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let reply = String::from_utf8_lossy(&raw)
            .trim_end_matches(['\r', '\0'])
            .to_string();
        let mask = status_mask_at(&reply, axis_no).ok_or_else(|| {
            pigcs2_err(format!(
                "status reply too short/non-hex for axis {axis_no}: {reply:?}"
            ))
        })?;
        Ok(status_from_bitmask(mask))
    }
}

/// One GCS2 axis sharing a controller. Implements [`AsynMotor`].
pub struct PIGCS2Axis {
    controller: Arc<Mutex<PIGCS2Controller>>,
    /// GCS wire name (e.g. `"1"`), from `SAI?`.
    axis_name: String,
    /// 0-based creation-order index, used only for the `char(4)` status
    /// bitmask offset (`2 + axis_no*4`).
    axis_no: usize,
    has_limit_switches: bool,
    has_reference: bool,
    homed: bool,
    is_homing: bool,
    servo_on: bool,
    problem: bool,
    last_position: f64,
    last_direction: bool,
    low_limit: f64,
    high_limit: f64,
    /// Lazily-queried `min(MOT_MAX_ACCEL, MOT_MAX_DECEL)`, cached like C's
    /// `pAxis->m_maxAcceleration` (`-1.0` sentinel there, `None` here).
    max_acceleration: Option<f64>,
}

impl PIGCS2Axis {
    /// Construct axis `axis_no` named `axis_name` (C `PIasynAxis::Init` +
    /// `PIGCSMotorController::initAxis`): probe limit-switch/reference
    /// capability, read the stage id (logged), enable the servo, and seed the
    /// cached position/limits/homed state. Performs blocking octet I/O.
    pub fn new(
        controller: Arc<Mutex<PIGCS2Controller>>,
        axis_name: String,
        axis_no: usize,
    ) -> AsynResult<Self> {
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        let has_limit_switches = ctrl.has_limit_switches(&axis_name)?;
        let has_reference = ctrl.has_reference_sensor(&axis_name)?;
        let stage_id = ctrl.stage_id(&axis_name)?;
        println!("PIGCS2Axis: axis {axis_name} stage configuration: {stage_id}");
        ctrl.set_servo(&axis_name, true)?;
        let last_position = ctrl.get_axis_position(&axis_name)?;
        let (low_limit, high_limit) = ctrl.get_travel_limits(&axis_name)?;
        let homed = ctrl.get_referenced_state(&axis_name)?;
        drop(ctrl);

        Ok(Self {
            controller,
            axis_name,
            axis_no,
            has_limit_switches,
            has_reference,
            homed,
            is_homing: false,
            servo_on: true,
            problem: false,
            last_position,
            last_direction: true,
            low_limit,
            high_limit,
            max_acceleration: None,
        })
    }

    fn lock(&self) -> MutexGuard<'_, PIGCS2Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `setAccelerationCts`: lazily cache `min(MAX_ACCEL, MAX_DECEL)`, clamp
    /// the requested acceleration to it, and write `CURR_ACCEL`/`CURR_DECEL`.
    /// Gated on `knows_vel`, matching both `getMaxAcceleration` and
    /// `setAcceleration`'s own early return.
    fn apply_acceleration(&mut self, acceleration: f64) -> AsynResult<()> {
        if !self.lock().knows_vel() {
            return Ok(());
        }
        if self.max_acceleration.is_none() {
            let max_acc = self
                .lock()
                .get_gcs_parameter(&self.axis_name, PARA_MOT_MAX_ACCEL)?;
            let max_dec = self
                .lock()
                .get_gcs_parameter(&self.axis_name, PARA_MOT_MAX_DECEL)?;
            self.max_acceleration = Some(max_acc.min(max_dec));
        }
        let accel = acceleration.abs().min(self.max_acceleration.unwrap());
        let ctrl = self.lock();
        ctrl.set_gcs_parameter(&self.axis_name, PARA_MOT_CURR_ACCEL, accel)?;
        ctrl.set_gcs_parameter(&self.axis_name, PARA_MOT_CURR_DECEL, accel)?;
        Ok(())
    }
}

impl AsynMotor for PIGCS2Axis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C `PIasynAxis::move`: VEL, then SPA accel/decel, then MOV — each an
        // independent command, no atomicity required between them.
        if velocity != 0.0 {
            self.lock().set_velocity(&self.axis_name, velocity)?;
        }
        if acceleration != 0.0 {
            self.apply_acceleration(acceleration)?;
        }
        self.lock().mov(&self.axis_name, position)?;
        self.last_direction = position > self.last_position;
        Ok(())
    }

    // move_relative: C's own `move(position, relative, ...)` never branches
    // on `relative` (a `//TODO` stub) — it always issues MOV with `position`
    // taken as an absolute target regardless of the flag. The trait's default
    // (poll for the current position, add the distance, delegate to
    // move_absolute) reaches that same MOV wire command, so no override.

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C `moveVelocity`: VEL, then MOV toward whichever travel limit the
        // sign of velocity selects. No acceleration write, and — unlike
        // move_absolute — `m_lastDirection` is not updated here in C either.
        let ctrl = self.lock();
        ctrl.set_velocity(&self.axis_name, velocity)?;
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        };
        ctrl.mov(&self.axis_name, target)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C sets m_isHoming=1 optimistically before attempting the reference
        // command; on failure it is left set until the next poll's real
        // status bitmask corrects it, matching upstream.
        self.is_homing = true;
        let ctrl = self.lock();
        ctrl.set_servo(&self.axis_name, true)?;
        if velocity != 0.0 {
            ctrl.set_reference_velocity(&self.axis_name, velocity)?;
        }
        ctrl.reference_axis(
            &self.axis_name,
            forward,
            self.has_reference,
            self.has_limit_switches,
        )?;
        drop(ctrl);
        self.servo_on = true;
        self.problem = false;
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.lock().halt(&self.axis_name)
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.lock().set_axis_position(&self.axis_name, position)
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.lock().set_servo(&self.axis_name, enable)?;
        self.servo_on = enable;
        if enable {
            self.problem = false;
        }
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let old_homing = self.is_homing;
        let old_servo_on = self.servo_on;

        let ctrl = self.lock();
        let (homing, raw_moving, neg_limit, pos_limit, servo_control) =
            ctrl.get_status(self.axis_no)?;
        // C only re-queries FRF? (the homed flag) right when homing just
        // finished (oldHoming true, new homing false) — not every cycle.
        let refreshed_homed = if old_homing && !homing {
            Some(ctrl.get_referenced_state(&self.axis_name)?)
        } else {
            None
        };
        let position = ctrl.get_axis_position(&self.axis_name)?;
        let (low_limit, high_limit) = ctrl.get_travel_limits(&self.axis_name)?;
        drop(ctrl);

        self.is_homing = homing;
        if let Some(homed) = refreshed_homed {
            self.homed = homed;
        }
        // Servo dropping out on its own (not via set_closed_loop) is a
        // problem (C poll: "servo changed without user interaction!").
        if old_servo_on && !servo_control {
            self.problem = true;
        }
        self.servo_on = servo_control;
        self.last_position = position;
        self.low_limit = low_limit;
        self.high_limit = high_limit;

        // C: done = (moving==0 && isHoming==0); the record's reported
        // "moving" bit is `!done`, folding homing into it too — not the raw
        // status-bitmask moving bit directly.
        let done = !(raw_moving || homing);

        Ok(MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            done,
            moving: !done,
            high_limit: pos_limit,
            low_limit: neg_limit,
            direction: self.last_direction,
            powered: servo_control,
            problem: self.problem,
            homed: self.homed,
            has_encoder: true,
            gain_support: true,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_str_extracts_after_equals() {
        assert_eq!(value_str("1=0.5"), Some("0.5"));
        assert_eq!(value_str("1="), Some(""));
        assert_eq!(value_str("no-equals-here"), None);
    }

    #[test]
    fn get_value_f64_parses_leading_numeric_prefix_after_equals() {
        assert_eq!(get_value_f64("1=0.500000").unwrap(), 0.5);
        assert_eq!(get_value_f64("1=-3.25 ").unwrap(), -3.25);
        assert!(get_value_f64("no-equals").is_err());
    }

    #[test]
    fn get_value_bool_treats_nonzero_as_true() {
        assert!(get_value_bool("1=1").unwrap());
        assert!(!get_value_bool("1=0").unwrap());
    }

    #[test]
    fn status_bitmask_decodes_all_fields() {
        // homing=0x4000, moving=0x2000, servo=0x1000, posLimit=0x0004, negLimit=0x0001
        assert_eq!(status_from_bitmask(0x7005), (true, true, true, true, true));
        assert_eq!(
            status_from_bitmask(0x0000),
            (false, false, false, false, false)
        );
        assert_eq!(
            status_from_bitmask(0x2000),
            (false, true, false, false, false)
        );
        assert_eq!(
            status_from_bitmask(0x0001),
            (false, false, true, false, false)
        );
    }

    #[test]
    fn status_mask_at_extracts_per_axis_slice() {
        // "01" header + one 4-hex-char field per axis, matching
        // `idx = 2 + axis_no*4` (C `PIGCSMotorController::getStatus`).
        let reply = "012000";
        assert_eq!(status_mask_at(reply, 0), Some(0x2000));

        let reply2 = "0120004001";
        assert_eq!(status_mask_at(reply2, 0), Some(0x2000));
        assert_eq!(status_mask_at(reply2, 1), Some(0x4001));

        assert_eq!(status_mask_at("short", 5), None);
        assert_eq!(status_mask_at("01zzzz", 0), None);
    }

    #[test]
    fn framed_appends_newline_terminator() {
        assert_eq!(framed("MOV 1 10.000000"), b"MOV 1 10.000000\n".to_vec());
    }

    #[test]
    fn error_codes_match_c_constants() {
        assert_eq!(PI_CNTR_STOP, 10);
        assert_eq!(PI_CNTR_UNKNOWN_COMMAND, 2);
    }

    #[test]
    fn done_folds_in_homing_state() {
        // done = !(raw_moving || homing); reported "moving" = !done.
        let done = |raw_moving: bool, homing: bool| !(raw_moving || homing);
        assert!(done(false, false));
        assert!(!done(true, false));
        assert!(!done(false, true), "homing alone must keep done false");
        assert!(!done(true, true));
    }
}
