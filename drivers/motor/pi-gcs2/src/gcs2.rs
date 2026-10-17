//! PI GCS2 generic stage-controller protocol (ASCII over asyn octet).
//!
//! Ported from `motorPIGCS2/pigcs2App/src/PIGCSController.cpp` (protocol
//! core, base class) + `PIGCSMotorController.cpp` (the C-863/C-867/C-663/
//! C-884/E-861/E-871/E-873 motor specialization) — the two classes the task
//! scopes as "generic GCS2 stage-controller path". Commands are plain ASCII
//! text terminated by `\n`; the driver owns output framing and
//! `PI_GCS2_CreateController` sets input EOS `"\n"` and an empty output EOS, as
//! the C `PIasynController` constructor does. Axes are addressed by the GCS *name
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
//! loop) — `PIGCS2Controller::find_connected_axes` replicates that instead
//! of assuming one line per reply.
//!
//! ## Units
//!
//! `POS`/`MOV`/`TMN`/`TMX` are physical units on the controller. As C, the
//! record's steps are converted by the axis counts-per-unit integer fraction
//! (`PI_PARA_MOT_CPU_Z`/`_N`, `getResolution`): commands send
//! `steps · N / Z`, the poll reports `position · Z / N`. C rounds that
//! readback with `int(x + 0.5)`, which reads every negative position one count
//! high; this port reports it unrounded.
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
//! [`PIGCS2Controller::read_idn`] and [`PIGCS2Controller::init`] mirror that
//! probe sequence, and `ioc::pigcs2_config_command` mirrors the axis-count
//! check (printed, no axes, if fewer were found than requested) and the
//! first-N-in-order axis creation.
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
//! - **Per-model controllers**: `PIE517Controller` and `PIE727Controller`
//!   (piezo-based, each with its own init and status quirks); those
//!   controllers run on [`Model::Motor`].
//! - **`TranslatePIError`**: the 2172-line errcode→string table
//!   (`TranslatePIError.cpp`/`picontrollererrors.h`) is not ported; only the
//!   numeric `ERR?` code (`atoi`) is surfaced, in error messages.
//! - **Deferred/coordinated moves**: `PIasynController::processDeferredMoves`
//!   / `PIGCSController::moveCts(array)` (multi-axis `MOV` in one command) —
//!   controller-wide, not part of the single-axis `AsynMotor` interface.
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
//! - **Velocity readback**: the init `getAxisVelocity` (`VEL?`) is sent, but
//!   the `m_velocity` it seeds is never read back (not in `PIasynAxis::poll`),
//!   so `MotorStatus::velocity` stays `0.0`.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus, OctetStep};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use motor_common::util::{atof, atoi, leading_hex};
use motor_common::{ControllerParams, ControllerReport};

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
const PARA_MOT_CPU_Z: i32 = 0x0E;
const PARA_MOT_HAT_REF: i32 = 0x14;
/// `PIC702Controller::PI_PARA_C702_REFERENCED`.
const PARA_C702_REFERENCED: i32 = 0x1C;
const PARA_MOT_CPU_N: i32 = 0x0F;

/// C `PIasynAxis` CPU defaults, kept when the `SPA?` reads fail.
const DEFAULT_CPU_Z: i32 = 1000;
const DEFAULT_CPU_N: i32 = 1;

/// C `PIGCSController::getResolution`: the fixed CPU of a controller whose
/// class reads none.
const BASE_CPU_Z: i32 = 10000;

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

/// Counts per controller unit from C's integer `m_CPUnumerator`/
/// `m_CPUdenominator`; a zero term is 1, as `getAxisPositionCts`.
fn counts_per_unit(numerator: i32, denominator: i32) -> f64 {
    if numerator == 0 || denominator == 0 {
        1.0
    } else {
        f64::from(numerator) / f64::from(denominator)
    }
}

/// C `PIGCSController::getBusy`: the `#7` reply's first byte is 0xB0.
fn busy_from_reply(raw: &[u8]) -> bool {
    raw.first() == Some(&0xB0)
}

/// C `PIGCSController::MAX_NR_AXES`.
const MAX_NR_AXES: usize = 64;

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

/// The C controller class `PIGCSController::CreateGCSController`
/// (PIGCSController.cpp:42-112) instantiates for an upper-cased `*IDN?`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Model {
    /// `PIGCSMotorController`. The controllers whose own C class is not
    /// ported yet run on it too.
    #[default]
    Motor,
    /// `PIGCSMotorControllerNoRefVel` (E-873.3QTU).
    MotorNoRefVel,
    /// `PIC885Controller`.
    C885,
    /// `PIE755Controller`.
    E755,
    /// `PIC702Controller`.
    C702,
}

impl Model {
    /// `PIGCSMotorController::getResolution`'s `SPA?` CPU reads; the others
    /// run the base's fixed one.
    fn reads_cpu(self) -> bool {
        !matches!(self, Self::C885 | Self::E755)
    }

    /// `PIGCSMotorController::setAccelerationCts`; the others' is a no-op.
    fn sets_acceleration(self) -> bool {
        !matches!(self, Self::C885 | Self::E755)
    }

    /// `getMaxAcceleration`'s parameters, and whether it reads them only
    /// on a controller that knows `VEL` (`PIGCSMotorController`'s) or
    /// always (`PIC702Controller`'s current, not maximum, ones).
    fn max_acceleration_params(self) -> ((i32, i32), bool) {
        match self {
            Self::C702 => ((PARA_MOT_CURR_ACCEL, PARA_MOT_CURR_DECEL), false),
            _ => ((PARA_MOT_MAX_ACCEL, PARA_MOT_MAX_DECEL), true),
        }
    }

    /// C `IsGCS2`: false only on a C-702, whose poll reads no position or
    /// servo while homing (PIasynAxis.cpp:159).
    fn is_gcs2(self) -> bool {
        self != Self::C702
    }

    /// `None` where C creates no controller.
    pub fn from_idn(idn: &str) -> Option<Self> {
        let any = |models: &[&str]| models.iter().any(|m| idn.contains(m));
        if idn.contains("E-873.3QTU") {
            Some(Self::MotorNoRefVel)
        } else if any(&[
            "C-663", "C-863", "C-867", "C-884", "E-861", "E-871", "E-873", "E-517", "E-518",
        ]) {
            Some(Self::Motor)
        } else if idn.contains("C-885") {
            Some(Self::C885)
        } else if any(&["E-753", "E-754", "E-709", "E-712", "E-725", "E-727"]) {
            Some(Self::Motor)
        } else if idn.contains("E-755") {
            Some(Self::E755)
        } else if idn.contains("C-702") {
            Some(Self::C702)
        } else if any(&["HEXAPOD", "F-HEX", "F-206", "M-8", "C-887"]) {
            Some(Self::Motor)
        } else {
            None
        }
    }
}

/// Shared controller endpoint: owns the octet handle, identification,
/// `VEL`-command support flag, and the discovered axis name list.
pub struct PIGCS2Controller {
    handle: SyncIOHandle,
    /// The octet port C's `m_pAsynInterface` (the default log sink) prints on.
    serial_port: String,
    ident: String,
    model: Model,
    knows_vel: bool,
    axis_names: Vec<String>,
    /// `PIC702Controller::m_timeREFstarted`: one for the controller.
    ref_started: Option<Instant>,
}

impl PIGCS2Controller {
    /// C `new PIInterface(pAsynCom)`: no I/O until [`Self::read_idn`] and
    /// [`Self::init`].
    pub fn new(handle: SyncIOHandle, serial_port: &str) -> Self {
        Self {
            handle,
            serial_port: serial_port.to_owned(),
            ident: String::new(),
            model: Model::default(),
            knows_vel: false,
            axis_names: Vec::new(),
            ref_started: None,
        }
    }

    /// The class C's `CreateGCSController` picked from the `*IDN?`.
    pub fn set_model(&mut self, model: Model) {
        self.model = model;
    }

    /// The constructor's `*IDN?` into an emptied buffer: empty when the
    /// exchange fails, whose status C ignores.
    pub fn read_idn(&mut self) -> &str {
        self.ident = self.query("*IDN?").unwrap_or_default();
        &self.ident
    }

    /// C `PIGCSController::init`: probe `VEL?` and discover the axes with
    /// `SAI?`. The constructor ignores its status, so a failed `SAI?` leaves
    /// no axes.
    pub fn init(&mut self) {
        // C only checks the transport status of the VEL? probe, not its
        // content (any real controller reply — even an error message — means
        // the round trip completed).
        self.knows_vel = match self.query("VEL?") {
            Ok(_) => true,
            Err(_) => {
                let _ = self.get_gcs_error();
                false
            }
        };
        self.axis_names = self.find_connected_axes().unwrap_or_default();
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

    /// Write a command with no reply expected (C `sendOnly`): the command,
    /// then its `\n` as a second write whose status alone decides.
    fn send_only(&self, cmd: &str) -> AsynResult<()> {
        let _ = self.handle.write_octet(0, cmd.as_bytes());
        let e = match self.handle.write_octet(0, TERMINATOR) {
            Ok(_) => return Ok(()),
            Err(e) => e,
        };
        self.print(format_args!(
            "PIGCSController:sendOnly: error sending command {cmd}, sent={}, status={}\n",
            e.partial_write().unwrap_or(0),
            e.status() as i32
        ));
        Err(e)
    }

    /// Write a command and read one reply line (C `sendAndReceive`),
    /// trimmed of any stray CR/NUL.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.send_and_receive(cmd)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_end_matches(['\r', '\0'])
            .to_string())
    }

    /// C `sendAndReceive(const char*)`: the command, then a writeRead of its
    /// `\n`. Either write falling short is asynError.
    fn send_and_receive(&self, cmd: &str) -> AsynResult<Vec<u8>> {
        let short_write = |status: i32, message: &str| {
            self.print(format_args!(
                "PIGCSController:sendAndReceive error calling write, output={cmd} status={status}, error={message}\n"
            ));
            pigcs2_err(message)
        };
        match self.handle.write_octet(0, cmd.as_bytes()) {
            Ok(n) if n == cmd.len() => {}
            Ok(_) => return Err(short_write(0, "")),
            Err(e) => return Err(short_write(e.status() as i32, &e.message())),
        }
        self.handle
            .write_read(0, TERMINATOR, READ_BUF)
            .map_err(|e| match e.octet_step() {
                Some(OctetStep::Read) => {
                    self.print(format_args!(
                        "PIGCSController:sendAndReceive error calling writeRead, output={cmd} status={}, error={}\n",
                        e.status() as i32,
                        e.message()
                    ));
                    e
                }
                _ => short_write(e.status() as i32, &e.message()),
            })
    }

    /// `asynPrint(pAsynCom, ASYN_TRACE_ERROR, ...)`.
    pub fn print(&self, args: std::fmt::Arguments<'_>) {
        PortManager::global().trace_manager().print(
            Some(&self.serial_port),
            Some(0),
            0,
            TraceMask::ERROR,
            args,
        );
    }

    /// `SAI?`: read every connected axis name. PI GCS marks a non-final reply
    /// line with a trailing space (before the EOS strip); keep issuing plain
    /// reads (no further write) while the previous line ended in a space,
    /// then trim that marker off each line (C `findConnectedAxes`).
    fn find_connected_axes(&self) -> AsynResult<Vec<String>> {
        if self.model == Model::C702 {
            // PIC702Controller::findConnectedAxes: one word of one-char
            // axes, last first.
            let word = self.query("SAI?")?;
            if word.len() >= MAX_NR_AXES {
                return Err(pigcs2_err(format!("{} axes in SAI? reply", word.len())));
            }
            return Ok(word.chars().rev().map(String::from).collect());
        }
        let mut raw = self.send_and_receive("SAI?")?;
        let mut names = Vec::new();
        loop {
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
            raw = self.handle.read_octet(0, READ_BUF)?;
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
        self.send_only(&format!("MOV {axis} {}", CFloat(position)))?;
        self.check_ok()
    }

    /// `VEL {axis} {|velocity|}`, gated on [`Self::knows_vel`]; no error
    /// check (C `setVelocityCts` is fire-and-forget).
    fn set_velocity(&self, axis: &str, velocity: f64) -> AsynResult<()> {
        if !self.knows_vel {
            return Ok(());
        }
        self.send_only(&format!("VEL {axis} {}", CFloat(velocity.abs())))
    }

    /// `SPA {axis} {param_id} {value}`; no error check (C `setGCSParameter`).
    fn set_gcs_parameter(&self, axis: &str, param_id: i32, value: f64) -> AsynResult<()> {
        self.send_only(&format!("SPA {axis} {param_id} {:.12}", CFloat(value).g()))
    }

    /// `SPA? {axis} {param_id}`.
    fn get_gcs_parameter(&self, axis: &str, param_id: i32) -> AsynResult<f64> {
        let reply = self.query(&format!("SPA? {axis} {param_id}"))?;
        get_value_f64(&reply)
    }

    /// `STP` + `ERR? == PI_CNTR_STOP`: stops every axis (C
    /// `PIC885Controller::haltAxis`).
    fn stop_all(&self) -> AsynResult<()> {
        self.send_only("STP")?;
        self.check_stopped()
    }

    /// `EAX? {axis}`, then `ERR? == 0` before the value counts (C
    /// `getEnableAxis`).
    fn get_enable_axis(&self, axis: &str) -> AsynResult<i32> {
        let reply = self.query(&format!("EAX? {axis}"))?;
        self.check_ok()?;
        value_str(&reply)
            .map(atoi)
            .ok_or_else(|| pigcs2_err(format!("no '=' in reply {reply:?}")))
    }

    /// `EAX {axis} {state}` + `ERR? == 0` (C `PIGCSController::setEnableAxis`).
    fn set_enable_axis(&self, axis: &str, state: i32) -> AsynResult<()> {
        self.send_only(&format!("EAX {axis} {state}"))?;
        self.check_ok()
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
        self.send_only(&format!("POS {axis} {}", CFloat(position)))?;
        self.send_only(&format!("RON {axis} 1"))?;
        self.check_ok()
    }

    /// C `getAxisVelocity`: `VEL? {axis}` on a controller that knows it.
    fn get_axis_velocity(&self, axis: &str) -> AsynResult<f64> {
        if !self.knows_vel {
            return Ok(0.0);
        }
        get_value_f64(&self.query(&format!("VEL? {axis}"))?)
    }

    /// `POS? {axis}`.
    fn get_axis_position(&self, axis: &str) -> AsynResult<f64> {
        get_value_f64(&self.query(&format!("POS? {axis}"))?)
    }

    /// `TMN? {axis}` / `TMX? {axis}` -> `(negLimit, posLimit)`.
    /// Each limit is stored as soon as it is read, so a failed `TMX?` keeps
    /// the `TMN?` value, as C's reference out-parameters do.
    fn get_travel_limits(&self, axis: &str, neg: &mut f64, pos: &mut f64) -> AsynResult<()> {
        *neg = get_value_f64(&self.query(&format!("TMN? {axis}"))?)?;
        *pos = get_value_f64(&self.query(&format!("TMX? {axis}"))?)?;
        Ok(())
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
            // `status()` reads *through* a `PartialRead` carrier, so a timeout
            // that returned partial bytes (EOS interpose installed) is still
            // recognized as "no limit switches" rather than a hard fault.
            Err(e) if e.status() == AsynStatus::Timeout => {
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

    /// `TRS? {axis}`; a C-702's `HAT_REF` parameter.
    fn has_reference_sensor(&self, axis: &str) -> AsynResult<bool> {
        if self.model == Model::C702 {
            return Ok(self.get_gcs_parameter(axis, PARA_MOT_HAT_REF)? > 0.1);
        }
        get_value_bool(&self.query(&format!("TRS? {axis}"))?)
    }

    /// `FRF? {axis}`; a C-702's `REFERENCED` parameter.
    fn get_referenced_state(&self, axis: &str) -> AsynResult<bool> {
        if self.model == Model::C702 {
            return Ok(self.get_gcs_parameter(axis, PARA_C702_REFERENCED)? > 0.1);
        }
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
            "SPA {axis} {REF_VEL_PARAM_HEX} {}",
            CFloat(velocity.abs())
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
    /// C `sendAndReceive(char)`: a short write is asynError.
    fn send_and_receive_char(&self, c: u8) -> AsynResult<Vec<u8>> {
        self.handle.write_read(0, &[c], READ_BUF).map_err(|e| {
            let e = match e.octet_step() {
                Some(OctetStep::Read) => e,
                _ => pigcs2_err(e.message()),
            };
            self.print(format_args!(
                "PIInterface::sendAndReceive error calling writeRead, output={c} status={}, error={}\n",
                e.status() as i32,
                e.message()
            ));
            e
        })
    }

    /// [`Self::send_and_receive_char`]'s reply line.
    fn query_char(&self, c: u8) -> AsynResult<String> {
        let raw = self.send_and_receive_char(c)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_end_matches(['\r', '\0'])
            .to_string())
    }

    /// C `getBusy`.
    fn get_busy(&self) -> AsynResult<bool> {
        Ok(busy_from_reply(&self.send_and_receive_char(7)?))
    }

    /// C `getMoving`: `#5`'s hex word masked by the axis bit, C's
    /// `m_movingStateMask` as `PIGCSController::initAxis` sets it.
    fn get_moving(&self, axis_no: usize) -> AsynResult<bool> {
        let state = leading_hex(self.query_char(5)?.trim_start()).unwrap_or(0);
        Ok(state & (1 << axis_no) != 0)
    }

    fn stage_id(&self, axis: &str) -> AsynResult<String> {
        self.query(&format!("CST? {axis}"))
    }

    /// `char(4)` -> `(homing, moving, negLimit, posLimit, servoControl)` for
    /// `axis_no` (C `PIGCSMotorController::getStatus`).
    fn get_status(&self, axis_no: usize) -> AsynResult<(bool, bool, bool, bool, bool)> {
        let reply = self.query_char(4)?;
        let mask = status_mask_at(&reply, axis_no).ok_or_else(|| {
            pigcs2_err(format!(
                "status reply too short/non-hex for axis {axis_no}: {reply:?}"
            ))
        })?;
        Ok(status_from_bitmask(mask))
    }
}

/// C `PIasynController::report` (PIasynController.cpp:196-221), over the
/// axes `PI_GCS2_CreateController` created.
#[derive(Clone, Default)]
pub struct PIGCS2Report {
    axes: Arc<Mutex<Vec<Arc<Mutex<PIGCS2Axis>>>>>,
}

impl PIGCS2Report {
    /// An axis the report lists, in creation order.
    pub fn add(&self, axis: Arc<Mutex<PIGCS2Axis>>) {
        self.axes
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(axis);
    }
}

impl ControllerParams for PIGCS2Report {
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "PIGCS2 motor driver {}, numAxes={}",
            ctx.port_name(),
            ctx.num_axes()
        );
        // C reads getPIAxis(axis) for every numAxes_, a NULL dereference
        // when the constructor returned before creating the axes; that C
        // defect is not copied: only created axes print.
        for axis in self.axes.lock().unwrap_or_else(|e| e.into_inner()).iter() {
            let axis = axis.lock().unwrap_or_else(|e| e.into_inner());
            let _ = writeln!(out, "  axis {}", axis.axis_no);
            if level > 0 && axis.is_homing {
                let _ = writeln!(out, "    Currently homing axis");
            }
        }
        ctx.report(out, level);
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
    /// The `#4` moving and limit bits the last poll read.
    moving: bool,
    neg_limit: bool,
    pos_limit: bool,
    /// Record steps per controller unit (C `m_CPUnumerator/m_CPUdenominator`).
    cpu: f64,
    /// Last polled position, in record steps.
    last_position: f64,
    last_direction: bool,
    low_limit: f64,
    high_limit: f64,
    /// Lazily-queried `min(MOT_MAX_ACCEL, MOT_MAX_DECEL)`, cached like C's
    /// `pAxis->m_maxAcceleration` (`-1.0` sentinel there, `None` here).
    max_acceleration: Option<f64>,
    /// C `m_acceleration`: the clamped acceleration last set.
    acceleration: f64,
}

impl PIGCS2Axis {
    /// Construct axis `axis_no` named `axis_name` on controller port
    /// `port_name` (C `PIasynAxis` + `PIasynAxis::Init`). C ignores every
    /// status here: a failed `initAxis` step skips the rest of it, each later
    /// read keeps its default, and the axis is always created.
    pub fn new(
        controller: Arc<Mutex<PIGCS2Controller>>,
        axis_name: String,
        axis_no: usize,
        port_name: &str,
    ) -> Self {
        println!("PIasynAxis::PIasynAxis() {axis_no}: {axis_name}");
        let mut axis = Self {
            controller,
            axis_name,
            axis_no,
            has_limit_switches: false,
            has_reference: false,
            homed: false,
            is_homing: false,
            servo_on: false,
            problem: false,
            moving: false,
            neg_limit: false,
            pos_limit: false,
            cpu: counts_per_unit(DEFAULT_CPU_Z, DEFAULT_CPU_N),
            last_position: 0.0,
            last_direction: false,
            low_limit: 0.0,
            high_limit: 0.0,
            max_acceleration: None,
            acceleration: 0.0,
        };
        let _ = axis.init_axis(port_name);
        let ctrl = axis.lock();
        // C `PIGCSMotorController::getResolution` stops at the first failed
        // read, keeps both defaults unless both succeed, and truncates the
        // replies into ints.
        let cpu = if !ctrl.model.reads_cpu() {
            counts_per_unit(BASE_CPU_Z, 1)
        } else {
            ctrl.get_gcs_parameter(&axis.axis_name, PARA_MOT_CPU_Z)
                .and_then(|num| {
                    let den = ctrl.get_gcs_parameter(&axis.axis_name, PARA_MOT_CPU_N)?;
                    Ok(counts_per_unit(num as i32, den as i32))
                })
                .unwrap_or(axis.cpu)
        };
        let _ = ctrl.get_axis_velocity(&axis.axis_name);
        let position = ctrl.get_axis_position(&axis.axis_name);
        let (mut low_limit, mut high_limit) = (axis.low_limit, axis.high_limit);
        let _ = ctrl.get_travel_limits(&axis.axis_name, &mut low_limit, &mut high_limit);
        let homed = ctrl.get_referenced_state(&axis.axis_name);
        drop(ctrl);
        axis.cpu = cpu;
        if let Ok(position) = position {
            axis.last_position = position * cpu;
        }
        (axis.low_limit, axis.high_limit) = (low_limit, high_limit);
        if let Ok(homed) = homed {
            axis.homed = homed;
        }
        axis
    }

    /// C `PIGCSMotorController::initAxis`: stops at the first failure.
    fn init_axis(&mut self, port_name: &str) -> AsynResult<()> {
        if self.lock().model == Model::C885 {
            // C PIC885Controller::initAxis: a reference sensor by
            // construction, no LIM?/CST? and no servo-on.
            self.has_reference = true;
            return self.enable_axis();
        }
        let addr = self.axis_no as i32;
        let flow = |args: std::fmt::Arguments<'_>| {
            PortManager::global().trace_manager().print(
                Some(port_name),
                Some(addr),
                0,
                TraceMask::FLOW,
                args,
            )
        };
        let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        self.has_limit_switches = ctrl.has_limit_switches(&self.axis_name)?;
        flow(format_args!(
            "PIGCSController::hasLimitSwitches() axis has {}limit switches\n",
            if self.has_limit_switches { "" } else { "no " }
        ));
        self.has_reference = ctrl.has_reference_sensor(&self.axis_name)?;
        flow(format_args!(
            "{}::hasReferenceSwitch() axis has {}reference sensor\n",
            if ctrl.model == Model::C702 {
                "PIC702Controller"
            } else {
                "PIGCSController"
            },
            if self.has_reference { "" } else { "no " }
        ));
        let stage_id = ctrl.stage_id(&self.axis_name)?;
        flow(format_args!(
            "PIGCSController::initAxis() stage configuration: {stage_id}\n"
        ));
        ctrl.set_servo(&self.axis_name, true)?;
        self.servo_on = true;
        Ok(())
    }

    fn lock(&self) -> MutexGuard<'_, PIGCS2Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The controller class's `getStatus`: `(homing, moving, negLimit,
    /// posLimit, servoControl)`.
    fn status(&self) -> AsynResult<(bool, bool, bool, bool, bool)> {
        let ctrl = self.lock();
        if !matches!(ctrl.model, Model::E755 | Model::C702) {
            return ctrl.get_status(self.axis_no);
        }
        // PIE755Controller::getStatus, and PIC702Controller's after the
        // first second of a REF: busy is homing and moving, no limits. C
        // defect not copied: neither sets servoControl, nor the limits in
        // that second, which poll then reads uninitialized; rs keeps the
        // last ones.
        if ctrl
            .ref_started
            .is_some_and(|at| at.elapsed() < Duration::from_secs(1))
        {
            return Ok((true, true, self.neg_limit, self.pos_limit, self.servo_on));
        }
        let busy = ctrl.get_busy()?;
        let moving = busy || ctrl.get_moving(self.axis_no)?;
        Ok((busy, moving, false, false, self.servo_on))
    }

    /// `setAccelerationCts`: convert the record's steps/s² by the CPU, skip
    /// it when equal to the last one set, lazily cache `min(MAX_ACCEL,
    /// MAX_DECEL)`, clamp the request to it, and write `CURR_ACCEL`/
    /// `CURR_DECEL`. Gated on `knows_vel`, matching both `getMaxAcceleration`
    /// and `setAcceleration`'s own early return.
    fn apply_acceleration(&mut self, acceleration: f64) -> AsynResult<()> {
        let (model, knows_vel) = {
            let ctrl = self.lock();
            (ctrl.model, ctrl.knows_vel())
        };
        if !model.sets_acceleration() {
            return Ok(());
        }
        let ((max_accel, max_decel), needs_vel) = model.max_acceleration_params();
        let acceleration = (acceleration / self.cpu).abs();
        // C compares the unclamped request with the clamped value stored.
        if acceleration == self.acceleration {
            return Ok(());
        }
        if self.max_acceleration.is_none() && (knows_vel || !needs_vel) {
            // C defect not copied: setAccelerationCts ignores a failed
            // getMaxAcceleration (PIGCSMotorController.cpp:47) and clamps to
            // the -1 sentinel, sending SPA -1; rs aborts the move instead.
            let max_acc = self.lock().get_gcs_parameter(&self.axis_name, max_accel)?;
            let max_dec = self.lock().get_gcs_parameter(&self.axis_name, max_decel)?;
            self.max_acceleration = Some(max_acc.min(max_dec));
        }
        // PIGCSMotorController::setAcceleration.
        if !knows_vel {
            return Ok(());
        }
        let accel = acceleration.min(self.max_acceleration.unwrap());
        let ctrl = self.lock();
        ctrl.set_gcs_parameter(&self.axis_name, PARA_MOT_CURR_ACCEL, accel)?;
        ctrl.set_gcs_parameter(&self.axis_name, PARA_MOT_CURR_DECEL, accel)?;
        drop(ctrl);
        self.acceleration = accel;
        Ok(())
    }

    /// C `PIC885Controller::setEnableAxis`: `EAX` only when `EAX?`
    /// differs.
    fn enable_axis(&self) -> AsynResult<()> {
        let ctrl = self.lock();
        if ctrl.get_enable_axis(&self.axis_name)? != 1 {
            ctrl.set_enable_axis(&self.axis_name, 1)?;
        }
        Ok(())
    }

    /// C `setServo`: a servo turned on clears the problem flag.
    fn set_servo(&mut self, enable: bool) -> AsynResult<()> {
        self.lock().set_servo(&self.axis_name, enable)?;
        self.servo_on = enable;
        if enable {
            self.problem = false;
        }
        Ok(())
    }

    /// C `referenceVelCts`: `PIGCSMotorController`'s writes the reference
    /// velocity, then `referenceAxis`; `PIGCSMotorControllerNoRefVel`'s
    /// skips the velocity; `PIC885Controller`'s sends `FRF` alone.
    fn reference_vel_cts(&mut self, velocity: f64, forward: bool) -> AsynResult<()> {
        let model = self.lock().model;
        if model == Model::C702 {
            return self.reference_c702(velocity, forward);
        }
        if model == Model::C885 {
            return self
                .lock()
                .reference_axis(&self.axis_name, forward, self.has_reference, false);
        }
        if model != Model::MotorNoRefVel && velocity != 0.0 {
            self.lock()
                .set_reference_velocity(&self.axis_name, velocity / self.cpu)?;
        }
        self.reference_axis(forward)
    }

    /// C `PIC702Controller::referenceVelCts`: servo on, `VEL`, then
    /// `REF`/`MPL`/`MNL` with no `ERR?`, starting the one-second window in
    /// which every axis reads as homing.
    fn reference_c702(&mut self, velocity: f64, forward: bool) -> AsynResult<()> {
        self.set_servo(true)?;
        self.lock()
            .set_velocity(&self.axis_name, velocity / self.cpu)?;
        let cmd = if self.has_reference {
            "REF"
        } else if self.has_limit_switches {
            if forward { "MPL" } else { "MNL" }
        } else {
            return Err(pigcs2_err(
                "PIC702Controller::referenceVelCts() failed - axis has no reference/limit switch",
            ));
        };
        let mut ctrl = self.lock();
        ctrl.send_only(&format!("{cmd} {}", self.axis_name))?;
        ctrl.ref_started = Some(Instant::now());
        drop(ctrl);
        self.is_homing = true;
        Ok(())
    }

    /// C `PIGCSMotorController::referenceAxis`: servo on, then the
    /// reference move.
    fn reference_axis(&mut self, forward: bool) -> AsynResult<()> {
        self.set_servo(true)?;
        self.lock().reference_axis(
            &self.axis_name,
            forward,
            self.has_reference,
            self.has_limit_switches,
        )
    }
}

impl AsynMotor for PIGCS2Axis {
    /// C `PIasynAxis::poll` ignores the controller's read status.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

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
            self.lock()
                .set_velocity(&self.axis_name, velocity / self.cpu)?;
        }
        if acceleration != 0.0 {
            self.apply_acceleration(acceleration)?;
        }
        self.lock().mov(&self.axis_name, position / self.cpu)?;
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
        // sign of velocity selects (TMN/TMX, already controller units). No
        // acceleration write, and — unlike move_absolute — `m_lastDirection`
        // is not updated here in C either.
        let ctrl = self.lock();
        ctrl.set_velocity(&self.axis_name, velocity / self.cpu)?;
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
        self.reference_vel_cts(velocity, forward)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        match ctrl.model {
            Model::C885 => ctrl.stop_all(),
            _ => ctrl.halt(&self.axis_name),
        }
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.lock()
            .set_axis_position(&self.axis_name, position / self.cpu)
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.set_servo(enable)
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let old_homing = self.is_homing;

        // C `PIasynAxis::poll` ignores every read's status and goes on, so a
        // failed read leaves what the last poll read. On a failed `#4`, C's
        // moving, limit and servo bits are uninitialized locals; that C
        // defect is not copied.
        let (homing, raw_moving, neg_limit, pos_limit, servo_control) = self.status().unwrap_or((
            old_homing,
            self.moving,
            self.neg_limit,
            self.pos_limit,
            self.servo_on,
        ));
        self.moving = raw_moving;
        self.neg_limit = neg_limit;
        self.pos_limit = pos_limit;
        self.is_homing = homing;
        // C only re-queries FRF? (the homed flag) right when homing just
        // finished (oldHoming true, new homing false) — not every cycle.
        if old_homing && !homing {
            let homed = self.lock().get_referenced_state(&self.axis_name);
            if let Ok(homed) = homed {
                self.homed = homed;
            }
            // C m_bEnableAxisAfterHoming / m_bSetServoAfterHoming, set by
            // PIC885Controller::initAxis alone.
            if self.homed && self.lock().model == Model::C885 {
                let _ = self.enable_axis();
                let _ = self.set_servo(true);
            }
        }
        // Servo dropping out on its own (not via set_closed_loop) is a
        // problem (C poll: "servo changed without user interaction!"); C
        // compares the servo state as the post-homing servo-on left it.
        if self.servo_on && !servo_control {
            self.problem = true;
        }
        if !homing || self.lock().model.is_gcs2() {
            self.servo_on = servo_control;
            let ctrl = self.lock();
            let position = ctrl
                .get_axis_position(&self.axis_name)
                .map_or(self.last_position, |p| p * self.cpu);
            let (mut low_limit, mut high_limit) = (self.low_limit, self.high_limit);
            let _ = ctrl.get_travel_limits(&self.axis_name, &mut low_limit, &mut high_limit);
            drop(ctrl);
            self.last_position = position;
            self.low_limit = low_limit;
            self.high_limit = high_limit;
        }
        let position = self.last_position;

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
            // PIasynAxis.cpp:197 motorStatusMoving_ = !done.
            msta_moving: !done,
            high_limit: pos_limit,
            low_limit: neg_limit,
            direction: self.last_direction,
            powered: self.servo_on,
            problem: self.problem,
            homed: self.homed,
            gain_support: true,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_per_unit_treats_a_zero_term_as_one() {
        assert_eq!(counts_per_unit(10000, 1), 10000.0);
        assert_eq!(counts_per_unit(5, 2), 2.5);
        assert_eq!(counts_per_unit(0, 1), 1.0);
        assert_eq!(counts_per_unit(1000, 0), 1.0);
    }

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

    /// Refuses the first write offered; records every write.
    struct FirstWriteRefused {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for FirstWriteRefused {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let mut written = self.written.lock().unwrap();
            written.push(String::from_utf8_lossy(data).into_owned());
            if written.len() == 1 {
                Err(pigcs2_err("refused"))
            } else {
                Ok(data.len())
            }
        }
    }

    #[test]
    fn send_only_status_is_the_terminator_write() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = FirstWriteRefused {
            base: PortDriverBase::new("gcs2SendOnly", 1, PortFlags::default()),
            written: written.clone(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("gcs2SendOnly").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl = PIGCS2Controller {
            handle: SyncIOHandle::from_handle(
                rt.port_handle().clone(),
                0,
                std::time::Duration::from_secs(1),
            ),
            serial_port: "gcs2SendOnly".into(),
            ident: String::new(),
            model: Model::default(),
            knows_vel: false,
            axis_names: Vec::new(),
            ref_started: None,
        };
        // C overwrites the refused command's status with the `\n` write's.
        assert!(ctrl.send_only("MOV 1 1").is_ok());
        assert_eq!(*written.lock().unwrap(), ["MOV 1 1", "\n"]);
    }

    /// Refuses the `#4` status query and answers `POS?`, `TMN?` and `TMX?`.
    struct StatusRefused {
        base: epics_rs::asyn::port::PortDriverBase,
        reply: Vec<u8>,
    }

    impl epics_rs::asyn::port::PortDriver for StatusRefused {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            match data {
                [4] => return Err(pigcs2_err("refused")),
                b"POS? 1" => self.reply = b"1=2.5".to_vec(),
                b"TMN? 1" => self.reply = b"1=-10".to_vec(),
                b"TMX? 1" => self.reply = b"1=10".to_vec(),
                _ => {}
            }
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    #[test]
    fn a_failed_status_read_keeps_the_last_bits_and_the_poll_goes_on() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = StatusRefused {
            base: PortDriverBase::new("gcs2StatusRefused", 1, PortFlags::default()),
            reply: Vec::new(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("gcs2StatusRefused").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl = PIGCS2Controller {
            handle: SyncIOHandle::from_handle(
                rt.port_handle().clone(),
                0,
                std::time::Duration::from_secs(1),
            ),
            serial_port: "gcs2StatusRefused".into(),
            ident: String::new(),
            model: Model::default(),
            knows_vel: false,
            axis_names: Vec::new(),
            ref_started: None,
        };
        let mut axis = PIGCS2Axis {
            controller: Arc::new(Mutex::new(ctrl)),
            axis_name: "1".into(),
            axis_no: 0,
            has_limit_switches: true,
            has_reference: true,
            homed: true,
            is_homing: false,
            servo_on: true,
            problem: false,
            moving: true,
            neg_limit: false,
            pos_limit: true,
            cpu: 1.0,
            last_position: 0.0,
            last_direction: true,
            low_limit: 0.0,
            high_limit: 0.0,
            max_acceleration: None,
            acceleration: 0.0,
        };
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        assert_eq!(status.position, 2.5);
        assert!(status.moving && status.msta_moving && status.high_limit && status.powered);
        assert!(!status.problem);
        assert_eq!((axis.low_limit, axis.high_limit), (-10.0, 10.0));
    }

    /// C sprintf's `%f` and `%.12g` (PIGCSController.cpp:142, 202, 242,
    /// 626; PIGCSMotorController.cpp:61): glibc spells a NaN `nan`/`-nan`,
    /// and SPA takes `%.12g`.
    #[test]
    fn commands_print_floats_as_c_sprintf() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2CFloat", |last| {
            if last == "ERR?" { "0" } else { "" }.into()
        });
        let mut ctrl = PIGCS2Controller::new(io, "gcs2CFloat");
        ctrl.knows_vel = true;
        let neg_nan = -f64::NAN;
        let _ = ctrl.mov("1", f64::NAN);
        let _ = ctrl.mov("1", neg_nan);
        let _ = ctrl.set_velocity("1", f64::NAN);
        let _ = ctrl.set_axis_position("1", neg_nan);
        let _ = ctrl.set_reference_velocity("1", f64::NAN);
        let _ = ctrl.set_gcs_parameter("1", 0x50, 1e-7);
        let _ = ctrl.set_gcs_parameter("1", 0x50, 2.5);
        let sent: Vec<String> = line
            .sent()
            .into_iter()
            .filter(|w| w != "ERR?" && w != "\n")
            .collect();
        assert_eq!(
            sent,
            [
                "MOV 1 nan",
                "MOV 1 -nan",
                "VEL 1 nan",
                "RON 1 0",
                "POS 1 -nan",
                "RON 1 1",
                "SPA 1 0x50 nan",
                "SPA 1 80 1e-07",
                "SPA 1 80 2.5",
            ]
        );
    }

    /// C `PIasynAxis::Init` (PIasynAxis.cpp:89-93): `getResolution`, then
    /// `getAxisVelocity`'s `VEL?` on a controller that knows it, then the
    /// position.
    #[test]
    fn axis_init_reads_the_velocity_before_the_position() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2InitVel", |last| {
            if last == "ERR?" { "0" } else { "1=1" }.into()
        });
        let mut ctrl = PIGCS2Controller::new(io, "gcs2InitVel");
        ctrl.knows_vel = true;
        let _axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "1".into(), 0, "gcs2InitVel");
        let sent: Vec<String> = line
            .sent()
            .into_iter()
            .filter(|w| w.contains('?') && w != "ERR?")
            .collect();
        let at = |q: &str| sent.iter().position(|w| w.starts_with(q));
        assert_eq!(at("VEL? 1").map(|i| i + 1), at("POS? 1"), "{sent:?}");
        assert!(at("SPA? 1 15") < at("VEL? 1"), "{sent:?}");
    }

    /// C `setAccelerationCts` (PIGCSMotorController.cpp:40-53) skips a
    /// request equal to the last acceleration set, compared before the
    /// clamp against the clamped value `setAcceleration` stored (:218).
    #[test]
    fn an_unchanged_acceleration_is_not_resent() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2AccelSkip", |last| {
            if last == "ERR?" { "0" } else { "1=5" }.into()
        });
        let mut ctrl = PIGCS2Controller::new(io, "gcs2AccelSkip");
        ctrl.knows_vel = true;
        let mut axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "1".into(), 0, "gcs2AccelSkip");
        line.sent();
        let mut sends = Vec::new();
        for accel in [2.0, 2.0, 3.0, 9.0, 9.0] {
            axis.apply_acceleration(accel).unwrap();
            let sent = line.sent();
            sends.push(sent.iter().filter(|w| w.starts_with("SPA 1 11 ")).count());
        }
        // 9 clamps to 5, so a second 9 differs from the stored 5.
        assert_eq!(sends, [1, 0, 1, 1, 1]);
    }

    /// C `PIGCSMotorController::referenceVelCts` (PIGCSMotorController.cpp:
    /// 55-77) writes the reference velocity first; its `referenceAxis`
    /// (:79-84) then turns the servo on before `FRF`.
    #[test]
    fn home_writes_the_reference_velocity_before_the_servo() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2HomeOrder", |last| {
            if last == "ERR?" { "0" } else { "1=1" }.into()
        });
        let ctrl = PIGCS2Controller::new(io, "gcs2HomeOrder");
        let mut axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "1".into(), 0, "gcs2HomeOrder");
        line.sent();
        axis.home(&AsynUser::new(0), 0.0, 2.0, 0.0, true).unwrap();
        let sent: Vec<String> = line.sent().into_iter().filter(|w| w != "\n").collect();
        assert_eq!(
            sent,
            [
                "SPA 1 0x50 2.000000",
                "ERR?",
                "SVO 1 1",
                "ERR?",
                "FRF 1",
                "ERR?"
            ]
        );
    }

    /// A C-885 on a line answering `reply`, after `PIasynAxis::Init`.
    fn c885_axis(
        name: &str,
        reply: fn(&str) -> String,
    ) -> (motor_common::testing::LineRig, PIGCS2Axis) {
        let (line, io) = motor_common::testing::LineRig::new(name, reply);
        let mut ctrl = PIGCS2Controller::new(io, name);
        ctrl.set_model(Model::C885);
        let axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "1".into(), 0, name);
        (line, axis)
    }

    fn c885_reply(last: &str) -> String {
        match last {
            "ERR?" => "0",
            "EAX? 1" => "1=0",
            "\u{4}" => "010000",
            _ => "1=1",
        }
        .into()
    }

    fn sent_lines(line: &motor_common::testing::LineRig) -> Vec<String> {
        line.sent().into_iter().filter(|w| w != "\n").collect()
    }

    /// C `PIC885Controller::initAxis` (PIC885Controller.cpp:51-70) asks no
    /// LIM?/TRS?/CST? and turns no servo on: it enables the axis through
    /// its `setEnableAxis` (:93-113), and the base `getResolution`
    /// (PIGCSController.cpp:738-744) reads no CPU.
    #[test]
    fn a_c885_axis_init_enables_the_axis() {
        let (line, axis) = c885_axis("gcs2C885Init", c885_reply);
        assert_eq!(
            sent_lines(&line),
            [
                "EAX? 1", "ERR?", "EAX 1 1", "ERR?", "POS? 1", "TMN? 1", "TMX? 1", "FRF? 1"
            ]
        );
        assert_eq!(axis.cpu, 10000.0);
        assert!(axis.has_reference && !axis.has_limit_switches && !axis.servo_on);
    }

    /// C-885 `referenceVelCts` (:115-152) sends `FRF` alone, `haltAxis`
    /// (:76-91) `STP`, and the base `setAccelerationCts` nothing.
    #[test]
    fn a_c885_axis_homes_with_frf_and_stops_with_stp() {
        let (line, mut axis) = c885_axis("gcs2C885Cmds", c885_reply);
        line.sent();
        let user = AsynUser::new(0);
        axis.home(&user, 0.0, 2.0, 0.0, true).unwrap();
        assert_eq!(sent_lines(&line), ["FRF 1", "ERR?"]);
        let _ = axis.stop(&user, 0.0);
        assert_eq!(sent_lines(&line), ["STP", "ERR?"]);
        axis.apply_acceleration(5.0).unwrap();
        assert!(sent_lines(&line).is_empty());
    }

    /// C `PIasynAxis::poll` (PIasynAxis.cpp:128-157): a C-885 homing that
    /// ends referenced re-enables the axis and turns the servo on before the
    /// position read; the status read before it still says servo off, so
    /// the servo check flags a problem.
    #[test]
    fn a_c885_axis_enables_and_servos_after_homing() {
        let (line, mut axis) = c885_axis("gcs2C885Poll", c885_reply);
        axis.is_homing = true;
        line.sent();
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        assert_eq!(
            sent_lines(&line),
            [
                "\u{4}", "FRF? 1", "EAX? 1", "ERR?", "EAX 1 1", "ERR?", "SVO 1 1", "ERR?",
                "POS? 1", "TMN? 1", "TMX? 1"
            ]
        );
        assert!(status.homed && status.problem && !status.powered);
    }

    /// An E-755 on a line answering `reply`, after `PIasynAxis::Init`.
    fn e755_axis(
        name: &str,
        reply: fn(&str) -> String,
    ) -> (motor_common::testing::LineRig, PIGCS2Axis) {
        let (line, io) = motor_common::testing::LineRig::new(name, reply);
        let mut ctrl = PIGCS2Controller::new(io, name);
        ctrl.set_model(Model::E755);
        ctrl.knows_vel = true;
        let axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "1".into(), 0, name);
        (line, axis)
    }

    fn e755_reply(last: &str) -> String {
        match last {
            "ERR?" => "0",
            // Not busy (0xB1); #5 says axis 1 moves.
            "\u{7}" => "1",
            "\u{5}" => "1",
            _ => "1=1",
        }
        .into()
    }

    /// `PIE755Controller` (PIE755Controller.h:22-37) runs the
    /// `PIGCSMotorController` init, the base `getResolution`
    /// (PIGCSController.cpp:760-766), which reads no CPU, and no-op
    /// acceleration writes.
    #[test]
    fn an_e755_axis_reads_no_cpu_and_sets_no_acceleration() {
        let (line, mut axis) = e755_axis("gcs2E755Init", e755_reply);
        assert_eq!(
            sent_lines(&line),
            [
                "LIM? 1", "TRS? 1", "CST? 1", "SVO 1 1", "ERR?", "VEL? 1", "POS? 1", "TMN? 1",
                "TMX? 1", "FRF? 1"
            ]
        );
        assert_eq!(axis.cpu, 10000.0);
        axis.apply_acceleration(5.0).unwrap();
        assert!(sent_lines(&line).is_empty());
        // The PIGCSMotorController referenceVelCts it inherits.
        axis.home(&AsynUser::new(0), 0.0, 20000.0, 0.0, true)
            .unwrap();
        assert_eq!(
            sent_lines(&line),
            [
                "SPA 1 0x50 2.000000",
                "ERR?",
                "SVO 1 1",
                "ERR?",
                "FRF 1",
                "ERR?"
            ]
        );
    }

    /// `PIE755Controller::getStatus` (PIE755Controller.cpp:9-35): `#7`
    /// busy is homing, else `#5` masked by the axis bit is moving; no
    /// limits. C never sets servoControl there; rs keeps the last servo.
    #[test]
    fn an_e755_axis_polls_busy_then_moving() {
        let (line, mut axis) = e755_axis("gcs2E755Poll", e755_reply);
        line.sent();
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        assert_eq!(
            sent_lines(&line),
            ["\u{7}", "\u{5}", "POS? 1", "TMN? 1", "TMX? 1"]
        );
        assert!(status.moving && !status.done && !status.high_limit && !status.low_limit);
        assert!(status.powered && !status.problem && !axis.is_homing);
    }

    /// A C-702 on a line answering `reply`, on a controller that knows
    /// `VEL`, after `PIasynAxis::Init`.
    fn c702_axis(name: &str, knows_vel: bool) -> (motor_common::testing::LineRig, PIGCS2Axis) {
        let (line, io) = motor_common::testing::LineRig::new(name, c702_reply);
        let mut ctrl = PIGCS2Controller::new(io, name);
        ctrl.set_model(Model::C702);
        ctrl.knows_vel = knows_vel;
        let axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "A".into(), 0, name);
        (line, axis)
    }

    fn c702_reply(last: &str) -> String {
        match last {
            "ERR?" => "0",
            "SAI?" => "BA",
            "\u{7}" => "0",
            "\u{5}" => "0",
            _ => "A=1",
        }
        .into()
    }

    /// `PIC702Controller::findConnectedAxes` (PIC702Controller.cpp:55-84):
    /// `SAI?` is one word of one-char axes, last char first.
    #[test]
    fn a_c702_finds_its_axes_in_one_reversed_word() {
        let (_line, io) = motor_common::testing::LineRig::new("gcs2C702Sai", c702_reply);
        let mut ctrl = PIGCS2Controller::new(io, "gcs2C702Sai");
        ctrl.set_model(Model::C702);
        ctrl.init();
        assert_eq!(ctrl.axis_names(), ["A", "B"]);
    }

    /// `PIC702Controller::hasReferenceSensor` (:107-122) is `SPA?` 0x14 and
    /// `getReferencedState` (:124-135) `SPA?` 0x1C.
    #[test]
    fn a_c702_axis_reads_reference_state_from_parameters() {
        let (line, axis) = c702_axis("gcs2C702Init", false);
        assert_eq!(
            sent_lines(&line),
            [
                "LIM? A",
                "SPA? A 20",
                "CST? A",
                "SVO A 1",
                "ERR?",
                "SPA? A 14",
                "SPA? A 15",
                "POS? A",
                "TMN? A",
                "TMX? A",
                "SPA? A 28"
            ]
        );
        assert!(axis.has_reference && axis.homed);
    }

    /// `PIC702Controller::getMaxAcceleration` (:86-105) reads CURR_ACCEL
    /// and CURR_DECEL, whether or not the controller knows `VEL`; the
    /// inherited `setAcceleration` writes only on one that does.
    #[test]
    fn a_c702_axis_caps_acceleration_by_the_current_parameters() {
        let (line, mut axis) = c702_axis("gcs2C702Acc", false);
        line.sent();
        axis.apply_acceleration(5.0).unwrap();
        assert_eq!(sent_lines(&line), ["SPA? A 11", "SPA? A 12"]);
        let (line, mut axis) = c702_axis("gcs2C702AccVel", true);
        line.sent();
        axis.apply_acceleration(5.0).unwrap();
        assert_eq!(
            sent_lines(&line),
            ["SPA? A 11", "SPA? A 12", "SPA A 11 1", "SPA A 12 1"]
        );
    }

    /// `PIC702Controller::referenceVelCts` (:137-185): servo, `VEL`, then
    /// `REF` with no `ERR?`. For a second after it, `getStatus` (:13-29)
    /// answers homing without I/O, and a C-702 (`IsGCS2` false) reads no
    /// position while homing (PIasynAxis.cpp:159).
    #[test]
    fn a_c702_axis_homes_with_ref_and_polls_nothing_while_it_starts() {
        let (line, mut axis) = c702_axis("gcs2C702Home", true);
        line.sent();
        let user = AsynUser::new(0);
        axis.home(&user, 0.0, 2000.0, 0.0, true).unwrap();
        assert_eq!(
            sent_lines(&line),
            ["SVO A 1", "ERR?", "VEL A 2000.000000", "REF A"]
        );
        let status = axis.poll(&user).unwrap();
        assert!(sent_lines(&line).is_empty());
        assert!(!status.done && status.moving && axis.is_homing && status.powered);
    }

    #[test]
    fn a_busy_byte_is_0xb0() {
        assert!(busy_from_reply(&[0xB0, b'\n']));
        assert!(!busy_from_reply(&[0xB1]));
        assert!(!busy_from_reply(&[]));
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

    #[test]
    fn models_are_c_create_gcs_controllers_classes() {
        assert_eq!(
            Model::from_idn("(C)2015 PHYSIK INSTRUMENTE, C-863.11, 0, 1.0"),
            Some(Model::Motor)
        );
        assert_eq!(Model::from_idn("PI E-873.3QTU"), Some(Model::MotorNoRefVel));
        assert_eq!(Model::from_idn("PI E-873.1AT"), Some(Model::Motor));
        assert_eq!(Model::from_idn("PI E-755.1CD"), Some(Model::E755));
        assert_eq!(Model::from_idn("PI C-702.00"), Some(Model::C702));
        assert_eq!(Model::from_idn("PI M-824"), Some(Model::Motor));
        assert_eq!(Model::from_idn("PI C-885.M1"), Some(Model::C885));
        assert_eq!(Model::from_idn("PI E-665"), None);
        assert_eq!(Model::from_idn(""), None);
    }

    /// C `PIGCSMotorControllerNoRefVel::referenceVelCts`
    /// (PIGCSMotorControllerNoRefVel.cpp:27-30) goes straight to
    /// `referenceAxis`: no `SPA 0x50` for the E-873.3QTU.
    #[test]
    fn a_no_ref_vel_controller_homes_without_the_reference_velocity() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2NoRefVel", |last| {
            if last == "ERR?" { "0" } else { "1=1" }.into()
        });
        let mut ctrl = PIGCS2Controller::new(io, "gcs2NoRefVel");
        ctrl.set_model(Model::MotorNoRefVel);
        let mut axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "1".into(), 0, "gcs2NoRefVel");
        line.sent();
        axis.home(&AsynUser::new(0), 0.0, 2.0, 0.0, true).unwrap();
        let sent: Vec<String> = line.sent().into_iter().filter(|w| w != "\n").collect();
        assert_eq!(sent, ["SVO 1 1", "ERR?", "FRF 1", "ERR?"]);
    }

    /// Takes every write and never replies.
    struct Silent(epics_rs::asyn::port::PortDriverBase);

    impl epics_rs::asyn::port::PortDriver for Silent {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, _buf: &mut [u8]) -> AsynResult<usize> {
            Err(pigcs2_err("timeout"))
        }
    }

    #[test]
    fn a_silent_controller_reads_an_empty_idn_and_finds_no_axes() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = Silent(PortDriverBase::new("gcs2Silent", 1, PortFlags::default()));
        let (rt, _actor) = create_port_runtime(
            &claim_port("gcs2Silent").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
        );
        let mut ctrl = PIGCS2Controller::new(handle, "gcs2Silent");
        assert_eq!(ctrl.read_idn(), "");
        ctrl.init();
        assert!(ctrl.axis_names().is_empty());
    }

    #[test]
    fn an_axis_is_created_when_the_controller_does_not_answer() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = Silent(PortDriverBase::new(
            "gcs2SilentAxis",
            1,
            PortFlags::default(),
        ));
        let (rt, _actor) = create_port_runtime(
            &claim_port("gcs2SilentAxis").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_millis(100),
        );
        let ctrl = PIGCS2Controller::new(handle, "gcs2SilentAxis");
        let axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "1".into(), 0, "gcs2SilentAxis");
        assert!(!axis.servo_on);
        assert!(!axis.homed);
        assert!(!axis.has_limit_switches);
        assert_eq!((axis.low_limit, axis.high_limit), (0.0, 0.0));
    }
}
