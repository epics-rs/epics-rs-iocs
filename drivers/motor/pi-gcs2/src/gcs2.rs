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
//! A reply can run over several lines: PI GCS marks a non-final line with a
//! trailing space before the EOS (`PIInterface::sendAndReceive`'s
//! `while(inputBuff[strlen(inputBuff)-1]==' ')` continuation loop), which
//! `PIGCS2Controller::send_and_receive` replicates instead of assuming one
//! line per reply.
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
//! - **Hexapod**: the `PIasynController` coordinate-system parameters
//!   (`PI_CS_ACTIVATE`/`LINK`/`KST`/`KSW`, octet writes) are not modeled.
//! - **Piezo**: `PIGCSPiezoController`, `PIGCS2PiezoCL` — closed-loop piezo
//!   gain/notch-filter parameters and the ~20 aux PVs `PIasynController` wires
//!   for them (`KP`/`KI`/`KFF`/`NTCHFR*`/`RBONT`/`RBOVF`/…).
//! - **Per-model controllers**: `PIE517Controller` and `PIE727Controller`
//!   (piezo-based, each with its own init and status quirks); those
//!   controllers run on [`Model::Motor`].
//! - **`SVO?` (`getServo`)**: as upstream, only the C-887's status asks it;
//!   the generic motor axis's servo state comes from the `char(4)` status
//!   bitmask. `PIGCSPiezoController`'s use of it is not ported.
//! - **`getAxisOnt`/`getAxisOvf`**: base-class no-ops (on-target/overflow
//!   status meaningful only for the piezo closed-loop path); not ported.
//! - **`getMoving`/`getBusy`** (`char(5)`/`char(7)`): the base class's
//!   fallback status transport for controllers that don't understand
//!   `char(4)`; unreachable once `PIGCSMotorController::getStatus` overrides
//!   it, so not ported.
//! - **Velocity readback**: the init `getAxisVelocity` (`VEL?`) is sent, but
//!   the `m_velocity` it seeds is never read back (not in `PIasynAxis::poll`),
//!   so `MotorStatus::velocity` stays `0.0`.

use std::cell::Cell;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus, OctetStep};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::motor::controller_port::ParamWrite;
use motor_common::util::{atof, atoi, nint};
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

/// C `strtol(s, NULL, radix)`: leading blanks, a sign and, in base 16, a
/// `0x` are read; no digit is 0 and an overflow saturates.
fn strtol(s: &str, radix: u32) -> i64 {
    let t = s.trim_start();
    let (negative, t) = match t.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, t.strip_prefix('+').unwrap_or(t)),
    };
    let t = match t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        Some(rest) if radix == 16 && rest.starts_with(|c: char| c.is_ascii_hexdigit()) => rest,
        _ => t,
    };
    let n = t.chars().take_while(|c| c.is_digit(radix)).count();
    if n == 0 {
        return 0;
    }
    let value = i64::from_str_radix(&t[..n], radix).unwrap_or(i64::MAX);
    if negative { -value } else { value }
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

/// A move target in whole counts. C defect not copied: C's `int` count
/// (PIGCSController.h:53 `moveCts(PIasynAxis*, int)`, called with the
/// record's double at PIasynAxis.cpp:263 and PIasynController.cpp:378, and
/// `targetsCts[n] = deferred_position` at PIasynController.cpp:250)
/// truncates toward zero; rs rounds with `NINT`, as the record's `RVAL`.
fn target_counts(position: f64) -> f64 {
    f64::from(nint(position))
}

/// C `TranslatePIError(epicsInt32)`.
fn error_message(error: i32) -> std::borrow::Cow<'static, str> {
    match crate::pi_errors::message(error) {
        Some(message) => message.into(),
        None => format!("Unknown Error - number {error}").into(),
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

/// `strtol` of axis `axis_no`'s 4 chars of a `char(4)` status reply
/// (`PIGCSMotorController::getStatus`: `idx = 2 + axisNo*4`). C defect not
/// copied: a reply ending before `idx` reads the buffer past it; rs has no
/// mask.
fn status_mask_at(reply: &str, axis_no: usize) -> Option<u32> {
    let idx = 2 + axis_no * 4;
    let mask_str = reply.get(idx..)?;
    Some(strtol(mask_str.get(..4).unwrap_or(mask_str), 16) as u32)
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
    /// `PIHexapodController`: a hexapod below GCS 2.0.
    Hexapod,
    /// `PIGCS2_HexapodController`.
    Gcs2Hexapod,
    /// `PICoordinateSystem` (C-887).
    CoordinateSystem,
}

impl Model {
    /// `PIGCSMotorController::getResolution`'s `SPA?` CPU reads; the others
    /// run the base's fixed one.
    fn reads_cpu(self) -> bool {
        !matches!(self, Self::C885 | Self::E755) && !self.is_hexapod()
    }

    /// `PIGCSMotorController::setAccelerationCts`; the others' is a no-op.
    fn sets_acceleration(self) -> bool {
        !matches!(self, Self::C885 | Self::E755) && !self.is_hexapod()
    }

    /// `PIHexapodController` or the GCS 2 class built on it.
    fn is_hexapod(self) -> bool {
        self == Self::Hexapod || self.is_gcs2_hexapod()
    }

    /// `PIGCS2_HexapodController` or `PICoordinateSystem`, built on it.
    fn is_gcs2_hexapod(self) -> bool {
        matches!(self, Self::Gcs2Hexapod | Self::CoordinateSystem)
    }

    /// C `CanCommunicateWhileHoming`: false on a GCS 1 hexapod, whose
    /// homing axis polls nothing (PIasynAxis.cpp:126).
    fn can_communicate_while_homing(self) -> bool {
        self != Self::Hexapod
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
        } else if any(&["HEXAPOD", "F-HEX", "F-206", "M-8"]) {
            Some(Self::Hexapod)
        } else if idn.contains("C-887") {
            Some(Self::CoordinateSystem)
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
    hexapod: HexapodState,
    /// `PIInterface::m_pCurrentLogSink`: the user of the axis entry point
    /// that ran last, `(port, addr)`; the serial port's until one does.
    log_sink: Option<(String, i32)>,
    /// `PIasynController::movesDeferred`.
    moves_deferred: bool,
    /// Each axis's `deferred_move`/`deferred_position`, by axis number.
    deferred: BTreeMap<usize, DeferredMove>,
    /// The counts each axis of the last `processDeferredMoves` was sent
    /// to, for its `m_lastDirection` against the position its next poll
    /// starts from (no poll ran in between, so C's `m_positionCts`).
    released: BTreeMap<usize, f64>,
    /// The controller port and its `PIasynController` parameters, which
    /// the poll posts.
    port: Option<(PortHandle, PiParams)>,
    /// `PIGCSController::m_LastError`: the last nonzero `ERR?` code.
    last_error: Cell<i32>,
    /// `PIHexapodController::m_PivotX/Y/Z`; `PIGCSController`'s `GetPivot*`
    /// are 0.
    pivot: [f64; 3],
    /// `PIGCSController::m_targetMode`, `PI_CS_TARGETMODE`.
    target_mode: i32,
}

/// `PIHexapodController`'s platform state, one for all its axes.
#[derive(Default)]
struct HexapodState {
    /// `m_bAnyAxisMoving`.
    any_moving: bool,
    /// Every axis's `m_bMoving`, as `getGlobalState` sets them all.
    moving: bool,
    /// `m_bHoming`: an `INI` runs until `#5` reads nothing moving.
    homing: bool,
    /// `m_bCanReadStatusWithChar4`.
    reads_status: bool,
    /// `m_bCanReadPosWithChar3`.
    reads_position: bool,
}

/// An axis's move held while moves are deferred.
struct DeferredMove {
    axis: String,
    counts: f64,
    cpu: f64,
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
            hexapod: HexapodState::default(),
            log_sink: None,
            moves_deferred: false,
            deferred: BTreeMap::new(),
            released: BTreeMap::new(),
            port: None,
            last_error: Cell::new(0),
            pivot: [0.0; 3],
            target_mode: 0,
        }
    }

    /// C `CreateGCSController`: the class for the `*IDN?`, asking a
    /// hexapod its `CSV?`: one at GCS 2.0 is a `PIGCS2_HexapodController`.
    pub fn create_model(&self, idn: &str) -> Option<Model> {
        match Model::from_idn(idn)? {
            Model::Hexapod if self.is_gcs_version(2.0) => Some(Model::Gcs2Hexapod),
            model => Some(model),
        }
    }

    /// C `IsGCSVersion`: a `CSV?` of at least `version`.
    fn is_gcs_version(&self, version: f32) -> bool {
        self.query("CSV?")
            .is_ok_and(|csv| atof(&csv) as f32 >= version)
    }

    /// C `PIasynController::poll`'s `getGlobalState`, run before the axes';
    /// `PIGCSController`'s is a no-op.
    pub fn poll(&mut self) {
        if self.model.is_hexapod() {
            let _ = self.get_global_state();
        }
        if self.model == Model::CoordinateSystem {
            // PICoordinateSystem::getActivate/getLink, whose replies feed
            // the PI_CS_ACTIVATE/PI_CS_LINK parameters, not modeled.
            let _ = self.query("KEN?");
            let _ = self.query("KLN?");
        }
        if let Some((port, params)) = &self.port {
            let [x, y, z] = self.pivot;
            let updates = vec![
                ParamSetValue::new(params.rbpivot_x, 0, ParamValue::Float64(x)),
                ParamSetValue::new(params.rbpivot_y, 0, ParamValue::Float64(y)),
                ParamSetValue::new(params.rbpivot_z, 0, ParamValue::Float64(z)),
                ParamSetValue::new(params.target_mode, 0, ParamValue::Int32(self.target_mode)),
                ParamSetValue::new(params.last_err, 0, ParamValue::Int32(self.last_error.get())),
            ];
            let _ = port.set_params_and_notify_blocking(0, updates);
        }
    }

    /// C `AcceptsNewTarget`: a GCS 1 hexapod takes none while its platform
    /// moves.
    fn accepts_new_target(&self) -> bool {
        self.model != Model::Hexapod || !self.hexapod.any_moving
    }

    /// `PIHexapodController::moveCts`: `cmd`, 0.2 s, then `#5`; a platform
    /// that has not started is an error when `ERR?` says so.
    fn hexapod_move(&mut self, cmd: &str) -> AsynResult<()> {
        self.send_only(cmd)?;
        std::thread::sleep(Duration::from_millis(200));
        self.get_global_state()?;
        if !self.hexapod.moving {
            let err = self.get_gcs_error()?;
            if err != 0 {
                self.print(format_args!(
                    "PIHexapodController::moveCts() failed, GCS error {err}\n"
                ));
                return Err(pigcs2_err(format!("MOV failed, GCS error {err}")));
            }
        }
        self.hexapod.any_moving = true;
        Ok(())
    }

    /// `PIHexapodController::getGlobalState`: a non-zero `#5` is every axis
    /// moving, and a zero one ends an `INI`.
    fn get_global_state(&mut self) -> AsynResult<()> {
        let reply = self.query_char(5).inspect_err(|e| {
            print!(
                "PIGCSController::getGlobalState() failed, status {}",
                e.status() as i32
            )
        })?;
        let moving = strtol(&reply, 16) != 0;
        let hexapod = &mut self.hexapod;
        hexapod.any_moving = moving;
        hexapod.moving = moving;
        if !moving {
            hexapod.homing = false;
        }
        Ok(())
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
        if self.model.is_hexapod() {
            self.init_hexapod();
        }
    }

    /// `PIHexapodController::init` after the base's: whether `#4` and `#3`
    /// answer, then the pivot point.
    fn init_hexapod(&mut self) {
        self.hexapod.reads_status = self.answers_char(4);
        self.hexapod.reads_position = self.answers_char(3);
        self.hexapod.homing = false;
        // ReadPivotSettings, with GetReadPivotCommand's query.
        let read_pivot = if self.model == Model::Hexapod {
            "SPI? RST"
        } else {
            "SPI? R S T"
        };
        let Ok(reply) = self.query(read_pivot) else {
            return;
        };
        if !matches!(self.get_gcs_error(), Ok(0)) {
            return;
        }
        // One "R = 1.0 " line per pivot coordinate; one unparsed keeps all.
        let mut pivot = [0.0; 3];
        for line in reply.split('\n').map(|line| line.trim_start_matches(' ')) {
            let Ok(value) = get_value_f64(line) else {
                return;
            };
            match line.bytes().next() {
                Some(b'R') => pivot[0] = value,
                Some(b'S') => pivot[1] = value,
                Some(b'T') => pivot[2] = value,
                _ => {}
            }
        }
        self.pivot = pivot;
    }

    /// C `SetPivotX/Y/Z` for coordinate `index` (`R`/`S`/`T`): a hexapod
    /// sends `SPI` + `ERR?` unless its platform moves (`SetPivot`); the
    /// others only log it. C's FLOW lines have no newline.
    fn set_pivot(&mut self, index: usize, value: f64) -> AsynResult<()> {
        let xyz = ["X", "Y", "Z"][index];
        if !self.model.is_hexapod() {
            self.trace(
                TraceMask::FLOW,
                format_args!("PIGCSController::SetPivot{xyz}() ignored"),
            );
            return Ok(());
        }
        self.trace(
            TraceMask::FLOW,
            format_args!(
                "PIHexapodController::SetPivot{xyz}() value {}",
                CFloat(value)
            ),
        );
        if self.hexapod.any_moving {
            self.trace(
                TraceMask::FLOW,
                format_args!(
                    "PIHexapodController::SetPivot() cannot change pivot point while platform is moving"
                ),
            );
            return Err(pigcs2_err("platform is moving"));
        }
        let rst = ["R", "S", "T"][index];
        self.send_only(&format!("SPI {rst} {}", CFloat(value)))?;
        let err = self.get_gcs_error()?;
        if err != 0 {
            self.print(format_args!(
                "PIHexapodController::SetPivot() failed, GCS error {err}\n"
            ));
            return Err(pigcs2_err(format!("SPI failed, GCS error {err}")));
        }
        self.pivot[index] = value;
        Ok(())
    }

    /// Whether byte `c` is answered; a timeout's `ERR?` clears the unknown
    /// command. C defect not copied: another failure leaves the flag
    /// uninitialized; rs reads it as no.
    fn answers_char(&self, c: u8) -> bool {
        match self.send_and_receive_char(c) {
            Ok(_) => true,
            Err(e) => {
                if e.status() == AsynStatus::Timeout {
                    let _ = self.get_gcs_error();
                }
                false
            }
        }
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
    /// `\n`. Either write falling short is asynError. PI GCS marks a
    /// non-final reply line with a trailing space: each line after one is a
    /// plain read, joined on `\n`, whose failure C does not print.
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
        let mut reply = self
            .handle
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
            })?;
        while reply.last() == Some(&b' ') {
            reply.push(b'\n');
            reply.extend(self.handle.read_octet(0, READ_BUF)?);
        }
        Ok(reply)
    }

    /// `asynPrint(pAsynCom, ASYN_TRACE_ERROR, ...)`.
    pub fn print(&self, args: std::fmt::Arguments<'_>) {
        self.trace(TraceMask::ERROR, args);
    }

    /// `asynPrint(m_pCurrentLogSink, mask, ...)`.
    fn trace(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        let (port, addr) = match &self.log_sink {
            Some((port, addr)) => (port.as_str(), *addr),
            None => (self.serial_port.as_str(), 0),
        };
        PortManager::global()
            .trace_manager()
            .print(Some(port), Some(addr), 0, mask, args);
    }

    /// `SAI?`: every connected axis name, one per reply line less its
    /// trailing spaces (C `findConnectedAxes`).
    fn find_connected_axes(&self) -> AsynResult<Vec<String>> {
        if self.model.is_hexapod() {
            // PIHexapodController::findConnectedAxes asks nothing.
            return Ok(["X", "Y", "Z", "U", "V", "W"].map(String::from).to_vec());
        }
        if self.model == Model::C702 {
            // PIC702Controller::findConnectedAxes: one word of one-char
            // axes, last first.
            let word = self.query("SAI?")?;
            if word.len() >= MAX_NR_AXES {
                return Err(pigcs2_err(format!("{} axes in SAI? reply", word.len())));
            }
            return Ok(word.chars().rev().map(String::from).collect());
        }
        let raw = self.send_and_receive("SAI?")?;
        Ok(String::from_utf8_lossy(&raw)
            .split('\n')
            .map(|line| line.trim_end_matches(['\r', '\0', ' ']))
            .filter(|name| !name.is_empty())
            .map(String::from)
            .collect())
    }

    /// C `getGCSError`: a non-zero code prints, with its message, once
    /// there is a log sink.
    fn get_gcs_error(&self) -> AsynResult<i32> {
        let err = atoi(&self.query("ERR?")?);
        if err != 0 {
            self.last_error.set(err);
        }
        if err != 0 && self.log_sink.is_some() {
            self.print(format_args!(
                "PIGCSController::getGCSError() GCS error code = {err}\n"
            ));
            self.print(format_args!(
                "PIGCSController::getGCSError() GCS error, {}\n",
                error_message(err)
            ));
        }
        Ok(err)
    }

    /// `ERR?` reading `expected`; any other code prints C's `failed` line
    /// for it and is an error.
    fn check(&self, expected: i32, failed: impl FnOnce(i32) -> String) -> AsynResult<()> {
        let err = self.get_gcs_error()?;
        if err == expected {
            return Ok(());
        }
        self.print(format_args!("{}", failed(err)));
        Err(pigcs2_err(format!("GCS error {err}")))
    }

    /// `MOV {axis} {position}` + `ERR? == 0`.
    fn mov(&self, axis: &str, position: f64) -> AsynResult<()> {
        self.send_only(&format!("MOV {axis} {}", CFloat(position)))?;
        self.check(0, |err| {
            format!("PIGCSController::move() failed, GCS error {err}\n")
        })
    }

    /// `VEL {axis} {|velocity|}`, gated on [`Self::knows_vel`]; no error
    /// check (C `setVelocityCts` is fire-and-forget).
    fn set_velocity(&self, axis: &str, velocity: f64) -> AsynResult<()> {
        if self.model.is_gcs2_hexapod() {
            // PIGCS2_HexapodController::setVelocityCts: the platform's.
            return self.send_only(&format!("VLS {}", CFloat(velocity.abs())));
        }
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

    /// C `PIasynController::processDeferredMoves` and
    /// `PIGCSController::moveCts(array)`: one `MOV` for every deferred
    /// axis, then `ERR?`.
    fn process_deferred_moves(&mut self) -> AsynResult<()> {
        let deferred = std::mem::take(&mut self.deferred);
        if deferred.is_empty() {
            return Ok(());
        }
        let mut cmd = String::from("MOV");
        for (&axis_no, mv) in &deferred {
            cmd.push_str(&format!(" {} {}", mv.axis, CFloat(mv.counts / mv.cpu)));
            self.released.insert(axis_no, mv.counts);
        }
        if self.model.is_hexapod() {
            return self.hexapod_move(&cmd);
        }
        self.send_only(&cmd)?;
        let err = self.get_gcs_error()?;
        if err != 0 {
            self.print(format_args!(
                "PIGCSController::moveCts(array) failed, GCS error {err}\n"
            ));
            return Err(pigcs2_err(format!("MOV failed, GCS error {err}")));
        }
        Ok(())
    }

    /// `STP` + `ERR? == PI_CNTR_STOP`: stops every axis (C
    /// `PIC885Controller::haltAxis`).
    fn stop_all(&self) -> AsynResult<()> {
        self.send_only("STP")?;
        self.check(PI_CNTR_STOP, |err| {
            format!("PIC885Controller::haltAxis() failed, GCS error {err}")
        })
    }

    /// `EAX? {axis}`, then `ERR? == 0` before the value counts (C
    /// `getEnableAxis`).
    fn get_enable_axis(&self, axis: &str) -> AsynResult<i32> {
        let reply = self.query(&format!("EAX? {axis}"))?;
        // C defect not copied: its line names setEnableAxis.
        self.check(0, |err| {
            format!("PIGCSController::getEnableAxis() failed, GCS error {err}\n")
        })?;
        value_str(&reply)
            .map(atoi)
            .ok_or_else(|| pigcs2_err(format!("no '=' in reply {reply:?}")))
    }

    /// `EAX {axis} {state}` + `ERR? == 0` (C `PIGCSController::setEnableAxis`).
    fn set_enable_axis(&self, axis: &str, state: i32) -> AsynResult<()> {
        self.send_only(&format!("EAX {axis} {state}"))?;
        self.check(0, |err| {
            format!("PIGCSController::setEnableAxis() failed, GCS error {err}\n")
        })
    }

    /// C `sendOnly(char)`: the byte alone, no terminator; a byte not
    /// written is asynError whatever the write returned.
    fn send_only_char(&self, c: u8) -> AsynResult<()> {
        let e = match self.handle.write_octet(0, &[c]) {
            Ok(1) => return Ok(()),
            Ok(n) => pigcs2_err(format!("{n} bytes written")).with_partial_write(n),
            Err(e) if e.partial_write() == Some(1) => e,
            Err(e) => {
                let sent = e.partial_write().unwrap_or(0);
                pigcs2_err(e.to_string()).with_partial_write(sent)
            }
        };
        self.print(format_args!(
            "PIGCSController:sendOnly: error sending command {c}, sent={}, status={}\n",
            e.partial_write().unwrap_or(0),
            e.status() as i32
        ));
        Err(e)
    }

    /// `PIHexapodController::haltAxis`: byte 24 + `ERR? == PI_CNTR_STOP`.
    fn halt_platform(&self) -> AsynResult<()> {
        self.send_only_char(24)?;
        self.check(PI_CNTR_STOP, |err| {
            format!("PIGCSController::haltAxis() failed, GCS error {err}")
        })
    }

    /// `PIHexapodController::referenceVelCts`: `INI X` homes the platform,
    /// whatever the axis.
    fn init_platform(&mut self) -> AsynResult<()> {
        if let Err(e) = self.send_only("INI X") {
            let _ = self.get_gcs_error();
            self.print(format_args!(
                "PIHexapodController::referenceVelCts() failed\n"
            ));
            return Err(e);
        }
        self.hexapod.homing = true;
        Ok(())
    }

    /// `HLT {axis}` + `ERR? == PI_CNTR_STOP`.
    fn halt(&self, axis: &str) -> AsynResult<()> {
        self.send_only(&format!("HLT {axis}"))?;
        self.check(PI_CNTR_STOP, |err| {
            format!("PIGCSController::haltAxis() failed, GCS error {err}")
        })
    }

    /// `RON {axis} 0` / `POS {axis} {position}` / `RON {axis} 1`, one
    /// `ERR? == 0` after all three (C `setAxisPosition`).
    fn set_axis_position(&self, axis: &str, position: f64) -> AsynResult<()> {
        self.send_only(&format!("RON {axis} 0"))?;
        self.send_only(&format!("POS {axis} {}", CFloat(position)))?;
        self.send_only(&format!("RON {axis} 1"))?;
        self.check(0, |err| {
            format!("PIGCSController::setAxisPosition() failed, GCS error {err}\n")
        })
    }

    /// C `getAxisVelocity`: `VEL? {axis}` on a controller that knows it.
    fn get_axis_velocity(&self, axis: &str) -> AsynResult<f64> {
        if self.model.is_gcs2_hexapod() {
            // PIGCS2_HexapodController::getAxisVelocity: the platform's.
            return get_value_f64(&self.query("VLS?")?);
        }
        if !self.knows_vel {
            return Ok(0.0);
        }
        get_value_f64(&self.query(&format!("VEL? {axis}"))?)
    }

    /// `POS? {axis}`. A GCS 1 hexapod whose platform moves reads the axis
    /// from `#3` when that answers, else keeps `last`
    /// (`PIHexapodController::getAxisPosition`).
    fn get_axis_position(&self, axis: &str, last: f64) -> AsynResult<f64> {
        if self.model == Model::Hexapod && self.hexapod.any_moving {
            if !self.hexapod.reads_position {
                return Ok(last);
            }
            // One "X = 1.0 " line per axis, matched on the first character.
            let reply = self.query_char(3)?;
            let line = reply
                .split('\n')
                .map(|line| line.trim_start_matches(' '))
                .find(|line| line.bytes().next() == axis.bytes().next())
                .ok_or_else(|| pigcs2_err(format!("no axis {axis} in #3 reply {reply:?}")))?;
            return get_value_f64(line);
        }
        get_value_f64(&self.query(&format!("POS? {axis}"))?)
    }

    /// `TMN? {axis}` / `TMX? {axis}` -> `(negLimit, posLimit)`.
    /// Each limit is stored as soon as it is read, so a failed `TMX?` keeps
    /// the `TMN?` value, as C's reference out-parameters do.
    fn get_travel_limits(&self, axis: &str, neg: &mut f64, pos: &mut f64) -> AsynResult<()> {
        if self.model == Model::CoordinateSystem {
            // PICoordinateSystem::getTravelLimits.
            *neg = get_value_f64(&self.query(&format!("TRA? {axis} -1"))?)?;
            *pos = get_value_f64(&self.query(&format!("TRA? {axis} 1"))?)?;
            return Ok(());
        }
        if self.model.is_hexapod() {
            // PIHexapodController::getTravelLimits.
            (*neg, *pos) = (-100.0, 100.0);
            return Ok(());
        }
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

    /// `FRF? {axis}`; a C-702's `REFERENCED` parameter; a GCS 1 hexapod's
    /// `#4` bit 0x10000, or referenced when `#4` does not answer.
    fn get_referenced_state(&self, axis: &str) -> AsynResult<bool> {
        if self.model.is_hexapod() {
            if !self.hexapod.reads_status {
                return Ok(true);
            }
            return Ok(strtol(&self.query_char(4)?, 10) & 0x10000 != 0);
        }
        if self.model == Model::C702 {
            return Ok(self.get_gcs_parameter(axis, PARA_C702_REFERENCED)? > 0.1);
        }
        get_value_bool(&self.query(&format!("FRF? {axis}"))?)
    }

    /// `SVO? {axis}`.
    fn get_servo(&self, axis: &str) -> AsynResult<bool> {
        get_value_bool(&self.query(&format!("SVO? {axis}"))?)
    }

    /// `SVO {axis} {0|1}` + `ERR? == 0`.
    fn set_servo(&self, axis: &str, enable: bool) -> AsynResult<()> {
        self.send_only(&format!("SVO {axis} {}", enable as i32))?;
        self.check(0, |_| "Could not set servo state!\n".into())
    }

    /// `SPA {axis} 0x50 {|velocity|}` + `ERR? == 0` (C `referenceVelCts`'s
    /// own inline error check, distinct from the unchecked
    /// [`Self::set_gcs_parameter`]).
    /// C ignores the write's status.
    fn set_reference_velocity(&self, axis: &str, velocity: f64) -> AsynResult<()> {
        let _ = self.send_only(&format!(
            "SPA {axis} {REF_VEL_PARAM_HEX} {}",
            CFloat(velocity.abs())
        ));
        self.check(0, |_| {
            "PIGCSMotorController::referenceVelCts() failed\n".into()
        })
    }

    /// `FRF`/`FPL`/`FNL {axis}` + `ERR? == 0` (C `referenceAxis`, and the
    /// C-885's `referenceVelCts`).
    fn reference_axis(
        &self,
        axis: &str,
        forward: bool,
        has_reference: bool,
        has_limit_switches: bool,
    ) -> AsynResult<()> {
        let class = if self.model == Model::C885 {
            "PIC885Controller"
        } else {
            "PIGCSMotorController"
        };
        if has_reference {
            self.send_only(&format!("FRF {axis}"))?;
        } else if has_limit_switches {
            let cmd = if forward { "FPL" } else { "FNL" };
            self.send_only(&format!("{cmd} {axis}"))?;
        } else {
            self.print(format_args!(
                "{class}::referenceVelCts() failed - axis has no reference/limit switch\n"
            ));
            return Err(pigcs2_err(format!(
                "axis {axis} has no reference sensor or limit switch to home against"
            )));
        }
        self.check(0, |_| format!("{class}::referenceVelCts() failed\n"))
    }

    /// C `sendAndReceive(char)`: a short write is asynError. The byte is
    /// written again for each line that ends in a space, whose failure C
    /// does not print. C defect not copied: it appends each line with no
    /// separator, so `PIHexapodController`'s `#3` parse sees only the first
    /// line; rs joins them with `\n`, as the string `sendAndReceive` does.
    fn send_and_receive_char(&self, c: u8) -> AsynResult<Vec<u8>> {
        let write_read = || {
            self.handle
                .write_read(0, &[c], READ_BUF)
                .map_err(|e| match e.octet_step() {
                    Some(OctetStep::Read) => e,
                    _ => pigcs2_err(e.message()),
                })
        };
        let mut reply = write_read().inspect_err(|e| {
            self.print(format_args!(
                "PIInterface::sendAndReceive error calling writeRead, output={c} status={}, error={}\n",
                e.status() as i32,
                e.message()
            ));
        })?;
        while reply.last() == Some(&b' ') {
            reply.push(b'\n');
            reply.extend(write_read()?);
        }
        Ok(reply)
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

    /// C `getMoving`: `#5`'s hex word masked by the axis's
    /// `m_movingStateMask`.
    fn get_moving(&self, mask: i64) -> AsynResult<bool> {
        Ok(strtol(&self.query_char(5)?, 16) & mask != 0)
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
        let reply = self.query_char(4)?;
        let mask = status_mask_at(&reply, axis_no).ok_or_else(|| {
            pigcs2_err(format!(
                "status reply too short for axis {axis_no}: {reply:?}"
            ))
        })?;
        Ok(status_from_bitmask(mask))
    }
}

/// C `PIasynController`'s parameters (PIasynController.cpp:68-102).
#[derive(Clone, Copy, Debug)]
struct PiParams {
    last_err: usize,
    clear_err: usize,
    pivot_x: usize,
    pivot_y: usize,
    pivot_z: usize,
    rbpivot_x: usize,
    rbpivot_y: usize,
    rbpivot_z: usize,
    target_mode: usize,
}

/// What C `PIasynController` adds to the `asynMotorController` port: its
/// parameters and writes, and `report` (PIasynController.cpp:196-221) over
/// the axes `PI_GCS2_CreateController` created.
#[derive(Clone, Default)]
pub struct PIGCS2Params {
    axes: Arc<Mutex<Vec<Arc<Mutex<PIGCS2Axis>>>>>,
    /// C `m_pGCSController`, NULL until the constructor makes one.
    controller: Arc<OnceLock<Arc<Mutex<PIGCS2Controller>>>>,
    params: Arc<OnceLock<PiParams>>,
}

impl PIGCS2Params {
    /// C `m_pGCSController = CreateGCSController(...)`: `controller` takes
    /// the port's writes and posts its poll to `port`.
    pub fn attach(&self, controller: Arc<Mutex<PIGCS2Controller>>, port: PortHandle) {
        if let Some(&params) = self.params.get() {
            controller.lock().unwrap_or_else(|e| e.into_inner()).port = Some((port, params));
        }
        let _ = self.controller.set(controller);
    }

    /// An axis the report lists, in creation order.
    pub fn add(&self, axis: Arc<Mutex<PIGCS2Axis>>) {
        self.axes
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(axis);
    }
}

impl ControllerParams for PIGCS2Params {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let mut create = |name, kind| base.create_param(name, kind);
        let params = PiParams {
            last_err: create("PI_SUP_LAST_ERR", ParamType::Int32)?,
            clear_err: create("PI_SUP_CLEAR_ERR", ParamType::Int32)?,
            pivot_x: create("PI_SUP_PIVOT_X", ParamType::Float64)?,
            pivot_y: create("PI_SUP_PIVOT_Y", ParamType::Float64)?,
            pivot_z: create("PI_SUP_PIVOT_Z", ParamType::Float64)?,
            rbpivot_x: create("PI_SUP_RBPIVOT_X", ParamType::Float64)?,
            rbpivot_y: create("PI_SUP_RBPIVOT_Y", ParamType::Float64)?,
            rbpivot_z: create("PI_SUP_RBPIVOT_Z", ParamType::Float64)?,
            target_mode: create("PI_CS_TARGETMODE", ParamType::Int32)?,
        };
        let _ = self.params.set(params);
        Ok(())
    }

    /// C `PIasynController::writeInt32`'s own parameters; the base stores
    /// the written value and runs the callbacks, as C's.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> ParamWrite {
        let (Some(controller), Some(params)) = (self.controller.get(), self.params.get()) else {
            return ParamWrite::PassToBase(Ok(()));
        };
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        // Each also sets its address-0 value, as C's setIntegerParam(0, ...).
        let (reason, value) = if user.reason == params.clear_err {
            ctrl.last_error.set(0);
            (params.last_err, 0)
        } else if user.reason == params.target_mode {
            ctrl.target_mode = value;
            (params.target_mode, value)
        } else {
            return ParamWrite::PassToBase(Ok(()));
        };
        ParamWrite::PassToBase(base.set_int32_param(reason, 0, value))
    }

    /// C `PIasynController::writeFloat64`'s own parameters.
    fn write_float64(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: f64,
    ) -> ParamWrite {
        let (Some(controller), Some(params)) = (self.controller.get(), self.params.get()) else {
            return ParamWrite::PassToBase(Ok(()));
        };
        let pivots = [params.pivot_x, params.pivot_y, params.pivot_z];
        let Some(index) = pivots.iter().position(|&r| r == user.reason) else {
            return ParamWrite::PassToBase(Ok(()));
        };
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        ctrl.log_sink = Some((base.port_name.to_string(), user.addr));
        ParamWrite::PassToBase(ctrl.set_pivot(index, value))
    }

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
    /// The controller port its user is on, C `pasynUser_`'s.
    port_name: String,
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
            port_name: port_name.to_owned(),
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
        axis.log_here();
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
        let position = ctrl.get_axis_position(&axis.axis_name, axis.last_position / cpu);
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
        if self.lock().model.is_hexapod() {
            // PIHexapodController::initAxis: no LIM?/TRS?/CST?.
            return self.set_servo(true);
        }
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

    /// `PIasynController::writeInt32`'s FLOW line for a flag write. C
    /// defect not copied: its format takes the verb first, printing
    /// "Sett:PIasynDriver: writeInt32ing ...".
    fn flag_write_flow(&self, verb: &str, flag: &str) {
        PortManager::global().trace_manager().print(
            Some(&self.port_name),
            Some(self.axis_no as i32),
            0,
            TraceMask::FLOW,
            format_args!(
                "PIasynDriver:writeInt32: {verb}ing {flag} flag on driver {}\n",
                self.port_name
            ),
        );
    }

    /// C `m_pCurrentLogSink = pasynUser_`, at the head of `Init` and of
    /// each axis entry point and controller write.
    fn log_here(&self) {
        self.lock().log_sink = Some((self.port_name.clone(), self.axis_no as i32));
    }

    /// The controller class's `getStatus`: `(homing, moving, negLimit,
    /// posLimit, servoControl)`.
    fn status(&self) -> AsynResult<(bool, bool, bool, bool, bool)> {
        let ctrl = self.lock();
        if ctrl.model == Model::CoordinateSystem {
            // PICoordinateSystem::getStatus: #5 against the mask of 1
            // PIHexapodController::initAxis gives every axis, and SVO?.
            let moving = ctrl.get_moving(1)?;
            let servo = ctrl.get_servo(&self.axis_name)?;
            return Ok((false, moving, false, false, servo));
        }
        if ctrl.model.is_hexapod() {
            // PIHexapodController::getStatus: the platform state, no limits.
            // C defect not copied: it never sets servoControl; rs keeps the
            // last servo state.
            let moving = ctrl.hexapod.moving || ctrl.deferred.contains_key(&self.axis_no);
            return Ok((ctrl.hexapod.homing, moving, false, false, self.servo_on));
        }
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
        // PIGCSController::initAxis's mask, the axis bit.
        let moving = busy || ctrl.get_moving(1 << self.axis_no)?;
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
        if model == Model::Hexapod {
            return self.lock().init_platform();
        }
        if model.is_gcs2_hexapod() {
            // PIGCS2_HexapodController::referenceVelCts ignores the velocity.
            self.set_servo(true)?;
            let ctrl = self.lock();
            ctrl.send_only(&format!("FRF {}", self.axis_name))?;
            return ctrl.check(0, |_| {
                "PIGCS2_HexapodController::referenceVelCts() failed\n".into()
            });
        }
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
            self.lock().print(format_args!(
                "PIC702Controller::referenceVelCts() failed - axis has no reference/limit switch\n"
            ));
            return Err(pigcs2_err(
                "PIC702Controller::referenceVelCts() failed - axis has no reference/limit switch",
            ));
        };
        let mut ctrl = self.lock();
        ctrl.send_only(&format!("{cmd} {}", self.axis_name))
            .inspect_err(|_| {
                ctrl.print(format_args!("PIC702Controller::referenceVelCts() failed\n"))
            })?;
        ctrl.ref_started = Some(Instant::now());
        drop(ctrl);
        self.is_homing = true;
        Ok(())
    }

    /// `PIasynAxis::move`/`moveVelocity`'s refusal when the controller
    /// takes no new target, printed on the axis user.
    fn refuse_target(&self, function: &str, end: &str) -> AsynError {
        PortManager::global().trace_manager().print(
            Some(&self.port_name),
            Some(self.axis_no as i32),
            0,
            TraceMask::ERROR,
            format_args!(
                "PIasynAxis:{function}: Set port {}, axis {} - controller does not accept new target (busy?){end}",
                self.port_name, self.axis_no
            ),
        );
        pigcs2_err("controller does not accept new target (busy?)")
    }

    /// C `PIasynAxis::poll`'s reads after the status, skipped while a
    /// controller that cannot talk while homing homes.
    fn poll_reads(&mut self, old_homing: bool, servo_control: bool) {
        let homing = self.is_homing;
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
                .get_axis_position(&self.axis_name, self.last_position / self.cpu)
                .map_or(self.last_position, |p| p * self.cpu);
            // C reads it again for PI_SUP_POSITION, not modeled yet.
            let _ = ctrl.get_axis_position(&self.axis_name, position / self.cpu);
            let (mut low_limit, mut high_limit) = (self.low_limit, self.high_limit);
            let _ = ctrl.get_travel_limits(&self.axis_name, &mut low_limit, &mut high_limit);
            drop(ctrl);
            self.last_position = position;
            self.low_limit = low_limit;
            self.high_limit = high_limit;
        }
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
        self.log_here();
        if !self.lock().accepts_new_target() {
            return Err(self.refuse_target("moveAxis", "\n"));
        }
        let position = target_counts(position);
        let mut ctrl = self.lock();
        if ctrl.moves_deferred {
            ctrl.deferred.insert(
                self.axis_no,
                DeferredMove {
                    axis: self.axis_name.clone(),
                    counts: position,
                    cpu: self.cpu,
                },
            );
            return Ok(());
        }
        drop(ctrl);
        // C `PIasynAxis::move`: VEL, then SPA accel/decel, then MOV — each an
        // independent command, no atomicity required between them.
        if velocity != 0.0 {
            self.lock()
                .set_velocity(&self.axis_name, velocity / self.cpu)?;
        }
        if acceleration != 0.0 {
            self.apply_acceleration(acceleration)?;
        }
        let mut ctrl = self.lock();
        if ctrl.model.is_hexapod() {
            // PIHexapodController::moveCts by m_targetMode, the distance
            // from the polled position. C defect not copied: another mode
            // sends an uninitialized command; rs refuses the move.
            let (cmd, value) = match ctrl.target_mode {
                0 => ("MOV", position),
                1 => ("MRT", position - self.last_position),
                2 => ("MRW", position - self.last_position),
                mode => return Err(pigcs2_err(format!("target mode {mode}"))),
            };
            ctrl.hexapod_move(&format!(
                "{cmd} {} {}",
                self.axis_name,
                CFloat(value / self.cpu)
            ))?;
        } else {
            ctrl.mov(&self.axis_name, position / self.cpu)?;
        }
        drop(ctrl);
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
        self.log_here();
        if !self.lock().accepts_new_target() {
            return Err(self.refuse_target("moveVelocityAxis", ""));
        }
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
        self.log_here();
        // C sets m_isHoming=1 optimistically before attempting the reference
        // command; on failure it is left set until the next poll's real
        // status bitmask corrects it, matching upstream.
        self.is_homing = true;
        self.reference_vel_cts(velocity, forward)
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.log_here();
        let mut ctrl = self.lock();
        ctrl.deferred.remove(&self.axis_no);
        match ctrl.model {
            Model::C885 => ctrl.stop_all(),
            model if model.is_hexapod() => ctrl.halt_platform(),
            _ => ctrl.halt(&self.axis_name),
        }
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.log_here();
        self.lock()
            .set_axis_position(&self.axis_name, position / self.cpu)
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.log_here();
        self.flag_write_flow(
            if enable { "Enabl" } else { "Disabl" },
            "Closed-Loop Control",
        );
        self.set_servo(enable)
    }

    /// C `PIasynController::writeInt32`, `motorDeferMoves_`: clearing a set
    /// deferral runs `processDeferredMoves`, whose status C ignores.
    fn set_deferred_moves(&mut self, _user: &AsynUser, defer: bool) -> AsynResult<()> {
        self.log_here();
        self.flag_write_flow(if defer { "Sett" } else { "Clear" }, "Deferred Move");
        let mut ctrl = self.lock();
        if !defer && ctrl.moves_deferred {
            let _ = ctrl.process_deferred_moves();
        }
        ctrl.moves_deferred = defer;
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let old_homing = self.is_homing;
        let released = self.lock().released.remove(&self.axis_no);
        if let Some(counts) = released {
            self.last_direction = counts > self.last_position;
        }

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
        let model = self.lock().model;
        if !homing || model.can_communicate_while_homing() {
            self.poll_reads(old_homing, servo_control);
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

        // C's strtol over the 4 chars: none hex is 0, a reply ending
        // inside the field reads what is there; only a reply ending before
        // it (C reads past the reply) has no mask.
        assert_eq!(status_mask_at("short", 5), None);
        assert_eq!(status_mask_at("01zzzz", 0), Some(0));
        assert_eq!(status_mask_at("0120zz", 0), Some(0x20));
        assert_eq!(status_mask_at("0120", 0), Some(0x20));
        assert_eq!(status_mask_at("01", 0), Some(0));
    }

    /// Refuses the first write offered with `status`; records every write.
    struct FirstWriteRefused {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        status: AsynStatus,
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
                Err(AsynError::Status {
                    status: self.status,
                    message: "refused".into(),
                })
            } else {
                Ok(data.len())
            }
        }
    }

    /// `PIInterface::sendOnly(char)` (PIInterface.cpp:80-81): a byte not
    /// written is asynError, whatever the write returned.
    #[test]
    fn a_char_not_written_is_an_asyn_error() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = FirstWriteRefused {
            base: PortDriverBase::new("gcs2CharTimeout", 1, PortFlags::default()),
            written: Arc::default(),
            status: AsynStatus::Timeout,
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("gcs2CharTimeout").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl = PIGCS2Controller::new(
            SyncIOHandle::from_handle(
                rt.port_handle().clone(),
                0,
                std::time::Duration::from_secs(1),
            ),
            "gcs2CharTimeout",
        );
        let mut status = None;
        let printed = traced("gcs2CharTimeout", || {
            status = ctrl.send_only_char(24).err().map(|e| e.status());
        });
        assert_eq!(status, Some(AsynStatus::Error));
        assert_eq!(
            messages(&printed),
            ["PIGCSController:sendOnly: error sending command 24, sent=0, status=3\n"]
        );
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
            status: AsynStatus::Error,
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("gcs2SendOnly").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl = PIGCS2Controller::new(
            SyncIOHandle::from_handle(
                rt.port_handle().clone(),
                0,
                std::time::Duration::from_secs(1),
            ),
            "gcs2SendOnly",
        );
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
        let ctrl = PIGCS2Controller::new(
            SyncIOHandle::from_handle(
                rt.port_handle().clone(),
                0,
                std::time::Duration::from_secs(1),
            ),
            "gcs2StatusRefused",
        );
        let mut axis = PIGCS2Axis {
            controller: Arc::new(Mutex::new(ctrl)),
            axis_name: "1".into(),
            axis_no: 0,
            port_name: "gcs2StatusRefused".into(),
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
                "POS? 1", "POS? 1", "TMN? 1", "TMX? 1"
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
            ["\u{7}", "\u{5}", "POS? 1", "POS? 1", "TMN? 1", "TMX? 1"]
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

    /// Axes `1` and `2` of one controller on a line answering `1=1`.
    fn two_axes(name: &str) -> (motor_common::testing::LineRig, PIGCS2Axis, PIGCS2Axis) {
        let (line, io) = motor_common::testing::LineRig::new(name, |last| {
            if last == "ERR?" { "0" } else { "1=1" }.into()
        });
        let ctrl = Arc::new(Mutex::new(PIGCS2Controller::new(io, name)));
        let first = PIGCS2Axis::new(ctrl.clone(), "1".into(), 0, name);
        let second = PIGCS2Axis::new(ctrl, "2".into(), 1, name);
        line.sent();
        (line, first, second)
    }

    /// `PIasynAxis::move` (PIasynAxis.cpp:232-239) only records a move
    /// while moves are deferred; releasing them at any axis runs
    /// `processDeferredMoves` (PIasynController.cpp:236-270): one `MOV` for
    /// every deferred axis, in axis order, and each axis's direction.
    #[test]
    fn deferred_moves_go_out_in_one_mov_on_release() {
        let (line, mut first, mut second) = two_axes("gcs2Defer");
        let user = AsynUser::new(0);
        first.set_deferred_moves(&user, true).unwrap();
        second.move_absolute(&user, -3.0, 0.0, 9.0, 4.0).unwrap();
        first.move_absolute(&user, 5.0, 0.0, 9.0, 4.0).unwrap();
        assert!(sent_lines(&line).is_empty());
        second.set_deferred_moves(&user, false).unwrap();
        assert_eq!(sent_lines(&line), ["MOV 1 5.000000 2 -3.000000", "ERR?"]);
        first.last_position = 0.0;
        second.last_position = 0.0;
        assert!(first.poll(&user).unwrap().direction);
        assert!(!second.poll(&user).unwrap().direction);
        line.sent();
        first.set_deferred_moves(&user, false).unwrap();
        assert!(sent_lines(&line).is_empty());
    }

    /// `ctrl`'s `PIasynController` port `name` with axis `axis` at address
    /// 0, as `PI_GCS2_CreateController` makes it.
    fn controller_port(
        name: &str,
        ctrl: &Arc<Mutex<PIGCS2Controller>>,
        axis: &str,
    ) -> (PortHandle, PiParams) {
        let params = PIGCS2Params::default();
        let port = motor_common::register_controller_port(
            &epics_rs::asyn::registry::open_owner(),
            name,
            1,
            params.clone(),
        )
        .unwrap();
        params.attach(ctrl.clone(), port.clone());
        let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(PIGCS2Axis::new(
            ctrl.clone(),
            axis.into(),
            0,
            name,
        )));
        epics_rs::motor::devices::register_axis(
            epics_rs::motor::builder::MotorBuilder::new(motor)
                .trace_user(motor_common::asyn_user(name, 0))
                .build_axis()
                .axis,
        )
        .unwrap();
        let params = *params.params.get().unwrap();
        (port, params)
    }

    /// `PIHexapodController::ReadPivotSettings` (PIHexapodController.cpp:
    /// 370-425) keeps the init's `SPI?` point, `PIasynController::poll`
    /// posts it as `PI_SUP_RBPIVOT_*` (PIasynController.cpp:583-585), and a
    /// `PI_SUP_PIVOT_*` write sends `SPI` + `ERR?` (:381-392, :338-367)
    /// unless the platform moves.
    #[test]
    fn a_hexapod_pivot_reads_back_and_takes_writes() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let more = AtomicUsize::new(0);
        let (line, ctrl) = hexapod("gcs2PivotLine", move |last| match last {
            // The continuation reads see the command they follow.
            "SPI? RST" => match more.fetch_add(1, Ordering::Relaxed) % 3 {
                0 => "R=1.5 ",
                1 => "S=2.5 ",
                _ => "T=3.5",
            }
            .into(),
            last => hexapod_reply(last),
        });
        let (port, p) = controller_port("gcs2Pivot", &ctrl, "X");
        let read = |reason| port.read_float64_blocking(reason, 0).unwrap();
        ctrl.lock().unwrap().poll();
        assert_eq!(
            [read(p.rbpivot_x), read(p.rbpivot_y), read(p.rbpivot_z)],
            [1.5, 2.5, 3.5]
        );
        line.sent();
        port.write_float64_blocking(p.pivot_y, 0, 4.0).unwrap();
        assert_eq!(sent_lines(&line), ["SPI S 4.000000", "ERR?"]);
        ctrl.lock().unwrap().poll();
        assert_eq!(read(p.rbpivot_y), 4.0);
        ctrl.lock().unwrap().hexapod.any_moving = true;
        line.sent();
        let printed = traced_mask("gcs2Pivot", TraceMask::FLOW, || {
            assert!(port.write_float64_blocking(p.pivot_z, 0, 1.0).is_err());
        });
        assert!(sent_lines(&line).is_empty());
        assert!(
            printed.contains(
                "PIHexapodController::SetPivot() cannot change pivot point while platform is moving"
            ),
            "{printed}"
        );
    }

    /// `PI_CS_TARGETMODE` (PIasynController.cpp:320-324, 587) picks the
    /// hexapod's move: `MOV` to the target, or `MRT`/`MRW` by the distance
    /// from the position (PIHexapodController.cpp:151-165).
    #[test]
    fn a_hexapod_target_mode_picks_mov_mrt_or_mrw() {
        let (line, ctrl) = hexapod("gcs2TModeLine", hexapod_reply);
        let (port, p) = controller_port("gcs2TMode", &ctrl, "X");
        let mut axis = PIGCS2Axis::new(ctrl.clone(), "X".into(), 0, "gcs2TMode");
        let user = AsynUser::new(0);
        let mut moves = Vec::new();
        for mode in [1, 2, 0] {
            port.write_int32_blocking(p.target_mode, 0, mode).unwrap();
            assert_eq!(port.read_int32_blocking(p.target_mode, 0).unwrap(), mode);
            ctrl.lock().unwrap().hexapod.any_moving = false;
            axis.last_position = 5000.0;
            line.sent();
            axis.move_absolute(&user, 15000.0, 0.0, 0.0, 0.0).unwrap();
            moves.push(sent_lines(&line)[0].clone());
        }
        assert_eq!(
            moves,
            ["MRT X 1.000000", "MRW X 1.000000", "MOV X 1.500000"]
        );
        port.write_int32_blocking(p.target_mode, 0, 2).unwrap();
        ctrl.lock().unwrap().poll();
        assert_eq!(port.read_int32_blocking(p.target_mode, 0).unwrap(), 2);
    }

    /// `PIGCSController::SetPivotX` (PIGCSController.cpp:768-776): a
    /// controller with no pivot only logs the write.
    #[test]
    fn a_motor_controller_ignores_a_pivot_write() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2NoPivotLine", |last| {
            if last == "ERR?" { "0" } else { "1=1" }.into()
        });
        let ctrl = Arc::new(Mutex::new(PIGCS2Controller::new(io, "gcs2NoPivotLine")));
        let (port, p) = controller_port("gcs2NoPivot", &ctrl, "1");
        line.sent();
        let printed = traced_mask("gcs2NoPivot", TraceMask::FLOW, || {
            port.write_float64_blocking(p.pivot_x, 0, 4.0).unwrap();
        });
        assert!(sent_lines(&line).is_empty());
        assert!(
            printed.contains("PIGCSController::SetPivotX() ignored"),
            "{printed}"
        );
        ctrl.lock().unwrap().poll();
        assert_eq!(port.read_float64_blocking(p.rbpivot_x, 0).unwrap(), 0.0);
    }

    /// `PIGCSController::getGCSError` keeps the last nonzero `ERR?` code
    /// (PIGCSController.cpp:274-276), `PIasynController::poll` posts it as
    /// `PI_SUP_LAST_ERR` (PIasynController.cpp:612) and a `PI_SUP_CLEAR_ERR`
    /// write zeroes it (:315-319).
    #[test]
    fn the_last_gcs_error_is_posted_and_cleared() {
        use std::sync::atomic::{AtomicI32, Ordering};
        let code = Arc::new(AtomicI32::new(0));
        let err = code.clone();
        let (_line, io) =
            motor_common::testing::LineRig::new("gcs2LastErrLine", move |last| match last {
                "ERR?" => err.load(Ordering::Relaxed).to_string(),
                _ => "1=1".into(),
            });
        let ctrl = Arc::new(Mutex::new(PIGCS2Controller::new(io, "gcs2LastErrLine")));
        let (port, p) = controller_port("gcs2LastErr", &ctrl, "1");
        let poll = || ctrl.lock().unwrap().poll();
        poll();
        assert_eq!(port.read_int32_blocking(p.last_err, 0).unwrap(), 0);
        code.store(5, Ordering::Relaxed);
        assert_eq!(ctrl.lock().unwrap().get_gcs_error().unwrap(), 5);
        code.store(0, Ordering::Relaxed);
        assert_eq!(ctrl.lock().unwrap().get_gcs_error().unwrap(), 0);
        poll();
        assert_eq!(port.read_int32_blocking(p.last_err, 0).unwrap(), 5);
        port.write_int32_blocking(p.clear_err, 0, 1).unwrap();
        assert_eq!(port.read_int32_blocking(p.last_err, 0).unwrap(), 0);
        poll();
        assert_eq!(port.read_int32_blocking(p.last_err, 0).unwrap(), 0);
    }

    /// `PIasynController::writeInt32` (PIasynController.cpp:305-308) logs
    /// each write of the flag on the writer's user, with C's misordered
    /// arguments put back in place.
    #[test]
    fn a_deferred_move_flag_write_is_logged() {
        let (_line, mut first, mut second) = two_axes("gcs2DeferLog");
        let user = AsynUser::new(0);
        let printed = traced_mask("gcs2DeferLog", TraceMask::FLOW, || {
            second.set_deferred_moves(&user, true).unwrap();
            first.set_deferred_moves(&user, false).unwrap();
        });
        assert_eq!(
            messages(&printed),
            [
                "PIasynDriver:writeInt32: Setting Deferred Move flag on driver gcs2DeferLog\n",
                "PIasynDriver:writeInt32: Clearing Deferred Move flag on driver gcs2DeferLog\n",
            ]
        );
    }

    /// C sends whole-count targets (`PIGCSController::moveCts(, int)`); rs
    /// rounds them as the record's `RVAL = NINT(dval/mres)` does, on single
    /// and deferred moves.
    #[test]
    fn move_targets_go_out_as_the_nearest_whole_count() {
        let (line, mut first, mut second) = two_axes("gcs2Nint");
        let user = AsynUser::new(0);
        // MRES 0.1, DVAL 0.3 and -0.3: C truncates to 2 and -2.
        first
            .move_absolute(&user, 0.3 / 0.1, 0.0, 0.0, 0.0)
            .unwrap();
        first
            .move_absolute(&user, -0.3 / 0.1, 0.0, 0.0, 0.0)
            .unwrap();
        // DVAL 0.25 and -0.25: half a count.
        first
            .move_absolute(&user, 0.25 / 0.1, 0.0, 0.0, 0.0)
            .unwrap();
        first
            .move_absolute(&user, -0.25 / 0.1, 0.0, 0.0, 0.0)
            .unwrap();
        assert_eq!(
            sent_lines(&line)
                .into_iter()
                .filter(|l| l.starts_with("MOV"))
                .collect::<Vec<_>>(),
            [
                "MOV 1 3.000000",
                "MOV 1 -3.000000",
                "MOV 1 3.000000",
                "MOV 1 -3.000000"
            ]
        );
        first.set_deferred_moves(&user, true).unwrap();
        first
            .move_absolute(&user, 0.25 / 0.1, 0.0, 0.0, 0.0)
            .unwrap();
        second
            .move_absolute(&user, -0.25 / 0.1, 0.0, 0.0, 0.0)
            .unwrap();
        first.set_deferred_moves(&user, false).unwrap();
        assert_eq!(sent_lines(&line)[0], "MOV 1 3.000000 2 -3.000000");
    }

    /// `PIasynController::writeInt32` (PIasynController.cpp:295-298) logs a
    /// closed-loop write on the writer's user, its misordered arguments put
    /// back in place.
    #[test]
    fn a_closed_loop_write_is_logged() {
        let (_line, mut first, _second) = two_axes("gcs2ServoLog");
        let user = AsynUser::new(0);
        let printed = traced_mask("gcs2ServoLog", TraceMask::FLOW, || {
            let _ = first.set_closed_loop(&user, true);
            let _ = first.set_closed_loop(&user, false);
        });
        let mine: Vec<_> = messages(&printed)
            .into_iter()
            .filter(|m| m.contains("Closed-Loop"))
            .collect();
        assert_eq!(
            mine,
            [
                "PIasynDriver:writeInt32: Enabling Closed-Loop Control flag on driver gcs2ServoLog\n",
                "PIasynDriver:writeInt32: Disabling Closed-Loop Control flag on driver gcs2ServoLog\n",
            ]
        );
    }

    /// `PIasynAxis::stop` (:310-316) drops the axis's deferred move.
    #[test]
    fn a_stop_drops_the_axis_deferred_move() {
        let (line, mut first, mut second) = two_axes("gcs2DeferStop");
        let user = AsynUser::new(0);
        first.set_deferred_moves(&user, true).unwrap();
        first.move_absolute(&user, 5.0, 0.0, 0.0, 0.0).unwrap();
        second.move_absolute(&user, 2.0, 0.0, 0.0, 0.0).unwrap();
        let _ = first.stop(&user, 0.0);
        line.sent();
        first.set_deferred_moves(&user, false).unwrap();
        assert_eq!(sent_lines(&line), ["MOV 2 2.000000", "ERR?"]);
    }

    /// What prints on `port` (every address) while `f` runs.
    fn traced(port: &str, f: impl FnOnce()) -> String {
        traced_mask(port, TraceMask::ERROR, f)
    }

    fn traced_mask(port: &str, mask: TraceMask, f: impl FnOnce()) -> String {
        use epics_rs::asyn::trace::TraceFile;
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), mask);
        trace.set_trace_file(Some(port), TraceFile::File(file.clone()));
        for addr in 0..2 {
            trace.set_device_trace_file(port, addr, TraceFile::File(file.clone()));
        }
        f();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        text
    }

    /// `PIasynAxis::Init` (PIasynAxis.cpp:76-85) and each axis entry point
    /// make the axis's user `PIInterface::m_pCurrentLogSink`, which every
    /// controller print goes to from then on.
    #[test]
    fn controller_prints_go_to_the_last_axis_user() {
        let (_line, io) = motor_common::testing::LineRig::new("gcs2SinkLine", |last| {
            if last == "ERR?" { "5" } else { "1=1" }.into()
        });
        let ctrl = Arc::new(Mutex::new(PIGCS2Controller::new(io, "gcs2SinkLine")));
        let mut axis = PIGCS2Axis::new(ctrl, "1".into(), 1, "gcs2SinkCtl");
        let user = AsynUser::new(0);
        axis.set_deferred_moves(&user, true).unwrap();
        axis.move_absolute(&user, 5.0, 0.0, 0.0, 0.0).unwrap();
        let mut on_line = String::new();
        let on_axis = traced("gcs2SinkCtl", || {
            on_line = traced("gcs2SinkLine", || {
                axis.set_deferred_moves(&user, false).unwrap();
            });
        });
        assert!(
            on_axis.contains("PIGCSController::moveCts(array) failed, GCS error 5\n"),
            "{on_axis}"
        );
        assert!(!on_line.contains("moveCts"), "{on_line}");
    }

    /// `PIGCSController::getGCSError` (PIGCSController.cpp:260-292) prints
    /// a non-zero code and its `TranslatePIError` message, once a log sink
    /// is set.
    #[test]
    fn a_gcs_error_prints_its_code_and_message_on_the_log_sink() {
        let (_line, io) = motor_common::testing::LineRig::new("gcs2ErrLine", |last| {
            if last == "ERR?" { "2" } else { "1=1" }.into()
        });
        let ctrl = Arc::new(Mutex::new(PIGCS2Controller::new(io, "gcs2ErrLine")));
        let before = traced("gcs2ErrLine", || {
            assert_eq!(ctrl.lock().unwrap().get_gcs_error().unwrap(), 2);
        });
        assert_eq!(before, "");
        let mut axis = PIGCS2Axis::new(ctrl, "1".into(), 0, "gcs2ErrCtl");
        let text = traced("gcs2ErrCtl", || {
            let _ = axis.set_closed_loop(&AsynUser::new(0), true);
        });
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.split_once(" PIGCSController::"))
            .map(|(_, rest)| rest)
            .collect();
        assert_eq!(
            lines[..2],
            [
                "getGCSError() GCS error code = 2",
                "getGCSError() GCS error, Unknown command"
            ],
            "{text}"
        );
    }

    #[test]
    fn an_unlisted_gcs_error_is_unknown() {
        assert_eq!(error_message(10), "Controller was stopped by command");
        assert_eq!(error_message(-7), "Timeout error");
        assert_eq!(error_message(123456), "Unknown Error - number 123456");
    }

    /// A GCS 1 hexapod on a line answering `reply`, after
    /// `PIHexapodController::init`.
    fn hexapod(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
    ) -> (motor_common::testing::LineRig, Arc<Mutex<PIGCS2Controller>>) {
        let (line, io) = motor_common::testing::LineRig::new(name, reply);
        let mut ctrl = PIGCS2Controller::new(io, name);
        ctrl.set_model(Model::Hexapod);
        ctrl.init();
        (line, Arc::new(Mutex::new(ctrl)))
    }

    fn hexapod_reply(last: &str) -> String {
        match last {
            "ERR?" | "\u{5}" => "0",
            "\u{4}" => "65536",
            "SPI? RST" => "R=0",
            _ => "X=0.5",
        }
        .into()
    }

    /// C `CreateGCSController` (PIGCSController.cpp:91-103): a hexapod
    /// below `CSV?` 2.0 is a `PIHexapodController`.
    #[test]
    fn a_hexapod_class_follows_its_gcs_version() {
        for (csv, model) in [("1.0", Model::Hexapod), ("2.0", Model::Gcs2Hexapod)] {
            let name = format!("gcs2HexCsv{csv}");
            let (line, io) = motor_common::testing::LineRig::new(&name, move |_| csv.to_string());
            let ctrl = PIGCS2Controller::new(io, &name);
            assert_eq!(ctrl.create_model("PI F-HEXAPOD"), Some(model));
            assert_eq!(sent_lines(&line), ["CSV?"]);
        }
    }

    /// `PIHexapodController::init` (PIHexapodController.cpp:12-46) after the
    /// base's `VEL?`: its fixed axes, the `#4`/`#3` probes and the pivot.
    #[test]
    fn a_hexapod_init_probes_its_status_and_position_bytes_and_reads_the_pivot() {
        let (line, ctrl) = hexapod("gcs2HexInit", hexapod_reply);
        assert_eq!(
            sent_lines(&line),
            ["VEL?", "\u{4}", "\u{3}", "SPI? RST", "ERR?"]
        );
        let ctrl = ctrl.lock().unwrap();
        assert_eq!(ctrl.axis_names(), ["X", "Y", "Z", "U", "V", "W"]);
        assert!(ctrl.hexapod.reads_status && ctrl.hexapod.reads_position);
    }

    /// `PIHexapodController::initAxis` (:126-130) asks no LIM?/TRS?/CST?;
    /// the base `getResolution` reads no CPU, the limits are a fixed ±100
    /// (PIHexapodController.h:34-39) and `#4` bit 0x10000 is referenced
    /// (:132-148).
    #[test]
    fn a_hexapod_axis_init_reads_no_stage() {
        let (line, ctrl) = hexapod("gcs2HexAxis", hexapod_reply);
        line.sent();
        let axis = PIGCS2Axis::new(ctrl, "X".into(), 0, "gcs2HexAxis");
        assert_eq!(
            sent_lines(&line),
            ["SVO X 1", "ERR?", "VEL? X", "POS? X", "\u{4}"]
        );
        assert_eq!(axis.cpu, 10000.0);
        assert_eq!((axis.low_limit, axis.high_limit), (-100.0, 100.0));
        assert_eq!(axis.last_position, 5000.0);
        assert!(axis.homed && axis.servo_on);
    }

    /// `PIHexapodController::getGlobalState` (:49-73), run by the
    /// controller poll, sets every axis moving from one `#5`; while it
    /// does, `getAxisPosition` (:229-284) reads the axis from `#3`.
    #[test]
    fn a_hexapod_reads_the_platform_state_once_for_all_axes() {
        let moving = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let lines = std::sync::atomic::AtomicUsize::new(0);
        let state = moving.clone();
        let (line, ctrl) = hexapod("gcs2HexGlobal", move |last| match last {
            "\u{5}" => if state.load(std::sync::atomic::Ordering::Relaxed) {
                "1"
            } else {
                "0"
            }
            .into(),
            "\u{3}" => match lines.fetch_add(1, std::sync::atomic::Ordering::Relaxed) % 2 {
                0 => "X = 1.0 ",
                _ => "Y = 2.0",
            }
            .into(),
            "POS? Y" => "Y=3.0".into(),
            last => hexapod_reply(last),
        });
        let mut axis = PIGCS2Axis::new(ctrl.clone(), "Y".into(), 1, "gcs2HexGlobal");
        line.sent();
        ctrl.lock().unwrap().poll();
        assert_eq!(sent_lines(&line), ["\u{5}"]);
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        // PIasynAxis::poll reads the position twice (PIasynAxis.cpp:162-164).
        assert_eq!(sent_lines(&line), ["\u{3}", "\u{3}", "\u{3}", "\u{3}"]);
        assert!(status.moving && !status.done && status.powered);
        assert_eq!(status.position, 20000.0);
        moving.store(false, std::sync::atomic::Ordering::Relaxed);
        ctrl.lock().unwrap().poll();
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        assert_eq!(sent_lines(&line), ["\u{5}", "POS? Y", "POS? Y"]);
        assert!(status.done && !status.moving);
        assert_eq!(status.position, 30000.0);
    }

    /// `PIHexapodController::CanCommunicateWhileHoming` is false: a homing
    /// axis polls nothing (PIasynAxis.cpp:126) until `#5` reads the
    /// platform stopped, which ends the homing.
    #[test]
    fn a_homing_hexapod_polls_nothing_until_the_platform_stops() {
        let moving = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let state = moving.clone();
        let (line, ctrl) = hexapod("gcs2HexHoming", move |last| match last {
            "\u{5}" => if state.load(std::sync::atomic::Ordering::Relaxed) {
                "1"
            } else {
                "0"
            }
            .into(),
            last => hexapod_reply(last),
        });
        let mut axis = PIGCS2Axis::new(ctrl.clone(), "X".into(), 0, "gcs2HexHoming");
        ctrl.lock().unwrap().hexapod.homing = true;
        ctrl.lock().unwrap().poll();
        line.sent();
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        assert!(sent_lines(&line).is_empty());
        assert!(axis.is_homing && !status.done);
        moving.store(false, std::sync::atomic::Ordering::Relaxed);
        ctrl.lock().unwrap().poll();
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        assert_eq!(sent_lines(&line), ["\u{5}", "\u{4}", "POS? X", "POS? X"]);
        assert!(!axis.is_homing && status.done && status.homed);
    }

    /// `PIHexapodController::moveCts` (PIHexapodController.cpp:150-195)
    /// waits 0.2 s after `MOV` and reads `#5`; a moving platform needs no
    /// `ERR?`. Its `AcceptsNewTarget` then refuses the next move until `#5`
    /// reads the platform stopped (PIasynAxis.cpp:218-224, 279-285).
    #[test]
    fn a_hexapod_move_reads_the_platform_and_refuses_the_next_while_it_moves() {
        let moving = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let state = moving.clone();
        let (line, ctrl) = hexapod("gcs2HexMove", move |last| match last {
            "\u{5}" => if state.load(std::sync::atomic::Ordering::Relaxed) {
                "1"
            } else {
                "0"
            }
            .into(),
            last => hexapod_reply(last),
        });
        let mut axis = PIGCS2Axis::new(ctrl.clone(), "X".into(), 0, "gcs2HexMove");
        line.sent();
        let user = AsynUser::new(0);
        let started = Instant::now();
        axis.move_absolute(&user, 20000.0, 0.0, 10000.0, 5.0)
            .unwrap();
        assert!(started.elapsed() >= Duration::from_millis(200));
        assert_eq!(
            sent_lines(&line),
            ["VEL X 1.000000", "MOV X 2.000000", "\u{5}"]
        );
        assert!(axis.last_direction);
        let mut refused = (Ok(()), Ok(()));
        let printed = traced("gcs2HexMove", || {
            refused = (
                axis.move_absolute(&user, 0.0, 0.0, 0.0, 0.0),
                axis.move_velocity(&user, 0.0, 1.0, 0.0),
            );
        });
        assert!(refused.0.is_err() && refused.1.is_err());
        assert!(sent_lines(&line).is_empty());
        assert!(
            printed.contains(
                "PIasynAxis:moveAxis: Set port gcs2HexMove, axis 0 - controller does not accept new target (busy?)\n"
            ),
            "{printed}"
        );
        assert!(
            printed.contains(
                "PIasynAxis:moveVelocityAxis: Set port gcs2HexMove, axis 0 - controller does not accept new target (busy?)"
            ),
            "{printed}"
        );
        moving.store(false, std::sync::atomic::Ordering::Relaxed);
        ctrl.lock().unwrap().poll();
        line.sent();
        axis.move_absolute(&user, 0.0, 0.0, 0.0, 0.0).unwrap();
        assert_eq!(sent_lines(&line), ["MOV X 0.000000", "\u{5}", "ERR?"]);
    }

    /// A hexapod move `#5` does not see start is an error when `ERR?` says
    /// so (PIHexapodController.cpp:180-189).
    #[test]
    fn a_hexapod_move_that_does_not_start_prints_its_gcs_error() {
        let (line, ctrl) = hexapod("gcs2HexMoveErr", |last| match last {
            "ERR?" => "7".into(),
            last => hexapod_reply(last),
        });
        let mut axis = PIGCS2Axis::new(ctrl, "X".into(), 0, "gcs2HexMoveErr");
        line.sent();
        let mut moved = Ok(());
        let printed = traced("gcs2HexMoveErr", || {
            moved = axis.move_absolute(&AsynUser::new(0), 10000.0, 0.0, 0.0, 0.0);
        });
        assert!(moved.is_err());
        assert_eq!(sent_lines(&line), ["MOV X 1.000000", "\u{5}", "ERR?"]);
        assert!(
            printed.contains("PIHexapodController::moveCts() failed, GCS error 7\n"),
            "{printed}"
        );
    }

    /// `processDeferredMoves` on a hexapod runs its `moveCts(array)`
    /// (PIHexapodController.cpp:197-227): one `MOV`, the wait and `#5`.
    #[test]
    fn hexapod_deferred_moves_go_out_in_one_mov_then_read_the_platform() {
        let (line, ctrl) = hexapod("gcs2HexDefer", |last| match last {
            "\u{5}" => "1".into(),
            last => hexapod_reply(last),
        });
        let mut x = PIGCS2Axis::new(ctrl.clone(), "X".into(), 0, "gcs2HexDefer");
        let mut y = PIGCS2Axis::new(ctrl.clone(), "Y".into(), 1, "gcs2HexDefer");
        line.sent();
        let user = AsynUser::new(0);
        x.set_deferred_moves(&user, true).unwrap();
        y.move_absolute(&user, 20000.0, 0.0, 0.0, 0.0).unwrap();
        x.move_absolute(&user, 10000.0, 0.0, 0.0, 0.0).unwrap();
        assert!(sent_lines(&line).is_empty());
        y.set_deferred_moves(&user, false).unwrap();
        assert_eq!(sent_lines(&line), ["MOV X 1.000000 Y 2.000000", "\u{5}"]);
        assert!(ctrl.lock().unwrap().hexapod.any_moving);
    }

    /// The message of each trace line `printed` holds, less its
    /// `yyyy/mm/dd hh:mm:ss.mmm ` stamp; a message without a newline runs
    /// into the next stamp.
    fn messages(printed: &str) -> Vec<String> {
        const STAMP: &[u8] = b"dddd/dd/dd dd:dd:dd.ddd ";
        let is_stamp = |at: &[u8]| {
            at.len() >= STAMP.len()
                && STAMP.iter().zip(at).all(|(want, got)| match want {
                    b'd' => got.is_ascii_digit(),
                    want => want == got,
                })
        };
        let bytes = printed.as_bytes();
        let starts: Vec<usize> = (0..bytes.len())
            .filter(|&i| is_stamp(&bytes[i..]))
            .collect();
        starts
            .iter()
            .zip(starts.iter().skip(1).chain([&bytes.len()]))
            .map(|(&from, &to)| printed[from + STAMP.len()..to].to_string())
            .collect()
    }

    /// Each failed `ERR?` check prints C's line (PIGCSController.cpp:224,
    /// 256, 310, 537-538, 558, 586; PIGCSMotorController.cpp:67-68,
    /// 108-109, 126-127; PIC885Controller.cpp:87-88, 135-136, 150-151).
    #[test]
    fn a_failed_command_prints_its_c_failure_line() {
        let (_line, io) = motor_common::testing::LineRig::new("gcs2FailPrint", |_| "1=5".into());
        let mut ctrl = PIGCS2Controller::new(io, "gcs2FailPrint");
        let printed = traced("gcs2FailPrint", || {
            assert!(ctrl.mov("1", 1.0).is_err());
            assert!(ctrl.set_axis_position("1", 1.0).is_err());
            assert!(ctrl.halt("1").is_err());
            assert!(ctrl.set_enable_axis("1", 1).is_err());
            assert!(ctrl.get_enable_axis("1").is_err());
            assert!(ctrl.set_servo("1", true).is_err());
            assert!(ctrl.set_reference_velocity("1", 1.0).is_err());
            assert!(ctrl.reference_axis("1", true, true, false).is_err());
            assert!(ctrl.reference_axis("1", true, false, false).is_err());
            ctrl.set_model(Model::C885);
            assert!(ctrl.stop_all().is_err());
            assert!(ctrl.reference_axis("1", true, true, false).is_err());
            assert!(ctrl.reference_axis("1", true, false, false).is_err());
        });
        assert_eq!(
            messages(&printed),
            [
                "PIGCSController::move() failed, GCS error 1\n",
                "PIGCSController::setAxisPosition() failed, GCS error 1\n",
                "PIGCSController::haltAxis() failed, GCS error 1",
                "PIGCSController::setEnableAxis() failed, GCS error 1\n",
                "PIGCSController::getEnableAxis() failed, GCS error 1\n",
                "Could not set servo state!\n",
                "PIGCSMotorController::referenceVelCts() failed\n",
                "PIGCSMotorController::referenceVelCts() failed\n",
                "PIGCSMotorController::referenceVelCts() failed - axis has no reference/limit switch\n",
                "PIC885Controller::haltAxis() failed, GCS error 1",
                "PIC885Controller::referenceVelCts() failed\n",
                "PIC885Controller::referenceVelCts() failed - axis has no reference/limit switch\n",
            ]
        );
    }

    /// `PIC702Controller::referenceVelCts` (PIC702Controller.cpp:178-185)
    /// prints when the axis has neither switch.
    #[test]
    fn a_c702_reference_without_a_switch_prints_its_c_line() {
        let (_line, mut axis) = c702_axis("gcs2C702NoSwitch", true);
        axis.has_reference = false;
        axis.has_limit_switches = false;
        let printed = traced("gcs2C702NoSwitch", || {
            assert!(axis.reference_c702(1.0, true).is_err());
        });
        assert_eq!(
            messages(&printed),
            ["PIC702Controller::referenceVelCts() failed - axis has no reference/limit switch\n"]
        );
    }

    /// `PIHexapodController::referenceVelCts` (PIHexapodController.cpp:
    /// 370-385) sends `INI X` for any axis and homes the platform until `#5`
    /// reads it stopped; its `haltAxis` (:387-403) is byte 24, then `ERR?`
    /// reading `PI_CNTR_STOP`.
    #[test]
    fn a_hexapod_homes_with_ini_x_and_stops_with_byte_24() {
        let (line, ctrl) = hexapod("gcs2HexIni", |last| match last {
            "\u{5}" => "1".into(),
            "ERR?" => "10".into(),
            last => hexapod_reply(last),
        });
        let mut axis = PIGCS2Axis::new(ctrl.clone(), "Y".into(), 1, "gcs2HexIni");
        line.sent();
        let user = AsynUser::new(0);
        axis.home(&user, 0.0, 1.0, 0.0, true).unwrap();
        assert_eq!(sent_lines(&line), ["INI X"]);
        ctrl.lock().unwrap().poll();
        assert!(!axis.poll(&user).unwrap().done && axis.is_homing);
        line.sent();
        axis.stop(&user, 0.0).unwrap();
        assert_eq!(sent_lines(&line), ["\u{18}", "ERR?"]);
    }

    /// A GCS 2 hexapod on a line answering `reply`, after its init, and
    /// its axis `name` number `axis_no`.
    fn gcs2_hexapod(
        port: &str,
        name: &str,
        axis_no: usize,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
    ) -> (
        motor_common::testing::LineRig,
        Arc<Mutex<PIGCS2Controller>>,
        PIGCS2Axis,
    ) {
        let (line, io) = motor_common::testing::LineRig::new(port, reply);
        let mut ctrl = PIGCS2Controller::new(io, port);
        ctrl.set_model(Model::Gcs2Hexapod);
        ctrl.init();
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axis = PIGCS2Axis::new(ctrl.clone(), name.into(), axis_no, port);
        (line, ctrl, axis)
    }

    /// `PIGCS2_HexapodController` (PIGCS2_HexapodController.h:20-44)
    /// keeps the GCS 1 init and axis init but reads the pivot with
    /// `SPI? R S T` and the velocity with `VLS?`.
    #[test]
    fn a_gcs2_hexapod_inits_like_gcs1_with_spi_r_s_t_and_vls() {
        let (line, ctrl, axis) = gcs2_hexapod("gcs2Hex2Init", "X", 0, hexapod_reply);
        assert_eq!(
            sent_lines(&line),
            [
                "VEL?",
                "\u{4}",
                "\u{3}",
                "SPI? R S T",
                "ERR?",
                "SVO X 1",
                "ERR?",
                "VLS?",
                "POS? X",
                "\u{4}"
            ]
        );
        assert_eq!(
            ctrl.lock().unwrap().axis_names(),
            ["X", "Y", "Z", "U", "V", "W"]
        );
        assert_eq!((axis.low_limit, axis.high_limit), (-100.0, 100.0));
        assert!(axis.homed && axis.servo_on);
    }

    /// `PIGCS2_HexapodController` accepts a target while the platform
    /// moves, sets the velocity with `VLS` (PIGCS2_HexapodController.cpp:
    /// 31-43) and reads positions with `POS?` throughout.
    #[test]
    fn a_gcs2_hexapod_moves_while_moving_with_vls_and_reads_pos() {
        let (line, ctrl, mut axis) = gcs2_hexapod("gcs2Hex2Move", "Y", 1, |last| match last {
            "\u{5}" => "1".into(),
            "POS? Y" => "Y=2.0".into(),
            last => hexapod_reply(last),
        });
        let user = AsynUser::new(0);
        ctrl.lock().unwrap().poll();
        line.sent();
        axis.move_absolute(&user, 20000.0, 0.0, -10000.0, 5.0)
            .unwrap();
        assert_eq!(
            sent_lines(&line),
            ["VLS 1.000000", "MOV Y 2.000000", "\u{5}"]
        );
        let status = axis.poll(&user).unwrap();
        assert_eq!(sent_lines(&line), ["POS? Y", "POS? Y"]);
        assert!(status.moving && status.position == 20000.0);
        axis.move_velocity(&user, 0.0, 10000.0, 0.0).unwrap();
        assert_eq!(
            sent_lines(&line),
            ["VLS 1.000000", "MOV Y 100.000000", "ERR?"]
        );
    }

    /// `PIGCS2_HexapodController::referenceVelCts` (:46-71): servo on,
    /// then `FRF` of the axis, and no platform homing state.
    #[test]
    fn a_gcs2_hexapod_homes_its_axis_with_frf() {
        let (line, ctrl, mut axis) = gcs2_hexapod("gcs2Hex2Home", "Z", 2, hexapod_reply);
        line.sent();
        axis.home(&AsynUser::new(0), 0.0, 1.0, 0.0, false).unwrap();
        assert_eq!(sent_lines(&line), ["SVO Z 1", "ERR?", "FRF Z", "ERR?"]);
        assert!(!ctrl.lock().unwrap().hexapod.homing);
    }

    fn c887_reply(last: &str) -> String {
        match last {
            "\u{5}" => "1",
            "SVO? Y" => "Y=0",
            "TRA? Y -1" => "Y=-12.5",
            "TRA? Y 1" => "Y=12.5",
            last => return hexapod_reply(last),
        }
        .into()
    }

    /// `PICoordinateSystem` (PICoordinateSystem.cpp) is the GCS 2 hexapod
    /// with `TRA?` limits (:150-185); its controller poll asks `KEN?` and
    /// `KLN?` after `#5` (PIasynController.cpp:590-596).
    #[test]
    fn a_c887_reads_tra_limits_and_polls_ken_and_kln() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2C887", c887_reply);
        let ctrl = PIGCS2Controller::new(io, "gcs2C887");
        let Some(model) = ctrl.create_model("PI C-887.52") else {
            panic!("no model")
        };
        let mut ctrl = ctrl;
        ctrl.set_model(model);
        ctrl.init();
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axis = PIGCS2Axis::new(ctrl.clone(), "Y".into(), 1, "gcs2C887");
        assert_eq!(
            sent_lines(&line),
            [
                "VEL?",
                "\u{4}",
                "\u{3}",
                "SPI? R S T",
                "ERR?",
                "SVO Y 1",
                "ERR?",
                "VLS?",
                "POS? Y",
                "TRA? Y -1",
                "TRA? Y 1",
                "\u{4}"
            ]
        );
        assert_eq!((axis.low_limit, axis.high_limit), (-12.5, 12.5));
        ctrl.lock().unwrap().poll();
        assert_eq!(sent_lines(&line), ["\u{5}", "KEN?", "KLN?"]);
    }

    /// `PICoordinateSystem::getStatus` (:5-29): `#5` against the mask
    /// `PIHexapodController::initAxis` gives every axis (1), `SVO?`, and
    /// no homing or limits.
    #[test]
    fn a_c887_axis_polls_moving_bit_0_and_its_servo() {
        let (line, io) = motor_common::testing::LineRig::new("gcs2C887Poll", c887_reply);
        let mut ctrl = PIGCS2Controller::new(io, "gcs2C887Poll");
        ctrl.set_model(Model::CoordinateSystem);
        ctrl.init();
        let mut axis = PIGCS2Axis::new(Arc::new(Mutex::new(ctrl)), "Y".into(), 1, "gcs2C887Poll");
        line.sent();
        let status = axis.poll(&AsynUser::new(0)).unwrap();
        assert_eq!(
            sent_lines(&line),
            [
                "\u{5}",
                "SVO? Y",
                "POS? Y",
                "POS? Y",
                "TRA? Y -1",
                "TRA? Y 1"
            ]
        );
        assert!(status.moving && !axis.is_homing && status.problem && !status.powered);
    }

    /// `PIInterface::sendAndReceive` (PIInterface.cpp:133-141) reads, with
    /// no further write, each line that follows one ending in a space, and
    /// joins them with `\n`.
    #[test]
    fn a_query_reads_the_lines_that_follow_one_ending_in_a_space() {
        let lines = std::sync::atomic::AtomicUsize::new(0);
        let (line, io) = motor_common::testing::LineRig::new("gcs2QueryMore", move |_| {
            match lines.fetch_add(1, std::sync::atomic::Ordering::Relaxed) {
                0 => "R=1 ",
                1 => "S=2 ",
                _ => "T=3",
            }
            .into()
        });
        let ctrl = PIGCS2Controller::new(io, "gcs2QueryMore");
        assert_eq!(ctrl.query("SPI? RST").unwrap(), "R=1 \nS=2 \nT=3");
        assert_eq!(line.sent(), ["SPI? RST", "\n"]);
    }

    /// `PIInterface::sendAndReceive(char)` (PIInterface.cpp:219-227)
    /// writes the byte again for each line that ends in a space; rs joins
    /// the lines with `\n`.
    #[test]
    fn a_char_query_resends_its_byte_while_a_line_ends_in_a_space() {
        let lines = std::sync::atomic::AtomicUsize::new(0);
        let (line, io) = motor_common::testing::LineRig::new("gcs2CharMore", move |_| {
            match lines.fetch_add(1, std::sync::atomic::Ordering::Relaxed) {
                0 => "X=1.5 ",
                1 => "Y=2.5 ",
                _ => "Z=3.5",
            }
            .into()
        });
        let ctrl = PIGCS2Controller::new(io, "gcs2CharMore");
        assert_eq!(ctrl.query_char(3).unwrap(), "X=1.5 \nY=2.5 \nZ=3.5");
        assert_eq!(line.sent(), ["\u{3}", "\u{3}", "\u{3}"]);
    }

    /// C `getMoving` (PIGCSController.cpp:591-605) reads `#5` with
    /// `strtol(buf, &pStr, 16)`, which takes a `0x` prefix.
    #[test]
    fn a_moving_word_may_carry_0x() {
        let (_line, io) = motor_common::testing::LineRig::new("gcs2Moving0x", |_| "0x2".into());
        let ctrl = PIGCS2Controller::new(io, "gcs2Moving0x");
        assert!(ctrl.get_moving(2).unwrap());
        assert!(!ctrl.get_moving(1).unwrap());
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
        assert_eq!(Model::from_idn("PI M-824"), Some(Model::Hexapod));
        assert_eq!(Model::from_idn("PI F-HEXAPOD"), Some(Model::Hexapod));
        assert_eq!(
            Model::from_idn("PI C-887.52"),
            Some(Model::CoordinateSystem)
        );
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
