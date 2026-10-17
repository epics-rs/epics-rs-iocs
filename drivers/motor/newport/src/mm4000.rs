//! Newport MM4000/MM4005/MM4006 motor controller driver (serial/GPIB ASCII).
//!
//! Ported from `motorNewport/newportApp/src/drvMM4000Asyn.c` (the model-3
//! asyn driver, the maintained reference). Commands are `{axis}CC{value}`
//! with a 1-based unpadded axis prefix; values are formatted with the
//! per-axis precision derived from the `TU` drive resolution (C
//! `maxDigits`). Commands go out bare and the port EOS `"\r"` both ways comes
//! from the startup script, as in C's Newport_MM4000.iocsh:23-24 (the C
//! driver sets no EOS).
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
//! `MM4000Config` (model 1) instead runs drvMM4000.cc's cycle: `start_status`
//! (`MS;`, `TP;` when it answered, `TH;`) once per TTL, and `set_status` per
//! axis, which reads the position from `TH;`, the encoder from `TP;` and asks
//! `TE;` itself.
//!
//! ## Deviations from C (documented)
//!
//! - drvMM4000Asyn.c queries `TE;` once per axis inside its poll loop even
//!   though the answer is controller-global (and a `TE` comm failure would
//!   mark the *next* axis in error); the asyn port queries it once per poll
//!   cycle.
//! - C's MM4005 torque-off path (`MF` then busy-wait until the `MS` power
//!   bit reads ON) loops forever if the bit never clears; this port bounds
//!   the wait at 50 tries (5 s) and errors.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus, OctetStep};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::util::{atof, atoi, max_digits};

/// Response buffer size for a single controller reply (C `BUFFER_SIZE` 160).
const READ_BUF: usize = 256;

/// C status-byte masks (same layout as the MM3000).
const MM4000_MOVING: u8 = 0x01;
const MM4000_POWER_OFF: u8 = 0x02;
const MM4000_DIRECTION: u8 = 0x04;
const MM4000_HIGH_LIMIT: u8 = 0x08;
const MM4000_LOW_LIMIT: u8 = 0x10;
const MM4000_HOME: u8 = 0x20;

/// Bounded replacement for C's unbounded MM4005 torque-off busy-wait.
const TORQUE_OFF_MAX_TRIES: usize = 50;

/// `asynPrint(pasynUser, ASYN_TRACE_ERROR, ...)` on the user connected to
/// `(port, addr)`.
fn print_error(port: &str, addr: i32, args: std::fmt::Arguments<'_>) {
    PortManager::global()
        .trace_manager()
        .print(Some(port), Some(addr), 0, TraceMask::ERROR, args);
}

/// C `sendOnly`/`sendAndReceive` turn an exchange that did not write the
/// whole command into `asynError`, whatever the port said: a refused queue
/// prints and polls as status 3, not 4.
fn c_exchange_error(e: AsynError, written: usize, requested: usize) -> AsynError {
    if written == requested {
        return e;
    }
    mm_err(e.message())
}

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

/// C's axis count: the `epicsStrtok_r(.., ",", ..)` token count, which
/// skips empty fields (an empty reply has none).
fn strtok_count(reply: &str) -> usize {
    reply.split(',').filter(|t| !t.is_empty()).count()
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
    /// The octet port C's `pController->pasynUser` prints its exchange
    /// errors on.
    serial_port: String,
    /// The motor port whose axis users the poller prints on
    /// (drvMotorAsyn.c:413).
    motor_port: String,
    firmware: String,
    model: Mm4000Model,
    num_axes: usize,
    cache: Mm4000PollData,
    fetched_at: Option<Instant>,
    cache_ttl: Duration,
    driver: Mm4000Driver,
}

/// Which C driver this controller follows. Only drvMM4000Asyn.c prints its
/// exchange and poller errors; drvMM4000.cc's send_mess/recv_mess are silent.
#[derive(Clone, Debug)]
enum Mm4000Driver {
    Asyn,
    Model1(Model1Poll),
}

/// drvMM4000.cc's `MMcontroller` strings and `CommStatus`, which
/// `start_status` fills and `set_status` reads. A failed read is empty.
#[derive(Clone, Debug, Default)]
struct Model1Poll {
    comm: CommStatus,
    /// `MS;` reply.
    status: String,
    /// `TH;` reply.
    position: String,
    /// `TP;` reply.
    feedback: String,
}

/// C `CommStatus`: one unanswered `MS;` is a RETRY, which leaves every axis
/// as it was; a second is COMM_ERR.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum CommStatus {
    #[default]
    Normal,
    Retry,
    CommErr,
}

/// Why [`Mm4000Controller::new`] refused a controller.
#[derive(Debug)]
pub enum Mm4000Refusal {
    /// `VE;` (asyn) or `TH;` (model 1) failed three times.
    NoAnswer,
    /// The firmware string (past `VE`) names no MM4000/4005/4006.
    InvalidModel(String),
    /// `TP;` reported fewer fields than the axes asked for.
    TooFewAxes { total_axes: usize },
}

impl Mm4000Controller {
    /// Connect and identify an MM4000/4005/4006 (C `MM4000AsynConfig`):
    /// `VE;` up to 3 tries → firmware/model, then `TP;` — the number of
    /// comma-separated fields is the axis count on the wire, which must
    /// cover `num_axes`. The refusal is returned for the caller to print.
    /// Performs blocking serial I/O.
    pub fn new(
        handle: SyncIOHandle,
        serial_port: &str,
        motor_port: &str,
        num_axes: usize,
        cache_ttl: Duration,
    ) -> Result<Self, Mm4000Refusal> {
        let mut ctrl = Self {
            handle,
            serial_port: serial_port.to_owned(),
            motor_port: motor_port.to_owned(),
            firmware: String::new(),
            model: Mm4000Model::Mm4000,
            num_axes,
            cache: Mm4000PollData::default(),
            fetched_at: None,
            cache_ttl,
            driver: Mm4000Driver::Asyn,
        };
        let ve = (0..3)
            .find_map(|_| ctrl.write_read("VE;").ok())
            .ok_or(Mm4000Refusal::NoAnswer)?;
        ctrl.firmware = ve.get(2..).unwrap_or("").to_string(); // skip "VE"
        let Some(model) = detect_model(&ctrl.firmware) else {
            return Err(Mm4000Refusal::InvalidModel(ctrl.firmware));
        };
        ctrl.model = model;

        // C ignores the `TP;` status; a failed read leaves an empty buffer.
        let tp = ctrl.write_read("TP;").unwrap_or_default();
        let total_axes = strtok_count(&tp);
        if total_axes < num_axes {
            return Err(Mm4000Refusal::TooFewAxes { total_axes });
        }
        Ok(ctrl)
    }

    /// drvMM4000.cc `motor_init` for one card, up to its per-axis block: `TH;`
    /// until it answers (3 tries), `ST;`, `VE;` for the model, `TH;` again
    /// for the axis count, then `start_status` (`MS;`, and `TH;` + `TP;` only
    /// when `MS;` answered). C ignores every status here; a failed read is
    /// an empty buffer. Performs blocking serial I/O.
    pub fn new_model1(
        handle: SyncIOHandle,
        serial_port: &str,
        cache_ttl: Duration,
    ) -> Result<Self, Mm4000Refusal> {
        let mut ctrl = Self {
            handle,
            serial_port: serial_port.to_owned(),
            motor_port: serial_port.to_owned(),
            firmware: String::new(),
            model: Mm4000Model::Mm4000,
            num_axes: 0,
            cache: Mm4000PollData::default(),
            fetched_at: None,
            cache_ttl,
            driver: Mm4000Driver::Model1(Model1Poll::default()),
        };
        let read = |ctrl: &Self, cmd: &str| ctrl.write_read(cmd).unwrap_or_default();
        if !(0..3).any(|_| !read(&ctrl, "TH;").is_empty()) {
            return Err(Mm4000Refusal::NoAnswer);
        }
        let _ = ctrl.write("ST;");
        let ve = read(&ctrl, "VE;");
        ctrl.firmware = ve.get(2..).unwrap_or("").to_string(); // skip "VE"
        let Some(model) = detect_model(&ctrl.firmware) else {
            return Err(Mm4000Refusal::InvalidModel(ctrl.firmware));
        };
        ctrl.model = model;
        ctrl.num_axes = strtok_count(&read(&ctrl, "TH;"));
        ctrl.start_status(false);
        Ok(ctrl)
    }

    /// drvMM4000.cc `start_status`. `start_status(card)` at init asks `TH;`
    /// and `TP;` only after `MS;` answered; the poll cycle (`card == -1`)
    /// asks `TP;` after an answered `MS;` and `TH;` always. A string not
    /// re-read keeps its last value, as C's buffers do.
    fn start_status(&mut self, cycle: bool) {
        let Mm4000Driver::Model1(last) = &self.driver else {
            return;
        };
        let mut poll = last.clone();
        let read = |cmd: &str| self.write_read(cmd).unwrap_or_default();
        poll.status = read("MS;");
        if !poll.status.is_empty() {
            poll.comm = CommStatus::Normal;
            if !cycle {
                poll.position = read("TH;");
            }
            poll.feedback = read("TP;");
        } else if poll.comm == CommStatus::Normal {
            poll.comm = CommStatus::Retry;
        } else {
            poll.comm = CommStatus::CommErr;
        }
        if cycle {
            poll.position = read("TH;");
        }
        self.driver = Mm4000Driver::Model1(poll);
    }

    /// The model-1 strings for this poll cycle, running `start_status` at
    /// most once per TTL; `None` on the asyn driver.
    fn model1_poll(&mut self) -> Option<Model1Poll> {
        if !matches!(self.driver, Mm4000Driver::Model1(_)) {
            return None;
        }
        if !self
            .fetched_at
            .is_some_and(|at| at.elapsed() < self.cache_ttl)
        {
            self.start_status(true);
            self.fetched_at = Some(Instant::now());
        }
        match &self.driver {
            Mm4000Driver::Model1(poll) => Some(poll.clone()),
            Mm4000Driver::Asyn => None,
        }
    }

    /// drvMM4000.cc `set_status` for axis `idx` (0-based) from `last`. The
    /// readback delay re-poll is skipped: `drvMM4000ReadbackDelay` is 0 and
    /// has no setter.
    fn set_status(
        &self,
        poll: &Model1Poll,
        idx: usize,
        encoder_present: bool,
        last: &MotorStatus,
    ) -> MotorStatus {
        let mut s = last.clone();
        match poll.comm {
            CommStatus::CommErr => {
                s.comms_error = true;
                s.problem = true;
                return s;
            }
            CommStatus::Retry => return s,
            CommStatus::Normal => s.comms_error = false,
        }
        // C reads the byte past a short reply out of its zeroed buffer.
        let byte = parse_axis_status(&poll.status, idx).unwrap_or(0);
        s.direction = byte & MM4000_DIRECTION != 0;
        s.done = byte & MM4000_MOVING == 0;
        s.moving = !s.done;
        s.high_limit = byte & MM4000_HIGH_LIMIT != 0;
        s.low_limit = byte & MM4000_LOW_LIMIT != 0;
        s.home = byte & MM4000_HOME != 0;
        s.powered = byte & MM4000_POWER_OFF == 0;
        // A field C's strtok does not find is a NULL dereference there; the
        // readback keeps its last value instead.
        if let Some(position) = parse_axis_position(&poll.position, idx) {
            s.position = position;
        }
        s.encoder_position = match encoder_present {
            true => parse_axis_position(&poll.feedback, idx).unwrap_or(s.encoder_position),
            false => 0.0,
        };
        s.velocity = 0.0;
        s.problem = !te_ok(&self.write_read("TE;").unwrap_or_default());
        s
    }

    /// `asynPrint(.., ASYN_TRACE_ERROR, ..)`, which only drvMM4000Asyn.c has.
    fn print_error(&self, port: &str, addr: i32, args: std::fmt::Arguments<'_>) {
        if matches!(self.driver, Mm4000Driver::Asyn) {
            print_error(port, addr, args);
        }
    }

    /// The axes this controller serves.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Firmware version from `VE;` (the `VE` prefix stripped).
    pub fn firmware(&self) -> &str {
        &self.firmware
    }

    /// Detected model.
    pub fn model(&self) -> Mm4000Model {
        self.model
    }

    /// Write a command (C `sendOnly`); the port output EOS terminates it.
    fn write(&self, cmd: &str) -> AsynResult<()> {
        let (sent, e) = match self.handle.write_octet(0, cmd.as_bytes()) {
            Ok(n) if n == cmd.len() => return Ok(()),
            Ok(n) => (n, mm_err("short write".into())),
            Err(e) => (e.partial_write().unwrap_or(0), e),
        };
        let e = c_exchange_error(e, sent, cmd.len());
        self.print_error(
            &self.serial_port,
            0,
            format_args!(
                "drvMM4000Asyn:sendOnly: error sending command {cmd}, sent={sent}, status={}",
                e.status() as i32
            ),
        );
        Err(e)
    }

    /// Write a command and read the reply (C `sendAndReceive`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        let e = match self.handle.write_read(0, cmd.as_bytes(), READ_BUF) {
            Ok(raw) => return Ok(String::from_utf8_lossy(&raw).into_owned()),
            Err(e) => e,
        };
        let written = match e.octet_step() {
            Some(OctetStep::Read) => cmd.len(),
            _ => e.partial_write().unwrap_or(0),
        };
        let e = c_exchange_error(e, written, cmd.len());
        self.print_error(
            &self.serial_port,
            0,
            format_args!(
                "drvMM4000Asyn:sendAndReceive error calling writeRead, output={cmd} status={}, error={}",
                e.status() as i32,
                e.message()
            ),
        );
        Err(e)
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
        // C sends `TP;` only after `MS;` succeeded.
        match self
            .write_read("MS;")
            .and_then(|ms| Ok((ms, self.write_read("TP;")?)))
        {
            Ok((status_all, position_all)) => {
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
                            for axis in 0..self.num_axes {
                                self.print_error(
                                    &self.motor_port,
                                    axis as i32,
                                    format_args!("MM4000Poller: controller error {te}"),
                                );
                            }
                            data.problem = true;
                        }
                    }
                    Err(_) => data.comms_error = true,
                }
            }
            // C MM4000Poller sets only motorAxisCommError; every other
            // parameter keeps the last value read.
            Err(e) => {
                for axis in 0..self.num_axes {
                    self.print_error(
                        &self.motor_port,
                        axis as i32,
                        format_args!("MM4000Poller: error reading status={}", e.status() as i32),
                    );
                }
                data = Mm4000PollData {
                    comms_error: true,
                    ..self.cache.clone()
                }
            }
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
    /// What the C driver that created the axis posts on a failed poll.
    poll_failure: PollFailure,
    /// drvMM4000.cc `encoder_present`: `TC` reports a closed loop.
    encoder_present: bool,
    /// The status drvMM4000.cc's `set_status` starts from and RETRY keeps.
    last: MotorStatus,
}

/// C model-1 `set_status`'s COMM_ERR branch raises `CNTRL_COMM_ERR` and
/// `RA_PROBLEM` (drvMM4000.cc; its first failure's silent RETRY is not
/// modelled); model-2 `MM4000Poller` only `motorAxisCommError`
/// (drvMM4000Asyn.c).
pub const MODEL1_POLL_FAILURE: PollFailure = PollFailure::PROBLEM_AND_COMMS_ERROR;
pub const MODEL2_POLL_FAILURE: PollFailure = PollFailure::COMMS_ERROR;

impl Mm4000Axis {
    /// Construct axis `axis` (1-based), running the C per-axis init queries:
    /// `TC` (closed-loop state — drvMM4000.cc's encoder flag, which
    /// drvMM4000Asyn.c stores unused),
    /// `TU` (drive resolution → command precision), `XH` (home preset),
    /// `TL`/`TR` (travel limits). C checks none of these replies, so a
    /// failed one reads as an empty buffer (`atof` → 0). Performs blocking
    /// serial I/O under the controller lock.
    pub fn new(
        controller: Arc<Mutex<Mm4000Controller>>,
        axis: usize,
        poll_failure: PollFailure,
    ) -> Self {
        let (model, digits, home_preset, low_limit, high_limit, encoder_present) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // C `&inputBuff[3]`: skip the echoed `nCC` prefix.
            let query = |cc: &str| {
                let reply = ctrl.write_read(&format!("{axis}{cc}")).unwrap_or_default();
                reply.get(3..).unwrap_or("").to_owned()
            };
            let encoder_present = atoi(&query("TC")) != 0;
            let step_size = atof(&query("TU"));
            let home_preset = atof(&query("XH"));
            let low_limit = atof(&query("TL"));
            let high_limit = atof(&query("TR"));
            (
                ctrl.model(),
                max_digits(step_size),
                home_preset,
                low_limit,
                high_limit,
                encoder_present,
            )
        };
        Self {
            controller,
            axis,
            model,
            digits,
            home_preset,
            low_limit,
            high_limit,
            poll_failure,
            encoder_present,
            // drvMM4000.cc motor_init clears the status and sets
            // GAIN_SUPPORT, and EA_PRESENT on a closed loop.
            last: MotorStatus {
                gain_support: true,
                has_encoder: encoder_present,
                vbas_supported: false,
                ..MotorStatus::default()
            },
        }
    }

    /// The `set_status` that ends drvMM4000.cc's per-axis init, on the
    /// strings init's `start_status` left. No-op on the asyn driver.
    pub fn model1_set_status(&mut self) {
        let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Mm4000Driver::Model1(poll) = &ctrl.driver {
            self.last = ctrl.set_status(poll, self.axis - 1, self.encoder_present, &self.last);
        }
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
    fn poll_failure(&self) -> PollFailure {
        self.poll_failure
    }

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
        let mut ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(poll) = ctrl.model1_poll() {
            self.last = ctrl.set_status(&poll, self.axis - 1, self.encoder_present, &self.last);
            return Ok(self.last.clone());
        }
        let data = ctrl.poll_data();
        drop(ctrl);
        let idx = self.axis - 1;
        let byte = data
            .status_bytes
            .get(idx)
            .copied()
            .flatten()
            .ok_or_else(|| {
                mm_err(format!(
                    "MM4000 axis {}: missing MS status field",
                    self.axis
                ))
            })?;
        let position = data.positions.get(idx).copied().flatten().ok_or_else(|| {
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
            comms_error: data.comms_error,
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
    use std::sync::atomic::{AtomicBool, Ordering};

    /// One MM4000 axis parked at 5 mm whose line goes silent on `dead`.
    struct Line {
        base: epics_rs::asyn::port::PortDriverBase,
        dead: Arc<AtomicBool>,
        reply: Vec<u8>,
        sent: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for Line {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).trim_end().to_owned();
            self.sent.lock().unwrap().push(cmd.clone());
            self.reply = match cmd.as_str() {
                "VE;" => b"VEMM4000 Version 2.04".to_vec(),
                "TP;" => b"1TP5.0000".to_vec(),
                "MS;" => b"1MS@".to_vec(),
                "TE;" => b"TE@".to_vec(),
                "1TU" => b"1TU0.0001".to_vec(),
                c => format!("{c}0").into_bytes(),
            };
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            if self.dead.load(Ordering::SeqCst) {
                return Err(mm_err("timeout".into()));
            }
            let n = self.reply.len();
            buf[..n].copy_from_slice(&self.reply);
            Ok(n)
        }
    }

    /// A two-axis MM4000 that records every command and stays silent on the
    /// ones in `silent`.
    struct Scripted {
        base: epics_rs::asyn::port::PortDriverBase,
        silent: Arc<Mutex<Vec<&'static str>>>,
        reply: Vec<u8>,
        sent: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for Scripted {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).trim_end().to_owned();
            self.sent.lock().unwrap().push(cmd.clone());
            self.reply = match cmd.as_str() {
                c if self.silent.lock().unwrap().contains(&c) => Vec::new(),
                "VE;" => b"VEMM4005 Version 2.04".to_vec(),
                "TH;" => b"1TH1.5000,2TH-2.2500".to_vec(),
                "TP;" => b"1TP1.4990,2TP-2.2490".to_vec(),
                // Axis 2 moving in the plus direction, on its plus limit.
                "MS;" => b"1MS@,2MSM".to_vec(),
                "TE;" => b"TE@".to_vec(),
                // Axis 1 open loop: C's atoi reads "0.001" as 0.
                "2TC" => b"2TC1".to_vec(),
                c => format!("{c}0.001").into_bytes(),
            };
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            if reply.is_empty() {
                return Err(mm_err("timeout".into()));
            }
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    /// Run drvMM4000.cc's motor_init for one card on a scripted line and
    /// return what it put on the wire.
    fn model1_init(name: &str, silent: &'static [&'static str]) -> (bool, Vec<String>) {
        let (axes, sent, _) = model1_card(name, silent, Duration::ZERO);
        let wire = sent.lock().unwrap().clone();
        (axes.is_some(), wire)
    }

    type Card = (
        Option<Vec<Mm4000Axis>>,
        Arc<Mutex<Vec<String>>>,
        Arc<Mutex<Vec<&'static str>>>,
    );

    /// drvMM4000.cc's motor_init for one card: its axes, the wire log, and
    /// the commands the line ignores.
    fn model1_card(name: &str, silent: &'static [&'static str], cache_ttl: Duration) -> Card {
        let silent = Arc::new(Mutex::new(silent.to_vec()));
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Scripted {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            silent: silent.clone(),
            reply: Vec::new(),
            sent: sent.clone(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_millis(100));
        let Ok(ctrl) = Mm4000Controller::new_model1(io, name, cache_ttl) else {
            return (None, sent, silent);
        };
        let num_axes = ctrl.num_axes();
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axes = (1..=num_axes)
            .map(|axis| {
                let mut ax = Mm4000Axis::new(ctrl.clone(), axis, MODEL1_POLL_FAILURE);
                ax.model1_set_status();
                ax
            })
            .collect();
        (Some(axes), sent, silent)
    }

    /// One drvMM4000.cc motor_task cycle: start_status(-1) asks MS;, TP; and
    /// TH; once, then set_status asks TE; per axis; the position comes from
    /// TH; and the encoder from TP; only on a closed loop.
    #[test]
    fn model1_poll_cycle_is_start_status_then_te_per_axis() {
        let (axes, sent, _) = model1_card("mm4000_m1_cycle", &[], Duration::from_secs(3600));
        let mut axes = axes.unwrap();
        sent.lock().unwrap().clear();
        let user = AsynUser::default();
        let one = axes[0].poll(&user).unwrap();
        let two = axes[1].poll(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TP;", "TH;", "TE;", "TE;"]);

        assert_eq!((one.position, one.encoder_position), (1.5, 0.0));
        assert!(one.done && one.powered && !one.has_encoder && !one.problem);
        assert_eq!((two.position, two.encoder_position), (-2.25, -2.249));
        assert!(two.moving && !two.done && two.direction && two.high_limit);
        assert!(two.has_encoder && !two.low_limit && !two.home);
    }

    /// An unanswered MS; is a RETRY, which asks TH; but no TP; or TE; and
    /// leaves the status as it was; the second is COMM_ERR, and an answer
    /// clears it.
    #[test]
    fn model1_poll_retries_once_then_flags_comm_error() {
        let (axes, sent, silent) = model1_card("mm4000_m1_retry", &[], Duration::ZERO);
        let mut axis = axes.unwrap().remove(1);
        let user = AsynUser::default();
        let before = axis.poll(&user).unwrap();

        silent.lock().unwrap().push("MS;");
        sent.lock().unwrap().clear();
        assert_eq!(axis.poll(&user).unwrap(), before);
        let comm_err = axis.poll(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TH;", "MS;", "TH;"]);
        assert!(comm_err.comms_error && comm_err.problem);
        assert_eq!(comm_err.position, before.position);

        silent.lock().unwrap().clear();
        sent.lock().unwrap().clear();
        let back = axis.poll(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TP;", "TH;", "TE;"]);
        assert_eq!(back, before);
    }

    /// drvMM4000.cc:700-811: TH; probe, ST;, VE;, TH; count, start_status,
    /// then TC/TU/XH/TL/TR and set_status's TE; per axis.
    #[test]
    fn model1_init_puts_drv_mm4000_sequence_on_the_wire() {
        let (kept, wire) = model1_init("mm4000_m1_seq", &[]);
        assert!(kept);
        let axis = |a: usize| {
            ["TC", "TU", "XH", "TL", "TR"]
                .iter()
                .map(move |cc| format!("{a}{cc}"))
                .chain(["TE;".to_owned()])
        };
        let expected: Vec<String> = ["TH;", "ST;", "VE;", "TH;", "MS;", "TH;", "TP;"]
            .iter()
            .map(|c| c.to_string())
            .chain(axis(1))
            .chain(axis(2))
            .collect();
        assert_eq!(wire, expected);
    }

    /// An unanswered MS; leaves the card RETRY: start_status skips TH;/TP;
    /// and set_status asks no TE;.
    #[test]
    fn model1_init_without_ms_asks_no_te() {
        let (kept, wire) = model1_init("mm4000_m1_noms", &["MS;"]);
        assert!(kept);
        assert_eq!(&wire[..5], ["TH;", "ST;", "VE;", "TH;", "MS;"]);
        assert_eq!(&wire[5..7], ["1TC", "1TU"]);
        assert!(!wire.iter().any(|c| c == "TE;" || c == "TP;"));
    }

    /// The TH; probe gets three tries; a card that never answers is dropped
    /// before ST;.
    #[test]
    fn model1_init_drops_a_card_silent_on_th() {
        let (kept, wire) = model1_init("mm4000_m1_silent", &["TH;"]);
        assert!(!kept);
        assert_eq!(wire, ["TH;", "TH;", "TH;"]);
    }

    /// C MM4000Poller only raises motorAxisCommError on a failed `MS;`/`TP;`
    /// (drvMM4000Asyn.c:653-657), so the readback keeps the last position.
    #[test]
    fn a_dead_line_keeps_the_last_position() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let dead = Arc::new(AtomicBool::new(false));
        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Line {
            base: PortDriverBase::new("mm4000_dead", 1, PortFlags::default()),
            dead: dead.clone(),
            reply: Vec::new(),
            sent: sent.clone(),
        };
        let (rt, _jh) = create_port_runtime(
            &claim_port("mm4000_dead").unwrap(),
            line,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let ctrl = Mm4000Controller::new(io, "mm4000_dead", "mm4000_dead_motor", 1, Duration::ZERO)
            .ok()
            .unwrap();
        let mut axis = Mm4000Axis::new(Arc::new(Mutex::new(ctrl)), 1, MODEL2_POLL_FAILURE);
        let user = AsynUser::default();
        assert_eq!(axis.poll(&user).unwrap().position, 5.0);

        dead.store(true, Ordering::SeqCst);
        sent.lock().unwrap().clear();
        let status = axis.poll(&user).unwrap();
        // C sends `TP;` only after `MS;` succeeded.
        assert_eq!(*sent.lock().unwrap(), ["MS;"]);
        assert!(status.comms_error);
        assert_eq!(status.position, 5.0, "a dead line reads back as 0");
        assert!(status.done);
    }

    /// C `MM4000AsynConfig` returns before any axis on a silent `VE;`
    /// (drvMM4000Asyn.c:817-825) and on fewer `TP;` fields than numAxes
    /// (:853-857).
    #[test]
    fn config_refusals_create_no_controller() {
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        for (name, dead, num_axes) in [("mm4000_silent", true, 1), ("mm4000_short", false, 2)] {
            let line = Line {
                base: PortDriverBase::new(name, 1, PortFlags::default()),
                dead: Arc::new(AtomicBool::new(dead)),
                reply: Vec::new(),
                sent: Arc::default(),
            };
            let (rt, _jh) =
                create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                    .unwrap();
            let io =
                SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_millis(100));
            assert!(
                Mm4000Controller::new(io, name, name, num_axes, Duration::ZERO).is_err(),
                "{name}"
            );
        }
    }

    /// drvMM4000Asyn.c:928,951: `nWrite != nWriteRequested` makes it asynError.
    #[test]
    fn an_exchange_short_of_the_command_is_asyn_error() {
        let refused = AsynError::Status {
            status: AsynStatus::Disconnected,
            message: "port SERIAL1 not connected".into(),
        };
        let e = c_exchange_error(refused, 0, 3);
        assert_eq!(
            (e.status(), e.message()),
            (AsynStatus::Error, "port SERIAL1 not connected".into())
        );
        let timeout = AsynError::Status {
            status: AsynStatus::Timeout,
            message: String::new(),
        };
        assert_eq!(
            c_exchange_error(timeout, 3, 3).status(),
            AsynStatus::Timeout
        );
    }

    #[test]
    fn axis_count_is_the_strtok_token_count() {
        assert_eq!(strtok_count(""), 0);
        assert_eq!(strtok_count("1TP5.0"), 1);
        assert_eq!(strtok_count("1TP5.0,,2TP1.0,"), 2);
    }

    #[test]
    fn model_detection_via_mm_substring() {
        assert_eq!(detect_model("MM4000 2.04"), Some(Mm4000Model::Mm4000));
        assert_eq!(detect_model(" MM4005 f/w 1.0"), Some(Mm4000Model::Mm4005));
        assert_eq!(detect_model("MM4006"), Some(Mm4000Model::Mm4005));
        assert_eq!(detect_model("MM3000"), None);
        assert_eq!(detect_model("ESP300"), None);
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
