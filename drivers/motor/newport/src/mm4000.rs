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
//! The record hands raw steps; as C, commands multiply them by the `TU` step
//! size into the wire's mm/deg and readbacks divide by it (drvMM4000.cc
//! rounds the position to `NINT` steps and truncates the encoder). The
//! command precision is `maxDigits = -log10(stepSize) + 2`. A `TU` of 0
//! (a failed query) refuses commands where C sends `0`; its polls read the
//! status as C's, but the readback C divides into `inf` holds and the axis
//! is a problem, as PM500's.
//!
//! ## Controller-wide poll
//!
//! C polls per controller, not per axis: one `MS;` returns every axis's
//! status byte (`1MSx,2MSy,...`) and one `TP;` every position
//! (`1TP5.012,2TP1.123,...`), then each axis in turn asks `TE;`. The
//! per-axis [`AsynMotor::poll`] calls share one cached pass with a TTL of
//! half the moving poll period, like the HXP driver.
//!
//! `MM4000Config` (model 1) instead runs drvMM4000.cc's cycle: `start_status`
//! (`MS;`, `TP;` when it answered, `TH;`) once per TTL, and `set_status` per
//! axis, which reads the position from `TH;`, the encoder from `TP;` and asks
//! `TE;` itself.
//!
//! ## Deviations from C (documented)
//!
//! - C's MM4005 torque-off path (`MF` then busy-wait until the `MS` power
//!   bit reads ON) loops forever if the bit never clears; this port bounds
//!   the wait at 50 tries (5 s) and errors.

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus, OctetStep};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, Model1Motor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use motor_common::drv_motor_asyn::{DrvSet, function};
use motor_common::model2::AxisLog;
use motor_common::param_lib::Params;

use crate::util::{atof, atoi, max_digits};
use motor_common::model1::{MoveStrings, init_read};

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

/// One axis's turn in C MM4000Poller's pass (drvMM4000Asyn.c:651-718).
#[derive(Clone, Copy, Debug)]
enum AxisTurn {
    /// `comStatus != 0`, from the pass's `MS;`/`TP;` or an earlier axis's
    /// `TE;`: only motorAxisCommError is set.
    CommError,
    /// The axis's fields of `MS;` and `TP;`, and whether its `TE;` reported
    /// a controller error.
    Read {
        byte: Option<u8>,
        position: Option<f64>,
        problem: bool,
    },
}

/// Shared controller endpoint: owns the serial handle and the cached
/// controller-wide poll. The caller holds the `Arc<Mutex<..>>` lock.
pub struct Mm4000Controller {
    handle: SyncIOHandle,
    /// The octet port C's `pController->pasynUser` prints its exchange
    /// errors on.
    serial_port: String,
    firmware: String,
    model: Mm4000Model,
    num_axes: usize,
    /// Each axis's `pAxis->logParam`, which the poller prints on.
    logs: Vec<Arc<AxisLog>>,
    /// The last pass.
    turns: Vec<AxisTurn>,
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
    /// `TP;` reported fewer fields than the axes asked for, past the
    /// firmware and model C has already stored.
    TooFewAxes {
        total_axes: usize,
        firmware: String,
        model: Mm4000Model,
    },
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
        num_axes: usize,
        cache_ttl: Duration,
    ) -> Result<Self, Mm4000Refusal> {
        let mut ctrl = Self {
            handle,
            serial_port: serial_port.to_owned(),
            firmware: String::new(),
            model: Mm4000Model::Mm4000,
            num_axes,
            logs: (0..num_axes).map(|_| Arc::default()).collect(),
            turns: Vec::new(),
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

        // C ignores the `TP;` status and counts what arrived. C defect, not
        // copied: a refused write leaves inputBuff the `VE;` reply, which
        // drvMM4000Asyn.c:846-850 counts as axes.
        let tp = ctrl.unchecked_reply("TP;");
        let total_axes = strtok_count(&tp);
        if total_axes < num_axes {
            return Err(Mm4000Refusal::TooFewAxes {
                total_axes,
                firmware: ctrl.firmware,
                model,
            });
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
            firmware: String::new(),
            model: Mm4000Model::Mm4000,
            num_axes: 0,
            logs: Vec::new(),
            turns: Vec::new(),
            fetched_at: None,
            cache_ttl,
            driver: Mm4000Driver::Model1(Model1Poll::default()),
        };
        let read = |ctrl: &Self, cmd: &str| ctrl.unchecked_reply(cmd);
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
        ctrl.logs = (0..ctrl.num_axes).map(|_| Arc::default()).collect();
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
        let read = |cmd: &str| self.unchecked_reply(cmd);
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
        if !self.is_model1() {
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
        step_size: f64,
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
        if let Some(position) =
            parse_axis_position(&poll.position, idx).and_then(|p| steps(p, step_size))
        {
            s.position = position.round();
        }
        s.encoder_position = match encoder_present {
            true => parse_axis_position(&poll.feedback, idx)
                .and_then(|e| steps(e, step_size))
                .map_or(s.encoder_position, f64::trunc),
            false => 0.0,
        };
        s.velocity = 0.0;
        s.problem = !te_ok(&self.unchecked_reply("TE;")) || step_size == 0.0;
        s
    }

    /// `asynPrint(.., ASYN_TRACE_ERROR, ..)`, which only drvMM4000Asyn.c has.
    fn print_error(&self, port: &str, addr: i32, args: std::fmt::Arguments<'_>) {
        if matches!(self.driver, Mm4000Driver::Asyn) {
            print_error(port, addr, args);
        }
    }

    /// The controller follows drvMM4000.cc / devMM4000.cc.
    fn is_model1(&self) -> bool {
        matches!(self.driver, Mm4000Driver::Model1(_))
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
    /// The poll snapshot predates the command, so the next poll reads the
    /// wire (C signals its poller after every move).
    fn write(&mut self, cmd: &str) -> AsynResult<()> {
        self.fetched_at = None;
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
                "drvMM4000Asyn:sendOnly: error sending command {cmd}, sent={sent}, status={}\n",
                e.status() as i32
            ),
        );
        Err(e)
    }

    /// Write a command and read the reply (C `sendAndReceive`).
    fn write_read(&self, cmd: &str) -> AsynResult<String> {
        let (reply, status) = self.exchange(cmd);
        status.map(|()| reply)
    }

    /// A reply whose status C ignores: drvMM4000Asyn.c parses whatever
    /// arrived, drvMM4000.cc's `recv_mess` empties the buffer on a failed
    /// read.
    fn unchecked_reply(&self, cmd: &str) -> String {
        match self.exchange(cmd) {
            (_, Err(_)) if self.is_model1() => String::new(),
            (reply, _) => reply,
        }
    }

    /// C `sendAndReceive`: the bytes that arrived, and the status.
    fn exchange(&self, cmd: &str) -> (String, AsynResult<()>) {
        // write_read is drvMM4000Asyn.c:947's writeRead; the model-1
        // recv_mess reads with no flush (drvMM4000.cc:582), so one late reply
        // shifts every later answer there: a C defect, not copied.
        let (raw, e) = match self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts()
        {
            (raw, Ok(())) => return (String::from_utf8_lossy(&raw).into_owned(), Ok(())),
            (raw, Err(e)) => (raw, e),
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
                "drvMM4000Asyn:sendAndReceive error calling writeRead, output={cmd} status={}, error={}\n",
                e.status() as i32,
                e.message()
            ),
        );
        (String::from_utf8_lossy(&raw).into_owned(), Err(e))
    }

    /// Axis `idx`'s (0-based) turn of the last pass, running C
    /// MM4000Poller's pass at most once per TTL; the six per-axis polls
    /// inside one record scan share one pass.
    fn axis_turn(&mut self, idx: usize) -> AxisTurn {
        if !self
            .fetched_at
            .is_some_and(|at| at.elapsed() < self.cache_ttl)
        {
            self.turns = self.poll_pass();
            self.fetched_at = Some(Instant::now());
        }
        self.turns[idx]
    }

    /// drvMM4000Asyn.c:644-718: `MS;`, `TP;` when it answered, then each
    /// axis in turn reads its fields and asks `TE;`, whose status the next
    /// axis takes as its `comStatus`.
    fn poll_pass(&self) -> Vec<AxisTurn> {
        let (mut com, status_all, position_all) = match self
            .write_read("MS;")
            .and_then(|ms| Ok((ms, self.write_read("TP;")?)))
        {
            Ok((ms, tp)) => (Ok(()), ms, tp),
            Err(e) => (Err(e), String::new(), String::new()),
        };
        let mut turns = Vec::with_capacity(self.num_axes);
        for (axis, log) in self.logs.iter().enumerate() {
            if let Err(e) = &com {
                log.print(
                    TraceMask::ERROR,
                    format_args!("MM4000Poller: error reading status={}\n", e.status() as i32),
                );
                turns.push(AxisTurn::CommError);
                continue;
            }
            let byte = parse_axis_status(&status_all, axis);
            let position = parse_axis_position(&position_all, axis);
            if let (Some(byte), Some(position)) = (byte, position) {
                log.print(
                    TraceMask::IO_DRIVER,
                    format_args!(
                        "MM4000Poller: axis {axis} axisStatus={:x}, position={}\n",
                        i32::from(byte as i8),
                        CFloat(position)
                    ),
                );
            }
            // C defect, not copied: a refused `TE;` write leaves `buff` the
            // strtok'd `TP;` copy of :692, which :713 prints as the
            // controller error; here a failed `TE;` reads as empty.
            let te = self.write_read("TE;");
            let reply = te.as_deref().unwrap_or("");
            let problem = !te_ok(reply);
            if problem {
                log.print(
                    TraceMask::ERROR,
                    format_args!("MM4000Poller: controller error {reply}\n"),
                );
            }
            com = te.map(drop);
            turns.push(AxisTurn::Read {
                byte,
                position,
                problem,
            });
        }
        turns
    }
}

/// drvMM4000Asyn.c `motorAxisReportAxis` (:194-206). The default is C's
/// calloc'd axis, which a Config that fails past its range checks leaves.
#[derive(Default)]
pub struct Mm4000AxisReport {
    /// 0-based, C's `pAxis->axis`.
    axis: usize,
    /// The `MS;` status byte the last poll read, C's `axisStatus`.
    status: Arc<AtomicI32>,
    high_limit: f64,
    low_limit: f64,
    home_preset: f64,
    step_size: f64,
    digits: usize,
}

impl Mm4000AxisReport {
    pub fn report(&self, out: &mut String, level: i32) {
        use std::fmt::Write;
        if level > 0 {
            let _ = write!(
                out,
                "Axis {}\n   axisStatus:  0x{:x}\n   high limit:  {:.6}\n   low limit:   {:.6}\n   \
                 home preset: {:.6}\n   step size:   {:.6}\n   max digits:  {}\n",
                self.axis,
                self.status.load(Ordering::Relaxed),
                CFloat(self.high_limit),
                CFloat(self.low_limit),
                CFloat(self.home_preset),
                CFloat(self.step_size),
                self.digits
            );
        }
    }
}

/// One MM4000 axis sharing a controller. Implements [`AsynMotor`].
pub struct Mm4000Axis {
    controller: Arc<Mutex<Mm4000Controller>>,
    /// C `pAxis->card`.
    card: i32,
    /// C `pAxis->logParam`.
    log: Arc<AxisLog>,
    /// 1-based controller axis number, sent unpadded (`%d`).
    axis: usize,
    model: Mm4000Model,
    /// Wire units per record step (C `stepSize` / `drive_resolution`, `TU`).
    step_size: f64,
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
    /// The last poll failed on drvMM4000.cc's RETRY, which posts nothing.
    retry_failed: bool,
    /// drvMM4000.cc `encoder_present`: `TC` reports a closed loop.
    encoder_present: bool,
    /// The status drvMM4000.cc's `set_status` starts from and RETRY keeps.
    last: MotorStatus,
    /// The axis follows devMM4000.cc (`MM4000Config`), whose commands are
    /// the record transaction's, not drvMM4000Asyn.c's.
    model1: bool,
    /// devMM4000.cc jogs to the record's DHLM/DLLM (controller units): the
    /// last limit the record wrote, the controller travel limit until then.
    jog_high: f64,
    jog_low: f64,
    /// The model-1 record's PREM and POST.
    strings: MoveStrings,
    /// What the model-2 poll last read from `MS;`, for [`Mm4000AxisReport`].
    status: Arc<AtomicI32>,
    /// drvMM4000Asyn.c `pAxis->params`.
    params: Arc<Params>,
    /// What model-1 `motor_init`'s `set_status` read, until the motor task
    /// takes it; `None` on the asyn driver.
    init_status: Option<AsynResult<MotorStatus>>,
}

/// C model-1 `set_status`'s COMM_ERR branch raises `CNTRL_COMM_ERR` and
/// `RA_PROBLEM` (drvMM4000.cc; its RETRY raises nothing); model-2
/// `MM4000Poller` only `motorAxisCommError` (drvMM4000Asyn.c).
pub const MODEL1_POLL_FAILURE: PollFailure = PollFailure::PROBLEM_AND_COMMS_ERROR;
pub const MODEL2_POLL_FAILURE: PollFailure = PollFailure::COMMS_ERROR;

impl Mm4000Axis {
    /// Construct axis `axis` (1-based), running the C per-axis init queries:
    /// `TC` (closed-loop state — drvMM4000.cc's encoder flag, which
    /// drvMM4000Asyn.c stores unused),
    /// `TU` (drive resolution → command precision), `XH` (home preset),
    /// `TL`/`TR` (travel limits). C checks none of these replies: a failed
    /// one parses what arrived, empty on model 1 (`atof` → 0).
    /// drvMM4000Asyn.c defect, not copied: a refused write leaves inputBuff
    /// the previous reply, which :869-896 then parse. Performs blocking
    /// serial I/O under the controller lock.
    pub fn new(
        controller: Arc<Mutex<Mm4000Controller>>,
        card: i32,
        axis: usize,
        poll_failure: PollFailure,
    ) -> Self {
        let (model1, model, log, step_size, home_preset, low_limit, high_limit, encoder_present) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // C `&inputBuff[3]`: skip the echoed `nCC` prefix.
            let query = |cc: &str| {
                let reply = ctrl.unchecked_reply(&format!("{axis}{cc}"));
                reply.get(3..).unwrap_or("").to_owned()
            };
            let encoder_present = atoi(query("TC")) != 0;
            let step_size = atof(&query("TU"));
            let home_preset = atof(&query("XH"));
            let low_limit = atof(&query("TL"));
            let high_limit = atof(&query("TR"));
            (
                ctrl.is_model1(),
                ctrl.model(),
                ctrl.logs[axis - 1].clone(),
                step_size,
                home_preset,
                low_limit,
                high_limit,
                encoder_present,
            )
        };
        let mut ax = Self {
            controller,
            card,
            log,
            axis,
            model,
            step_size,
            digits: max_digits(step_size),
            home_preset,
            low_limit,
            high_limit,
            poll_failure,
            retry_failed: false,
            encoder_present,
            model1,
            jog_high: high_limit,
            jog_low: low_limit,
            // drvMM4000.cc motor_init clears the status and sets
            // GAIN_SUPPORT, and EA_PRESENT on a closed loop.
            last: MotorStatus {
                gain_support: true,
                has_encoder: encoder_present,
                vbas_supported: false,
                ..MotorStatus::default()
            },
            strings: MoveStrings::default(),
            status: Arc::default(),
            params: {
                // motorAxisInit's GAIN_SUPPORT (drvMM4000Asyn.c:242).
                let params = Params::default();
                let _ = params.set_integer(function::HAS_CLOSED_LOOP, 1);
                Arc::new(params)
            },
            init_status: None,
        };
        // drvMM4000.cc:811: model-1 motor_init ends each axis with
        // set_status, on the strings its start_status left.
        let controller = ax.controller.clone();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Mm4000Driver::Model1(poll) = &ctrl.driver {
            ax.init_status = Some(ax.model1_status(&ctrl, poll));
        }
        ax
    }

    /// The axis' drvMM4000Asyn.c parameters, which drvAsynMotorConfigure
    /// reads and calls back through.
    pub fn params(&self) -> Arc<Params> {
        self.params.clone()
    }

    /// The log `drvAsynMotorConfigure` attaches, which the controller's
    /// pass prints on too.
    pub fn log(&self) -> Arc<AxisLog> {
        self.log.clone()
    }

    /// drvMM4000Asyn.c `motorAxisReportAxis`'s view of this axis.
    pub fn report(&self) -> Mm4000AxisReport {
        Mm4000AxisReport {
            axis: self.axis - 1,
            status: self.status.clone(),
            high_limit: self.high_limit,
            low_limit: self.low_limit,
            home_preset: self.home_preset,
            step_size: self.step_size,
            digits: self.digits,
        }
    }

    /// drvMM4000.cc `set_status` on `poll`'s strings: RETRY fails raising
    /// nothing, COMM_ERR raising `poll_failure`'s bits (:341-353).
    fn model1_status(
        &mut self,
        ctrl: &Mm4000Controller,
        poll: &Model1Poll,
    ) -> AsynResult<MotorStatus> {
        if poll.comm != CommStatus::Normal {
            self.retry_failed = poll.comm == CommStatus::Retry;
            return Err(mm_err(format!("MM4000 axis {}: no MS; reply", self.axis)));
        }
        let (idx, encoder, step) = (self.axis - 1, self.encoder_present, self.step_size);
        self.last = ctrl.set_status(poll, idx, encoder, step, &self.last);
        Ok(self.last.clone())
    }

    fn lock(&self) -> MutexGuard<'_, Mm4000Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// `stepSize`, refused at 0, where C sends every value as 0.
    fn step_size(&self) -> AsynResult<f64> {
        match self.step_size {
            0.0 => Err(mm_err(format!(
                "MM4000 axis {}: TU step size is 0",
                self.axis
            ))),
            step => Ok(step),
        }
    }

    /// Send a move or home and call back the done it clears.
    fn start_motion(&mut self, cmd: &str) -> AsynResult<()> {
        self.lock().write(cmd)?;
        // drvMM4000Asyn.c:485-486, 518-519.
        let _ = self.params.set_integer(function::DONE, 0);
        self.params.call_callback();
        Ok(())
    }

    /// devMM4000.cc:196-206: a model-1 move's PREM and `;`, ahead of its
    /// own command; its POST waits for the move to end.
    fn model1_prem(&mut self) -> String {
        let prem = self.strings.arm();
        match prem.is_empty() {
            true => String::new(),
            false => format!("{};", String::from_utf8_lossy(&prem)),
        }
    }

    /// The record's SET_ACCEL part of a model-1 move or home, which it
    /// leaves out for a zero rate (motorRecord.cc:874, 940).
    fn model1_accel(&self, acceleration: f64) -> String {
        let (a, d) = (self.axis, self.digits);
        if acceleration > 0.0 {
            format!(
                "{a}AC{acceleration:.d$};",
                acceleration = CFloat(acceleration)
            )
        } else {
            String::new()
        }
    }

    /// drvMM4000Asyn.c:466: `motorAxisMove`'s FLOW line, 0-based axis, which
    /// a jog prints too (:532-535).
    fn print_move(&self, position: f64, min_velocity: f64, velocity: f64, acceleration: f64) {
        if self.model1 {
            return;
        }
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "Set card {}, axis {} move to {}, min vel={}, max_vel={}, accel={}\n",
                self.card,
                self.axis - 1,
                CFloat(position),
                CFloat(min_velocity),
                CFloat(velocity),
                CFloat(acceleration)
            ),
        );
    }

    /// The drvMM4000Asyn.c refusal `PRINT`s, which model 1 does not have.
    fn print_error(&self, args: std::fmt::Arguments<'_>) {
        if !self.model1 {
            self.log.print(TraceMask::ERROR, args);
        }
    }

    /// C `motorAxisMove` transaction: `AC;VA;PA|PR;` (trailing `;` as C
    /// sends it), steps scaled to wire units at this axis's precision. On
    /// model 1 it is the record's SET_VELOCITY, SET_ACCEL, MOVE: `VA;AC;PA;`.
    fn move_command(
        &mut self,
        cmd: &str,
        value: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<String> {
        let (a, d, step) = (self.axis, self.digits, self.step_size()?);
        let (acceleration, velocity, value) = (acceleration * step, velocity * step, value * step);
        if self.model1 {
            return Ok(format!(
                "{a}VA{velocity:.d$};{}{}{a}{cmd}{value:.d$};",
                self.model1_accel(acceleration),
                self.model1_prem(),
                velocity = CFloat(velocity),
                value = CFloat(value)
            ));
        }
        Ok(format!(
            "{a}AC{acceleration:.d$};{a}VA{velocity:.d$};{a}{cmd}{value:.d$};",
            acceleration = CFloat(acceleration),
            velocity = CFloat(velocity),
            value = CFloat(value)
        ))
    }
}

impl Model1Motor for Mm4000Axis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.last.clone(), self.poll_failure())
    }
}

impl AsynMotor for Mm4000Axis {
    fn poll_failure(&self) -> PollFailure {
        match self.retry_failed {
            true => PollFailure::NONE,
            false => self.poll_failure,
        }
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.print_move(position, min_velocity, velocity, acceleration);
        let cmd = self.move_command("PA", position, velocity, acceleration)?;
        self.start_motion(&cmd)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.print_move(distance, min_velocity, velocity, acceleration);
        let cmd = self.move_command("PR", distance, velocity, acceleration)?;
        self.start_motion(&cmd)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if self.model1 {
            // devMM4000.cc JOG after the record's SET_ACCEL: `AC;VA;PA` to
            // the record's DHLM/DLLM.
            let (a, d, step) = (self.axis, self.digits, self.step_size()?);
            let (speed, acceleration) = (velocity.abs() * step, acceleration * step);
            let target = if velocity > 0.0 {
                self.jog_high
            } else {
                self.jog_low
            };
            let prem = self.model1_prem();
            return self.start_motion(&format!(
                "{a}AC{acceleration:.d$};{prem}{a}VA{speed:.d$};{a}PA{target:.d$};",
                acceleration = CFloat(acceleration),
                speed = CFloat(speed),
                target = CFloat(target)
            ));
        }
        // C `motorAxisVelocityMove`: no jog command — move absolute to the
        // controller travel limit captured at init, in the jog direction.
        let target = if velocity > 0.0 {
            self.high_limit
        } else {
            self.low_limit
        };
        let speed = if velocity > 0.0 { velocity } else { -velocity };
        self.print_move(target / self.step_size, min_velocity, speed, acceleration);
        let step = self.step_size()?;
        let cmd = self.move_command("PA", target / step, speed, acceleration)?;
        self.start_motion(&cmd)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        if !self.model1 {
            self.log.print(
                TraceMask::FLOW,
                format_args!(
                    "motorAxisHome: set card {}, axis {} to home\n",
                    self.card,
                    self.axis - 1
                ),
            );
        }
        // C `motorAxisHome` — direction is not parameterized; the stray
        // space after the first `;` is C's format string, kept for wire
        // parity.
        let (a, d, step) = (self.axis, self.digits, self.step_size()?);
        let (acceleration, velocity) = (acceleration * step, velocity * step);
        if self.model1 {
            // devMM4000.cc HOME after the record's SET_VELOCITY, SET_ACCEL.
            let accel = self.model1_accel(acceleration);
            let prem = self.model1_prem();
            return self.start_motion(&format!(
                "{a}VA{velocity:.d$};{accel}{prem}{a}OR;",
                velocity = CFloat(velocity)
            ));
        }
        self.start_motion(&format!(
            "{a}AC{acceleration:.d$}; {a}VA{velocity:.d$};{a}OR;",
            acceleration = CFloat(acceleration),
            velocity = CFloat(velocity)
        ))
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let a = self.axis;
        // devMM4000.cc STOP_AXIS is `ST;` alone; only drvMM4000Asyn.c
        // motorAxisStop sets the deceleration first.
        if self.model1 {
            return self.lock().write(&format!("{a}ST;"));
        }
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "Set card {}, axis {} to stop with accel={}\n",
                self.card,
                a - 1,
                CFloat(acceleration)
            ),
        );
        let (d, step) = (self.digits, self.step_size()?);
        let acceleration = acceleration * step;
        let mut ctrl = self.lock();
        ctrl.write(&format!(
            "{a}AC{acceleration:.d$};{a}ST;",
            acceleration = CFloat(acceleration)
        ))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `motorAxisPosition`: set home to the value, define home, then
        // restore the home preset (`SH;DH;SH`). devMM4000.cc LOAD_POS sends
        // it only to an MM4000 and nothing to the other models.
        if self.model1 && self.model != Mm4000Model::Mm4000 {
            return Ok(());
        }
        let (a, d, step) = (self.axis, self.digits, self.step_size()?);
        let position = position * step;
        let mut ctrl = self.lock();
        ctrl.write(&format!(
            "{a}SH{position:.d$};{a}DH;{a}SH{:.d$}",
            CFloat(self.home_preset),
            position = CFloat(position)
        ))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        if self.model1 {
            // devMM4000.cc ENABLE/DISABL_TORQUE: the MM4000 switches every
            // motor, the other models one axis.
            let (a, cc) = (self.axis, if enable { "MO" } else { "MF" });
            let cmd = match self.model {
                Mm4000Model::Mm4000 => format!("{cc};"),
                _ => format!("{a}{cc};"),
            };
            return self.lock().write(&cmd);
        }
        // C `motorAxisClosedLoop`: the MM4000 can only switch ALL motors
        // (MO/MF), so it is a no-op there; the MM4005 addresses one axis.
        if self.model == Mm4000Model::Mm4000 {
            return Ok(());
        }
        let a = self.axis;
        let mut ctrl = self.lock();
        if enable {
            return ctrl.write(&format!("{a}MO"));
        }
        ctrl.write(&format!("{a}MF"))?;
        // C busy-waits (unbounded) until the axis power bit reads ON after
        // MF; bounded here (see module Deviations).
        for _ in 0..TORQUE_OFF_MAX_TRIES {
            let status_all = ctrl.unchecked_reply("MS;");
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
        let (a, d, step) = (self.axis, self.digits, self.step_size()?);
        let position = position * step;
        if self.model1 {
            // devMM4000.cc SET_HIGH_LIMIT sends nothing; past the controller
            // limit it clamps DHLM to it and refuses.
            if position > self.high_limit {
                self.jog_high = self.high_limit;
                return Err(mm_err(format!(
                    "MM4000 axis {a}: high limit {} beyond the controller's {}",
                    CFloat(position).g(),
                    CFloat(self.high_limit).g()
                )));
            }
            self.jog_high = position;
            return Ok(());
        }
        let mut ctrl = self.lock();
        ctrl.write(&format!("{a}SR{position:.d$}", position = CFloat(position)))
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let (a, d, step) = (self.axis, self.digits, self.step_size()?);
        let position = position * step;
        if self.model1 {
            if position < self.low_limit {
                self.jog_low = self.low_limit;
                return Err(mm_err(format!(
                    "MM4000 axis {a}: low limit {} beyond the controller's {}",
                    CFloat(position).g(),
                    CFloat(self.low_limit).g()
                )));
            }
            self.jog_low = position;
            return Ok(());
        }
        let mut ctrl = self.lock();
        ctrl.write(&format!("{a}SL{position:.d$}", position = CFloat(position)))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        if self.model1 {
            // devMM4000.cc SET_[PID]GAIN: the record's gain, unscaled.
            let a = self.axis;
            let cc = match kind {
                PidGainKind::Proportional => "KP",
                PidGainKind::Integral => "KI",
                PidGainKind::Derivative => "KD",
            };
            return self
                .lock()
                .write(&format!("{a}{cc}{gain:.6};{a}UF;", gain = CFloat(gain)));
        }
        let what = match kind {
            PidGainKind::Proportional => "proportional",
            PidGainKind::Integral => "integral",
            PidGainKind::Derivative => "derivative",
        };
        self.print_error(format_args!(
            "MM4000 does not support setting {what} gain\n"
        ));
        Err(mm_err(format!(
            "MM4000 does not support setting {kind:?} gain"
        )))
    }

    /// drvMM4000Asyn.c:344-348: model 2 refuses the ratio devMotorAsyn
    /// writes at init.
    fn set_encoder_ratio(&mut self, _user: &AsynUser, _ratio: f64) -> AsynResult<()> {
        if self.model1 {
            return Ok(());
        }
        self.print_error(format_args!(
            "motorAxisSetDouble: MM4000 does not support setting encoder ratio\n"
        ));
        Err(mm_err(
            "MM4000 does not support setting encoder ratio".to_owned(),
        ))
    }

    /// devMM4000.cc:188-192: a model-1 INIT and a CR.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        if !self.model1 {
            return Ok(());
        }
        let init = format!("{}\r", String::from_utf8_lossy(init));
        self.lock().write(&init)
    }

    fn set_move_strings(
        &mut self,
        _user: &AsynUser,
        prem: &[u8],
        post: Option<PostMessage>,
    ) -> AsynResult<()> {
        if self.model1 {
            self.strings.set(prem, post);
        }
        Ok(())
    }

    /// drvMM4000Asyn.c `motorAxisforceCallback`; the poll that follows is
    /// the poller's.
    fn force_callback(&mut self, _user: &AsynUser) {
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "motorAxisforceCallback: request card {}, axis {} status update\n",
                self.card,
                self.axis - 1
            ),
        );
        self.params.force_callback();
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        self.retry_failed = false;
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(poll) = ctrl.model1_poll() {
            let s = self.model1_status(&ctrl, &poll)?;
            // drvMM4000.cc:495-501: POST once the move is done or stopped by
            // the limit in its direction, past an answered MS; and TE;.
            let ls_active = (s.high_limit && s.direction) || (s.low_limit && !s.direction);
            if !s.problem
                && (s.done || ls_active)
                && let Some(post) = self.strings.post_due()
            {
                let _ = ctrl.write(&String::from_utf8_lossy(&post));
            }
            *status = s;
            return Ok(());
        }
        let idx = self.axis - 1;
        let turn = ctrl.axis_turn(idx);
        drop(ctrl);
        let AxisTurn::Read {
            byte,
            position,
            problem,
        } = turn
        else {
            let _ = self.params.set_integer(function::COMM_ERROR, 1);
            self.params.call_callback();
            *status = self.motor_status();
            return Ok(());
        };
        let byte = byte.ok_or_else(|| {
            mm_err(format!(
                "MM4000 axis {}: missing MS status field",
                self.axis
            ))
        })?;
        // C stores the status `char`, signed on its targets.
        self.status.store(i32::from(byte as i8), Ordering::Relaxed);
        let position = position.ok_or_else(|| {
            mm_err(format!(
                "MM4000 axis {}: missing TP position field",
                self.axis
            ))
        })?;
        let step = self.step_size;
        self.set_read_params(byte, steps(position, step), problem || step == 0.0);
        *status = self.motor_status();
        Ok(())
    }
}

/// A wire readback in record steps; `None` at a 0 `TU`, which C divides by
/// into `inf` and posts (a C defect).
fn steps(wire: f64, step_size: f64) -> Option<f64> {
    (step_size != 0.0).then(|| wire / step_size)
}

impl Mm4000Axis {
    /// MM4000Poller's parameters of an axis it read (drvMM4000Asyn.c:659-716).
    /// The limit, home and direction parameters are C's masked status bits,
    /// not 0/1, and done is the moving bit alone.
    fn set_read_params(&self, byte: u8, position: Option<f64>, problem: bool) {
        let params = &self.params;
        for (f, v) in [
            (function::COMM_ERROR, 0),
            (function::DONE, i32::from(byte & MM4000_MOVING == 0)),
            (function::HOME_SIGNAL, i32::from(byte & MM4000_HOME)),
            (
                function::HIGH_HARD_LIMIT,
                i32::from(byte & MM4000_HIGH_LIMIT),
            ),
            (function::LOW_HARD_LIMIT, i32::from(byte & MM4000_LOW_LIMIT)),
            (function::DIRECTION, i32::from(byte & MM4000_DIRECTION)),
            (function::POWER_ON, i32::from(byte & MM4000_POWER_OFF == 0)),
            (function::PROBLEM, i32::from(problem)),
        ] {
            let _ = params.set_integer(f, v);
        }
        if let Some(position) = position {
            let _ = params.set_double(function::POSITION, position);
            let _ = params.set_double(function::ENCODER_POSN, position);
        }
        params.call_callback();
    }

    /// The parameters as the poll returns them; the MM4000 move transaction
    /// has no base-velocity (VB) command.
    fn motor_status(&self) -> MotorStatus {
        MotorStatus {
            vbas_supported: false,
            ..self.params.motor_status()
        }
    }
}

/// C `motorAxisSetDouble`/`motorAxisSetInteger` (drvMM4000Asyn.c:323-452),
/// which drvMotorAsyn's writes reach.
impl DrvSet for Mm4000Axis {
    fn set_double(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()> {
        self.set_double_only(user, function, value)?;
        let _ = self.params.set_double(function, value);
        self.params.call_callback();
        Ok(())
    }

    fn set_integer(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()> {
        self.set_integer_only(user, function, value)?;
        let _ = self.params.set_integer(function, value);
        self.params.call_callback();
        Ok(())
    }
}

impl Mm4000Axis {
    fn set_double_only(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()> {
        let kind = match function {
            function::POSITION => return self.set_position(user, value),
            function::ENCODER_RATIO => return self.set_encoder_ratio(user, value),
            function::LOW_LIMIT => return self.set_low_limit(user, value),
            function::HIGH_LIMIT => return self.set_high_limit(user, value),
            function::P_GAIN => PidGainKind::Proportional,
            function::I_GAIN => PidGainKind::Integral,
            function::D_GAIN => PidGainKind::Derivative,
            function::RESOLUTION => {
                self.print_error(format_args!(
                    "motorAxisSetDouble: MM4000 does not support setting resolution\n"
                ));
                return Err(mm_err(
                    "MM4000 does not support setting resolution".to_owned(),
                ));
            }
            _ => {
                self.print_error(format_args!(
                    "motorAxisSetDouble: unknown function {function}\n"
                ));
                return Err(mm_err(format!(
                    "motorAxisSetDouble: unknown function {function}"
                )));
            }
        };
        self.set_pid_gain(user, kind, value)
    }

    fn set_integer_only(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()> {
        if function == function::CLOSED_LOOP {
            return self.set_closed_loop(user, value != 0);
        }
        self.print_error(format_args!(
            "motorAxisSetInteger: unknown function {function}\n"
        ));
        Err(mm_err(format!(
            "motorAxisSetInteger: unknown function {function}"
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::Polled;
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
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            if self.dead.load(Ordering::SeqCst) {
                return epics_rs::asyn::interpose::OctetReadResult::failed(mm_err(
                    "timeout".into(),
                ));
            }
            let n = self.reply.len();
            buf[..n].copy_from_slice(&self.reply);
            epics_rs::asyn::interpose::OctetReadResult::ok(
                n,
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
    }

    /// A two-axis MM4000 that records every command and stays silent on the
    /// ones in `silent`.
    struct Scripted {
        base: epics_rs::asyn::port::PortDriverBase,
        silent: Arc<Mutex<Vec<&'static str>>>,
        /// The `TP;` reply.
        tp: Arc<Mutex<&'static str>>,
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
                "TP;" => self.tp.lock().unwrap().as_bytes().to_vec(),
                // Axis 2 moving in the plus direction, on its plus limit.
                "MS;" => b"1MS@,2MSM".to_vec(),
                "TE;" => b"TE@".to_vec(),
                // Axis 1 open loop: C's atoi reads "0.001" as 0.
                "2TC" => b"2TC1".to_vec(),
                c => format!("{c}0.001").into_bytes(),
            };
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            let reply = std::mem::take(&mut self.reply);
            if reply.is_empty() {
                return epics_rs::asyn::interpose::OctetReadResult::failed(mm_err(
                    "timeout".into(),
                ));
            }
            buf[..reply.len()].copy_from_slice(&reply);
            epics_rs::asyn::interpose::OctetReadResult::ok(
                reply.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            )
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
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Scripted {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            silent: silent.clone(),
            tp: Arc::new(Mutex::new("1TP1.4990,2TP-2.2490")),
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
            .map(|axis| Mm4000Axis::new(ctrl.clone(), 0, axis, MODEL1_POLL_FAILURE))
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
        let one = axes[0].polled(&user).unwrap();
        let two = axes[1].polled(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TP;", "TH;", "TE;", "TE;"]);

        assert_eq!((one.position, one.encoder_position), (1500.0, 0.0));
        assert!(one.done && one.powered && !one.has_encoder && !one.problem);
        assert_eq!((two.position, two.encoder_position), (-2250.0, -2249.0));
        assert!(two.moving && !two.done && two.direction && two.high_limit);
        assert!(two.has_encoder && !two.low_limit && !two.home);
    }

    /// devMM4000.cc builds each command from the record's transaction parts:
    /// VA;AC before a move or home, AC;VA;PA to DHLM/DLLM for a jog, no SR/SL,
    /// unscaled gains, per-axis MO;/MF; and no LOAD_POS on an MM4005. TU
    /// 0.001 here, so 5 digits.
    #[test]
    fn model1_commands_are_the_record_transaction() {
        let (axes, sent, _) = model1_card("mm4000_m1_cmds", &[], Duration::ZERO);
        let mut axis = axes.unwrap().remove(0);
        let user = AsynUser::default();
        let wire = |f: &mut dyn FnMut()| {
            sent.lock().unwrap().clear();
            f();
            sent.lock().unwrap().clone()
        };
        assert_eq!(
            wire(&mut || axis
                .move_absolute(&user, 2000.0, 0.0, 1000.0, 500.0)
                .unwrap()),
            ["1VA1.00000;1AC0.50000;1PA2.00000;"]
        );
        assert_eq!(
            wire(&mut || axis.home(&user, 0.0, 1000.0, 500.0, true).unwrap()),
            ["1VA1.00000;1AC0.50000;1OR;"]
        );
        assert_eq!(
            wire(&mut || axis.set_high_limit(&user, 0.5).unwrap()),
            [] as [&str; 0]
        );
        assert!(axis.set_high_limit(&user, 2000.0).is_err());
        axis.set_high_limit(&user, 0.5).unwrap();
        assert_eq!(
            wire(&mut || axis.move_velocity(&user, 0.0, 1000.0, 500.0).unwrap()),
            ["1AC0.50000;1VA1.00000;1PA0.00050;"]
        );
        assert_eq!(
            wire(&mut || axis
                .set_pid_gain(&user, PidGainKind::Proportional, 0.25)
                .unwrap()),
            ["1KP0.250000;1UF;"]
        );
        assert_eq!(
            wire(&mut || axis.set_closed_loop(&user, false).unwrap()),
            ["1MF;"]
        );
        assert_eq!(
            wire(&mut || axis.set_position(&user, 5.0).unwrap()),
            [] as [&str; 0]
        );
    }

    /// devMM4000.cc:188-206 and drvMM4000.cc:495-501: INIT alone, PREM
    /// ahead of each move's own command, POST once after that move is done
    /// or stopped by the limit in its direction.
    #[test]
    fn model1_init_prem_and_post_go_out_as_c_builds_them() {
        let (axes, sent, _) = model1_card("mm4000_m1_strings", &[], Duration::ZERO);
        let mut axes = axes.unwrap();
        let user = AsynUser::default();
        let post = |text: &[u8]| {
            let p = PostMessage::default();
            p.set(text);
            Some(p)
        };
        let posts = |sent: &Arc<Mutex<Vec<String>>>| -> Vec<String> {
            let wire = std::mem::take(&mut *sent.lock().unwrap());
            wire.into_iter().filter(|c| c.contains('Y')).collect()
        };
        sent.lock().unwrap().clear();
        axes[0].primitive(&user, b"1MO").unwrap();
        axes[0]
            .set_move_strings(&user, b"1XX", post(b"1YY"))
            .unwrap();
        axes[0]
            .move_absolute(&user, 2000.0, 0.0, 1000.0, 500.0)
            .unwrap();
        axes[0]
            .move_relative(&user, 2000.0, 0.0, 1000.0, 0.0)
            .unwrap();
        axes[0].set_move_strings(&user, b"1XX", None).unwrap();
        axes[0].home(&user, 0.0, 1000.0, 500.0, true).unwrap();
        axes[1].set_move_strings(&user, b"2XX", None).unwrap();
        axes[1].move_velocity(&user, 0.0, 1000.0, 500.0).unwrap();
        assert_eq!(
            std::mem::take(&mut *sent.lock().unwrap()),
            [
                "1MO",
                "1VA1.00000;1AC0.50000;1XX;1PA2.00000;",
                "1VA1.00000;1PR2.00000;",
                "1VA1.00000;1AC0.50000;1XX;1OR;",
                "2AC0.50000;2XX;2VA1.00000;2PA0.00100;"
            ]
        );
        // The home armed no POST, so axis 1 has none due.
        axes[0].polled(&user).unwrap();
        assert!(posts(&sent).is_empty());
        axes[0].set_move_strings(&user, b"", post(b"1YY")).unwrap();
        axes[0]
            .move_absolute(&user, 0.0, 0.0, 1000.0, 500.0)
            .unwrap();
        axes[0].polled(&user).unwrap();
        axes[0].polled(&user).unwrap();
        assert_eq!(posts(&sent), ["1YY"], "sent once");
        // Axis 2 is moving, but on its plus limit while moving plus.
        axes[1].set_move_strings(&user, b"", post(b"2YY")).unwrap();
        axes[1]
            .move_absolute(&user, 0.0, 0.0, 1000.0, 500.0)
            .unwrap();
        axes[1].polled(&user).unwrap();
        assert_eq!(posts(&sent), ["2YY"]);
    }

    /// devMM4000.cc STOP_AXIS sends `ST;` with no deceleration.
    #[test]
    fn model1_stop_is_st_alone() {
        let (axes, sent, _) = model1_card("mm4000_m1_stop", &[], Duration::ZERO);
        let mut axes = axes.unwrap();
        sent.lock().unwrap().clear();
        axes[0].stop(&AsynUser::default(), 0.0).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["1ST;"]);
    }

    /// drvMM4000Asyn.c `%.*f`: glibc's nan, sign bit included.
    #[test]
    fn model1_a_nan_move_goes_out_as_c_prints_it() {
        let (axes, sent, _) = model1_card("mm4000_m1_nan", &[], Duration::ZERO);
        let mut axes = axes.unwrap();
        let user = AsynUser::default();
        sent.lock().unwrap().clear();
        axes[0]
            .move_absolute(&user, -f64::NAN, 0.0, f64::NAN, 0.0)
            .unwrap();
        assert_eq!(*sent.lock().unwrap(), ["1VAnan;1PA-nan;"]);
    }

    /// motorRecord.cc:874, 940: no SET_ACCEL for a zero move or home rate;
    /// the jog's is unconditional.
    #[test]
    fn model1_zero_rate_move_and_home_have_no_ac() {
        let (axes, sent, _) = model1_card("mm4000_m1_acc0", &[], Duration::ZERO);
        let mut axes = axes.unwrap();
        let user = AsynUser::default();
        sent.lock().unwrap().clear();
        axes[0].move_absolute(&user, 100.0, 0.0, 10.0, 0.0).unwrap();
        axes[0].home(&user, 0.0, 10.0, 0.0, true).unwrap();
        let wire = sent.lock().unwrap().clone();
        assert!(wire.iter().all(|c| !c.contains("AC")), "{wire:?}");
    }

    /// drvMM4000.cc:763-811: a `motor_init` read that start_status's
    /// unanswered MS; failed (RETRY, :341-353) stores the GAIN_SUPPORT
    /// `motor_init` set, and EA_PRESENT on axis 2's closed loop.
    #[test]
    fn model1_a_failed_init_read_keeps_what_motor_init_set() {
        let (axes, _, _) = model1_card("mm4000_m1_init_fail", &["MS;"], Duration::ZERO);
        let init: Vec<_> = axes
            .unwrap()
            .iter_mut()
            .map(|axis| axis.take_init_status())
            .collect();
        let prepared = |encoder| MotorStatus {
            gain_support: true,
            has_encoder: encoder,
            vbas_supported: false,
            ..MotorStatus::default()
        };
        assert_eq!(init, [prepared(false), prepared(true)]);
    }

    /// drvMM4000.cc set_status reads a 0 `TU`'s axis like any other
    /// (:365-468) and divides its readback by the 0; the readback holds
    /// instead and the axis is a problem, at init and on the poll.
    #[test]
    fn model1_a_zero_tu_reads_the_status_and_holds_the_position() {
        let (axes, sent, _) = model1_card("mm4000_m1_tu0", &["1TU"], Duration::from_secs(3600));
        let mut axes = axes.unwrap();
        let init = axes[0].take_init_status();
        assert!(init.done && init.powered && init.problem && !init.comms_error);
        assert_eq!(init.position, 0.0);
        sent.lock().unwrap().clear();
        let user = AsynUser::default();
        let one = axes[0].polled(&user).unwrap();
        let two = axes[1].polled(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TP;", "TH;", "TE;", "TE;"]);
        assert!(one.done && one.powered && one.problem && !one.comms_error);
        assert_eq!((one.position, one.encoder_position), (0.0, 0.0));
        assert!(!two.problem && two.high_limit);
        assert_eq!((two.position, two.encoder_position), (-2250.0, -2249.0));
    }

    /// An unanswered MS; is a RETRY, which asks TH; but no TP; or TE; and
    /// posts nothing; the second is COMM_ERR, and an answer clears it.
    #[test]
    fn model1_poll_retries_once_then_flags_comm_error() {
        let (axes, sent, silent) = model1_card("mm4000_m1_retry", &[], Duration::ZERO);
        let mut axis = axes.unwrap().remove(1);
        let user = AsynUser::default();
        let before = axis.polled(&user).unwrap();

        silent.lock().unwrap().push("MS;");
        sent.lock().unwrap().clear();
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), MODEL1_POLL_FAILURE);
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TH;", "MS;", "TH;"]);

        silent.lock().unwrap().clear();
        sent.lock().unwrap().clear();
        let back = axis.polled(&user).unwrap();
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
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let dead = Arc::new(AtomicBool::new(false));
        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Line {
            base: PortDriverBase::new(
                "mm4000_dead",
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
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
        let ctrl = Mm4000Controller::new(io, "mm4000_dead", 1, Duration::ZERO)
            .ok()
            .unwrap();
        let mut axis = Mm4000Axis::new(Arc::new(Mutex::new(ctrl)), 0, 1, MODEL2_POLL_FAILURE);
        let user = AsynUser::default();
        assert_eq!(axis.polled(&user).unwrap().position, 50000.0);

        dead.store(true, Ordering::SeqCst);
        sent.lock().unwrap().clear();
        let status = axis.polled(&user).unwrap();
        // C sends `TP;` only after `MS;` succeeded.
        assert_eq!(*sent.lock().unwrap(), ["MS;"]);
        assert!(status.comms_error);
        assert_eq!(status.position, 50000.0, "a dead line reads back as 0");
        assert!(status.done);
    }

    /// drvMM4000Asyn.c:653-656: a pass whose `MS;` fails sets only
    /// motorAxisCommError and calls it back, the first pass too.
    #[test]
    fn a_failed_first_pass_is_the_comm_error() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let dead = Arc::new(AtomicBool::new(false));
        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Line {
            base: PortDriverBase::new(
                "mm4000_first",
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            dead: dead.clone(),
            reply: Vec::new(),
            sent: sent.clone(),
        };
        let (rt, _jh) = create_port_runtime(
            &claim_port("mm4000_first").unwrap(),
            line,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let ctrl = Mm4000Controller::new(io, "mm4000_first", 1, Duration::ZERO)
            .ok()
            .unwrap();
        let mut axis = Mm4000Axis::new(Arc::new(Mutex::new(ctrl)), 0, 1, MODEL2_POLL_FAILURE);
        dead.store(true, Ordering::SeqCst);
        sent.lock().unwrap().clear();
        let status = axis.polled(&AsynUser::default()).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;"]);
        assert!(status.comms_error);
        let params = axis.params();
        assert_eq!(params.get_integer(function::COMM_ERROR), Ok(Some(1)));
        assert_eq!(params.get_integer(function::DONE), Ok(None));
    }

    /// A two-axis [`Scripted`] MM4000 on port `name` whose pass is cached
    /// for an hour, both its axes, and what it put on the wire.
    fn scripted_pair(
        name: &'static str,
        silent: &[&'static str],
        tp: &'static str,
    ) -> (Vec<Mm4000Axis>, Arc<Mutex<Vec<String>>>) {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Scripted {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            silent: Arc::new(Mutex::new(silent.to_vec())),
            tp: Arc::new(Mutex::new(tp)),
            reply: Vec::new(),
            sent: sent.clone(),
        };
        let (rt, _jh) =
            create_port_runtime(&claim_port(name).unwrap(), line, RuntimeConfig::default())
                .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_millis(100));
        let ttl = Duration::from_secs(3600);
        let ctrl = Mm4000Controller::new(io, name, 2, ttl).ok().unwrap();
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axes = (1..=2)
            .map(|axis| Mm4000Axis::new(ctrl.clone(), 0, axis, MODEL2_POLL_FAILURE))
            .collect();
        sent.lock().unwrap().clear();
        (axes, sent)
    }

    /// drvMM4000Asyn.c:705: MM4000Poller asks `TE;` in each axis's turn.
    #[test]
    fn each_axis_of_the_pass_asks_te() {
        let (mut axes, sent) = scripted_pair("mm4000_te_each", &[], "1TP1.4990,2TP-2.2490");
        let user = AsynUser::default();
        axes[0].polled(&user).unwrap();
        axes[1].polled(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TP;", "TE;", "TE;"]);
        for axis in &axes {
            let params = axis.params();
            assert_eq!(params.get_integer(function::COMM_ERROR), Ok(Some(0)));
            assert_eq!(params.get_integer(function::PROBLEM), Ok(Some(0)));
        }
    }

    /// drvMM4000Asyn.c:651-718: a failed `TE;` still leaves its axis read,
    /// PROBLEM raised, and its status is every later axis's `comStatus`, which
    /// sets only motorAxisCommError there.
    #[test]
    fn a_failed_te_fails_every_later_axis_of_the_pass() {
        let (mut axes, sent) = scripted_pair("mm4000_te", &["TE;"], "1TP3.0000,2TP-2.2490");
        let user = AsynUser::default();
        let first = axes[0].polled(&user).unwrap();
        axes[1].polled(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TP;", "TE;"]);
        let params = axes[0].params();
        assert_eq!(params.get_integer(function::COMM_ERROR), Ok(Some(0)));
        assert_eq!(params.get_integer(function::PROBLEM), Ok(Some(1)));
        assert_eq!(params.get_integer(function::DONE), Ok(Some(1)));
        assert_eq!(params.get_double(function::POSITION), Ok(Some(3.0 / 0.001)));
        assert!(first.problem && !first.comms_error);
        let params = axes[1].params();
        assert_eq!(params.get_integer(function::COMM_ERROR), Ok(Some(1)));
        assert_eq!(params.get_integer(function::PROBLEM), Ok(None));
        assert_eq!(params.get_integer(function::DONE), Ok(None));
        assert_eq!(params.get_double(function::POSITION), Ok(None));
    }

    /// MM4000Poller sets a 0 `TU`'s axis like any other and divides its
    /// position by the 0 (drvMM4000Asyn.c:659-716); the position holds
    /// instead and the axis is a problem.
    #[test]
    fn a_zero_tu_reads_the_status_and_holds_the_position() {
        let (mut axes, sent) = scripted_pair("mm4000_tu0", &["1TU"], "1TP3.0000,2TP-2.2490");
        let user = AsynUser::default();
        let first = axes[0].polled(&user).unwrap();
        axes[1].polled(&user).unwrap();
        assert_eq!(*sent.lock().unwrap(), ["MS;", "TP;", "TE;", "TE;"]);
        let params = axes[0].params();
        assert_eq!(params.get_integer(function::COMM_ERROR), Ok(Some(0)));
        assert_eq!(params.get_integer(function::DONE), Ok(Some(1)));
        assert_eq!(params.get_integer(function::PROBLEM), Ok(Some(1)));
        assert_eq!(params.get_double(function::POSITION), Ok(None));
        assert!(first.done && first.problem && !first.comms_error);
        let params = axes[1].params();
        assert_eq!(params.get_integer(function::PROBLEM), Ok(Some(0)));
        assert_eq!(
            params.get_double(function::POSITION),
            Ok(Some(-2.249 / 0.001))
        );
    }

    /// drvMM4000Asyn.c:485-486 motorAxisMove clears motorAxisDone, calls it
    /// back and wakes the poller: the poll after a move reads the wire and
    /// sets done from `MS;` alone (:661), the one after it is inside the TTL.
    #[test]
    fn the_poll_after_a_move_is_fresh_and_reads_ms() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Line {
            base: PortDriverBase::new(
                "mm4000_moved",
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            ),
            dead: Arc::new(AtomicBool::new(false)),
            reply: Vec::new(),
            sent: sent.clone(),
        };
        let (rt, _jh) = create_port_runtime(
            &claim_port("mm4000_moved").unwrap(),
            line,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let ttl = Duration::from_secs(3600);
        let ctrl = Mm4000Controller::new(io, "mm4000_moved", 1, ttl)
            .ok()
            .unwrap();
        let mut axis = Mm4000Axis::new(Arc::new(Mutex::new(ctrl)), 0, 1, MODEL2_POLL_FAILURE);
        let user = AsynUser::default();
        assert!(axis.polled(&user).unwrap().done);
        sent.lock().unwrap().clear();
        assert!(axis.polled(&user).unwrap().done);
        assert!(sent.lock().unwrap().is_empty(), "inside the TTL");

        axis.move_absolute(&user, 1000.0, 0.0, 10.0, 20.0).unwrap();
        assert_eq!(axis.params().get_integer(function::DONE), Ok(Some(0)));
        let after = axis.polled(&user).unwrap();
        assert_eq!(
            *sent.lock().unwrap(),
            ["1AC0.002000;1VA0.001000;1PA0.100000;", "MS;", "TP;", "TE;"]
        );
        assert!(after.done, "the line's MS; reports the axis idle");

        sent.lock().unwrap().clear();
        assert!(axis.polled(&user).unwrap().done);
        assert!(sent.lock().unwrap().is_empty());
    }

    /// C `MM4000AsynConfig` returns before any axis on a silent `VE;`
    /// (drvMM4000Asyn.c:817-825) and on fewer `TP;` fields than numAxes
    /// (:853-857).
    #[test]
    fn config_refusals_create_no_controller() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        for (name, dead, num_axes) in [("mm4000_silent", true, 1), ("mm4000_short", false, 2)] {
            let line = Line {
                base: PortDriverBase::new(
                    name,
                    1,
                    PortFlags::default(),
                    InterfaceMask::OCTET_TRANSPORT,
                    InterfaceMask::OCTET,
                ),
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
                Mm4000Controller::new(io, name, num_axes, Duration::ZERO).is_err(),
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

    /// drvMM4000Asyn.c:579-592: a GET_INFO's motorAxisforceCallback prints
    /// its FLOW line, 0-based axis, on the axis's log.
    #[epics_rs::base::epics_test]
    async fn force_callback_prints_the_c_flow_line() {
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, Model2Axis, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("mm4kForceTest", 0);
        let (_line, io) = LineRig::new("mm4kForceLine", |last| {
            match last {
                "VE;" => "VEMM4005 Version 2.0",
                "TP;" => "1TP0.0,2TP0.0",
                _ => "",
            }
            .to_string()
        });
        let ctrl = Mm4000Controller::new(io, "mm4kForceLine", 2, Duration::ZERO)
            .ok()
            .unwrap();
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axes: Vec<_> = (1..=2)
            .map(|axis| Mm4000Axis::new(ctrl.clone(), 2, axis, MODEL2_POLL_FAILURE))
            .collect();
        let axes: Vec<_> = axes.into_iter().map(|a| Arc::new(Mutex::new(a))).collect();
        let model2 = axes
            .iter()
            .map(|a| {
                let (getter, log) = {
                    let a = a.lock().unwrap();
                    (a.params(), a.log())
                };
                Model2Axis::new(a.clone(), getter, log)
            })
            .collect();
        DRIVER.start_card(2, model2, 100, 1000).unwrap();
        let args = vec![
            ArgValue::String("mm4kForce".into()),
            ArgValue::String("mm4kForceTest".into()),
            ArgValue::Int(2),
            ArgValue::Int(2),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&DRIVER]), args).await;

        let path = std::env::temp_dir().join(format!("mm4k-force-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("mm4kForce"), TraceMask::FLOW);
        trace.set_device_trace_mask("mm4kForce", 1, TraceMask::FLOW);
        trace.set_device_trace_info_mask("mm4kForce", 1, TraceInfoMask::empty());
        trace.set_device_trace_file("mm4kForce", 1, TraceFile::File(file));
        axes[1].lock().unwrap().force_callback(&AsynUser::default());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            text,
            "motorAxisforceCallback: request card 2, axis 1 status update\n"
        );
    }

    /// Two model-2 axes of card 2 on port `port`, their logs attached by
    /// drvAsynMotorConfigure, axis 1's trace going to a file under `mask`;
    /// `te` is the `TE;` reply.
    async fn traced_axes(
        port: &'static str,
        mask: TraceMask,
        te: &'static str,
    ) -> (Vec<Arc<Mutex<Mm4000Axis>>>, std::path::PathBuf) {
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, Model2Axis, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        let driver: &'static Model2 = Box::leak(Box::new(Model2::new(port, 0)));
        let line: &'static str = Box::leak(format!("{port}Line").into_boxed_str());
        let (_line, io) = LineRig::new(line, |last| {
            match last {
                "VE;" => "VEMM4005 Version 2.0",
                "MS;" => "1MS@,2MSE",
                "TP;" => "1TP0.0,2TP-1.25",
                "TE;" => te,
                "1TU" => "1TU0.001",
                "2TU" => "2TU0.001",
                "2TL" => "2TL-10.0",
                "2TR" => "2TR20.0",
                _ => "",
            }
            .to_string()
        });
        let ctrl = Mm4000Controller::new(io, line, 2, Duration::ZERO)
            .ok()
            .unwrap();
        let ctrl = Arc::new(Mutex::new(ctrl));
        let axes: Vec<_> = (1..=2)
            .map(|axis| Mm4000Axis::new(ctrl.clone(), 2, axis, MODEL2_POLL_FAILURE))
            .collect();
        let axes: Vec<_> = axes.into_iter().map(|a| Arc::new(Mutex::new(a))).collect();
        let model2 = axes
            .iter()
            .map(|a| {
                let (getter, log) = {
                    let a = a.lock().unwrap();
                    (a.params(), a.log())
                };
                Model2Axis::new(a.clone(), getter, log)
            })
            .collect();
        // Idle polls an hour apart, so only the test's own calls trace.
        driver.start_card(2, model2, 3_600_000, 3_600_000).unwrap();
        let args = vec![
            ArgValue::String(port.into()),
            ArgValue::String(port.into()),
            ArgValue::Int(2),
            ArgValue::Int(2),
        ];
        run_command(drv_asyn_motor_configure_command(vec![driver]), args).await;

        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), mask);
        trace.set_device_trace_mask(port, 1, mask);
        trace.set_device_trace_info_mask(port, 1, TraceInfoMask::empty());
        trace.set_device_trace_file(port, 1, TraceFile::File(file));
        (axes, path)
    }

    fn take_trace(path: &std::path::Path) -> String {
        let text = std::fs::read_to_string(path).unwrap();
        let _ = std::fs::remove_file(path);
        text
    }

    /// drvMM4000Asyn.c:384-391, 653-718: a write C acts on goes into the
    /// parameters the port reads back, a refused one does not, and the
    /// poller posts C's masked status bits.
    #[epics_rs::base::epics_test]
    async fn the_port_reads_back_what_the_driver_keeps() {
        let (axes, _path) = traced_axes("mm4kParams", TraceMask::ERROR, "TE@").await;
        let port = PortManager::global()
            .find_port_handle("mm4kParams")
            .unwrap();
        let reads = std::thread::spawn(move || {
            let limit = port.write_float64_blocking(function::LOW_LIMIT as usize, 1, -500.0);
            let gain = port.write_float64_blocking(function::P_GAIN as usize, 1, 0.5);
            (
                limit.is_ok(),
                gain.is_err(),
                port.read_float64_blocking(function::LOW_LIMIT as usize, 1)
                    .ok(),
                port.read_float64_blocking(function::P_GAIN as usize, 1)
                    .ok(),
            )
        })
        .join()
        .unwrap();
        assert_eq!(reads, (true, true, Some(-500.0), Some(0.0)));
        let mut axis = axes[1].lock().unwrap();
        axis.polled(&AsynUser::default()).unwrap();
        let params = axis.params();
        // `E`: moving, plus direction.
        assert_eq!(params.get_integer(function::DIRECTION), Ok(Some(4)));
        assert_eq!(params.get_integer(function::DONE), Ok(Some(0)));
        assert_eq!(params.get_integer(function::POWER_ON), Ok(Some(1)));
        assert_eq!(params.get_double(function::POSITION), Ok(Some(-1250.0)));
        assert_eq!(params.get_integer(function::COMM_ERROR), Ok(Some(0)));
    }

    /// drvMM4000Asyn.c:671-678: motorAxisDone is the status byte's moving
    /// bit, also on the poll after a move.
    #[epics_rs::base::epics_test]
    async fn the_done_parameter_is_the_moving_bit() {
        let (axes, _path) = traced_axes("mm4kDone", TraceMask::ERROR, "TE@").await;
        let user = AsynUser::default();
        let mut axis = axes[0].lock().unwrap();
        axis.move_absolute(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        let status = axis.polled(&user).unwrap();
        assert!(status.done);
        assert_eq!(axis.params().get_integer(function::DONE), Ok(Some(1)));
    }

    /// drvMM4000Asyn.c:344-381: the encoder ratio and the gains are refused
    /// with an ERROR line on the axis's log.
    #[epics_rs::base::epics_test]
    async fn refusals_print_the_c_error_lines() {
        let (axes, path) = traced_axes("mm4kRefuse", TraceMask::ERROR, "TE@").await;
        let user = AsynUser::default();
        let mut axis = axes[1].lock().unwrap();
        assert!(axis.set_encoder_ratio(&user, 2.0).is_err());
        for kind in [
            PidGainKind::Proportional,
            PidGainKind::Integral,
            PidGainKind::Derivative,
        ] {
            assert!(axis.set_pid_gain(&user, kind, 0.5).is_err());
        }
        assert_eq!(
            take_trace(&path),
            "motorAxisSetDouble: MM4000 does not support setting encoder ratio\n\
             MM4000 does not support setting proportional gain\n\
             MM4000 does not support setting integral gain\n\
             MM4000 does not support setting derivative gain\n"
        );
    }

    /// drvMM4000Asyn.c:351, 384, 443: the asyn writes of a
    /// function the driver does not set print an ERROR line and fail.
    #[epics_rs::base::epics_test]
    async fn unknown_functions_print_the_c_error_lines() {
        let (_axes, path) = traced_axes("mm4kUnknown", TraceMask::ERROR, "TE@").await;
        let port = PortManager::global()
            .find_port_handle("mm4kUnknown")
            .unwrap();
        std::thread::spawn(move || {
            assert!(port.write_int32_blocking(26, 1, 1).is_err());
            assert!(port.write_float64_blocking(27, 1, 3.0).is_err());
            assert!(port.write_float64_blocking(1, 1, 0.5).is_err());
        })
        .join()
        .unwrap();
        assert_eq!(
            take_trace(&path),
            "motorAxisSetInteger: unknown function 26\n\
             motorAxisSetDouble: unknown function 27\n\
             motorAxisSetDouble: MM4000 does not support setting resolution\n"
        );
    }

    /// drvMM4000Asyn.c:466, 504, 532-535, 567: a move, a jog either way, a
    /// home and a stop print their FLOW lines, 0-based axis, unscaled.
    #[epics_rs::base::epics_test]
    async fn motion_commands_print_the_c_flow_lines() {
        let (axes, path) = traced_axes("mm4kMotion", TraceMask::FLOW, "TE@").await;
        let user = AsynUser::default();
        let mut axis = axes[1].lock().unwrap();
        axis.move_absolute(&user, 1500.0, 10.0, 200.0, 400.0)
            .unwrap();
        axis.move_relative(&user, -25.5, 0.0, 100.0, 50.0).unwrap();
        axis.move_velocity(&user, 1.0, 300.0, 60.0).unwrap();
        axis.move_velocity(&user, 1.0, -0.0, 60.0).unwrap();
        axis.home(&user, 0.0, 100.0, 20.0, true).unwrap();
        axis.stop(&user, 80.0).unwrap();
        assert_eq!(
            take_trace(&path),
            "Set card 2, axis 1 move to 1500.000000, min vel=10.000000, max_vel=200.000000, accel=400.000000\n\
             Set card 2, axis 1 move to -25.500000, min vel=0.000000, max_vel=100.000000, accel=50.000000\n\
             Set card 2, axis 1 move to 20000.000000, min vel=1.000000, max_vel=300.000000, accel=60.000000\n\
             Set card 2, axis 1 move to -10000.000000, min vel=1.000000, max_vel=0.000000, accel=60.000000\n\
             motorAxisHome: set card 2, axis 1 to home\n\
             Set card 2, axis 1 to stop with accel=80.000000\n"
        );
    }

    /// drvMM4000Asyn.c:700: each poll prints the axis's status byte and its
    /// position in controller units.
    #[epics_rs::base::epics_test]
    async fn the_poll_prints_the_c_iodriver_line() {
        let (axes, path) = traced_axes("mm4kPoll", TraceMask::IO_DRIVER, "TE@").await;
        axes[1]
            .lock()
            .unwrap()
            .polled(&AsynUser::default())
            .unwrap();
        // The poller's own startup poll may land in the file too.
        let text = take_trace(&path);
        assert!(!text.is_empty());
        for line in text.split_inclusive('\n') {
            assert_eq!(
                line,
                "MM4000Poller: axis 1 axisStatus=45, position=-1.250000\n"
            );
        }
    }

    /// drvMM4000Asyn.c:713: the poller's error lines print on each axis's
    /// logParam, which drvAsynMotorConfigure points at the motor port.
    #[epics_rs::base::epics_test]
    async fn the_poller_errors_print_on_the_axis_log() {
        let (axes, path) = traced_axes("mm4kTeErr", TraceMask::ERROR, "TEC").await;
        axes[0]
            .lock()
            .unwrap()
            .polled(&AsynUser::default())
            .unwrap();
        // The poller's own startup pass may land in the file too.
        let text = take_trace(&path);
        assert!(!text.is_empty());
        for line in text.split_inclusive('\n') {
            assert_eq!(line, "MM4000Poller: controller error TEC\n");
        }
    }

    /// drvMM4000Asyn.c:846-896 and :427 ignore the `sendAndReceive` status
    /// and parse what arrived: a reply cut by a timeout still counts the
    /// axes, sets the step size and limits, and ends the torque-off wait.
    #[test]
    fn replies_cut_by_a_timeout_are_still_parsed() {
        use motor_common::testing::LineRig;

        let (_line, io) = LineRig::cut(
            "mm4000_cut",
            |last| {
                match last {
                    "VE;" => "VEMM4005 Version 2.0",
                    "TP;" => "1TP0.0,2TP-1.25",
                    "MS;" => "1MS@,2MS@",
                    "1TU" => "1TU0.001",
                    "1TL" => "1TL-10.0",
                    "1TR" => "1TR20.0",
                    _ => "",
                }
                .to_string()
            },
            |last| last != "VE;",
        );
        let Ok(ctrl) = Mm4000Controller::new(io, "mm4000_cut", 2, Duration::ZERO) else {
            panic!("a cut TP; still counts two axes");
        };
        let mut axis = Mm4000Axis::new(Arc::new(Mutex::new(ctrl)), 0, 1, MODEL2_POLL_FAILURE);
        let report = axis.report();
        assert_eq!(
            (report.step_size, report.low_limit, report.high_limit),
            (0.001, -10.0, 20.0)
        );
        axis.set_closed_loop(&AsynUser::default(), false)
            .expect("a cut MS; still reads the power bit");
    }
}
