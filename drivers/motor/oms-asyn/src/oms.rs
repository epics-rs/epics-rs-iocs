//! Pro-Dex OMS MAXnet / MXA motor controller driver (ASCII, over an asyn octet
//! port).
//!
//! Ported from `motorOmsAsyn/omsAsynApp/src` (`omsBaseController` /
//! `omsBaseAxis` plus the `omsMAXnet` / `omsMXA` transports, all model-3). The
//! MAXnet and MXA speak the identical OMS ASCII command set over a serial or
//! TCP `drvAsyn*Port`; they differ only in a label and minimum firmware, so one
//! driver covers both. The VME `omsMAXv` sibling is register/interrupt-mapped
//! (not an octet transport) and is out of scope.
//!
//! ## Protocol
//!
//! Commands are `;`-separated OMS mnemonics prefixed with an axis selector:
//! `A<char>` selects one axis (chars `X Y Z T U V R S W K` for axes 0..9), `AM`
//! addresses all. Commands go out bare and are framed by the port EOS (see
//! `crate::ioc` for the defaults). Accepted
//! commands are acknowledged with a leading `0x06` (stripped here).
//!
//! The controller also emits unsolicited `"%000 ..."` *notification* lines when
//! a move completes. C counts them in an asyn interrupt handler fed by the
//! reads its poller makes while it waits; here the poller's wait reads the
//! line itself, and a notification wakes it. One that arrives ahead of a
//! command's reply is skipped, and wakes the poller too.
//!
//! ## Polling
//!
//! As in C, one poller thread per controller ([`crate::poller`]) reads the
//! status of every axis at once, owns the axes' status parameters and posts
//! each axis at the end of its loop. The axes' motor poll loops have no
//! timed pass; [`OmsAxis::poll`] wakes the poller and returns the parameters
//! without touching the line.
//!
//! ## Units
//!
//! Positions, encoder counts, velocities and accelerations are controller-
//! native steps / steps-per-second; with the motor record's `MRES` = 1 a step
//! is one EGU, so the record's EGU values pass straight through (clamped to the
//! OMS ranges: velocity 1..4000000, home velocity 1..1000000, acceleration
//! 1..8000000, position magnitude <= 67000000). There is no resolution factor
//! to cancel: the boundary is controller-native steps with `MRES` = 1.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::exception::{AsynException, ExceptionCallbackGuard};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::motor::controller_port::{
    AxisUser, FinishedWrite, ParamWrite, WriteHook, WriteOutcome, WriteValue, index,
};
use epics_rs::motor::trace::MotorTrace;

use motor_common::util::{atoi, atol};
use motor_common::{ControllerParams, ControllerReport};

use crate::poller::PollShared;

/// C `omsTotalControllerNumber`.
static CONTROLLERS: AtomicUsize = AtomicUsize::new(0);

/// Reply read buffer size (C `OMSINPUTBUFFERLEN` = 12*10 + 2).
const READ_BUF: usize = 256;

/// Per-axis command selector characters (C `axisChrArr`).
pub(crate) const AXIS_CHARS: [char; 10] = ['X', 'Y', 'Z', 'T', 'U', 'V', 'R', 'S', 'W', 'K'];

/// Move parameter clamps (C `omsBaseAxis`).
const MAX_VELOCITY: i32 = 4_000_000;
const MAX_HOME_VELOCITY: i32 = 1_000_000;
const MAX_ACCEL: i32 = 8_000_000;
const MAX_POSITION: i32 = 67_000_000;

/// C `fwMajor`, `fwMinor`, `fwRevision`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FirmwareVersion {
    pub major: i32,
    pub minor: i32,
    pub revision: i32,
}

/// `sscanf(p, "ver:%d.%d.%d,")` after C's `ver:B`, `ver:S`, `ver:` search,
/// needing the first two numbers; the revision stays 0 when absent.
fn parse_firmware_version(reply: &str) -> Option<FirmwareVersion> {
    let at = ["ver:B", "ver:S", "ver:"]
        .iter()
        .find_map(|tag| reply.find(tag).map(|i| i + tag.len()))?;
    let (major, rest) = scan_int(&reply[at..])?;
    let (minor, rest) = scan_int(rest.strip_prefix('.')?)?;
    let revision = rest
        .strip_prefix('.')
        .and_then(scan_int)
        .map_or(0, |(revision, _)| revision);
    Some(FirmwareVersion {
        major,
        minor,
        revision,
    })
}

/// `sscanf`'s `%d`: blanks, an optional sign, at least one digit.
fn scan_int(s: &str) -> Option<(i32, &str)> {
    let t = s.trim_start();
    let sign = usize::from(t.starts_with(['+', '-']));
    let digits = t[sign..].bytes().take_while(u8::is_ascii_digit).count();
    if digits == 0 {
        return None;
    }
    let end = sign + digits;
    Some((t[..end].parse().ok()?, &t[end..]))
}

impl FirmwareVersion {
    /// C `firmwareMin(major, minor, revision)`: this version is at least
    /// `min`.
    pub fn at_least(self, min: FirmwareVersion) -> bool {
        (min.major, min.minor, min.revision) <= (self.major, self.minor, self.revision)
    }
}

fn oms_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// What the Config command fills once the port exists: the version once
/// read, the controller, and the axes once Init has created them.
#[derive(Default)]
pub struct OmsReportState {
    pub(crate) fw: OnceLock<FirmwareVersion>,
    pub(crate) controller: OnceLock<Arc<Mutex<OmsController>>>,
    pub(crate) axes: OnceLock<Arc<PollShared>>,
}

/// The constructor's `createParam`s (omsBaseController.cpp:101-104).
#[derive(Clone, Copy, Default)]
struct OmsReasons {
    poll: usize,
    send: usize,
    send_receive: usize,
    receive: usize,
}

/// `omsBaseController::writeInt32` / `writeFloat64`'s closing line
/// (omsBaseController.cpp:306-313, 409-416).
fn closing_trace(
    print: impl Fn(TraceMask, std::fmt::Arguments<'_>),
    function: usize,
    value: WriteValue,
    status: &AsynResult<()>,
) {
    let (method, value) = match value {
        WriteValue::Int32(v) => ("writeInt32", v.to_string()),
        WriteValue::Float64(v) => ("writeFloat64", format!("{v:.6}")),
    };
    match status {
        Err(e) => print(
            TraceMask::ERROR,
            format_args!(
                "omsBaseDriver:{method}: error, status={} function={function}, value={value}\n",
                e.c_status_code()
            ),
        ),
        Ok(()) => print(
            TraceMask::IO_DRIVER,
            format_args!("omsBaseDriver:{method}: function={function}, value={value}\n"),
        ),
    }
}

/// The closing line of every writeFloat64, and of the writeInt32s C
/// handles itself rather than returning the base's (omsBaseController.cpp:
/// 266-305); OMS_POLL prints its own in [`OmsParams::write_int32`].
struct OmsWriteHook;

impl WriteHook for OmsWriteHook {
    /// The writes C's branches take without calling the base, which
    /// prints nothing for them (omsBaseController.cpp:262-295, 333-406).
    fn handles(&self, function: usize) -> bool {
        matches!(
            function,
            index::DEFER_MOVES
                | index::CLOSED_LOOP
                | index::MOVE_TO_HOME
                | index::ENCODER_POSITION
                | index::RESOLUTION
                | index::LOW_LIMIT
                | index::HIGH_LIMIT
                | index::PGAIN
                | index::IGAIN
                | index::DGAIN
        )
    }

    fn after_write(&self, write: &FinishedWrite<'_>) -> WriteOutcome {
        let own = match write.value {
            WriteValue::Int32(_) => matches!(
                write.function,
                index::DEFER_MOVES | index::CLOSED_LOOP | index::MOVE_TO_HOME
            ),
            WriteValue::Float64(_) => true,
        };
        if own {
            closing_trace(
                |mask, args| write.print(mask, args),
                write.function,
                write.value,
                write.status,
            );
        }
        WriteOutcome::default()
    }
}

/// MOTOR_STATUS_HAS_ENCODER, MSTA bit 8.
const HAS_ENCODER: usize = index::STATUS_DIRECTION + 8;

/// C `MAX_STRING_SIZE`, the size of writeOctet's reply buffer.
const MAX_STRING_SIZE: usize = 40;

/// The controller's own parameters and report, and the state the Config
/// command fills.
pub struct OmsParams {
    controller_type: &'static str,
    state: Arc<OmsReportState>,
    reasons: OmsReasons,
}

impl OmsParams {
    pub fn new(controller_type: &'static str) -> (Self, Arc<OmsReportState>) {
        let state = Arc::new(OmsReportState::default());
        let params = Self {
            controller_type,
            state: state.clone(),
            reasons: OmsReasons::default(),
        };
        (params, state)
    }

    /// C `getAxis(pasynUser)`: the axis Init created at the address.
    fn axis(&self, user: AxisUser<'_>) -> Option<i32> {
        let count = self.state.axes.get().map_or(0, |a| a.axis_count());
        Some(user.addr).filter(|&addr| (addr as usize) < count)
    }
}

impl ControllerParams for OmsParams {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        // Address 0's list alone (omsBaseController.cpp:95-98).
        self.reasons = OmsReasons {
            poll: base.create_param_in_list(0, "OMS_POLL", ParamType::Int32)?,
            send: base.create_param_in_list(0, "OMS_STRING_SEND", ParamType::Octet)?,
            send_receive: base.create_param_in_list(0, "OMS_STRING_SENDRECV", ParamType::Octet)?,
            receive: base.create_param_in_list(0, "OMS_STRING_RECV", ParamType::Octet)?,
        };
        let r = self.reasons;
        for reason in [r.send, r.send_receive, r.receive] {
            base.set_string_param(reason, 0, "")?;
        }
        Ok(())
    }

    /// C `omsBaseController::writeInt32`'s OMS_POLL, which wakes the
    /// poller (omsBaseController.cpp:275-308).
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: i32,
    ) -> ParamWrite {
        if user.reason != self.reasons.poll {
            return ParamWrite::PassToBase(Ok(()));
        }
        let Some(axis) = self.axis(user) else {
            return ParamWrite::Handled(Err(oms_err("oms: no axis at this address")));
        };
        // Past address 0 the set fails, and that is the write's status.
        let set = base.set_int32_param(user.reason, axis, value);
        if value != 0
            && let Some(axes) = self.state.axes.get()
        {
            axes.wakeup();
        }
        let _ = base.call_param_callbacks(axis);
        closing_trace(
            |mask, args| {
                PortManager::global().trace_manager().print(
                    Some(&base.port_name),
                    Some(user.asyn().addr),
                    user.reason as i32,
                    mask,
                    args,
                )
            },
            user.reason,
            WriteValue::Int32(value),
            &set,
        );
        ParamWrite::Handled(set)
    }

    /// C `omsBaseController::readInt32` (omsBaseController.cpp:225-254):
    /// the position, and an encoder axis's encoder position, read from the
    /// controller.
    fn read_int32(&mut self, base: &mut PortDriverBase, user: AxisUser<'_>) -> AsynResult<i32> {
        let Some(axis) = self.axis(user) else {
            return Err(oms_err("oms: no axis at this address"));
        };
        let query = match user.reason {
            index::POSITION => "RP",
            index::ENCODER_POSITION => {
                if base.get_int32_param(HAS_ENCODER, axis).unwrap_or(0) == 0 {
                    // C answers asynSuccess without touching *value; the
                    // stored encoder position stands in for that value.
                    return Ok(base.get_float64_param(index::ENCODER_POSITION, axis)? as i32);
                }
                "RE"
            }
            _ => return base.read_int32_param(user.asyn()),
        };
        let Some(controller) = self.state.controller.get() else {
            return Err(oms_err("oms: controller not connected"));
        };
        // C ignores sendReceiveReplace's status and converts whatever
        // inputBuffer holds.
        let (reply, _) = controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .send_receive_buffer(&format!("A{} {query}", AXIS_CHARS[axis as usize]));
        Ok(atol(&reply) as i32)
    }

    fn write_hook(&self) -> Option<Arc<dyn WriteHook>> {
        Some(Arc::new(OmsWriteHook))
    }

    /// C `omsBaseController::writeOctet` (omsBaseController.cpp:178-219):
    /// OMS_STRING_SEND goes to the controller, OMS_STRING_SENDRECV too with
    /// its reply posted as OMS_STRING_RECV; neither is stored, and any
    /// other octet write is taken and dropped.
    fn write_octet(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        data: &[u8],
    ) -> ParamWrite {
        let Some(axis) = self.axis(user) else {
            return ParamWrite::Handled(Err(oms_err("oms: no axis at this address")));
        };
        let r = self.reasons;
        if user.reason != r.send && user.reason != r.send_receive {
            return ParamWrite::Handled(Ok(()));
        }
        let Some(controller) = self.state.controller.get() else {
            return ParamWrite::Handled(Err(oms_err("oms: controller not connected")));
        };
        let value = data.split(|&b| b == 0).next().unwrap_or_default();
        let value = String::from_utf8_lossy(value);
        let trace = MotorTrace::new(&*base.port_name, user.asyn().addr);
        let print =
            |args: std::fmt::Arguments<'_>| trace.print(TraceMask::ERROR, user.reason as i32, args);
        let port = base.port_name.to_string();
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
        if user.reason == r.send {
            let sent = ctrl.send_only(&value);
            if sent.is_err() {
                print(format_args!(
                    "omsBaseDriver:writeOctet:{port}: send Error\n"
                ));
            }
            return ParamWrite::Handled(sent);
        }
        let (mut reply, status) = ctrl.send_receive_buffer(&value);
        drop(ctrl);
        if status.is_err() {
            print(format_args!(
                "omsBaseDriver:writeOctet:{port}: sendReceive Error\n"
            ));
            return ParamWrite::Handled(status);
        }
        // strncpy into a MAX_STRING_SIZE buffer, its last byte the NUL.
        reply.truncate(
            (0..MAX_STRING_SIZE)
                .rev()
                .find(|&i| reply.is_char_boundary(i))
                .unwrap_or(0),
        );
        // Past address 0 the set fails, and callParamCallbacks' status
        // replaces its own (omsBaseController.cpp:197-198).
        let _ = base.set_string_param(r.receive, axis, &reply);
        let _ = base.call_param_callbacks(axis);
        print(format_args!(
            "omsBaseDriver:writeOctet:{port}: answer is {reply}\n"
        ));
        ParamWrite::Handled(Ok(()))
    }

    /// C `omsBaseController::report` (omsBaseController.cpp:130-161). C
    /// dereferences a null `pAxes` on a controller whose Init failed; here
    /// it has no axis lines.
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let fw = self.state.fw.get().copied().unwrap_or_default();
        let axes = self.state.axes.get();
        let _ = writeln!(
            out,
            "Oms {} motor driver {}, numAxes={}; Firmware: {}.{}.{}",
            self.controller_type,
            ctx.port_name(),
            axes.map_or(ctx.num_axes(), |a| a.axis_count()),
            fw.major,
            fw.minor,
            fw.revision,
        );
        for axis in 0..axes.map_or(0, |a| a.axis_count()) {
            let _ = writeln!(out, "  axis {axis}");
            if level > 0 {
                let addr = axis as i32;
                // An unset parameter prints 0, where C prints an
                // uninitialised local.
                let float = |reason| ctx.float64_param(addr, reason).unwrap_or(0.0);
                let _ = writeln!(
                    out,
                    "    Current position = {:.6}, velocity = {:.6}",
                    float(index::POSITION),
                    float(index::VELOCITY),
                );
                if ctx.int32_param(addr, HAS_ENCODER).unwrap_or(0) != 0 {
                    let _ = writeln!(
                        out,
                        "    Encoder position {:.6}",
                        float(index::ENCODER_POSITION)
                    );
                }
                if axes.is_some_and(|a| a.homing(axis)) {
                    let _ = writeln!(out, "    Currently homing axis");
                }
            }
        }
        ctx.report(out, level);
    }
}

/// A line the controller sends unsolicited on move completion
/// (`"%000 SSSSSSSS"`); C `isNotification` keys on the `"000 0"` substring.
fn is_notification(s: &str) -> bool {
    s.contains("000 0")
}

/// A reply less C's leading ACK/CR/LF and the terminator.
fn trim_reply(raw: &[u8]) -> String {
    String::from_utf8_lossy(raw)
        .trim_start_matches(['\x06', '\r', '\n'])
        .trim_end_matches(['\r', '\n', '\0', ' '])
        .to_string()
}

/// An OMS MAXnet/MXA controller endpoint owning the asyn octet handle, shared by
/// its axes behind a mutex so command/reply pairs stay atomic.
pub struct OmsController {
    handle: SyncIOHandle,
    /// The motor port, C `this->portName`.
    port_name: String,
    /// The octet port `pasynUserSyncIOSerial` prints on.
    serial_port: String,
    /// `"MAXnet"` or `"MXA"` (label only; the protocol is identical).
    controller_type: &'static str,
    /// C `portConnected`, kept by [`Self::watch_connection`]'s callback.
    port_connected: Arc<AtomicBool>,
    /// The `exceptionCallbackAdd` registration; dropped with the controller.
    _connect_callback: Option<ExceptionCallbackGuard>,
    /// C `controllerNumber`, which names the poller thread.
    controller_number: usize,
    /// The axes' status parameters and the poll event.
    poll_shared: Arc<PollShared>,
    /// C `firmwareMin(minFwMajor, minFwMinor, minFwRevision)`, which picks
    /// the command forms of firmware 1.30 on.
    modern: bool,
}

impl OmsController {
    /// Wrap a connected octet handle.
    pub fn new(
        handle: SyncIOHandle,
        port_name: &str,
        serial_port: &str,
        controller_type: &'static str,
    ) -> Self {
        Self {
            handle,
            port_name: port_name.to_owned(),
            serial_port: serial_port.to_owned(),
            controller_type,
            port_connected: Arc::new(AtomicBool::new(false)),
            _connect_callback: None,
            controller_number: CONTROLLERS.fetch_add(1, Ordering::Relaxed),
            poll_shared: Arc::default(),
            modern: true,
        }
    }

    /// Pick the command forms for firmware `fw` on a controller whose
    /// newer forms need `min`.
    pub fn set_firmware(&mut self, fw: FirmwareVersion, min: FirmwareVersion) {
        self.modern = fw.at_least(min);
    }

    /// Whether the firmware takes the 1.30 command forms.
    pub(crate) fn modern(&self) -> bool {
        self.modern
    }

    pub(crate) fn port_name(&self) -> &str {
        &self.port_name
    }

    pub(crate) fn controller_number(&self) -> usize {
        self.controller_number
    }

    pub(crate) fn poll_shared(&self) -> &Arc<PollShared> {
        &self.poll_shared
    }

    /// The constructor's `exceptionCallbackAdd(connectCallback)` and
    /// `isConnected` (omsMAXnet.cpp:105-111, omsMXA.cpp:100-106), on the
    /// serial user at addr 0. `connectCallback`'s trace lines are gated on
    /// `motorMAXnetdebug`/`motorMXAdebug`, which the dbd never registers
    /// (`#variable`), so they cannot fire and are not ported.
    pub fn watch_connection(&mut self) -> AsynResult<()> {
        let port = self.handle.port()?;
        let cells = port.connect_device(0);
        let flag = Arc::clone(&self.port_connected);
        let callback = cells.clone();
        self._connect_callback = Some(port.exception_callback_add(0, move |ev| {
            if ev.exception == AsynException::Connect {
                flag.store(callback.connected(), Ordering::Release);
            }
        }));
        self.port_connected
            .store(cells.connected(), Ordering::Release);
        Ok(())
    }

    /// C `portConnected`.
    pub fn port_connected(&self) -> bool {
        self.port_connected.load(Ordering::Acquire)
    }

    /// C `resetConnection` (omsMAXnet.cpp:332-350, omsMXA.cpp:327-345):
    /// the serial transport's disconnect through the serial user at addr 0,
    /// its connect unless autoConnect will do it, then after 0.1 s a line
    /// if `connectCallback` saw it come back. C ignores both statuses.
    pub fn reset_connection(&self) {
        let Ok(port) = self.handle.port() else {
            return;
        };
        let auto_connect = port.is_auto_connect_blocking(0).unwrap_or(false);
        let user = || AsynUser::new(0).with_addr(0);
        errlog_vprintf(format_args!(
            "*** disconnect and reconnect serial/IP connection ****\n"
        ));
        let _ = port.submit_blocking(RequestOp::Disconnect, user());
        if !auto_connect {
            let _ = port.submit_blocking(RequestOp::Connect, user());
        }
        std::thread::sleep(Duration::from_millis(100));
        if self.port_connected() {
            errlog_vprintf(format_args!("*** reconnect done ****\n"));
        }
    }

    /// Controller label, for logging.
    pub fn controller_type(&self) -> &'static str {
        self.controller_type
    }

    /// Send a command expecting no reply (C `sendOnly`).
    pub fn send_only(&self, cmd: &str) -> AsynResult<()> {
        let e = match self.handle.write_octet(0, cmd.as_bytes()) {
            Ok(_) => return Ok(()),
            Err(e) => e,
        };
        PortManager::global().trace_manager().print(
            Some(&self.serial_port),
            Some(0),
            0,
            TraceMask::ERROR,
            format_args!(
                "drv{}Asyn:sendOnly: error sending command {cmd}, sent={}, status={}\n",
                self.controller_type,
                e.partial_write().unwrap_or(0),
                e.status() as i32
            ),
        );
        Err(e)
    }

    /// Send a command and read one reply, skipping any notification lines and
    /// stripping the leading ACK/CR/LF (C `sendReceive`).
    pub fn send_receive(&self, cmd: &str) -> AsynResult<String> {
        let (reply, status) = self.send_receive_buffer(cmd);
        status.map(|()| reply)
    }

    /// [`Self::send_receive`] as C fills `inputBuff`: a failed read still
    /// leaves the bytes it got there, and C callers that ignore the status
    /// read them.
    pub fn send_receive_buffer(&self, cmd: &str) -> (String, AsynResult<()>) {
        let failed = |e: AsynError| {
            let reply = e
                .partial_read()
                .map_or_else(String::new, |p| trim_reply(&p.data));
            (reply, Err(e))
        };
        let mut raw = match self.handle.write_read(0, cmd.as_bytes(), READ_BUF) {
            Ok(raw) => raw,
            Err(e) => return failed(e),
        };
        // A move notification may sit ahead of the real reply; skip it. Bound
        // the retries so a chatty controller cannot spin forever.
        for _ in 0..12 {
            let trimmed = trim_reply(&raw);
            if self.notification(&trimmed) {
                raw = match self.handle.read_octet(0, READ_BUF) {
                    Ok(raw) => raw,
                    Err(e) => return failed(e),
                };
                continue;
            }
            return (trimmed, Ok(()));
        }
        (
            String::new(),
            Err(oms_err("oms: only notification lines received")),
        )
    }

    /// C `isNotification` (omsMAXnet.cpp:310-327): a notification signals
    /// the poll event, except `"000 01"`, the controller's command error.
    fn notification(&self, line: &str) -> bool {
        if !is_notification(line) {
            return false;
        }
        if line.contains("000 01") {
            println!(
                "oms{}Driver:isNotification:{}: CMD_ERR_FLAG received",
                self.controller_type, self.port_name
            );
        } else {
            self.poll_shared.wakeup();
        }
        true
    }

    /// waitInterruptible's flush of the serial port (omsMAXnet.cpp:184-186).
    pub(crate) fn flush_line(&self) {
        if let Ok(port) = self.handle.port() {
            let _ = port.submit_blocking(RequestOp::Flush, AsynUser::new(0).with_addr(0));
        }
    }

    /// waitInterruptible's zero-timeout read (omsMAXnet.cpp:187-195), whose
    /// `%` `asynCallback` turns into a poll event. C leaves the line for
    /// sendReceive's drain to run `isNotification` on; read here, it is run
    /// here.
    pub(crate) fn poll_line(&self) {
        let Ok(port) = self.handle.port() else {
            return;
        };
        let user = AsynUser::new(0).with_addr(0).with_timeout(Duration::ZERO);
        let read = port.submit_blocking(RequestOp::OctetRead { buf_size: READ_BUF }, user);
        if let Some(data) = read.ok().and_then(|r| r.data)
            && data.contains(&b'%')
        {
            self.poll_shared.wakeup();
            self.notification(&trim_reply(&data));
        }
    }

    /// C `omsBaseController::getFirmwareVersion`: up to three `WY` tries a
    /// second apart, each reply logged, then the version from its `ver:`
    /// field. An unparsable reply fails with an empty message, as C's caller
    /// prints the user's untouched `errorMessage`.
    pub fn firmware_version(&self) -> AsynResult<FirmwareVersion> {
        let (mut reply, mut status) = (String::new(), Err(oms_err("")));
        for _ in 0..3 {
            std::thread::sleep(Duration::from_secs(1));
            (reply, status) = self.send_receive_buffer("WY");
            errlog_vprintf(format_args!("OMS Firmware Version: {reply}\n"));
            if status.is_ok() {
                break;
            }
        }
        let version = parse_firmware_version(&reply);
        if version.is_none() {
            errlog_vprintf(format_args!(
                "omsBaseController::getFirmwareVersion: unable to retrieve Firmware version\n"
            ));
        }
        status.and_then(|()| version.ok_or_else(|| oms_err("")))
    }

    /// Interrupt-clear and stop-all, then send an optional init string (C
    /// `Init` preamble). Called once at controller creation. C ignores each
    /// sendOnly's status here; a failure is only its printed line.
    pub fn init(&self, init_string: &str) {
        let _ = self.send_only("IC;");
        let _ = self.send_only("AM SA;");
        std::thread::sleep(Duration::from_millis(500));
        if !init_string.is_empty() {
            let _ = self.send_only(init_string);
        }
        // Some init commands (like "LT") need time to process.
        std::thread::sleep(Duration::from_millis(500));
    }

    /// The axis count: the fields of the `AA RP;` reply, empty ones skipped
    /// as `epicsStrtok_r` does. C ignores the exchange's status and counts
    /// whatever a failed read left in the buffer.
    pub fn axis_count(&self) -> usize {
        let (reply, _) = self.send_receive_buffer("AA RP;");
        reply.split(',').filter(|f| !f.is_empty()).count()
    }
}

/// One OMS axis sharing a controller. Implements [`AsynMotor`].
pub struct OmsAxis {
    controller: Arc<Mutex<OmsController>>,
    /// 0-based axis index (limit-flag bit position).
    axis_index: usize,
    /// Command selector char (`A<char>`).
    axis_char: char,
    has_encoder: bool,
    /// C `omsBaseAxis::encoderRatio`, set by `setEncoderRatio`.
    encoder_ratio: f64,
    /// Last base velocity sent, to order `VL`/`VB` so `VB < VL` always holds.
    last_min_velo: i32,
    /// The status parameters the controller's poller writes.
    poll_shared: Arc<PollShared>,
}

impl OmsAxis {
    /// Construct axis `index` and probe its type (`PS?`, before firmware
    /// 1.30 `?PS`) and limit true-state (`LT?` or `?LS`), and add its
    /// status parameters. Axes are created in order.
    pub fn new(controller: Arc<Mutex<OmsController>>, index: usize) -> AsynResult<Self> {
        let axis_char = *AXIS_CHARS
            .get(index)
            .ok_or_else(|| oms_err(format!("oms: axis index {index} out of range (max 9)")))?;

        let (has_encoder, poll_shared) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // C ignores both exchanges' status and keeps the axis on a reply
            // it does not know, as an unencoded axis with uninverted limits.
            // Axis type: "=O" stepper/no-encoder, "=M" servo, "=E" stepper+enc.
            let modern = ctrl.modern();
            let ps = if modern { "PS?" } else { "?PS" };
            let (ps, _) = ctrl.send_receive_buffer(&format!("A{axis_char} {ps}"));
            let has_encoder = match ps.as_bytes().get(1) {
                Some(b'O') => false,
                Some(b'M') | Some(b'E') => true,
                _ => {
                    errlog_vprintf(format_args!(
                        "omsBaseController:Init: error: unknown axis type! ({ps})\n"
                    ));
                    false
                }
            };
            // Limit true-state: "=l"/"=L" invert, "=h"/"=H" no invert.
            let lt = if modern { "LT?" } else { "?LS" };
            let (lt, _) = ctrl.send_receive_buffer(&format!("A{axis_char} {lt}"));
            let invert_limit = match lt.as_bytes().get(1) {
                Some(b'l') | Some(b'L') => true,
                Some(b'h') | Some(b'H') => false,
                _ => {
                    errlog_vprintf(format_args!(
                        "omsBaseController:Init: error: unknown limit true state!\n"
                    ));
                    false
                }
            };
            let poll_shared = ctrl.poll_shared().clone();
            poll_shared.add_axis(index, has_encoder, invert_limit);
            (has_encoder, poll_shared)
        };

        Ok(Self {
            controller,
            axis_index: index,
            axis_char,
            has_encoder,
            encoder_ratio: 1.0,
            last_min_velo: 0,
            poll_shared,
        })
    }

    fn lock(&self) -> MutexGuard<'_, OmsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Round to nearest step, away from zero (C `(int)(x +/- 0.5)`).
    fn nint(x: f64) -> i32 {
        if x < 0.0 {
            (x - 0.5) as i32
        } else {
            (x + 0.5) as i32
        }
    }

    /// Shared move body (absolute or relative).
    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        min_v: f64,
        max_v: f64,
        accel: f64,
    ) -> AsynResult<()> {
        // Servo/encoder axes: sync the motor position to the encoder first.
        if self.has_encoder {
            let ctrl = self.lock();
            let ch = self.axis_char;
            let enc = atoi(&ctrl.send_receive(&format!("A{ch};RE;"))?);
            let motor_position = (f64::from(enc) / self.encoder_ratio) as i32;
            ctrl.send_only(&format!("A{ch};LO{motor_position};"))?;
        }

        let pos = Self::nint(position);
        if pos.abs() > MAX_POSITION {
            return Err(oms_err(format!("oms: position {pos} out of range")));
        }
        let velo = Self::nint(max_v).clamp(1, MAX_VELOCITY);
        let minvelo = Self::nint(min_v).clamp(0, velo - 1);
        let acc = (accel.abs() as i32).clamp(1, MAX_ACCEL);
        let relabs = if relative { "MR" } else { "MA" };

        let ch = self.axis_char;
        let cmd = if velo < self.last_min_velo {
            format!("A{ch};AC{acc};VB{minvelo};VL{velo};{relabs}{pos};GO;ID;")
        } else {
            format!("A{ch};AC{acc};VL{velo};VB{minvelo};{relabs}{pos};GO;ID;")
        };
        self.last_min_velo = minvelo;
        self.lock().send_only(&cmd)
    }

    /// C's `asynPrint(pasynUser, ...)` in `omsBaseController::writeInt32`
    /// / `writeFloat64`, through the writer's user at this axis.
    fn print(&self, mask: TraceMask, function: usize, args: std::fmt::Arguments<'_>) {
        let port = self.lock().port_name().to_owned();
        MotorTrace::new(port, self.axis_index as i32).print(mask, function as i32, args);
    }
}

impl AsynMotor for OmsAxis {
    /// [`Self::poll`] reads parameters and cannot fail.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    /// C `omsBaseAxis::setEncoderRatio` (omsBaseAxis.h:37): stored for the
    /// encoder sync before each move.
    fn set_encoder_ratio(&mut self, _user: &AsynUser, ratio: f64) -> AsynResult<()> {
        self.encoder_ratio = ratio;
        Ok(())
    }

    /// C `omsBaseController::writeInt32`'s motorClosedLoop_
    /// (omsBaseController.cpp:260-293).
    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let (port, axis, ch) = (
            self.lock().port_name().to_owned(),
            self.axis_index,
            self.axis_char,
        );
        let flow = if enable {
            format!("omsBaseDriver:writeInt32:{port} axis {axis} closed loop enable\n")
        } else {
            format!("omsBaseDriver:writeInt32:{port} SetInteger axis {axis} closed loop disable\n")
        };
        self.print(TraceMask::FLOW, index::CLOSED_LOOP, format_args!("{flow}"));
        let cmd = match (self.lock().modern(), enable) {
            (true, true) => "CL1",
            (true, false) => "CL0",
            (false, true) => "HN",
            (false, false) => "HF",
        };
        self.lock().send_only(&format!("A{ch} {cmd}"))
    }

    /// C `writeInt32`'s motorDeferMoves_, which it does not implement
    /// (omsBaseController.cpp:254-259).
    fn set_deferred_moves(&mut self, _user: &AsynUser, _defer: bool) -> AsynResult<()> {
        let port = self.lock().port_name().to_owned();
        self.print(
            TraceMask::ERROR,
            index::DEFER_MOVES,
            format_args!("omsBaseDriver:writeInt32:{port} Deferred Move: not yet implemented\n"),
        );
        Ok(())
    }

    /// C `writeFloat64`'s motorEncoderPosition_, an encoder axis only
    /// (omsBaseController.cpp:333-343).
    fn set_encoder_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        if !self.has_encoder {
            return Ok(());
        }
        let (port, axis, ch) = (
            self.lock().port_name().to_owned(),
            self.axis_index,
            self.axis_char,
        );
        self.print(
            TraceMask::FLOW,
            index::ENCODER_POSITION,
            format_args!(
                "omsBaseDriver:writeFloat64:{port} axis {axis} set encoder position to {:.6}\n",
                CFloat(position)
            ),
        );
        self.lock()
            .send_only(&format!("A{ch} LPE{};", position as i32))
    }

    /// C `writeFloat64`'s motorPGain_, motorIGain_ and motorDGain_
    /// (omsBaseController.cpp:361-406). C tests `0.00 >= value`, which
    /// sends only a gain at or below zero; rs takes the range its message
    /// names.
    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let (port, axis, ch) = (
            self.lock().port_name().to_owned(),
            self.axis_index,
            self.axis_char,
        );
        let (function, what, cmd) = match kind {
            PidGainKind::Proportional => (index::PGAIN, "proportional", "KP"),
            PidGainKind::Integral => (index::IGAIN, "integral", "KI"),
            PidGainKind::Derivative => (index::DGAIN, "derivative", "KD"),
        };
        if !(0.0..32768.0).contains(&gain) {
            self.print(
                TraceMask::ERROR,
                function,
                format_args!(
                    "omsBaseDriver:writeFloat64:{port} axis {axis} {what} gain {:.6} not in range 0.0 -> 32768.0\n",
                    CFloat(gain)
                ),
            );
            return Ok(());
        }
        self.print(
            TraceMask::FLOW,
            function,
            format_args!(
                "omsBaseDriver:writeFloat64:{port} axis {axis} set {what} gain to {:.6}\n",
                CFloat(gain)
            ),
        );
        self.lock()
            .send_only(&format!("A{ch} {cmd}{:.6};", CFloat(gain)))
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let moved = self.do_move(position, false, min_velocity, velocity, acceleration);
        self.poll_shared.moved(self.axis_index);
        moved
    }

    /// C keeps MOVE_TO_HOME from asynMotorController's move-to-home thread
    /// (omsBaseController.cpp:293-295), so no move is made.
    fn move_to_home(
        &mut self,
        _user: &AsynUser,
        _position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let moved = self.do_move(distance, true, min_velocity, velocity, acceleration);
        self.poll_shared.moved(self.axis_index);
        moved
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C truncates (int)acceleration / (int)maxVelocity — not round-to-nearest.
        let acc = (acceleration as i32).clamp(1, MAX_ACCEL);
        let velo = (velocity as i32).clamp(-MAX_VELOCITY, MAX_VELOCITY);
        let ch = self.axis_char;
        let moved = self.lock().send_only(&format!("A{ch} AC{acc}; JG{velo};"));
        self.poll_shared.moved(self.axis_index);
        moved
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C home() truncates (int)max_velocity (no +0.5, unlike move()); minvelo
        // does round-to-nearest.
        let velo = (velocity as i32).clamp(1, MAX_HOME_VELOCITY);
        let minvelo = Self::nint(min_velocity).clamp(0, velo - 1);
        let acc = (acceleration.abs() as i32).clamp(1, MAX_ACCEL);
        let dir = if forward { "HM" } else { "HR" };
        let ch = self.axis_char;
        let cmd = if velo < self.last_min_velo {
            format!("A{ch};AC{acc};VB{minvelo};VL{velo};{dir};MA0;GO;ID;")
        } else {
            format!("A{ch};AC{acc};VL{velo};VB{minvelo};{dir};MA0;GO;ID;")
        };
        self.last_min_velo = minvelo;
        let moved = self.lock().send_only(&cmd);
        self.poll_shared.moved(self.axis_index);
        self.poll_shared.homed(self.axis_index);
        moved
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        let mut acc = (acceleration.abs() + 0.5) as i32;
        if acc > MAX_ACCEL {
            acc = MAX_ACCEL;
        }
        if acc < 1 {
            acc = 200_000;
        }
        let ch = self.axis_char;
        self.lock().send_only(&format!("A{ch} AC{acc}; ST ID;"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.poll_shared.set_position(self.axis_index, position);
        let ch = self.axis_char;
        let pos = position as i32;
        self.lock().send_only(&format!("A{ch} LP{pos};"))
    }

    /// C `omsBaseAxis::poll`: wake the poller; the status is the
    /// parameters as it left them.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        self.poll_shared.wakeup();
        Ok(self.poll_shared.status(self.axis_index))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port_handle::PortHandle;

    /// A controller port on the poller tests' one-axis MAXnet line.
    fn params_port(
        name: &str,
        axes: usize,
        replies: &[(&str, &str)],
    ) -> (PortHandle, crate::poller::tests::Rig) {
        let rig = crate::poller::tests::rig(&format!("{name}_IO"), axes, replies);
        let (params, state) = OmsParams::new("MAXnet");
        let _ = state.controller.set(rig.controller.clone());
        let _ = state
            .axes
            .set(rig.controller.lock().unwrap().poll_shared().clone());
        let port = epics_rs::motor::devices::register_controller_port(
            &epics_rs::asyn::registry::open_owner(),
            name,
            2,
            params,
        )
        .unwrap();
        (port, rig)
    }

    fn reason(port: &PortHandle, name: &str) -> usize {
        port.drv_user_create_blocking(&epics_rs::asyn::port::DrvUserRequest::new(name, 0))
            .unwrap()
            .reason
    }

    /// OMS_STRING_SEND goes to the line, OMS_STRING_SENDRECV too with its
    /// reply in OMS_STRING_RECV; neither is stored, other strings are
    /// dropped, and an address with no axis is refused
    /// (omsBaseController.cpp:178-219).
    #[test]
    fn string_writes_go_to_the_controller() {
        let (port, rig) = params_port("omsStrings", 1, &[("AX RP", "1234")]);
        let io = SyncIOHandle::from_handle(port.clone(), 0, Duration::from_secs(1));
        let send = reason(&port, "OMS_STRING_SEND");
        let send_receive = reason(&port, "OMS_STRING_SENDRECV");
        let receive = reason(&port, "OMS_STRING_RECV");
        io.write_octet(send, b"AX ST;").unwrap();
        io.write_octet(send_receive, b"AX RP").unwrap();
        assert_eq!(*rig.written.lock().unwrap(), ["AX ST;", "AX RP"]);
        assert_eq!(io.read_octet(receive, 40).unwrap(), b"1234\0");
        assert_eq!(io.read_octet(send, 40).unwrap(), b"\0");
        assert_eq!(io.read_octet(send_receive, 40).unwrap(), b"\0");
        let message = epics_rs::motor::controller_port::index::PROFILE_BUILD_MESSAGE;
        io.write_octet(message, b"dropped").unwrap();
        assert!(io.read_octet(message, 40).is_err());
        let past = SyncIOHandle::from_handle(port, 1, Duration::from_secs(1));
        assert!(past.write_octet(send, b"AX ST;").is_err());
        assert_eq!(rig.written.lock().unwrap().len(), 2);
    }

    /// A non-zero OMS_POLL wakes the poller (omsBaseController.cpp:298-302).
    #[test]
    fn an_oms_poll_write_wakes_the_poller() {
        let (port, rig) = params_port("omsPollWrite", 1, &[]);
        let shared = rig.controller.lock().unwrap().poll_shared().clone();
        let poll = reason(&port, "OMS_POLL");
        port.write_int32_blocking(poll, 0, 0).unwrap();
        assert!(!shared.wait(Duration::ZERO));
        port.write_int32_blocking(poll, 0, 1).unwrap();
        assert!(shared.wait(Duration::ZERO));
        assert_eq!(port.read_int32_blocking(poll, 0).unwrap(), 1);
    }

    /// C creates OMS_POLL and the strings in address 0's list alone
    /// (omsBaseController.cpp:95-98): another address cannot bind them, an
    /// OMS_POLL write there wakes the poller and then fails on its set
    /// (:264, :298-312), and a string write there goes to the line with its
    /// reply set nowhere (:192-221).
    #[test]
    fn the_oms_parameters_live_in_address_0_s_list_alone() {
        let (port, rig) = params_port("omsListOnly", 2, &[("AX RP", "1234")]);
        let shared = rig.controller.lock().unwrap().poll_shared().clone();
        for name in [
            "OMS_POLL",
            "OMS_STRING_SEND",
            "OMS_STRING_SENDRECV",
            "OMS_STRING_RECV",
        ] {
            let at_1 = epics_rs::asyn::port::DrvUserRequest::new(name, 1);
            assert!(port.drv_user_create_blocking(&at_1).is_err(), "{name}");
        }
        let poll = reason(&port, "OMS_POLL");
        assert!(port.write_int32_blocking(poll, 1, 1).is_err());
        assert!(shared.wait(Duration::ZERO));
        let io = SyncIOHandle::from_handle(port.clone(), 1, Duration::from_secs(1));
        let receive = reason(&port, "OMS_STRING_RECV");
        io.write_octet(reason(&port, "OMS_STRING_SEND"), b"AX ST;")
            .unwrap();
        io.write_octet(reason(&port, "OMS_STRING_SENDRECV"), b"AX RP")
            .unwrap();
        assert_eq!(*rig.written.lock().unwrap(), ["AX ST;", "AX RP"]);
        assert!(io.read_octet(receive, 40).is_err());
        let io = SyncIOHandle::from_handle(port, 0, Duration::from_secs(1));
        assert_eq!(io.read_octet(receive, 40).unwrap(), b"\0");
    }

    /// OMS_POLL's closing line: IO_DRIVER on success, ERROR with the
    /// status of the set that failed past address 0
    /// (omsBaseController.cpp:306-313).
    #[test]
    fn an_oms_poll_write_prints_c_s_closing_line() {
        use epics_rs::asyn::trace::TraceFile;
        let name = "omsPollTrace";
        let (port, _rig) = params_port(name, 2, &[]);
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        // Each address shows one mask: the success line, then the error's.
        for (addr, mask) in [(0, TraceMask::IO_DRIVER), (1, TraceMask::ERROR)] {
            trace.set_device_trace_mask(name, addr, mask);
            trace.set_device_trace_file(name, addr, TraceFile::File(file.clone()));
        }
        let poll = reason(&port, "OMS_POLL");
        port.write_int32_blocking(poll, 0, 0).unwrap();
        assert!(port.write_int32_blocking(poll, 1, 1).is_err());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<&str> = text
            .lines()
            .filter(|l| l.contains("omsBaseDriver"))
            .collect();
        let expected = [
            format!("omsBaseDriver:writeInt32: function={poll}, value=0"),
            format!("omsBaseDriver:writeInt32: error, status=9 function={poll}, value=1"),
        ];
        assert_eq!(lines.len(), expected.len(), "{text}");
        for (line, want) in lines.iter().zip(&expected) {
            assert!(line.ends_with(want.as_str()), "{line:?} vs {want:?}");
        }
    }

    #[test]
    fn nint_rounds_away_from_zero() {
        assert_eq!(OmsAxis::nint(1.4), 1);
        assert_eq!(OmsAxis::nint(1.5), 2);
        assert_eq!(OmsAxis::nint(-1.4), -1);
        assert_eq!(OmsAxis::nint(-1.5), -2);
        assert_eq!(OmsAxis::nint(0.0), 0);
    }

    /// `connectCallback` follows the serial port's connect exceptions.
    #[test]
    fn port_connected_follows_the_connect_exceptions() {
        let name = "omsConnectCallback";
        let base = epics_rs::asyn::port::PortDriverBase::new(
            name,
            1,
            epics_rs::asyn::port::PortFlags::default(),
            epics_rs::asyn::port::InterfaceMask::UNMIGRATED_ALL,
            epics_rs::asyn::port::InterfaceMask::UNMIGRATED_ALL,
        );
        let (rt, _jh) = epics_rs::asyn::runtime::create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            DeadLine {
                base,
                written: Default::default(),
            },
            epics_rs::asyn::runtime::RuntimeConfig::default(),
        )
        .unwrap();
        let port = rt.port_handle().clone();
        let io = SyncIOHandle::from_handle(port.clone(), 0, Duration::from_millis(100));
        let mut controller = OmsController::new(io, name, name, "MXA");
        controller.watch_connection().unwrap();
        assert!(controller.port_connected());
        port.disconnect_blocking().unwrap();
        assert!(!controller.port_connected());
        port.connect_blocking().unwrap();
        assert!(controller.port_connected());
    }

    #[test]
    fn notification_detection() {
        assert!(is_notification("%000 01000000"));
        assert!(is_notification("000 00000001"));
        assert!(!is_notification("MDNN,MDNN"));
        assert!(!is_notification("12345"));
        assert!(!is_notification("3F"));
    }

    /// An encoder axis whose `RE` reads 1000; records every command.
    struct EncoderAxis {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        reply: Vec<u8>,
    }

    impl epics_rs::asyn::port::PortDriver for EncoderAxis {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            self.reply = if cmd.ends_with("PS?") {
                b"=E".to_vec()
            } else if cmd.ends_with("LT?") {
                b"=h".to_vec()
            } else if cmd.ends_with("RE;") {
                b"1000".to_vec()
            } else if cmd == "AA RP;" {
                b"100,,-5,0,".to_vec()
            } else {
                Vec::new()
            };
            self.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    #[test]
    fn the_encoder_sync_before_a_move_divides_by_the_encoder_ratio() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = EncoderAxis {
            base: PortDriverBase::new(
                "omsEncoderRatio",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_ALL,
            ),
            written: written.clone(),
            reply: Vec::new(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("omsEncoderRatio").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_secs(1),
        );
        let controller = Arc::new(Mutex::new(OmsController::new(
            io,
            "omsEncoderRatio",
            "omsEncoderRatio",
            "MAXnet",
        )));
        let mut axis = OmsAxis::new(controller, 0).unwrap();
        let user = AsynUser::new(0);
        axis.set_encoder_ratio(&user, 4.0).unwrap();
        axis.move_absolute(&user, 10.0, 1.0, 100.0, 1000.0).unwrap();
        assert!(written.lock().unwrap().iter().any(|c| c == "AX;LO250;"));
    }

    #[test]
    fn the_axis_count_skips_empty_fields_as_strtok_does() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = EncoderAxis {
            base: PortDriverBase::new(
                "omsAxisCount",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_ALL,
            ),
            written: Arc::new(Mutex::new(Vec::new())),
            reply: Vec::new(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("omsAxisCount").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = OmsController::new(io, "omsAxisCount", "omsAxisCount", "MAXnet");
        assert_eq!(controller.axis_count(), 3);
    }

    /// A line that refuses every write; records what was offered.
    struct DeadLine {
        base: epics_rs::asyn::port::PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for DeadLine {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.written
                .lock()
                .unwrap()
                .push(String::from_utf8_lossy(data).into_owned());
            Err(oms_err("dead line"))
        }
    }

    #[test]
    fn init_sends_every_command_past_a_failed_send() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = DeadLine {
            base: PortDriverBase::new(
                "omsDeadInit",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_ALL,
            ),
            written: written.clone(),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("omsDeadInit").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_secs(1),
        );
        OmsController::new(io, "omsDeadInit", "omsDeadInit", "MAXnet").init("AX LP0;");
        assert_eq!(*written.lock().unwrap(), ["IC;", "AM SA;", "AX LP0;"]);
    }

    /// C `firmwareMin` (omsBaseController.cpp:920-929) takes the minimum
    /// itself; a lower field anywhere is below it.
    #[test]
    fn firmware_at_least_takes_the_minimum_itself() {
        let v = |major, minor, revision| FirmwareVersion {
            major,
            minor,
            revision,
        };
        let min = v(1, 30, 0);
        assert!(v(1, 30, 0).at_least(min));
        assert!(v(1, 30, 1).at_least(min));
        assert!(v(2, 0, 0).at_least(min));
        assert!(!v(1, 29, 9).at_least(min));
        assert!(!v(0, 99, 0).at_least(min));
    }

    #[test]
    fn axis_chars_cover_ten_axes() {
        assert_eq!(AXIS_CHARS[0], 'X');
        assert_eq!(AXIS_CHARS[3], 'T');
        assert_eq!(AXIS_CHARS[9], 'K');
    }

    #[test]
    fn the_firmware_version_is_read_as_c_sscanf_reads_it() {
        let v = |major, minor, revision| {
            Some(FirmwareVersion {
                major,
                minor,
                revision,
            })
        };
        assert_eq!(
            parse_firmware_version("MAXnet ver:1.33.4, s/n:1"),
            v(1, 33, 4)
        );
        assert_eq!(parse_firmware_version("MAXnet ver:B1.29.7,"), v(1, 29, 7));
        assert_eq!(parse_firmware_version("MXA ver:S2.0"), v(2, 0, 0));
        assert_eq!(parse_firmware_version("MXA ver:2.1.x"), v(2, 1, 0));
        assert_eq!(parse_firmware_version("MAXnet ver:1"), None);
        assert_eq!(parse_firmware_version("garbage"), None);
    }

    /// A line that times out after `partial` bytes of every reply.
    struct TimesOutMidReply {
        base: epics_rs::asyn::port::PortDriverBase,
        partial: &'static [u8],
    }

    impl epics_rs::asyn::port::PortDriver for TimesOutMidReply {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, _buf: &mut [u8]) -> AsynResult<usize> {
            Err(AsynError::Status {
                status: AsynStatus::Timeout,
                message: "timeout".into(),
            }
            .with_partial_read(epics_rs::asyn::interpose::PartialOctetRead {
                data: self.partial.to_vec(),
                eom_reason: epics_rs::asyn::interpose::EomReason::empty(),
            }))
        }
    }

    #[test]
    fn the_axis_count_reads_a_timed_out_reply_as_c_does() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = TimesOutMidReply {
            base: PortDriverBase::new(
                "omsPartialAxisCount",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_ALL,
            ),
            partial: b"0,0",
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("omsPartialAxisCount").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller =
            OmsController::new(io, "omsPartialAxisCount", "omsPartialAxisCount", "MAXnet");
        assert_eq!(controller.axis_count(), 2);
    }

    #[test]
    fn a_failed_axis_query_counts_no_axes() {
        use epics_rs::asyn::port::InterfaceMask;
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = DeadLine {
            base: PortDriverBase::new(
                "omsDeadAxisCount",
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_ALL,
            ),
            written: Arc::new(Mutex::new(Vec::new())),
        };
        let (rt, _actor) = create_port_runtime(
            &claim_port("omsDeadAxisCount").unwrap(),
            port,
            RuntimeConfig::default(),
        )
        .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = OmsController::new(io, "omsDeadAxisCount", "omsDeadAxisCount", "MAXnet");
        assert_eq!(controller.axis_count(), 0);
    }
}
