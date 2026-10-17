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
//! a move completes. The C driver registers an asyn interrupt handler to count
//! them and wake the poller; this port skips any notification line that
//! arrives in the reply stream, and the poller finds the move done at its
//! next loop.
//!
//! ## Polling
//!
//! As in C, one poller thread per controller ([`crate::poller`]) reads the
//! status of every axis at once and owns the axes' status parameters;
//! [`OmsAxis::poll`] returns them without touching the line. The asyn-rs
//! motor poll loops still run per axis, so a change reaches the record at the
//! axis loop's next pass rather than at the poller's `callParamCallbacks`,
//! and a forced status refresh posts the parameters without waking the
//! poller as C `omsBaseAxis::poll` does: the axis poll also serves the timed
//! passes, where a wakeup would run the controller loop once per axis.
//!
//! ## Units
//!
//! Positions, encoder counts, velocities and accelerations are controller-
//! native steps / steps-per-second; with the motor record's `MRES` = 1 a step
//! is one EGU, so the record's EGU values pass straight through (clamped to the
//! OMS ranges: velocity 1..4000000, home velocity 1..1000000, acceleration
//! 1..8000000, position magnitude <= 67000000). There is no resolution factor
//! to cancel: the boundary is controller-native steps with `MRES` = 1.
//!
//! ## Not modeled (documented)
//!
//! - The notification interrupt (`asynCallback`): see above.
//! - The auxiliary parameter records exposed through the C `writeInt32` /
//!   `writeFloat64` overrides. These are not motor-record motion fields.
//! - The init-time firmware-version gate: the driver reads the firmware
//!   string once as a comms check and assumes the modern (>= 1.30) command
//!   forms MAXnet mandates, `CL?` included; only the watchdog check reads
//!   the version.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::exception::{AsynException, ExceptionCallbackGuard};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;

use motor_common::util::atoi;

use crate::poller::PollShared;

/// C `omsTotalControllerNumber`.
static CONTROLLERS: AtomicUsize = AtomicUsize::new(0);

/// Reply read buffer size (C `OMSINPUTBUFFERLEN` = 12*10 + 2).
const READ_BUF: usize = 256;

/// Per-axis command selector characters (C `axisChrArr`).
const AXIS_CHARS: [char; 10] = ['X', 'Y', 'Z', 'T', 'U', 'V', 'R', 'S', 'W', 'K'];

/// Move parameter clamps (C `omsBaseAxis`).
const MAX_VELOCITY: i32 = 4_000_000;
const MAX_HOME_VELOCITY: i32 = 1_000_000;
const MAX_ACCEL: i32 = 8_000_000;
const MAX_POSITION: i32 = 67_000_000;

/// `sscanf(p, "ver:%d.%d.%d,")` after C's `ver:B`, `ver:S`, `ver:` search,
/// needing the first two numbers.
fn parse_firmware_version(reply: &str) -> Option<(i32, i32)> {
    let at = ["ver:B", "ver:S", "ver:"]
        .iter()
        .find_map(|tag| reply.find(tag).map(|i| i + tag.len()))?;
    let (major, rest) = scan_int(&reply[at..])?;
    let (minor, _) = scan_int(rest.strip_prefix('.')?)?;
    Some((major, minor))
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

fn oms_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// A line the controller sends unsolicited on move completion
/// (`"%000 SSSSSSSS"`); C `isNotification` keys on the `"000 0"` substring.
fn is_notification(s: &str) -> bool {
    s.contains("000 0")
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
        }
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
        let trim = |raw: &[u8]| {
            String::from_utf8_lossy(raw)
                .trim_start_matches(['\x06', '\r', '\n'])
                .trim_end_matches(['\r', '\n', '\0', ' '])
                .to_string()
        };
        let failed = |e: AsynError| {
            let reply = e.partial_read().map_or_else(String::new, |p| trim(&p.data));
            (reply, Err(e))
        };
        let mut raw = match self.handle.write_read(0, cmd.as_bytes(), READ_BUF) {
            Ok(raw) => raw,
            Err(e) => return failed(e),
        };
        // A move notification may sit ahead of the real reply; skip it. Bound
        // the retries so a chatty controller cannot spin forever.
        for _ in 0..12 {
            let trimmed = trim(&raw);
            if is_notification(&trimmed) {
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

    /// C `omsBaseController::getFirmwareVersion`: up to three `WY` tries a
    /// second apart, each reply logged, then `(major, minor)` from its `ver:`
    /// field. An unparsable reply fails with an empty message, as C's caller
    /// prints the user's untouched `errorMessage`.
    pub fn firmware_version(&self) -> AsynResult<(i32, i32)> {
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
    /// Construct axis `index` and probe its type (`PS?`) and limit true-state
    /// (`LT?`), and add its status parameters. Axes are created in order.
    /// Assumes the modern (firmware >= 1.30) command forms.
    pub fn new(controller: Arc<Mutex<OmsController>>, index: usize) -> AsynResult<Self> {
        let axis_char = *AXIS_CHARS
            .get(index)
            .ok_or_else(|| oms_err(format!("oms: axis index {index} out of range (max 9)")))?;

        let (has_encoder, poll_shared) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // Axis type: "=O" stepper/no-encoder, "=M" servo, "=E" stepper+enc.
            let ps = ctrl.send_receive(&format!("A{axis_char} PS?"))?;
            let has_encoder = match ps.as_bytes().get(1) {
                Some(b'O') => false,
                Some(b'M') | Some(b'E') => true,
                _ => return Err(oms_err(format!("oms: unknown axis type '{ps}'"))),
            };
            // Limit true-state: "=l"/"=L" invert, "=h"/"=H" no invert.
            let lt = ctrl.send_receive(&format!("A{axis_char} LT?"))?;
            let invert_limit = match lt.as_bytes().get(1) {
                Some(b'l') | Some(b'L') => true,
                Some(b'h') | Some(b'H') => false,
                _ => return Err(oms_err(format!("oms: unknown limit true-state '{lt}'"))),
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

    /// The status parameters as the controller's poller left them.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        Ok(self.poll_shared.status(self.axis_index))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = EncoderAxis {
            base: PortDriverBase::new("omsEncoderRatio", 1, PortFlags::default()),
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
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = EncoderAxis {
            base: PortDriverBase::new("omsAxisCount", 1, PortFlags::default()),
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
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let written = Arc::new(Mutex::new(Vec::new()));
        let port = DeadLine {
            base: PortDriverBase::new("omsDeadInit", 1, PortFlags::default()),
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

    #[test]
    fn axis_chars_cover_ten_axes() {
        assert_eq!(AXIS_CHARS[0], 'X');
        assert_eq!(AXIS_CHARS[3], 'T');
        assert_eq!(AXIS_CHARS[9], 'K');
    }

    #[test]
    fn the_firmware_version_is_read_as_c_sscanf_reads_it() {
        assert_eq!(
            parse_firmware_version("MAXnet ver:1.33.4, s/n:1"),
            Some((1, 33))
        );
        assert_eq!(parse_firmware_version("MAXnet ver:B1.29.0,"), Some((1, 29)));
        assert_eq!(parse_firmware_version("MXA ver:S2.0"), Some((2, 0)));
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
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = TimesOutMidReply {
            base: PortDriverBase::new("omsPartialAxisCount", 1, PortFlags::default()),
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
        use epics_rs::asyn::port::{PortDriverBase, PortFlags};
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let port = DeadLine {
            base: PortDriverBase::new("omsDeadAxisCount", 1, PortFlags::default()),
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
