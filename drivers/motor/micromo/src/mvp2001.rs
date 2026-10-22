//! MicroMo MVP 2001 motion controller driver (serial ASCII).
//!
//! Ported from `motorMicroMo/microMoApp/src/MVP2001Driver.cpp` (the model-3
//! `asynMotorController`/`asynMotorAxis` form; the older `drvMVP2001.cc` model-1
//! pair is [`crate::drv_mvp2001`]). Commands are ASCII `<axis> <CMD> <args>` where `<axis>`
//! is 1-based; commands are written bare, as C writes them, and the port's
//! output EOS terminates them.
//!
//! ## Prepended-terminator responses
//!
//! Commands that return data prepend a terminator to the reply. The C
//! `writeRead2xController` handles this by reading twice with a short delay: the
//! first read consumes the prepended terminator, then after ~33 ms the real
//! reply is read. This port replicates that with `Mvp2001Controller::query`.
//! Replies have the form `"0001 000001F4"`; the value is the hex field starting
//! at byte offset 5 (C `parseReply` copies from `inString[5]` and `sscanf`s
//! `%x`).
//!
//! ## Units
//!
//! Positions are the controller's native encoder counts: `move` sends
//! `NINT(position)` directly (the header's `stepsPerRev_` is declared but never
//! used in the C source), and `POS` reads counts back as a hex value. Positions
//! cross the asyn-rs motor boundary (dial-frame EGU) unscaled: `MRES` is 1,
//! `EGU` is counts. Velocity and acceleration are converted to controller
//! `SP`/`AC` units using the loop sample period and encoder lines-per-rev,
//! matching C.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `POS`/`ST` reads run inside
//!   [`poll`](AsynMotor::poll).
//! - Homing is unimplemented in C (returns success without acting); `home` is
//!   likewise a no-op.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, StatusWrite,
};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::trace::MotorTrace;

use motor_common::util::nint;
use motor_common::{AxisReport, ControllerParams, ControllerReport};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 32;

/// Delay between the prepended-terminator read and the real reply (C 0.033 s).
const REPLY_DELAY: Duration = Duration::from_millis(33);

/// Default loop sample period in µs if the `SR` query fails (C fallback 500).
const DEFAULT_SAMPLE_PERIOD: i32 = 500;

fn micromo_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Parse a hex value from a reply of the form `"0001 000001F4"`: the field
/// starts at byte offset 5 and is up to `nchars` long, parsed as `%x` (C
/// `parseReply`). Values wrap through `u32` so negative positions decode
/// correctly.
fn parse_hex_reply(reply: &str, nchars: usize) -> Option<i32> {
    let tail = reply.get(5..)?;
    let end = nchars.min(tail.len());
    let field = tail[..end].trim();
    u32::from_str_radix(field, 16).ok().map(|v| v as i32)
}

/// Max-current command value (C `NINT(maxCurrent * 0.865909 + 2103.431)`).
fn ano_param(max_current_ma: i32) -> i32 {
    nint(max_current_ma as f64 * 0.865909 + 2103.431)
}

/// Velocity command value (C `NINT(samplePeriod * 6e-5 * velocity)`).
fn speed_param(sample_period: i32, velocity: f64) -> i32 {
    nint(sample_period as f64 * 6e-5 * velocity)
}

/// Acceleration command value, clamped to `[1, speed]` (C `AC` logic):
/// `NINT(7.5e-12 * samplePeriod^2 * encoderLinesPerRev * accel)`, then bounded
/// below by 1 and above by the speed parameter.
fn accel_param(sample_period: i32, enc_lines_per_rev: i32, accel: f64, speed: i32) -> i32 {
    let sp = sample_period as f64;
    let mut ac = nint(7.5e-12 * sp * sp * enc_lines_per_rev as f64 * accel);
    if ac < speed {
        if ac <= 0 {
            ac = 1;
        }
    } else {
        ac = speed;
    }
    ac
}

/// Shared controller endpoint owning the serial handle.
pub struct Mvp2001Controller {
    handle: SyncIOHandle,
    num_axes: usize,
}

impl Mvp2001Controller {
    /// Connect to an MVP 2001 controller. Axes are created separately (C
    /// `MVP2001CreateAxis`), so this performs no per-axis I/O.
    pub fn new(handle: SyncIOHandle, num_axes: usize) -> Self {
        Self { handle, num_axes }
    }

    /// Number of axes configured for this controller.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    /// Write a command with no reply expected (C `writeController`).
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Write a data-returning command and read its reply, consuming the
    /// prepended terminator first (C `writeRead2xController`).
    fn query(&self, cmd: &str) -> AsynResult<String> {
        // First read: the prepended terminator (discarded).
        let (_, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        status?;
        sleep(REPLY_DELAY);
        // Second read: the real reply, as the C string `parseReply` indexes.
        // Nothing is trimmed: a terminator the port's input EOS leaves in it
        // shifts the field for C as well.
        let (raw, status) = self.handle.read_octet(0, READ_BUF).into_parts();
        status?;
        let reply = raw.split(|&b| b == 0).next().unwrap_or_default();
        Ok(String::from_utf8_lossy(reply).into_owned())
    }
}

/// `MVP2001Controller`'s `report` override.
pub struct Mvp2001Params {
    /// C's `movingPollPeriod_`/`idlePollPeriod_`, in seconds.
    moving_poll_s: f64,
    idle_poll_s: f64,
}

impl Mvp2001Params {
    pub fn new(moving_poll_ms: u64, idle_poll_ms: u64) -> Self {
        Self {
            moving_poll_s: moving_poll_ms as f64 / 1000.0,
            idle_poll_s: idle_poll_ms as f64 / 1000.0,
        }
    }
}

impl ControllerParams for Mvp2001Params {
    /// C `MVP2001Controller::report` (MVP2001Driver.cpp:85-92).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "MVP 2001 motor driver {}, numAxes={}, moving poll period={:.6}, idle poll period={:.6}",
            ctx.port_name(),
            ctx.num_axes(),
            CFloat(self.moving_poll_s),
            CFloat(self.idle_poll_s)
        );
        ctx.report(out, level);
    }
}

/// C `MVP2001Axis::report` (MVP2001Driver.cpp:235-248): the axis's
/// constructor settings.
pub struct Mvp2001AxisReport {
    axis_index: i32,
    encoder_lines_per_rev: i32,
    max_current_ma: i32,
    sample_period: i32,
    limit_polarity: i32,
}

impl AxisReport for Mvp2001AxisReport {
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        if level > 0 {
            let _ = write!(
                out,
                "  axis {}\n  axis index {}\n  encoderLinesPerRev {}\n  maxCurrent {} (mA)\n  \
                 samplePeriod {} (us)\n  limitPolarity {}\n",
                self.axis_index - 1,
                self.axis_index,
                self.encoder_lines_per_rev,
                self.max_current_ma,
                self.sample_period,
                self.limit_polarity
            );
        }
    }
}

/// One MVP 2001 axis sharing a controller. Implements [`AsynMotor`].
pub struct Mvp2001Axis {
    controller: Arc<Mutex<Mvp2001Controller>>,
    /// C `pasynUser_`.
    trace: MotorTrace,
    /// 1-based wire axis index.
    axis_index: i32,
    encoder_lines_per_rev: i32,
    max_current_ma: i32,
    sample_period: i32,
    limit_polarity: i32,
    /// What the constructor set on the axis's status.
    status_writes: Vec<StatusWrite>,
}

impl Mvp2001Axis {
    /// Construct and initialize axis `axis_no` (0-based). Sets the limit/estop
    /// polarity (`LP`), zeroes the position (`HO`), and reads the loop sample
    /// period (`SR`), matching the C `MVP2001Axis` constructor. Performs
    /// blocking serial I/O.
    ///
    /// `max_current_ma` is clamped to `[100, 2300]` (C accepts 0.1–2.3 A);
    /// `limit_polarity` is normalized to 0 (NC) or 1 (NO).
    pub fn new(
        controller: Arc<Mutex<Mvp2001Controller>>,
        port_name: &str,
        axis_no: usize,
        encoder_lines_per_rev: i32,
        max_current_ma: i32,
        limit_polarity: i32,
    ) -> Self {
        let axis_index = axis_no as i32 + 1;
        let max_current_ma = if !(100..=2300).contains(&max_current_ma) {
            100
        } else {
            max_current_ma
        };
        let limit_polarity = if limit_polarity != 0 { 1 } else { 0 };

        let sample_period = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // Set limit/estop polarity, then home (zeroes position). C
            // ignores both statuses (MVP2001Driver.cpp:183-188).
            let _ = ctrl.write_only(&format!("{axis_index} LP {limit_polarity}"));
            let _ = ctrl.write_only(&format!("{axis_index} HO"));
            // Query loop sample period (4 hex chars); fall back to the default.
            ctrl.query(&format!("{axis_index} SR"))
                .ok()
                .and_then(|r| parse_hex_reply(&r, 4))
                .unwrap_or(DEFAULT_SAMPLE_PERIOD)
        };

        Self {
            controller,
            trace: MotorTrace::new(port_name, axis_no as i32),
            axis_index,
            encoder_lines_per_rev,
            max_current_ma,
            sample_period,
            limit_polarity,
            // MVP2001Driver.cpp:204-206.
            status_writes: vec![
                StatusWrite::GainSupport(true),
                StatusWrite::HasEncoder(true),
            ],
        }
    }

    /// The axis's `report`.
    pub fn report(&self) -> Mvp2001AxisReport {
        Mvp2001AxisReport {
            axis_index: self.axis_index,
            encoder_lines_per_rev: self.encoder_lines_per_rev,
            max_current_ma: self.max_current_ma,
            sample_period: self.sample_period,
            limit_polarity: self.limit_polarity,
        }
    }

    fn lock(&self) -> MutexGuard<'_, Mvp2001Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Send the max-current, velocity and acceleration commands (C
    /// `sendAccelAndVelocity`). Every caller overwrites the status C returns
    /// with its own command's (MVP2001Driver.cpp:250-341), so none is
    /// returned.
    fn send_accel_and_velocity(&self, ctrl: &Mvp2001Controller, accel: f64, velocity: f64) {
        let n = self.axis_index;
        let _ = ctrl.write_only(&format!("{n} ANO {}", ano_param(self.max_current_ma)));
        let sp = speed_param(self.sample_period, velocity);
        let _ = ctrl.write_only(&format!("{n} SP {sp}"));
        let ac = accel_param(self.sample_period, self.encoder_lines_per_rev, accel, sp);
        let _ = ctrl.write_only(&format!("{n} AC {ac}"));
    }
}

impl AsynMotor for Mvp2001Axis {
    /// C `MVP2001Axis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (MVP2001Driver.cpp).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        self.send_accel_and_velocity(&ctrl, acceleration, velocity);
        let _ = ctrl.write_only(&format!("{n} LA {}", nint(position)));
        ctrl.write_only(&format!("{n} M"))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        self.send_accel_and_velocity(&ctrl, acceleration, velocity);
        let _ = ctrl.write_only(&format!("{n} LR {}", nint(distance)));
        ctrl.write_only(&format!("{n} M"))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.trace.print(
            TraceMask::FLOW,
            0,
            format_args!(
                "MVP2001Axis::moveVelocity: minVelocity={:.6}, maxVelocity={:.6}, acceleration={:.6}\n",
                CFloat(min_velocity),
                CFloat(velocity),
                CFloat(acceleration)
            ),
        );
        let n = self.axis_index;
        let ctrl = self.lock();
        self.send_accel_and_velocity(&ctrl, acceleration, velocity);
        let sp = speed_param(self.sample_period, velocity);
        ctrl.write_only(&format!("{n} V {sp}"))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // Homing is unimplemented in the C driver (returns success).
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write_only(&format!("{n} AB"))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        ctrl.write_only(&format!("{n} HO {}", nint(position)))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let n = self.axis_index;
        let ctrl = self.lock();
        if enable {
            // An AB is needed before EN (EN fails if status ends in 8, not E);
            // EN's status overwrites AB's (MVP2001Driver.cpp:408-429).
            let _ = ctrl.write_only(&format!("{n} AB"));
            drop(ctrl);
            sleep(REPLY_DELAY);
            let ctrl = self.lock();
            ctrl.write_only(&format!("{n} EN"))
        } else {
            ctrl.write_only(&format!("{n} DI"))
        }
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        let n = self.axis_index;
        // C gain-to-register conversions.
        let cmd = match kind {
            PidGainKind::Proportional => format!("{n} POR {}", nint(gain * 28000.0 + 4000.0)),
            PidGainKind::Integral => format!("{n} I {}", nint(gain * 31999.0 + 1.0)),
            PidGainKind::Derivative => format!("{n} DER {}", nint(gain * 31000.0 + 1000.0)),
        };
        let ctrl = self.lock();
        ctrl.write_only(&cmd)
    }

    fn take_status_writes(&mut self) -> Vec<StatusWrite> {
        std::mem::take(&mut self.status_writes)
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let n = self.axis_index;
        let controller = Arc::clone(&self.controller);
        let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        // Position (8 hex chars, signed via u32 wrap). An unparseable POS or
        // ST reply raises PROBLEM on purpose: C's `parseReply` leaves the
        // uninitialised `position` / `status` int as it was and uses it.
        let pos_reply = ctrl.query(&format!("{n} POS"))?;
        let position = parse_hex_reply(&pos_reply, 8)
            .ok_or_else(|| micromo_err(format!("MVP2001: unparseable POS reply {pos_reply:?}")))?;

        status.position = position as f64;
        // C sets no motorEncoderPosition_ though its constructor sets
        // HasEncoder (MVP2001Driver.cpp:206, :453), so UEIP reads 0. C
        // defect, not copied: POS is the encoder count.
        status.encoder_position = position as f64;

        // Status word (4 hex chars).
        let status_reply = ctrl.query(&format!("{n} ST"))?;
        drop(ctrl);
        let st = parse_hex_reply(&status_reply, 4).ok_or_else(|| {
            micromo_err(format!("MVP2001: unparseable ST reply {status_reply:?}"))
        })?;

        // Bit 0 set = moving; 0x2000 = high limit; 0x8000 = low limit;
        // 0x100 set = power off. No home bit on this controller.
        let done = (st & 0x1) == 0;
        status.done = done;
        status.moving = !done;
        status.high_limit = (st & 0x2000) != 0;
        status.low_limit = (st & 0x8000) != 0;
        status.powered = (st & 0x100) == 0;
        status.problem = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use motor_common::testing::Polled;

    #[test]
    fn an_axis_of_an_unanswering_controller_is_still_built() {
        let (handle, _) =
            SyncIOHandle::connect_port("MVP2001_NONE", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(Mvp2001Controller::new(handle, 1)));
        let axis = Mvp2001Axis::new(controller, "MVP2001_NONE", 0, 512, 500, 1);
        assert_eq!(axis.sample_period, DEFAULT_SAMPLE_PERIOD);
    }

    /// MVP2001Driver.cpp:328: moveVelocity's FLOW line, on the axis's user
    /// and before any command.
    #[test]
    fn move_velocity_traces_as_c() {
        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};

        let (handle, _) =
            SyncIOHandle::connect_port("MVP2001_NONE", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(Mvp2001Controller::new(handle, 2)));
        let mut axis = Mvp2001Axis::new(controller, "MVP2001_FLOW", 1, 512, 500, 1);
        let path = std::env::temp_dir().join(format!("mvp2001-flow-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_device_trace_mask("MVP2001_FLOW", 1, TraceMask::FLOW);
        trace.set_device_trace_info_mask("MVP2001_FLOW", 1, TraceInfoMask::empty());
        trace.set_device_trace_file("MVP2001_FLOW", 1, TraceFile::File(file));
        let _ = axis.move_velocity(&AsynUser::new(1), 0.5, 100.0, 250.0);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            text,
            "MVP2001Axis::moveVelocity: minVelocity=0.500000, maxVelocity=100.000000, acceleration=250.000000\n"
        );
    }

    /// C sets GAIN_SUPPORT and EA_PRESENT once, in the constructor
    /// (MVP2001Driver.cpp:204-206), so a poll that fails before any reply
    /// still posts them.
    #[test]
    fn a_failed_poll_keeps_the_constructor_status_bits() {
        let (handle, _) =
            SyncIOHandle::connect_port("MVP2001_DEAD", 0, Duration::ZERO, InterfaceType::Octet);
        let controller = Arc::new(Mutex::new(Mvp2001Controller::new(handle, 1)));
        let mut axis = Mvp2001Axis::new(controller, "MVP2001_DEAD", 0, 512, 500, 1);
        let mut left = axis.constructed();
        assert!(axis.poll(&AsynUser::default(), &mut left).is_err());
        assert!(left.gain_support && left.has_encoder, "{left:?}");
        // C's status_ starts at 0; only a good poll sets Done (:467).
        assert!(!left.done, "{left:?}");
    }

    /// MVP2001Driver.cpp:437-485 sets only what it read before a failed
    /// read's `skip`, over the params as they stood, the Done=0 a move start
    /// set included (asynMotorController.cpp:301).
    #[test]
    fn a_failed_read_leaves_the_done_a_move_start_set() {
        use motor_common::testing::LineRig;
        let refused = Arc::new(Mutex::new(String::new()));
        let pick = refused.clone();
        let (_rig, io) = LineRig::failing(
            "mvp2001MoveStartDone",
            |cmd| {
                match cmd.trim_end() {
                    "1 POS" => "0001 000001F4",
                    "1 ST" => "0001 0000",
                    _ => "",
                }
                .into()
            },
            move |cmd| *pick.lock().unwrap() == cmd.trim_end(),
        );
        let controller = Arc::new(Mutex::new(Mvp2001Controller::new(io, 1)));
        let mut axis = Mvp2001Axis::new(controller, "mvp2001MoveStartDone", 0, 512, 500, 1);
        let user = AsynUser::default();
        // The PROBLEM the poller raised over an earlier failure; `skip` clears it.
        let mut status = MotorStatus {
            problem: true,
            ..MotorStatus::default()
        };
        axis.poll(&user, &mut status).unwrap();
        assert!(status.done && !status.problem, "{status:?}");
        status.done = false;
        *refused.lock().unwrap() = "1 ST".into();
        assert!(axis.poll(&user, &mut status).is_err());
        assert!(status.position == 500.0 && !status.done, "{status:?}");
    }

    /// C sets `status =` on each write and returns the last
    /// (MVP2001Driver.cpp:250-341, :408-429): a refused setting still sends
    /// the command after it.
    #[test]
    fn a_refused_setting_still_sends_the_command_after_it() {
        use motor_common::testing::LineRig;
        let (rig, io) = LineRig::failing(
            "mvp2001Refused",
            |_| String::new(),
            |cmd| {
                let cmd = cmd.trim_end();
                cmd.contains(" ANO ") || cmd.contains(" LA ") || cmd == "1 AB" || cmd == "1 DI"
            },
        );
        let controller = Arc::new(Mutex::new(Mvp2001Controller::new(io, 1)));
        let mut axis = Mvp2001Axis::new(controller, "mvp2001Refused", 0, 512, 500, 1);
        let user = AsynUser::default();
        let sent = || -> Vec<String> {
            rig.sent()
                .iter()
                .map(|c| c.trim_end().to_string())
                .filter(|c| !c.contains(" ANO ") && !c.contains(" SP ") && !c.contains(" AC "))
                .collect()
        };
        sent();
        axis.move_absolute(&user, 10.0, 0.0, 100.0, 250.0).unwrap();
        axis.move_relative(&user, 5.0, 0.0, 100.0, 250.0).unwrap();
        axis.move_velocity(&user, 0.0, 100.0, 250.0).unwrap();
        axis.set_closed_loop(&user, true).unwrap();
        let v = format!("1 V {}", speed_param(DEFAULT_SAMPLE_PERIOD, 100.0));
        assert_eq!(
            sent(),
            ["1 LA 10", "1 M", "1 LR 5", "1 M", &v, "1 AB", "1 EN"]
        );
        assert!(axis.set_closed_loop(&user, false).is_err());
    }

    /// C `writeController` and `writeRead2xController` write `outString_`
    /// bare (MVP2001Driver.cpp:129, :183-198); the port's output EOS
    /// terminates it.
    #[test]
    fn the_commands_go_out_bare() {
        use motor_common::testing::LineRig;
        let (rig, io) = LineRig::new("mvp2001Bare", |cmd| match cmd {
            "1 SR" => "0001 0190".into(),
            _ => String::new(),
        });
        let controller = Arc::new(Mutex::new(Mvp2001Controller::new(io, 1)));
        let mut axis = Mvp2001Axis::new(controller, "mvp2001Bare", 0, 512, 1000, 1);
        let user = AsynUser::default();
        axis.move_absolute(&user, 10.0, 0.0, 100.0, 250.0).unwrap();
        axis.stop(&user, 1.0).unwrap();
        assert_eq!(
            rig.sent(),
            [
                "1 LP 1",
                "1 HO",
                "1 SR",
                "1 ANO 2969",
                "1 SP 2",
                "1 AC 1",
                "1 LA 10",
                "1 M",
                "1 AB"
            ]
        );
    }

    #[test]
    fn parse_hex_reply_offset_and_width() {
        assert_eq!(parse_hex_reply("0001 000001F4", 8), Some(500));
        assert_eq!(parse_hex_reply("0001 0008", 4), Some(8));
        // Negative position via u32 wrap.
        assert_eq!(parse_hex_reply("0001 FFFFFFF4", 8), Some(-12));
        // Too short after the offset.
        assert_eq!(parse_hex_reply("0001", 4), None);
    }

    #[test]
    fn accel_param_clamps() {
        // Below speed but positive -> unchanged.
        let ac = accel_param(500, 1000, 1.0, 100);
        assert!((1..=100).contains(&ac));
        // Zero/negative accel -> clamped to 1.
        assert_eq!(accel_param(500, 1000, 0.0, 100), 1);
        // Huge accel -> clamped to the speed.
        assert_eq!(accel_param(500, 1000, 1e9, 100), 100);
    }

    #[test]
    fn speed_and_current_params() {
        assert_eq!(speed_param(500, 100.0), nint(500.0 * 6e-5 * 100.0));
        assert_eq!(ano_param(1000), nint(1000.0 * 0.865909 + 2103.431));
    }
}
