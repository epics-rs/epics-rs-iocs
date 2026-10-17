//! attocube ANC150 piezo stepper controller driver (serial ASCII).
//!
//! Ported from `motorAttocube/attocubeApp/src/drvANC150Asyn.cc`. The ANC150 is
//! an **open-loop** stepper: it reports no position, so the driver tracks
//! position in software (steps) and estimates motion completion from a timer —
//! `moveinterval = |steps| / frequency` seconds, where `frequency` is the
//! axis's step rate read back with `getf`. Commands are `stepu`/`stepd`
//! (up/down by a step count); every command line is `\r\n`-terminated and the
//! controller answers with an echo, a reply line, and an acknowledgement,
//! framed by a `"> "` prompt (input EOS).
//!
//! ## Units
//!
//! Positions are wire steps. The asyn-rs motor boundary is dial-frame EGU, so
//! for this step-native controller EGU ≡ steps: positions pass through with
//! `NINT` rounding (C `NINT`) and the record's `MRES` is 1. The record's
//! velocity/acceleration have no effect — the ANC150 has no velocity command;
//! its step rate (`frequency`) is a controller setting, read but never written
//! from the record.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller runs as a background thread; here the interpolation and the
//!   `getf`/`getm` status reads run inside [`poll`](AsynMotor::poll), whose
//!   cadence supplies the elapsed time from a monotonic clock.
//! - C never stores the high/low limits (`motorAxisSetDouble` breaks on
//!   `motorAxisHighLimit`/`LowLimit`), so its jog targets 0; this port keeps
//!   the same non-storing behaviour (jog targets the unset limit) — see
//!   [`Anc150Axis::high_limit`].
//! - When `frequency` is unknown (0), C divides by it (move never completes);
//!   this port falls back to a one-quantum move interval so the move finishes.
//! - The ANC150 reports no home/limit switches (C `axisStatus` is never set),
//!   so those status bits are always clear.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus, OctetStep};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use motor_common::model2::AxisLog;
use motor_common::util::nint;

/// Response buffer size (C `BUFFER_SIZE` 100, padded for the multi-line reply).
const READ_BUF: usize = 256;

/// Port EOS set at connect (C `ANC150_IN_EOS`/`ANC150_OUT_EOS`,
/// drvANC150Asyn.cc:742-743).
pub const INPUT_EOS: &[u8] = b"> ";
pub const OUTPUT_EOS: &[u8] = b"\r\n";

/// Fallback move interval when the step frequency is unknown
/// (C `epicsThreadSleepQuantum()`, ~1 clock tick).
const MOVE_QUANTUM: f64 = 0.02;

fn anc_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and command framing.
/// The caller holds the `Arc<Mutex<_>>` lock across a write→read exchange.
pub struct Anc150Controller {
    handle: SyncIOHandle,
    /// The octet port `pController->pasynUser` prints on.
    serial_port: String,
    ident: String,
}

impl Anc150Controller {
    /// Connect and identify an ANC150 (C `ANC150AsynConfig`): probe `ver` up to
    /// three times and require an `attocube …` reply. Performs blocking serial
    /// I/O.
    pub fn new(handle: SyncIOHandle, serial_port: &str) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            serial_port: serial_port.to_owned(),
            ident: String::new(),
        };
        for _ in 0..3 {
            if let Ok(reply) = ctrl.query("ver")
                && reply.starts_with("attocube")
            {
                ctrl.ident = reply;
                return Ok(ctrl);
            }
        }
        Err(anc_err("ANC150: no 'attocube' identification from 'ver'"))
    }

    /// The `ver` identification string.
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Send a command and discard the reply (C `sendOnly`, which still reads
    /// to keep the stream synchronized). `writeRead` flushes a stray reply
    /// first.
    fn send_only(&self, cmd: &str) -> AsynResult<()> {
        let (sent, e) = match self.handle.write_read(0, cmd.as_bytes(), READ_BUF) {
            Ok(_) => return Ok(()),
            Err(e) => (
                match e.octet_step() {
                    Some(OctetStep::Read) => cmd.len(),
                    _ => e.partial_write().unwrap_or(0),
                },
                e,
            ),
        };
        let e = if sent == cmd.len() {
            e
        } else {
            anc_err(e.message())
        };
        // C formats the command pointer with %d; the text is printed instead.
        self.print_error(format_args!(
            "drvANC150Asyn:sendOnly: error sending command {cmd}, sent={sent}, status={}\n",
            e.status() as i32
        ));
        Err(e)
    }

    /// Send a query and return its reply line (C `sendAndReceive`): the
    /// response must open with the echoed command and hold the echo, the
    /// reply and the ack, each `\r\n`-terminated, or it is asynError.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let e = match self.handle.write_read(0, cmd.as_bytes(), READ_BUF) {
            Ok(raw) => {
                if let Some(reply) = parse_reply(cmd, &String::from_utf8_lossy(&raw)) {
                    return Ok(reply.to_string());
                }
                // C prints whatever errorMessage the user last held.
                anc_err("")
            }
            // A failed writeRead keeps its own status in C.
            Err(e) => e,
        };
        self.print_error(format_args!(
            "drvANC150Asyn:sendAndReceive error calling writeRead, output={cmd} status={}, error={}\n",
            e.status() as i32,
            e.message()
        ));
        Err(e)
    }

    fn print_error(&self, args: std::fmt::Arguments<'_>) {
        PortManager::global().trace_manager().print(
            Some(&self.serial_port),
            Some(0),
            0,
            TraceMask::ERROR,
            args,
        );
    }
}

/// C `sendAndReceive`'s framing: the response opens with the echoed
/// command and holds the echo, the reply and the ack, each `\r\n`-terminated.
fn parse_reply<'a>(cmd: &str, blob: &'a str) -> Option<&'a str> {
    if !blob.starts_with(cmd) {
        return None;
    }
    let (_echo, rest) = blob.split_once("\r\n")?;
    let (reply, ack) = rest.split_once("\r\n")?;
    ack.contains("\r\n").then_some(reply)
}

/// Compute the move plan (C `motorAxisMove`): the step count magnitude, its
/// direction (`true` = up/positive), and the axis's new target position.
fn plan_move(current: f64, arg: f64, relative: bool) -> (i64, bool, f64) {
    if relative {
        let imove = i64::from(nint(arg));
        (imove.abs(), arg >= 0.0, current + arg)
    } else {
        let imove = i64::from(nint(arg - current));
        (imove.abs(), imove >= 0, arg)
    }
}

/// Interpolate the reported position during a timed move (C poller): once the
/// move timer has elapsed the axis is at the target and done; otherwise the
/// position is linearly interpolated by the fraction of `move_interval`
/// elapsed. Returns `(position, done)`.
fn slew(current: f64, target: f64, time_remaining: f64, move_interval: f64) -> (f64, bool) {
    if time_remaining < 0.0 {
        (target, true)
    } else {
        let proportion = 1.0 - (time_remaining / move_interval);
        (current + (target - current) * proportion, false)
    }
}

/// One ANC150 axis sharing a controller. Implements [`AsynMotor`].
pub struct Anc150Axis {
    controller: Arc<Mutex<Anc150Controller>>,
    /// C `pAxis->card`.
    card: i32,
    /// C `pAxis->logParam`.
    log: Arc<AxisLog>,
    /// 0-based axis index; the wire axis number is `axis + 1`.
    axis: usize,
    current_position: f64,
    target_position: f64,
    /// Jog target limits — never updated (C ignores limit sets); default 0.
    high_limit: f64,
    low_limit: f64,
    /// Step rate (Hz) from `getf`, used to time moves.
    frequency: i64,
    moving: bool,
    /// Wall-clock deadline of the in-flight move, and its planned duration.
    move_deadline: Option<Instant>,
    move_interval: f64,
    last_status: MotorStatus,
}

impl Anc150Axis {
    /// Construct axis `axis` (0-based): read its step frequency (`getf`) and put
    /// it in stepping mode (`setm N stp`). Performs blocking serial I/O.
    pub fn new(
        controller: Arc<Mutex<Anc150Controller>>,
        card: i32,
        axis: usize,
    ) -> AsynResult<Self> {
        let mut frequency = 0;
        {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            if let Ok(reply) = ctrl.query(&format!("getf {}", axis + 1)) {
                frequency = parse_frequency(&reply).unwrap_or(0);
            }
            ctrl.send_only(&format!("setm {} stp", axis + 1))?;
        }
        Ok(Self {
            controller,
            card,
            log: Arc::default(),
            axis,
            current_position: 0.0,
            target_position: 0.0,
            high_limit: 0.0,
            low_limit: 0.0,
            frequency,
            moving: false,
            move_deadline: None,
            move_interval: 0.0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                ..MotorStatus::default()
            },
        })
    }

    fn lock(&self) -> MutexGuard<'_, Anc150Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The log `Model2::set_axis_logs` attaches.
    pub fn log(&self) -> Arc<AxisLog> {
        self.log.clone()
    }

    /// The jog limit fields (kept for parity with C's non-storing behaviour).
    pub fn high_limit(&self) -> f64 {
        self.high_limit
    }

    /// drvANC150Asyn.cc:408-410: `motorAxisMove`'s FLOW line, which a jog
    /// prints too (:489-494).
    fn print_move(&self, position: f64, min_velocity: f64, velocity: f64, acceleration: f64) {
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "Set card {}, axis {} move to {}, min vel={}, max_vel={}, accel={}\n",
                self.card,
                self.axis,
                CFloat(position),
                CFloat(min_velocity),
                CFloat(velocity),
                CFloat(acceleration)
            ),
        );
    }

    /// Issue a move to `arg` (absolute) or by `arg` (relative), tracking the
    /// software position and starting the completion timer (C `motorAxisMove`).
    fn do_move(&mut self, arg: f64, relative: bool) -> AsynResult<()> {
        let (imove, posdir, new_target) = plan_move(self.current_position, arg, relative);
        self.target_position = new_target;

        let move_command = if posdir { "stepu" } else { "stepd" };
        self.move_interval = if self.frequency > 0 {
            imove as f64 / self.frequency as f64
        } else {
            MOVE_QUANTUM
        };
        if self.move_interval <= 0.0 {
            self.move_interval = MOVE_QUANTUM;
        }
        self.moving = true;
        self.move_deadline = Some(Instant::now() + Duration::from_secs_f64(self.move_interval));

        let ctrl = self.lock();
        ctrl.send_only(&format!("{} {} {}", move_command, self.axis + 1, imove))?;
        Ok(())
    }
}

/// Parse a `getf` reply `frequency = %d` into the step rate.
fn parse_frequency(reply: &str) -> Option<i64> {
    reply.strip_prefix("frequency = ")?.trim().parse().ok()
}

impl AsynMotor for Anc150Axis {
    /// C `ANC150Poller` sets only `motorAxisCommError` when `getFreq` fails.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::COMMS_ERROR
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
        self.do_move(position, false)
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
        self.do_move(distance, true)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // C: the ANC150 has no jog; move to a soft limit instead. The limit
        // fields are never set (module Deviations), so this targets 0.
        let (target, speed) = if velocity > 0.0 {
            (self.high_limit, velocity)
        } else {
            (self.low_limit, -velocity)
        };
        self.print_move(target, min_velocity, speed, acceleration);
        self.do_move(target, false)
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C `motorAxisHome` returns an error — the ANC150 has no home.
        Err(anc_err("ANC150: home not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, acceleration: f64) -> AsynResult<()> {
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "Set card {}, axis {} to stop with accel={}\n",
                self.card,
                self.axis,
                CFloat(acceleration)
            ),
        );
        let ctrl = self.lock();
        ctrl.send_only(&format!("stop {}", self.axis + 1))?;
        drop(ctrl);
        // Reset the timer so the next poll reports done.
        self.moving = false;
        self.move_deadline = None;
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C `motorAxisSetDouble(motorAxisPosition)`: redefine the software
        // position (no hardware command).
        self.current_position = position;
        self.target_position = position;
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        let mode = if enable { "stp" } else { "gnd" };
        ctrl.send_only(&format!("setm {} {}", self.axis + 1, mode))?;
        Ok(())
    }

    /// drvANC150Asyn.cc:316-320: refused, as the ratio devMotorAsyn writes
    /// at init.
    fn set_encoder_ratio(&mut self, _user: &AsynUser, _ratio: f64) -> AsynResult<()> {
        self.log.print(
            TraceMask::ERROR,
            format_args!("motorAxisSetDouble: ANC150 does not support setting encoder ratio\n"),
        );
        Err(anc_err("ANC150 does not support setting encoder ratio"))
    }

    /// drvANC150Asyn.cc:331-348: every gain is refused.
    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        let what = match kind {
            PidGainKind::Proportional => "proportional",
            PidGainKind::Integral => "integral",
            PidGainKind::Derivative => "derivative",
        };
        self.log.print(
            TraceMask::ERROR,
            format_args!("ANC150 does not support setting {what} gain\n"),
        );
        Err(anc_err(format!(
            "ANC150 does not support setting {what} gain"
        )))
    }

    /// C `motorAxisforceCallback`; the poll that follows is the poller's.
    fn force_callback(&mut self, _user: &AsynUser) {
        self.log.print(
            TraceMask::FLOW,
            format_args!(
                "motorAxisforceCallback: request card {}, axis {} status update\n",
                self.card, self.axis
            ),
        );
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        // Interpolate the software position (C poller).
        let (position, done) = if self.moving {
            let time_remaining = self
                .move_deadline
                .map(|d| {
                    let now = Instant::now();
                    if now >= d {
                        -1.0
                    } else {
                        (d - now).as_secs_f64()
                    }
                })
                .unwrap_or(-1.0);
            let (pos, finished) = slew(
                self.current_position,
                self.target_position,
                time_remaining,
                self.move_interval,
            );
            if finished {
                self.moving = false;
                self.move_deadline = None;
                self.current_position = self.target_position;
            }
            (pos, finished)
        } else {
            self.current_position = self.target_position;
            (self.current_position, true)
        };

        // C `axisStatus` is never set (module Deviations).
        self.log.print(
            TraceMask::IO_DRIVER,
            format_args!(
                "ANC150Poller: axis {} axisStatus=0, position={}\n",
                self.axis,
                CFloat(position)
            ),
        );

        // Update the step frequency and power state. Read into locals first so
        // the controller lock is released before mutating `self.frequency`.
        let axis1 = self.axis + 1;
        let ctrl = self.lock();
        let freq_reply = ctrl.query(&format!("getf {axis1}"));
        let mode_reply = ctrl.query(&format!("getm {axis1}"));
        drop(ctrl);

        let freq_ok = match freq_reply {
            Ok(reply) => {
                if let Some(f) = parse_frequency(&reply) {
                    self.frequency = f;
                    true
                } else {
                    reply.starts_with("Axis not in computer control mode")
                }
            }
            Err(_) => false,
        };
        let powered = matches!(mode_reply, Ok(reply) if reply.starts_with("mode = stp"));

        self.last_status = MotorStatus {
            position,
            encoder_position: position,
            velocity: 0.0,
            done,
            moving: !done,
            // ANC150 reports no home/limit switches (module Deviations).
            high_limit: false,
            low_limit: false,
            home: false,
            powered,
            comms_error: !freq_ok,
            problem: !freq_ok,
            gain_support: true,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plan_move_absolute_direction_and_target() {
        // Absolute move forward from 10 to 25: 15 steps up.
        let (imove, up, target) = plan_move(10.0, 25.0, false);
        assert_eq!(imove, 15);
        assert!(up);
        assert_eq!(target, 25.0);
        // Absolute move backward from 10 to 4: 6 steps down.
        let (imove, up, target) = plan_move(10.0, 4.0, false);
        assert_eq!(imove, 6);
        assert!(!up);
        assert_eq!(target, 4.0);
    }

    #[test]
    fn plan_move_relative_accumulates_and_rounds() {
        // Relative +3.4 rounds (NINT) to 3 steps up; target accumulates.
        let (imove, up, target) = plan_move(10.0, 3.4, true);
        assert_eq!(imove, 3);
        assert!(up);
        assert_eq!(target, 13.4);
        // Relative -2.6 rounds to 3 steps down.
        let (imove, up, target) = plan_move(10.0, -2.6, true);
        assert_eq!(imove, 3);
        assert!(!up);
        assert!((target - 7.4).abs() < 1e-9);
    }

    #[test]
    fn slew_interpolates_then_completes() {
        // Halfway through a 2 s move from 0 to 10: position 5, not done.
        let (pos, done) = slew(0.0, 10.0, 1.0, 2.0);
        assert!((pos - 5.0).abs() < 1e-9);
        assert!(!done);
        // Timer elapsed: at target, done.
        let (pos, done) = slew(0.0, 10.0, -0.1, 2.0);
        assert_eq!(pos, 10.0);
        assert!(done);
    }

    /// drvANC150Asyn.cc:835-856.
    #[test]
    fn a_reply_needs_the_echo_and_three_lines() {
        assert_eq!(
            parse_reply("getf 1", "getf 1\r\nfrequency = 1000\r\nOK\r\n"),
            Some("frequency = 1000")
        );
        assert_eq!(
            parse_reply("getf 1", "getf 2\r\nfrequency = 1000\r\nOK\r\n"),
            None
        );
        assert_eq!(
            parse_reply("getf 1", "getf 1\r\nfrequency = 1000\r\nOK"),
            None
        );
        assert_eq!(parse_reply("getf 1", "getf 1\r\n"), None);
    }

    #[test]
    fn parse_frequency_reads_getf_reply() {
        assert_eq!(parse_frequency("frequency = 400"), Some(400));
        assert_eq!(parse_frequency("frequency = 1000 "), Some(1000));
        assert_eq!(parse_frequency("mode = stp"), None);
    }

    /// drvANC150Asyn.cc:532-546: a GET_INFO's motorAxisforceCallback prints
    /// its FLOW line, 0-based axis, on the axis's log.
    #[epics_rs::base::epics_test]
    async fn force_callback_prints_the_c_flow_line() {
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        static DRIVER: Model2 = Model2::new("ancForceTest", 0);
        let (_line, io) = LineRig::new("ancForceLine", |last| match last {
            "ver" => "ver\r\nattocube ANC150\r\nOK\r\n".into(),
            c if c.starts_with("getf") => format!("{c}\r\nfrequency = 1000\r\nOK\r\n"),
            _ => String::new(),
        });
        let ctrl = Arc::new(Mutex::new(
            Anc150Controller::new(io, "ancForceLine").unwrap(),
        ));
        let axes: Vec<_> = (0..2)
            .map(|axis| Anc150Axis::new(ctrl.clone(), 1, axis).unwrap())
            .collect();
        let logs = axes.iter().map(Anc150Axis::log).collect();
        let axes: Vec<_> = axes.into_iter().map(|a| Arc::new(Mutex::new(a))).collect();
        let motors = axes
            .iter()
            .map(|a| a.clone() as Arc<Mutex<dyn AsynMotor>>)
            .collect();
        DRIVER.set_card(1, motors, 100, 1000);
        DRIVER.set_axis_logs(1, logs);
        let args = vec![
            ArgValue::String("ancForce".into()),
            ArgValue::String("ancForceTest".into()),
            ArgValue::Int(1),
            ArgValue::Int(2),
        ];
        run_command(drv_asyn_motor_configure_command(vec![&DRIVER]), args).await;

        let path = std::env::temp_dir().join(format!("anc-force-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some("ancForce"), TraceMask::FLOW);
        trace.set_device_trace_mask("ancForce", 1, TraceMask::FLOW);
        trace.set_device_trace_info_mask("ancForce", 1, TraceInfoMask::empty());
        trace.set_device_trace_file("ancForce", 1, TraceFile::File(file));
        axes[1].lock().unwrap().force_callback(&AsynUser::default());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            text,
            "motorAxisforceCallback: request card 1, axis 1 status update\n"
        );
    }

    /// Two axes of card 1 on port `port`, axis 1 at 12.5 steps, their logs
    /// attached by drvAsynMotorConfigure, axis 1's trace going to a file
    /// under `mask`.
    async fn traced_axes(
        port: &'static str,
        mask: TraceMask,
    ) -> (Vec<Arc<Mutex<Anc150Axis>>>, std::path::PathBuf) {
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};
        use epics_rs::base::server::iocsh::registry::ArgValue;
        use motor_common::model2::{Model2, drv_asyn_motor_configure_command};
        use motor_common::testing::{LineRig, run_command};

        let driver: &'static Model2 = Box::leak(Box::new(Model2::new(port, 0)));
        let line: &'static str = Box::leak(format!("{port}Line").into_boxed_str());
        let (_line, io) = LineRig::new(line, |last| match last {
            "ver" => "ver\r\nattocube ANC150\r\nOK\r\n".into(),
            c if c.starts_with("getf") => format!("{c}\r\nfrequency = 1000\r\nOK\r\n"),
            c if c.starts_with("getm") => format!("{c}\r\nmode = stp\r\nOK\r\n"),
            _ => String::new(),
        });
        let ctrl = Arc::new(Mutex::new(Anc150Controller::new(io, line).unwrap()));
        let mut axes: Vec<_> = (0..2)
            .map(|axis| Anc150Axis::new(ctrl.clone(), 1, axis).unwrap())
            .collect();
        axes[1].set_position(&AsynUser::default(), 12.5).unwrap();
        let logs = axes.iter().map(Anc150Axis::log).collect();
        let axes: Vec<_> = axes.into_iter().map(|a| Arc::new(Mutex::new(a))).collect();
        let motors = axes
            .iter()
            .map(|a| a.clone() as Arc<Mutex<dyn AsynMotor>>)
            .collect();
        // Idle polls an hour apart: past the startup poll only the test's.
        driver.set_card(1, motors, 3_600_000, 3_600_000);
        driver.set_axis_logs(1, logs);
        let args = vec![
            ArgValue::String(port.into()),
            ArgValue::String(port.into()),
            ArgValue::Int(1),
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

    /// drvANC150Asyn.cc:316-348: the encoder ratio and the gains are refused
    /// with an ERROR line on the axis's log.
    #[epics_rs::base::epics_test]
    async fn refusals_print_the_c_error_lines() {
        let (axes, path) = traced_axes("ancRefuse", TraceMask::ERROR).await;
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
            "motorAxisSetDouble: ANC150 does not support setting encoder ratio\n\
             ANC150 does not support setting proportional gain\n\
             ANC150 does not support setting integral gain\n\
             ANC150 does not support setting derivative gain\n"
        );
    }

    /// drvANC150Asyn.cc:408-410, 489-494, 518: a move, a jog either way and a
    /// stop print their FLOW lines, 0-based axis.
    #[epics_rs::base::epics_test]
    async fn motion_commands_print_the_c_flow_lines() {
        let (axes, path) = traced_axes("ancMotion", TraceMask::FLOW).await;
        let user = AsynUser::default();
        let mut axis = axes[1].lock().unwrap();
        axis.move_absolute(&user, 150.0, 10.0, 200.0, 400.0)
            .unwrap();
        axis.move_relative(&user, -25.5, 0.0, 100.0, 50.0).unwrap();
        axis.move_velocity(&user, 1.0, 300.0, 60.0).unwrap();
        axis.move_velocity(&user, 1.0, -0.0, 60.0).unwrap();
        axis.stop(&user, 80.0).unwrap();
        assert_eq!(
            take_trace(&path),
            "Set card 1, axis 1 move to 150.000000, min vel=10.000000, max_vel=200.000000, accel=400.000000\n\
             Set card 1, axis 1 move to -25.500000, min vel=0.000000, max_vel=100.000000, accel=50.000000\n\
             Set card 1, axis 1 move to 0.000000, min vel=1.000000, max_vel=300.000000, accel=60.000000\n\
             Set card 1, axis 1 move to 0.000000, min vel=1.000000, max_vel=0.000000, accel=60.000000\n\
             Set card 1, axis 1 to stop with accel=80.000000\n"
        );
    }

    /// drvANC150Asyn.cc:633-634: each poll prints the axis's never-set
    /// status and its software position.
    #[epics_rs::base::epics_test]
    async fn the_poll_prints_the_c_iodriver_line() {
        let (axes, path) = traced_axes("ancPoll", TraceMask::IO_DRIVER).await;
        axes[1].lock().unwrap().poll(&AsynUser::default()).unwrap();
        // The poller's own startup poll may land in the file too.
        let text = take_trace(&path);
        assert!(!text.is_empty());
        for line in text.split_inclusive('\n') {
            assert_eq!(
                line,
                "ANC150Poller: axis 1 axisStatus=0, position=12.500000\n"
            );
        }
    }
}
