//! nPoint C300 controller driver (ASCII SCPI-style, over an asyn octet port).
//!
//! Ported from `motorNPoint/nPointApp/src/C300MotorDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` pair). Per the C header note, the C300
//! "behaves more like a temperature controller than a motor controller": the
//! position is a set point, there is no speed/acceleration command, and there
//! is no done-moving indicator — done is *simulated* by comparing the
//! theoretical set point against the monitored encoder position.
//!
//! ## Protocol
//!
//! SCPI-style ASCII commands, one per axis, addressed as `CHAN<n>` (1-based).
//! Commands are terminated by LF (the driver owns output framing; the startup
//! script sets the input EOS). Set commands (`CHAN<n>:POS <value>`) take no
//! reply; queries (`...?`) are written and their reply read.
//!
//! - Startup unlocks the controller with `SYS:PASS:CEN "nPoint"`.
//! - Per axis, `SYST:CHAN<n>:STAG:RANG?` confirms the stage (and marks the axis
//!   as a servo with encoder / gain support), and `SYST:CHAN<n>:DI:FACT?` reads
//!   the digital-input scale factor used to convert the monitor reading.
//! - `move` writes `CHAN<n>:POS <target>`; relative moves add the cached
//!   theoretical position.
//! - `poll` reads `CHAN<n>:DATA?` (a comma-separated record; the corrected
//!   position-monitor value and the analog-in value are parsed from fixed byte
//!   offsets, matching the C `sscanf(&inString[112]/[28])`) to form the encoder
//!   position, then reads `CHAN<n>:POS?` for the theoretical position.
//!
//! ## Units and scaling
//!
//! The C `move`/readback work in the controller's native units: both uses of
//! the `bitsPerUnit_` scale factor are commented out in the C source, so it is
//! dead there and is not carried here. Positions cross the asyn-rs motor
//! boundary (dial-frame EGU) unscaled: `MRES` is 1. The encoder position is
//! derived from the monitor values via the digital-input scale factor, matching
//! C, so it lands in the same units as the theoretical position for the done
//! comparison.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the reads run inside
//!   [`poll`](AsynMotor::poll).
//! - The C300 has no limit switches, home, speed or acceleration; those are
//!   reported clear / are no-ops, matching C.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynResult, OctetStep};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::atof;
use motor_common::{ControllerParams, ControllerReport};

/// Response buffer size (C `MAX_C300_STRING_SIZE`).
const READ_BUF: usize = 300;

/// Command terminator; the driver owns output framing.
const TERMINATOR: &[u8] = b"\n";

/// Byte offset of the corrected position-monitor value in the `DATA?` record
/// (C `C300_DATA_OFFSET`).
const DATA_OFFSET: usize = 112;

/// Byte offset of the analog-in scaled value in the `DATA?` record
/// (C `C300_CORR_OFFSET`).
const CORR_OFFSET: usize = 28;

/// Done-tolerance in controller units (C global `C300Tolerance`).
const TOLERANCE: f64 = 256.0;

/// Parse a leading float from a fixed byte offset into an ASCII record, matching
/// the C `sscanf(&inString[offset], "%lG", ...)`. Returns `None` if the record
/// is shorter than the offset.
fn parse_at(record: &str, offset: usize) -> Option<f64> {
    record.get(offset..).map(|tail| atof(tail.trim_start()))
}

/// Shared controller endpoint owning the asyn octet handle.
pub struct C300Controller {
    handle: SyncIOHandle,
    num_axes: usize,
    /// Each axis's positions, by axis number, where the report reads them.
    axes: Vec<AxisState>,
    /// C `inString_`: what the last read delivered, whatever its status. A
    /// failed write skips the read and leaves it as it was.
    in_string: String,
}

/// C `C300Axis`'s `bitsPerUnit_`, `encoderPosition_` and `theoryPosition_`.
#[derive(Default)]
struct AxisState {
    /// 2^20 over the stage range, 0 while the range is unread.
    bits_per_unit: f64,
    encoder_position: f64,
    /// The theoretical (set-point) position, for relative moves and the
    /// simulated done comparison.
    theory_position: f64,
}

impl C300Controller {
    /// Connect to a C300 and unlock it (`SYS:PASS:CEN "nPoint"`), matching the C
    /// constructor. Performs blocking I/O.
    pub fn new(handle: SyncIOHandle, num_axes: usize) -> Self {
        let axes = (0..num_axes).map(|_| AxisState::default()).collect();
        let ctrl = Self {
            handle,
            num_axes,
            axes,
            in_string: String::new(),
        };
        // C ignores the unlock's status (C300MotorDriver.cpp:68-69).
        let _ = ctrl.write_only("SYS:PASS:CEN \"nPoint\"");
        ctrl
    }

    /// Number of axes configured for this controller.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(cmd: &str) -> Vec<u8> {
        let mut out = Vec::with_capacity(cmd.len() + TERMINATOR.len());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Write a command with no reply expected (C `writeController`), and
    /// print it whatever the write returned (C300MotorDriver.cpp:171-172).
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        let status = self.handle.write_octet(0, &Self::framed(cmd)).map(drop);
        self.handle.print(
            0,
            TraceMask::IO_DRIVER,
            format_args!("C300MotorDriver:writeController: {cmd}\n"),
        );
        status
    }

    /// Write a query and return its reply (C `writeReadController`), printing
    /// the command and `inString_` whatever the exchange returned
    /// (C300MotorDriver.cpp:203-204).
    fn query(&mut self, cmd: &str) -> AsynResult<String> {
        let (raw, status) = self
            .handle
            .write_read(0, &Self::framed(cmd), READ_BUF)
            .into_parts();
        let read = match &status {
            Err(e) => e.octet_step() == Some(OctetStep::Read),
            Ok(()) => true,
        };
        if read {
            let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
            self.in_string = String::from_utf8_lossy(&raw[..end]).into_owned();
        }
        self.handle.print(
            0,
            TraceMask::IO_DRIVER,
            format_args!(
                "C300MotorDriver:writeReadController: output: {cmd}  input: {}\n",
                self.in_string
            ),
        );
        status?;
        Ok(self.in_string.trim_matches(['\r', '\n', '\0']).to_string())
    }

    /// Probe axis `index` (0-based) at construction: query the stage range (to
    /// confirm the axis, mark encoder/gain support and set `bitsPerUnit_`)
    /// and the digital-input scale factor. Returns `(di_scale_factor,
    /// has_encoder)`.
    fn probe_axis(&mut self, index: usize) -> (f64, bool) {
        let chan = index + 1;
        let range = self
            .query(&format!("SYST:CHAN{chan}:STAG:RANG?"))
            .map(|r| atof(&r));
        if let (Ok(range), Some(axis)) = (&range, self.axes.get_mut(index)) {
            // 2^20 bits per total range (C300MotorDriver.cpp:232-233).
            axis.bits_per_unit = 1_048_576.0 / range;
        }
        let has_encoder = range.is_ok_and(|r| r != 0.0);

        let di_scale_factor = self
            .query(&format!("SYST:CHAN{chan}:DI:FACT?"))
            .map(|r| atof(&r))
            .ok()
            .filter(|&f| f != 0.0)
            .unwrap_or(1.0);

        (di_scale_factor, has_encoder)
    }

    fn axis(&mut self, index: usize) -> &mut AxisState {
        &mut self.axes[index]
    }
}

/// The C300 controller's report on its asyn port, with its poll periods in
/// seconds.
pub struct C300Params {
    pub controller: Arc<Mutex<C300Controller>>,
    pub moving_poll: f64,
    pub idle_poll: f64,
}

impl ControllerParams for C300Params {
    /// C `C300Controller::report` (C300MotorDriver.cpp:108-132).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "C300 motor driver {}, numAxes={}, moving poll period={:.6}, idle poll period={:.6}",
            ctx.port_name(),
            ctx.num_axes(),
            CFloat(self.moving_poll),
            CFloat(self.idle_poll),
        );
        if level > 0 {
            let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
            for (axis, state) in ctrl.axes.iter().enumerate() {
                // C prints `currentLimits_`, which nothing sets.
                let _ = write!(
                    out,
                    "  axis {axis}\n  bitsPerUnit_ = {:.6}\n    encoder position={:.6}\n    \
                     theory position={:.6}\n    limits=0x0\n",
                    CFloat(state.bits_per_unit),
                    CFloat(state.encoder_position),
                    CFloat(state.theory_position),
                );
            }
        }
        ctx.report(out, level);
    }
}

/// One C300 axis sharing a controller. Implements [`AsynMotor`].
pub struct C300Axis {
    controller: Arc<Mutex<C300Controller>>,
    /// Axis number, 0-based.
    index: usize,
    /// Wire channel name, `CHAN<n>` (1-based).
    chan: String,
    di_scale_factor_inv: f64,
    has_encoder: bool,
}

impl C300Axis {
    /// Construct axis `index` (0-based), probing its stage range and DI factor.
    pub fn new(controller: Arc<Mutex<C300Controller>>, index: usize) -> Self {
        let (di_scale_factor, has_encoder) = controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .probe_axis(index);
        Self {
            controller,
            index,
            chan: format!("CHAN{}", index + 1),
            di_scale_factor_inv: 1.0 / di_scale_factor,
            has_encoder,
        }
    }

    fn lock(&self) -> MutexGuard<'_, C300Controller> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl AsynMotor for C300Axis {
    /// C `C300Axis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (C300MotorDriver.cpp).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    /// C300MotorDriver.cpp:250.
    fn constructor_calls_param_callbacks(&self) -> bool {
        true
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!("{}:POS {}", self.chan, position))
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C: relative target = cached theoretical position + distance.
        let mut ctrl = self.lock();
        let target = ctrl.axis(self.index).theory_position + distance;
        ctrl.write_only(&format!("{}:POS {}", self.chan, target))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // No jog on the C300 (no speed command).
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // No home on the C300.
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // No stop command; a move is a set point.
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // No set-position command on the C300.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, _enable: bool) -> AsynResult<()> {
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut ctrl = self.lock();
        let data = ctrl.query(&format!("{}:DATA?", self.chan))?;
        // Encoder position from the monitor record (fixed-offset fields).
        let pos_mon = parse_at(&data, DATA_OFFSET).unwrap_or(0.0);
        let analog_in = parse_at(&data, CORR_OFFSET).unwrap_or(0.0);
        let encoder_position = (pos_mon - analog_in) * self.di_scale_factor_inv;
        ctrl.axis(self.index).encoder_position = encoder_position;

        // Theoretical (set-point) position.
        let pos_query = ctrl.query(&format!("{}:POS?", self.chan))?;
        let theory_position = atof(&pos_query);
        ctrl.axis(self.index).theory_position = theory_position;
        drop(ctrl);

        // Simulated done: within tolerance of the encoder position.
        let done = (theory_position - encoder_position).abs() <= TOLERANCE;

        Ok(MotorStatus {
            position: theory_position,
            encoder_position,
            velocity: 0.0,
            done,
            moving: !done,
            has_encoder: self.has_encoder,
            gain_support: self.has_encoder,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;

    #[test]
    fn the_constructor_calls_back() {
        let (handle, _) = SyncIOHandle::connect_port(
            "C300_CALLBACK",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let controller = Arc::new(Mutex::new(C300Controller::new(handle, 1)));
        assert!(C300Axis::new(controller, 0).constructor_calls_param_callbacks());
    }

    #[test]
    fn a_controller_of_a_missing_port_is_still_built() {
        let (handle, _) = SyncIOHandle::connect_port(
            "C300_NONE",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let controller = Arc::new(Mutex::new(C300Controller::new(handle, 1)));
        let axis = C300Axis::new(controller, 0);
        assert!(!axis.has_encoder);
    }

    /// C prints `inString_` after every exchange: what a failed read
    /// delivered, and the previous reply when the write failed.
    #[test]
    fn write_read_controller_prints_the_reply_whatever_the_status() {
        use std::sync::Mutex as StdMutex;

        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use motor_common::testing::LineRig;

        let name = "c300TraceLine";
        let (rig, handle) = LineRig::failing(
            name,
            |cmd| match cmd {
                c if c.starts_with("CHAN1:DATA?") => "D1".into(),
                _ => "P1".into(),
            },
            |line| line.starts_with("CHAN1:POS?"),
        );
        let path = std::env::temp_dir().join(format!("c300-trace-{}", std::process::id()));
        let file = Arc::new(StdMutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_trace_file(Some(name), TraceFile::File(file.clone()));
        trace.set_device_trace_mask(name, 0, TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_device_trace_file(name, 0, TraceFile::File(file));

        let mut ctrl = C300Controller::new(handle, 1);
        assert_eq!(ctrl.query("CHAN1:DATA?").unwrap(), "D1");
        assert!(ctrl.query("CHAN1:POS?").is_err());
        drop(rig);

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let mine: Vec<_> = text
            .lines()
            .filter_map(|l| l.split_once("C300MotorDriver:").map(|(_, m)| m))
            .collect();
        assert_eq!(
            mine,
            [
                "writeController: SYS:PASS:CEN \"nPoint\"",
                "writeReadController: output: CHAN1:DATA?  input: D1",
                "writeReadController: output: CHAN1:POS?  input: D1",
            ],
            "{text}"
        );
    }

    /// A reply cut by a timeout is what C's line prints.
    #[test]
    fn a_cut_reply_is_printed() {
        use std::sync::Mutex as StdMutex;

        use epics_rs::asyn::manager::PortManager;
        use epics_rs::asyn::trace::TraceFile;
        use motor_common::testing::LineRig;

        let name = "c300CutLine";
        let (_rig, handle) = LineRig::cut(name, |_| "12.5".into(), |c| c.starts_with("CHAN1"));
        let path = std::env::temp_dir().join(format!("c300-cut-{}", std::process::id()));
        let file = Arc::new(StdMutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(name), TraceMask::IO_DRIVER);
        trace.set_trace_file(Some(name), TraceFile::File(file.clone()));
        trace.set_device_trace_mask(name, 0, TraceMask::IO_DRIVER);
        trace.set_device_trace_file(name, 0, TraceFile::File(file));

        let mut ctrl = C300Controller::new(handle, 1);
        assert!(ctrl.query("CHAN1:POS?").is_err());

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.lines().any(|l| l
                .ends_with("C300MotorDriver:writeReadController: output: CHAN1:POS?  input: 12.5")),
            "{text}"
        );
    }

    #[test]
    fn parse_at_reads_offset_field() {
        // Two space-padded fields; parse the leading float at each offset.
        let record = "aaaa 12.5,bbbb 99.75";
        assert_eq!(parse_at(record, 4), Some(12.5));
        assert_eq!(parse_at(record, 14), Some(99.75));
    }

    #[test]
    fn parse_at_short_record_is_none() {
        assert_eq!(parse_at("short", 112), None);
    }

    #[test]
    fn done_uses_tolerance() {
        // |theory - encoder| within 256 -> done; outside -> moving.
        let within = (1000.0_f64 - 1200.0).abs() <= TOLERANCE;
        let outside = (1000.0_f64 - 1300.0).abs() <= TOLERANCE;
        assert!(within);
        assert!(!outside);
    }
}
