//! Parker OEM750 series controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorParker/parkerApp/src/OEMMotorDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` pair). Commands are ASCII, prefixed by
//! the 1-based unit address (`<addr>` = axis + 1), CR-terminated by the port
//! EOS, CR both ways as the C constructor sets it. Connects over a
//! `drvAsynIPPort`.
//!
//! ## Reply model
//!
//! The controller echoes each command and then, for a query, sends a data line.
//! Command-only requests (`MN`, `A`, `V`, `D`, `G`, `S`, `H`, `MC`, `FSC`, and
//! the configuration commands) read a single acknowledgement line; queries
//! (`R`, `PR`, `W3`, `IS`) read the echo and then the data line.
//!
//! ## Units
//!
//! Positions (`D`, `PR`, `W3`) are record steps. `OEM_RESOLUTION` sets the
//! controller's `MR` and C's `pulsesPerUnit_`, which divides the step velocity
//! into the `V` revolutions per second.
//!
//! ## Positioning mode and relative moves
//!
//! `configureController` puts the axis in absolute mode (`MPA`), so `D` is always
//! an absolute target. The C `move` ignores its `relative` flag (a latent bug for
//! relative moves in absolute mode); this port instead honours it by converting a
//! relative move to an absolute target from the last polled position.
//!
//! `OEM_SELECT_SWITCH` picks which detected switch is the home/low-limit end;
//! the poll posts `OEM_SWITCH_DETEC`, the switch it found.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PortSets,
};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{
    ControllerParams, FinishedWrite, WriteHook, WriteOutcome, WriteValue,
};

use motor_common::ControllerReport;
use motor_common::util::{atof, leading_hex};

/// Response buffer size.
const READ_BUF: usize = 64;

/// 2^32, subtracted to sign-extend a negative 32-bit step count (C).
const TWO_POW_32: f64 = 4_294_967_296.0;

/// `pulsesPerUnit_` until `OEM_RESOLUTION` is written. C leaves the member
/// uninitialised in a fresh heap block, so its constructor sends `MR0`.
const DEFAULT_PULSES_PER_UNIT: f64 = 0.0;

/// Shared OEM750 controller endpoint owning the asyn octet handle.
pub struct OemController {
    handle: SyncIOHandle,
    /// C `OEMAxis::pulsesPerUnit_`, per axis.
    pulses_per_unit: Vec<f64>,
    /// The last `OEM_SELECT_SWITCH` write, per axis, for the poll to read
    /// without asking the port.
    select_switch: Vec<i32>,
    /// `OEM_SWITCH_DETEC`'s index.
    switch_detected: Option<usize>,
}

impl OemController {
    /// Wrap a connected octet handle. The C controller constructor connects,
    /// sets the CR EOS, creates the axes and starts the poller; here
    /// `OEMCreateController` sets the EOS and creates the axes.
    pub fn new(handle: SyncIOHandle, num_axes: usize) -> Self {
        Self {
            handle,
            pulses_per_unit: vec![DEFAULT_PULSES_PER_UNIT; num_axes],
            // C reads OEM_SELECT_SWITCH unset (OEMMotorDriver.cpp:397) and
            // polls on an uninitialised `selectSwitch` (:338); rs starts at 0.
            select_switch: vec![0; num_axes],
            switch_detected: None,
        }
    }

    fn trimmed(raw: &[u8]) -> String {
        String::from_utf8_lossy(raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string()
    }

    /// Send a command and read its single acknowledgement line.
    fn command(&self, cmd: &str) -> AsynResult<()> {
        let (_ack, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        status?;
        Ok(())
    }

    /// Send a query and read the echo line, then the data line with its own
    /// status: C parses `inString_` after `readController` whatever it
    /// returned (OEMMotorDriver.cpp:349-385), and a failed read leaves there
    /// what it delivered. Only the echo's failure fails the query.
    fn query(&self, cmd: &str) -> AsynResult<(String, AsynResult<()>)> {
        let (_echo, status) = self
            .handle
            .write_read(0, cmd.as_bytes(), READ_BUF)
            .into_parts();
        status?;
        let (raw, status) = self.handle.read_octet(0, READ_BUF).into_parts();
        Ok((Self::trimmed(&raw), status))
    }
}

/// One OEM750 axis sharing a controller. Implements [`AsynMotor`].
pub struct OemAxis {
    controller: Arc<Mutex<OemController>>,
    axis_no: usize,
    /// 1-based unit address used to prefix every command.
    address: i32,
    /// Set by `home`, cleared by `move` (drives the poll's auto-zero step).
    homed: bool,
    /// Persistent base position from `PR` (added to the live `W3` step).
    encoder_base: f64,
    /// Last reported position (used to resolve relative moves to absolute).
    position: f64,
    /// The poll's `OEM_SWITCH_DETEC`, for [`AsynMotor::take_port_sets`].
    sets: PortSets,
}

impl OemAxis {
    /// Construct axis `axis_no` (0-based; unit address = axis_no + 1) and run the
    /// controller configuration sequence, matching the C `OEMAxis` constructor.
    pub fn new(controller: Arc<Mutex<OemController>>, axis_no: usize) -> Self {
        let axis = Self {
            controller,
            axis_no,
            address: axis_no as i32 + 1,
            homed: false,
            encoder_base: 0.0,
            position: 0.0,
            sets: PortSets::default(),
        };
        // C ignores configureController's status (OEMMotorDriver.cpp:208).
        let _ = axis.lock().configure(axis.axis_no);
        axis
    }

    fn lock(&self) -> MutexGuard<'_, OemController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Shared absolute move: normal mode, ramp time, velocity, target, go.
    fn do_move(
        &mut self,
        target: f64,
        velocity: f64,
        min_velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let a = ramp_time(velocity, min_velocity, acceleration);
        let addr = self.address;
        let ctrl = self.lock();
        let v = velocity / ctrl.pulses_per_unit[self.axis_no];
        // Each command's status overwrites the one before
        // (OEMMotorDriver.cpp:230-253).
        let _ = ctrl.command(&format!("{addr}MN"));
        let _ = ctrl.command(&format!("{addr}A{a:.2}", a = CFloat(a)));
        let _ = ctrl.command(&format!("{addr}V{v:.2}", v = CFloat(v)));
        let _ = ctrl.command(&format!("{addr}D{target:.0}", target = CFloat(target)));
        let status = ctrl.command(&format!("{addr}G"));
        drop(ctrl);
        self.homed = false;
        status
    }
}

impl OemController {
    /// Re-apply the axis configuration (echo on, SSA0, absolute mode, MR). The
    /// C driver runs this at construction and at the head of every poll.
    fn configure(&self, axis_no: usize) -> AsynResult<()> {
        let addr = axis_no + 1;
        self.command(&format!("{addr}E"))?;
        self.command(&format!("{addr}SSA0"))?;
        self.command(&format!("{addr}MPA"))?;
        self.command(&format!("{addr}MR{}", self.pulses_per_unit[axis_no] as i32))
    }
}

/// `OEMController`'s parameters; the `OEM_RESOLUTION` write hook runs `MR`.
pub struct OemParams {
    controller: Arc<Mutex<OemController>>,
    resolution: Option<usize>,
    select_switch: Option<usize>,
    /// C's `movingPollPeriod_`/`idlePollPeriod_`, in seconds.
    moving_poll_s: f64,
    idle_poll_s: f64,
}

impl OemParams {
    pub fn new(
        controller: Arc<Mutex<OemController>>,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
    ) -> Self {
        Self {
            controller,
            resolution: None,
            select_switch: None,
            moving_poll_s: moving_poll_ms as f64 / 1000.0,
            idle_poll_s: idle_poll_ms as f64 / 1000.0,
        }
    }
}

/// `OEMController::writeInt32` / `writeFloat64` after the base: for
/// `OEM_RESOLUTION` the `MR` command, whose status the write returns, then
/// the closing line, which the record's writes reach as well as the port's.
struct OemWriteHook {
    controller: Arc<Mutex<OemController>>,
    resolution: Option<usize>,
    select_switch: Option<usize>,
}

impl WriteHook for OemWriteHook {
    fn after_write(&self, write: &FinishedWrite<'_>) -> WriteOutcome {
        let function = write.function;
        if let WriteValue::Int32(v) = write.value
            && self.select_switch == Some(function)
        {
            let mut ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(slot) = ctrl.select_switch.get_mut(write.addr as usize) {
                *slot = v;
            }
        }
        let (method, value, mr) = match write.value {
            WriteValue::Int32(v) => ("writeInt32", v.to_string(), Some(v)),
            WriteValue::Float64(v) => ("writeFloat64", format!("{v:.6}", v = CFloat(v)), None),
        };
        let status = match mr.filter(|_| self.resolution == Some(function)) {
            Some(v) => {
                let mut ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
                let status = ctrl.command(&format!("{}MR{v}", write.addr + 1));
                ctrl.pulses_per_unit[write.addr as usize] = f64::from(v);
                Some(status)
            }
            None => None,
        };
        match status.as_ref().unwrap_or(write.status) {
            Err(e) => write.print(
                TraceMask::ERROR,
                format_args!(
                    "OEMMotorDriver:{method}: error, status={} function={function}, value={value}\n",
                    e.c_status_code()
                ),
            ),
            Ok(()) => write.print(
                TraceMask::IO_DRIVER,
                format_args!("OEMMotorDriver:{method}: function={function}, value={value}\n"),
            ),
        }
        WriteOutcome {
            status,
            ..WriteOutcome::default()
        }
    }
}

/// No `write_int32`/`write_float64`: C dereferences the null axis of an
/// address with no axis (OEMMotorDriver.cpp:130-135, 171-175); rs refuses
/// there as the base does.
impl ControllerParams for OemParams {
    /// C `OEMController::report` (OEMMotorDriver.cpp:94-101); `OEMAxis::report`
    /// adds nothing.
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "OEM motor driver {}, numAxes={}, moving poll period={:.6}, idle poll period={:.6}",
            ctx.port_name(),
            ctx.num_axes(),
            CFloat(self.moving_poll_s),
            CFloat(self.idle_poll_s)
        );
        ctx.report(out, level);
    }

    /// OEMMotorDriver.cpp:48-50, in C's order.
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        self.select_switch = Some(base.create_param("OEM_SELECT_SWITCH", ParamType::Int32)?);
        let detected = base.create_param("OEM_SWITCH_DETEC", ParamType::Int32)?;
        self.resolution = Some(base.create_param("OEM_RESOLUTION", ParamType::Int32)?);
        let mut ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        ctrl.switch_detected = Some(detected);
        Ok(())
    }

    fn write_hook(&self) -> Option<Arc<dyn WriteHook>> {
        Some(Arc::new(OemWriteHook {
            controller: self.controller.clone(),
            resolution: self.resolution,
            select_switch: self.select_switch,
        }))
    }
}

/// Acceleration ramp time `A` = (velocity - min_velocity) / acceleration (C).
///
/// C divides by a zero acceleration and sends `A` with `%.2f` of inf or nan
/// (OEMMotorDriver.cpp:238); here a zero acceleration sends `A0.00`.
fn ramp_time(velocity: f64, min_velocity: f64, acceleration: f64) -> f64 {
    if acceleration != 0.0 {
        (velocity - min_velocity) / acceleration
    } else {
        0.0
    }
}

impl AsynMotor for OemAxis {
    /// C `OEMAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (OEMMotorDriver.cpp).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    fn take_port_sets(&mut self) -> PortSets {
        std::mem::take(&mut self.sets)
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, velocity, min_velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // The controller is in absolute mode (MPA); resolve the delta to an
        // absolute target from the last polled position.
        let target = self.position + distance;
        self.do_move(target, velocity, min_velocity, acceleration)
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // Not implemented in the C OEM driver.
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let addr = self.address;
        let ctrl = self.lock();
        let v = velocity / ctrl.pulses_per_unit[self.axis_no];
        // As the move (OEMMotorDriver.cpp:255-280).
        let _ = ctrl.command(&format!("{addr}MC"));
        let _ = ctrl.command(&format!("{addr}V{v:.2}", v = CFloat(v)));
        // C maps forward -> H-, reverse -> H+.
        let _ = ctrl.command(&format!("{addr}H{}", if forward { '-' } else { '+' }));
        let status = ctrl.command(&format!("{addr}G"));
        drop(ctrl);
        self.homed = true;
        status
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let addr = self.address;
        self.lock().command(&format!("{addr}S"))
    }

    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        // The C OEM driver has no set-position support.
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let addr = self.address;
        self.lock()
            .command(&format!("{addr}FSC{}", i32::from(enable)))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // Not implemented in the C OEM driver.
        Ok(())
    }

    /// A failed read returns at `skip` as C returns asynError, keeping what
    /// the reads before it set (OEMMotorDriver.cpp:340-425).
    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        let addr = self.address;
        // The field, not `self`: the poll pushes its params with it held.
        let ctrl = self.controller.lock().unwrap_or_else(|e| e.into_inner());
        ctrl.configure(self.axis_no)?;

        // Moving flags: "*B"/"*C" mean in motion.
        let (flags, _) = ctrl.query(&format!("{addr}R"))?;
        let done = flags != "*B" && flags != "*C";
        status.done = done;
        status.moving = !done;

        if done {
            // Position readback (skip the leading status char); C posts it
            // only from a good read (:357-361).
            let (pr, read) = ctrl.query(&format!("{addr}PR"))?;
            if read.is_ok() {
                self.encoder_base = atof(pr.get(1..).unwrap_or(""));
                status.encoder_position = self.encoder_base;
                status.position = self.encoder_base;
            }
        } else {
            // Live step count in hex (skip the leading char); "*F" flags negative.
            let (w3, _) = ctrl.query(&format!("{addr}W3"))?;
            let mut step = leading_hex(w3.get(1..).unwrap_or("")).unwrap_or(0) as f64;
            status.direction = !w3.contains("*F");
            if !status.direction {
                step -= TWO_POW_32;
            }
            status.encoder_position = self.encoder_base + step;
            status.position = status.encoder_position;
        }
        self.position = status.position;

        // Switch status (character flags at fixed offsets 6 and 7).
        // C's poll fails on this read's status, or on the PZ's below.
        let (is, mut read) = ctrl.query(&format!("{addr}IS"))?;
        let switch_detected = switch_from_is(&is);
        let select_switch = ctrl.select_switch[self.axis_no];
        // Set ahead of the PZ that may fail: C's poll posts it either way
        // (OEMMotorDriver.cpp:387-395, :422-423).
        if let Some(reason) = ctrl.switch_detected {
            self.sets.axis(reason, ParamValue::Int32(switch_detected));
        }

        // C sets only the flags the switch names; nothing clears homed
        // (:398-419). It sets motorStatusAtHome_ and motorStatusHome_
        // together.
        if switch_detected > 0 {
            if select_switch == switch_detected - 1 {
                if self.homed {
                    read = ctrl.command(&format!("{addr}PZ"));
                    status.homed = true;
                }
                status.low_limit = true;
                status.home = true;
                status.encoder_home = true;
            } else {
                status.high_limit = true;
            }
        } else {
            status.high_limit = false;
            status.low_limit = false;
            status.home = false;
            status.encoder_home = false;
        }
        status.problem = read.is_err();
        read
    }
}

/// Decode the `IS` reply into a detected-switch code: 1 if the flag at offset 6
/// is set, 2 if the flag at offset 7 is set, else 0 (C `poll`).
fn switch_from_is(is: &str) -> i32 {
    let b = is.as_bytes();
    if b.get(6) == Some(&b'1') {
        1
    } else if b.get(7) == Some(&b'1') {
        2
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::InterfaceMask;
    use std::collections::HashMap;

    use epics_rs::asyn::interfaces::InterfaceType;
    use epics_rs::asyn::interfaces::motor::PortSet;
    use epics_rs::asyn::manager::PortManager;
    use epics_rs::asyn::port::DrvUserRequest;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::motor::builder::MotorBuilder;
    use epics_rs::motor::controller_port::index;
    use epics_rs::motor::devices::{register_axis, register_controller_port};
    use epics_rs::motor::trace::TraceUser;
    use motor_common::testing::Polled as _;

    /// C overwrites each command's status with the next one's and sets
    /// `homed` whatever they returned (OEMMotorDriver.cpp:230-280).
    #[test]
    fn a_refused_command_still_sends_the_ones_after_it() {
        use motor_common::testing::LineRig;
        let (rig, io) = LineRig::failing(
            "oemRefused",
            |_| "OK".into(),
            |cmd| ["1MC", "1H-", "1MN"].contains(&cmd),
        );
        let controller = Arc::new(Mutex::new(OemController::new(io, 1)));
        controller.lock().unwrap().pulses_per_unit[0] = 2.0;
        let mut axis = OemAxis::new(controller, 0);
        rig.sent();
        let user = AsynUser::new(0);
        axis.home(&user, 0.0, 3.0, 4.0, true).unwrap();
        assert!(axis.homed);
        axis.move_absolute(&user, 7.0, 1.0, 3.0, 4.0).unwrap();
        assert!(!axis.homed);
        assert_eq!(
            rig.sent(),
            [
                "1MC", "1V1.50", "1H-", "1G", "1MN", "1A0.50", "1V1.50", "1D7", "1G"
            ]
        );
    }

    /// OEMMotorDriver.cpp:345-389: a refused IS skips after the R and PR
    /// reads set Done and the position.
    #[test]
    fn a_failed_read_keeps_what_the_reads_before_it_set() {
        use motor_common::testing::LineRig;
        let (_rig, io) = LineRig::failing(
            "oemSkip",
            |cmd| {
                match cmd {
                    "1R" => "*R",
                    "1PR" => "*+40",
                    _ => "*",
                }
                .into()
            },
            |cmd| cmd == "1IS",
        );
        let controller = Arc::new(Mutex::new(OemController::new(io, 1)));
        let mut axis = OemAxis::new(controller, 0);
        let mut status = MotorStatus::default();
        assert!(axis.poll(&AsynUser::new(0), &mut status).is_err());
        assert!(status.done);
        assert_eq!((status.position, status.encoder_position), (40.0, 40.0));
    }

    /// A refused `G` is what the move and the home return, and C sets
    /// `homed` after it all the same (OEMMotorDriver.cpp:247-250, :274-277).
    #[test]
    fn a_refused_go_still_sets_homed() {
        use motor_common::testing::LineRig;
        let (_rig, io) = LineRig::failing("oemRefusedGo", |_| "OK".into(), |cmd| cmd == "1G");
        let controller = Arc::new(Mutex::new(OemController::new(io, 1)));
        controller.lock().unwrap().pulses_per_unit[0] = 2.0;
        let mut axis = OemAxis::new(controller, 0);
        let user = AsynUser::new(0);
        assert!(axis.home(&user, 0.0, 3.0, 4.0, true).is_err());
        assert!(axis.homed);
        assert!(axis.move_absolute(&user, 7.0, 1.0, 3.0, 4.0).is_err());
        assert!(!axis.homed);
    }

    #[test]
    fn an_axis_of_a_missing_port_is_still_built() {
        let (handle, _) = SyncIOHandle::connect_port(
            "OEM_NONE",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let controller = Arc::new(Mutex::new(OemController::new(handle, 1)));
        assert_eq!(OemAxis::new(controller, 0).address, 1);
    }

    /// A port that acknowledges every command, recording it.
    struct Acks {
        base: PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
    }

    impl epics_rs::asyn::port::PortDriver for Acks {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            self.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            buf[0] = b'\r';
            epics_rs::asyn::interpose::OctetReadResult::ok(
                1,
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
    }

    /// OEMMotorDriver.cpp:208, 320: the constructor's configureController
    /// sends `MR` from a `pulsesPerUnit_` no resolution write has set yet.
    #[test]
    fn boot_configuration_sends_mr0() {
        use epics_rs::asyn::port::PortFlags;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let name = "OEM_BOOT";
        let written = Arc::new(Mutex::new(Vec::new()));
        let driver = Acks {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: written.clone(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let handle = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_secs(1),
        );
        let controller = Arc::new(Mutex::new(OemController::new(handle, 1)));
        let _axis = OemAxis::new(controller, 0);
        assert_eq!(*written.lock().unwrap(), ["1E", "1SSA0", "1MPA", "1MR0"]);
    }

    /// A port that acknowledges every command and answers the poll's
    /// queries (`R`, `PR`, `IS`) with an echo line then `replies`' line, or
    /// `cut`'s line ended by a timeout.
    struct Polled {
        base: PortDriverBase,
        replies: Arc<Mutex<HashMap<&'static str, &'static str>>>,
        cut: Arc<Mutex<HashMap<&'static str, &'static str>>>,
        queued: std::collections::VecDeque<(String, bool)>,
    }

    impl epics_rs::asyn::port::PortDriver for Polled {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            self.queued.push_back(("\r".into(), false));
            if let Some(partial) = self.cut.lock().unwrap().get(&cmd[1..]) {
                self.queued.push_back((partial.to_string(), true));
            } else if let Some(reply) = self.replies.lock().unwrap().get(&cmd[1..]) {
                self.queued.push_back((format!("{reply}\r"), false));
            }
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            let (line, cut) = self.queued.pop_front().unwrap_or(("\r".into(), false));
            buf[..line.len()].copy_from_slice(line.as_bytes());
            let read = epics_rs::asyn::interpose::OctetReadResult::ok(
                line.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            );
            if !cut {
                return read;
            }
            epics_rs::asyn::interpose::OctetReadResult {
                status: Err(epics_rs::asyn::error::AsynError::Status {
                    status: epics_rs::asyn::error::AsynStatus::Timeout,
                    message: "timeout".into(),
                }),
                ..read
            }
        }
    }

    /// OEMMotorDriver.cpp:385-412: the poll posts the switch it found on
    /// OEM_SWITCH_DETEC, and OEM_SELECT_SWITCH picks which one is home.
    #[test]
    fn the_selected_switch_is_the_home_end() {
        use epics_rs::asyn::port::PortFlags;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let name = "OEM_SWITCH_LINE";
        let replies = Arc::new(Mutex::new(HashMap::from([
            ("R", "*R"),
            ("PR", "*+0"),
            ("IS", "*ABCD001"),
        ])));
        let driver = Polled {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            replies: replies.clone(),
            cut: Default::default(),
            queued: Default::default(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let handle = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_secs(1),
        );
        let ctrl = Arc::new(Mutex::new(OemController::new(handle, 1)));
        let port = register_controller_port(
            &open_owner(),
            "oemSwitch",
            1,
            OemParams::new(ctrl.clone(), 100, 1000),
        )
        .unwrap();
        let reason = |param| {
            port.drv_user_create_blocking(&DrvUserRequest::new(param, 0))
                .unwrap()
                .reason
        };
        let (select, detected) = (reason("OEM_SELECT_SWITCH"), reason("OEM_SWITCH_DETEC"));
        let axis = Arc::new(Mutex::new(OemAxis::new(ctrl, 0)));
        let motor: Arc<Mutex<dyn AsynMotor>> = axis.clone();
        let user = TraceUser::Asyn {
            port: "oemSwitch".into(),
            addr: 0,
        };
        register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
        // The poller's status, which keeps what a poll does not set.
        let mut last = MotorStatus::default();
        let mut poll = || {
            axis.lock()
                .unwrap()
                .poll(&AsynUser::new(0), &mut last)
                .unwrap();
            last.clone()
        };

        // UPDATE_STATUS: the port's poll sets it, as C's poll does.
        port.write_int32_blocking(index::UPDATE_STATUS, 0, 1)
            .unwrap();
        assert_eq!(port.read_int32_blocking(detected, 0).unwrap(), 2);
        let status = poll();
        assert!(status.high_limit && !status.low_limit && !status.home);
        // The poll's only step on the port, on the axis's own address.
        let sets: Vec<_> = axis
            .lock()
            .unwrap()
            .take_port_sets()
            .into_iter()
            .map(|set| format!("{set:?}"))
            .collect();
        assert_eq!(
            sets,
            [format!(
                "{:?}",
                PortSet::Axis {
                    reason: detected,
                    value: ParamValue::Int32(2)
                }
            )]
        );

        // C writes only the flags the switch names; the rest keep their
        // last value (OEMMotorDriver.cpp:398-419).
        port.write_int32_blocking(select, 0, 1).unwrap();
        let status = poll();
        assert!(status.high_limit && status.low_limit && status.home);
        assert!(!status.homed);

        // A home that ends on the switch latches homed: C never clears
        // motorStatusHomed_ (:404).
        axis.lock()
            .unwrap()
            .home(&AsynUser::new(0), 0.0, 1.0, 1.0, true)
            .unwrap();
        assert!(poll().homed);
        replies.lock().unwrap().insert("IS", "*ABCD000");
        let status = poll();
        assert!(!status.high_limit && !status.low_limit && !status.home);
        assert!(status.homed);
        axis.lock()
            .unwrap()
            .move_absolute(&AsynUser::new(0), 5.0, 0.0, 1.0, 1.0)
            .unwrap();
        assert!(poll().homed);
    }

    /// OEMMotorDriver.cpp:345-424: each data line is parsed whatever its
    /// read returned and the poll goes on; it fails on the `IS` read or the
    /// `PZ` after it, having set everything first.
    #[test]
    fn data_lines_cut_by_a_timeout_are_still_parsed() {
        use epics_rs::asyn::port::PortFlags;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let name = "OEM_CUT_LINE";
        let replies = Arc::new(Mutex::new(HashMap::from([
            ("R", "*R"),
            ("PR", "*+40"),
            ("W3", "*10"),
            ("IS", "*ABCD000"),
        ])));
        let cut = Arc::new(Mutex::new(HashMap::new()));
        let driver = Polled {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            replies,
            cut: cut.clone(),
            queued: Default::default(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let handle = SyncIOHandle::from_handle(
            rt.port_handle().clone(),
            0,
            std::time::Duration::from_secs(1),
        );
        let ctrl = Arc::new(Mutex::new(OemController::new(handle, 1)));
        let mut axis = OemAxis::new(ctrl, 0);
        let user = AsynUser::new(0);
        // The poller's status, which keeps what a poll does not set.
        let mut status = MotorStatus::default();
        axis.poll(&user, &mut status).unwrap();
        assert_eq!(status.position, 40.0);

        cut.lock().unwrap().insert("R", "*B");
        axis.poll(&user, &mut status).unwrap();
        assert!(!status.done);
        assert_eq!(status.position, 56.0);

        let cut_only = |query, partial| {
            let mut cut = cut.lock().unwrap();
            cut.clear();
            cut.insert(query, partial);
        };
        cut_only("PR", "*+7");
        axis.poll(&user, &mut status).unwrap();
        assert!(status.done);
        assert_eq!(status.position, 56.0);

        cut_only("IS", "*ABCD01");
        let mut left = MotorStatus::default();
        assert!(axis.poll(&user, &mut left).is_err());
        assert!(left.low_limit && left.home && !left.homed);
        assert_eq!(left.position, 40.0);

        axis.home(&user, 0.0, 1.0, 1.0, true).unwrap();
        assert!(axis.polled(&user).unwrap().homed);
    }

    #[test]
    fn ramp_time_is_delta_v_over_a() {
        assert_eq!(ramp_time(10.0, 2.0, 4.0), 2.0);
        // Zero acceleration must not divide by zero.
        assert_eq!(ramp_time(10.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn switch_decode() {
        // Offsets 6 and 7 hold the switch flags.
        assert_eq!(switch_from_is("ABCDEF10"), 1);
        assert_eq!(switch_from_is("ABCDEF01"), 2);
        assert_eq!(switch_from_is("ABCDEF00"), 0);
        // Short replies decode as no switch.
        assert_eq!(switch_from_is("AB"), 0);
    }

    /// `body`'s lines in `port`'s trace at C's error and IO_DRIVER masks.
    fn traced(port: &str, body: impl FnOnce()) -> Vec<String> {
        let path = std::env::temp_dir().join(format!("oem-trace-{port}-{}", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        trace.set_trace_file(Some(port), TraceFile::File(file.clone()));
        body();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        text.lines().map(str::to_owned).collect()
    }

    #[test]
    fn every_write_ends_on_cs_line_the_base_included() {
        let (handle, _) = SyncIOHandle::connect_port(
            "OEM_NONE",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let ctrl = Arc::new(Mutex::new(OemController::new(handle, 2)));
        let port = register_controller_port(
            &open_owner(),
            "oemTrace",
            2,
            OemParams::new(ctrl.clone(), 100, 1000),
        )
        .unwrap();
        let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(OemAxis::new(ctrl, 1)));
        let user = TraceUser::Asyn {
            port: "oemTrace".into(),
            addr: 1,
        };
        register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
        let lines = traced("oemTrace", || {
            port.write_float64_blocking(index::VELOCITY, 1, 2.5)
                .unwrap();
            port.write_int32_blocking(index::REC_DIRECTION, 1, 1)
                .unwrap();
        });
        let mine: Vec<_> = lines
            .iter()
            .filter(|l| l.contains("OEMMotorDriver"))
            .collect();
        let velocity = index::VELOCITY;
        let direction = index::REC_DIRECTION;
        let want = [
            format!("OEMMotorDriver:writeFloat64: function={velocity}, value=2.500000"),
            format!("OEMMotorDriver:writeInt32: function={direction}, value=1"),
        ];
        assert_eq!(mine.len(), want.len(), "{lines:?}");
        for (line, want) in mine.iter().zip(&want) {
            assert!(line.ends_with(want.as_str()), "{line:?} vs {want:?}");
        }
    }

    /// OEMMotorDriver.cpp:188 `value=%f`: glibc's -nan, sign bit included.
    #[test]
    fn a_nan_write_traces_as_c_prints_it() {
        let (handle, _) = SyncIOHandle::connect_port(
            "OEM_NONE",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let ctrl = Arc::new(Mutex::new(OemController::new(handle, 2)));
        let port = register_controller_port(
            &open_owner(),
            "oemNanTrace",
            2,
            OemParams::new(ctrl.clone(), 100, 1000),
        )
        .unwrap();
        let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(OemAxis::new(ctrl, 1)));
        let user = TraceUser::Asyn {
            port: "oemNanTrace".into(),
            addr: 1,
        };
        register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
        let lines = traced("oemNanTrace", || {
            port.write_float64_blocking(index::VELOCITY, 1, -f64::NAN)
                .unwrap();
        });
        let want = format!(
            "OEMMotorDriver:writeFloat64: function={}, value=-nan",
            index::VELOCITY
        );
        assert!(lines.iter().any(|l| l.ends_with(&want)), "{lines:?}");
    }

    #[test]
    fn resolution_runs_mr_after_the_base_and_returns_its_status() {
        let (handle, _) = SyncIOHandle::connect_port(
            "OEM_NONE",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let ctrl = Arc::new(Mutex::new(OemController::new(handle, 2)));
        let port = register_controller_port(
            &open_owner(),
            "oemMr",
            2,
            OemParams::new(ctrl.clone(), 100, 1000),
        )
        .unwrap();
        let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(OemAxis::new(ctrl.clone(), 1)));
        let user = TraceUser::Asyn {
            port: "oemMr".into(),
            addr: 1,
        };
        register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
        let reason = port
            .drv_user_create_blocking(&DrvUserRequest::new("OEM_RESOLUTION", 1))
            .unwrap()
            .reason;
        let mut status = None;
        let lines = traced("oemMr", || {
            status = Some(port.write_int32_blocking(reason, 1, 4000));
        });
        // The base stored the value; MR to the missing port failed after it.
        assert!(status.unwrap().is_err());
        assert_eq!(port.read_int32_blocking(reason, 1).unwrap(), 4000);
        assert_eq!(ctrl.lock().unwrap().pulses_per_unit[1], 4000.0);
        // The base's line, then the closing line with MR's (disconnected) status.
        let mine: Vec<_> = lines.iter().filter(|l| l.contains("writeInt32")).collect();
        assert_eq!(mine.len(), 2, "{lines:?}");
        assert!(
            mine[0].contains("asynMotorController:writeInt32"),
            "{lines:?}"
        );
        assert!(
            mine[1].ends_with(&format!(
                "OEMMotorDriver:writeInt32: error, status=3 function={reason}, value=4000"
            )),
            "{lines:?}"
        );
    }
}
