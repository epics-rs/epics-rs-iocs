//! New Focus 8742 / 8743-CL picomotor controllers (`874xMotorDriver.cpp`, a
//! model-3 `asynMotorController`).
//!
//! Axes are numbered from 1, as the controller numbers them; address 0 of
//! the port is unused. Axes past the first controller of a daisy chain are
//! addressed `c>n`, and their replies carry the same `c>` prefix.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{
    ControllerParams, FinishedWrite, ParamWrite, WriteHook, WriteOutcome, WriteValue, index,
};

use motor_common::util::{atof, atoi};

/// `DEFAULT_CONTROLLER_TIMEOUT` of asynMotorController.
pub const NF874X_TIMEOUT: Duration = Duration::from_secs(2);
/// `MAX_CONTROLLER_STRING_SIZE`.
const MAX_REPLY: usize = 256;

fn nf_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// One 8742 or 8743-CL, or a daisy chain of them.
pub struct Nf874xController {
    /// `None` when the connect failed, which C reports and carries on from.
    handle: Option<SyncIOHandle>,
    /// `hasClosedLoopSupport_`: the master answered `VE?` as an 8743-CL.
    closed_loop: bool,
    port: Option<PortHandle>,
    params: Option<Nf874xParams>,
}

impl Nf874xController {
    /// The constructor's `VE?`: an 8743-CL has closed-loop support.
    pub fn new(handle: Option<SyncIOHandle>) -> Self {
        let mut controller = Self {
            handle,
            closed_loop: false,
            port: None,
            params: None,
        };
        let reply = controller.write_read("VE?").unwrap_or_default();
        // C copies at most 9 characters and dereferences the space it
        // expects in them, crashing on a reply without one.
        let model: String = reply.chars().take(9).take_while(|&c| c != ' ').collect();
        controller.closed_loop = model == "8743-CL";
        controller
    }

    pub fn attach_port(&mut self, port: PortHandle) {
        self.port = Some(port);
    }

    /// `axesPerController`.
    fn axes_per_controller(&self) -> i32 {
        if self.closed_loop { 2 } else { 4 }
    }

    /// `axisName_`: `n` on the master, `c>n` down the chain.
    fn axis_name(&self, axis: i32) -> String {
        let per = self.axes_per_controller();
        if axis > per {
            format!("{}>{}", (axis - 1) / per + 1, (axis - 1) % per + 1)
        } else {
            axis.to_string()
        }
    }

    fn handle(&self) -> AsynResult<&SyncIOHandle> {
        self.handle
            .as_ref()
            .ok_or_else(|| nf_err("not connected to the nf874x controller"))
    }

    /// `writeController`.
    fn write(&self, command: &str) -> AsynResult<()> {
        self.handle()?.write_octet(0, command.as_bytes()).map(drop)
    }

    /// `writeReadController`.
    fn write_read(&self, command: &str) -> AsynResult<String> {
        let reply = self
            .handle()?
            .write_read(0, command.as_bytes(), MAX_REPLY)?;
        Ok(String::from_utf8_lossy(&reply).into_owned())
    }

    /// `PH?` of the controller `axis` is on, and the shift of its three
    /// switch bits. C sends a bare `PH?` and shifts by the chain-wide axis
    /// number, so an axis past the master reads bits its controller does
    /// not have; this asks the axis's own controller for its local bits.
    fn hardware_status(&self, axis: i32, name: &str) -> AsynResult<(i32, i32)> {
        let prefix = name.rfind('>').map_or("", |i| &name[..=i]);
        let reply = self.write_read(&format!("{prefix}PH?"))?;
        let local = (axis - 1) % self.axes_per_controller();
        Ok((atoi(skip(&reply, name)), 3 * local))
    }
}

/// The reply past the `c>` prefix an axis name `c>n` puts on it (C's
/// `inString_ + strlen(axisName_) - 1`).
fn skip<'a>(reply: &'a str, name: &str) -> &'a str {
    reply.get(name.len() - 1..).unwrap_or("")
}

fn lock(controller: &Mutex<Nf874xController>) -> MutexGuard<'_, Nf874xController> {
    controller.lock().unwrap_or_else(|e| e.into_inner())
}

/// The reasons of the parameters `nf874xController` creates.
#[derive(Clone, Copy)]
struct Nf874xParams {
    update_interval: usize,
    deadband: usize,
    following_error: usize,
}

/// One picomotor axis, numbered from 1.
pub struct Nf874xAxis {
    controller: Arc<Mutex<Nf874xController>>,
    axis: i32,
    name: String,
    /// `lastDirection_`.
    last_direction: bool,
    /// `encoderPosition_`.
    encoder_position: f64,
    /// `limitChecking_`: off when both limit inputs read active, which is a
    /// stage with no limit switches (8410 rotary).
    limit_checking: bool,
    status: MotorStatus,
}

impl Nf874xAxis {
    /// The axis constructor: on an 8743-CL, read whether the stage has
    /// limit switches and close the loop.
    pub fn new(controller: Arc<Mutex<Nf874xController>>, axis: i32) -> Self {
        let (name, closed_loop) = {
            let ctrl = lock(&controller);
            (ctrl.axis_name(axis), ctrl.closed_loop)
        };
        let mut this = Self {
            controller,
            axis,
            name,
            last_direction: false,
            encoder_position: 0.0,
            limit_checking: false,
            status: MotorStatus::default(),
        };
        if closed_loop {
            let (bits, shift) = lock(&this.controller)
                .hardware_status(axis, &this.name)
                .unwrap_or((0, 0));
            this.limit_checking = (bits >> shift) & 0x3 != 0x3;
            this.status.gain_support = true;
            this.status.has_encoder = true;
            let _ = this.closed_loop(true);
        }
        this
    }

    fn send(&self, commands: &[String]) -> AsynResult<()> {
        let ctrl = lock(&self.controller);
        // C keeps the status of the last write only.
        let mut result = Ok(());
        for command in commands {
            result = ctrl.write(command);
        }
        result
    }

    /// `AC` and `VA`, which every motion sends first.
    fn speeds(&self, velocity: f64, acceleration: f64) -> [String; 2] {
        [
            format!("{} AC {acceleration:.6}", self.name),
            format!("{} VA {velocity:.6}", self.name),
        ]
    }

    /// `setClosedLoop`.
    fn closed_loop(&mut self, enable: bool) -> AsynResult<()> {
        let n = &self.name;
        let command = if enable {
            format!(
                "{n}SN1;{n}ZH{};{n}MM1",
                if self.limit_checking { 0 } else { 1 }
            )
        } else {
            format!("{n}SN0;{n}ZH3;{n}MM0")
        };
        let result = self.send(&[command]);
        self.status.powered = enable;
        result
    }

    /// Post the soft limits an 8743-CL poll read, as C's `setDoubleParam`.
    fn post(&self, updates: Vec<ParamSetValue>) {
        if updates.is_empty() {
            return;
        }
        let port = lock(&self.controller).port.clone();
        if let Some(port) = port {
            let _ = port.set_params_and_notify_blocking(self.axis, updates);
        }
    }

    /// The reads of `poll` up to the first that fails; the soft limits read
    /// go to `limits`.
    fn read_status(&mut self, limits: &mut Vec<ParamSetValue>) -> AsynResult<()> {
        let ctrl = lock(&self.controller);
        let name = self.name.clone();
        let ask = |q: &str| ctrl.write_read(&format!("{name} {q}"));

        self.encoder_position = atof(skip(&ask("TP?")?, &name));
        let status = &mut self.status;
        status.encoder_position = self.encoder_position;
        status.position = self.encoder_position;
        status.direction = self.last_direction;

        status.done = atoi(skip(&ask("MD?")?, &name)) != 0;
        status.moving = !status.done;
        status.msta_moving = status.moving;
        status.encoder_home = self.encoder_position == 0.0;

        if !ctrl.closed_loop {
            return Ok(());
        }
        status.powered = atoi(skip(&ask("MM?")?, &name)) != 0;
        for (reason, query) in [(index::HIGH_LIMIT, "SR?"), (index::LOW_LIMIT, "SL?")] {
            let value = ParamValue::Float64(atof(skip(&ask(query)?, &name)));
            limits.push(ParamSetValue::new(reason, self.axis, value));
        }
        let (bits, shift) = ctrl.hardware_status(self.axis, &name)?;
        let bit = |i: i32| bits & (1 << (shift + i)) != 0;
        status.high_limit = self.limit_checking && bit(0);
        status.low_limit = self.limit_checking && bit(1);
        status.home = bit(2);
        Ok(())
    }
}

impl AsynMotor for Nf874xAxis {
    /// C `poll`'s `skip` label sets only `motorStatusProblem_`.
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
        let [ac, va] = self.speeds(velocity, acceleration);
        let result = self.send(&[ac, va, format!("{} PA {position:.6}", self.name)]);
        self.last_direction = position > self.encoder_position;
        result
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let [ac, va] = self.speeds(velocity, acceleration);
        let result = self.send(&[ac, va, format!("{} PR {distance:.6}", self.name)]);
        self.last_direction = distance > 0.0;
        result
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let forward = velocity >= 0.0;
        let [ac, va] = self.speeds(velocity.abs(), acceleration);
        let way = if forward { "+" } else { "-" };
        let result = self.send(&[ac, va, format!("{} MV {way}", self.name)]);
        self.last_direction = forward;
        result
    }

    /// To the limit switch (`MT`), or the index (`MZ`) on a stage without
    /// limit switches.
    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let [ac, va] = self.speeds(velocity, acceleration);
        let mnemonic = if self.limit_checking { "MT" } else { "MZ" };
        let way = if forward { "+" } else { "-" };
        let result = self.send(&[ac, va, format!("{} {mnemonic} {way}", self.name)]);
        self.last_direction = forward;
        result
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.send(&[format!("{} ST", self.name)])
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(&[format!("{} DH {position:.6}", self.name)])
    }

    /// The same `DH` as [`Self::set_position`], as C's `setEncoderPosition`.
    fn set_encoder_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(&[format!("{} DH {position:.6}", self.name)])
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.closed_loop(enable)
    }

    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(&[format!("{} SR {position:.6}", self.name)])
    }

    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(&[format!("{} SL {position:.6}", self.name)])
    }

    /// `doMoveToHome`: to absolute 0, the position `DH` defined.
    fn move_to_home(
        &mut self,
        _user: &AsynUser,
        _position: f64,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        let result = self.send(&[format!("{} PA 0", self.name)]);
        self.last_direction = self.encoder_position < 0.0;
        result
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let mut limits = Vec::new();
        self.status.problem = self.read_status(&mut limits).is_err();
        self.post(limits);
        Ok(self.status.clone())
    }
}

/// `nf874xController`'s parameters and its `writeInt32` / `writeFloat64`.
pub struct Nf874xControllerParams {
    controller: Arc<Mutex<Nf874xController>>,
}

impl Nf874xControllerParams {
    pub fn new(controller: Arc<Mutex<Nf874xController>>) -> Self {
        Self { controller }
    }

    /// Send `mnemonic value` to the axis at `user`. C dereferences the null
    /// axis at address 0; this refuses it.
    fn send(&self, user: &AsynUser, mnemonic: &str, value: String) -> AsynResult<()> {
        if user.addr < 1 {
            return Err(nf_err("nf874x has no axis 0"));
        }
        let ctrl = lock(&self.controller);
        ctrl.write(&format!("{} {mnemonic} {value}", ctrl.axis_name(user.addr)))
    }
}

/// `writeInt32` / `writeFloat64`'s closing line, which the record's writes
/// reach as well as the port's.
struct Nf874xWriteHook;

impl WriteHook for Nf874xWriteHook {
    /// C traces every write at its end, the base's included.
    fn after_write(&self, write: &FinishedWrite<'_>) -> WriteOutcome {
        let function = write.function;
        let (method, value) = match write.value {
            WriteValue::Int32(v) => ("writeInt32", v.to_string()),
            WriteValue::Float64(v) => ("writeFloat64", format!("{v:.6}")),
        };
        match write.status {
            Err(e) => write.print(
                TraceMask::ERROR,
                format_args!(
                    "nf874xMotorDriver:{method}: error, status={} function={function}, value={value}\n",
                    e.c_status_code()
                ),
            ),
            Ok(()) => write.print(
                TraceMask::IO_DRIVER,
                format_args!("nf874xMotorDriver:{method}: function={function}, value={value}\n"),
            ),
        }
        WriteOutcome::default()
    }
}

impl ControllerParams for Nf874xControllerParams {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let params = Nf874xParams {
            update_interval: base.create_param("MOTOR_UPDATE_INTERVAL", ParamType::Float64)?,
            deadband: base.create_param("MOTOR_DEADBAND", ParamType::Int32)?,
            following_error: base.create_param("MOTOR_FOLLOWING_ERROR", ParamType::Int32)?,
        };
        lock(&self.controller).params = Some(params);
        Ok(())
    }

    fn write_hook(&self) -> Option<Arc<dyn WriteHook>> {
        Some(Arc::new(Nf874xWriteHook))
    }

    /// C `writeInt32`: the parameter is set and posted for every write,
    /// and the driver's own registers never reach the base.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> ParamWrite {
        let Some(p) = lock(&self.controller).params else {
            return ParamWrite::PassToBase(Ok(()));
        };
        let mnemonic = match user.reason {
            r if r == p.deadband => "DB",
            r if r == p.following_error => "FE",
            _ => return ParamWrite::PassToBase(Ok(())),
        };
        let _ = base.set_int32_param(user.reason, user.addr, value);
        let status = self.send(user, mnemonic, value.to_string());
        let _ = base.call_param_callbacks(user.addr);
        ParamWrite::Handled(status)
    }

    /// C `writeFloat64`; see [`Self::write_int32`].
    fn write_float64(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: f64,
    ) -> ParamWrite {
        let Some(p) = lock(&self.controller).params else {
            return ParamWrite::PassToBase(Ok(()));
        };
        if user.reason != p.update_interval {
            return ParamWrite::PassToBase(Ok(()));
        }
        let _ = base.set_float64_param(user.reason, user.addr, value);
        let status = self.send(user, "CL", format!("{value:.6}"));
        let _ = base.call_param_callbacks(user.addr);
        ParamWrite::Handled(status)
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::manager::PortManager;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::motor::builder::MotorBuilder;
    use epics_rs::motor::devices::{register_axis, register_controller_port};
    use epics_rs::motor::trace::TraceUser;

    use super::*;

    fn controller(closed_loop: bool) -> Nf874xController {
        Nf874xController {
            handle: None,
            closed_loop,
            port: None,
            params: None,
        }
    }

    #[test]
    fn chained_axes_take_a_controller_prefix() {
        let open = controller(false);
        assert_eq!(open.axis_name(4), "4");
        assert_eq!(open.axis_name(5), "2>1");
        let closed = controller(true);
        assert_eq!(closed.axis_name(2), "2");
        assert_eq!(closed.axis_name(3), "2>1");
        assert_eq!(closed.axis_name(6), "3>2");
    }

    #[test]
    fn reply_skips_the_chain_prefix() {
        assert_eq!(skip("2>-150", "2>1"), "-150");
        assert_eq!(skip("-150", "1"), "-150");
    }

    /// A port `name` with no line and axis 1.
    fn port(name: &str) -> (PortHandle, Arc<Mutex<Nf874xController>>) {
        let ctrl = Arc::new(Mutex::new(controller(false)));
        let params = Nf874xControllerParams::new(ctrl.clone());
        let port = register_controller_port(&open_owner(), name, 2, params).unwrap();
        let motor: Arc<Mutex<dyn AsynMotor>> =
            Arc::new(Mutex::new(Nf874xAxis::new(ctrl.clone(), 1)));
        let user = TraceUser::Asyn {
            port: name.into(),
            addr: 1,
        };
        register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
        (port, ctrl)
    }

    /// `body`'s lines in `port`'s trace at C's error and IO_DRIVER masks.
    fn traced(port: &str, body: impl FnOnce()) -> Vec<String> {
        let path = std::env::temp_dir().join(format!("nf874x-trace-{port}-{}", std::process::id()));
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
    fn md_sets_the_msta_moving_bit() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let done = Arc::new(AtomicBool::new(false));
        let md = done.clone();
        let (_rig, io) =
            motor_common::testing::LineRig::new("nf874xMsta", move |sent| match sent {
                "1 MD?" => if md.load(Ordering::Relaxed) { "1" } else { "0" }.into(),
                _ => "0".into(),
            });
        let ctrl = Arc::new(Mutex::new(Nf874xController::new(Some(io))));
        let mut axis = Nf874xAxis::new(ctrl, 1);
        let user = AsynUser::default();
        assert!(axis.poll(&user).unwrap().msta_moving);
        done.store(true, Ordering::Relaxed);
        assert!(!axis.poll(&user).unwrap().msta_moving);
    }

    #[test]
    fn every_write_ends_on_cs_line_the_base_included() {
        let (port, _) = port("nf874xTrace");
        let lines = traced("nf874xTrace", || {
            port.write_float64_blocking(index::VELOCITY, 1, 2.5)
                .unwrap();
            port.write_int32_blocking(index::REC_DIRECTION, 1, 1)
                .unwrap();
        });
        let mine: Vec<_> = lines
            .iter()
            .filter(|l| l.contains("nf874xMotorDriver"))
            .collect();
        let velocity = index::VELOCITY;
        let direction = index::REC_DIRECTION;
        let want = [
            format!("nf874xMotorDriver:writeFloat64: function={velocity}, value=2.500000"),
            format!("nf874xMotorDriver:writeInt32: function={direction}, value=1"),
        ];
        assert_eq!(mine.len(), want.len(), "{lines:?}");
        for (line, want) in mine.iter().zip(&want) {
            assert!(line.ends_with(want.as_str()), "{line:?} vs {want:?}");
        }
    }

    #[test]
    fn the_drivers_own_write_never_runs_the_base() {
        let (port, ctrl) = port("nf874xOwn");
        let deadband = lock(&ctrl).params.unwrap().deadband;
        let lines = traced("nf874xOwn", || {
            // No line: the send fails, as C's would.
            assert!(port.write_int32_blocking(deadband, 1, 4).is_err());
        });
        assert!(
            !lines.iter().any(|l| l.contains("asynMotorController")),
            "{lines:?}"
        );
        assert!(
            lines.iter().any(|l| l.ends_with(&format!(
                "nf874xMotorDriver:writeInt32: error, status=3 function={deadband}, value=4"
            ))),
            "{lines:?}"
        );
        assert_eq!(port.read_int32_blocking(deadband, 1).unwrap(), 4);
    }
}
