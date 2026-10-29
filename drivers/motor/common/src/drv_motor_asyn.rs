//! drvMotorAsyn.c's asyn interfaces on a model-2 port: the int32,
//! uint32Digital and float64 reads and writes a record makes through
//! `@asyn(port,axis)MOTOR_*`, which reach the driver's move, home and stop
//! or its `motorAxisGet*`/`motorAxisSet*`, and `intCallback`, which turns
//! the driver's parameter callbacks into the port's interrupts and the motor
//! record's status.

use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::SystemTime;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus};
use epics_rs::asyn::interrupt::{InterruptManager, InterruptValue};
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{
    DrvUserInfo, DrvUserRequest, InterfaceMask, PortDriver, PortDriverBase, PortFlags,
};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::motor::device_support::StatusPoster;
use epics_rs::motor::flags::MstaFlags;
use epics_rs::motor::poll_loop::PollCommand;

use crate::controller_status;
use crate::param_lib::Params;

/// `motorAxisParam_t` (motor_interface.h:301-333): the functions a driver's
/// [`DrvSet`] takes.
pub mod function {
    pub const POSITION: i32 = 0;
    pub const RESOLUTION: i32 = 1;
    pub const ENCODER_RATIO: i32 = 2;
    pub const LOW_LIMIT: i32 = 3;
    pub const HIGH_LIMIT: i32 = 4;
    pub const P_GAIN: i32 = 5;
    pub const I_GAIN: i32 = 6;
    pub const D_GAIN: i32 = 7;
    pub const CLOSED_LOOP: i32 = 8;
    pub const ENCODER_POSN: i32 = 9;
    pub const ACTUAL_VEL: i32 = 10;
    pub const DIRECTION: i32 = 11;
    pub const DONE: i32 = 12;
    pub const HIGH_HARD_LIMIT: i32 = 13;
    pub const HOME_SIGNAL: i32 = 14;
    pub const SLIP: i32 = 15;
    pub const POWER_ON: i32 = 16;
    pub const FOLLOWING_ERROR: i32 = 17;
    pub const HOME_ENCODER: i32 = 18;
    pub const HAS_ENCODER: i32 = 19;
    pub const PROBLEM: i32 = 20;
    pub const MOVING: i32 = 21;
    pub const HAS_CLOSED_LOOP: i32 = 22;
    pub const COMM_ERROR: i32 = 23;
    pub const LOW_HARD_LIMIT: i32 = 24;
    pub const HOMED: i32 = 25;
    pub const DEFER_MOVES: i32 = 26;
    pub const MOVE_TO_HOME: i32 = 27;
}

/// The driver's `motorAxisSetInteger`/`motorAxisSetDouble`, which the port's
/// writes call for every function they do not act on themselves.
pub trait DrvSet: Send {
    fn set_integer(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()>;
    fn set_double(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()>;
}

/// The driver's `motorAxisGetInteger`/`motorAxisGetDouble` and
/// `motorAxisSetCallback`: the axis' [`Params`], which a driver overrides for
/// a value it keeps elsewhere. `None` is a parameter the driver never set,
/// whose read C leaves untouched. It is called from the parameter callback,
/// so it must not take the axis' own lock.
pub trait DrvGet: Send + Sync {
    fn params(&self) -> &Params;

    fn get_integer(&self, function: i32) -> Option<i32> {
        self.params().get_integer(function).ok().flatten()
    }

    fn get_double(&self, function: i32) -> Option<f64> {
        self.params().get_double(function).ok().flatten()
    }
}

/// A driver whose getters are its paramLib store alone.
impl DrvGet for Params {
    fn params(&self) -> &Params {
        self
    }
}

// drvMotorAsyn.c's `motorCommand` beyond the parameters (:101-106).
const VELOCITY: usize = 100;
const VEL_BASE: usize = 101;
const ACCEL: usize = 102;
const MOVE_REL: usize = 103;
const MOVE_ABS: usize = 104;
const MOVE_VEL: usize = 105;
const HOME: usize = 106;
const STOP: usize = 107;
const STATUS: usize = 108;
const UPDATE_STATUS: usize = 109;

/// C `motorCommands[]` (drvMotorAsyn.c:113-152), which `drvUserCreate`
/// searches in order; the status bits are `motorAxisDirection` (11) on.
const COMMANDS: [(&str, usize); 38] = [
    ("MOTOR_MOVE_REL", MOVE_REL),
    ("MOTOR_MOVE_ABS", MOVE_ABS),
    ("MOTOR_MOVE_VEL", MOVE_VEL),
    ("MOTOR_HOME", HOME),
    ("MOTOR_STOP_AXIS", STOP),
    ("MOTOR_VELOCITY", VELOCITY),
    ("MOTOR_VEL_BASE", VEL_BASE),
    ("MOTOR_ACCEL", ACCEL),
    ("MOTOR_POSITION", 0),
    ("MOTOR_ENCODER_POSITION", 9),
    ("MOTOR_DEFER_MOVES", 26),
    ("MOTOR_RESOLUTION", 1),
    ("MOTOR_ENCODER_RATIO", 2),
    ("MOTOR_PGAIN", 5),
    ("MOTOR_IGAIN", 6),
    ("MOTOR_DGAIN", 7),
    ("MOTOR_HIGH_LIMIT", 4),
    ("MOTOR_LOW_LIMIT", 3),
    ("MOTOR_CLOSED_LOOP", 8),
    ("MOTOR_DEFER_MOVES", 26),
    ("MOTOR_MOVE_HOME", 27),
    ("MOTOR_STATUS", STATUS),
    ("MOTOR_UPDATE_STATUS", UPDATE_STATUS),
    ("MOTOR_STATUS_DIRECTION", 11),
    ("MOTOR_STATUS_DONE", 12),
    ("MOTOR_STATUS_HIGH_LIMIT", 13),
    ("MOTOR_STATUS_AT_HOME", 14),
    ("MOTOR_STATUS_SLIP", 15),
    ("MOTOR_STATUS_POWERED", 16),
    ("MOTOR_STATUS_FOLLOWING_ERROR", 17),
    ("MOTOR_STATUS_HOME", 18),
    ("MOTOR_STATUS_HAS_ENCODER", 19),
    ("MOTOR_STATUS_PROBLEM", 20),
    ("MOTOR_STATUS_MOVING", 21),
    ("MOTOR_STATUS_GAIN_SUPPORT", 22),
    ("MOTOR_STATUS_COMMS_ERROR", 23),
    ("MOTOR_STATUS_LOW_LIMIT", 24),
    ("MOTOR_STATUS_HOMED", 25),
];

fn refuse(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

fn lock<T: ?Sized>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// An opened axis: C `drvmotorAxisPvt`.
pub(crate) struct Axis {
    pub motor: Arc<Mutex<dyn AsynMotor>>,
    pub setter: Arc<Mutex<dyn DrvSet>>,
    getter: Arc<dyn DrvGet>,
    /// The axis' poller, which the driver's own task stands for in C.
    pub poller: mpsc::Sender<PollCommand>,
    max_velocity: f64,
    min_velocity: f64,
    accel: f64,
    /// C `pAxis->status`, which `intCallback` keeps.
    status: Arc<Mutex<MotorStatus>>,
    /// The motor record's devMotorAsyn, the genericPointer client.
    poster: StatusPoster,
}

impl Axis {
    pub fn new(
        motor: Arc<Mutex<dyn AsynMotor>>,
        setter: Arc<Mutex<dyn DrvSet>>,
        getter: Arc<dyn DrvGet>,
        poller: mpsc::Sender<PollCommand>,
        poster: StatusPoster,
    ) -> Self {
        Self {
            motor,
            setter,
            getter,
            poller,
            max_velocity: 0.0,
            min_velocity: 0.0,
            accel: 0.0,
            status: Arc::default(),
            poster,
        }
    }

    /// C's `getInteger` into a caller's value it leaves untouched for an
    /// undefined or out-of-range parameter, and readInt32/readFloat64 still
    /// return asynSuccess. That value is 0 here, which is what the PV shows in
    /// C: devAsynInt32's result is calloc'd (devAsynInt32.c:208) and a
    /// successful read stores it with udf 0 (:848-851). initAo reads into an
    /// uninitialised `epicsInt32 value` instead (:934, :954); that C defect
    /// is not copied.
    fn get_integer(&self, command: usize) -> i32 {
        i32::try_from(command)
            .ok()
            .and_then(|f| self.getter.get_integer(f))
            .unwrap_or(0)
    }

    fn set_integer(&self, user: &AsynUser, command: usize, value: i32) -> AsynResult<()> {
        lock(&self.setter).set_integer(user, command as i32, value)
    }

    /// The driver's own task wakes on a move it was given
    /// (drvMM4000Asyn.c:491, :524, :591); here that task is the poller.
    fn wake(&self) {
        let _ = self.poller.try_send(PollCommand::Wakeup);
    }
}

/// The `MotorStatus` bit of status parameter `function`, C's `bit_num`.
fn status_bit(status: &mut MotorStatus, function: i32) -> Option<&mut bool> {
    use function::*;
    Some(match function {
        DIRECTION => &mut status.direction,
        DONE => &mut status.done,
        HIGH_HARD_LIMIT => &mut status.high_limit,
        HOME_SIGNAL => &mut status.home,
        SLIP => &mut status.slip,
        POWER_ON => &mut status.powered,
        FOLLOWING_ERROR => &mut status.slip_stall,
        HOME_ENCODER => &mut status.encoder_home,
        HAS_ENCODER => &mut status.has_encoder,
        PROBLEM => &mut status.problem,
        MOVING => &mut status.msta_moving,
        HAS_CLOSED_LOOP => &mut status.gain_support,
        COMM_ERROR => &mut status.comms_error,
        LOW_HARD_LIMIT => &mut status.low_limit,
        HOMED => &mut status.homed,
        _ => return None,
    })
}

/// C `intCallback` (drvMotorAsyn.c:822-920) for the axis at `addr`: fold
/// the changed status bits, position, encoder position and velocity into
/// `status`, then call back the float64 clients of each changed parameter,
/// the motor record with `status` whatever changed, and the int32 clients
/// of each changed status bit, of `MOTOR_STATUS`, and of every other
/// reason. A value goes only to the clients of its reason and interface,
/// which is C's walk of each interface's list.
fn int_callback(
    interrupts: &InterruptManager,
    getter: &dyn DrvGet,
    addr: i32,
    (status, poster): (&Mutex<MotorStatus>, &StatusPoster),
    changed: &[i32],
) {
    let bits = function::DIRECTION..=function::HOMED;
    let status = {
        let mut status = lock(status);
        for &c in changed {
            if let Some(bit) = status_bit(&mut status, c) {
                *bit = getter.get_integer(c).unwrap_or(0) != 0;
            }
            let set: fn(&mut MotorStatus, f64) = match c {
                function::POSITION => MotorStatus::set_position,
                function::ENCODER_POSN => MotorStatus::set_encoder_position,
                function::ACTUAL_VEL => MotorStatus::set_velocity,
                _ => continue,
            };
            // C's getDouble leaves the value of an undefined one.
            if let Some(v) = getter.get_double(c) {
                set(&mut status, v);
            }
        }
        status.moving = !status.done;
        status.clone()
    };
    let fire = |iface, reason: usize, value| {
        interrupts.notify(InterruptValue {
            reason,
            addr,
            value,
            timestamp: SystemTime::now(),
            iface: Some(iface),
            ..InterruptValue::default()
        })
    };
    for &c in changed {
        let value = getter.get_double(c).unwrap_or(0.0);
        fire(
            InterfaceType::Float64,
            c as usize,
            ParamValue::Float64(value),
        );
    }
    let word = MstaFlags::from_status(&status).bits() as i32;
    poster.post(status);
    for &c in changed.iter().filter(|c| bits.contains(c)) {
        let value = getter.get_integer(c).unwrap_or(0);
        fire(InterfaceType::Int32, c as usize, ParamValue::Int32(value));
    }
    fire(InterfaceType::Int32, STATUS, ParamValue::Int32(word));
    // Every other int32 client, changed or not. C sends its uninitialised
    // `ivalue` for a reason past the store (:905-909); here 0.
    for (reason, _) in interrupts
        .subscribed_bindings()
        .into_iter()
        .filter(|&(r, a)| a == addr && r != STATUS)
    {
        let Ok(function) = i32::try_from(reason) else {
            continue;
        };
        if !bits.contains(&function) {
            let value = getter.get_integer(function).unwrap_or(0);
            fire(InterfaceType::Int32, reason, ParamValue::Int32(value));
        }
    }
}

/// The `drvAsynMotorConfigure` port: C `drvmotorPvt`.
pub(crate) struct DrvMotorAsyn {
    base: PortDriverBase,
    /// By address; `None` for an axis the driver did not open.
    axes: Vec<Option<Axis>>,
}

impl DrvMotorAsyn {
    /// C registers the port `ASYN_MULTIDEVICE | ASYN_CANBLOCK`
    /// (drvMotorAsyn.c:313-317), then gives each opened axis `intCallback`
    /// (:411).
    pub fn new(port_name: &str, axes: Vec<Option<Axis>>) -> Self {
        let base = PortDriverBase::new_exact_flags(
            port_name,
            axes.len(),
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: false,
            },
            InterfaceMask::COMMON
                | InterfaceMask::INT32
                | InterfaceMask::UINT32_DIGITAL
                | InterfaceMask::FLOAT64
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::GENERIC_POINTER
                | InterfaceMask::DRV_USER,
            InterfaceMask::INT32
                | InterfaceMask::UINT32_DIGITAL
                | InterfaceMask::FLOAT64
                | InterfaceMask::GENERIC_POINTER,
        );
        for (addr, axis) in axes.iter().enumerate() {
            let Some(Axis {
                getter,
                status,
                poster,
                ..
            }) = axis
            else {
                continue;
            };
            let interrupts = InterruptManager::from_shared_state(base.interrupts.shared_state());
            // The store holds its callbacks, so the callback holds the getter
            // weakly.
            let weak: Weak<dyn DrvGet> = Arc::downgrade(getter);
            let (status, poster) = (status.clone(), poster.clone());
            let addr = addr as i32;
            getter
                .params()
                .add_callback(Arc::new(move |changed: &[i32]| {
                    if let Some(getter) = weak.upgrade() {
                        let record = (&*status, &poster);
                        int_callback(&interrupts, &*getter, addr, record, changed);
                    }
                }));
        }
        Self { base, axes }
    }

    /// C's checks on `getAddr`'s channel; a negative one C indexes
    /// `axisData` with unchecked (drvMotorAsyn.c:592, :690), here invalid.
    fn axis(&self, user: &AsynUser, what: &str) -> AsynResult<&Axis> {
        let slot = usize::try_from(user.addr)
            .ok()
            .and_then(|a| self.axes.get(a))
            .ok_or_else(|| refuse(format!("drvMotorAsyn::{what} Invalid axis {}", user.addr)))?;
        slot.as_ref().ok_or_else(|| {
            refuse(format!(
                "drvMotorAsyn::{what} Uninitialised axis {}",
                user.addr
            ))
        })
    }

    fn axis_mut(&mut self, user: &AsynUser, what: &str) -> AsynResult<&mut Axis> {
        self.axis(user, what)?;
        Ok(self.axes[user.addr as usize].as_mut().expect("checked"))
    }

    fn flow(&self, addr: i32, args: std::fmt::Arguments<'_>) {
        PortManager::global().trace_manager().print(
            Some(&self.base.port_name),
            Some(addr),
            0,
            TraceMask::FLOW,
            args,
        );
    }
}

impl PortDriver for DrvMotorAsyn {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// C `drvUserCreate` (drvMotorAsyn.c:950-984): the names,
    /// case-insensitive. Device support passes no type or size pointers.
    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        self.flow(
            req.addr,
            format_args!(
                "drvMotorAsyn::drvUserCreate, drvInfo={}, pptypeName=(nil), psize=(nil), pasynUser={:p}\n",
                req.drv_info, req
            ),
        );
        let Some(&(name, reason)) = COMMANDS
            .iter()
            .find(|(name, _)| name.eq_ignore_ascii_case(&req.drv_info))
        else {
            return Err(refuse(format!(
                "drvmotorAsyn::drvUserCreate, unknown command={}",
                req.drv_info
            )));
        };
        self.flow(
            req.addr,
            format_args!("drvMotorAsyn::drvUserCreate, command={name}\n"),
        );
        Ok(DrvUserInfo::from_reason(reason))
    }

    /// C `readInt32` (drvMotorAsyn.c:439-482).
    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        let command = user.reason;
        let axis = self.axis(user, "readInt32")?;
        let value = match command {
            STATUS => MstaFlags::from_status(&lock(&axis.status)).bits() as i32,
            _ => axis.get_integer(command),
        };
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("drvMotorAsyn::readInt32, reason={command}, value={value}\n"),
        );
        Ok(value)
    }

    /// C `readUInt32Digital` (drvMotorAsyn.c:484-522), whose errors name
    /// readInt32.
    fn read_uint32_digital(&mut self, user: &AsynUser, mask: u32) -> AsynResult<u32> {
        let command = user.reason;
        let axis = self.axis(user, "readInt32")?;
        let value = axis.get_integer(command) as u32 & mask;
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!(
                "drvMotorAsyn::readUInt32Digital, reason={command}, value={}\n",
                value as i32
            ),
        );
        Ok(value)
    }

    /// C `readFloat64` (drvMotorAsyn.c:524-574).
    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let command = user.reason;
        let axis = self.axis(user, "readFloat64")?;
        let value = match command {
            VELOCITY => axis.max_velocity,
            VEL_BASE => axis.min_velocity,
            ACCEL => axis.accel,
            // 0 for an undefined parameter, as at `Axis::get_integer`.
            _ => i32::try_from(command)
                .ok()
                .and_then(|f| axis.getter.get_double(f))
                .unwrap_or(0.0),
        };
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!(
                "drvMotorAsyn::readFloat64, reason={command}, value={}\n",
                CFloat(value)
            ),
        );
        Ok(value)
    }

    /// C `getBounds` (drvMotorAsyn.c:566-574).
    fn get_bounds_int32(&self, user: &AsynUser) -> AsynResult<(i32, i32)> {
        let (low, high) = (0, 65535);
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("drvMotorAsyn::getBounds,low={low}, high={high}\n"),
        );
        Ok((low, high))
    }

    /// C `writeUInt32Digital` (drvMotorAsyn.c:631-672): the masked bits into
    /// the driver's value, whose errors name writeInt32.
    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        let command = user.reason;
        let axis = self.axis(user, "writeInt32")?;
        let old = axis.get_integer(command) as u32;
        let temp = (old & !mask) | (value & mask);
        let status = axis.set_integer(user, command, temp as i32);
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!(
                "drvMotorAsyn::writeUInt32Digital, reason={command}, value={}\n",
                temp as i32
            ),
        );
        // A drvset call returns a bare status: C passes it no asynUser to
        // put a message on.
        status.map_err(controller_status)
    }

    /// C `writeInt32` (drvMotorAsyn.c:576-629).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let command = user.reason;
        let axis = self.axis(user, "writeInt32")?;
        let status = match command {
            STOP => lock(&axis.motor).stop(user, axis.accel),
            HOME => {
                let r = lock(&axis.motor).home(
                    user,
                    axis.min_velocity,
                    axis.max_velocity,
                    axis.accel,
                    value != 0,
                );
                axis.wake();
                r
            }
            // C's forceCallback makes the driver's next callCallback call
            // back unchanged parameters too; the poller's poll is that one.
            UPDATE_STATUS => {
                lock(&axis.motor).force_callback(user);
                let _ = axis.poller.try_send(PollCommand::ForcePoll(None));
                Ok(())
            }
            _ => axis.set_integer(user, command, value),
        };
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("drvMotorAsyn::writeInt32, reason={command}, value={value}\n"),
        );
        status.map_err(controller_status)
    }

    /// C `writeFloat64` (drvMotorAsyn.c:674-752).
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let command = user.reason;
        let axis = self.axis_mut(user, "writeFloat64")?;
        user.print(
            TraceMask::FLOW,
            file!(),
            line!(),
            format_args!(
                "drvMotorAsyn::writeFloat64, reason={command}, pasynUser={:p} pAxis={:p}\n",
                &*user, &*axis
            ),
        );
        let (min, max, accel) = (axis.min_velocity, axis.max_velocity, axis.accel);
        let status = match command {
            MOVE_REL | MOVE_ABS | MOVE_VEL | HOME => {
                let mut motor = lock(&axis.motor);
                let r = match command {
                    MOVE_REL => motor.move_relative(user, value, min, max, accel),
                    MOVE_ABS => motor.move_absolute(user, value, min, max, accel),
                    MOVE_VEL => motor.move_velocity(user, min, value, accel),
                    _ => motor.home(user, min, max, accel, value != 0.0),
                };
                drop(motor);
                axis.wake();
                r
            }
            VELOCITY => {
                axis.max_velocity = value;
                Ok(())
            }
            VEL_BASE => {
                axis.min_velocity = value;
                Ok(())
            }
            ACCEL => {
                axis.accel = value;
                Ok(())
            }
            _ => lock(&axis.setter).set_double(user, command as i32, value),
        };
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!(
                "drvMotorAsyn::writeFloat64, reason={command}, value={}\n",
                CFloat(value)
            ),
        );
        status.map_err(controller_status)
    }
}
