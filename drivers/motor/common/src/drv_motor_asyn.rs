//! drvMotorAsyn.c's asyn write interfaces on a model-2 port: the int32 and
//! float64 writes a record makes through
//! `@asyn(port,axis)MOTOR_*`, which reach the driver's move, home and stop
//! or its `motorAxisSetInteger`/`motorAxisSetDouble`.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::motor::poll_loop::PollCommand;

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
    pub const DEFER_MOVES: i32 = 26;
    pub const MOVE_TO_HOME: i32 = 27;
}

/// The driver's `motorAxisSetInteger`/`motorAxisSetDouble`, which the port's
/// writes call for every function they do not act on themselves.
pub trait DrvSet: Send {
    fn set_integer(&mut self, user: &AsynUser, function: i32, value: i32) -> AsynResult<()>;
    fn set_double(&mut self, user: &AsynUser, function: i32, value: f64) -> AsynResult<()>;
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
    /// `None` for a driver whose port has not registered its setters.
    pub setter: Option<Arc<Mutex<dyn DrvSet>>>,
    /// The axis' poller, which the driver's own task stands for in C.
    pub poller: mpsc::Sender<PollCommand>,
    max_velocity: f64,
    min_velocity: f64,
    accel: f64,
}

impl Axis {
    pub fn new(
        motor: Arc<Mutex<dyn AsynMotor>>,
        setter: Option<Arc<Mutex<dyn DrvSet>>>,
        poller: mpsc::Sender<PollCommand>,
    ) -> Self {
        Self {
            motor,
            setter,
            poller,
            max_velocity: 0.0,
            min_velocity: 0.0,
            accel: 0.0,
        }
    }

    fn setter(&self, what: &str, command: usize) -> AsynResult<&Arc<Mutex<dyn DrvSet>>> {
        self.setter.as_ref().ok_or_else(|| {
            refuse(format!(
                "drvMotorAsyn::{what} driver has no setter for reason={command}"
            ))
        })
    }

    fn set_integer(&self, user: &AsynUser, command: usize, value: i32) -> AsynResult<()> {
        lock(self.setter("writeInt32", command)?).set_integer(user, command as i32, value)
    }

    /// The driver's own task wakes on a move it was given
    /// (drvMM4000Asyn.c:491, :524, :591); here that task is the poller.
    fn wake(&self) {
        let _ = self.poller.try_send(PollCommand::Wakeup);
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
    /// (drvMotorAsyn.c:313-317).
    pub fn new(port_name: &str, axes: Vec<Option<Axis>>) -> Self {
        let base = PortDriverBase::new_exact_flags(
            port_name,
            axes.len(),
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: false,
            },
        );
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
            // C's forceCallback posts the axis' parameters; the poller posts.
            UPDATE_STATUS => {
                lock(&axis.motor).force_callback(user);
                let _ = axis.poller.try_send(PollCommand::ForcePoll);
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
        status
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
            _ => {
                lock(axis.setter("writeFloat64", command)?).set_double(user, command as i32, value)
            }
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
        status
    }
}
