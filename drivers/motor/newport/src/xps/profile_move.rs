//! The profile-move half of C `XPSController` (XPSController.cpp:167-172,
//! 502-1300): the `XPS_PROFILE_*` parameters, and `buildProfile`,
//! `executeProfile` with its `XPSProfile` thread's `runProfile`,
//! `abortProfile` and `readbackProfile` behind the motor-rs profile hooks.

use std::collections::BTreeMap;
use std::marker::PhantomData;
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::PortSets;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::runtime::task::{StackSizeClass, ThreadPriority, spawn_dedicated_thread};
use epics_rs::motor::controller_port::{
    ControllerCtx, ControllerParams, ControllerPort, PROFILE_BUILD_BUSY, PROFILE_BUILD_DONE,
    PROFILE_EXECUTE_DONE, PROFILE_EXECUTE_EXECUTING, PROFILE_EXECUTE_FLYBACK,
    PROFILE_EXECUTE_MOVE_START, PROFILE_MOVE_MODE_ABSOLUTE, PROFILE_READBACK_BUSY,
    PROFILE_READBACK_DONE, PROFILE_STATUS_ABORT, PROFILE_STATUS_FAILURE, PROFILE_STATUS_SUCCESS,
    PROFILE_STATUS_UNDEFINED, index,
};
use epics_rs::motor::trace::MotorTrace;

use super::controller::{RegisteredAxis, XpsController, verify_message};
use super::gathering::{GatheringReadback, parse_gathering_buffer};
use super::profile::{self, Profile, ProfileAxis};
use super::rpc::{XpsError, XpsSocket};
use super::{ftp, sftp};

/// XPSController.h `MAX_FILENAME_LEN` and `MAX_GROUPNAME_LEN`.
pub(crate) const MAX_FILENAME_LEN: usize = 256;
const MAX_GROUPNAME_LEN: usize = 64;

/// The trajectory folders by controller (XPSController.h:19-21).
const XPS_C_TRAJECTORY_DIRECTORY: &str = "/Admin/Public/Trajectories";
const XPS_Q_TRAJECTORY_DIRECTORY: &str = "/Public/Trajectories";
const XPS_D_TRAJECTORY_DIRECTORY: &str = "/Admin/Public/Trajectories";

/// `buildProfile`'s firmware switch (XPSController.cpp:696-707): the
/// trajectory folder, and whether the upload is SFTP (C `useSFTP_`).
pub(crate) fn trajectory_upload(firmware: &str) -> Result<(&'static str, bool), String> {
    if firmware.contains("XPS-C") {
        Ok((XPS_C_TRAJECTORY_DIRECTORY, false))
    } else if firmware.contains("XPS-Q") {
        Ok((XPS_Q_TRAJECTORY_DIRECTORY, false))
    } else if firmware.contains("XPS-D") {
        Ok((XPS_D_TRAJECTORY_DIRECTORY, true))
    } else {
        Err(format!(
            "Firmware version does not contain XPS-C, XPS-Q, or XPS-D={firmware}\n"
        ))
    }
}

/// C `strerror(errno)`: the OS text without Rust's ` (os error N)`.
pub(crate) fn strerror(e: &std::io::Error) -> String {
    let text = e.to_string();
    match e.raw_os_error() {
        Some(code) => text
            .strip_suffix(&format!(" (os error {code})"))
            .unwrap_or(&text)
            .to_string(),
        None => text,
    }
}

/// The parameters C creates at XPSController.cpp:167-172, in its order.
#[derive(Clone, Copy)]
struct Params {
    max_velocity: usize,
    max_acceleration: usize,
    min_position: usize,
    max_position: usize,
    group_name: usize,
    trajectory_file: usize,
}

/// The controller's profile state: its parameters, `XPSCreateProfile`'s FTP
/// account, each axis's `profilePreDistance_`/`profilePostDistance_` and
/// C's `profileExecuteEvent_`.
pub struct XpsProfile {
    port_name: String,
    trace: MotorTrace,
    params: Option<Params>,
    ftp_login: Option<(String, String)>,
    /// By axis address, from the last build that reached the file.
    distances: BTreeMap<i32, (f64, f64)>,
    execute: SyncSender<()>,
}

/// C's `XPSProfile` thread, before it is started.
pub struct ProfileThread {
    port_name: String,
    execute: Receiver<()>,
    /// The group of the trajectory running, from EXECUTING to FLYBACK: the
    /// runner sets it on the port thread with PROFILE_EXECUTE_STATE, so the
    /// poll reads that state without the parameter library.
    executing: Arc<Mutex<Option<String>>>,
}

/// A controller driver that owns an [`XpsProfile`].
pub trait ProfileHost: ControllerParams + Send + 'static {
    fn xps_profile(&mut self) -> &mut XpsProfile;
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// C `asynError`.
fn xps_error() -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: String::new(),
    }
}

/// C `getStringParam(index, size, value)`: at most `size - 1` bytes.
fn string_param(base: &PortDriverBase, index: usize, size: usize) -> String {
    let bytes = base.get_string_param(index, 0).unwrap_or_default();
    let bytes = &bytes[..bytes.len().min(size - 1)];
    String::from_utf8_lossy(bytes).into_owned()
}

/// C `getIntegerParam(index, &value)` at address 0; C leaves an unset value
/// 0.
fn int_param(base: &PortDriverBase, index: usize) -> i32 {
    base.get_int32_param(index, 0).unwrap_or(0)
}

/// C `getIntegerParam(j, profileUseAxis_, &useAxis[j])`.
fn use_axis(base: &PortDriverBase, addr: i32) -> bool {
    base.get_int32_param(index::PROFILE_USE_AXIS, addr)
        .unwrap_or(0)
        != 0
}

/// The first `n` profile positions of the axis at `addr`.
fn positions(ctx: &ControllerCtx<'_>, addr: i32, n: usize) -> Vec<f64> {
    ctx.profile
        .axes
        .get(addr as usize)
        .map_or_else(|| vec![0.0; n], |a| a.positions[..n].to_vec())
}

/// Where `buildProfile` stops: C's flags, `status` and `message` at `done:`.
struct BuildDone {
    build_ok: bool,
    verify_ok: bool,
    status: i32,
    message: String,
}

/// What `runProfile` ends with: C's `executeOK`, `aborted` and `message`.
struct RunDone {
    ok: bool,
    aborted: bool,
    message: String,
}

impl RunDone {
    fn failed(&mut self, message: String) {
        self.ok = false;
        self.message = message;
    }
}

/// What `readbackProfile` reaches `done:` with.
struct ReadbackDone {
    ok: bool,
    status: i32,
    message: String,
    samples: GatheringReadback,
}

impl XpsProfile {
    pub fn new(port_name: &str) -> (Self, ProfileThread) {
        let (execute, signalled) = sync_channel(1);
        let profile = Self {
            port_name: port_name.to_string(),
            trace: MotorTrace::new(port_name, 0),
            params: None,
            ftp_login: None,
            distances: BTreeMap::new(),
            execute,
        };
        let thread = ProfileThread {
            port_name: port_name.to_string(),
            execute: signalled,
            executing: Arc::new(Mutex::new(None)),
        };
        (profile, thread)
    }

    fn params(&self) -> Params {
        self.params
            .expect("create_params runs when the port is registered")
    }

    /// XPSController.cpp:167-172.
    pub fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        self.params = Some(Params {
            max_velocity: base.create_param("XPS_PROFILE_MAX_VELOCITY", ParamType::Float64)?,
            max_acceleration: base
                .create_param("XPS_PROFILE_MAX_ACCELERATION", ParamType::Float64)?,
            min_position: base.create_param("XPS_PROFILE_MIN_POSITION", ParamType::Float64)?,
            max_position: base.create_param("XPS_PROFILE_MAX_POSITION", ParamType::Float64)?,
            group_name: base.create_param("XPS_PROFILE_GROUP_NAME", ParamType::Octet)?,
            trajectory_file: base.create_param("XPS_TRAJECTORY_FILE", ParamType::Octet)?,
        });
        Ok(())
    }

    /// `XPSCreateProfile`'s FTP account; `None` before it, which C prints
    /// as "(null)".
    pub fn ftp_login(&self) -> Option<&(String, String)> {
        self.ftp_login.as_ref()
    }

    fn print(&self, mask: TraceMask, function: &str, args: std::fmt::Arguments<'_>) {
        self.trace
            .print_self(mask, format_args!("XPSController:{function}: {args}"));
    }

    /// C `XPSController::initializeProfile` (XPSController.cpp:521-527).
    pub fn initialize_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        max_points: usize,
        ftp_username: &str,
        ftp_password: &str,
    ) -> AsynResult<()> {
        self.ftp_login = Some((ftp_username.to_string(), ftp_password.to_string()));
        let _ = ctx.initialize_profile(max_points);
        Ok(())
    }

    /// C `XPSAxis::defineProfile` (XPSAxis.cpp): the base conversion to
    /// steps, then steps to XPS units.
    pub fn define_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        controller: &Mutex<XpsController>,
        addr: i32,
        positions: &[f64],
    ) -> AsynResult<()> {
        ctx.define_profile(addr, positions)?;
        let step = lock(controller)
            .registered_axes()
            .into_iter()
            .find(|a| a.axis_no == addr)
            .map(|a| a.step_size);
        if let (Some(step), Some(axis)) = (step, ctx.profile.axes.get_mut(addr as usize)) {
            for p in &mut axis.positions[..positions.len()] {
                *p *= step;
            }
        }
        Ok(())
    }

    /// C `XPSController::buildProfile` (XPSController.cpp:532-834).
    pub fn build_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        controller: &Mutex<XpsController>,
        ip: &str,
    ) -> AsynResult<()> {
        self.print(TraceMask::FLOW, "buildProfile", format_args!("entry\n"));
        let _ = ctx.build_profile();
        let _ = ctx
            .base
            .set_string_param(index::PROFILE_BUILD_MESSAGE, 0, "");
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_BUILD_STATE, 0, PROFILE_BUILD_BUSY);
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_BUILD_STATUS, 0, PROFILE_STATUS_UNDEFINED);
        let _ = ctx.base.call_param_callbacks(0);

        let done = self.build(ctx, controller, ip);
        let build_status = if done.build_ok && done.verify_ok {
            PROFILE_STATUS_SUCCESS
        } else {
            PROFILE_STATUS_FAILURE
        };
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_BUILD_STATUS, 0, build_status);
        let _ = ctx
            .base
            .set_string_param(index::PROFILE_BUILD_MESSAGE, 0, &done.message);
        if build_status != PROFILE_STATUS_SUCCESS {
            self.print(
                TraceMask::ERROR,
                "buildProfile",
                format_args!("{}\n", done.message),
            );
        }
        let _ = ctx.base.set_int32_param(index::PROFILE_BUILD, 0, 0);
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_BUILD_STATE, 0, PROFILE_BUILD_DONE);
        let _ = ctx.base.call_param_callbacks(0);
        if done.status != 0 {
            Err(xps_error())
        } else {
            Ok(())
        }
    }

    /// `buildProfile` from its parameter reads to `done:`.
    fn build(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        controller: &Mutex<XpsController>,
        ip: &str,
    ) -> BuildDone {
        let p = self.params();
        let mut done = BuildDone {
            build_ok: true,
            verify_ok: true,
            status: 0,
            message: String::new(),
        };
        let num_points = int_param(ctx.base, index::PROFILE_NUM_POINTS);
        let file_name = string_param(ctx.base, p.trajectory_file, MAX_FILENAME_LEN);
        let group = string_param(ctx.base, p.group_name, MAX_GROUPNAME_LEN);
        let registered = lock(controller).registered_axes();
        // Each axis in the group, and whether useAxis turns it on.
        let in_group: Vec<(&RegisteredAxis, bool)> = registered
            .iter()
            .filter(|a| a.group == group)
            .map(|a| (a, use_axis(ctx.base, a.axis_no)))
            .collect();

        // C indexes the arrays at numPoints-2 and past their end unchecked.
        let max_points = ctx.profile.max_points;
        if num_points < 2 || num_points as usize > max_points {
            done.build_ok = false;
            done.status = -1;
            done.message = format!("Error, numPoints={num_points}, maxPoints={max_points}\n");
            return done;
        }
        let n = num_points as usize;
        let times = ctx.profile.times[..n].to_vec();

        let mut max_accelerations = vec![None; in_group.len()];
        let (mut pre_time_max, mut post_time_max) = (0.0f64, 0.0f64);
        for (k, &(axis, used)) in in_group.iter().enumerate() {
            if !used {
                continue;
            }
            let sgamma = lock(controller)
                .poll_socket()
                .positioner_sgamma_parameters_get(&axis.positioner);
            let max_acceleration = match sgamma {
                Ok((_, acceleration, _, _)) => acceleration,
                Err(e) => {
                    done.build_ok = false;
                    done.status = e.code();
                    done.message = format!(
                        "Error calling positionerSGammaParametersSet, status={}\n",
                        e.code()
                    );
                    return done;
                }
            };
            max_accelerations[k] = Some(max_acceleration);
            // C reduces it 10% for the ASCII roundoff; generate() does too.
            let reduced = max_acceleration * 0.9;
            let pos = positions(ctx, axis.axis_no, n);
            pre_time_max = pre_time_max.max(((pos[1] - pos[0]) / times[0]).abs() / reduced);
            post_time_max =
                post_time_max.max(((pos[n - 1] - pos[n - 2]) / times[n - 1]).abs() / reduced);
            self.print(
                TraceMask::FLOW,
                "buildProfile",
                format_args!(
                    "axis {} profilePositions[0]={:.6}, profilePositions[{}]={:.6}, maxAcceleration={:.6}, preTimeMax={:.6}, postTimeMax={:.6}\n",
                    axis.axis_no,
                    CFloat(pos[0]),
                    n - 1,
                    CFloat(pos[n - 1]),
                    CFloat(reduced),
                    CFloat(pre_time_max),
                    CFloat(post_time_max)
                ),
            );
        }

        let profile = Profile {
            group: group.clone(),
            times,
            axes: in_group
                .iter()
                .map(|(a, _)| ProfileAxis {
                    positioner: a.positioner.clone(),
                    positions: positions(ctx, a.axis_no, n),
                })
                .collect(),
        };
        let trajectory = match profile.generate(&max_accelerations) {
            Ok(trajectory) => trajectory,
            Err(e) => {
                done.build_ok = false;
                done.status = -1;
                done.message = format!("{e}\n");
                return done;
            }
        };
        self.distances = registered
            .iter()
            .map(|a| {
                let ramp = in_group.iter().position(|(g, _)| g.axis_no == a.axis_no);
                let distances = ramp.map_or((0.0, 0.0), |i| {
                    (trajectory.pre_distance[i], trajectory.post_distance[i])
                });
                (a.axis_no, distances)
            })
            .collect();

        if let Err(e) = std::fs::write(&file_name, &trajectory.text) {
            done.build_ok = false;
            done.status = -1;
            done.message = format!(
                "Error creating trajectory file {file_name}, error={}\n",
                strerror(&e)
            );
            return done;
        }

        let (directory, use_sftp) = {
            let mut ctrl = lock(controller);
            match trajectory_upload(ctrl.firmware()) {
                Ok((directory, sftp)) => {
                    if sftp {
                        ctrl.set_use_sftp();
                    }
                    (directory, ctrl.use_sftp())
                }
                Err(message) => {
                    done.build_ok = false;
                    done.message = message;
                    return done;
                }
            }
        };
        let (user, password) = self
            .ftp_login
            .clone()
            .unwrap_or_else(|| ("(null)".into(), "(null)".into()));
        if let Err((status, message)) = if use_sftp {
            self.sftp_upload(ip, directory, &file_name, &user, &password)
        } else {
            ftp_upload(ip, directory, &file_name, &user, &password)
        } {
            done.build_ok = false;
            done.status = status;
            done.message = message;
            return done;
        }

        let ctrl = lock(controller);
        let sock = ctrl.poll_socket();
        self.print(
            TraceMask::FLOW,
            "buildProfile",
            format_args!(
                "calling MultipleAxesPVTVerification({}, {group}, {file_name})\n",
                sock.index()
            ),
        );
        done.status = sock
            .multiple_axes_pvt_verification(&group, &file_name)
            .map_or_else(|e| e.code(), |()| 0);
        if done.status != 0 {
            done.verify_ok = false;
        }
        done.message = verify_message(done.status);
        if !done.verify_ok {
            return done;
        }

        let (mut min_position, mut max_position) = (0.0, 0.0);
        // C reads them uninitialized when PositionerUserTravelLimitsGet
        // fails.
        let (mut low_limit, mut high_limit) = (f64::NEG_INFINITY, f64::INFINITY);
        for &(axis, used) in &in_group {
            let (mut max_velocity, mut max_acceleration) = (0.0, 0.0);
            let result = sock.multiple_axes_pvt_verification_result_get(&axis.positioner);
            done.status = match result {
                Ok((_, min, max, velocity, acceleration)) => {
                    (min_position, max_position) = (min, max);
                    (max_velocity, max_acceleration) = (velocity, acceleration);
                    0
                }
                Err(e) => e.code(),
            };
            let addr = axis.axis_no;
            let _ = ctx
                .base
                .set_float64_param(p.min_position, addr, min_position);
            let _ = ctx
                .base
                .set_float64_param(p.max_position, addr, max_position);
            let _ = ctx
                .base
                .set_float64_param(p.max_velocity, addr, max_velocity);
            let _ = ctx
                .base
                .set_float64_param(p.max_acceleration, addr, max_acceleration);
            if done.status != 0 {
                done.verify_ok = false;
                done.message = format!(
                    "MultipleAxesPVTVerificationResultGet error for axis {}, status={}\n",
                    axis.positioner, done.status
                );
                return done;
            }
            if !used {
                continue;
            }
            done.status = match sock.positioner_user_travel_limits_get(&axis.positioner) {
                Ok((low, high)) => {
                    (low_limit, high_limit) = (low, high);
                    0
                }
                Err(e) => e.code(),
            };
            let start = positions(ctx, addr, 1)[0];
            let min_profile = start + min_position;
            if min_profile < low_limit {
                done.verify_ok = false;
                done.message = format!(
                    "Low soft limit violation for axis {}, position={:.6}, limit={:.6}\n",
                    axis.positioner,
                    CFloat(min_profile),
                    CFloat(low_limit)
                );
                return done;
            }
            let max_profile = start + max_position;
            if max_profile > high_limit {
                done.verify_ok = false;
                done.message = format!(
                    "High soft limit violation for axis {}, position={:.6}, limit={:.6}\n",
                    axis.positioner,
                    CFloat(max_profile),
                    CFloat(high_limit)
                );
                return done;
            }
        }
        done
    }

    /// XPSController.cpp:710-721: `xpsSFTPUpload` between C's FLOW lines,
    /// verbose when the controller's own user traces IO_DRIVER.
    fn sftp_upload(
        &self,
        ip: &str,
        directory: &str,
        file_name: &str,
        user: &str,
        password: &str,
    ) -> Result<(), (i32, String)> {
        let verbose = PortManager::global().trace_manager().is_enabled_device(
            &self.port_name,
            0,
            TraceMask::IO_DRIVER,
        );
        self.trace.print_self(
            TraceMask::FLOW,
            format_args!("XPSController::buildProfile calling xpsSFTPUpload\n"),
        );
        let status = sftp::xps_sftp_upload(ip, directory, file_name, user, password, verbose);
        self.trace.print_self(
            TraceMask::FLOW,
            format_args!("XPSController::buildProfile xpsSFTPUpload returned {status}\n"),
        );
        match status {
            0 => Ok(()),
            _ => Err((
                status,
                format!("Error calling xpsSFTPUpload, status={status}\n"),
            )),
        }
    }

    /// C `XPSController::executeProfile`: signal the `XPSProfile` thread.
    pub fn execute_profile(&mut self) -> AsynResult<()> {
        // A signal already pending is one epicsEvent; a stopped thread
        // drops it, as C's would never wake.
        let _ = self.execute.try_send(());
        Ok(())
    }

    /// C `XPSController::abortProfile` (XPSController.cpp:1172-1185).
    pub fn abort_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        controller: &Mutex<XpsController>,
    ) -> AsynResult<()> {
        let group = string_param(ctx.base, self.params().group_name, MAX_GROUPNAME_LEN);
        let aborted = lock(controller).poll_socket().group_move_abort(&group);
        if let Err(e) = aborted {
            self.print(
                TraceMask::ERROR,
                "abortProfile",
                format_args!("Error performing GroupMoveAbort, status={}\n", e.code()),
            );
            return Err(xps_error());
        }
        Ok(())
    }

    /// C `XPSController::readbackProfile` (XPSController.cpp:1188-1300) with
    /// each `XPSAxis::readbackProfile`.
    pub fn readback_profile(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        controller: &Mutex<XpsController>,
    ) -> AsynResult<()> {
        self.print(TraceMask::FLOW, "readbackProfile", format_args!("entry\n"));
        let _ = ctx
            .base
            .set_string_param(index::PROFILE_READBACK_MESSAGE, 0, "");
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_READBACK_STATE, 0, PROFILE_READBACK_BUSY);
        let _ =
            ctx.base
                .set_int32_param(index::PROFILE_READBACK_STATUS, 0, PROFILE_STATUS_UNDEFINED);
        let _ = ctx.base.call_param_callbacks(0);
        let num_pulses = int_param(ctx.base, index::PROFILE_NUM_PULSES);
        for axis in &mut ctx.profile.axes {
            axis.readbacks.fill(0.0);
            axis.following_errors.fill(0.0);
        }

        let registered = lock(controller).registered_axes();
        let done = {
            let ctrl = lock(controller);
            self.read_gathering(
                ctrl.poll_socket(),
                registered.len(),
                num_pulses,
                ctx.profile.max_points,
            )
        };
        let num_read = done.samples.num_samples();
        for (j, axis) in registered.iter().enumerate() {
            let Some(profile) = ctx.profile.axes.get_mut(axis.axis_no as usize) else {
                continue;
            };
            let n = num_read.min(profile.readbacks.len());
            profile.readbacks[..n].copy_from_slice(&done.samples.actuals[j][..n]);
            profile.following_errors[..n].copy_from_slice(&done.samples.following_errors[j][..n]);
        }
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_ACTUAL_PULSES, 0, num_read as i32);
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_NUM_READBACKS, 0, num_read as i32);
        for axis in &registered {
            // XPSAxis::readbackProfile: XPS units to steps, then the base.
            if let Some(profile) = ctx.profile.axes.get_mut(axis.axis_no as usize) {
                let n = num_read.min(profile.readbacks.len());
                for v in &mut profile.readbacks[..n] {
                    *v /= axis.step_size;
                }
                for v in &mut profile.following_errors[..n] {
                    *v /= axis.step_size;
                }
            }
            let _ = ctx.axis_readback_profile(axis.axis_no);
        }
        let status = if done.ok {
            PROFILE_STATUS_SUCCESS
        } else {
            PROFILE_STATUS_FAILURE
        };
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_READBACK_STATUS, 0, status);
        let _ = ctx
            .base
            .set_string_param(index::PROFILE_READBACK_MESSAGE, 0, &done.message);
        if !done.ok {
            self.print(
                TraceMask::ERROR,
                "readbackProfile",
                format_args!("{}\n", done.message),
            );
        }
        let _ = ctx.base.set_int32_param(index::PROFILE_READBACK, 0, 0);
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_READBACK_STATE, 0, PROFILE_READBACK_DONE);
        let _ = ctx.base.call_param_callbacks(0);
        if done.status != 0 {
            Err(xps_error())
        } else {
            Ok(())
        }
    }

    /// `readbackProfile` from `GatheringCurrentNumberGet` to `done:`.
    fn read_gathering(
        &self,
        sock: &XpsSocket,
        num_axes: usize,
        num_pulses: i32,
        max_points: usize,
    ) -> ReadbackDone {
        let flow = |args: std::fmt::Arguments<'_>| {
            self.print(TraceMask::FLOW, "readbackProfile", args);
        };
        let mut done = ReadbackDone {
            ok: true,
            status: 0,
            message: String::new(),
            samples: GatheringReadback::new(num_axes),
        };
        let (current_samples, max_samples) = match sock.gathering_current_number_get() {
            Ok(n) => n,
            Err(e) => {
                done.status = e.code();
                (0, 0)
            }
        };
        flow(format_args!(
            "GatheringCurrentNumberGet, status={}, currentSamples={current_samples}, maxSamples={max_samples}\n",
            done.status
        ));
        if done.status != 0 {
            done.ok = false;
            done.message = format!(
                "Error calling GatherCurrentNumberGet, status={}",
                done.status
            );
            return done;
        }
        if current_samples < num_pulses {
            done.ok = false;
            done.message =
                format!("Error, numPulses={num_pulses}, currentSamples={current_samples}");
        }
        let current_samples = current_samples.min(max_points as i32);
        let mut num_read = 0;
        while num_read < current_samples {
            done.status = -1;
            let mut num_in_buffer = current_samples - num_read;
            let mut buffer = String::new();
            while done.status != 0 && num_in_buffer > 0 {
                done.status = match sock.gathering_data_multiple_lines_get(num_read, num_in_buffer)
                {
                    Ok(read) => {
                        buffer = read;
                        0
                    }
                    Err(e) => e.code(),
                };
                flow(format_args!(
                    "GatheringDataMultipleLinesGet, status={}, numInBuffer={num_in_buffer}\n",
                    done.status
                ));
                if done.status != 0 {
                    num_in_buffer /= 2;
                }
            }
            if num_in_buffer == 0 {
                done.ok = false;
                done.message = "Error reading gathering data, numInBuffer = 0".into();
                return done;
            }
            match parse_gathering_buffer(&buffer, num_axes, &mut done.samples) {
                Ok(lines) if lines as i32 == num_in_buffer => num_read += num_in_buffer,
                // C's sscanf past the last line answers EOF.
                Ok(_) => {
                    done.ok = false;
                    done.message =
                        "Error reading Gathering.dat file, nitems=-1, should be 2".into();
                    return done;
                }
                Err(message) => {
                    done.ok = false;
                    done.message = message;
                    return done;
                }
            }
        }
        done
    }

    /// `runProfile`'s first locked block (XPSController.cpp:887-904) and
    /// the reads its moves use.
    fn start_run(
        &mut self,
        ctx: &mut ControllerCtx<'_>,
        registered: Vec<RegisteredAxis>,
    ) -> RunPlan {
        let p = self.params();
        let base = &mut *ctx.base;
        let file_name = string_param(base, p.trajectory_file, MAX_FILENAME_LEN);
        let group = string_param(base, p.group_name, MAX_GROUPNAME_LEN);
        let start_pulses = int_param(base, index::PROFILE_START_PULSES);
        let end_pulses = int_param(base, index::PROFILE_END_PULSES);
        let num_points = int_param(base, index::PROFILE_NUM_POINTS);
        let num_pulses = int_param(base, index::PROFILE_NUM_PULSES);
        let _ = base.set_string_param(index::PROFILE_EXECUTE_MESSAGE, 0, " ");
        let _ = base.set_int32_param(index::PROFILE_EXECUTE_STATE, 0, PROFILE_EXECUTE_MOVE_START);
        let _ = base.set_int32_param(index::PROFILE_EXECUTE_STATUS, 0, PROFILE_STATUS_UNDEFINED);
        let _ = base.call_param_callbacks(0);
        let absolute = int_param(base, index::PROFILE_MOVE_MODE) == PROFILE_MOVE_MODE_ABSOLUTE;

        let n = (num_points.max(0) as usize).min(ctx.profile.times.len());
        let axes = registered
            .iter()
            .filter(|a| a.group == group && use_axis(ctx.base, a.axis_no))
            .map(|a| {
                let pos = positions(ctx, a.axis_no, n);
                let (pre, post) = self.distances.get(&a.axis_no).copied().unwrap_or_default();
                let (start, end) = if absolute {
                    let first = pos.first().copied().unwrap_or(0.0);
                    (first - pre, pos.last().copied().unwrap_or(0.0))
                } else {
                    (-pre, -post)
                };
                RunAxis {
                    positioner: a.positioner.clone(),
                    move_sock: a.move_sock.clone(),
                    start,
                    end,
                }
            })
            .collect();
        RunPlan {
            file_name,
            group,
            pulses: (start_pulses, end_pulses, num_pulses),
            times: ctx.profile.times[..n].to_vec(),
            absolute,
            axes,
            gathered: registered.into_iter().map(|a| a.positioner).collect(),
        }
    }

    /// `runProfile`'s last locked block (XPSController.cpp:1131-1146).
    fn finish_run(&mut self, ctx: &mut ControllerCtx<'_>, done: RunDone) {
        let status = if done.ok {
            PROFILE_STATUS_SUCCESS
        } else if done.aborted {
            PROFILE_STATUS_ABORT
        } else {
            PROFILE_STATUS_FAILURE
        };
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_EXECUTE_STATUS, 0, status);
        let _ = ctx
            .base
            .set_string_param(index::PROFILE_EXECUTE_MESSAGE, 0, &done.message);
        if !done.ok {
            self.print(
                TraceMask::ERROR,
                "runProfile",
                format_args!("{}\n", done.message),
            );
        }
        let _ = ctx.base.set_int32_param(index::PROFILE_EXECUTE, 0, 0);
        let _ = ctx
            .base
            .set_int32_param(index::PROFILE_EXECUTE_STATE, 0, PROFILE_EXECUTE_DONE);
        let _ = ctx.base.call_param_callbacks(0);
    }
}

/// XPSController.cpp:722-747: C's FTP session, its status and message.
fn ftp_upload(
    ip: &str,
    directory: &str,
    file_name: &str,
    user: &str,
    password: &str,
) -> Result<(), (i32, String)> {
    let mut sock = ftp::ftp_connect(ip, user, password).map_err(|status| {
        (
            status,
            format!("Error calling ftpConnect, status={status}\n"),
        )
    })?;
    let check = |status: i32, call: &str| match status {
        0 => Ok(()),
        _ => Err((status, format!("Error calling  {call}, status={status}\n"))),
    };
    check(ftp::ftp_change_dir(&mut sock, directory), "ftpChangeDir")?;
    check(ftp::ftp_store_file(&mut sock, file_name), "ftpStoreFile")?;
    check(ftp::ftp_disconnect(sock), "ftpDisconnect")
}

/// What `runProfile` reads under its first lock.
struct RunPlan {
    file_name: String,
    group: String,
    /// `(startPulses, endPulses, numPulses)`.
    pulses: (i32, i32, i32),
    times: Vec<f64>,
    absolute: bool,
    /// The axes in the group that useAxis turns on, which move.
    axes: Vec<RunAxis>,
    /// Every axis, which the gathering samples.
    gathered: Vec<String>,
}

struct RunAxis {
    positioner: String,
    move_sock: Arc<XpsSocket>,
    /// The targets of the moves before and after the trajectory.
    start: f64,
    end: f64,
}

impl ProfileThread {
    /// C `XPSController::poll` (XPSController.cpp:1150-1170): the current
    /// trajectory element while one executes.
    pub fn poll(
        &self,
        controller: Arc<Mutex<XpsController>>,
    ) -> impl FnMut() -> PortSets + Send + 'static {
        let executing = self.executing.clone();
        move || {
            let mut sets = PortSets::default();
            let Some(group) = lock(&executing).clone() else {
                return sets;
            };
            let (mut file, mut element) = (String::new(), 0);
            let read = lock(&controller)
                .poll_socket()
                .multiple_axes_pvt_parameters_get(&group, &mut file, &mut element);
            if read.is_ok() {
                sets.set(0, index::PROFILE_CURRENT_POINT, ParamValue::Int32(element));
                sets.call_param_callbacks(0);
            }
            sets
        }
    }

    /// C's `epicsThreadCreate("XPSProfile", epicsThreadPriorityLow,
    /// epicsThreadGetStackSize(epicsThreadStackMedium), ...)`: `runProfile`
    /// on each `executeProfile`.
    pub fn spawn<P: ProfileHost>(
        self,
        port: PortHandle,
        controller: Arc<Mutex<XpsController>>,
        profile_sock: Arc<XpsSocket>,
    ) -> std::io::Result<()> {
        let runner = Runner::<P> {
            trace: MotorTrace::new(&self.port_name, 0),
            port,
            controller,
            profile_sock,
            executing: self.executing,
            host: PhantomData,
        };
        let execute = self.execute;
        spawn_dedicated_thread(
            "XPSProfile".into(),
            ThreadPriority::Low,
            StackSizeClass::Medium,
            move || {
                while execute.recv().is_ok() {
                    runner.run();
                }
            },
        )
        .map(drop)
    }
}

/// The `XPSProfile` thread's side of the controller.
struct Runner<P> {
    trace: MotorTrace,
    port: PortHandle,
    controller: Arc<Mutex<XpsController>>,
    /// The controller's `moveSocket_`, on which `MultipleAxesPVTExecution`
    /// blocks until the trajectory ends.
    profile_sock: Arc<XpsSocket>,
    executing: Arc<Mutex<Option<String>>>,
    host: PhantomData<fn() -> P>,
}

impl<P: ProfileHost> Runner<P> {
    /// C `lock()` ... `unlock()`. The caller holds no controller lock.
    fn locked<R: Send + 'static>(
        &self,
        f: impl FnOnce(&mut XpsProfile, &mut ControllerCtx<'_>) -> R + Send + 'static,
    ) -> AsynResult<R> {
        self.port
            .with_driver_blocking::<ControllerPort<P>, _>(move |d| {
                d.with_params(|p, ctx| f(p.xps_profile(), ctx))
            })
    }

    fn flow(&self, args: std::fmt::Arguments<'_>) {
        run_flow(&self.trace, args);
    }

    /// C `XPSController::runProfile` (XPSController.cpp:862-1147).
    fn run(&self) {
        let registered = lock(&self.controller).registered_axes();
        let Ok(plan) = self.locked(move |profile, ctx| profile.start_run(ctx, registered)) else {
            return;
        };
        self.moves(&plan, |a| a.start);
        let _ = self.locked(|_, ctx| ctx.wakeup_poller());
        self.wait_motors(&plan.group);

        let (executing, group) = (self.executing.clone(), plan.group.clone());
        let _ = self.locked(move |_, ctx| {
            let _ = ctx.base.set_int32_param(
                index::PROFILE_EXECUTE_STATE,
                0,
                PROFILE_EXECUTE_EXECUTING,
            );
            *lock(&executing) = Some(group);
            let _ = ctx.base.call_param_callbacks(0);
        });

        let mut done = RunDone {
            ok: true,
            aborted: false,
            message: " ".into(),
        };
        self.execute_gathered(&plan, &mut done);

        let executing = self.executing.clone();
        let _ = self.locked(move |_, ctx| {
            let _ =
                ctx.base
                    .set_int32_param(index::PROFILE_EXECUTE_STATE, 0, PROFILE_EXECUTE_FLYBACK);
            *lock(&executing) = None;
            let _ = ctx.base.call_param_callbacks(0);
        });
        self.moves(&plan, |a| a.end);
        let _ = self.locked(|_, ctx| ctx.wakeup_poller());
        self.wait_motors(&plan.group);
        let _ = self.locked(move |profile, ctx| profile.finish_run(ctx, done));
    }

    /// The moves before and after the trajectory, each on its axis's move
    /// socket; C ignores their status.
    fn moves(&self, plan: &RunPlan, target: impl Fn(&RunAxis) -> f64) {
        for axis in &plan.axes {
            let _ = if plan.absolute {
                axis.move_sock
                    .group_move_absolute(&axis.positioner, &[target(axis)])
            } else {
                axis.move_sock
                    .group_move_relative(&axis.positioner, target(axis))
            };
        }
    }

    /// C `XPSController::waitMotors` (XPSController.cpp:502-517).
    fn wait_motors(&self, group: &str) {
        loop {
            match lock(&self.controller).poll_socket().group_status_get(group) {
                Ok(10..=18) | Err(_) => return,
                Ok(_) => {}
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// `runProfile` from the gathering set-up to `done:`.
    fn execute_gathered(&self, plan: &RunPlan, done: &mut RunDone) {
        let event_id = {
            let ctrl = lock(&self.controller);
            start_gathering(
                &self.trace,
                ctrl.poll_socket(),
                &plan.group,
                &plan.gathered,
                &plan.times,
                plan.pulses,
            )
        };
        let event_id = match event_id {
            Ok(id) => id,
            Err(message) => return done.failed(message),
        };
        let _ = self.locked(|_, ctx| ctx.wakeup_poller());

        self.flow(format_args!(
            "calling MultipleAxesPVTExecution({}, {}, {}, 1)\n",
            self.profile_sock.index(),
            plan.group,
            plan.file_name
        ));
        match self
            .profile_sock
            .multiple_axes_pvt_execution(&plan.group, &plan.file_name, 1)
        {
            Ok(()) => {}
            // -27: the trajectory was aborted.
            Err(XpsError::Api(-27)) => {
                done.aborted = true;
                done.failed("MultipleAxesPVTExecution aborted".into());
            }
            Err(e) => done.failed(format!(
                "Error performing MultipleAxesPVTExecution, status={}",
                e.code()
            )),
        }
        let ctrl = lock(&self.controller);
        if let Some(message) = stop_gathering(&self.trace, ctrl.poll_socket(), event_id) {
            done.failed(message);
        }
    }
}

/// `asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: ...")` in
/// `runProfile`.
pub(crate) fn run_flow(trace: &MotorTrace, args: std::fmt::Arguments<'_>) {
    trace.print_self(
        TraceMask::FLOW,
        format_args!("XPSController:runProfile: {args}"),
    );
}

/// Configure and start gathering for a trajectory run: reset the buffer,
/// declare `SetpointPosition`+`CurrentPosition` per positioner, window the
/// trajectory pulse train, and fire `GatheringOneData` on every pulse (C
/// `runProfile`, XPSController.cpp:935-1055). Returns the extended-event ID
/// to remove after the run, or C's message.
pub(crate) fn start_gathering(
    trace: &MotorTrace,
    sock: &XpsSocket,
    group: &str,
    positioners: &[String],
    times: &[f64],
    (start_pulses, end_pulses, num_pulses): (i32, i32, i32),
) -> Result<i32, String> {
    let flow = |args: std::fmt::Arguments<'_>| run_flow(trace, args);
    let index = sock.index();
    flow(format_args!("calling GatheringReset({index})\n"));
    sock.gathering_reset()
        .map_err(|e| format!("Error performing GatheringReset, status={}", e.code()))?;
    let types: Vec<String> = positioners
        .iter()
        .flat_map(|p| {
            [
                format!("{p}.SetpointPosition"),
                format!("{p}.CurrentPosition"),
            ]
        })
        .collect();
    let buffer: String = types.iter().map(|t| format!("{t};")).collect();
    flow(format_args!(
        "calling GatheringConfigurationSet({index}, {}, {buffer})\n",
        types.len()
    ));
    sock.gathering_configuration_set(&types).map_err(|e| {
        format!(
            "Error performing GatheringConfigurationSet, status={}, buffer={buffer}",
            e.code()
        )
    })?;
    let (pulse_start, pulse_end, pulse_period) =
        profile::pulse_output_window(times, start_pulses, end_pulses, num_pulses)?;
    flow(format_args!(
        "calling MultipleAxesPVTPulseOutputSet({index}, {group}, {pulse_start}, {pulse_end}, {:.6})\n",
        CFloat(pulse_period)
    ));
    // C ignores its status.
    let _ = sock.multiple_axes_pvt_pulse_output_set(group, pulse_start, pulse_end, pulse_period);
    let triggers = vec!["Always".to_string(), format!("{group}.PVT.TrajectoryPulse")];
    let buffer = triggers.join(";");
    // C's format has a '.' for the last separator.
    flow(format_args!(
        "calling EventExtendedConfigurationTriggerSet({index}, {}, {buffer}, , , . )\n",
        triggers.len()
    ));
    sock.event_extended_configuration_trigger_set(&triggers)
        .map_err(|e| {
            format!(
                "Error performing EventExtendedConfigurationTriggerSet, status={}, buffer={buffer}, strlen(buffer)={}",
                e.code(),
                buffer.len()
            )
        })?;
    flow(format_args!(
        "calling EventExtendedConfigurationActionSet({index}, 1, GatheringOneData, , , , )\n"
    ));
    sock.event_extended_configuration_action_set(&["GatheringOneData".to_string()])
        .map_err(|e| {
            format!(
                "Error performing EventExtendedConfigurationActionSet, status={}",
                e.code()
            )
        })?;
    // C prints `&eventId` with %p.
    let event_id = 0i32;
    flow(format_args!(
        "calling EventExtendedStart({index}, {:p})\n",
        &event_id
    ));
    sock.event_extended_start()
        .map_err(|e| format!("Error performing EventExtendedStart, status={}", e.code()))
}

/// After `MultipleAxesPVTExecution`, whatever it returned: remove the event
/// and stop gathering (XPSController.cpp:1075-1100). C's message for the
/// last of them that failed.
pub(crate) fn stop_gathering(
    trace: &MotorTrace,
    sock: &XpsSocket,
    event_id: i32,
) -> Option<String> {
    let flow = |args: std::fmt::Arguments<'_>| run_flow(trace, args);
    let mut message = None;
    flow(format_args!(
        "calling EventExtendedRemove({}, {event_id})\n",
        sock.index()
    ));
    if let Err(e) = sock.event_extended_remove(event_id) {
        message = Some(format!(
            "Error performing ExtendedEventRemove, status={}",
            e.code()
        ));
    }
    flow(format_args!("calling GatheringStop({})\n", sock.index()));
    match sock.gathering_stop() {
        // -30: gathering never started (aborted before one element
        // completed); C tolerates it.
        Ok(()) | Err(XpsError::Api(-30)) => {}
        Err(e) => {
            message = Some(format!(
                "Error performing GatheringStop, status={}",
                e.code()
            ))
        }
    }
    message
}
