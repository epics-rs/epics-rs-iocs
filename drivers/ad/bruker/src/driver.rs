//! The BIS areaDetector driver (C `BISDetector.cpp`).
//!
//! Ownership: every command to BIS goes through [`BisServer`]; the port actor
//! owns the parameter library and is the only thing that reacts to a record
//! write. The two background threads — the acquisition task and the status task
//! — reach the parameter library through the actor and never call into the
//! driver directly (see the invariant in [`crate::connection`]).

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::sync_channel;

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::write_tail::{OwnList, WriteTail};
use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode, ShutterMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;

use crate::connection::{BisServer, Exchange};
use crate::filename;
use crate::params::BrukerParams;
use crate::protocol;
use crate::task::{self, AcquisitionContext, StatusContext};
use crate::types::*;

const INT32_TAIL: WriteTail = WriteTail {
    error: "BISDetector:writeInt32: error, ",
    done: "BISDetector:writeInt32: ",
    list: OwnList::Zero,
};

/// The one thing the actor tells the acquisition task.
pub enum TaskCommand {
    Start,
}

/// State the port actor shares with the acquisition task.
pub struct SharedState {
    /// Set by the actor when Acquire goes to 0, cleared by the task when it
    /// takes the next Start: a stop that arrives while idle cannot abort the
    /// run after it.
    ///
    /// C had one `stopEventId` for both the exposure timer's expiry and the
    /// user's Stop, so a Stop could be swallowed by the timer's `break` — and
    /// when it was *not* swallowed, the task went on to wait for the readout
    /// and read the frame file as if the exposure had run to completion. Here
    /// the timer is just the end of the countdown loop and the Stop is a flag
    /// of its own.
    pub stop_requested: AtomicBool,
}

pub struct BrukerDetector {
    pub ad: ADDriverBase,
    pub params: BrukerParams,
    server: BisServer,
    shared: Arc<SharedState>,
    commands: rt::CommandSender<TaskCommand>,
    /// C `BISTask`'s `fullFileName` local, kept across frames.
    full_file_name: String,
}

impl BrukerDetector {
    fn new(
        port_name: &str,
        server: BisServer,
        max_memory: usize,
        shared: Arc<SharedState>,
        commands: rt::CommandSender<TaskCommand>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            max_memory,
            ASYN_CANBLOCK,
            InterfaceMask::UNMIGRATED_SUBCLASS,
            InterfaceMask::UNMIGRATED_SUBCLASS_INTERRUPTS,
        )?;
        let params = BrukerParams::create(&mut ad.port_base)?;
        let p = ad.params;

        let base = &mut ad.port_base;
        base.set_string_param(p.base.manufacturer, 0, "Bruker")?;
        base.set_string_param(p.base.model, 0, "BIS")?;
        base.set_int32_param(p.max_size_x, 0, MAX_SIZE)?;
        base.set_int32_param(p.max_size_y, 0, MAX_SIZE)?;
        // BIS says how big a frame is on the status socket; until it does, the
        // geometry is unknown.
        base.set_int32_param(p.size_x, 0, 0)?;
        base.set_int32_param(p.size_y, 0, 0)?;
        base.set_int32_param(p.base.array_size_x, 0, 0)?;
        base.set_int32_param(p.base.array_size_y, 0, 0)?;
        base.set_int32_param(p.base.array_size, 0, 0)?;
        base.set_int32_param(p.base.data_type, 0, NDDataType::UInt32 as u8 as i32)?;
        base.set_int32_param(p.image_mode, 0, ImageMode::Continuous as i32)?;
        // The same values the template's PINI records write.
        base.set_float64_param(params.sfrm_timeout, 0, 30.0)?;
        base.set_int32_param(params.num_darks, 0, 2)?;

        let mut det = Self {
            ad,
            params,
            server,
            shared,
            commands,
            full_file_name: String::new(),
        };
        det.ad.port_base.call_param_callbacks(0)?;
        Ok(det)
    }

    /// Send one command; publish both sides of it (C `writeBIS`,
    /// BISDetector.cpp:309-330). No caller of C's `writeBIS` looks at its
    /// status: a failure is the trace line.
    fn command(&mut self, command: &str, timeout: std::time::Duration) {
        let (reply, status) = match self.server.command(command, timeout) {
            Exchange::NotSent(e) => (None, Err(e)),
            Exchange::Read { reply, status } => (Some(reply), status),
        };
        if let Err(e) = status {
            self.trace_error(&format!(
                "BISDetector:writeBIS, status={}, sent\n{command}\n",
                e.c_status_code()
            ));
        }
        let p = self.ad.params;
        let base = &mut self.ad.port_base;
        let _ = base.set_string_param(p.string_to_server, 0, command);
        // fromBIS is what the read left; a failure before it keeps the last.
        if let Some(reply) = reply {
            let _ = base.set_string_param(p.string_from_server, 0, reply);
        }
    }

    fn trace_error(&self, message: &str) {
        self.ad
            .port_base
            .trace_print(TraceMask::ERROR, format_args!("{}", message));
    }

    fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    /// Build the next frame's file name (C `asynNDArrayDriver::createFileName`,
    /// which refuses when any of its five reads fails, asynNDArrayDriver.cpp:205-210).
    fn create_file_name(&mut self) -> AsynResult<String> {
        let p = self.ad.params.base;
        let base = &mut self.ad.port_base;
        let path = String::from_utf8_lossy(base.get_string_param(p.file_path, 0)?).into_owned();
        let name = String::from_utf8_lossy(base.get_string_param(p.file_name, 0)?).into_owned();
        let template =
            String::from_utf8_lossy(base.get_string_param(p.file_template, 0)?).into_owned();
        let number = base.get_int32_param(p.file_number, 0)?;
        let auto_increment = base.get_int32_param(p.auto_increment, 0)?;

        let full = filename::expand(&template, &path, &name, number);
        base.set_string_param(p.full_file_name, 0, full.clone())?;
        if auto_increment != 0 {
            base.set_int32_param(p.file_number, 0, number + 1)?;
        }
        Ok(full)
    }

    /// Name this frame's file and tell BIS to collect it (C `BISTask`'s
    /// `createFileName` + `switch (frameType)` + `writeBIS(2.0)`, which ran on
    /// the acquisition thread with the port locked).
    ///
    /// The acquisition task asks for this through `BIS_START_SCAN`, so the
    /// command socket has exactly one user and the parameters the command is
    /// built from are read where they are owned.
    fn start_scan(&mut self) {
        // C ignores every status here (BISDetector.cpp:496-530): a failed
        // name leaves BISTask's fullFileName at the last one.
        if let Ok(full) = self.create_file_name() {
            self.full_file_name = full;
        }
        let file_name = self.full_file_name.clone();

        let p = self.ad.params;
        let frame_type =
            FrameType::from_i32(self.get_i32(p.frame_type)).unwrap_or(FrameType::Normal);
        let acquire_time = self
            .ad
            .port_base
            .get_float64_param(p.acquire_time, 0)
            .unwrap_or(0.0);
        let num_darks = self.get_i32(self.params.num_darks);

        let _ = self
            .ad
            .port_base
            .set_string_param(p.status_message, 0, "Starting exposure");
        let _ = self.ad.port_base.call_param_callbacks(0);

        let command = protocol::acquire(frame_type, &file_name, acquire_time, num_darks);
        self.command(&command, BIS_COMMAND_TIMEOUT);
    }
}

impl PortDriver for BrukerDetector {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let p = self.ad.params;
        let q = self.params;
        INT32_TAIL.write_int32(self, user, value, |det, user, status| {
            if reason == p.acquire {
                let idle = det.get_i32(p.status) == ADStatus::Idle as i32;
                if value != 0 && idle {
                    // The actor, not the task, leaves Idle: two Acquire writes in a
                    // row cannot queue two acquisitions.
                    let _ = det
                        .ad
                        .port_base
                        .set_int32_param(p.status, 0, ADStatus::Acquire as i32);
                    let _ = det.commands.try_send(TaskCommand::Start);
                } else if value == 0 && !idle {
                    det.shared.stop_requested.store(true, Ordering::Release);
                }
            } else if reason == p.bin_x {
                if matches!(value, 1 | 2 | 4 | 8) {
                    let max_size_x = det.get_i32(p.max_size_x);
                    // There is one binning: X and Y are the same.
                    let _ = det.ad.port_base.set_int32_param(p.bin_y, 0, value);
                    det.command(
                        &protocol::change_frame_size(max_size_x / value),
                        BIS_DEFAULT_TIMEOUT,
                    );
                } else {
                    det.trace_error(&format!(
                        "BISDetector:writeInt32: invalid binning={value}, must be 1,2,4 or 8\n"
                    ));
                    // C sets no errorMessage (BISDetector.cpp:660-664).
                    return Err(AsynError::Status {
                        status: AsynStatus::Error,
                        message: String::new(),
                    });
                }
            } else if reason == q.start_scan {
                det.start_scan();
            } else if reason == q.epics_shutter {
                // C called `ADDriver::setShutter` from the acquisition task, going
                // past its own override on purpose: in EPICS shutter mode the
                // shutter is not BIS's to drive. It is void there
                // (BISDetector.cpp:541, :561).
                det.ad.set_shutter(value != 0);
            } else if reason < q.sfrm_timeout {
                // FIRST_BIS_PARAM is BISSFRMTimeout (BISDetector.cpp:84).
                return ADDriver::ad_write_int32(det, user, value);
            }

            status
        })
    }

    /// C inherits `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, value: &[u8]) -> AsynResult<usize> {
        self.ad.write_octet(user, value)
    }
}

impl ADDriver for BrukerDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }

    /// BIS drives the shutter itself when the shutter is wired to the detector
    /// (C `BISDetector::setShutter`, void like `ADDriver::setShutter`).
    fn set_shutter(&mut self, open: bool) {
        let p = self.ad.params;
        if ShutterMode::from_i32(self.get_i32(p.shutter_mode)) != Some(ShutterMode::DetectorOnly) {
            self.ad.set_shutter(open);
            return;
        }

        let open_delay = self
            .ad
            .port_base
            .get_float64_param(p.shutter_open_delay, 0)
            .unwrap_or(0.0);
        let close_delay = self
            .ad
            .port_base
            .get_float64_param(p.shutter_close_delay, 0)
            .unwrap_or(0.0);
        self.command(&protocol::shutter(open), BIS_COMMAND_TIMEOUT);

        // Opening: wait out the difference between the opening and the closing
        // time so the exposure is the length that was asked for, and never less
        // than a millisecond so two commands do not go out back to back.
        // Closing: wait out the closing time.
        let delay = if open {
            (open_delay - close_delay).max(0.001)
        } else {
            close_delay
        };
        if delay > 0.0 {
            std::thread::sleep(std::time::Duration::from_secs_f64(delay));
        }
        let _ = self.ad.port_base.call_param_callbacks(0);
    }
}

/// What the IOC layer holds on to.
pub struct BrukerRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub params: BrukerParams,
    pool: DriverPool,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    #[allow(dead_code)]
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl BrukerRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
        self.array_output.lock().add(sender);
    }
}

/// Create the detector port and start its two threads (C `BISDetectorConfig`).
///
/// `command_handle` and `status_handle` are the users of the two
/// `drvAsynIPPort`s that reach BIS: one to send commands on and one BIS
/// broadcasts its status on. C ignores both connect statuses
/// (BISDetector.cpp:759-760), so either may be a user with no port.
pub fn create_bruker_detector(
    claim: &PortClaim,
    command_handle: SyncIOHandle,
    status_handle: SyncIOHandle,
    max_memory: usize,
) -> AsynResult<BrukerRuntime> {
    let port_name = claim.name();
    let shared = Arc::new(SharedState {
        stop_requested: AtomicBool::new(false),
    });
    let (command_tx, command_rx) = rt::command_channel::<TaskCommand>(1);
    // BIS says "processing finished" on the status socket; the acquisition task
    // is what waits for it.
    let (readout_tx, readout_rx) = sync_channel::<()>(1);

    let det = BrukerDetector::new(
        port_name,
        BisServer::new(command_handle),
        max_memory,
        shared.clone(),
        command_tx,
    )?;
    let ad_params = det.ad.params;
    let params = det.params;
    let pool = det.ad.pool.clone();
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let threads = vec![
        task::start_acquisition_task(AcquisitionContext {
            handle: runtime_handle.port_handle().clone(),
            output: ArrayPublisher::new(array_output.clone()),
            pool: pool.clone(),
            ad_params,
            params,
            shared: shared.clone(),
            commands: command_rx,
            readout: readout_rx,
        }),
        task::start_status_task(StatusContext {
            status: status_handle,
            handle: runtime_handle.port_handle().clone(),
            ad_params,
            params,
            readout: readout_tx,
        }),
    ];

    Ok(BrukerRuntime {
        runtime_handle,
        ad_params,
        params,
        pool,
        array_output,
        threads,
    })
}
