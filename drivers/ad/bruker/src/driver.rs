//! The BIS areaDetector driver (C `BISDetector.cpp`).
//!
//! Ownership: every command to BIS goes through [`BisServer`]; the port actor
//! owns the parameter library and is the only thing that reacts to a record
//! write. The two background threads — the acquisition task and the status task
//! — reach the parameter library through the actor and never call into the
//! driver directly (see the invariant in [`crate::connection`]).

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::sync_channel;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode, ShutterMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;

use crate::connection::BisServer;
use crate::filename;
use crate::params::BrukerParams;
use crate::protocol;
use crate::task::{self, AcquisitionContext, StatusContext};
use crate::types::*;

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
}

impl BrukerDetector {
    fn new(
        port_name: &str,
        server: BisServer,
        max_memory: usize,
        shared: Arc<SharedState>,
        commands: rt::CommandSender<TaskCommand>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, MAX_SIZE, MAX_SIZE, max_memory)?;
        let params = BrukerParams::create(&mut ad.port_base)?;
        let p = ad.params;

        let base = &mut ad.port_base;
        base.set_string_param(p.base.manufacturer, 0, "Bruker".into())?;
        base.set_string_param(p.base.model, 0, "BIS".into())?;
        base.set_string_param(p.base.driver_version, 0, env!("CARGO_PKG_VERSION").into())?;
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
        };
        det.ad.port_base.call_param_callbacks(0)?;
        Ok(det)
    }

    /// Send one command; publish both sides of it (C `writeBIS`).
    fn command(&mut self, command: &str, timeout: std::time::Duration) -> AsynResult<()> {
        let exchange = self.server.command(command, timeout);
        let p = self.ad.params;
        let base = &mut self.ad.port_base;
        base.set_string_param(p.string_to_server, 0, command.into())?;
        base.set_string_param(p.string_from_server, 0, exchange.reply)?;
        exchange.result
    }

    fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    /// Build the next frame's file name (C `asynNDArrayDriver::createFileName`).
    fn create_file_name(&mut self) -> AsynResult<()> {
        let p = self.ad.params.base;
        let base = &mut self.ad.port_base;
        let path = base.get_string_param(p.file_path, 0)?.to_string();
        let name = base.get_string_param(p.file_name, 0)?.to_string();
        let template = base.get_string_param(p.file_template, 0)?.to_string();
        let number = base.get_int32_param(p.file_number, 0)?;
        let auto_increment = base.get_int32_param(p.auto_increment, 0).unwrap_or(0);

        let full = filename::expand(&template, &path, &name, number);
        base.set_string_param(p.full_file_name, 0, full)?;
        if auto_increment != 0 {
            base.set_int32_param(p.file_number, 0, number + 1)?;
        }
        Ok(())
    }

    /// Name this frame's file and tell BIS to collect it (C `BISTask`'s
    /// `createFileName` + `switch (frameType)` + `writeBIS(2.0)`, which ran on
    /// the acquisition thread with the port locked).
    ///
    /// The acquisition task asks for this through `BIS_START_SCAN`, so the
    /// command socket has exactly one user and the parameters the command is
    /// built from are read where they are owned.
    fn start_scan(&mut self) -> AsynResult<()> {
        self.create_file_name()?;

        let p = self.ad.params;
        let frame_type =
            FrameType::from_i32(self.get_i32(p.frame_type)).unwrap_or(FrameType::Normal);
        let acquire_time = self.ad.port_base.get_float64_param(p.acquire_time, 0)?;
        let num_darks = self.get_i32(self.params.num_darks);
        let file_name = self
            .ad
            .port_base
            .get_string_param(p.base.full_file_name, 0)?
            .to_string();

        self.ad
            .port_base
            .set_string_param(p.status_message, 0, "Starting exposure".into())?;
        self.ad.port_base.call_param_callbacks(0)?;

        let command = protocol::acquire(frame_type, &file_name, acquire_time, num_darks);
        self.command(&command, BIS_COMMAND_TIMEOUT)
    }

    /// Report whether the directory BIS is told to write into exists here too
    /// (`asynNDArrayDriver::checkPath`, which the C base class ran on every
    /// `FilePath` write).
    fn check_path(&mut self) -> AsynResult<()> {
        let p = self.ad.params.base;
        let path = self
            .ad
            .port_base
            .get_string_param(p.file_path, 0)?
            .to_string();
        let exists = !path.is_empty() && Path::new(&path).is_dir();
        self.ad
            .port_base
            .set_int32_param(p.file_path_exists, 0, exists as i32)?;
        Ok(())
    }

    /// BIS drives the shutter itself when the shutter is wired to the detector
    /// (C `BISDetector::setShutter`).
    fn set_shutter(&mut self, open: bool) -> AsynResult<()> {
        let p = self.ad.params;
        if ShutterMode::from_i32(self.get_i32(p.shutter_mode)) != Some(ShutterMode::DetectorOnly) {
            return self.ad.set_shutter(open);
        }

        let open_delay = self
            .ad
            .port_base
            .get_float64_param(p.shutter_open_delay, 0)?;
        let close_delay = self
            .ad
            .port_base
            .get_float64_param(p.shutter_close_delay, 0)?;
        self.command(&protocol::shutter(open), BIS_COMMAND_TIMEOUT)?;

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
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
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
        self.ad
            .port_base
            .params
            .set_int32(reason, user.addr, value)?;

        let p = self.ad.params;
        let q = self.params;
        let mut status = Ok(());

        if reason == p.acquire {
            let idle = self.get_i32(p.status) == ADStatus::Idle as i32;
            if value != 0 && idle {
                // The actor, not the task, leaves Idle: two Acquire writes in a
                // row cannot queue two acquisitions.
                self.ad
                    .port_base
                    .set_int32_param(p.status, 0, ADStatus::Acquire as i32)?;
                if self.commands.try_send(TaskCommand::Start).is_err() {
                    log::error!("bruker: the acquisition task is not running");
                }
            } else if value == 0 && !idle {
                self.shared.stop_requested.store(true, Ordering::Release);
            }
        } else if reason == p.bin_x {
            if matches!(value, 1 | 2 | 4 | 8) {
                let max_size_x = self.get_i32(p.max_size_x);
                // There is one binning: X and Y are the same.
                self.ad.port_base.set_int32_param(p.bin_y, 0, value)?;
                self.command(
                    &protocol::change_frame_size(max_size_x / value),
                    BIS_DEFAULT_TIMEOUT,
                )?;
            } else {
                log::error!("bruker: binning {value} is not 1, 2, 4 or 8");
                status = Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!("binning {value} is not 1, 2, 4 or 8"),
                });
            }
        } else if reason == p.shutter_control {
            self.set_shutter(value != 0)?;
        } else if reason == q.start_scan {
            self.start_scan()?;
        } else if reason == q.epics_shutter {
            // C called `ADDriver::setShutter` from the acquisition task, going
            // past its own override on purpose: in EPICS shutter mode the
            // shutter is not BIS's to drive.
            self.ad.set_shutter(value != 0)?;
        } else {
            self.ad.write_int32_pool(reason, value)?;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        status
    }

    fn write_octet(&mut self, user: &mut AsynUser, value: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let text = String::from_utf8_lossy(value)
            .trim_end_matches('\0')
            .to_string();
        self.ad
            .port_base
            .params
            .set_string(reason, user.addr, text)?;

        if reason == self.ad.params.base.file_path {
            self.check_path()?;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(value.len())
    }
}

impl ADDriver for BrukerDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

/// What the IOC layer holds on to.
pub struct BrukerRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub params: BrukerParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    #[allow(dead_code)]
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl BrukerRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.array_output.lock().add(sender);
    }
}

/// Create the detector port and start its two threads (C `BISDetectorConfig`).
///
/// `command_handle` and `status_handle` are the two `drvAsynIPPort`s that reach
/// BIS: one to send commands on and one BIS broadcasts its status on.
pub fn create_bruker_detector(
    port_name: &str,
    command_handle: PortHandle,
    status_handle: PortHandle,
    max_memory: usize,
) -> AsynResult<BrukerRuntime> {
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

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let queued_counter = Arc::new(QueuedArrayCounter::new());

    let threads = vec![
        task::start_acquisition_task(AcquisitionContext {
            handle: runtime_handle.port_handle().clone(),
            output: ArrayPublisher::new(array_output.clone()),
            queued: queued_counter.clone(),
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
        queued_counter,
        threads,
    })
}
