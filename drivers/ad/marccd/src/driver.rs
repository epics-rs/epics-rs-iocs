//! The `PortDriver` half of the marCCD port: parameter writes and the
//! bookkeeping C does inline in `writeInt32` / `writeFloat64` / `writeOctet`.
//!
//! marServer I/O is not done here — a `PortDriver` method runs inside the port
//! actor, whose runtime cannot block on a second port — so each branch either
//! signals a worker event directly (start/stop) or enqueues a [`Cmd`] for the
//! `MarccdCmdTask`.

use std::sync::Arc;
use std::sync::atomic::AtomicU64;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;
use parking_lot::Mutex;
use tokio::sync::Mutex as AsyncMutex;

use crate::params::MarccdParams;
use crate::server::Server;
use crate::task::{Cmd, Worker, start_cmd_task, start_det_task, start_image_task};
use crate::types::{Event, TriggerMode};

/// C `DRIVER_VERSION`.
const DRIVER_VERSION: &str = "2.3.0";

pub struct MarccdDriver {
    pub ad: ADDriverBase,
    pub p: MarccdParams,
    cmd_tx: rt::CommandSender<Cmd>,
    /// C `stopEventId`. Start signalling is routed through the command task (it
    /// needs a `getState` round-trip first), so the driver only holds `stop`.
    stop: Arc<Event>,
}

/// The handles the driver shares with its worker threads.
struct DriverLinks {
    cmd_tx: rt::CommandSender<Cmd>,
    stop: Arc<Event>,
}

impl MarccdDriver {
    fn new(port_name: &str, max_memory: usize, links: DriverLinks) -> AsynResult<Self> {
        // marCCDConfig passes no sensor size; C's ADDriver base leaves
        // ADMaxSizeX/Y at 0 and `getConfig` fills them from the server.
        let mut ad = ADDriverBase::new(port_name, 0, 0, max_memory)?;
        let p = MarccdParams::create(&mut ad.port_base)?;

        let params = ad.params;
        let base = &mut ad.port_base;
        // C constructor defaults (marCCD.cpp:1563-1574).
        base.set_string_param(params.base.manufacturer, 0, "MAR".into())?;
        base.set_string_param(params.base.model, 0, "CCD".into())?;
        base.set_int32_param(params.base.data_type, 0, NDDataType::Int16 as i32)?;
        base.set_int32_param(params.image_mode, 0, ImageMode::Single as i32)?;
        base.set_int32_param(params.trigger_mode, 0, TriggerMode::Internal as i32)?;
        base.set_float64_param(params.acquire_time, 0, 1.0)?;
        base.set_float64_param(params.acquire_period, 0, 0.0)?;
        base.set_int32_param(params.num_images, 0, 1)?;
        base.set_int32_param(p.overlap, 0, 0)?;
        base.set_string_param(params.base.driver_version, 0, DRIVER_VERSION.into())?;
        base.set_float64_param(p.tiff_timeout, 0, 20.0)?;

        Ok(Self {
            ad,
            p,
            cmd_tx: links.cmd_tx,
            stop: links.stop,
        })
    }

    fn send(&self, cmd: Cmd) {
        if let Err(e) = self.cmd_tx.try_send(cmd) {
            log::error!("marccd: command queue full or closed, dropped {:?}", e.0);
        }
    }
}

impl PortDriver for MarccdDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let function = user.reason;
        let params = self.ad.params;
        let p = self.p;

        // C: `getIntegerParam(ADAcquire, &acquiring)` (the value *before* the
        // set) then `setIntegerParam(function, value)`.
        let acquiring = self
            .ad
            .port_base
            .get_int32_param(params.acquire, 0)
            .unwrap_or(0);

        if function == params.acquire {
            self.ad.set_acquire(value)?;
        } else {
            self.ad
                .port_base
                .params
                .set_int32(function, user.addr, value)?;
        }

        if function == params.acquire {
            if value != 0 {
                // C calls getState here to gate the start; that server round-trip
                // and the start signalling run on the command task.
                self.send(Cmd::StartAcquire);
            } else if acquiring != 0 {
                // Stop needs no server I/O: signal the acquisition task directly.
                // The inline exposure deadline replaces C's epicsTimerCancel.
                self.stop.signal();
            }
        } else if function == params.bin_x || function == params.bin_y {
            self.send(Cmd::SetBin);
        } else if function == p.gate_mode {
            self.send(Cmd::SetGating(value));
        } else if function == p.readout_mode {
            self.send(Cmd::SetReadoutMode(value));
        } else if function == p.frame_shift {
            self.send(Cmd::SetFrameShift(value));
        } else if function == params.read_status {
            if value != 0 {
                self.send(Cmd::ReadStatus);
            }
        } else if function == params.base.write_file {
            self.send(Cmd::WriteFile);
        } else if function < p.first() {
            // Base-class parameter: pool controls plus plain stores.
            self.ad.write_int32_pool(function, value)?;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let function = user.reason;
        let p = self.p;

        self.ad
            .port_base
            .params
            .set_float64(function, user.addr, value)?;

        if function == p.stability {
            self.send(Cmd::SetStability(value));
        }
        // Parameters below FIRST_MARCCD_PARAM need no extra work beyond the
        // `set_float64` above, which is all `ADDriver::writeFloat64` does.

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let function = user.reason;

        // C receives a NUL-terminated `char *`; waveform records pad with NULs.
        let payload = data.split(|&b| b == 0).next().unwrap_or(&[]);
        let value = String::from_utf8_lossy(payload).into_owned();

        self.ad
            .port_base
            .params
            .set_string(function, user.addr, value)?;
        // marCCD does not override writeOctet; every octet parameter is simply
        // stored. (Deviation: the base `asynNDArrayDriver::writeOctet` side
        // effects — NDFilePathExists via checkPath, NDAttributesFile reload —
        // are not reproduced; see the crate docs.)

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(data.len())
    }
}

impl ADDriver for MarccdDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

pub struct MarccdRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub marccd_params: MarccdParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    /// Worker threads; kept alive for the IOC's lifetime.
    pub tasks: Vec<std::thread::JoinHandle<()>>,
}

impl MarccdRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.array_output.lock().add(sender);
    }
}

/// C `marCCDConfig`.
///
/// `server_port` must already exist (`drvAsynIPPortConfigure`), because C
/// connects to it with `pasynOctetSyncIO->connect` in the constructor. Unlike
/// the C constructor, `getServerMode` / `getConfig` / `getState` run
/// asynchronously on the command task (a `PortDriver` cannot block on the
/// server port from inside its own actor), enqueued as [`Cmd::Init`].
pub fn create_marccd_detector(
    port_name: &str,
    server_port: &str,
    max_memory: usize,
) -> Result<MarccdRuntime, String> {
    let server_entry = get_port(server_port).ok_or_else(|| {
        format!("marServer port '{server_port}' not found (call drvAsynIPPortConfigure first)")
    })?;
    let server_handle = server_entry.handle.clone();

    let (cmd_tx, cmd_rx) = rt::command_channel::<Cmd>(64);
    let start = Arc::new(Event::new());
    let stop = Arc::new(Event::new());

    let driver = MarccdDriver::new(
        port_name,
        max_memory,
        DriverLinks {
            cmd_tx: cmd_tx.clone(),
            stop: stop.clone(),
        },
    )
    .map_err(|e| format!("failed to create marCCD driver: {e}"))?;

    let ad_params = driver.ad.params;
    let marccd_params = driver.p;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(driver, RuntimeConfig::default());
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(Mutex::new(NDArrayOutput::new()));
    let queued_counter = Arc::new(QueuedArrayCounter::new());

    // The three worker contexts share one `Server` behind a `tokio::sync::Mutex`
    // — C's driver-lock analog serialising all marServer I/O.
    let server = Arc::new(AsyncMutex::new(Server::new(
        handle,
        server_handle,
        ad_params,
        marccd_params,
    )));
    let acq_start = Arc::new(AtomicU64::new(0));
    // `image_event` (C `imageEventId`) is signalled by the det task in overlap
    // mode and awaited by the image task; the other events/state are shared by
    // all three workers.
    let image_ev = Arc::new(Event::new());

    let det_worker = Worker {
        server: server.clone(),
        start: start.clone(),
        stop: stop.clone(),
        image_event: image_ev.clone(),
        acq_start: acq_start.clone(),
        output: ArrayPublisher::new(array_output.clone()),
    };
    let image_worker = Worker {
        server: server.clone(),
        start: start.clone(),
        stop: stop.clone(),
        image_event: image_ev.clone(),
        acq_start: acq_start.clone(),
        output: ArrayPublisher::new(array_output.clone()),
    };
    let cmd_worker = Worker {
        server,
        start,
        stop,
        image_event: image_ev,
        acq_start,
        output: ArrayPublisher::new(array_output.clone()),
    };

    let tasks = vec![
        start_cmd_task(cmd_worker, cmd_rx),
        start_det_task(det_worker),
        start_image_task(image_worker),
    ];

    // C runs getServerMode -> getConfig -> getState at the end of the
    // constructor; do the same on the command task now the workers are up.
    if cmd_tx.try_send(Cmd::Init).is_err() {
        return Err("marccd: command task did not start".to_string());
    }

    Ok(MarccdRuntime {
        runtime_handle,
        ad_params,
        marccd_params,
        pool,
        array_output,
        queued_counter,
        tasks,
    })
}
