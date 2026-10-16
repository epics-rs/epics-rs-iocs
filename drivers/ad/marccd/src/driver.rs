//! The `PortDriver` half of the marCCD port: parameter writes and the
//! bookkeeping C does inline in `writeInt32` / `writeFloat64` / `writeOctet`.
//!
//! marServer I/O is not done here — a `PortDriver` method runs inside the port
//! actor, whose runtime cannot block on a second port — so each branch either
//! signals a worker event directly (start/stop) or enqueues a [`Cmd`] for the
//! `MarccdCmdTask`.

use std::sync::Arc;
use std::sync::atomic::AtomicU64;

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode, ShutterMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
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
        let mut ad = ADDriverBase::new(port_name, 1, 0, 0, max_memory, ASYN_CANBLOCK)?;
        let p = MarccdParams::create(&mut ad.port_base)?;

        let params = ad.params;
        let base = &mut ad.port_base;
        // C constructor defaults (marCCD.cpp:1563-1574).
        base.set_string_param(params.base.manufacturer, 0, "MAR")?;
        base.set_string_param(params.base.model, 0, "CCD")?;
        base.set_int32_param(params.base.data_type, 0, NDDataType::Int16 as i32)?;
        base.set_int32_param(params.image_mode, 0, ImageMode::Single as i32)?;
        base.set_int32_param(params.trigger_mode, 0, TriggerMode::Internal as i32)?;
        base.set_float64_param(params.acquire_time, 0, 1.0)?;
        base.set_float64_param(params.acquire_period, 0, 0.0)?;
        base.set_int32_param(params.num_images, 0, 1)?;
        base.set_int32_param(p.overlap, 0, 0)?;
        base.set_string_param(params.base.driver_version, 0, DRIVER_VERSION)?;
        base.set_float64_param(p.tiff_timeout, 0, 20.0)?;

        Ok(Self {
            ad,
            p,
            cmd_tx: links.cmd_tx,
            stop: links.stop,
        })
    }

    fn send(&self, cmd: Cmd) {
        let _ = self.cmd_tx.try_send(cmd);
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
                .set_int32_param(function, user.addr, value)?;
        }

        let mut status = Ok(());
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
            status = ADDriver::ad_write_int32(self, user, value);
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "marCCD:writeInt32: error, status={} function={function}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("marCCD:writeInt32: function={function}, value={value}"),
            ),
        }
        status
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let function = user.reason;
        let p = self.p;

        self.ad
            .port_base
            .set_float64_param(function, user.addr, value)?;

        if function == p.stability {
            self.send(Cmd::SetStability(value));
        }
        // Parameters below FIRST_MARCCD_PARAM need no extra work beyond the
        // `set_float64` above, which is all `ADDriver::writeFloat64` does.

        let _ = self.ad.port_base.call_param_callbacks(0);
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("marCCD:writeFloat64: function={function}, value={value:.6}"),
        );
        Ok(())
    }

    /// C inherits `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.ad.write_octet(user, data)
    }
}

impl ADDriver for MarccdDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }

    /// `marCCD::setShutter` (marCCD.cpp:723-748): only the detector branch
    /// talks to marServer, so only it runs on the worker; every other mode is
    /// `ADDriver::setShutter` inside this write.
    fn set_shutter(&mut self, open: bool) -> AsynResult<()> {
        let mode = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.shutter_mode, 0)
            .unwrap_or(0);
        if ShutterMode::from_i32(mode) == Some(ShutterMode::DetectorOnly) {
            self.send(Cmd::SetShutter(open));
            return Ok(());
        }
        self.ad.set_shutter(open)
    }
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

pub struct MarccdRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub marccd_params: MarccdParams,
    pool: DriverPool,
    array_output: Arc<Mutex<NDArrayOutput>>,
    /// Worker threads; kept alive for the IOC's lifetime.
    pub tasks: Vec<std::thread::JoinHandle<()>>,
}

impl MarccdRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
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
    claim: &PortClaim,
    server_port: &str,
    max_memory: usize,
) -> Result<MarccdRuntime, String> {
    let port_name = claim.name();
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

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the marCCD port runtime: {e}"))?;
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(Mutex::new(NDArrayOutput::new()));

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
        pool: pool.clone(),
    };
    let image_worker = Worker {
        server: server.clone(),
        start: start.clone(),
        stop: stop.clone(),
        image_event: image_ev.clone(),
        acq_start: acq_start.clone(),
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
    };
    let cmd_worker = Worker {
        server,
        start,
        stop,
        image_event: image_ev,
        acq_start,
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
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
        tasks,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::time::{Duration, Instant};

    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

    /// A marServer that answers each command with `reply(command)` and logs
    /// what it was sent.
    fn fake_server(name: &str, reply: fn(&str) -> &'static str) -> Arc<Mutex<Vec<String>>> {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let log = Arc::new(Mutex::new(Vec::new()));
        let seen = log.clone();
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = [0u8; 512];
            while let Ok(n) = stream.read(&mut buf) {
                if n == 0 {
                    break;
                }
                let command = String::from_utf8_lossy(&buf[..n]).into_owned();
                let answer = reply(&command);
                seen.lock().push(command);
                if !answer.is_empty() && stream.write_all(answer.as_bytes()).is_err() {
                    break;
                }
            }
        });
        let ip = format!("{name}_IP");
        let claim = claim_port(&ip).unwrap();
        let (runtime, _) = create_port_runtime(
            &claim,
            DrvAsynIPPort::new(&ip, &addr).unwrap(),
            RuntimeConfig::default(),
        )
        .unwrap();
        claim
            .publish(runtime.port_handle().clone(), &InterruptGate::opened())
            .unwrap();
        std::mem::forget(runtime);
        log
    }

    /// Build the detector on `name`'s fake server, then wait for a `set_bin`
    /// queued behind the constructor's Init to reach it.
    fn after_init(name: &str, reply: fn(&str) -> &'static str) -> (MarccdRuntime, Vec<String>) {
        let log = fake_server(name, reply);
        let det =
            create_marccd_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        det.port_handle()
            .write_int32_blocking(det.ad_params.bin_x, 0, 1)
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(20);
        while !log.lock().iter().any(|c| c.starts_with("set_bin")) {
            assert!(Instant::now() < deadline, "set_bin never arrived");
            std::thread::sleep(Duration::from_millis(10));
        }
        let sent = log.lock().clone();
        (det, sent)
    }

    /// ADDriver.cpp:96-98 calls the virtual setShutter, which marCCD
    /// overrides to drive a detector shutter through marServer (:723-729).
    #[test]
    fn shutter_control_reaches_marccds_set_shutter() {
        let name = "MARCCD_SHUTTER";
        let log = fake_server(name, |c| if c.starts_with("get_") { "0" } else { "" });
        let det =
            create_marccd_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let (h, ad) = (det.port_handle(), det.ad_params);
        h.write_int32_blocking(ad.shutter_mode, 0, ShutterMode::DetectorOnly as i32)
            .unwrap();
        h.write_int32_blocking(ad.shutter_control, 0, 1).unwrap();
        let deadline = Instant::now() + Duration::from_secs(20);
        while !log.lock().iter().any(|c| c.starts_with("shutter,1")) {
            assert!(Instant::now() < deadline, "shutter,1 never arrived");
            std::thread::sleep(Duration::from_millis(10));
        }
        while h.read_int32_blocking(ad.shutter_status, 0).unwrap() != 1 {
            assert!(Instant::now() < deadline, "ADShutterStatus stayed closed");
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// A non-detector mode is ADDriver::setShutter inside the write
    /// (ADDriver.cpp:43-47), with no worker to reach.
    #[test]
    fn an_epics_shutter_is_set_inside_the_write() {
        let mut d = MarccdDriver::new(
            "MARCCD_EPICS_SHUTTER",
            0,
            DriverLinks {
                cmd_tx: rt::command_channel::<Cmd>(1).0,
                stop: Arc::new(Event::new()),
            },
        )
        .unwrap();
        let ad = d.ad.params;
        d.ad.port_base
            .set_int32_param(ad.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        d.ad_write_int32(&AsynUser::new(ad.shutter_control).with_addr(0), 1)
            .unwrap();
        assert_eq!(
            d.ad.port_base
                .get_int32_param(ad.shutter_control_epics, 0)
                .unwrap(),
            1
        );
    }

    #[test]
    fn a_failed_get_server_mode_ends_the_init() {
        let (_det, sent) = after_init("MARCCD_BAD_MODE", |_| "7");
        assert_eq!(sent[0], "get_mode");
        assert!(sent[1].starts_with("set_bin"), "{sent:?}");
    }

    #[test]
    fn the_raw_buffer_is_charged_after_get_config() {
        let (det, _) = after_init("MARCCD_RAW", |c| {
            if c.starts_with("get_mode") {
                "1"
            } else if c.starts_with("get_size") {
                "64,32"
            } else if c.starts_with("get_bin") {
                "2,2"
            } else if c.starts_with("get_") {
                "0"
            } else {
                ""
            }
        });
        assert_eq!(det.pool().num_alloc_buffers(), 1);
        assert_eq!(det.pool().allocated_bytes(), 128 * 64 * 2);
    }
}
