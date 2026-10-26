//! The single `mar345Task` worker thread and the acquisition state machine
//! (`erase` / `changeMode` / `acquireFrame` / `getImageData`).
//!
//! `mar345.cpp` runs all of this on one `mar345Task` thread under the driver
//! lock, woken by the start event and told what to do by the shared `mode`.
//! The worker holds the lock (`Held`) over exactly C's spans, releasing it
//! only where C calls `unlock()`: the start event, the exposure poll, the
//! acquire-period wait and each `readServer` read.

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::driver::ndarray_driver::ArrayOwner;
use epics_rs::ad_core::driver::{ADDriver, ImageMode};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::port_handle::{DriverHold, PortHandle};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

use crate::driver::Mar345Driver;
use crate::params::Mar345Params;
use crate::pck::get_pck;
use crate::protocol::{
    CMD_ERASE, DONE_MODE_CHANGE, DONE_SCAN_DATA, cmd_change, cmd_scan, full_file_name,
    response_done,
};
use crate::types::{
    EraseMode, Event, IMAGE_SIZES, MAR345_COMMAND_TIMEOUT, MAR345_POLL_DELAY, MAX_FILENAME_LEN,
    MAX_MESSAGE_SIZE, Mode, Status, secs,
};

/// C `imageSizes[res][size]` with C's out-of-bounds index made safe: an
/// out-of-range `res`/`size` (impossible via the template's mbbo records) yields
/// 0 rather than reading past the table.
fn image_size(res: i32, size: i32) -> i32 {
    IMAGE_SIZES
        .get(res.max(0) as usize)
        .and_then(|row| row.get(size.max(0) as usize))
        .copied()
        .unwrap_or(0)
}

/// The driver lock the worker holds — C `lock()` … `unlock()`.
pub(crate) type Held = DriverHold<Mar345Driver>;

/// [`DriverHold::run`] for a thread that carries on past a stopped port:
/// the step yields `R::default()`.
pub(crate) trait Step {
    fn step<R: Default + Send + 'static>(
        &mut self,
        f: impl FnOnce(&mut Mar345Driver) -> R + Send + 'static,
    ) -> R;
}

impl Step for Held {
    fn step<R: Default + Send + 'static>(
        &mut self,
        f: impl FnOnce(&mut Mar345Driver) -> R + Send + 'static,
    ) -> R {
        self.run(f).unwrap_or_default()
    }
}

/// Everything the `mar345Task` worker needs.
pub struct Worker {
    /// The mar345 port, whose actor is the driver lock.
    pub port: PortHandle,
    /// C `pasynUserServer`, shared with the driver.
    pub server_port: Arc<SyncIOHandle>,
    pub ad: ADBaseParams,
    pub p: Mar345Params,
    /// C `fromServer`.
    pub from: String,
    pub start: Arc<Event>,
    pub stop: Arc<Event>,
    pub abort: Arc<Event>,
    /// C `mar345Mode_t mode`, shared with the driver (set in `writeInt32`, read
    /// and reset here).
    pub mode: Arc<AtomicI32>,
    /// C `acqStartTime` as an f64 epoch second count (bits).
    pub acq_start: Arc<AtomicU64>,
    pub output: ArrayPublisher,
    pub pool: DriverPool,
    /// The driver's attribute set, for C `getAttributes`.
    pub owner: Arc<ArrayOwner>,
}

/// Current time as an f64 epoch second count (C `secPastEpoch + nsec/1e9`).
fn now_epoch_f64() -> f64 {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_secs_f64(),
        Err(e) => -(e.duration().as_secs_f64()),
    }
}

impl Worker {
    fn store_acq_start(&self, v: f64) {
        self.acq_start.store(v.to_bits(), Ordering::Relaxed);
    }

    fn load_acq_start(&self) -> f64 {
        f64::from_bits(self.acq_start.load(Ordering::Relaxed))
    }

    /// C `asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, ...)`.
    fn trace_error(&self, message: &str) {
        self.port.trace().output_device(
            self.port.port_name(),
            Some(0),
            0,
            TraceMask::ERROR,
            message,
        );
    }

    // -----------------------------------------------------------------------
    // marServer reads
    // -----------------------------------------------------------------------

    /// C `readServer`: a single read with the given timeout, the lock
    /// released around it. Any non-empty read publishes
    /// `ADStringFromServer`, a failed one too (C returns before
    /// `setStringParam` only when `nread == 0`).
    fn read_server(&mut self, l: &mut Held, timeout: f64) -> AsynStatus {
        let read = l.unlocked(|| {
            self.server_port
                .with_timeout(secs(timeout))
                .read_octet(0, MAX_MESSAGE_SIZE)
        });
        let (data, status) = read.into_parts();
        let status = status.map_or_else(|e| e.status(), |()| AsynStatus::Success);
        if data.is_empty() {
            self.from.clear();
            return status;
        }
        self.from = String::from_utf8_lossy(&data).into_owned();
        if status != AsynStatus::Success {
            self.server_port.print(
                0,
                TraceMask::ERROR,
                format_args!(
                    "mar345:readServer, timeout={timeout:.6}, status={} received {} bytes\n{}\n",
                    status as i32,
                    data.len(),
                    self.from,
                    timeout = CFloat(timeout)
                ),
            );
        }
        let (reason, from) = (self.ad.string_from_server, self.from.clone());
        l.step(move |d| {
            d.set_str(reason, &from);
            d.callbacks();
        });
        status
    }

    /// C `waitForCompletion`: poll `readServer` at `MAR345_POLL_DELAY` until the
    /// reply contains `done`, or `timeout` elapses.
    fn wait_for_completion(&mut self, l: &mut Held, done: &str, timeout: f64) -> AsynStatus {
        let start = Instant::now();
        loop {
            let status = self.read_server(l, MAR345_POLL_DELAY);
            if status == AsynStatus::Success && response_done(&self.from, done) {
                return AsynStatus::Success;
            }
            if start.elapsed().as_secs_f64() > timeout {
                self.trace_error(
                    "mar345:waitForCompletion: error waiting for response from marServer\n",
                );
                return AsynStatus::Error;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Slow tasks
    // -----------------------------------------------------------------------

    /// C `erase`.
    fn erase(&mut self, l: &mut Held) -> AsynStatus {
        let (ad, p) = (self.ad, self.p);
        let num_erase = l.step(move |d| {
            let num_erase = d.get_i32(p.num_erase).max(1);
            d.set_i32(ad.status, Status::Erase as i32);
            d.set_i32(p.num_erased, 0);
            d.callbacks();
            num_erase
        });

        let mut status = AsynStatus::Success;
        for i in 0..num_erase {
            if self.abort.try_wait() {
                status = AsynStatus::Error;
                break;
            }
            l.step(|d| {
                d.write_server(CMD_ERASE);
            });
            status = self.wait_for_completion(l, DONE_SCAN_DATA, MAR345_COMMAND_TIMEOUT);
            if status != AsynStatus::Success {
                break;
            }
            l.step(move |d| {
                d.set_i32(p.num_erased, i + 1);
                d.callbacks();
            });
        }

        l.step(move |d| {
            d.set_i32(ad.status, Status::Idle as i32);
            d.set_i32(p.erase, 0);
            d.callbacks();
        });
        status
    }

    /// C `changeMode`.
    fn change_mode(&mut self, l: &mut Held) -> AsynStatus {
        let (ad, p) = (self.ad, self.p);
        l.step(move |d| {
            d.set_i32(ad.status, Status::ChangeMode as i32);
            d.callbacks();
            let size_x = image_size(d.get_i32(p.res), d.get_i32(p.size));
            d.set_i32(ad.base.array_size_x, size_x);
            d.set_i32(ad.base.array_size_y, size_x);
            d.set_i32(
                ad.base.array_size,
                size_x.wrapping_mul(size_x).wrapping_mul(2),
            );
            d.write_server(&cmd_change(size_x));
        });
        let status = self.wait_for_completion(l, DONE_MODE_CHANGE, MAR345_COMMAND_TIMEOUT);

        l.step(move |d| {
            d.set_i32(ad.status, Status::Idle as i32);
            d.set_i32(p.change_mode, 0);
            d.callbacks();
        });
        status
    }

    /// C `acquireFrame`. Returns [`AsynStatus::Error`] on an abort or a failed
    /// erase / scan (C's `return asynError` paths).
    async fn acquire_frame(&mut self, l: &mut Held) -> AsynStatus {
        let (ad, p) = (self.ad, self.p);

        // C stores acqStartTime here (used later for the NDArray timestamp).
        self.store_acq_start(now_epoch_f64());
        let (acquire_time, use_shutter, array_callbacks, erase_mode, full) = l.step(move |d| {
            let acquire_time = d.get_f64(ad.acquire_time);
            // ADShutterModeNone == 0.
            let use_shutter = d.get_i32(ad.shutter_mode) != 0;
            let size = d.get_i32(p.size);
            let res = d.get_i32(p.res);
            let array_callbacks = d.get_i32(ad.base.array_callbacks);
            let erase_mode = d.get_i32(p.erase_mode);
            let full = full_file_name(&d.create_file_name(), image_size(res, size));
            (acquire_time, use_shutter, array_callbacks, erase_mode, full)
        });

        // Erase before exposure if set.
        if erase_mode == EraseMode::Before as i32 {
            let status = self.erase(l);
            if status != AsynStatus::Success {
                return status;
            }
        }

        // Start the exposure. C arms an epicsTimer that signals the stop event
        // when the exposure time expires; this port folds that deadline into the
        // wait loop below. `start` is the timeRemaining reference (C `startTime`,
        // captured before the shutter opens).
        let start = Instant::now();
        l.step(move |d| {
            if use_shutter {
                ADDriver::set_shutter(d, true);
            }
            d.set_i32(ad.status, Status::Expose as i32);
            d.callbacks();
        });
        let deadline = Instant::now() + secs(acquire_time);

        let mut status = AsynStatus::Success;
        loop {
            if self.abort.try_wait() {
                status = AsynStatus::Error;
                break;
            }
            let now = Instant::now();
            if now >= deadline {
                // The exposure timer would have fired the stop event.
                break;
            }
            let wait = (deadline - now).min(secs(MAR345_POLL_DELAY));
            if l.unlocked(|| self.stop.wait_timeout(wait)) {
                // Acquisition stopped before the time was complete (client set
                // ADAcquire=0). This ends the exposure but is not an abort.
                break;
            }
            let time_remaining = (acquire_time - start.elapsed().as_secs_f64()).max(0.0);
            l.step(move |d| {
                d.set_f64(ad.time_remaining, time_remaining);
                d.callbacks();
            });
        }

        let scan = l.step(move |d| {
            d.set_f64(ad.time_remaining, 0.0);
            if use_shutter {
                ADDriver::set_shutter(d, false);
            }
            d.set_i32(ad.status, Status::Idle as i32);
            d.callbacks();
            // If the exposure was aborted return error.
            if status != AsynStatus::Success {
                return false;
            }
            d.set_i32(ad.status, Status::Scan as i32);
            d.callbacks();
            d.set_str(ad.base.full_file_name, &full);
            d.callbacks();
            d.write_server(&cmd_scan(&full));
            true
        });
        if !scan {
            return AsynStatus::Error;
        }
        let status = self.wait_for_completion(l, DONE_SCAN_DATA, MAR345_COMMAND_TIMEOUT);
        if status != AsynStatus::Success {
            return AsynStatus::Error;
        }

        l.step(move |d| {
            let image_counter = d.get_i32(ad.base.array_counter) + 1;
            d.set_i32(ad.base.array_counter, image_counter);
            d.callbacks();
        });

        // If arrayCallbacks is set then read the file back in.
        if array_callbacks != 0 {
            self.get_image_data(l).await;
        }

        // Erase after scanning if set.
        if erase_mode == EraseMode::After as i32 {
            return self.erase(l);
        }
        AsynStatus::Success
    }

    /// C `getImageData` — read the `.mar` file, decode the pck stream, publish.
    async fn get_image_data(&mut self, l: &mut Held) {
        let ad = self.ad;
        let base = ad.base;
        let (full, nx, ny, image_counter) = l.step(move |d| {
            let full = d.get_str(base.full_file_name, MAX_FILENAME_LEN);
            let nx = d.get_i32(base.array_size_x).max(0) as usize;
            let ny = d.get_i32(base.array_size_y).max(0) as usize;
            let image_counter = d.get_i32(base.array_counter);
            d.set_str(ad.status_message, &format!("Reading mar345 file {full}"));
            d.callbacks();
            (full, nx, ny, image_counter)
        });

        let data = match std::fs::read(&full) {
            Ok(d) => d,
            Err(e) => {
                // C's "%s%s:" has no colon between driver and function.
                let error = e.to_string();
                let error = error.split(" (os error").next().unwrap_or(&error);
                self.trace_error(&format!(
                    "mar345getImageData: unable to open input file {full}, error={error}\n"
                ));
                return;
            }
        };

        let pixels = get_pck(&data, nx, ny);

        // C dereferences a refused alloc (mar345.cpp:194); the frame is
        // dropped after the pool's reached-limit line.
        let Ok(mut array) = self.pool.alloc(
            vec![NDDimension::new(nx), NDDimension::new(ny)],
            NDDataType::UInt16,
        ) else {
            return;
        };
        let mut frame = array.fill();
        let out = frame
            .data_mut()
            .as_mut_slice::<u16>()
            .expect("allocated as NDUInt16");
        for (o, &v) in out.iter_mut().zip(&pixels) {
            *o = v as u16;
        }
        array.unique_id = image_counter;
        // C sets the double timeStamp from acqStartTime and the epicsTS from the
        // port's updateTimeStamp (mar345.cpp:212-213).
        array.time_stamp = self.load_acq_start();
        array.timestamp = l.step(|d| EpicsTimestamp::from(d.ad.port_base.update_time_stamp()));
        // C `getAttributes`, evaluated where its PARAM values live.
        let owner = self.owner.clone();
        let fresh = l.step(move |d| {
            let mut attributes = owner.attributes().lock();
            (!attributes.list.is_empty()).then(|| attributes.update(d.base()))
        });
        if let Some(fresh) = fresh {
            array.attributes.copy_from(&fresh);
        }
        self.output.publish(array).await;
    }

    // -----------------------------------------------------------------------
    // Acquisition loop
    // -----------------------------------------------------------------------

    /// C `mar345Task`'s `mar345ModeAcquire` case.
    async fn run_acquire(&mut self, l: &mut Held) {
        let ad = self.ad;

        let (image_mode, num_images) =
            l.step(move |d| (d.get_i32(ad.image_mode), d.get_i32(ad.num_images)));
        let num_images = if image_mode == ImageMode::Single as i32 {
            1
        } else {
            num_images.max(1)
        };
        let continuous = image_mode == ImageMode::Continuous as i32;

        let mut counter = 0;
        while counter < num_images || continuous {
            if self.abort.try_wait() {
                break;
            }
            l.step(move |d| {
                d.set_i32(ad.num_images_counter, counter);
                d.callbacks();
            });
            let status = self.acquire_frame(l).await;
            if status != AsynStatus::Success {
                break;
            }
            // We get out of the loop in single-shot mode or if acquire was set
            // to 0 by the client. Otherwise (continuous or multiple mode) sleep
            // until the acquire period expires or acquire is set to stop.
            let acq_start = self.load_acq_start();
            let delay_time = l.step(move |d| {
                if image_mode == ImageMode::Single as i32 {
                    d.set_i32(ad.acquire, 0);
                }
                if d.get_i32(ad.acquire) == 0 {
                    return None;
                }
                let elapsed = now_epoch_f64() - acq_start;
                let delay_time = d.get_f64(ad.acquire_period) - elapsed;
                if delay_time > 0.0 {
                    d.set_i32(ad.status, Status::Waiting as i32);
                    d.callbacks();
                }
                Some(delay_time)
            });
            let Some(delay_time) = delay_time else {
                break;
            };
            if delay_time > 0.0 && l.unlocked(|| self.abort.wait_timeout(secs(delay_time))) {
                break;
            }
            counter += 1;
        }

        self.mode.store(Mode::Idle as i32, Ordering::SeqCst);
        l.step(move |d| {
            d.set_i32(ad.acquire, 0);
            d.set_i32(ad.status, Status::Idle as i32);
        });
    }

    /// C `mar345Task` main loop.
    async fn run(mut self) {
        let ad = self.ad;
        let mut l = match self.port.hold_driver_blocking::<Mar345Driver>() {
            Ok(l) => l,
            Err(e) => {
                self.trace_error(&format!("mar345:mar345Task: no driver lock: {e}\n"));
                return;
            }
        };
        loop {
            l.step(move |d| {
                d.set_str(ad.status_message, "Waiting for event");
                d.callbacks();
            });
            // Release the lock while we wait for an event that says acquire
            // has started, then lock again.
            l.unlocked(|| self.start.wait());

            match Mode::from_i32(self.mode.load(Ordering::SeqCst)) {
                Some(Mode::Erase) => {
                    self.erase(&mut l);
                    self.mode.store(Mode::Idle as i32, Ordering::SeqCst);
                }
                Some(Mode::Acquire) => {
                    self.run_acquire(&mut l).await;
                }
                Some(Mode::Change) => {
                    self.change_mode(&mut l);
                    self.mode.store(Mode::Idle as i32, Ordering::SeqCst);
                }
                // Idle or unknown: nothing to do.
                _ => {}
            }

            l.step(|d| d.callbacks());
        }
    }
}

/// Spawn the `mar345Task` worker thread.
pub(crate) fn start_task(w: Worker) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("mar345Task", move || w.run())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::{Mar345Driver, create_mar345_detector, tests::fake_server};
    use epics_rs::ad_core::attributes::NDAttrValue;
    use epics_rs::ad_core::plugin::channel::ndarray_channel;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::user::AsynUser;

    /// mar345.cpp:487-503 holds the lock from setShutter(0)'s close delay
    /// through NDFullFileName and writeServer("COMMAND SCAN"), releasing it
    /// only in readServer: a client write issued during the delay lands
    /// after the driver's own NDFullFileName.
    #[test]
    fn a_client_write_waits_out_the_exposure_end_span() {
        use epics_rs::ad_core::driver::ShutterMode;
        use epics_rs::asyn::request::RequestOp;
        use epics_rs::asyn::user::AsynUser;

        let name = "MAR345_SPAN";
        let log = fake_server(name);
        let det =
            create_mar345_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let (h, ad, p) = (det.port_handle().clone(), det.ad_params, det.mar345_params);
        h.write_int32_blocking(ad.shutter_mode, 0, ShutterMode::DetectorOnly as i32)
            .unwrap();
        h.write_float64_blocking(ad.shutter_close_delay, 0, 0.5)
            .unwrap();
        h.write_float64_blocking(ad.acquire_time, 0, 0.05).unwrap();
        h.write_int32_blocking(p.erase_mode, 0, crate::types::EraseMode::None as i32)
            .unwrap();
        h.write_int32_blocking(ad.base.array_callbacks, 0, 0)
            .unwrap();
        h.write_int32_blocking(ad.acquire, 0, 1).unwrap();

        let wait_for = |text: &str| {
            let deadline = Instant::now() + std::time::Duration::from_secs(20);
            while !log.lock().contains(text) {
                assert!(Instant::now() < deadline, "{text} never reached marServer");
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        };
        wait_for("COMMAND SHUTTER CLOSE");
        h.submit_blocking(
            RequestOp::OctetWrite {
                data: b"client".to_vec(),
            },
            AsynUser::new(ad.base.full_file_name).with_addr(0),
        )
        .unwrap();
        wait_for("COMMAND SCAN");
        assert_eq!(
            h.get_string_param_blocking(ad.base.full_file_name, 0, 256)
                .unwrap(),
            b"client"
        );
        std::mem::forget(det);
    }

    /// A worker whose hold fails says so on the port trace instead of
    /// exiting silently.
    #[test]
    fn the_task_without_the_driver_lock_traces_and_exits() {
        use epics_rs::asyn::trace::TraceFile;

        let name = "MAR345_NOLOCK";
        fake_server(name);
        let det =
            create_mar345_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let h = det.port_handle().clone();
        // The marServer port: its driver is no Mar345Driver, so no hold.
        let ip = epics_rs::asyn::asyn_record::get_port(&format!("{name}_IP"))
            .unwrap()
            .handle
            .clone();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        ip.trace().set_trace_file(
            Some(ip.port_name()),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let worker = Worker {
            port: ip.clone(),
            server_port: Arc::new(crate::driver::tests::server_user(name)),
            ad: det.ad_params,
            p: det.mar345_params,
            from: String::new(),
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            abort: Arc::new(Event::new()),
            mode: Arc::new(AtomicI32::new(0)),
            acq_start: Arc::new(AtomicU64::new(0)),
            output: ArrayPublisher::new(det.array_output().clone()),
            pool: det.pool().clone(),
            owner: h
                .with_driver_blocking::<Mar345Driver, _>(|d| d.ad.owner.clone())
                .unwrap(),
        };
        start_task(worker).join().unwrap();
        ip.trace()
            .set_trace_file(Some(ip.port_name()), TraceFile::default());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.contains("mar345:mar345Task: no driver lock"),
            "{text:?}"
        );
        std::mem::forget(det);
    }

    /// A marServer port whose every read sends `abc`, then ends with `status`.
    struct SendsAbc(epics_rs::asyn::port::PortDriverBase, AsynStatus);

    impl epics_rs::asyn::port::PortDriver for SendsAbc {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.0
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
            buf[..3].copy_from_slice(b"abc");
            let status = match self.1 {
                AsynStatus::Success => Ok(()),
                status => Err(epics_rs::asyn::error::AsynError::Status {
                    status,
                    message: "refused".into(),
                }),
            };
            OctetReadResult {
                status,
                ..OctetReadResult::ok(3, EomReason::empty())
            }
        }
    }

    /// `readServer` with a 0.1 s timeout on a marServer port answering as
    /// [`SendsAbc`]: its status, `from`, `ADStringFromServer` and the port's
    /// TRACEIO_DEVICE output.
    fn read_server_on(name: &str, status: AsynStatus) -> (AsynStatus, String, Vec<u8>, String) {
        use epics_rs::asyn::port::{InterfaceMask, PortDriverBase, PortFlags};
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use epics_rs::asyn::trace::{TraceFile, TraceIoMask};
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;

        let ip = format!("{name}_IP");
        let claim = claim_port(&ip).unwrap();
        let (runtime, _) = create_port_runtime(
            &claim,
            SendsAbc(
                PortDriverBase::new(
                    &ip,
                    1,
                    PortFlags::default(),
                    InterfaceMask::UNMIGRATED_ALL,
                    InterfaceMask::UNMIGRATED_INTERRUPTS,
                ),
                status,
            ),
            RuntimeConfig::default(),
        )
        .unwrap();
        let path = std::env::temp_dir().join(format!("{ip}_{}", std::process::id()));
        let trace = runtime.port_handle().trace();
        trace.set_trace_file(
            Some(&ip),
            TraceFile::File(Arc::new(std::sync::Mutex::new(
                std::fs::File::create(&path).unwrap(),
            ))),
        );
        trace.set_trace_mask(Some(&ip), TraceMask::IO_DEVICE);
        trace.set_trace_io_mask(Some(&ip), TraceIoMask::ASCII);
        claim
            .publish(runtime.port_handle().clone(), &InterruptGate::opened())
            .unwrap();
        std::mem::forget(runtime);
        let det = create_mar345_detector(&claim_port(name).unwrap(), &ip, 0).unwrap();
        let (h, ad) = (det.port_handle().clone(), det.ad_params);
        let mut worker = Worker {
            port: h.clone(),
            server_port: Arc::new(crate::driver::tests::server_user(name)),
            ad,
            p: det.mar345_params,
            from: String::new(),
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            abort: Arc::new(Event::new()),
            mode: Arc::new(AtomicI32::new(0)),
            acq_start: Arc::new(AtomicU64::new(0)),
            output: ArrayPublisher::new(det.array_output().clone()),
            pool: det.pool().clone(),
            owner: h
                .with_driver_blocking::<Mar345Driver, _>(|d| d.ad.owner.clone())
                .unwrap(),
        };
        let status = {
            let mut l = worker.port.clone().hold_driver_blocking().unwrap();
            worker.read_server(&mut l, 0.1)
        };
        let from_server = h
            .get_string_param_blocking(ad.string_from_server, 0, 256)
            .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        std::mem::forget(det);
        (status, worker.from, from_server, text)
    }

    /// C `readServer` (mar345.cpp:264-269) publishes whatever a timed-out
    /// read transferred, and returns the timeout; `pasynOctetSyncIO->read`
    /// prints no read line for it (asynOctetSyncIO.c:219).
    #[test]
    fn a_timed_out_read_publishes_its_bytes() {
        let (status, from, from_server, text) =
            read_server_on("MAR345_PARTIAL", AsynStatus::Timeout);
        assert_eq!(status, AsynStatus::Timeout);
        assert_eq!(from, "abc");
        assert_eq!(from_server, b"abc");
        assert!(!text.contains("asynOctetSyncIO read"), "{text}");
    }

    /// `readServer` is `pasynOctetSyncIO->read` (mar345.cpp:261), whose
    /// success prints its TRACEIO_DEVICE line (asynOctetSyncIO.c:219-223).
    #[test]
    fn a_read_prints_its_sync_io_line() {
        let (status, _, _, text) = read_server_on("MAR345_READLINE", AsynStatus::Success);
        assert_eq!(status, AsynStatus::Success);
        assert!(text.contains("asynOctetSyncIO read:\nabc"), "{text}");
    }

    /// mar345.cpp:215-216 reads the driver's attributes into every frame.
    #[test]
    fn each_frame_carries_the_driver_attributes() {
        let name = "MAR345_ATTRS";
        fake_server(name);
        let det =
            create_mar345_detector(&claim_port(name).unwrap(), &format!("{name}_IP"), 0).unwrap();
        let (h, ad) = (det.port_handle().clone(), det.ad_params);
        let xml = r#"<Attributes>
            <Attribute name="Period" type="PARAM" source="ACQ_PERIOD" datatype="DOUBLE"/>
        </Attributes>"#;
        let file = std::env::temp_dir().join(format!("{name}_{}.mar", std::process::id()));
        std::fs::write(&file, b"").unwrap();
        let path = file.to_string_lossy().into_owned();
        h.write_int32_blocking(ad.base.array_size_x, 0, 4).unwrap();
        h.write_int32_blocking(ad.base.array_size_y, 0, 4).unwrap();
        h.write_float64_blocking(ad.acquire_period, 0, 2.5).unwrap();

        let (sender, mut receiver) = ndarray_channel(&format!("{name}_SINK"), 4);
        det.connect_downstream(sender);
        let worker = Worker {
            port: h.clone(),
            server_port: Arc::new(crate::driver::tests::server_user(name)),
            ad,
            p: det.mar345_params,
            from: String::new(),
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            abort: Arc::new(Event::new()),
            mode: Arc::new(AtomicI32::new(0)),
            acq_start: Arc::new(AtomicU64::new(0)),
            output: ArrayPublisher::new(det.array_output().clone()),
            pool: det.pool().clone(),
            owner: h
                .with_driver_blocking::<Mar345Driver, _>(|d| d.ad.owner.clone())
                .unwrap(),
        };
        let writer = h.clone();
        rt::run_thread(move || async move {
            let (h, mut worker) = (writer, worker);
            h.write_octet(ad.base.attributes_file, 0, xml.as_bytes().to_vec())
                .await
                .unwrap();
            h.write_octet(ad.base.full_file_name, 0, path.into_bytes())
                .await
                .unwrap();
            let mut l = worker.port.hold_driver_blocking().unwrap();
            worker.get_image_data(&mut l).await;
        })
        .join()
        .unwrap();
        let _ = std::fs::remove_file(&file);

        let frame = receiver.blocking_recv().unwrap();
        assert_eq!(
            frame.attributes.get("Period").unwrap().value,
            NDAttrValue::Float64(2.5)
        );
        std::mem::forget(det);
    }
}
