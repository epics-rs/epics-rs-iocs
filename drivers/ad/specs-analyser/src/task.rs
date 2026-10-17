//! The persistent acquisition worker: `SpecsAnalyser::specsAnalyserTask`
//! (`specsAnalyser.cpp:268-706`). It holds the driver as C holds
//! `this->lock()`, releasing it only where C unlocks, so its exchanges and
//! parameter steps run on the driver ([`SpecsAnalyserDriver::with_wire`]).
//! It runs on its own runtime (`rt::run_thread_named`) because
//! `ArrayPublisher::publish` is async.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDimension};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::{DriverHold, PortHandle};

use crate::codec;
use crate::driver::SpecsAnalyserDriver;
use crate::params::SpecsParams;
use crate::types::{Event, MAX_VALUES, RunMode, UPDATE_RATE, secs};
use crate::wire::{ValidateResult, WireError, WireLink};

/// Everything the acquisition worker needs.
pub struct Worker {
    /// This driver's own port (parameter library).
    pub self_handle: PortHandle,
    pub p: SpecsParams,
    pub ad: ADDriverParams,
    /// C `startEventId_`.
    pub start: Arc<Event>,
    /// C `stopEventId_`.
    pub stop: Arc<Event>,
    pub output: ArrayPublisher,
    pub pool: DriverPool,
}

/// The running task, on its own thread: a hold is tied to the thread that
/// took it.
struct Task {
    w: Worker,
    /// C `this->lock()`; `None` once the port is gone.
    hold: Option<DriverHold<SpecsAnalyserDriver>>,
    /// C's `status` local, carried across outer-loop passes — used only to
    /// decide whether the wait branch may overwrite `ADStatusMessage`/
    /// `ADStatus` (`specsAnalyser.cpp:307-313`).
    last_status_ok: bool,
}

/// Current time as an f64 epoch second count (C `secPastEpoch + nsec/1e9`).
fn now_epoch_f64() -> f64 {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_secs_f64(),
        Err(e) => -(e.duration().as_secs_f64()),
    }
}

/// State carried from the preamble (`specsAnalyser.cpp:300-432`) into the
/// acquisition body (`specsAnalyser.cpp:441-704`). These are function-scope
/// locals in C (`specsAnalyser.cpp:270-294`) that persist across outer-loop
/// passes where `ADAcquire` was already 1 at the top of `while(1)`
/// (`specsAnalyser.cpp:301-303`) — e.g. `ImageMode::Continuous`, which never
/// resets `ADAcquire` between batches — so the preamble (and the
/// `start`/wait) is skipped and the previous pass's values are reused
/// verbatim; see `Task::run`.
#[derive(Clone, Copy)]
struct Prepared {
    iterations: i32,
    acquire: i32,
    non_energy_channels: i32,
    energy_channels: i32,
}

impl Prepared {
    /// The C locals' initial values, which an `ADAcquire` already 1 at the
    /// first pass runs with.
    const C_LOCALS: Prepared = Prepared {
        iterations: 1,
        acquire: 0,
        non_energy_channels: 0,
        energy_channels: 0,
    };
}

impl Task {
    // -----------------------------------------------------------------------
    // Parameter library (C `setIntegerParam`/`setDoubleParam`/`getIntegerParam`/
    // `getDoubleParam`/`callParamCallbacks`).
    // -----------------------------------------------------------------------

    /// `f` on the driver, inside the hold. A failed step drops the hold:
    /// the port is gone.
    fn driver<R: Send + 'static>(
        &mut self,
        f: impl FnOnce(&mut SpecsAnalyserDriver) -> R + Send + 'static,
    ) -> Result<R, WireError> {
        let result = match self.hold.as_mut() {
            Some(hold) => hold.run(f),
            None => Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "specsAnalyserTask holds no driver".into(),
            }),
        };
        if result.is_err() {
            self.hold = None;
        }
        result.map_err(WireError::Io)
    }

    /// C `this->unlock()` … `this->lock()` around `wait`.
    fn unlocked<R>(&mut self, wait: impl FnOnce() -> R) -> R {
        match self.hold.as_mut() {
            Some(hold) => hold.unlocked(wait),
            None => wait(),
        }
    }

    fn set(&mut self, reason: usize, value: ParamValue) {
        let _ = self.driver(move |d| {
            let _ = d.ad.port_base.set_param_value(reason, 0, value);
        });
    }

    fn set_i32(&mut self, reason: usize, value: i32) {
        self.set(reason, ParamValue::Int32(value));
    }

    fn set_f64(&mut self, reason: usize, value: f64) {
        self.set(reason, ParamValue::Float64(value));
    }

    fn set_str(&mut self, reason: usize, value: impl Into<Vec<u8>>) {
        self.set(reason, ParamValue::Octet(value.into().into()));
    }

    fn set_f64_array(&mut self, reason: usize, value: Vec<f64>) {
        self.set(reason, ParamValue::Float64Array(value.into()));
    }

    fn callbacks(&mut self) {
        let _ = self.driver(|d| {
            let _ = d.ad.port_base.call_param_callbacks(0);
        });
    }

    fn get_i32(&mut self, reason: usize) -> i32 {
        self.driver(move |d| d.ad.port_base.get_int32_param(reason, 0).unwrap_or(0))
            .unwrap_or(0)
    }

    fn get_f64(&mut self, reason: usize) -> f64 {
        self.driver(move |d| d.ad.port_base.get_float64_param(reason, 0).unwrap_or(0.0))
            .unwrap_or(0.0)
    }

    fn wire<R: Send + 'static>(
        &mut self,
        f: impl FnOnce(&mut WireLink) -> Result<R, WireError> + Send + 'static,
    ) -> Result<R, WireError> {
        self.driver(move |d| d.with_wire(f))?
    }

    fn command_response(&mut self, command: &str) -> Result<HashMap<String, String>, WireError> {
        let command = command.to_string();
        self.wire(move |link| link.command_response(&command))
    }

    // -----------------------------------------------------------------------
    // Acquisition loop — `SpecsAnalyser::specsAnalyserTask`
    // (`specsAnalyser.cpp:268-706`).
    // -----------------------------------------------------------------------

    /// The "not acquiring" branch through the end of the preamble
    /// (`specsAnalyser.cpp:300-432`). Returns `None` if the preamble itself
    /// failed (C's `status != asynSuccess` branch, `specsAnalyser.cpp:436-440`)
    /// — the caller then simply loops back to the wait branch, exactly as C's
    /// `while(1)` falls through with `ADAcquire` already 0.
    fn wait_and_prepare(&mut self) -> Option<Prepared> {
        self.set_i32(self.w.p.pause_acq, 0);
        if self.last_status_ok {
            self.set_str(self.w.ad.status_message, "Waiting for the acquire command");
            let adstatus = self.get_i32(self.w.ad.status);
            if adstatus != ADStatus::Aborted as i32 && adstatus != ADStatus::Error as i32 {
                self.set_i32(self.w.ad.status, ADStatus::Idle as i32);
            }
        }
        self.set_i32(self.w.ad.num_exposures_counter, 0);
        self.set_i32(self.w.ad.num_images_counter, 0);
        self.callbacks();

        let start = self.w.start.clone();
        self.unlocked(|| start.wait());

        let acquire = self.get_i32(self.w.ad.acquire);
        self.set_i32(self.w.p.pause_acq, 0);
        let iterations = self.get_i32(self.w.ad.num_exposures);

        let mut ok = true;
        let mut non_energy_channels = 0;
        match self.wire(|link| link.get_analyser_parameter_int("NumNonEnergyChannels")) {
            Ok(v) => {
                non_energy_channels = v;
                self.set_i32(self.w.p.non_energy_channels, v);
            }
            Err(_) => ok = false,
        }

        let run_mode = if ok {
            // C discards `sendSimpleCommand(SPECS_CMD_CLEAR)`'s own status.
            let _ = self.command_response("ClearSpectrum");
            let run_mode_val = self.get_i32(self.w.p.run_mode);
            let run_mode = RunMode::from_i32(run_mode_val);
            if run_mode.is_some() {
                ok = self.run_define();
            }
            run_mode
        } else {
            None
        };

        // validateSpectrum() (specsAnalyser.cpp:979) runs whenever status is
        // still success, independent of whether runMode matched a known
        // case — the C `default:` branch only skips Define, not Validate.
        let mut validated: Option<ValidateResult> = None;
        if ok {
            let kinetic_energy = self.get_f64(self.w.p.kinetic_energy);
            let validated_now = self.driver(move |d| d.validate_spectrum(run_mode, kinetic_energy));
            match validated_now.and_then(|r| r) {
                Ok(result) => validated = Some(result),
                Err(_) => ok = false,
            }
        }

        let mut energy_channels = 0;
        if ok {
            let result = validated.expect("ok implies validated is Some");
            self.set_f64(self.w.p.start_energy, result.start_energy);
            self.set_f64(self.w.p.end_energy, result.end_energy);
            self.set_f64(self.w.p.step_width, result.step_width);
            self.set_i32(self.w.p.samples_iteration, result.samples);
            self.set_f64(self.w.ad.acquire_time, result.dwell_time);
            self.set_f64(self.w.p.pass_energy, result.pass_energy);
            self.set_i32(self.w.p.lens_mode, result.lens_mode);
            self.set_i32(self.w.p.scan_range, result.scan_range);
            energy_channels = result.samples;
            self.set_i32(self.w.p.samples, energy_channels * iterations);

            if run_mode == Some(RunMode::Sfat) {
                // The number of channels is (end-start)/width + 1
                // (specsAnalyser.cpp:388-398).
                let width = result.step_width;
                if width != 0.0 {
                    energy_channels = ((result.end_energy - result.start_energy) / width + 0.5)
                        .floor() as i32
                        + 1;
                }
                self.set_i32(self.w.p.samples_iteration, energy_channels);
                self.set_i32(self.w.p.samples, energy_channels * iterations);
            }

            let nbytes = (energy_channels.max(0) as i64) * (non_energy_channels.max(0) as i64) * 8;
            self.set_i32(self.w.ad.base.array_size_x, energy_channels);
            self.set_i32(self.w.ad.base.array_size_y, non_energy_channels);
            self.set_i32(self.w.ad.base.array_size, nbytes as i32);
            self.callbacks();
        }

        self.last_status_ok = ok;
        if !ok {
            self.set_i32(self.w.ad.acquire, 0);
            self.set_i32(self.w.ad.status, ADStatus::Error as i32);
            self.callbacks();
            return None;
        }

        Some(Prepared {
            iterations,
            acquire,
            non_energy_channels,
            energy_channels,
        })
    }

    /// The `DefineSpectrum*` command for the current run mode
    /// (`specsAnalyser.cpp:344-366`), as `writeInt32` sends it.
    fn run_define(&mut self) -> bool {
        matches!(self.driver(|d| d.dispatch_define().is_ok()), Ok(true))
    }

    /// The acquisition body (`specsAnalyser.cpp:441-704`).
    async fn run_acquisition(&mut self, prepared: Prepared) {
        let Prepared {
            iterations,
            mut acquire,
            non_energy_channels,
            energy_channels,
        } = prepared;
        let nx = energy_channels.max(0) as usize;
        let ny = non_energy_channels.max(0) as usize;
        let mut image = vec![0.0f64; nx * ny];
        let mut spectrum = vec![0.0f64; nx];

        self.set_i32(self.w.p.percent_complete_iteration, 0);
        self.set_i32(self.w.p.current_sample_iteration, 0);
        self.set_i32(self.w.p.percent_complete, 0);
        self.set_i32(self.w.p.current_sample, 0);

        let acq_start_epoch = now_epoch_f64();
        let acq_start_instant = Instant::now();

        let acquire_time = self.get_f64(self.w.ad.acquire_time);
        let acquire_period = self.get_f64(self.w.ad.acquire_period);
        let num_images = self.get_i32(self.w.ad.num_images);
        let image_mode = self.get_i32(self.w.ad.image_mode);
        let safe_state = self.get_i32(self.w.p.safe_state);

        self.set_i32(self.w.ad.status, ADStatus::Initializing as i32);
        self.set_str(self.w.ad.status_message, "Executing pre-scan...");

        let mut ok = true;
        let mut iteration = 0i32;
        let mut last_data: HashMap<String, String> = HashMap::new();
        while iteration < iterations && acquire == 1 {
            let _ = self.command_response("ClearSpectrum");
            let _ = self.command_response(&codec::start_command(safe_state != 0));

            let mut current_data_point: i32 = 0;
            let mut num_data_points: i32;
            last_data = self
                .command_response("GetAcquisitionStatus")
                .unwrap_or_default();

            loop {
                // specsAnalyser.cpp:498 — matches exactly, including the
                // C author's own upgrade from the commented-out original
                // condition just above it.
                let controller_state = last_data
                    .get("ControllerState")
                    .map(String::as_str)
                    .unwrap_or("");
                let should_continue = acquire != 0
                    && ok
                    && (controller_state != "finished" || current_data_point < energy_channels)
                    && controller_state != "aborted"
                    && controller_state != "error";
                if !should_continue {
                    break;
                }

                self.unlocked(|| std::thread::sleep(secs(UPDATE_RATE)));

                // sendSimpleCommand always replaces the whole map
                // (specsAnalyser.cpp:1335-1337), even on a device-level
                // ERROR reply or an I/O failure — never merges into stale
                // data from a prior poll.
                match self.command_response("GetAcquisitionStatus") {
                    Ok(data) => {
                        ok = true;
                        last_data = data;
                        if last_data.contains_key("Code") {
                            last_data.insert("ControllerState".to_string(), "error".to_string());
                        }
                    }
                    Err(WireError::Device(info)) => {
                        ok = false;
                        last_data = HashMap::from([
                            ("Code".to_string(), info.code),
                            ("Message".to_string(), info.message),
                            ("ControllerState".to_string(), "error".to_string()),
                        ]);
                    }
                    Err(_) => {
                        ok = false;
                        last_data = HashMap::new();
                    }
                }

                num_data_points = codec::parse_integer_field(
                    last_data
                        .get("NumberOfAcquiredPoints")
                        .map(String::as_str)
                        .unwrap_or(""),
                )
                .unwrap_or(0);

                if num_data_points > current_data_point {
                    if current_data_point == 0 {
                        self.set_i32(self.w.ad.status, ADStatus::Acquire as i32);
                        self.set_str(self.w.ad.status_message, "Acquiring data...");

                        let data_delay_max = self.get_f64(self.w.p.data_delay_max);
                        let period = acquire_time.min(data_delay_max);
                        // C sleeps here holding the lock (specsAnalyser.cpp:525).
                        std::thread::sleep(secs(period));
                        if let Ok(range) = self.wire(|link| link.read_ordinate_range()) {
                            self.set_str(self.w.p.non_energy_units, range.unit);
                            if let Some(min) = range.min {
                                self.set_f64(self.w.p.non_energy_min, min);
                            }
                            if let Some(max) = range.max {
                                self.set_f64(self.w.p.non_energy_max, max);
                            }
                            self.callbacks();
                        }
                    }

                    let mut read_end_data_point = num_data_points;
                    if (read_end_data_point - current_data_point) * non_energy_channels.max(1)
                        > MAX_VALUES
                    {
                        read_end_data_point =
                            current_data_point + MAX_VALUES / non_energy_channels.max(1);
                    }
                    let (from, to) = (current_data_point, read_end_data_point - 1);
                    let values = self
                        .wire(move |link| link.read_acquisition_data(from, to))
                        .unwrap_or_default();

                    if (values.len() as i32)
                        < (read_end_data_point - current_data_point) * non_energy_channels
                    {
                        let _ = self.command_response("Abort");
                        ok = false;
                        self.set_i32(self.w.ad.acquire, 0);
                        self.set_i32(self.w.ad.status, ADStatus::Error as i32);
                        self.set_str(self.w.ad.status_message, "SPECS Receive Error, see log");
                        continue;
                    }

                    let mut index = 0usize;
                    for y in 0..non_energy_channels.max(0) {
                        if num_data_points > energy_channels {
                            let _ = self.command_response("Abort");
                            ok = false;
                            self.set_i32(self.w.ad.acquire, 0);
                            self.set_i32(self.w.ad.status, ADStatus::Error as i32);
                            self.set_str(
                                self.w.ad.status_message,
                                "SPECS Controller Error(B), see log",
                            );
                            break;
                        }
                        for x in current_data_point..read_end_data_point {
                            let (xu, yu) = (x.max(0) as usize, y.max(0) as usize);
                            if xu >= nx || yu >= ny || index >= values.len() {
                                index += 1;
                                continue;
                            }
                            if iteration == 0 {
                                image[yu * nx + xu] = values[index];
                            } else {
                                image[yu * nx + xu] += values[index];
                            }
                            if xu < spectrum.len() {
                                spectrum[xu] += values[index];
                            }
                            index += 1;
                        }
                    }
                    current_data_point = read_end_data_point;

                    if iteration == 0 {
                        self.set_f64_array(
                            self.w.p.acq_spectrum,
                            spectrum[..current_data_point.max(0) as usize].to_vec(),
                        );
                    } else {
                        self.set_f64_array(self.w.p.acq_spectrum, spectrum.clone());
                    }
                    self.set_f64_array(self.w.p.acq_image, image.clone());

                    let pct_iter = if energy_channels > 0 {
                        (current_data_point * 100) / energy_channels
                    } else {
                        0
                    };
                    self.set_i32(self.w.p.percent_complete_iteration, pct_iter);
                    let total = iterations * energy_channels;
                    let pct_total = if total > 0 {
                        ((current_data_point + iteration * energy_channels) * 100) / total
                    } else {
                        0
                    };
                    self.set_i32(self.w.p.percent_complete, pct_total);
                    self.set_i32(self.w.p.current_sample_iteration, current_data_point);
                    self.set_i32(
                        self.w.p.current_sample,
                        current_data_point + iteration * energy_channels,
                    );

                    // specsAnalyser.cpp:614-615 re-reads ADAcquireTime fresh
                    // here (distinct from the `acquireTime` captured once
                    // before the iteration loop) so a client changing it
                    // mid-scan is reflected in the estimate immediately.
                    let current_acquire_time = self.get_f64(self.w.ad.acquire_time);
                    let remaining_iter =
                        ((energy_channels - current_data_point) as f64) * current_acquire_time;
                    self.set_f64(self.w.p.remaining_time_iteration, remaining_iter);
                    let remaining_total = ((energy_channels - current_data_point
                        + (iterations - (iteration + 1)) * energy_channels)
                        as f64)
                        * current_acquire_time;
                    self.set_f64(self.w.p.remaining_time, remaining_total);

                    self.callbacks();
                }

                acquire = self.get_i32(self.w.ad.acquire);
            }

            if last_data.get("ControllerState").map(String::as_str) == Some("error") {
                ok = false;
                self.set_i32(self.w.ad.acquire, 0);
                self.set_i32(self.w.ad.status, ADStatus::Error as i32);
                match last_data.get("Message").filter(|m| !m.is_empty()) {
                    Some(msg) => self.set_str(self.w.ad.status_message, msg.clone()),
                    None => {
                        self.set_str(self.w.ad.status_message, "SPECS Controller Error, see log")
                    }
                }
            }

            acquire = self.get_i32(self.w.ad.acquire);
            iteration += 1;
        }
        let _ = &last_data;

        let array_counter = self.get_i32(self.w.ad.base.array_counter) + 1;
        let num_images_counter = self.get_i32(self.w.ad.num_images_counter) + 1;
        self.set_i32(self.w.ad.base.array_counter, array_counter);
        self.set_i32(self.w.ad.num_images_counter, num_images_counter);

        let array_callbacks = self.get_i32(self.w.ad.base.array_callbacks);
        // C dereferences a refused alloc unchecked (specsAnalyser.cpp:454, :493);
        // here the frame is dropped after the pool's limit line.
        if array_callbacks != 0
            && let Ok(mut array) = self.w.pool.alloc_with(
                vec![NDDimension::new(nx), NDDimension::new(ny)],
                NDDataBuffer::F64(image),
            )
        {
            array.unique_id = array_counter;
            array.time_stamp = acq_start_epoch;
            array.timestamp = EpicsTimestamp::now();
            // C unlock() … lock() around doCallbacksGenericPointer
            // (specsAnalyser.cpp:664-667); the publish awaits, so the hold is
            // dropped and retaken rather than lent to a closure.
            drop(self.hold.take());
            self.w.output.publish(array).await;
            self.hold = self.w.self_handle.hold_driver_blocking().ok();
        }

        self.last_status_ok = ok;
        if ok {
            if image_mode == 0 /* Single */ || (image_mode == 1 /* Multiple */ && num_images_counter >= num_images)
            {
                self.set_i32(self.w.ad.acquire, 0);
            }
            self.callbacks();
            acquire = self.get_i32(self.w.ad.acquire);

            if acquire != 0 {
                let elapsed = acq_start_instant.elapsed().as_secs_f64();
                let delay = acquire_period - elapsed;
                if delay >= 0.0 {
                    self.set_i32(self.w.ad.status, ADStatus::Waiting as i32);
                    self.callbacks();
                    let stop = self.w.stop.clone();
                    self.unlocked(|| stop.wait_timeout(secs(delay)));
                }
            }
        }
    }

    /// C `specsAnalyserTask`'s outer `while(1)` (`specsAnalyser.cpp:300-433`).
    /// The preamble only runs when `ADAcquire` reads 0 at the top of the
    /// pass; otherwise (e.g. `ImageMode::Continuous`, which never resets
    /// `ADAcquire` between batches) it falls straight through to rerun the
    /// acquisition body against the previous pass's `Prepared` state.
    async fn run(mut self) {
        let mut prepared = Prepared::C_LOCALS;
        while self.hold.is_some() {
            let acquire = self.get_i32(self.w.ad.acquire);
            if acquire == 0 {
                match self.wait_and_prepare() {
                    Some(p) => prepared = p,
                    None => continue,
                }
            } else {
                prepared.acquire = acquire;
            }
            self.run_acquisition(prepared).await;
        }
    }
}

/// Spawn the acquisition worker thread (C `epicsThreadCreate(...,
/// specsAnalyserTaskC, ...)`).
pub(crate) fn start_task(w: Worker) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("specsAnalyserTask", move || async move {
        let hold = w.self_handle.hold_driver_blocking().ok();
        Task {
            w,
            hold,
            last_status_ok: true,
        }
        .run()
        .await
    })
}
