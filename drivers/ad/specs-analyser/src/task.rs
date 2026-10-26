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
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
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
    l: Locals,
}

/// Current time as an f64 epoch second count (C `secPastEpoch + nsec/1e9`).
fn now_epoch_f64() -> f64 {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_secs_f64(),
        Err(e) => -(e.duration().as_secs_f64()),
    }
}

/// C `specsAnalyserTask`'s function-scope locals (`specsAnalyser.cpp:270-294`)
/// that `getIntegerParam`/`getDoubleParam` read into. They persist across
/// outer-loop passes, so a pass that finds `ADAcquire` already 1 (e.g.
/// `ImageMode::Continuous`) reruns the body on the previous pass's values,
/// and a read of an undefined parameter leaves the local as it was.
#[derive(Clone, Copy)]
struct Locals {
    acquire: i32,
    iterations: i32,
    run_mode: i32,
    non_energy_channels: i32,
    energy_channels: i32,
    acquire_time: f64,
    acquire_period: f64,
    num_images: i32,
    image_mode: i32,
    safe_state: i32,
    array_callbacks: i32,
    image_counter: i32,
    num_images_counter: i32,
}

impl Locals {
    /// The initial values C declares (`acquire`, `acquireTime` and
    /// `acquirePeriod` are uninitialised there).
    const C: Locals = Locals {
        acquire: 0,
        iterations: 1,
        run_mode: 0,
        non_energy_channels: 0,
        energy_channels: 0,
        acquire_time: 0.0,
        acquire_period: 0.0,
        num_images: 0,
        image_mode: 0,
        safe_state: 1,
        array_callbacks: 0,
        image_counter: 0,
        num_images_counter: 0,
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

    /// C `doCallbacksFloat64Array(value, n, reason, 0)`: posted at once, not
    /// stored.
    fn do_callbacks_f64_array(&mut self, reason: usize, value: Vec<f64>) {
        let _ = self.driver(move |d| {
            let value = ParamValue::Float64Array(value.into());
            let _ = d.ad.port_base.do_callbacks_array(reason, 0, value);
        });
    }

    fn callbacks(&mut self) {
        let _ = self.driver(|d| {
            let _ = d.ad.port_base.call_param_callbacks(0);
        });
    }

    /// C `getIntegerParam(reason, &local)`: the value, or `local` when the
    /// read fails (an undefined parameter).
    fn get_i32(&mut self, reason: usize, local: i32) -> i32 {
        self.driver(move |d| d.ad.port_base.get_int32_param(reason, 0).ok())
            .ok()
            .flatten()
            .unwrap_or(local)
    }

    /// C `getDoubleParam(reason, &local)`.
    fn get_f64(&mut self, reason: usize, local: f64) -> f64 {
        self.driver(move |d| d.ad.port_base.get_float64_param(reason, 0).ok())
            .ok()
            .flatten()
            .unwrap_or(local)
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
    /// (`specsAnalyser.cpp:300-432`). Returns `false` if the preamble itself
    /// failed (C's `status != asynSuccess` branch, `specsAnalyser.cpp:436-440`)
    /// — the caller then simply loops back to the wait branch, exactly as C's
    /// `while(1)` falls through with `ADAcquire` already 0.
    fn wait_and_prepare(&mut self) -> bool {
        self.set_i32(self.w.p.pause_acq, 0);
        if self.last_status_ok {
            self.set_str(self.w.ad.status_message, "Waiting for the acquire command");
            let adstatus = self.get_i32(self.w.ad.status, 0);
            if adstatus != ADStatus::Aborted as i32 && adstatus != ADStatus::Error as i32 {
                self.set_i32(self.w.ad.status, ADStatus::Idle as i32);
            }
        }
        self.set_i32(self.w.ad.num_exposures_counter, 0);
        self.set_i32(self.w.ad.num_images_counter, 0);
        self.callbacks();

        let start = self.w.start.clone();
        self.unlocked(|| start.wait());

        self.l.acquire = self.get_i32(self.w.ad.acquire, self.l.acquire);
        self.set_i32(self.w.p.pause_acq, 0);
        self.l.iterations = self.get_i32(self.w.ad.num_exposures, self.l.iterations);
        let iterations = self.l.iterations;

        let mut ok = true;
        match self.wire(|link| link.get_analyser_parameter_int("NumNonEnergyChannels")) {
            Ok(v) => {
                self.l.non_energy_channels = v;
                self.set_i32(self.w.p.non_energy_channels, v);
            }
            Err(_) => ok = false,
        }

        let run_mode = if ok {
            // C discards `sendSimpleCommand(SPECS_CMD_CLEAR)`'s own status.
            let _ = self.command_response("ClearSpectrum");
            self.l.run_mode = self.get_i32(self.w.p.run_mode, self.l.run_mode);
            let run_mode = RunMode::from_i32(self.l.run_mode);
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
            let kinetic_energy = self.get_f64(self.w.p.kinetic_energy, 0.0);
            let validated_now = self.driver(move |d| d.validate_spectrum(run_mode, kinetic_energy));
            match validated_now.and_then(|r| r) {
                Ok(result) => validated = Some(result),
                Err(_) => ok = false,
            }
        }

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
            let mut energy_channels = result.samples;
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

            self.l.energy_channels = energy_channels;
            let non_energy_channels = self.l.non_energy_channels;
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
        }
        ok
    }

    /// The `DefineSpectrum*` command for the current run mode
    /// (`specsAnalyser.cpp:344-366`), as `writeInt32` sends it.
    fn run_define(&mut self) -> bool {
        matches!(self.driver(|d| d.dispatch_define().is_ok()), Ok(true))
    }

    /// The acquisition body (`specsAnalyser.cpp:441-704`).
    async fn run_acquisition(&mut self) {
        let Locals {
            iterations,
            non_energy_channels,
            energy_channels,
            ..
        } = self.l;
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

        self.l.acquire_time = self.get_f64(self.w.ad.acquire_time, self.l.acquire_time);
        self.l.acquire_period = self.get_f64(self.w.ad.acquire_period, self.l.acquire_period);
        self.l.num_images = self.get_i32(self.w.ad.num_images, self.l.num_images);
        self.l.image_mode = self.get_i32(self.w.ad.image_mode, self.l.image_mode);
        self.l.safe_state = self.get_i32(self.w.p.safe_state, self.l.safe_state);
        let (acquire_time, acquire_period, safe_state) = (
            self.l.acquire_time,
            self.l.acquire_period,
            self.l.safe_state,
        );

        self.set_i32(self.w.ad.status, ADStatus::Initializing as i32);
        self.set_str(self.w.ad.status_message, "Executing pre-scan...");

        let mut ok = true;
        let mut iteration = 0i32;
        let mut last_data: HashMap<String, String> = HashMap::new();
        while iteration < iterations && self.l.acquire == 1 {
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
                let should_continue = self.l.acquire != 0
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

                        let data_delay_max = self.get_f64(self.w.p.data_delay_max, 0.0);
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
                        self.do_callbacks_f64_array(
                            self.w.p.acq_spectrum,
                            spectrum[..current_data_point.max(0) as usize].to_vec(),
                        );
                    } else {
                        self.do_callbacks_f64_array(self.w.p.acq_spectrum, spectrum.clone());
                    }
                    self.do_callbacks_f64_array(self.w.p.acq_image, image.clone());

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
                    let current_acquire_time = self.get_f64(self.w.ad.acquire_time, 0.0);
                    let remaining_iter =
                        ((energy_channels - current_data_point) as f64) * current_acquire_time;
                    self.set_f64(self.w.p.remaining_time_iteration, remaining_iter);
                    let remaining_total = ((energy_channels - current_data_point
                        + (iterations - (iteration + 1)) * energy_channels)
                        as f64)
                        * current_acquire_time;
                    self.set_f64(self.w.p.remaining_time, remaining_total);

                    // C unlock() … lock() around callParamCallbacks
                    // (specsAnalyser.cpp:617-619).
                    let handle = self.w.self_handle.clone();
                    self.unlocked(|| {
                        let _ = handle.call_param_callbacks_blocking(0);
                    });
                }

                self.l.acquire = self.get_i32(self.w.ad.acquire, self.l.acquire);
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

            self.l.acquire = self.get_i32(self.w.ad.acquire, self.l.acquire);
            iteration += 1;
        }
        let _ = &last_data;

        // C re-reads these after the loop (specsAnalyser.cpp:650-654).
        self.l.num_images = self.get_i32(self.w.ad.num_images, self.l.num_images);
        self.l.image_mode = self.get_i32(self.w.ad.image_mode, self.l.image_mode);
        self.l.array_callbacks =
            self.get_i32(self.w.ad.base.array_callbacks, self.l.array_callbacks);
        self.l.image_counter = self.get_i32(self.w.ad.base.array_counter, self.l.image_counter) + 1;
        self.l.num_images_counter =
            self.get_i32(self.w.ad.num_images_counter, self.l.num_images_counter) + 1;
        let (array_counter, num_images_counter, array_callbacks) = (
            self.l.image_counter,
            self.l.num_images_counter,
            self.l.array_callbacks,
        );
        self.set_i32(self.w.ad.base.array_counter, array_counter);
        self.set_i32(self.w.ad.num_images_counter, num_images_counter);

        // C dereferences a refused alloc unchecked (specsAnalyser.cpp:454, :493);
        // here the frame is dropped after the pool's limit line.
        if array_callbacks != 0
            && let Ok(mut array) = self.w.pool.alloc(
                vec![NDDimension::new(nx), NDDimension::new(ny)],
                NDDataType::Float64,
            )
        {
            array
                .fill()
                .data_mut()
                .as_mut_slice::<f64>()
                .expect("allocated as NDFloat64")
                .copy_from_slice(&image);
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
            let (image_mode, num_images) = (self.l.image_mode, self.l.num_images);
            if image_mode == 0 /* Single */ || (image_mode == 1 /* Multiple */ && num_images_counter >= num_images)
            {
                self.set_i32(self.w.ad.acquire, 0);
            }
            self.callbacks();
            self.l.acquire = self.get_i32(self.w.ad.acquire, self.l.acquire);

            if self.l.acquire != 0 {
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
    /// acquisition body against the previous pass's [`Locals`].
    async fn run(mut self) {
        while self.hold.is_some() {
            self.l.acquire = self.get_i32(self.w.ad.acquire, self.l.acquire);
            if self.l.acquire == 0 && !self.wait_and_prepare() {
                continue;
            }
            self.run_acquisition().await;
        }
    }
}

/// Spawn the acquisition worker thread (C `epicsThreadCreate(...,
/// specsAnalyserTaskC, ...)`). Returns once the task holds the driver, so a
/// driver call made next lands after its first pass.
pub(crate) fn start_task(w: Worker) -> std::thread::JoinHandle<()> {
    let (held, holding) = std::sync::mpsc::sync_channel(1);
    let task = rt::run_thread_named("specsAnalyserTask", move || async move {
        let hold = w.self_handle.hold_driver_blocking().ok();
        let _ = held.send(());
        Task {
            w,
            hold,
            last_status_ok: true,
            l: Locals::C,
        }
        .run()
        .await
    });
    let _ = holding.recv();
    task
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::plugin::channel::NDArrayOutput;
    use epics_rs::asyn::interfaces::InterfaceType;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;
    use epics_rs::asyn::sync_io::SyncIOHandle;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

    use crate::types::SOCKET_TIMEOUT;

    /// The spectrum and the image go out with doCallbacksFloat64Array
    /// (specsAnalyser.cpp:599-606), typed and in that order.
    #[test]
    fn the_spectrum_and_image_post_as_array_callbacks() {
        let name = "SPECS_ARRAYS";
        let (driver_port, _) = SyncIOHandle::connect_port(
            &format!("{name}_NONE"),
            0,
            secs(SOCKET_TIMEOUT),
            InterfaceType::Octet,
        );
        let (start, stop) = (Arc::new(Event::new()), Arc::new(Event::new()));
        let mut driver = SpecsAnalyserDriver::new(
            name,
            Arc::new(driver_port),
            0,
            1 << 20,
            start.clone(),
            stop.clone(),
        )
        .unwrap();
        driver.ad.port_base.own_interrupts(InterruptGate::opened());
        driver.ad.port_base.call_param_callbacks(0).unwrap();
        let (p, ad, pool) = (driver.p, driver.ad.params, driver.ad.pool.clone());
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            driver,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = runtime.port_handle().clone();
        let mut rx = handle.interrupts().subscribe_async(64, 1 << 20);
        let mut task = Task {
            hold: handle.hold_driver_blocking().ok(),
            w: Worker {
                self_handle: handle.clone(),
                p,
                ad,
                start,
                stop,
                output: ArrayPublisher::new(Arc::new(
                    parking_lot::Mutex::new(NDArrayOutput::new()),
                )),
                pool,
            },
            last_status_ok: true,
            l: Locals::C,
        };
        task.do_callbacks_f64_array(p.acq_spectrum, vec![1.0, 2.0]);
        task.do_callbacks_f64_array(p.acq_image, vec![3.0]);
        task.callbacks();
        drop(task);

        let posted: Vec<_> = std::iter::from_fn(|| rx.try_recv().ok())
            .filter(|v| [p.acq_spectrum, p.acq_image].contains(&v.reason))
            .map(|v| (v.reason, v.iface))
            .collect();
        assert_eq!(
            posted,
            [
                (p.acq_spectrum, Some(InterfaceType::Float64Array)),
                (p.acq_image, Some(InterfaceType::Float64Array)),
            ]
        );
        std::mem::forget(runtime);
    }
}
