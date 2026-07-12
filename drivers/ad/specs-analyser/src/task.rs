//! The persistent acquisition worker: `SpecsAnalyser::specsAnalyserTask`
//! (`specsAnalyser.cpp:268-706`). Runs on its own dedicated Tokio runtime
//! (`rt::run_thread_named`) because publishing an `NDArray`
//! (`ArrayPublisher::publish`) is `async fn` with no blocking variant; every
//! wire/param access here uses `.await`-based `PortHandle` methods, never the
//! `_blocking` ones, so it carries no panic risk regardless of runtime
//! flavor. `write_int32` (a different context — the port actor's own
//! current-thread runtime) cannot join this loop's structure, so the split
//! documented in `driver.rs` applies: `write_int32` only ever signals `start`
//! or reads/writes params directly; every acquisition-loop wire round trip
//! lives here.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::user::AsynUser;

use crate::codec::{self, BeginOutcome, ResponseAssembler};
use crate::params::SpecsParams;
use crate::types::{
    Event, MAX_MESSAGE_SIZE, MAX_VALUES, RunMode, SOCKET_TIMEOUT, UPDATE_RATE, secs,
};
use crate::wire::{OrdinateRange, ValidateResult, WireError};

/// Everything the acquisition worker needs. Shares `driver_port`/`wire_state`/
/// `connected` with `write_int32`'s spawned threads (see `driver.rs`'s
/// module doc comment) and `lens_modes`/`scan_ranges` with the driver (kept
/// current by `SPECSConnect_` handling, read here to build `Define*`
/// commands exactly as C's `lensModes_[ivalue]`/`scanRanges_[ivalue]` does).
pub struct Worker {
    /// This driver's own port (parameter library).
    pub self_handle: PortHandle,
    /// `driverPort_`.
    pub driver_port: PortHandle,
    pub addr: i32,
    pub wire_state: Arc<Mutex<u32>>,
    pub connected: Arc<AtomicBool>,
    pub p: SpecsParams,
    pub ad: ADDriverParams,
    /// C `startEventId_`.
    pub start: Arc<Event>,
    /// C `stopEventId_`.
    pub stop: Arc<Event>,
    pub lens_modes: Arc<Mutex<Vec<String>>>,
    pub scan_ranges: Arc<Mutex<Vec<String>>>,
    pub output: ArrayPublisher,
    /// Pending parameter writes, flushed at each C `callParamCallbacks()`.
    /// Always constructed empty.
    pub batch: Vec<ParamSetValue>,
    /// C's `status` local, carried across outer-loop passes — used only to
    /// decide whether the wait branch may overwrite `ADStatusMessage`/
    /// `ADStatus` (`specsAnalyser.cpp:307-313`). Always constructed `true`.
    pub last_status_ok: bool,
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
/// verbatim; see `Worker::run`.
#[derive(Clone, Copy)]
struct Prepared {
    iterations: i32,
    acquire: i32,
    non_energy_channels: i32,
    energy_channels: i32,
}

impl Worker {
    // -----------------------------------------------------------------------
    // Parameter library (C `setIntegerParam`/`setDoubleParam`/`getIntegerParam`/
    // `getDoubleParam`/`callParamCallbacks`).
    // -----------------------------------------------------------------------

    fn set_i32(&mut self, reason: usize, value: i32) {
        self.batch.push(ParamSetValue::Int32 {
            reason,
            addr: 0,
            value,
        });
    }

    fn set_f64(&mut self, reason: usize, value: f64) {
        self.batch.push(ParamSetValue::Float64 {
            reason,
            addr: 0,
            value,
        });
    }

    fn set_str(&mut self, reason: usize, value: impl Into<String>) {
        self.batch.push(ParamSetValue::Octet {
            reason,
            addr: 0,
            value: value.into(),
        });
    }

    fn set_f64_array(&mut self, reason: usize, value: Vec<f64>) {
        self.batch.push(ParamSetValue::Float64Array {
            reason,
            addr: 0,
            value,
        });
    }

    async fn callbacks(&mut self) {
        let updates = std::mem::take(&mut self.batch);
        if let Err(e) = self.self_handle.set_params_and_notify(0, updates).await {
            log::error!("specs-analyser: callParamCallbacks failed: {e}");
        }
    }

    async fn flush(&mut self) {
        if !self.batch.is_empty() {
            self.callbacks().await;
        }
    }

    async fn get_i32(&mut self, reason: usize) -> i32 {
        self.flush().await;
        self.self_handle.read_int32(reason, 0).await.unwrap_or(0)
    }

    async fn get_f64(&mut self, reason: usize) -> f64 {
        self.flush().await;
        self.self_handle
            .read_float64(reason, 0)
            .await
            .unwrap_or(0.0)
    }

    // -----------------------------------------------------------------------
    // Wire transport — async twin of `crate::wire::WireLink`. Duplicates only
    // the thin write/read loop (the parsing/framing logic all lives in
    // `crate::codec`); see this module's top doc comment for why a shared
    // sync/async abstraction was not worth the complexity here.
    // -----------------------------------------------------------------------

    fn user(&self) -> AsynUser {
        AsynUser::new(0)
            .with_addr(self.addr)
            .with_timeout(secs(SOCKET_TIMEOUT))
    }

    /// `SpecsAnalyser::commandResponse` + `SpecsAnalyser::asynWriteRead`
    /// (`specsAnalyser.cpp:1897-2118`); see [`crate::wire::WireLink::command_response`].
    async fn command_response(
        &mut self,
        command: &str,
    ) -> Result<HashMap<String, String>, WireError> {
        if !self.connected.load(Ordering::SeqCst) {
            return Err(WireError::NotConnected);
        }
        let counter = {
            let mut guard = self.wire_state.lock().unwrap();
            let next = codec::next_counter(*guard);
            *guard = next;
            next
        };
        let request = codec::format_request(counter, command);

        let result = self
            .driver_port
            .submit_async(
                RequestOp::OctetWriteRead {
                    data: request.into_bytes(),
                    buf_size: MAX_MESSAGE_SIZE,
                    flush: true,
                },
                self.user(),
            )
            .await
            .map_err(WireError::Io)?;
        let raw = String::from_utf8_lossy(&result.data.unwrap_or_default()).into_owned();
        let payload = codec::strip_response_frame(&raw, counter).map_err(WireError::Frame)?;

        let mut assembler = ResponseAssembler::new();
        match assembler.begin(payload) {
            BeginOutcome::Ok => {}
            BeginOutcome::NeedsMore => loop {
                let more = self
                    .driver_port
                    .submit_async(
                        RequestOp::OctetRead {
                            buf_size: MAX_MESSAGE_SIZE,
                        },
                        self.user(),
                    )
                    .await
                    .map_err(WireError::Io)?;
                let chunk = String::from_utf8_lossy(&more.data.unwrap_or_default()).into_owned();
                if !assembler.continue_with(&chunk) {
                    break;
                }
            },
            BeginOutcome::Error(info) => {
                if info.code == "3" {
                    self.connected.store(false, Ordering::SeqCst);
                    self.set_i32(self.p.connected, 0);
                }
                return Err(WireError::Device(info));
            }
        }
        Ok(assembler.finish())
    }

    async fn get_analyser_parameter_int(&mut self, name: &str) -> Result<i32, WireError> {
        let data = self
            .command_response(&codec::get_value_command(name))
            .await?;
        let value = data.get("Value").map(String::as_str).unwrap_or("");
        Ok(match value {
            "\"false\"" => 0,
            "\"true\"" => 1,
            _ => codec::parse_integer_field(value).unwrap_or(0),
        })
    }

    async fn read_ordinate_range(&mut self) -> Result<OrdinateRange, WireError> {
        let data = self
            .command_response(&codec::get_data_info_command("OrdinateRange"))
            .await?;
        let unit = match data.get("Unit").map(String::as_str) {
            Some("\"\"") | None => String::new(),
            Some(u) => codec::clean_string(u, "\""),
        };
        Ok(OrdinateRange {
            unit,
            min: data.get("Min").and_then(|s| codec::parse_double_field(s)),
            max: data.get("Max").and_then(|s| codec::parse_double_field(s)),
        })
    }

    async fn read_acquisition_data(
        &mut self,
        start_index: i32,
        end_index: i32,
    ) -> Result<Vec<f64>, WireError> {
        let data = self
            .command_response(&codec::get_data_command(start_index, end_index))
            .await?;
        Ok(codec::parse_data_array(
            data.get("Data").map(String::as_str).unwrap_or(""),
        ))
    }

    async fn define_fat(
        &mut self,
        args: codec::DefineFatArgs,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_fat_command(args, lens_mode, scan_range);
        self.command_response(&cmd).await.map(|_| ())
    }

    async fn define_sfat(
        &mut self,
        start_energy: f64,
        end_energy: f64,
        samples: i32,
        dwell_time: f64,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_sfat_command(
            start_energy,
            end_energy,
            samples,
            dwell_time,
            lens_mode,
            scan_range,
        );
        self.command_response(&cmd).await.map(|_| ())
    }

    async fn define_frr(
        &mut self,
        args: codec::DefineFrrArgs,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_frr_command(args, lens_mode, scan_range);
        self.command_response(&cmd).await.map(|_| ())
    }

    async fn define_fe(
        &mut self,
        kinetic_energy: f64,
        samples: i32,
        dwell_time: f64,
        pass_energy: f64,
        lens_mode: &str,
        scan_range: &str,
    ) -> Result<(), WireError> {
        let cmd = codec::define_fe_command(
            kinetic_energy,
            samples,
            dwell_time,
            pass_energy,
            lens_mode,
            scan_range,
        );
        self.command_response(&cmd).await.map(|_| ())
    }

    async fn validate_spectrum(
        &mut self,
        run_mode: Option<RunMode>,
        kinetic_energy: f64,
    ) -> Result<ValidateResult, WireError> {
        let data = self.command_response("ValidateSpectrum").await?;
        let field_f64 = |name: &str| {
            data.get(name)
                .and_then(|s| codec::parse_double_field(s))
                .unwrap_or(0.0)
        };
        let field_i32 = |name: &str| {
            data.get(name)
                .and_then(|s| codec::parse_integer_field(s))
                .unwrap_or(0)
        };

        let mut start_energy = field_f64("StartEnergy");
        let mut end_energy = field_f64("EndEnergy");
        let pass_energy = field_f64("PassEnergy");

        let lens_modes = self.lens_modes.lock().unwrap().clone();
        let scan_ranges = self.scan_ranges.lock().unwrap().clone();
        let lookup = |values: &[String], data_key: &str| -> i32 {
            let raw = data.get(data_key).map(String::as_str).unwrap_or("");
            let cleaned = codec::clean_string(raw, "\"");
            values
                .iter()
                .position(|v| *v == cleaned)
                .map(|i| i as i32)
                .unwrap_or(-1)
        };

        if run_mode == Some(RunMode::Fe) {
            // ***** WORKAROUND FOR FIXED ENERGY START AND END *****
            start_energy = kinetic_energy - 0.1 * pass_energy;
            end_energy = kinetic_energy + 0.1 * pass_energy;
        }

        Ok(ValidateResult {
            start_energy,
            end_energy,
            step_width: field_f64("StepWidth"),
            samples: field_i32("Samples"),
            dwell_time: field_f64("DwellTime"),
            pass_energy,
            lens_mode: lookup(&lens_modes, "LensMode"),
            scan_range: lookup(&scan_ranges, "ScanRange"),
        })
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
    async fn wait_and_prepare(&mut self) -> Option<Prepared> {
        self.set_i32(self.p.pause_acq, 0);
        if self.last_status_ok {
            self.set_str(self.ad.status_message, "Waiting for the acquire command");
            let adstatus = self.get_i32(self.ad.status).await;
            if adstatus != ADStatus::Aborted as i32 && adstatus != ADStatus::Error as i32 {
                self.set_i32(self.ad.status, ADStatus::Idle as i32);
            }
        }
        self.set_i32(self.ad.num_exposures_counter, 0);
        self.set_i32(self.ad.num_images_counter, 0);
        self.callbacks().await;

        self.start.wait();

        let acquire = self.get_i32(self.ad.acquire).await;
        self.set_i32(self.p.pause_acq, 0);
        let iterations = self.get_i32(self.ad.num_exposures).await;

        let mut ok = true;
        let mut non_energy_channels = 0;
        match self
            .get_analyser_parameter_int("NumNonEnergyChannels")
            .await
        {
            Ok(v) => {
                non_energy_channels = v;
                self.set_i32(self.p.non_energy_channels, v);
            }
            Err(_) => ok = false,
        }

        let run_mode = if ok {
            // C discards `sendSimpleCommand(SPECS_CMD_CLEAR)`'s own status.
            let _ = self.command_response("ClearSpectrum").await;
            let run_mode_val = self.get_i32(self.p.run_mode).await;
            let run_mode = RunMode::from_i32(run_mode_val);
            if let Some(mode) = run_mode {
                let define_ok = self.run_define(mode).await;
                ok = define_ok;
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
            let kinetic_energy = self.get_f64(self.p.kinetic_energy).await;
            match self.validate_spectrum(run_mode, kinetic_energy).await {
                Ok(result) => validated = Some(result),
                Err(_) => ok = false,
            }
        }

        let mut energy_channels = 0;
        if ok {
            let result = validated.expect("ok implies validated is Some");
            self.set_f64(self.p.start_energy, result.start_energy);
            self.set_f64(self.p.end_energy, result.end_energy);
            self.set_f64(self.p.step_width, result.step_width);
            self.set_i32(self.p.samples_iteration, result.samples);
            self.set_f64(self.ad.acquire_time, result.dwell_time);
            self.set_f64(self.p.pass_energy, result.pass_energy);
            self.set_i32(self.p.lens_mode, result.lens_mode);
            self.set_i32(self.p.scan_range, result.scan_range);
            energy_channels = result.samples;
            self.set_i32(self.p.samples, energy_channels * iterations);

            if run_mode == Some(RunMode::Sfat) {
                // The number of channels is (end-start)/width + 1
                // (specsAnalyser.cpp:388-398).
                let width = result.step_width;
                if width != 0.0 {
                    energy_channels = ((result.end_energy - result.start_energy) / width + 0.5)
                        .floor() as i32
                        + 1;
                }
                self.set_i32(self.p.samples_iteration, energy_channels);
                self.set_i32(self.p.samples, energy_channels * iterations);
            }

            let nbytes = (energy_channels.max(0) as i64) * (non_energy_channels.max(0) as i64) * 8;
            self.set_i32(self.ad.base.array_size_x, energy_channels);
            self.set_i32(self.ad.base.array_size_y, non_energy_channels);
            self.set_i32(self.ad.base.array_size, nbytes as i32);
            self.callbacks().await;
        }

        self.last_status_ok = ok;
        if !ok {
            self.set_i32(self.ad.acquire, 0);
            self.set_i32(self.ad.status, ADStatus::Error as i32);
            self.callbacks().await;
            return None;
        }

        Some(Prepared {
            iterations,
            acquire,
            non_energy_channels,
            energy_channels,
        })
    }

    /// Dispatch to the `DefineSpectrum*` command matching the current run
    /// mode (`specsAnalyser.cpp:344-366`).
    async fn run_define(&mut self, mode: RunMode) -> bool {
        let start_energy = self.get_f64(self.p.start_energy).await;
        let end_energy = self.get_f64(self.p.end_energy).await;
        let step_width = self.get_f64(self.p.step_width).await;
        let dwell_time = self.get_f64(self.ad.acquire_time).await;
        let pass_energy = self.get_f64(self.p.pass_energy).await;
        let retarding_ratio = self.get_f64(self.p.retarding_ratio).await;
        let kinetic_energy = self.get_f64(self.p.kinetic_energy).await;
        let snapshot_values = self.get_i32(self.p.snapshot_values).await;
        let lens_mode_idx = self.get_i32(self.p.lens_mode).await;
        let scan_range_idx = self.get_i32(self.p.scan_range).await;
        let lens_mode = self
            .lens_modes
            .lock()
            .unwrap()
            .get(lens_mode_idx.max(0) as usize)
            .cloned()
            .unwrap_or_default();
        let scan_range = self
            .scan_ranges
            .lock()
            .unwrap()
            .get(scan_range_idx.max(0) as usize)
            .cloned()
            .unwrap_or_default();

        let result = match mode {
            RunMode::Fat => {
                self.define_fat(
                    codec::DefineFatArgs {
                        start_energy,
                        end_energy,
                        step_width,
                        dwell_time,
                        pass_energy,
                    },
                    &lens_mode,
                    &scan_range,
                )
                .await
            }
            RunMode::Sfat => {
                self.define_sfat(
                    start_energy,
                    end_energy,
                    snapshot_values,
                    dwell_time,
                    &lens_mode,
                    &scan_range,
                )
                .await
            }
            RunMode::Frr => {
                self.define_frr(
                    codec::DefineFrrArgs {
                        start_energy,
                        end_energy,
                        step_width,
                        dwell_time,
                        retarding_ratio,
                    },
                    &lens_mode,
                    &scan_range,
                )
                .await
            }
            RunMode::Fe => {
                self.define_fe(
                    kinetic_energy,
                    snapshot_values,
                    dwell_time,
                    pass_energy,
                    &lens_mode,
                    &scan_range,
                )
                .await
            }
        };
        result.is_ok()
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

        self.set_i32(self.p.percent_complete_iteration, 0);
        self.set_i32(self.p.current_sample_iteration, 0);
        self.set_i32(self.p.percent_complete, 0);
        self.set_i32(self.p.current_sample, 0);

        let acq_start_epoch = now_epoch_f64();
        let acq_start_instant = Instant::now();

        let acquire_time = self.get_f64(self.ad.acquire_time).await;
        let acquire_period = self.get_f64(self.ad.acquire_period).await;
        let num_images = self.get_i32(self.ad.num_images).await;
        let image_mode = self.get_i32(self.ad.image_mode).await;
        let safe_state = self.get_i32(self.p.safe_state).await;

        self.set_i32(self.ad.status, ADStatus::Initializing as i32);
        self.set_str(self.ad.status_message, "Executing pre-scan...");

        let mut ok = true;
        let mut iteration = 0i32;
        let mut last_data: HashMap<String, String> = HashMap::new();
        while iteration < iterations && acquire == 1 {
            let _ = self.command_response("ClearSpectrum").await;
            let _ = self
                .command_response(&codec::start_command(safe_state != 0))
                .await;

            let mut current_data_point: i32 = 0;
            let mut num_data_points: i32;
            last_data = self
                .command_response("GetAcquisitionStatus")
                .await
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

                rt::sleep(secs(UPDATE_RATE)).await;

                // sendSimpleCommand always replaces the whole map
                // (specsAnalyser.cpp:1335-1337), even on a device-level
                // ERROR reply or an I/O failure — never merges into stale
                // data from a prior poll.
                match self.command_response("GetAcquisitionStatus").await {
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
                        self.set_i32(self.ad.status, ADStatus::Acquire as i32);
                        self.set_str(self.ad.status_message, "Acquiring data...");

                        let data_delay_max = self.get_f64(self.p.data_delay_max).await;
                        let period = acquire_time.min(data_delay_max);
                        rt::sleep(secs(period)).await;
                        if let Ok(range) = self.read_ordinate_range().await {
                            self.set_str(self.p.non_energy_units, range.unit);
                            if let Some(min) = range.min {
                                self.set_f64(self.p.non_energy_min, min);
                            }
                            if let Some(max) = range.max {
                                self.set_f64(self.p.non_energy_max, max);
                            }
                            self.callbacks().await;
                        }
                    }

                    let mut read_end_data_point = num_data_points;
                    if (read_end_data_point - current_data_point) * non_energy_channels.max(1)
                        > MAX_VALUES
                    {
                        read_end_data_point =
                            current_data_point + MAX_VALUES / non_energy_channels.max(1);
                    }
                    let values = self
                        .read_acquisition_data(current_data_point, read_end_data_point - 1)
                        .await
                        .unwrap_or_default();

                    if (values.len() as i32)
                        < (read_end_data_point - current_data_point) * non_energy_channels
                    {
                        let _ = self.command_response("Abort").await;
                        ok = false;
                        self.set_i32(self.ad.acquire, 0);
                        self.set_i32(self.ad.status, ADStatus::Error as i32);
                        self.set_str(self.ad.status_message, "SPECS Receive Error, see log");
                        continue;
                    }

                    let mut index = 0usize;
                    for y in 0..non_energy_channels.max(0) {
                        if num_data_points > energy_channels {
                            let _ = self.command_response("Abort").await;
                            ok = false;
                            self.set_i32(self.ad.acquire, 0);
                            self.set_i32(self.ad.status, ADStatus::Error as i32);
                            self.set_str(
                                self.ad.status_message,
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
                            self.p.acq_spectrum,
                            spectrum[..current_data_point.max(0) as usize].to_vec(),
                        );
                    } else {
                        self.set_f64_array(self.p.acq_spectrum, spectrum.clone());
                    }
                    self.set_f64_array(self.p.acq_image, image.clone());

                    let pct_iter = if energy_channels > 0 {
                        (current_data_point * 100) / energy_channels
                    } else {
                        0
                    };
                    self.set_i32(self.p.percent_complete_iteration, pct_iter);
                    let total = iterations * energy_channels;
                    let pct_total = if total > 0 {
                        ((current_data_point + iteration * energy_channels) * 100) / total
                    } else {
                        0
                    };
                    self.set_i32(self.p.percent_complete, pct_total);
                    self.set_i32(self.p.current_sample_iteration, current_data_point);
                    self.set_i32(
                        self.p.current_sample,
                        current_data_point + iteration * energy_channels,
                    );

                    // specsAnalyser.cpp:614-615 re-reads ADAcquireTime fresh
                    // here (distinct from the `acquireTime` captured once
                    // before the iteration loop) so a client changing it
                    // mid-scan is reflected in the estimate immediately.
                    let current_acquire_time = self.get_f64(self.ad.acquire_time).await;
                    let remaining_iter =
                        ((energy_channels - current_data_point) as f64) * current_acquire_time;
                    self.set_f64(self.p.remaining_time_iteration, remaining_iter);
                    let remaining_total = ((energy_channels - current_data_point
                        + (iterations - (iteration + 1)) * energy_channels)
                        as f64)
                        * current_acquire_time;
                    self.set_f64(self.p.remaining_time, remaining_total);

                    self.callbacks().await;
                }

                acquire = self.get_i32(self.ad.acquire).await;
            }

            if last_data.get("ControllerState").map(String::as_str) == Some("error") {
                ok = false;
                self.set_i32(self.ad.acquire, 0);
                self.set_i32(self.ad.status, ADStatus::Error as i32);
                match last_data.get("Message").filter(|m| !m.is_empty()) {
                    Some(msg) => self.set_str(self.ad.status_message, msg.clone()),
                    None => self.set_str(self.ad.status_message, "SPECS Controller Error, see log"),
                }
            }

            acquire = self.get_i32(self.ad.acquire).await;
            iteration += 1;
        }
        let _ = &last_data;

        let array_counter = self.get_i32(self.ad.base.array_counter).await + 1;
        let num_images_counter = self.get_i32(self.ad.num_images_counter).await + 1;
        self.set_i32(self.ad.base.array_counter, array_counter);
        self.set_i32(self.ad.num_images_counter, num_images_counter);

        let array_callbacks = self.get_i32(self.ad.base.array_callbacks).await;
        if array_callbacks != 0 {
            let mut array = NDArray::with_data(
                vec![NDDimension::new(nx), NDDimension::new(ny)],
                NDDataBuffer::F64(image),
            );
            array.unique_id = array_counter;
            array.time_stamp = acq_start_epoch;
            array.timestamp = EpicsTimestamp::now();
            self.output.publish(Arc::new(array)).await;
        }

        self.last_status_ok = ok;
        if ok {
            if image_mode == 0 /* Single */ || (image_mode == 1 /* Multiple */ && num_images_counter >= num_images)
            {
                self.set_i32(self.ad.acquire, 0);
            }
            self.callbacks().await;
            acquire = self.get_i32(self.ad.acquire).await;

            if acquire != 0 {
                let elapsed = acq_start_instant.elapsed().as_secs_f64();
                let delay = acquire_period - elapsed;
                if delay >= 0.0 {
                    self.set_i32(self.ad.status, ADStatus::Waiting as i32);
                    self.callbacks().await;
                    self.stop.wait_timeout(secs(delay));
                }
            }
        }
    }

    /// C `specsAnalyserTask`'s outer `while(1)` (`specsAnalyser.cpp:300-433`).
    /// The preamble only runs when `ADAcquire` reads 0 at the top of the
    /// pass; otherwise (e.g. `ImageMode::Continuous`, which never resets
    /// `ADAcquire` between batches) it falls straight through to rerun the
    /// acquisition body against the previous pass's `Prepared` state.
    pub async fn run(mut self) {
        let mut prepared: Option<Prepared> = None;
        loop {
            let acquire = self.get_i32(self.ad.acquire).await;
            if acquire == 0 {
                prepared = self.wait_and_prepare().await;
                if prepared.is_none() {
                    continue;
                }
            } else {
                match prepared.as_mut() {
                    Some(p) => p.acquire = acquire,
                    // ADAcquire was already 1 with nothing ever prepared —
                    // only reachable if the IOC starts up with ADAcquire=1,
                    // a state C itself has no derivable behaviour for
                    // (energyChannels/image/spectrum are all still
                    // zero/NULL at that point). Wait for a real start
                    // signal instead of running against unprepared state.
                    None => continue,
                }
            }
            self.run_acquisition(prepared.expect("checked above")).await;
        }
    }
}

/// Spawn the acquisition worker thread (C `epicsThreadCreate(...,
/// specsAnalyserTaskC, ...)`).
pub(crate) fn start_task(w: Worker) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("specsAnalyserTask", move || w.run())
}
