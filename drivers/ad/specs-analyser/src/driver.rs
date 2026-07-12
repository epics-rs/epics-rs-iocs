//! `PortDriver`/`ADDriver` implementation: `SpecsAnalyser`'s constructor
//! (`specsAnalyser.cpp:44-178`), `writeInt32`/`writeFloat64`
//! (`specsAnalyser.cpp:770-971`), and `specsAnalyserConfig`'s equivalent
//! runtime wiring.
//!
//! `write_int32`/`write_float64` run inside the port actor's own
//! `current_thread` runtime, which cannot block on a second port (the
//! `driverPort_` connection). Every branch that needs a wire round trip
//! therefore spawns a plain OS thread with no tokio runtime of its own (so
//! `PortHandle::submit_blocking` inside `wire::WireLink` is safe to call
//! from it), `.join()`s it synchronously (safe from a plain thread with no
//! active runtime — the same reasoning that makes joining safe from the
//! port actor's own dispatch thread), then applies the result via `&mut
//! self` after the join returns. The persistent acquisition worker
//! (`task::Worker`) is the only place wire I/O happens `.await`-based.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::{EnumEntry, ParamType};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex as PlMutex;

use crate::codec::{DefineFatArgs, DefineFrrArgs};
use crate::params::SpecsParams;
use crate::task::{Worker, start_task};
use crate::types::{Event, RunMode};
use crate::wire::{self, DiscoveredValue, WireLink};

fn enum_choices(values: &[String]) -> Arc<[EnumEntry]> {
    values
        .iter()
        .enumerate()
        .map(|(i, s)| EnumEntry {
            string: s.clone(),
            value: i as i32,
            severity: 0,
        })
        .collect()
}

pub struct SpecsAnalyserDriver {
    pub ad: ADDriverBase,
    pub p: SpecsParams,
    /// C `paramIndexes_`: function (asyn reason) -> EPICS parameter name.
    param_indexes: HashMap<usize, String>,
    /// C `paramMap_`: EPICS parameter name -> raw device parameter name.
    param_map: HashMap<String, String>,
    lens_modes: Arc<Mutex<Vec<String>>>,
    scan_ranges: Arc<Mutex<Vec<String>>>,
    wire_state: Arc<Mutex<u32>>,
    connected: Arc<AtomicBool>,
    driver_port: PortHandle,
    driver_addr: i32,
    /// C `firstConnect_`.
    first_connect: bool,
    start: Arc<Event>,
    stop: Arc<Event>,
}

/// Everything a `write_int32`/`write_float64` branch needs from a
/// `setAnalyserParameter` + re-confirm round trip.
struct SetConfirmOutcome<T> {
    ok: bool,
    confirmed: T,
}

/// Shared handles threaded through both `SpecsAnalyserDriver` (`with_wire`,
/// ADAcquire start/stop signalling) and the acquisition `task::Worker` —
/// grouped into one bundle so `SpecsAnalyserDriver::new` takes one parameter
/// for them instead of four.
pub struct SharedWireHandles {
    pub wire_state: Arc<Mutex<u32>>,
    pub connected: Arc<AtomicBool>,
    pub start: Arc<Event>,
    pub stop: Arc<Event>,
}

impl SpecsAnalyserDriver {
    fn new(
        port_name: &str,
        driver_port: PortHandle,
        driver_addr: i32,
        max_buffers: i32,
        max_memory: usize,
        shared: SharedWireHandles,
    ) -> AsynResult<Self> {
        let SharedWireHandles {
            wire_state,
            connected,
            start,
            stop,
        } = shared;
        let mut ad = ADDriverBase::new(port_name, 0, 0, max_memory)?;
        let _ = max_buffers;
        let p = SpecsParams::create(&mut ad.port_base)?;
        let base = &mut ad.port_base;

        // C constructor defaults (specsAnalyser.cpp:126-138).
        base.set_int32_param(p.connected, 0, 0)?;
        base.set_int32_param(p.pause_acq, 0, 0)?;
        base.set_int32_param(p.msg_counter, 0, 0)?;
        base.set_int32_param(p.percent_complete, 0, 0)?;
        base.set_int32_param(p.current_sample, 0, 0)?;
        base.set_int32_param(p.snapshot_values, 0, 1)?;
        base.set_int32_param(p.samples_iteration, 0, 0)?;
        base.set_int32_param(p.percent_complete_iteration, 0, 0)?;
        base.set_int32_param(p.current_sample_iteration, 0, 0)?;
        base.set_float64_param(p.remaining_time, 0, 0.0)?;
        base.set_string_param(ad.params.base.manufacturer, 0, "SPECS".into())?;
        base.set_int32_param(p.safe_state, 0, 1)?;
        base.set_float64_param(p.data_delay_max, 0, 5.0)?;

        Ok(Self {
            ad,
            p,
            param_indexes: HashMap::new(),
            param_map: HashMap::new(),
            lens_modes: lens_modes_default(),
            scan_ranges: lens_modes_default(),
            wire_state,
            connected,
            driver_port,
            driver_addr,
            first_connect: true,
            start,
            stop,
        })
    }

    fn with_wire<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut WireLink) -> R + Send + 'static,
        R: Send + 'static,
    {
        let driver_port = self.driver_port.clone();
        let addr = self.driver_addr;
        let wire_state = self.wire_state.clone();
        let connected = self.connected.clone();
        std::thread::spawn(move || {
            let mut link = WireLink {
                driver_port: &driver_port,
                addr,
                wire_state: &wire_state,
                connected: &connected,
            };
            f(&mut link)
        })
        .join()
        .expect("specs-analyser wire thread panicked")
    }

    /// C `SpecsAnalyser::makeConnection` + the always-run
    /// `readSpectrumParameter`/`readRunModes` calls that both the
    /// constructor and `writeInt32`'s `SPECSConnect_` case perform
    /// (`specsAnalyser.cpp:153-169`, `798-814`).
    fn reconnect(&mut self) -> bool {
        let first_connect = self.first_connect;
        let outcome = self.with_wire(move |link| {
            let mut ok = link.connect().is_ok();
            let mut visible_name = None;
            let mut discovered = Vec::new();
            let mut non_energy_channels = None;
            if ok && first_connect {
                match link.read_device_visible_name() {
                    Ok(name) => visible_name = Some(name),
                    Err(_) => ok = false,
                }
                if ok {
                    match link.setup_epics_parameters() {
                        Ok(params) => discovered = params,
                        Err(_) => ok = false,
                    }
                }
                if ok {
                    non_energy_channels =
                        link.get_analyser_parameter_int("NumNonEnergyChannels").ok();
                }
            }
            let (lens_modes, _) = link.read_spectrum_parameter("LensMode");
            let (scan_ranges, _) = link.read_spectrum_parameter("ScanRange");
            let run_modes = wire::read_run_modes();
            ReconnectOutcome {
                ok,
                visible_name,
                discovered,
                non_energy_channels,
                lens_modes,
                scan_ranges,
                run_modes,
            }
        });

        if outcome.ok {
            if let Some(name) = outcome.visible_name {
                let _ = self
                    .ad
                    .port_base
                    .set_string_param(self.ad.params.base.model, 0, name);
            }
            for param in outcome.discovered {
                let reason = match &param.value {
                    DiscoveredValue::Int(_) => self
                        .ad
                        .port_base
                        .create_param(&param.epics_name, ParamType::Int32),
                    DiscoveredValue::Double(_) => self
                        .ad
                        .port_base
                        .create_param(&param.epics_name, ParamType::Float64),
                    DiscoveredValue::String(_) => self
                        .ad
                        .port_base
                        .create_param(&param.epics_name, ParamType::Octet),
                };
                let Ok(reason) = reason else { continue };
                match param.value {
                    DiscoveredValue::Int(Some(v)) => {
                        let _ = self.ad.port_base.set_int32_param(reason, 0, v);
                    }
                    DiscoveredValue::Double(Some(v)) => {
                        let _ = self.ad.port_base.set_float64_param(reason, 0, v);
                    }
                    DiscoveredValue::String(Some(v)) => {
                        let _ = self.ad.port_base.set_string_param(reason, 0, v);
                    }
                    _ => {}
                }
                self.param_indexes.insert(reason, param.epics_name.clone());
                self.param_map.insert(param.epics_name, param.raw_name);
            }
            if let Some(v) = outcome.non_energy_channels {
                let _ = self
                    .ad
                    .port_base
                    .set_int32_param(self.p.non_energy_channels, 0, v);
            }
            self.first_connect = false;
        }

        *self.lens_modes.lock().unwrap() = outcome.lens_modes.clone();
        *self.scan_ranges.lock().unwrap() = outcome.scan_ranges.clone();
        let _ = self.ad.port_base.set_enum_choices_param(
            self.p.lens_mode,
            0,
            enum_choices(&outcome.lens_modes),
        );
        let _ = self.ad.port_base.set_enum_choices_param(
            self.p.scan_range,
            0,
            enum_choices(&outcome.scan_ranges),
        );
        let _ = self.ad.port_base.set_enum_choices_param(
            self.p.run_mode,
            0,
            enum_choices(&outcome.run_modes),
        );

        outcome.ok
    }

    /// `SpecsAnalyser::writeInt32`'s `SPECSDefine_` case
    /// (`specsAnalyser.cpp:815-832`).
    fn dispatch_define(&mut self) {
        let Some(mode) = RunMode::from_i32(
            self.ad
                .port_base
                .get_int32_param(self.p.run_mode, 0)
                .unwrap_or(0),
        ) else {
            return;
        };
        let start_energy = self
            .ad
            .port_base
            .get_float64_param(self.p.start_energy, 0)
            .unwrap_or(0.0);
        let end_energy = self
            .ad
            .port_base
            .get_float64_param(self.p.end_energy, 0)
            .unwrap_or(0.0);
        let step_width = self
            .ad
            .port_base
            .get_float64_param(self.p.step_width, 0)
            .unwrap_or(0.0);
        let dwell_time = self
            .ad
            .port_base
            .get_float64_param(self.ad.params.acquire_time, 0)
            .unwrap_or(0.0);
        let pass_energy = self
            .ad
            .port_base
            .get_float64_param(self.p.pass_energy, 0)
            .unwrap_or(0.0);
        let retarding_ratio = self
            .ad
            .port_base
            .get_float64_param(self.p.retarding_ratio, 0)
            .unwrap_or(0.0);
        let kinetic_energy = self
            .ad
            .port_base
            .get_float64_param(self.p.kinetic_energy, 0)
            .unwrap_or(0.0);
        let snapshot_values = self
            .ad
            .port_base
            .get_int32_param(self.p.snapshot_values, 0)
            .unwrap_or(0);
        let lens_mode_idx = self
            .ad
            .port_base
            .get_int32_param(self.p.lens_mode, 0)
            .unwrap_or(0);
        let scan_range_idx = self
            .ad
            .port_base
            .get_int32_param(self.p.scan_range, 0)
            .unwrap_or(0);
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

        let _ = self.with_wire(move |link| match mode {
            RunMode::Fat => link
                .define_fat(
                    DefineFatArgs {
                        start_energy,
                        end_energy,
                        step_width,
                        dwell_time,
                        pass_energy,
                    },
                    &lens_mode,
                    &scan_range,
                )
                .map(|_| ()),
            RunMode::Sfat => link.define_sfat(
                start_energy,
                end_energy,
                snapshot_values,
                dwell_time,
                &lens_mode,
                &scan_range,
            ),
            RunMode::Frr => link
                .define_frr(
                    DefineFrrArgs {
                        start_energy,
                        end_energy,
                        step_width,
                        dwell_time,
                        retarding_ratio,
                    },
                    &lens_mode,
                    &scan_range,
                )
                .map(|_| ()),
            RunMode::Fe => link.define_fe(
                kinetic_energy,
                snapshot_values,
                dwell_time,
                pass_energy,
                &lens_mode,
                &scan_range,
            ),
        });
    }

    /// `SpecsAnalyser::writeInt32`'s `SPECSValidate_` case, i.e. a direct
    /// call to `validateSpectrum()` (`specsAnalyser.cpp:833-835`,
    /// `979-1073`) outside the acquisition loop — no `SPECSSamples_` total
    /// and no SFAT recompute, both of which are `specsAnalyserTask`-only.
    fn dispatch_validate(&mut self) {
        let run_mode = RunMode::from_i32(
            self.ad
                .port_base
                .get_int32_param(self.p.run_mode, 0)
                .unwrap_or(0),
        );
        let kinetic_energy = self
            .ad
            .port_base
            .get_float64_param(self.p.kinetic_energy, 0)
            .unwrap_or(0.0);
        let lens_modes = self.lens_modes.lock().unwrap().clone();
        let scan_ranges = self.scan_ranges.lock().unwrap().clone();
        let result = self.with_wire(move |link| {
            link.validate_spectrum(run_mode, kinetic_energy, &lens_modes, &scan_ranges)
        });
        if let Ok(r) = result {
            let _ = self
                .ad
                .port_base
                .set_float64_param(self.p.start_energy, 0, r.start_energy);
            let _ = self
                .ad
                .port_base
                .set_float64_param(self.p.end_energy, 0, r.end_energy);
            let _ = self
                .ad
                .port_base
                .set_float64_param(self.p.step_width, 0, r.step_width);
            let _ = self
                .ad
                .port_base
                .set_int32_param(self.p.samples_iteration, 0, r.samples);
            let _ =
                self.ad
                    .port_base
                    .set_float64_param(self.ad.params.acquire_time, 0, r.dwell_time);
            let _ = self
                .ad
                .port_base
                .set_float64_param(self.p.pass_energy, 0, r.pass_energy);
            let _ = self
                .ad
                .port_base
                .set_int32_param(self.p.lens_mode, 0, r.lens_mode);
            let _ = self
                .ad
                .port_base
                .set_int32_param(self.p.scan_range, 0, r.scan_range);
        }
    }

    /// The `setAnalyserParameter` + re-confirm-or-revert pattern shared by
    /// `SPECSNonEnergyChannels_` (`specsAnalyser.cpp:865-875`) and the
    /// dynamic `paramIndexes_` fallback (`specsAnalyser.cpp:886-902`).
    fn set_confirm_int(&self, raw_name: String, value: i32) -> SetConfirmOutcome<i32> {
        self.with_wire(move |link| {
            let ok = link.set_analyser_parameter_int(&raw_name, value).is_ok();
            let confirmed = if ok {
                link.get_analyser_parameter_int(&raw_name).ok()
            } else {
                None
            };
            match confirmed {
                Some(v) => SetConfirmOutcome {
                    ok: true,
                    confirmed: v,
                },
                None => SetConfirmOutcome {
                    ok: false,
                    confirmed: value,
                },
            }
        })
    }

    fn set_confirm_double(&self, raw_name: String, value: f64) -> SetConfirmOutcome<f64> {
        self.with_wire(move |link| {
            let ok = link.set_analyser_parameter_double(&raw_name, value).is_ok();
            let confirmed = if ok {
                link.get_analyser_parameter_double(&raw_name).ok()
            } else {
                None
            };
            match confirmed {
                Some(v) => SetConfirmOutcome {
                    ok: true,
                    confirmed: v,
                },
                None => SetConfirmOutcome {
                    ok: false,
                    confirmed: value,
                },
            }
        })
    }
}

struct ReconnectOutcome {
    ok: bool,
    visible_name: Option<String>,
    discovered: Vec<wire::DiscoveredParam>,
    non_energy_channels: Option<i32>,
    lens_modes: Vec<String>,
    scan_ranges: Vec<String>,
    run_modes: Vec<String>,
}

fn lens_modes_default() -> Arc<Mutex<Vec<String>>> {
    Arc::new(Mutex::new(Vec::new()))
}

impl PortDriver for SpecsAnalyserDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let function = user.reason;
        let addr = user.addr;
        let old_value = self
            .ad
            .port_base
            .get_int32_param(function, addr)
            .unwrap_or(0);
        self.ad.port_base.set_int32_param(function, addr, value)?;
        let adstatus = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.status, addr)
            .unwrap_or(0);
        let idle_like = adstatus == ADStatus::Idle as i32
            || adstatus == ADStatus::Error as i32
            || adstatus == ADStatus::Aborted as i32;

        let mut result: AsynResult<()> = Ok(());

        if function == self.ad.params.acquire {
            if value != 0 && idle_like {
                self.start.signal();
            }
            if value == 0 && adstatus != ADStatus::Idle as i32 {
                self.ad
                    .port_base
                    .set_int32_param(self.p.pause_acq, addr, 0)?;
                self.ad.port_base.set_int32_param(
                    self.ad.params.status,
                    addr,
                    ADStatus::Aborted as i32,
                )?;
                self.stop.signal();
                let _ = self.with_wire(|link| link.command_response("Abort"));
            }
        } else if function == self.p.connect {
            let connected = self
                .ad
                .port_base
                .get_int32_param(self.p.connected, addr)
                .unwrap_or(0);
            if connected == 0 {
                self.reconnect();
            }
        } else if function == self.p.define {
            self.dispatch_define();
        } else if function == self.p.validate {
            self.dispatch_validate();
        } else if function == self.p.pause_acq {
            let acquire = self
                .ad
                .port_base
                .get_int32_param(self.ad.params.acquire, addr)
                .unwrap_or(0);
            if value == 1 {
                if old_value == 0 && acquire == 1 {
                    let _ = self.with_wire(|link| link.command_response("Pause"));
                    self.ad.port_base.set_string_param(
                        self.ad.params.status_message,
                        addr,
                        "Acquisition paused".into(),
                    )?;
                } else {
                    self.ad
                        .port_base
                        .set_int32_param(self.p.pause_acq, addr, old_value)?;
                }
            } else if value == 0 {
                if old_value == 1 && acquire == 1 {
                    let _ = self.with_wire(|link| link.command_response("Resume"));
                    self.ad.port_base.set_string_param(
                        self.ad.params.status_message,
                        addr,
                        "Acquiring data...".into(),
                    )?;
                } else {
                    self.ad
                        .port_base
                        .set_int32_param(self.p.pause_acq, addr, old_value)?;
                }
            }
        } else if function == self.p.non_energy_channels {
            let outcome = self.set_confirm_int("NumNonEnergyChannels".to_string(), value);
            self.ad.port_base.set_int32_param(
                function,
                addr,
                if outcome.ok {
                    outcome.confirmed
                } else {
                    old_value
                },
            )?;
        } else if function == self.p.safe_state && value != 0 {
            if idle_like {
                let _ = self.with_wire(|link| link.command_response("SetSafeState"));
            } else {
                self.ad
                    .port_base
                    .set_int32_param(self.p.safe_state, addr, old_value)?;
                result = Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: "Unable to enter safe state while busy".to_string(),
                });
            }
        } else if let Some(raw_name) = self
            .param_indexes
            .get(&function)
            .and_then(|epics_name| self.param_map.get(epics_name))
            .cloned()
        {
            let outcome = self.set_confirm_int(raw_name, value);
            self.ad.port_base.set_int32_param(
                function,
                addr,
                if outcome.ok {
                    outcome.confirmed
                } else {
                    old_value
                },
            )?;
        }

        self.ad.port_base.call_param_callbacks(addr)?;
        result
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let function = user.reason;
        let addr = user.addr;
        let old_value = self
            .ad
            .port_base
            .get_float64_param(function, addr)
            .unwrap_or(0.0);
        self.ad.port_base.set_float64_param(function, addr, value)?;

        // C's writeFloat64 only ever dispatches to the dynamic
        // `paramIndexes_` fallback (specsAnalyser.cpp:927-971) — none of
        // the statically-created float params get special handling here.
        if let Some(raw_name) = self
            .param_indexes
            .get(&function)
            .and_then(|epics_name| self.param_map.get(epics_name))
            .cloned()
        {
            let outcome = self.set_confirm_double(raw_name, value);
            self.ad.port_base.set_float64_param(
                function,
                addr,
                if outcome.ok {
                    outcome.confirmed
                } else {
                    old_value
                },
            )?;
        }

        self.ad.port_base.call_param_callbacks(addr)?;
        Ok(())
    }
}

impl ADDriver for SpecsAnalyserDriver {
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

pub struct SpecsAnalyserRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub specs_params: SpecsParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<PlMutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    pub tasks: Vec<std::thread::JoinHandle<()>>,
}

impl SpecsAnalyserRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<PlMutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.array_output.lock().add(sender);
    }
}

/// `specsAnalyserConfig`'s equivalent. `driver_port_name` must already exist
/// (`drvAsynIPPortConfigure` + `asynOctetSetInputEos`/`asynOctetSetOutputEos`
/// in `st.cmd`) — C connects to it with `pasynOctetSyncIO->connect` in the
/// constructor; here the EOS is configured entirely by `st.cmd`.
pub fn create_specs_analyser_detector(
    port_name: &str,
    driver_port_name: &str,
    max_buffers: i32,
    max_memory: usize,
) -> Result<SpecsAnalyserRuntime, String> {
    let driver_port_entry = get_port(driver_port_name).ok_or_else(|| {
        format!("driver port '{driver_port_name}' not found (call drvAsynIPPortConfigure first)")
    })?;
    let driver_port = driver_port_entry.handle.clone();

    let wire_state = Arc::new(Mutex::new(0u32));
    let connected = Arc::new(AtomicBool::new(false));
    let start = Arc::new(Event::new());
    let stop = Arc::new(Event::new());

    let mut driver = SpecsAnalyserDriver::new(
        port_name,
        driver_port.clone(),
        0,
        max_buffers,
        max_memory,
        SharedWireHandles {
            wire_state: wire_state.clone(),
            connected: connected.clone(),
            start: start.clone(),
            stop: stop.clone(),
        },
    )
    .map_err(|e| format!("failed to create specs-analyser driver: {e}"))?;

    // C constructor: attempt the initial connection synchronously before
    // the acquisition task and the actor are even created
    // (specsAnalyser.cpp:153-169). Failure is not terminal (upstream's own
    // commented-out early-return, specsAnalyser.cpp:156-162).
    if !driver.reconnect() {
        let _ =
            driver
                .ad
                .port_base
                .set_int32_param(driver.ad.params.status, 0, ADStatus::Error as i32);
        let _ = driver.ad.port_base.set_string_param(
            driver.ad.params.status_message,
            0,
            "Failed to initialise - check connection".into(),
        );
    }

    let ad_params = driver.ad.params;
    let specs_params = driver.p;
    let lens_modes = driver.lens_modes.clone();
    let scan_ranges = driver.scan_ranges.clone();
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(driver, RuntimeConfig::default());
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(PlMutex::new(NDArrayOutput::new()));
    let queued_counter = Arc::new(QueuedArrayCounter::new());

    let worker = Worker {
        self_handle: handle.clone(),
        driver_port,
        addr: 0,
        wire_state,
        connected,
        p: specs_params,
        ad: ad_params,
        start,
        stop,
        lens_modes,
        scan_ranges,
        output: ArrayPublisher::new(array_output.clone()),
        batch: Vec::new(),
        last_status_ok: true,
    };
    let tasks = vec![start_task(worker)];

    Ok(SpecsAnalyserRuntime {
        runtime_handle,
        ad_params,
        specs_params,
        pool,
        array_output,
        queued_counter,
        tasks,
    })
}
