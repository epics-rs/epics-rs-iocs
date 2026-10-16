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
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::param::{EnumEntry, ParamType};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use parking_lot::Mutex as PlMutex;

use crate::codec::{DefineFatArgs, DefineFrrArgs};
use crate::params::SpecsParams;
use crate::task::{Worker, start_task};
use crate::types::{Event, RunMode, SOCKET_TIMEOUT, secs};
use crate::wire::{self, DiscoveredValue, ExchangeTrail, WireLink};

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
    driver_port: Arc<SyncIOHandle>,
    driver_addr: i32,
    /// C `firstConnect_`.
    first_connect: bool,
    start: Arc<Event>,
    stop: Arc<Event>,
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
        driver_port: Arc<SyncIOHandle>,
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
        let mut ad = ADDriverBase::new(port_name, 1, 0, 0, max_memory, ASYN_CANBLOCK)?;
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
        base.set_string_param(ad.params.base.manufacturer, 0, "SPECS")?;
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

    /// Runs `f`'s exchanges, then sets what they left in the parameter
    /// library. The port is held meanwhile, so the counter C posts before
    /// each exchange is posted once, with the write's own callbacks.
    fn with_wire<F, R>(&mut self, f: F) -> R
    where
        F: FnOnce(&mut WireLink) -> R + Send + 'static,
        R: Send + 'static,
    {
        let driver_port = self.driver_port.clone();
        let addr = self.driver_addr;
        let wire_state = self.wire_state.clone();
        let connected = self.connected.clone();
        let (result, trail) = std::thread::spawn(move || {
            let mut link = WireLink {
                driver_port: &driver_port,
                addr,
                wire_state: &wire_state,
                connected: &connected,
                trail: ExchangeTrail::default(),
            };
            let result = f(&mut link);
            (result, link.trail)
        })
        .join()
        .expect("specs-analyser wire thread panicked");
        let base = &mut self.ad.port_base;
        if let Some(counter) = trail.msg_counter {
            let _ = base.set_int32_param(self.p.msg_counter, 0, counter as i32);
        }
        if let Some(message) = trail.status_message {
            let _ = base.set_string_param(self.ad.params.status_message, 0, message);
        }
        result
    }

    /// C `SpecsAnalyser::makeConnection` + the always-run
    /// `readSpectrumParameter`/`readRunModes` calls that both the
    /// constructor and `writeInt32`'s `SPECSConnect_` case perform
    /// (`specsAnalyser.cpp:153-169`, `798-814`). The status is the first
    /// failing step's: C ORs the step statuses, which can only differ from
    /// that by mixing two different failure codes into a third.
    fn reconnect(&mut self) -> AsynResult<()> {
        let first_connect = self.first_connect;
        let outcome = self.with_wire(move |link| {
            let mut connection = link.connect().map(|_| ());
            let mut visible_name = None;
            let mut discovered = Vec::new();
            let mut non_energy_channels = None;
            if connection.is_ok() && first_connect {
                connection = link.read_device_visible_name().map(|name| {
                    visible_name = Some(name);
                });
                if connection.is_ok() {
                    connection = link.setup_epics_parameters().map(|params| {
                        discovered = params;
                    });
                }
                if connection.is_ok() {
                    non_energy_channels =
                        link.get_analyser_parameter_int("NumNonEnergyChannels").ok();
                }
            }
            let lens_modes = link.read_spectrum_parameter("LensMode");
            let scan_ranges = link.read_spectrum_parameter("ScanRange");
            let run_modes = wire::read_run_modes();
            ReconnectOutcome {
                connection,
                visible_name,
                discovered,
                non_energy_channels,
                lens_modes,
                scan_ranges,
                run_modes,
            }
        });

        if outcome.connection.is_ok() {
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

        // C keeps one entry on a failed read so the mbbi/mbbo SDEF is set.
        let or_not_connected = |values: &Result<Vec<String>, wire::WireError>| match values {
            Ok(values) => values.clone(),
            Err(_) => vec!["Not connected".to_string()],
        };
        let lens_modes = or_not_connected(&outcome.lens_modes);
        let scan_ranges = or_not_connected(&outcome.scan_ranges);
        let _ = self.ad.port_base.set_enum_choices_param(
            self.p.lens_mode,
            0,
            enum_choices(&lens_modes),
        );
        let _ = self.ad.port_base.set_enum_choices_param(
            self.p.scan_range,
            0,
            enum_choices(&scan_ranges),
        );
        *self.lens_modes.lock().unwrap() = lens_modes;
        *self.scan_ranges.lock().unwrap() = scan_ranges;
        let _ = self.ad.port_base.set_enum_choices_param(
            self.p.run_mode,
            0,
            enum_choices(&outcome.run_modes),
        );

        outcome.connection?;
        outcome.lens_modes?;
        outcome.scan_ranges?;
        Ok(())
    }

    /// `SpecsAnalyser::writeInt32`'s `SPECSDefine_` case
    /// (`specsAnalyser.cpp:815-832`).
    fn dispatch_define(&mut self) -> AsynResult<()> {
        let Some(mode) = RunMode::from_i32(
            self.ad
                .port_base
                .get_int32_param(self.p.run_mode, 0)
                .unwrap_or(0),
        ) else {
            return Ok(());
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

        self.with_wire(move |link| match mode {
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
        })?;
        Ok(())
    }

    /// `SpecsAnalyser::writeInt32`'s `SPECSValidate_` case, i.e. a direct
    /// call to `validateSpectrum()` (`specsAnalyser.cpp:833-835`,
    /// `979-1073`) outside the acquisition loop — no `SPECSSamples_` total
    /// and no SFAT recompute, both of which are `specsAnalyserTask`-only.
    fn dispatch_validate(&mut self) -> AsynResult<()> {
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
        let r = self.with_wire(move |link| {
            link.validate_spectrum(run_mode, kinetic_energy, &lens_modes, &scan_ranges)
        })?;
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
        let _ = self
            .ad
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
        Ok(())
    }

    /// The `setAnalyserParameter` + re-confirm pattern shared by
    /// `SPECSNonEnergyChannels_` (`specsAnalyser.cpp:865-875`) and the
    /// dynamic `paramIndexes_` fallback (`specsAnalyser.cpp:886-902`); the
    /// caller reverts to the old value on `Err`.
    fn set_confirm_int(&mut self, raw_name: String, value: i32) -> AsynResult<i32> {
        self.with_wire(move |link| {
            link.set_analyser_parameter_int(&raw_name, value)?;
            link.get_analyser_parameter_int(&raw_name)
        })
        .map_err(AsynError::from)
    }

    fn set_confirm_double(&mut self, raw_name: String, value: f64) -> AsynResult<f64> {
        self.with_wire(move |link| {
            link.set_analyser_parameter_double(&raw_name, value)?;
            link.get_analyser_parameter_double(&raw_name)
        })
        .map_err(AsynError::from)
    }
}

struct ReconnectOutcome {
    /// `makeConnection`'s status.
    connection: Result<(), wire::WireError>,
    visible_name: Option<String>,
    discovered: Vec<wire::DiscoveredParam>,
    non_energy_channels: Option<i32>,
    lens_modes: Result<Vec<String>, wire::WireError>,
    scan_ranges: Result<Vec<String>, wire::WireError>,
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

    /// C inherits `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.ad.write_octet(user, data)
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let function = user.reason;
        let old_value = self.ad.port_base.get_int32_param(function, 0).unwrap_or(0);
        let _ = self.ad.port_base.set_int32_param(function, 0, value);
        let adstatus = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.status, 0)
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
                let _ = self.ad.port_base.set_int32_param(self.p.pause_acq, 0, 0);
                let _ = self.ad.port_base.set_int32_param(
                    self.ad.params.status,
                    0,
                    ADStatus::Aborted as i32,
                );
                self.stop.signal();
                let _ = self.with_wire(|link| link.command_response("Abort"));
            }
        } else if function == self.p.connect {
            let connected = self
                .ad
                .port_base
                .get_int32_param(self.p.connected, 0)
                .unwrap_or(0);
            if connected == 0 {
                result = self.reconnect();
            }
        } else if function == self.p.define {
            result = self.dispatch_define();
        } else if function == self.p.validate {
            result = self.dispatch_validate();
        } else if function == self.p.pause_acq {
            let acquire = self
                .ad
                .port_base
                .get_int32_param(self.ad.params.acquire, 0)
                .unwrap_or(0);
            if value == 1 {
                if old_value == 0 && acquire == 1 {
                    let _ = self.with_wire(|link| link.command_response("Pause"));
                    let _ = self.ad.port_base.set_string_param(
                        self.ad.params.status_message,
                        0,
                        "Acquisition paused",
                    );
                } else {
                    let _ = self
                        .ad
                        .port_base
                        .set_int32_param(self.p.pause_acq, 0, old_value);
                }
            } else if value == 0 {
                if old_value == 1 && acquire == 1 {
                    let _ = self.with_wire(|link| link.command_response("Resume"));
                    let _ = self.ad.port_base.set_string_param(
                        self.ad.params.status_message,
                        0,
                        "Acquiring data...",
                    );
                } else {
                    let _ = self
                        .ad
                        .port_base
                        .set_int32_param(self.p.pause_acq, 0, old_value);
                }
            }
        } else if function == self.p.non_energy_channels {
            let confirmed = self.set_confirm_int("NumNonEnergyChannels".to_string(), value);
            let _ = self.ad.port_base.set_int32_param(
                function,
                0,
                *confirmed.as_ref().unwrap_or(&old_value),
            );
            result = confirmed.map(|_| ());
        } else if function == self.p.safe_state && value != 0 {
            if idle_like {
                let _ = self.with_wire(|link| link.command_response("SetSafeState"));
            } else {
                let _ = self
                    .ad
                    .port_base
                    .set_int32_param(self.p.safe_state, 0, old_value);
                // C only passes the refusal to debug() (specsAnalyser.cpp:883).
                result = Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: String::new(),
                });
            }
        } else if let Some(raw_name) = self
            .param_indexes
            .get(&function)
            .and_then(|epics_name| self.param_map.get(epics_name))
            .cloned()
        {
            let confirmed = self.set_confirm_int(raw_name, value);
            let _ = self.ad.port_base.set_int32_param(
                function,
                0,
                *confirmed.as_ref().unwrap_or(&old_value),
            );
            result = confirmed.map(|_| ());
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        result
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let function = user.reason;
        let old_value = self
            .ad
            .port_base
            .get_float64_param(function, 0)
            .unwrap_or(0.0);
        let _ = self.ad.port_base.set_float64_param(function, 0, value);
        let mut result: AsynResult<()> = Ok(());

        // C's writeFloat64 only ever dispatches to the dynamic
        // `paramIndexes_` fallback (specsAnalyser.cpp:927-971) — none of
        // the statically-created float params get special handling here.
        if let Some(raw_name) = self
            .param_indexes
            .get(&function)
            .and_then(|epics_name| self.param_map.get(epics_name))
            .cloned()
        {
            let confirmed = self.set_confirm_double(raw_name, value);
            let _ = self.ad.port_base.set_float64_param(
                function,
                0,
                *confirmed.as_ref().unwrap_or(&old_value),
            );
            result = confirmed.map(|_| ());
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        result
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
    pool: DriverPool,
    array_output: Arc<PlMutex<NDArrayOutput>>,
    pub tasks: Vec<std::thread::JoinHandle<()>>,
}

impl SpecsAnalyserRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<PlMutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
        self.array_output.lock().add(sender);
    }
}

/// `specsAnalyserConfig`'s equivalent. `driver_port_name` is created by
/// `drvAsynIPPortConfigure` + `asynOctetSetInputEos`/`asynOctetSetOutputEos`
/// in `st.cmd` — C connects to it with `pasynOctetSyncIO->connect` in the
/// constructor; here the EOS is configured entirely by `st.cmd`. A missing
/// port keeps C's unconnected user, so the detector comes up failed to
/// initialise.
pub fn create_specs_analyser_detector(
    claim: &PortClaim,
    driver_port_name: &str,
    max_buffers: i32,
    max_memory: usize,
) -> Result<SpecsAnalyserRuntime, String> {
    let port_name = claim.name();
    // C's debug() trace of the failure is off by default.
    let (driver_port, _) = SyncIOHandle::connect_port(
        driver_port_name,
        0,
        secs(SOCKET_TIMEOUT),
        InterfaceType::Octet,
    );
    let driver_port = Arc::new(driver_port);

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
    if driver.reconnect().is_err() {
        let _ =
            driver
                .ad
                .port_base
                .set_int32_param(driver.ad.params.status, 0, ADStatus::Error as i32);
        let _ = driver.ad.port_base.set_string_param(
            driver.ad.params.status_message,
            0,
            "Failed to initialise - check connection",
        );
    }

    let ad_params = driver.ad.params;
    let specs_params = driver.p;
    let lens_modes = driver.lens_modes.clone();
    let scan_ranges = driver.scan_ranges.clone();
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the SPECS analyser port runtime: {e}"))?;
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(PlMutex::new(NDArrayOutput::new()));

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
        pool: pool.clone(),
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
        tasks,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A driver whose `driverPort_` does not exist, so no write reaches a wire.
    fn offline(name: &str) -> SpecsAnalyserDriver {
        let (driver_port, _) = SyncIOHandle::connect_port(
            &format!("{name}_NONE"),
            0,
            secs(SOCKET_TIMEOUT),
            InterfaceType::Octet,
        );
        SpecsAnalyserDriver::new(
            name,
            Arc::new(driver_port),
            0,
            0,
            1 << 20,
            SharedWireHandles {
                wire_state: Arc::new(Mutex::new(0)),
                connected: Arc::new(AtomicBool::new(false)),
                start: Arc::new(Event::new()),
                stop: Arc::new(Event::new()),
            },
        )
        .unwrap()
    }

    /// C refuses SafeState while busy with asynError and no errorMessage
    /// (specsAnalyser.cpp:880-885).
    #[test]
    fn a_busy_safe_state_is_refused_with_no_message() {
        let mut driver = offline("SPECS_SAFE");
        let (status, safe_state) = (driver.ad.params.status, driver.p.safe_state);
        driver
            .ad
            .port_base
            .set_int32_param(status, 0, ADStatus::Acquire as i32)
            .unwrap();
        let e = driver
            .write_int32(&mut AsynUser::new(safe_state), 1)
            .unwrap_err();
        assert_eq!(
            (e.status(), e.message()),
            (AsynStatus::Error, String::new())
        );
    }

    /// writeInt32/writeFloat64 set and post list 0 whatever the address
    /// (`specsAnalyser.cpp:781`, `938`), as `getAddr` gives a non-multidevice
    /// port's user.
    #[test]
    fn a_write_at_another_address_lands_on_list_zero() {
        let mut d = offline("SPECS_LIST0");
        let samples = d.p.snapshot_values;
        let mut user = AsynUser::new(samples).with_addr(1);
        d.write_int32(&mut user, 7).unwrap();
        assert_eq!(d.ad.port_base.get_int32_param(samples, 0).unwrap(), 7);
        let energy = d.p.kinetic_energy;
        let mut user = AsynUser::new(energy).with_addr(1);
        d.write_float64(&mut user, 2.5).unwrap();
        assert_eq!(d.ad.port_base.get_float64_param(energy, 0).unwrap(), 2.5);
    }
}
