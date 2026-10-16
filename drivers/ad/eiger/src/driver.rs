//! The Eiger asyn port driver (port of `eigerDetector`, eigerDetector.cpp).

use std::sync::Arc;

use epics_rs::ad_core::driver::{ADDriverBase, ADStatus, ImageMode};
use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::param::{AsynType, ParamOps, ParamRegistry, ParamUpdate};
use crate::params::{
    self, EigerParams, Model, SOURCE_STREAM, TRIGGER_MODE_EXTE, TRIGGER_MODE_INTE,
};
use crate::rest::{ApiVersion, RestApi, RestError, Sys};
use crate::tasks::Signals;
use crate::trace::Trace;

/// The file permissions the driver is willing to set (C `value & 0666`).
const FILE_PERMS_MASK: i32 = 0o666;

pub struct EigerDriver {
    pub ad: ADDriverBase,
    pub p: EigerParams,
    pub model: Model,
    pub api: ApiVersion,
    pub ops: Arc<ParamOps>,
    signals: Signals,
}

/// asyn address of the Monitor interface's NDArray callbacks (C
/// `MONITOR_ASYN_ADDRESS`).
pub const MONITOR_ASYN_ADDRESS: usize = 10;
/// C `MAX_ASYN_ADDRESS`, the port's maxAddr.
pub const MAX_ASYN_ADDRESS: usize = MONITOR_ASYN_ADDRESS + 1;

/// Everything [`EigerDriver::new`] needs that is not read from the detector.
pub struct EigerConfig {
    pub port_name: String,
    pub api: ApiVersion,
    pub model: Model,
    pub max_size_x: i32,
    pub max_size_y: i32,
    pub max_memory: usize,
}

impl EigerDriver {
    pub fn new(cfg: EigerConfig, rest: RestApi, signals: Signals) -> AsynResult<Self> {
        let EigerConfig {
            port_name,
            api,
            model,
            max_size_x,
            max_size_y,
            max_memory,
        } = cfg;
        let mut ad = ADDriverBase::new(
            &port_name,
            MAX_ASYN_ADDRESS,
            max_size_x,
            max_size_y,
            max_memory,
            PortFlags {
                multi_device: true,
                ..ASYN_CANBLOCK
            },
        )?;

        let mut reg = ParamRegistry::new();
        let p = params::create(&mut ad.port_base, &mut reg, api, model)?;
        let ops = Arc::new(ParamOps::new(rest, reg, Trace::new(&port_name)));

        let mut driver = Self {
            ad,
            p,
            model,
            api,
            ops,
            signals,
        };
        driver.init_params()?;
        Ok(driver)
    }

    /// Probe a detector that is marked disconnected and, if it answers, re-read
    /// everything the constructor reads.
    ///
    /// The only way back to connected. C has none: a detector that was off when
    /// the IOC booted keeps its parameters at whatever `initParams` left them
    /// (eigerDetector.cpp:229-235) until the IOC restarts.
    pub fn reconnect(&mut self) -> AsynResult<()> {
        // A detector that comes back speaking another API keeps the parameter
        // set this port was created with; only an IOC restart rebuilds it.
        self.ops.rest.probe().map_err(rest_err)?;
        self.init_params()
    }

    /// Fetch every parameter and apply the driver's fixed defaults
    /// (C `eigerDetector::initParams`, eigerDetector.cpp:1621).
    fn init_params(&mut self) -> AsynResult<()> {
        self.set_driver_version()?;

        // Cleared here because this runs again on every reconnect: a detector
        // that came back must not leave the operator reading "FAILED TO
        // CONNECT" off a working port. The initialization below may set it again.
        self.status_message("")?;

        // C tests the detector with a single fetch of `state` before it touches
        // anything else, and abandons the rest of the constructor when that
        // fails (eigerDetector.cpp:229-235).
        if self.fetch(self.p.state).is_err() {
            self.ops.trace.err(
                "eigerDetector",
                "Cannot fetch state. Eiger could be disconnected.",
            );
            return self.boot_disconnected();
        }
        if !self.initialize_if_needed()? {
            return self.ad.port_base.call_param_callbacks(0);
        }
        // Only parameter-library failures reach here, as only they reach C's
        // `initParams` status (eigerDetector.cpp:425-429).
        if self.fetch_and_default().is_err() {
            self.ops
                .trace
                .err("eigerDetector", "unable to set detector parameters");
        }
        self.ad.port_base.call_param_callbacks(0)
    }

    /// A detector reporting `state` "na" has not been initialized since power
    /// on: initialize it and arm/disarm once (eigerDetector.cpp:239-263).
    /// Returns false when the initialize failed, where C abandons the rest of
    /// the constructor.
    fn initialize_if_needed(&mut self) -> AsynResult<bool> {
        let state = self
            .ad
            .port_base
            .get_string_param(self.p.state, 0)
            .unwrap_or_default();
        if state != b"na" {
            return Ok(true);
        }
        let trace = self.ops.trace.clone();
        trace.err(
            "eigerDetector",
            "Eiger seems to be uninitialized\nInitializing... (may take a while)",
        );
        if self.ops.rest.initialize().is_err() {
            trace.err("eigerDetector", "Eiger FAILED TO INITIALIZE");
            self.status_message("Eiger FAILED TO INITIALIZE")?;
            return Ok(false);
        }
        // C stores the arm's sequence id here, but `initParams` zeroes it
        // before anything can read it (eigerDetector.cpp:1660).
        if self.ops.rest.arm().is_err() {
            trace.err(
                "eigerDetector",
                "Failed to arm the detector for the first time",
            );
            self.status_message("Eiger failed to arm for the first time")?;
        } else {
            let _ = self.ops.rest.disarm();
        }
        Ok(true)
    }

    /// Report the detector as unusable while keeping the port, so the IOC still
    /// reaches `iocInit` (C sets `ADStatusMessage` and returns,
    /// eigerDetector.cpp:231-234).
    fn boot_disconnected(&mut self) -> AsynResult<()> {
        self.status_message("Eiger FAILED TO CONNECT")?;
        self.ad.port_base.set_int32_param(
            self.ad.params.status,
            0,
            ADStatus::Disconnected as i32,
        )?;
        self.ad.port_base.call_param_callbacks(0)
    }

    /// The body of C `eigerDetector::initParams` (eigerDetector.cpp:1621).
    ///
    /// C issues these requests and looks at none of their return codes — a
    /// failed one prints its own lines and changes no status — so the `?`s
    /// here are all parameter-library operations, which never touch the
    /// detector.
    fn fetch_and_default(&mut self) -> AsynResult<()> {
        let (updates, _failures) = self.ops.fetch_all();
        self.apply(updates)?;

        // Read the sensor size with the ROI disabled — that is the maximum.
        // Writing roi_mode invalidates x/y_pixels_in_detector, and `put_int`
        // re-fetches every DetConfig parameter the reply names, so the sizes
        // below are the ones that belong to the mode in force.
        //
        // `roi_mode` is an enum: the parameter library holds its index, only
        // the detector speaks the names, so the comparison and the restore both
        // stay in the index domain. Asking the library for a string here — as
        // this did — is a type error against an `asynParamInt32`, which failed
        // `init_params` for every detector, reachable or not.
        let roi_mode = self
            .ad
            .port_base
            .get_int32_param(self.p.roi_mode, 0)
            .unwrap_or(0);
        let restore = match self.ops.enum_index_of(self.p.roi_mode, "disabled") {
            Some(disabled) if disabled != roi_mode => {
                let _ = self.put_int(self.p.roi_mode, disabled);
                true
            }
            _ => false,
        };
        let max_size_x = self
            .ad
            .port_base
            .get_int32_param(self.p.nd_array_size_x, 0)
            .unwrap_or(0);
        let max_size_y = self
            .ad
            .port_base
            .get_int32_param(self.p.nd_array_size_y, 0)
            .unwrap_or(0);
        if restore {
            let _ = self.put_int(self.p.roi_mode, roi_mode);
        }

        let base = &mut self.ad.port_base;
        base.set_int32_param(self.ad.params.max_size_x, 0, max_size_x)?;
        base.set_int32_param(self.ad.params.max_size_y, 0, max_size_y)?;

        // The description is "<manufacturer> <model>". C ORs its status into
        // the return and still sets every default below (eigerDetector.cpp:1646).
        let (description, described) = match base.get_string_param(self.p.description, 0) {
            Ok(bytes) => (String::from_utf8_lossy(bytes).into_owned(), Ok(())),
            Err(e) => (String::new(), Err(e)),
        };
        let (manufacturer, model) = match description.split_once(' ') {
            Some((m, rest)) => (m.to_string(), rest.to_string()),
            None => (description.clone(), String::new()),
        };
        base.set_string_param(self.ad.params.base.manufacturer, 0, manufacturer)?;
        base.set_string_param(self.ad.params.base.model, 0, model)?;

        base.set_int32_param(self.ad.params.base.array_size, 0, 0)?;
        base.set_int32_param(self.ad.params.image_mode, 0, ImageMode::Multiple as i32)?;
        base.set_int32_param(self.ad.params.status, 0, ADStatus::Idle as i32)?;

        // Driver-only defaults (C, eigerDetector.cpp:1659-1673).
        base.set_int32_param(self.p.armed, 0, 0)?;
        base.set_int32_param(self.p.sequence_id, 0, 0)?;
        base.set_int32_param(self.p.pending_files, 0, 0)?;
        base.set_int32_param(self.p.monitor_timeout, 0, 500)?;
        base.set_string_param(self.p.file_owner, 0, String::new())?;
        base.set_string_param(self.p.file_owner_group, 0, String::new())?;
        base.set_int32_param(self.p.file_perms, 0, 0o644)?;

        // The monitor interface starts disabled: it is a background poll that
        // costs the detector work, and the driver only wants it on request.
        //
        // UPSTREAM DEFECT (eigerParam.cpp:226 + eigerDetector.cpp:1662): C's
        // `put(bool)` indexes the enum with `!value`, so `mMonitorEnable->put(
        // false)` writes `enum_values[1]` — "enabled" — and the monitor is
        // *switched on* by the very line meant to switch it off. See
        // `param::encode_bool`, which inverts nothing.
        let _ = self.put_bool(self.p.monitor_enable, false);

        // Values this driver requires to be constant.
        let _ = self.put_bool(self.p.auto_summation, true);
        let _ = self.put_int(self.p.fw_img_num_start, params::DEFAULT_NR_START);
        let _ = self.put_int(self.p.monitor_buf_size, 1);
        described
    }

    fn apply(&mut self, updates: Vec<ParamUpdate>) -> AsynResult<()> {
        for u in updates {
            match u {
                ParamUpdate::Int32(i, v) => self.ad.port_base.set_int32_param(i, 0, v)?,
                ParamUpdate::Float64(i, v) => self.ad.port_base.set_float64_param(i, 0, v)?,
                ParamUpdate::Octet(i, v) => self.ad.port_base.set_string_param(i, 0, v)?,
            }
        }
        Ok(())
    }

    fn string_param(&self, index: usize) -> String {
        String::from_utf8_lossy(
            self.ad
                .port_base
                .get_string_param(index, 0)
                .unwrap_or_default(),
        )
        .into_owned()
    }

    fn put_int(&mut self, index: usize, value: i32) -> AsynResult<()> {
        let updates = self.ops.put_int(index, value).map_err(rest_err)?;
        self.apply(updates)
    }

    fn put_bool(&mut self, index: usize, value: bool) -> AsynResult<()> {
        let updates = self.ops.put_bool(index, value).map_err(rest_err)?;
        self.apply(updates)
    }

    fn put_f64(&mut self, index: usize, value: f64) -> AsynResult<()> {
        let current = self.ad.port_base.get_float64_param(index, 0).unwrap_or(0.0);
        let updates = self.ops.put_f64(index, value, current).map_err(rest_err)?;
        self.apply(updates)
    }

    fn put_str(&mut self, index: usize, value: &str) -> AsynResult<()> {
        let updates = self.ops.put_str(index, value).map_err(rest_err)?;
        self.apply(updates)
    }

    fn fetch(&mut self, index: usize) -> AsynResult<()> {
        let updates = self.ops.fetch(index).map_err(rest_err)?;
        self.apply(updates)
    }

    /// C sets `NDDriverVersion` before it first talks to the detector
    /// (eigerDetector.cpp:214), so it is filled in even on a port that never
    /// reaches the rest of the constructor.
    fn set_driver_version(&mut self) -> AsynResult<()> {
        self.ad.port_base.set_string_param(
            self.ad.params.base.driver_version,
            0,
            env!("CARGO_PKG_VERSION"),
        )
    }

    fn status_message(&mut self, msg: &str) -> AsynResult<()> {
        self.ad
            .port_base
            .set_string_param(self.ad.params.status_message, 0, msg)
    }

    /// Refresh the interesting status parameters (C `eigerStatus`,
    /// eigerDetector.cpp:2006).
    fn eiger_status(&mut self) -> AsynResult<()> {
        // While acquiring, the status poll would fight the control task for the
        // detector's attention; C returns immediately and so does this.
        if self
            .ad
            .port_base
            .get_int32_param(self.ad.params.acquire, 0)
            .unwrap_or(0)
            != 0
        {
            return Ok(());
        }

        if self.api == ApiVersion::V1_6_0 {
            self.ops.rest.status_update().map_err(rest_err)?;
        }

        let mut indices = vec![
            self.p.state,
            self.p.error,
            self.p.th_temp0,
            self.p.th_humid0,
        ];
        if self.api == ApiVersion::V1_6_0 {
            indices.extend([self.p.link0, self.p.link1].into_iter().flatten());
            // The Eiger 500K has no link2/link3.
            let model = self.string_param(self.ad.params.base.model);
            if !model.contains("500K") {
                indices.extend([self.p.link2, self.p.link3].into_iter().flatten());
            }
            indices.extend(self.p.dcu_buf_free);
        }
        if self.model.has_thresholds_1_2() {
            indices.extend(self.p.hv_state);
        }
        indices.extend([
            self.p.fw_state,
            self.p.monitor_state,
            self.p.stream_state,
            self.p.stream_dropped,
            self.p.fw_free,
        ]);

        let mut first_err = None;
        for index in indices {
            if let Err(e) = self.fetch(index) {
                first_err.get_or_insert(e);
            }
        }
        self.ad.port_base.call_param_callbacks(0)?;
        match first_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// `writeInt32` up to its closing lines.
    fn write_int32_status(&mut self, user: &AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let acquire = self.ad.params.acquire;
        let ad_status = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.status, 0)
            .unwrap_or(0);

        if reason == acquire {
            if value != 0 && ad_status != ADStatus::Acquire as i32 {
                // Starting an acquisition is the one moment worth spending a
                // real request on a detector that is not answering: the
                // operator asked for data, and every parameter the acquisition
                // needs was left at its default by the constructor that could
                // not read them.
                if !self.ops.rest.is_connected() {
                    self.reconnect()?;
                }
                self.ad.port_base.set_int32_param(
                    self.ad.params.status,
                    0,
                    ADStatus::Acquire as i32,
                )?;
                self.signals.start();
            } else if value == 0 && ad_status == ADStatus::Acquire as i32 {
                self.ad.port_base.set_int32_param(
                    self.ad.params.status,
                    0,
                    ADStatus::Aborted as i32,
                )?;
                // C ignores mApi.abort()'s status: the stop is signalled anyway.
                let _ = self.ops.rest.abort();
                self.signals.stop();
            }
            self.ad.port_base.set_int32_param(acquire, 0, value)?;
        } else if Some(reason) == self.p.fw_clear {
            self.put_int(reason, 1)?;
            let _ = self.fetch(self.p.fw_free);
        } else if reason == self.ad.params.read_status {
            self.eiger_status()?;
        } else if reason == self.p.restart && value == 1 {
            self.ad.port_base.set_int32_param(reason, 0, 1)?;
            self.signals.restart();
        } else if reason == self.p.initialize && value == 1 {
            self.ad.port_base.set_int32_param(reason, 0, 1)?;
            self.signals.initialize();
        } else if reason == self.p.trigger {
            self.signals.trigger();
        } else if reason == self.p.file_perms {
            self.put_int(reason, value & FILE_PERMS_MASK)?;
        } else if Some(reason) == self.p.hv_reset {
            let reset_time = self
                .ad
                .port_base
                .get_float64_param(
                    self.p
                        .hv_reset_time
                        .expect("hv_reset implies hv_reset_time"),
                    0,
                )
                .unwrap_or(0.0);
            let _ = self.ops.rest.hv_reset(reset_time as i32);
        } else if reason == self.p.trigger_mode {
            // INTE and EXTE deliver exactly one image per trigger.
            if value == TRIGGER_MODE_INTE || value == TRIGGER_MODE_EXTE {
                let _ = self.put_int(self.p.num_images, 1);
            }
            self.put_int(reason, value)?;
        } else if self.ops.reg.lock().contains(reason) {
            self.put_int(reason, value)?;
            if reason == self.p.data_source || reason == self.p.stream_version {
                let data_source = self
                    .ad
                    .port_base
                    .get_int32_param(self.p.data_source, 0)
                    .unwrap_or(0);
                // Selecting the stream re-arms the detector's stream interface:
                // it has to be off and on again for the socket to deliver.
                if data_source == SOURCE_STREAM {
                    let version = self
                        .ad
                        .port_base
                        .get_int32_param(self.p.stream_version, 0)
                        .unwrap_or(0);
                    self.signals.stream_apis(Some(version));
                    let _ = self.put_bool(self.p.stream_enable, false);
                    let _ = self.put_bool(self.p.stream_enable, true);
                } else {
                    self.signals.stream_apis(None);
                }
            }
        } else {
            self.ad.port_base.set_int32_param(reason, addr, value)?;
        }

        Ok(())
    }

    /// `writeFloat64` up to its closing lines.
    fn write_float64_status(&mut self, user: &AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;

        if reason == self.p.photon_energy {
            let _ = self.status_message("Setting Photon Energy...");
            let _ = self.ad.port_base.call_param_callbacks(0);
            let _ = self.put_f64(reason, value);
            let _ = self.status_message("Photon Energy set");
        } else if reason == self.p.threshold {
            let _ = self.status_message("Setting Threshold Energy...");
            let _ = self.ad.port_base.call_param_callbacks(0);
            let _ = self.put_f64(reason, value);
            let _ = self.status_message("Threshold Energy set");
        } else if reason == self.p.wavelength {
            let _ = self.status_message("Setting Wavelength...");
            let _ = self.ad.port_base.call_param_callbacks(0);
            let _ = self.put_f64(reason, value);
            let _ = self.status_message("Wavelength set");
        } else if reason == self.p.wavelength_epsilon {
            let _ = self.ad.port_base.set_float64_param(reason, 0, value);
            self.ops.reg.lock().set_epsilon(self.p.wavelength, value);
        } else if reason == self.p.energy_epsilon {
            let _ = self.ad.port_base.set_float64_param(reason, 0, value);
            let mut reg = self.ops.reg.lock();
            reg.set_epsilon(self.p.photon_energy, value);
            for t in params::thresholds(&self.p) {
                reg.set_epsilon(t.energy, value);
            }
        } else if self.ops.reg.lock().contains(reason) {
            self.put_f64(reason, value)?;
        } else {
            self.ad.port_base.set_float64_param(reason, addr, value)?;
        }

        Ok(())
    }

    /// `writeOctet` up to its closing lines.
    fn write_octet_status(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<()> {
        let reason = user.reason;
        let value = String::from_utf8_lossy(data);

        // The C driver resolves FILE_OWNER / FILE_OWNER_GROUP to a uid/gid with
        // getpwnam/getgrnam and then setfsuid()s to it around every write.
        //
        // Not ported: see `tasks::save_task`. The two parameters are kept as
        // plain strings so the PV surface is unchanged.
        if self.ops.reg.lock().contains(reason)
            && reason != self.p.file_owner
            && reason != self.p.file_owner_group
        {
            self.put_str(reason, &value)
        } else {
            self.ad.write_octet(user, data).map(drop)
        }
    }
}

fn rest_err(e: RestError) -> AsynError {
    asyn_err(e.to_string())
}

/// An asyn error carrying a driver-level message.
pub(crate) fn asyn_err(message: String) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message,
    }
}

impl PortDriver for EigerDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    /// Create an `EIG_XYZ_name` parameter on demand (C `drvUserCreate`,
    /// eigerDetector.cpp:2074).
    ///
    /// ```text
    /// EIG_XYZ_name
    ///     X  D detector | F filewriter | M monitor | S stream
    ///     Y  C config   | S status
    ///     Z  I int32    | D float64    | S octet
    /// ```
    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        let drv_info = req.drv_info.as_str();
        if let Some(reason) = self.ad.port_base.params.find_param(drv_info) {
            return Ok(DrvUserInfo::from_reason(reason));
        }
        if !drv_info.starts_with("EIG_") || drv_info.len() <= 8 {
            return Err(AsynError::ParamNotFound(drv_info.to_string()));
        }

        let sys = Sys::from_drv_info_code(&drv_info[4..6]).ok_or_else(|| {
            asyn_err(format!(
                "[{drv_info}] couldn't match {} to any subsystem",
                &drv_info[4..6]
            ))
        })?;
        let asyn_type = match drv_info.as_bytes()[6] {
            b'I' => AsynType::Int32,
            b'D' => AsynType::Float64,
            b'S' => AsynType::Octet,
            c => {
                return Err(asyn_err(format!(
                    "[{drv_info}] couldn't match {} to an asyn type",
                    c as char
                )));
            }
        };
        let remote_name = drv_info[8..].to_string();

        let param_type = match asyn_type {
            AsynType::Int32 => epics_rs::asyn::param::ParamType::Int32,
            AsynType::Float64 => epics_rs::asyn::param::ParamType::Float64,
            AsynType::Octet => epics_rs::asyn::param::ParamType::Octet,
        };
        let index = self.ad.port_base.create_param(drv_info, param_type)?;
        self.ops
            .reg
            .lock()
            .add(index, drv_info, asyn_type, sys, &remote_name);

        // C `p->fetch()`: its failure lines are the parameter layer's.
        let _ = self.fetch(index);
        Ok(DrvUserInfo::from_reason(index))
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let function = user.reason;
        if let Err(e) = self.write_int32_status(user, value) {
            // C's ERR_ARGS, on pasynUserSelf (eigerDetector.cpp:590-594).
            self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "eigerDetector::writeInt32: error status={} function={function}, value={value}",
                    e.status() as i32
                ),
            );
            return Err(e);
        }
        let _ = self.ad.port_base.call_param_callbacks(user.addr);
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("eigerDetector:writeInt32: function={function}, value={value}"),
        );
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let function = user.reason;
        let status = self.write_float64_status(user, value);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "eigerDetector:writeFloat64 error, status={} function={function}, value={value:.6}",
                    e.status() as i32
                ),
            ),
            Ok(()) => {
                user.print(
                    TraceMask::IO_DRIVER,
                    file!(),
                    line!(),
                    format_args!("eigerDetector:writeFloat64: function={function}, value={value:.6}"),
                );
                let _ = self.ad.port_base.call_param_callbacks(user.addr);
            }
        }
        status
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let function = user.reason;
        let status = self.write_octet_status(user, data);
        // C prints the value as the `char *` it was handed: up to the NUL.
        let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
        let value = String::from_utf8_lossy(&data[..end]);
        match &status {
            // C's error line has no newline (eigerDetector.cpp:763-766).
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "eigerDetector:writeOctet: status={}, function={function}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => {
                user.print(
                    TraceMask::IO_DRIVER,
                    file!(),
                    line!(),
                    format_args!("eigerDetector:writeOctet: function={function}, value={value}"),
                );
                let _ = self.ad.port_base.call_param_callbacks(user.addr);
            }
        }
        status.map(|()| data.len())
    }
}
