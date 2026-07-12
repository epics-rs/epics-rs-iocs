//! The Eiger asyn port driver (port of `eigerDetector`, eigerDetector.cpp).

use std::sync::Arc;

use epics_rs::ad_core::driver::{ADDriverBase, ADStatus, ImageMode};
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{DrvUserInfo, PortDriver, PortDriverBase};
use epics_rs::asyn::user::AsynUser;

use crate::param::{AsynType, ParamOps, ParamRegistry, ParamUpdate};
use crate::params::{
    self, EigerParams, Model, SOURCE_STREAM, TRIGGER_MODE_EXTE, TRIGGER_MODE_INTE,
};
use crate::rest::{ApiVersion, RestApi, RestError, Sys};
use crate::tasks::Signals;

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
        let mut ad = ADDriverBase::new(&port_name, max_size_x, max_size_y, max_memory)?;

        let mut reg = ParamRegistry::new();
        let p = params::create(&mut ad.port_base, &mut reg, api, model)?;
        let ops = Arc::new(ParamOps::new(rest, reg));

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

    /// Fetch every parameter and apply the driver's fixed defaults
    /// (C `eigerDetector::initParams`, eigerDetector.cpp:1621).
    fn init_params(&mut self) -> AsynResult<()> {
        let (updates, failures) = self.ops.fetch_all();
        if failures > 0 {
            log::warn!("eiger: {failures} parameter(s) failed to fetch at startup");
        }
        self.apply(updates)?;

        // Read the sensor size with the ROI disabled — that is the maximum.
        // Writing roi_mode invalidates x/y_pixels_in_detector, and `put_str`
        // re-fetches every DetConfig parameter the reply names, so the sizes
        // below are the ones that belong to the mode in force.
        let roi_mode = self.string_param(self.p.roi_mode)?;
        if roi_mode != "disabled" {
            self.put_str(self.p.roi_mode, "disabled")?;
        }
        let max_size_x = self
            .ad
            .port_base
            .get_int32_param(self.p.nd_array_size_x, 0)?;
        let max_size_y = self
            .ad
            .port_base
            .get_int32_param(self.p.nd_array_size_y, 0)?;
        if roi_mode != "disabled" {
            self.put_str(self.p.roi_mode, &roi_mode)?;
        }

        let base = &mut self.ad.port_base;
        base.set_int32_param(self.ad.params.max_size_x, 0, max_size_x)?;
        base.set_int32_param(self.ad.params.max_size_y, 0, max_size_y)?;

        // The description is "<manufacturer> <model>".
        let description = base.params.get_string(self.p.description, 0)?.to_string();
        let (manufacturer, model) = match description.split_once(' ') {
            Some((m, rest)) => (m.to_string(), rest.to_string()),
            None => (description.clone(), String::new()),
        };
        base.set_string_param(self.ad.params.base.manufacturer, 0, manufacturer)?;
        base.set_string_param(self.ad.params.base.model, 0, model)?;
        base.set_string_param(
            self.ad.params.base.driver_version,
            0,
            env!("CARGO_PKG_VERSION").into(),
        )?;

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
        self.put_bool(self.p.monitor_enable, false)?;

        // Values this driver requires to be constant.
        self.put_bool(self.p.auto_summation, true)?;
        self.put_int(self.p.fw_img_num_start, params::DEFAULT_NR_START)?;
        self.put_int(self.p.monitor_buf_size, 1)?;

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn apply(&mut self, updates: Vec<ParamUpdate>) -> AsynResult<()> {
        for u in updates {
            match u {
                ParamUpdate::Int32(i, v) => self.ad.port_base.params.set_int32(i, 0, v)?,
                ParamUpdate::Float64(i, v) => self.ad.port_base.params.set_float64(i, 0, v)?,
                ParamUpdate::Octet(i, v) => self.ad.port_base.params.set_string(i, 0, v)?,
            }
        }
        Ok(())
    }

    fn string_param(&self, index: usize) -> AsynResult<String> {
        self.ad
            .port_base
            .params
            .get_string(index, 0)
            .map(str::to_string)
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
        let current = self
            .ad
            .port_base
            .params
            .get_float64(index, 0)
            .unwrap_or(f64::NAN);
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

    fn status_message(&mut self, msg: &str) -> AsynResult<()> {
        self.ad
            .port_base
            .set_string_param(self.ad.params.status_message, 0, msg.into())
    }

    /// Refresh the interesting status parameters (C `eigerStatus`,
    /// eigerDetector.cpp:2006).
    fn eiger_status(&mut self) -> AsynResult<()> {
        // While acquiring, the status poll would fight the control task for the
        // detector's attention; C returns immediately and so does this.
        if self
            .ad
            .port_base
            .get_int32_param(self.ad.params.acquire, 0)?
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
            let model = self.string_param(self.ad.params.base.model)?;
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
    fn drv_user_create(&mut self, drv_info: &str, _addr: i32) -> AsynResult<DrvUserInfo> {
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

        if let Err(e) = self.fetch(index) {
            log::warn!("eiger: [{drv_info}] initial fetch failed: {e}");
        }
        Ok(DrvUserInfo::from_reason(index))
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let acquire = self.ad.params.acquire;
        let ad_status = self
            .ad
            .port_base
            .get_int32_param(self.ad.params.status, 0)?;

        if reason == acquire {
            if value != 0 && ad_status != ADStatus::Acquire as i32 {
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
                self.ops.rest.abort().map_err(rest_err)?;
                self.signals.stop();
            }
            self.ad.port_base.set_int32_param(acquire, 0, value)?;
        } else if Some(reason) == self.p.fw_clear {
            self.put_int(reason, 1)?;
            self.fetch(self.p.fw_free)?;
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
            let reset_time = self.ad.port_base.params.get_float64(
                self.p
                    .hv_reset_time
                    .expect("hv_reset implies hv_reset_time"),
                0,
            )?;
            self.ops
                .rest
                .hv_reset(reset_time as i32)
                .map_err(rest_err)?;
        } else if reason == self.p.trigger_mode {
            // INTE and EXTE deliver exactly one image per trigger.
            if value == TRIGGER_MODE_INTE || value == TRIGGER_MODE_EXTE {
                self.put_int(self.p.num_images, 1)?;
            }
            self.put_int(reason, value)?;
        } else if self.ops.reg.lock().contains(reason) {
            self.put_int(reason, value)?;
            if reason == self.p.data_source || reason == self.p.stream_version {
                let data_source = self.ad.port_base.get_int32_param(self.p.data_source, 0)?;
                // Selecting the stream re-arms the detector's stream interface:
                // it has to be off and on again for the socket to deliver.
                if data_source == SOURCE_STREAM {
                    self.put_bool(self.p.stream_enable, false)?;
                    self.put_bool(self.p.stream_enable, true)?;
                }
            }
        } else {
            self.ad.port_base.params.set_int32(reason, addr, value)?;
        }

        self.ad.port_base.call_param_callbacks(addr)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;

        if reason == self.p.photon_energy {
            self.status_message("Setting Photon Energy...")?;
            self.ad.port_base.call_param_callbacks(0)?;
            self.put_f64(reason, value)?;
            self.status_message("Photon Energy set")?;
        } else if reason == self.p.threshold {
            self.status_message("Setting Threshold Energy...")?;
            self.ad.port_base.call_param_callbacks(0)?;
            self.put_f64(reason, value)?;
            self.status_message("Threshold Energy set")?;
        } else if reason == self.p.wavelength {
            self.status_message("Setting Wavelength...")?;
            self.ad.port_base.call_param_callbacks(0)?;
            self.put_f64(reason, value)?;
            self.status_message("Wavelength set")?;
        } else if reason == self.p.wavelength_epsilon {
            self.ad.port_base.params.set_float64(reason, 0, value)?;
            self.ops.reg.lock().set_epsilon(self.p.wavelength, value);
        } else if reason == self.p.energy_epsilon {
            self.ad.port_base.params.set_float64(reason, 0, value)?;
            let mut reg = self.ops.reg.lock();
            reg.set_epsilon(self.p.photon_energy, value);
            for t in params::thresholds(&self.p) {
                reg.set_epsilon(t.energy, value);
            }
        } else if self.ops.reg.lock().contains(reason) {
            self.put_f64(reason, value)?;
        } else {
            self.ad.port_base.params.set_float64(reason, addr, value)?;
        }

        self.ad.port_base.call_param_callbacks(addr)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let addr = user.addr;
        let value = String::from_utf8_lossy(data).into_owned();

        // The C driver resolves FILE_OWNER / FILE_OWNER_GROUP to a uid/gid with
        // getpwnam/getgrnam and then setfsuid()s to it around every write.
        //
        // Not ported: see `tasks::save_task`. The two parameters are kept as
        // plain strings so the PV surface is unchanged.
        if self.ops.reg.lock().contains(reason)
            && reason != self.p.file_owner
            && reason != self.p.file_owner_group
        {
            self.put_str(reason, &value)?;
        } else {
            self.ad.port_base.params.set_string(reason, addr, value)?;
        }

        self.ad.port_base.call_param_callbacks(addr)?;
        Ok(data.len())
    }
}
