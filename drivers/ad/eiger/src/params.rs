//! The Eiger parameter set (port of the `mParams.create(...)` block in the
//! `eigerDetector` constructor, eigerDetector.cpp:226-422).
//!
//! Every parameter is created in the asyn param library *and* registered in the
//! [`ParamRegistry`], which is what binds an asyn index to its SIMPLON
//! subsystem and remote name. Base-class parameters (`ACQ_TIME`, `NIMAGES`, …)
//! resolve to the index `ADDriverParams` already created for them, so writing
//! `ADAcquireTime` reaches the detector's `count_time` exactly as in C.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

use crate::param::{AsynType, ParamRegistry};
use crate::rest::{ApiVersion, Sys};

/// Detector family, derived from the `description` config string
/// (C `eigerModel_t`, eigerDetector.cpp:292-296).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    Eiger1,
    Eiger2,
    Pilatus4,
}

impl Model {
    /// C matches both spellings of each family name.
    pub fn from_description(description: &str) -> Self {
        if description.contains("Eiger2") || description.contains("EIGER2") {
            Self::Eiger2
        } else if description.contains("Pilatus4") || description.contains("PILATUS4") {
            Self::Pilatus4
        } else {
            Self::Eiger1
        }
    }

    /// Eiger2 and Pilatus4 share the multi-threshold / high-voltage parameter
    /// set; Eiger1 has none of it.
    pub fn has_thresholds_1_2(self) -> bool {
        matches!(self, Self::Eiger2 | Self::Pilatus4)
    }

    pub fn has_thresholds_3_4(self) -> bool {
        self == Self::Pilatus4
    }
}

/// NDArray data source (C `data_source`, eigerDetector.h).
pub const SOURCE_NONE: i32 = 0;
pub const SOURCE_FILEWRITER: i32 = 1;
pub const SOURCE_STREAM: i32 = 2;

/// Trigger modes, in the order the mbbo record presents them
/// (C `trigger_mode`, eigerDetector.h).
pub const TRIGGER_MODE_INTS: i32 = 0;
pub const TRIGGER_MODE_INTE: i32 = 1;
pub const TRIGGER_MODE_EXTS: i32 = 2;
pub const TRIGGER_MODE_EXTE: i32 = 3;
pub const TRIGGER_MODE_CONTINUOUS: i32 = 4;
/// External gate — Eiger2 / Pilatus4 only (C `HAVE_EXTG_FIRMWARE`).
pub const TRIGGER_MODE_EXTG: i32 = 5;

/// The wire names of each trigger mode. Index 4 (continuous) is `ints` on the
/// wire too: continuous acquisition is internal-series triggering re-armed by
/// the control task (C, eigerDetector.cpp:354-358).
pub const TRIGGER_MODE_NAMES: [&str; 5] = ["ints", "inte", "exts", "exte", "ints"];
/// Eiger2 / Pilatus4 add external-gate triggering at index 5
/// (C, eigerDetector.cpp:359-363).
pub const TRIGGER_MODE_NAMES_EXTG: [&str; 6] = ["ints", "inte", "exts", "exte", "ints", "extg"];

/// Stream interface version (C `stream_version`).
pub const STREAM_VERSION_STREAM: i32 = 0;
pub const STREAM_VERSION_STREAM2: i32 = 1;

/// The first image number the FileWriter uses (C `DEFAULT_NR_START`).
pub const DEFAULT_NR_START: i32 = 1;

/// Highest threshold index that can carry its own NDArray stream.
pub const MAX_THRESHOLDS: usize = 4;

pub const WAVELENGTH_EPSILON: f64 = 0.0005;
pub const ENERGY_EPSILON: f64 = 0.05;

/// `disabled`/`enabled` — the SIMPLON `mode` parameters answer with an
/// `allowed_values` list whose order is not guaranteed, so C pins it
/// (eigerDetector.cpp:300-303).
const MODE_ENUM: [&str; 2] = ["disabled", "enabled"];
/// API 1.6.0 omits `allowed_values` on the link status parameters entirely.
const LINK_ENUM: [&str; 2] = ["down", "up"];

/// Every asyn index the driver refers to by name.
#[derive(Debug, Clone, Copy)]
pub struct EigerParams {
    // Driver-only
    pub data_source: usize,
    pub fw_auto_remove: usize,
    pub trigger: usize,
    pub manual_trigger: usize,
    pub armed: usize,
    pub sequence_id: usize,
    pub pending_files: usize,
    pub save_files: usize,
    pub file_owner: usize,
    pub file_owner_group: usize,
    pub file_perms: usize,
    pub monitor_timeout: usize,
    pub restart: usize,
    pub initialize: usize,
    pub stream_decompress: usize,
    pub wavelength_epsilon: usize,
    pub energy_epsilon: usize,
    pub signed_data: usize,
    pub stream_as_ts_source: usize,

    // Metadata
    pub description: usize,

    // Acquisition
    pub wavelength: usize,
    pub photon_energy: usize,
    pub threshold: usize,
    pub n_triggers: usize,
    pub compression_algo: usize,
    pub roi_mode: usize,
    pub auto_summation: usize,

    // Detector status
    pub state: usize,
    pub error: usize,
    pub th_temp0: usize,
    pub th_humid0: usize,

    // FileWriter
    pub fw_enable: usize,
    pub fw_compression: usize,
    pub fw_name_pattern: usize,
    pub fw_nimgs_per_file: usize,
    pub fw_img_num_start: usize,
    pub fw_state: usize,
    pub fw_free: usize,

    // Monitor
    pub monitor_enable: usize,
    pub monitor_buf_size: usize,
    pub monitor_state: usize,

    // Stream
    pub stream_enable: usize,
    pub stream_state: usize,
    pub stream_dropped: usize,
    pub stream_version: usize,

    // Base-class parameters bound to detector config
    pub acquire_time: usize,
    pub acquire_period: usize,
    pub num_images: usize,
    pub trigger_mode: usize,
    pub nd_array_size_x: usize,
    pub nd_array_size_y: usize,

    // API 1.6.0 only
    pub link0: Option<usize>,
    pub link1: Option<usize>,
    pub link2: Option<usize>,
    pub link3: Option<usize>,
    pub dcu_buf_free: Option<usize>,
    pub fw_clear: Option<usize>,

    // Eiger2 / Pilatus4
    pub threshold1_enable: Option<usize>,
    pub trigger_start_delay: Option<usize>,
    pub threshold2: Option<usize>,
    pub threshold2_enable: Option<usize>,
    pub threshold_diff_enable: Option<usize>,
    pub hv_state: Option<usize>,
    pub hv_reset_time: Option<usize>,
    pub hv_reset: Option<usize>,
    pub fw_hdf5_format: Option<usize>,
    pub ext_gate_mode: Option<usize>,
    pub num_exposures: Option<usize>,

    // Pilatus4 only
    pub threshold3: Option<usize>,
    pub threshold3_enable: Option<usize>,
    pub threshold4: Option<usize>,
    pub threshold4_enable: Option<usize>,
}

/// Create every parameter, in the asyn param library and in `reg`.
struct Builder<'a> {
    base: &'a mut PortDriverBase,
    reg: &'a mut ParamRegistry,
}

impl Builder<'_> {
    /// A parameter backed by a detector-side value.
    fn remote(&mut self, name: &str, ty: AsynType, sys: Sys, remote: &str) -> AsynResult<usize> {
        let param_type = match ty {
            AsynType::Int32 => ParamType::Int32,
            AsynType::Float64 => ParamType::Float64,
            AsynType::Octet => ParamType::Octet,
        };
        let index = self.base.create_param(name, param_type)?;
        self.reg.add(index, name, ty, sys, remote);
        Ok(index)
    }

    /// A parameter that lives only in the driver (C `create(name, type)` with
    /// no subsystem).
    fn local(&mut self, name: &str, ty: AsynType) -> AsynResult<usize> {
        self.remote(name, ty, Sys::DetConfig, "")
    }
}

/// Create the parameter set for one detector (C, eigerDetector.cpp:226-422).
pub fn create(
    base: &mut PortDriverBase,
    reg: &mut ParamRegistry,
    api: ApiVersion,
    model: Model,
) -> AsynResult<EigerParams> {
    let mut b = Builder { base, reg };

    use AsynType::{Float64, Int32, Octet};
    use Sys::{DetConfig as DC, DetStatus as DS};
    use Sys::{FwConfig as FC, FwStatus as FS};
    use Sys::{MonConfig as MC, MonStatus as MS};
    use Sys::{StreamConfig as SC, StreamStatus as SS};

    let state = b.remote("STATE", Octet, DS, "state")?;

    // Driver-only parameters.
    let data_source = b.local("DATA_SOURCE", Int32)?;
    let fw_auto_remove = b.local("AUTO_REMOVE", Int32)?;
    let trigger = b.local("TRIGGER", Int32)?;
    let manual_trigger = b.local("MANUAL_TRIGGER", Int32)?;
    let armed = b.local("ARMED", Int32)?;
    let sequence_id = b.local("SEQ_ID", Int32)?;
    let pending_files = b.local("PENDING_FILES", Int32)?;
    let save_files = b.local("SAVE_FILES", Int32)?;
    let file_owner = b.local("FILE_OWNER", Octet)?;
    let file_owner_group = b.local("FILE_OWNER_GROUP", Octet)?;
    let file_perms = b.local("FILE_PERMISSIONS", Int32)?;
    let monitor_timeout = b.local("MONITOR_TIMEOUT", Int32)?;
    let restart = b.local("RESTART", Int32)?;
    let initialize = b.local("INITIALIZE", Int32)?;
    let stream_decompress = b.local("STREAM_DECOMPRESS", Int32)?;
    let wavelength_epsilon = b.local("WAVELENGTH_EPSILON", Float64)?;
    let energy_epsilon = b.local("ENERGY_EPSILON", Float64)?;
    let signed_data = b.local("SIGNED_DATA", Int32)?;
    let stream_as_ts_source = b.local("STREAM_AS_TIMESTAMP_SOURCE", Int32)?;

    let description = b.remote("DESCRIPTION", Octet, DC, "description")?;

    // Acquisition.
    let wavelength = b.remote("WAVELENGTH", Float64, DC, "wavelength")?;
    b.reg.set_epsilon(wavelength, WAVELENGTH_EPSILON);
    let photon_energy = b.remote("PHOTON_ENERGY", Float64, DC, "photon_energy")?;
    b.reg.set_epsilon(photon_energy, ENERGY_EPSILON);
    let threshold = b.remote("THRESHOLD", Float64, DC, "threshold_energy")?;
    b.reg.set_epsilon(threshold, ENERGY_EPSILON);
    let n_triggers = b.remote("NUM_TRIGGERS", Int32, DC, "ntrigger")?;
    let compression_algo = b.remote("COMPRESSION_ALGO", Int32, DC, "compression")?;
    let roi_mode = b.remote("ROI_MODE", Int32, DC, "roi_mode")?;
    let auto_summation = b.remote("AUTO_SUMMATION", Int32, DC, "auto_summation")?;

    // Detector status.
    let error = b.remote("ERROR", Octet, DS, "error")?;
    let th_temp0 = b.remote("TH_TEMP_0", Float64, DS, "board_000/th0_temp")?;
    let th_humid0 = b.remote("TH_HUMID_0", Float64, DS, "board_000/th0_humidity")?;

    // FileWriter.
    let fw_enable = b.remote("FW_ENABLE", Int32, FC, "mode")?;
    b.reg.set_enum_values(fw_enable, &MODE_ENUM);
    let fw_compression = b.remote("COMPRESSION", Int32, FC, "compression_enabled")?;
    let fw_name_pattern = b.remote("NAME_PATTERN", Octet, FC, "name_pattern")?;
    let fw_nimgs_per_file = b.remote("NIMAGES_PER_FILE", Int32, FC, "nimages_per_file")?;
    let fw_img_num_start = b.remote("FW_IMG_NUM_START", Int32, FC, "image_nr_start")?;
    let fw_state = b.remote("FW_STATE", Octet, FS, "state")?;
    let fw_free = b.remote("FW_FREE", Float64, FS, "buffer_free")?;

    // Monitor.
    let monitor_enable = b.remote("MONITOR_ENABLE", Int32, MC, "mode")?;
    b.reg.set_enum_values(monitor_enable, &MODE_ENUM);
    let monitor_buf_size = b.remote("MONITOR_BUF_SIZE", Int32, MC, "buffer_size")?;
    let monitor_state = b.remote("MONITOR_STATE", Octet, MS, "state")?;

    // Stream.
    let stream_enable = b.remote("STREAM_ENABLE", Int32, SC, "mode")?;
    b.reg.set_enum_values(stream_enable, &MODE_ENUM);
    let stream_state = b.remote("STREAM_STATE", Octet, SS, "state")?;
    let stream_dropped = b.remote("STREAM_DROPPED", Int32, SS, "dropped")?;
    let stream_version = b.remote("STREAM_VERSION", Int32, SC, "format")?;

    // Base-class parameters, bound to their detector-config counterparts.
    // `create_param` resolves an existing name to the index `ADDriverParams`
    // already assigned, so these are the same parameters the AD records use.
    let acquire_time = b.remote("ACQ_TIME", Float64, DC, "count_time")?;
    let acquire_period = b.remote("ACQ_PERIOD", Float64, DC, "frame_time")?;
    let num_images = b.remote("NIMAGES", Int32, DC, "nimages")?;
    let trigger_mode = b.remote("TRIGGER_MODE", Int32, DC, "trigger_mode")?;
    if model.has_thresholds_1_2() {
        b.reg
            .set_enum_values(trigger_mode, &TRIGGER_MODE_NAMES_EXTG);
    } else {
        b.reg.set_enum_values(trigger_mode, &TRIGGER_MODE_NAMES);
    }
    b.remote("SDK_VERSION", Octet, DC, "software_version")?;
    b.remote("FIRMWARE_VERSION", Octet, DC, "eiger_fw_version")?;
    b.remote("SERIAL_NUMBER", Octet, DC, "detector_number")?;
    b.remote("TEMPERATURE_ACTUAL", Float64, DS, "board_000/th0_temp")?;
    let nd_array_size_x = b.remote("ARRAY_SIZE_X", Int32, DC, "x_pixels_in_detector")?;
    let nd_array_size_y = b.remote("ARRAY_SIZE_Y", Int32, DC, "y_pixels_in_detector")?;

    let mut p = EigerParams {
        data_source,
        fw_auto_remove,
        trigger,
        manual_trigger,
        armed,
        sequence_id,
        pending_files,
        save_files,
        file_owner,
        file_owner_group,
        file_perms,
        monitor_timeout,
        restart,
        initialize,
        stream_decompress,
        wavelength_epsilon,
        energy_epsilon,
        signed_data,
        stream_as_ts_source,
        description,
        wavelength,
        photon_energy,
        threshold,
        n_triggers,
        compression_algo,
        roi_mode,
        auto_summation,
        state,
        error,
        th_temp0,
        th_humid0,
        fw_enable,
        fw_compression,
        fw_name_pattern,
        fw_nimgs_per_file,
        fw_img_num_start,
        fw_state,
        fw_free,
        monitor_enable,
        monitor_buf_size,
        monitor_state,
        stream_enable,
        stream_state,
        stream_dropped,
        stream_version,
        acquire_time,
        acquire_period,
        num_images,
        trigger_mode,
        nd_array_size_x,
        nd_array_size_y,
        link0: None,
        link1: None,
        link2: None,
        link3: None,
        dcu_buf_free: None,
        fw_clear: None,
        threshold1_enable: None,
        trigger_start_delay: None,
        threshold2: None,
        threshold2_enable: None,
        threshold_diff_enable: None,
        hv_state: None,
        hv_reset_time: None,
        hv_reset: None,
        fw_hdf5_format: None,
        ext_gate_mode: None,
        num_exposures: None,
        threshold3: None,
        threshold3_enable: None,
        threshold4: None,
        threshold4_enable: None,
    };

    if api == ApiVersion::V1_6_0 {
        for (field, name, remote) in [
            (0usize, "LINK_0", "link_0"),
            (1, "LINK_1", "link_1"),
            (2, "LINK_2", "link_2"),
            (3, "LINK_3", "link_3"),
        ] {
            let idx = b.remote(name, Int32, DS, remote)?;
            b.reg.set_enum_values(idx, &LINK_ENUM);
            match field {
                0 => p.link0 = Some(idx),
                1 => p.link1 = Some(idx),
                2 => p.link2 = Some(idx),
                _ => p.link3 = Some(idx),
            }
        }
        p.dcu_buf_free = Some(b.remote("DCU_BUF_FREE", Float64, DS, "builder/dcu_buffer_free")?);
        p.fw_clear = Some(b.remote("CLEAR", Int32, Sys::FwCommand, "clear")?);
    } else if model.has_thresholds_1_2() {
        let t1 = b.remote("THRESHOLD1_ENABLE", Int32, DC, "threshold/1/mode")?;
        b.reg.set_enum_values(t1, &MODE_ENUM);
        p.threshold1_enable = Some(t1);
        p.trigger_start_delay =
            Some(b.remote("TRIGGER_START_DELAY", Float64, DC, "trigger_start_delay")?);
        let t2 = b.remote("THRESHOLD2", Float64, DC, "threshold/2/energy")?;
        b.reg.set_epsilon(t2, ENERGY_EPSILON);
        p.threshold2 = Some(t2);
        let t2e = b.remote("THRESHOLD2_ENABLE", Int32, DC, "threshold/2/mode")?;
        b.reg.set_enum_values(t2e, &MODE_ENUM);
        p.threshold2_enable = Some(t2e);
        let tde = b.remote(
            "THRESHOLD_DIFF_ENABLE",
            Int32,
            DC,
            "threshold/difference/mode",
        )?;
        b.reg.set_enum_values(tde, &MODE_ENUM);
        p.threshold_diff_enable = Some(tde);
        p.hv_state = Some(b.remote("HV_STATE", Octet, DS, "high_voltage/state")?);
        p.hv_reset_time = Some(b.local("HV_RESET_TIME", Float64)?);
        p.hv_reset = Some(b.local("HV_RESET", Int32)?);
        p.fw_hdf5_format = Some(b.remote("FWHDF5_FORMAT", Int32, FC, "format")?);
        p.ext_gate_mode = Some(b.remote("EXT_GATE_MODE", Int32, DC, "extg_mode")?);
        p.num_exposures = Some(b.remote("NEXPOSURES", Int32, DC, "nexpi")?);
    }

    if model.has_thresholds_3_4() {
        let t3 = b.remote("THRESHOLD3", Float64, DC, "threshold/3/energy")?;
        b.reg.set_epsilon(t3, ENERGY_EPSILON);
        p.threshold3 = Some(t3);
        let t3e = b.remote("THRESHOLD3_ENABLE", Int32, DC, "threshold/3/mode")?;
        b.reg.set_enum_values(t3e, &MODE_ENUM);
        p.threshold3_enable = Some(t3e);
        let t4 = b.remote("THRESHOLD4", Float64, DC, "threshold/4/energy")?;
        b.reg.set_epsilon(t4, ENERGY_EPSILON);
        p.threshold4 = Some(t4);
        let t4e = b.remote("THRESHOLD4_ENABLE", Int32, DC, "threshold/4/mode")?;
        b.reg.set_enum_values(t4e, &MODE_ENUM);
        p.threshold4_enable = Some(t4e);
    }

    Ok(p)
}

/// One threshold the detector can expose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThresholdParam {
    /// 1-based threshold number, as published in the `ThresholdNumber`
    /// NDArray attribute.
    pub number: i32,
    /// The `threshold/N/mode` parameter, or `None` on Eiger1, whose single
    /// threshold is always active.
    pub enable: Option<usize>,
    /// The `threshold/N/energy` parameter.
    pub energy: usize,
}

/// Every threshold this model can expose, in threshold order.
///
/// UPSTREAM DEFECT (eigerDetector.cpp:1829): the branch that collects
/// thresholds 3 and 4 is `else if (mEigerModel == Pilatus4)` sitting *after*
/// `else if ((mEigerModel == Eiger2) || (mEigerModel == Pilatus4))`, so it is
/// unreachable — a Pilatus4 always takes the Eiger2 arm and its thresholds 3
/// and 4 never reach the `ThresholdNumber` / `ThresholdEnergy` attributes.
/// Replaced here by one model-driven list, with no branch that can shadow
/// another.
pub fn thresholds(p: &EigerParams) -> Vec<ThresholdParam> {
    let mut out = vec![ThresholdParam {
        number: 1,
        enable: p.threshold1_enable,
        energy: p.threshold,
    }];
    for (number, enable, energy) in [
        (2, p.threshold2_enable, p.threshold2),
        (3, p.threshold3_enable, p.threshold3),
        (4, p.threshold4_enable, p.threshold4),
    ] {
        if let (Some(enable), Some(energy)) = (enable, energy) {
            out.push(ThresholdParam {
                number,
                enable: Some(enable),
                energy,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_model_comes_from_the_description_string() {
        assert_eq!(
            Model::from_description("Dectris Eiger2 CdTe 4M"),
            Model::Eiger2
        );
        assert_eq!(
            Model::from_description("DECTRIS EIGER2 Si 16M"),
            Model::Eiger2
        );
        assert_eq!(
            Model::from_description("Dectris Pilatus4 2M"),
            Model::Pilatus4
        );
        assert_eq!(
            Model::from_description("DECTRIS PILATUS4 1M"),
            Model::Pilatus4
        );
        assert_eq!(Model::from_description("Dectris Eiger 1M"), Model::Eiger1);
    }

    #[test]
    fn continuous_triggers_as_an_internal_series_on_the_wire() {
        assert_eq!(TRIGGER_MODE_NAMES[TRIGGER_MODE_CONTINUOUS as usize], "ints");
        assert_eq!(TRIGGER_MODE_NAMES[TRIGGER_MODE_INTE as usize], "inte");
        assert_eq!(TRIGGER_MODE_NAMES[TRIGGER_MODE_EXTS as usize], "exts");
        assert_eq!(TRIGGER_MODE_NAMES[TRIGGER_MODE_EXTE as usize], "exte");
    }
}
