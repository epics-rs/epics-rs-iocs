//! Merlin-specific asyn parameters (C `merlinDetector.h` `*String` names).
//!
//! C also declared DELAY_TIME, ARMED, THRESHOLD_AUTO_APPLY and
//! STARTTHRESHOLDSCANNING; none is read or written by the driver or bound by
//! any record in `merlin.template`, so they are not created here.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

#[derive(Clone, Copy)]
pub struct MerlinParams {
    pub thresholds: [usize; 8],
    pub operating_energy: usize,
    pub threshold_apply: usize,
    pub threshold_scan: usize,
    pub start_threshold_scan: usize,
    pub stop_threshold_scan: usize,
    pub step_threshold_scan: usize,
    pub counter_depth: usize,
    pub reset: usize,
    pub software_trigger: usize,
    pub enable_counter1: usize,
    pub continuous_rw: usize,
    // XBPM
    pub profile_control: usize,
    pub profile_x: usize,
    pub profile_y: usize,
    // UoM XBPM
    pub enable_background_corr: usize,
    pub enable_image_sum: usize,
    // Merlin Quad
    pub quad_merlin_mode: usize,
    pub select_gui: usize,
}

impl MerlinParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        let mut thresholds = [0usize; 8];
        for (i, slot) in thresholds.iter_mut().enumerate() {
            *slot = base.create_param(&format!("THRESHOLD{i}"), ParamType::Float64)?;
        }
        Ok(Self {
            thresholds,
            operating_energy: base.create_param("OPERATINGENERGY", ParamType::Float64)?,
            threshold_apply: base.create_param("THRESHOLD_APPLY", ParamType::Int32)?,
            threshold_scan: base.create_param("THRESHOLDSCAN", ParamType::Int32)?,
            start_threshold_scan: base.create_param("THRESHOLDSTART", ParamType::Float64)?,
            stop_threshold_scan: base.create_param("THRESHOLDSTOP", ParamType::Float64)?,
            step_threshold_scan: base.create_param("THRESHOLDSTEP", ParamType::Float64)?,
            counter_depth: base.create_param("COUNTERDEPTH", ParamType::Int32)?,
            reset: base.create_param("RESET", ParamType::Int32)?,
            software_trigger: base.create_param("SOFTWARETRIGGER", ParamType::Int32)?,
            enable_counter1: base.create_param("ENABLECOUNTER1", ParamType::Int32)?,
            continuous_rw: base.create_param("CONTINUOUSRW", ParamType::Int32)?,
            profile_control: base.create_param("PROFILECONTROL", ParamType::Int32)?,
            profile_x: base.create_param("PROFILE_AVERAGE_X", ParamType::Int32Array)?,
            profile_y: base.create_param("PROFILE_AVERAGE_Y", ParamType::Int32Array)?,
            enable_background_corr: base.create_param("ENABLEBACKGROUNDCORR", ParamType::Int32)?,
            enable_image_sum: base.create_param("ENABLESUMAVERAGE", ParamType::Int32)?,
            quad_merlin_mode: base.create_param("QUADMERLINMODE", ParamType::Int32)?,
            select_gui: base.create_param("SELECTGUI", ParamType::Octet)?,
        })
    }
}
