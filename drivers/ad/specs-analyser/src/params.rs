//! The SPECS-specific asyn parameters (`SPECSConnect_` through
//! `SPECSDataDelayMax_`, `specsAnalyser.h:183-222`).
//!
//! `drvInfo` strings are identical to `specsAnalyser.h`'s `SPECS*String`
//! macros, so `specsAnalyser.template`'s DTYP/INP/OUT links bind unchanged.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// Parameter indices, in the same creation order as the C constructor. The
/// first (`connect`) plays the role of C's `FIRST_SPECS_PARAM`.
#[derive(Debug, Clone, Copy)]
pub struct SpecsParams {
    pub connect: usize,
    pub connected: usize,
    pub pause_acq: usize,
    pub msg_counter: usize,
    pub server_name: usize,
    pub protocol_version: usize,
    pub protocol_version_minor: usize,
    pub protocol_version_major: usize,
    pub start_energy: usize,
    pub end_energy: usize,
    pub retarding_ratio: usize,
    pub kinetic_energy: usize,
    pub step_width: usize,
    pub samples: usize,
    pub samples_iteration: usize,
    pub snapshot_values: usize,
    pub pass_energy: usize,
    pub lens_mode: usize,
    pub scan_range: usize,
    pub current_sample: usize,
    pub percent_complete: usize,
    pub remaining_time: usize,
    pub current_sample_iteration: usize,
    pub percent_complete_iteration: usize,
    pub remaining_time_iteration: usize,
    pub acq_spectrum: usize,
    pub acq_image: usize,

    pub run_mode: usize,
    pub define: usize,
    pub validate: usize,

    pub non_energy_channels: usize,
    pub non_energy_units: usize,
    pub non_energy_min: usize,
    pub non_energy_max: usize,
    pub safe_state: usize,
    pub data_delay_max: usize,
}

impl SpecsParams {
    pub fn create(port_base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            connect: port_base.create_param("SPECS_CONNECT", ParamType::Int32)?,
            connected: port_base.create_param("SPECS_CONNECTED", ParamType::Int32)?,
            pause_acq: port_base.create_param("SPECS_PAUSE_ACQ", ParamType::Int32)?,
            msg_counter: port_base.create_param("SPECS_MSG_COUNTER", ParamType::Int32)?,
            server_name: port_base.create_param("SPECS_SERVER_NAME", ParamType::Octet)?,
            // Upstream creates this as asynParamInt32 (specsAnalyser.cpp:91) but
            // only ever calls `setStringParam` on it (specsAnalyser.cpp:1868) —
            // a self-inconsistent createParam type, fixed here to Octet so the
            // protocol version string actually stores/publishes.
            protocol_version: port_base.create_param("SPECS_PROTOCOL_VERSION", ParamType::Octet)?,
            protocol_version_minor: port_base
                .create_param("SPECS_PROTOCOL_VER_MINOR", ParamType::Int32)?,
            protocol_version_major: port_base
                .create_param("SPECS_PROTOCOL_VER_MAJOR", ParamType::Int32)?,
            start_energy: port_base.create_param("SPECS_START_ENERGY", ParamType::Float64)?,
            end_energy: port_base.create_param("SPECS_END_ENERGY", ParamType::Float64)?,
            retarding_ratio: port_base.create_param("SPECS_RETARDING_RATIO", ParamType::Float64)?,
            kinetic_energy: port_base.create_param("SPECS_KINETIC_ENERGY", ParamType::Float64)?,
            step_width: port_base.create_param("SPECS_STEP_WIDTH", ParamType::Float64)?,
            samples: port_base.create_param("SPECS_SAMPLES", ParamType::Int32)?,
            samples_iteration: port_base
                .create_param("SPECS_SAMPLES_ITERATION", ParamType::Int32)?,
            snapshot_values: port_base.create_param("SPECS_SNAPSHOT_VALUES", ParamType::Int32)?,
            pass_energy: port_base.create_param("SPECS_PASS_ENERGY", ParamType::Float64)?,
            lens_mode: port_base.create_param("SPECS_LENS_MODE", ParamType::Int32)?,
            scan_range: port_base.create_param("SPECS_SCAN_RANGE", ParamType::Int32)?,
            current_sample: port_base.create_param("SPECS_CURRENT_SAMPLE", ParamType::Int32)?,
            percent_complete: port_base.create_param("SPECS_PERCENT_COMPLETE", ParamType::Int32)?,
            remaining_time: port_base.create_param("SPECS_REMAINING_TIME", ParamType::Float64)?,
            current_sample_iteration: port_base
                .create_param("SPECS_CRT_SAMPLE_ITER", ParamType::Int32)?,
            percent_complete_iteration: port_base
                .create_param("SPECS_PCT_COMPLETE_ITER", ParamType::Int32)?,
            remaining_time_iteration: port_base
                .create_param("SPECS_RMG_TIME_ITER", ParamType::Float64)?,
            acq_spectrum: port_base.create_param("SPECS_ACQ_SPECTRUM", ParamType::Float64Array)?,
            acq_image: port_base.create_param("SPECS_ACQ_IMAGE", ParamType::Float64Array)?,

            run_mode: port_base.create_param("SPECS_RUN_MODE", ParamType::Int32)?,
            define: port_base.create_param("SPECS_DEFINE", ParamType::Int32)?,
            validate: port_base.create_param("SPECS_VALIDATE", ParamType::Int32)?,

            non_energy_channels: port_base
                .create_param("SPECS_NON_ENERGY_CHANNELS", ParamType::Int32)?,
            non_energy_units: port_base.create_param("SPECS_NON_ENERGY_UNITS", ParamType::Octet)?,
            non_energy_min: port_base.create_param("SPECS_NON_ENERGY_MIN", ParamType::Float64)?,
            non_energy_max: port_base.create_param("SPECS_NON_ENERGY_MAX", ParamType::Float64)?,
            safe_state: port_base.create_param("SPECS_SAFE_STATE", ParamType::Int32)?,
            data_delay_max: port_base.create_param("SPECS_DATA_DELAY_MAX", ParamType::Float64)?,
        })
    }

    /// C `FIRST_SPECS_PARAM` — a reason below this belongs to the base class.
    pub fn first(&self) -> usize {
        self.connect
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::PortFlags;

    #[test]
    fn creates_all_params_with_distinct_indices() {
        let mut base = PortDriverBase::new("test-specs-params", 1, PortFlags::default());
        let p = SpecsParams::create(&mut base).unwrap();
        assert_eq!(base.find_param("SPECS_CONNECT"), Some(p.connect));
        assert_eq!(
            base.find_param("SPECS_DATA_DELAY_MAX"),
            Some(p.data_delay_max)
        );
        assert_eq!(p.first(), p.connect);
    }
}
