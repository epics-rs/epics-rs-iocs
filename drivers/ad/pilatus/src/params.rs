//! The 47 Pilatus-specific asyn parameters.
//!
//! `drvInfo` strings are identical to `pilatusDetector.cpp`, so the C
//! `pilatus.template` records bind unchanged.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// Parameter indices, in the same creation order as C's constructor. The first
/// one (`delay_time`) plays the role of C's `FIRST_PILATUS_PARAM`.
#[derive(Debug, Clone, Copy)]
pub struct PilatusParams {
    pub delay_time: usize,
    pub threshold: usize,
    pub threshold_apply: usize,
    pub threshold_auto_apply: usize,
    pub energy: usize,
    pub armed: usize,
    pub reset_power: usize,
    pub reset_power_time: usize,
    pub image_file_tmot: usize,
    pub bad_pixel_file: usize,
    pub num_bad_pixels: usize,
    pub flat_field_file: usize,
    pub min_flat_field: usize,
    pub flat_field_valid: usize,
    pub gap_fill: usize,
    pub wavelength: usize,
    pub energy_low: usize,
    pub energy_high: usize,
    pub det_dist: usize,
    pub det_voffset: usize,
    pub beam_x: usize,
    pub beam_y: usize,
    pub flux: usize,
    pub filter_transm: usize,
    pub start_angle: usize,
    pub angle_incr: usize,
    pub det_2theta: usize,
    pub polarization: usize,
    pub alpha: usize,
    pub kappa: usize,
    pub phi: usize,
    pub phi_incr: usize,
    pub chi: usize,
    pub chi_incr: usize,
    pub omega: usize,
    pub omega_incr: usize,
    pub oscill_axis: usize,
    pub num_oscill: usize,
    pub pixel_cutoff: usize,
    pub th_temp_0: usize,
    pub th_temp_1: usize,
    pub th_temp_2: usize,
    pub th_humid_0: usize,
    pub th_humid_1: usize,
    pub th_humid_2: usize,
    pub tvx_version: usize,
    pub cbf_template_file: usize,
    pub header_string: usize,
}

impl PilatusParams {
    pub fn create(port_base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            delay_time: port_base.create_param("DELAY_TIME", ParamType::Float64)?,
            threshold: port_base.create_param("THRESHOLD", ParamType::Float64)?,
            threshold_apply: port_base.create_param("THRESHOLD_APPLY", ParamType::Int32)?,
            threshold_auto_apply: port_base
                .create_param("THRESHOLD_AUTO_APPLY", ParamType::Int32)?,
            energy: port_base.create_param("ENERGY", ParamType::Float64)?,
            armed: port_base.create_param("ARMED", ParamType::Int32)?,
            reset_power: port_base.create_param("RESET_POWER", ParamType::Int32)?,
            reset_power_time: port_base.create_param("RESET_POWER_TIME", ParamType::Int32)?,
            image_file_tmot: port_base.create_param("IMAGE_FILE_TMOT", ParamType::Float64)?,
            bad_pixel_file: port_base.create_param("BAD_PIXEL_FILE", ParamType::Octet)?,
            num_bad_pixels: port_base.create_param("NUM_BAD_PIXELS", ParamType::Int32)?,
            flat_field_file: port_base.create_param("FLAT_FIELD_FILE", ParamType::Octet)?,
            min_flat_field: port_base.create_param("MIN_FLAT_FIELD", ParamType::Int32)?,
            flat_field_valid: port_base.create_param("FLAT_FIELD_VALID", ParamType::Int32)?,
            gap_fill: port_base.create_param("GAP_FILL", ParamType::Int32)?,
            wavelength: port_base.create_param("WAVELENGTH", ParamType::Float64)?,
            energy_low: port_base.create_param("ENERGY_LOW", ParamType::Float64)?,
            energy_high: port_base.create_param("ENERGY_HIGH", ParamType::Float64)?,
            det_dist: port_base.create_param("DET_DIST", ParamType::Float64)?,
            det_voffset: port_base.create_param("DET_VOFFSET", ParamType::Float64)?,
            beam_x: port_base.create_param("BEAM_X", ParamType::Float64)?,
            beam_y: port_base.create_param("BEAM_Y", ParamType::Float64)?,
            flux: port_base.create_param("FLUX", ParamType::Float64)?,
            filter_transm: port_base.create_param("FILTER_TRANSM", ParamType::Float64)?,
            start_angle: port_base.create_param("START_ANGLE", ParamType::Float64)?,
            angle_incr: port_base.create_param("ANGLE_INCR", ParamType::Float64)?,
            det_2theta: port_base.create_param("DET_2THETA", ParamType::Float64)?,
            polarization: port_base.create_param("POLARIZATION", ParamType::Float64)?,
            alpha: port_base.create_param("ALPHA", ParamType::Float64)?,
            kappa: port_base.create_param("KAPPA", ParamType::Float64)?,
            phi: port_base.create_param("PHI", ParamType::Float64)?,
            phi_incr: port_base.create_param("PHI_INCR", ParamType::Float64)?,
            chi: port_base.create_param("CHI", ParamType::Float64)?,
            chi_incr: port_base.create_param("CHI_INCR", ParamType::Float64)?,
            omega: port_base.create_param("OMEGA", ParamType::Float64)?,
            omega_incr: port_base.create_param("OMEGA_INCR", ParamType::Float64)?,
            oscill_axis: port_base.create_param("OSCILL_AXIS", ParamType::Octet)?,
            num_oscill: port_base.create_param("NUM_OSCILL", ParamType::Int32)?,
            pixel_cutoff: port_base.create_param("PIXEL_CUTOFF", ParamType::Int32)?,
            th_temp_0: port_base.create_param("TH_TEMP_0", ParamType::Float64)?,
            th_temp_1: port_base.create_param("TH_TEMP_1", ParamType::Float64)?,
            th_temp_2: port_base.create_param("TH_TEMP_2", ParamType::Float64)?,
            th_humid_0: port_base.create_param("TH_HUMID_0", ParamType::Float64)?,
            th_humid_1: port_base.create_param("TH_HUMID_1", ParamType::Float64)?,
            th_humid_2: port_base.create_param("TH_HUMID_2", ParamType::Float64)?,
            tvx_version: port_base.create_param("TVXVERSION", ParamType::Octet)?,
            cbf_template_file: port_base.create_param("CBFTEMPLATEFILE", ParamType::Octet)?,
            header_string: port_base.create_param("HEADERSTRING", ParamType::Octet)?,
        })
    }

    /// C `FIRST_PILATUS_PARAM` — a reason below this belongs to the base class.
    pub fn first(&self) -> usize {
        self.delay_time
    }
}
