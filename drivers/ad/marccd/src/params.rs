//! The 28 marCCD-specific asyn parameters.
//!
//! `drvInfo` strings are identical to `marCCD.cpp`, so the C `marCCD.template`
//! records bind unchanged.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// Parameter indices, in the same creation order as C's constructor. The first
/// one (`gate_mode`) plays the role of C's `FIRST_MARCCD_PARAM`.
#[derive(Debug, Clone, Copy)]
pub struct MarccdParams {
    pub gate_mode: usize,
    pub readout_mode: usize,
    pub server_mode: usize,
    pub tiff_timeout: usize,
    pub series_file_template: usize,
    pub series_file_digits: usize,
    pub series_file_first: usize,
    pub overlap: usize,
    pub state: usize,
    pub status: usize,
    pub task_acquire_status: usize,
    pub task_readout_status: usize,
    pub task_correct_status: usize,
    pub task_writing_status: usize,
    pub task_dezinger_status: usize,
    pub task_series_status: usize,
    pub stability: usize,
    pub frame_shift: usize,
    pub detector_distance: usize,
    pub beam_x: usize,
    pub beam_y: usize,
    pub start_phi: usize,
    pub rotation_axis: usize,
    pub rotation_range: usize,
    pub two_theta: usize,
    pub wavelength: usize,
    pub file_comments: usize,
    pub dataset_comments: usize,
}

impl MarccdParams {
    pub fn create(port_base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            gate_mode: port_base.create_param("MAR_GATE_MODE", ParamType::Int32)?,
            readout_mode: port_base.create_param("MAR_READOUT_MODE", ParamType::Int32)?,
            server_mode: port_base.create_param("MAR_SERVER_MODE", ParamType::Int32)?,
            tiff_timeout: port_base.create_param("MAR_TIFF_TIMEOUT", ParamType::Float64)?,
            series_file_template: port_base
                .create_param("MAR_SERIES_FILE_TEMPLATE", ParamType::Octet)?,
            series_file_digits: port_base
                .create_param("MAR_SERIES_FILE_DIGITS", ParamType::Int32)?,
            series_file_first: port_base.create_param("MAR_SERIES_FILE_FIRST", ParamType::Int32)?,
            overlap: port_base.create_param("MAR_OVERLAP", ParamType::Int32)?,
            state: port_base.create_param("MAR_STATE", ParamType::Int32)?,
            status: port_base.create_param("MAR_STATUS", ParamType::Int32)?,
            task_acquire_status: port_base.create_param("MAR_ACQUIRE_STATUS", ParamType::Int32)?,
            task_readout_status: port_base.create_param("MAR_READOUT_STATUS", ParamType::Int32)?,
            task_correct_status: port_base.create_param("MAR_CORRECT_STATUS", ParamType::Int32)?,
            task_writing_status: port_base.create_param("MAR_WRITING_STATUS", ParamType::Int32)?,
            task_dezinger_status: port_base
                .create_param("MAR_DEZINGER_STATUS", ParamType::Int32)?,
            task_series_status: port_base.create_param("MAR_SERIES_STATUS", ParamType::Int32)?,
            stability: port_base.create_param("MAR_STABILITY", ParamType::Float64)?,
            frame_shift: port_base.create_param("MAR_FRAME_SHIFT", ParamType::Int32)?,
            detector_distance: port_base
                .create_param("MAR_DETECTOR_DISTANCE", ParamType::Float64)?,
            beam_x: port_base.create_param("MAR_BEAM_X", ParamType::Float64)?,
            beam_y: port_base.create_param("MAR_BEAM_Y", ParamType::Float64)?,
            start_phi: port_base.create_param("MAR_START_PHI", ParamType::Float64)?,
            rotation_axis: port_base.create_param("MAR_ROTATION_AXIS", ParamType::Octet)?,
            rotation_range: port_base.create_param("MAR_ROTATION_RANGE", ParamType::Float64)?,
            two_theta: port_base.create_param("MAR_TWO_THETA", ParamType::Float64)?,
            wavelength: port_base.create_param("MAR_WAVELENGTH", ParamType::Float64)?,
            file_comments: port_base.create_param("MAR_FILE_COMMENTS", ParamType::Octet)?,
            dataset_comments: port_base.create_param("MAR_DATASET_COMMENTS", ParamType::Octet)?,
        })
    }

    /// C `FIRST_MARCCD_PARAM` — a reason below this belongs to the base class.
    pub fn first(&self) -> usize {
        self.gate_mode
    }
}
