//! `simDetector`-specific asyn parameters (`simDetector.h:95-126`).

use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::ndarray::NDDataType;

use crate::image::{Geometry, SimConfig, SineParams};
use crate::types::{SimMode, SineOperation};

/// Parameter indices, created in the same order as the C constructor
/// (simDetector.cpp:1059-1090). The order is load-bearing: `writeInt32` decides
/// whether a write dirties the image by testing `SimPeakStartX <= reason <=
/// SimPeakStepY`, and `writeFloat64` by testing `reason >= SimGainX`.
#[derive(Clone, Copy)]
pub struct SimParams {
    /// `FIRST_SIM_DETECTOR_PARAM` — every index at or above this belongs to the
    /// simulation driver rather than a base class.
    pub gain_x: usize,
    pub gain_y: usize,
    pub gain_red: usize,
    pub gain_green: usize,
    pub gain_blue: usize,
    pub offset: usize,
    pub noise: usize,
    pub reset_image: usize,
    pub mode: usize,

    pub peak_start_x: usize,
    pub peak_start_y: usize,
    pub peak_width_x: usize,
    pub peak_width_y: usize,
    pub peak_num_x: usize,
    pub peak_num_y: usize,
    pub peak_step_x: usize,
    pub peak_step_y: usize,
    pub peak_height_variation: usize,

    pub x_sine_operation: usize,
    pub y_sine_operation: usize,
    pub x_sine1_amplitude: usize,
    pub x_sine1_frequency: usize,
    pub x_sine1_phase: usize,
    pub x_sine2_amplitude: usize,
    pub x_sine2_frequency: usize,
    pub x_sine2_phase: usize,
    pub y_sine1_amplitude: usize,
    pub y_sine1_frequency: usize,
    pub y_sine1_phase: usize,
    pub y_sine2_amplitude: usize,
    pub y_sine2_frequency: usize,
    pub y_sine2_phase: usize,
}

impl SimParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            gain_x: base.create_param("SIM_GAIN_X", ParamType::Float64)?,
            gain_y: base.create_param("SIM_GAIN_Y", ParamType::Float64)?,
            gain_red: base.create_param("SIM_GAIN_RED", ParamType::Float64)?,
            gain_green: base.create_param("SIM_GAIN_GREEN", ParamType::Float64)?,
            gain_blue: base.create_param("SIM_GAIN_BLUE", ParamType::Float64)?,
            offset: base.create_param("SIM_OFFSET", ParamType::Float64)?,
            noise: base.create_param("SIM_NOISE", ParamType::Float64)?,
            reset_image: base.create_param("RESET_IMAGE", ParamType::Int32)?,
            mode: base.create_param("SIM_MODE", ParamType::Int32)?,
            peak_start_x: base.create_param("SIM_PEAK_START_X", ParamType::Int32)?,
            peak_start_y: base.create_param("SIM_PEAK_START_Y", ParamType::Int32)?,
            peak_width_x: base.create_param("SIM_PEAK_WIDTH_X", ParamType::Int32)?,
            peak_width_y: base.create_param("SIM_PEAK_WIDTH_Y", ParamType::Int32)?,
            peak_num_x: base.create_param("SIM_PEAK_NUM_X", ParamType::Int32)?,
            peak_num_y: base.create_param("SIM_PEAK_NUM_Y", ParamType::Int32)?,
            peak_step_x: base.create_param("SIM_PEAK_STEP_X", ParamType::Int32)?,
            peak_step_y: base.create_param("SIM_PEAK_STEP_Y", ParamType::Int32)?,
            peak_height_variation: base
                .create_param("SIM_PEAK_HEIGHT_VARIATION", ParamType::Float64)?,
            x_sine_operation: base.create_param("SIM_XSINE_OPERATION", ParamType::Int32)?,
            y_sine_operation: base.create_param("SIM_YSINE_OPERATION", ParamType::Int32)?,
            x_sine1_amplitude: base.create_param("SIM_XSINE1_AMPLITUDE", ParamType::Float64)?,
            x_sine1_frequency: base.create_param("SIM_XSINE1_FREQUENCY", ParamType::Float64)?,
            x_sine1_phase: base.create_param("SIM_XSINE1_PHASE", ParamType::Float64)?,
            x_sine2_amplitude: base.create_param("SIM_XSINE2_AMPLITUDE", ParamType::Float64)?,
            x_sine2_frequency: base.create_param("SIM_XSINE2_FREQUENCY", ParamType::Float64)?,
            x_sine2_phase: base.create_param("SIM_XSINE2_PHASE", ParamType::Float64)?,
            y_sine1_amplitude: base.create_param("SIM_YSINE1_AMPLITUDE", ParamType::Float64)?,
            y_sine1_frequency: base.create_param("SIM_YSINE1_FREQUENCY", ParamType::Float64)?,
            y_sine1_phase: base.create_param("SIM_YSINE1_PHASE", ParamType::Float64)?,
            y_sine2_amplitude: base.create_param("SIM_YSINE2_AMPLITUDE", ParamType::Float64)?,
            y_sine2_frequency: base.create_param("SIM_YSINE2_FREQUENCY", ParamType::Float64)?,
            y_sine2_phase: base.create_param("SIM_YSINE2_PHASE", ParamType::Float64)?,
        })
    }

    /// `FIRST_SIM_DETECTOR_PARAM` (simDetector.h:24).
    pub fn first_sim_param(&self) -> usize {
        self.gain_x
    }

    /// A write to `reason` belongs to this driver, not to a base class.
    pub fn owns(&self, reason: usize) -> bool {
        reason >= self.first_sim_param()
    }

    /// `writeInt32`: which integer writes force a full image recompute
    /// (simDetector.cpp:935-939).
    pub fn int32_write_dirties_image(&self, reason: usize, base: &ADBaseParams) -> bool {
        reason == base.base.data_type
            || reason == base.base.color_mode
            || reason == self.mode
            || (reason >= self.peak_start_x && reason <= self.peak_step_y)
    }

    /// `writeFloat64`: `ADGain` and every simulation double
    /// (simDetector.cpp:975).
    pub fn float64_write_dirties_image(&self, reason: usize, gain: usize) -> bool {
        reason == gain || self.owns(reason)
    }

    /// Read the full `computeImage` input set. The geometry is returned raw and
    /// must still be [`Geometry::clamp`]ed by the caller.
    pub async fn read_config(
        &self,
        handle: &PortHandle,
        ad: &ADBaseParams,
    ) -> AsynResult<SimConfig> {
        Ok(SimConfig {
            geometry: Geometry {
                max_size_x: handle.read_int32(ad.max_size_x, 0).await?,
                max_size_y: handle.read_int32(ad.max_size_y, 0).await?,
                min_x: handle.read_int32(ad.min_x, 0).await?,
                min_y: handle.read_int32(ad.min_y, 0).await?,
                size_x: handle.read_int32(ad.size_x, 0).await?,
                size_y: handle.read_int32(ad.size_y, 0).await?,
                bin_x: handle.read_int32(ad.bin_x, 0).await?,
                bin_y: handle.read_int32(ad.bin_y, 0).await?,
                reverse_x: handle.read_int32(ad.reverse_x, 0).await? != 0,
                reverse_y: handle.read_int32(ad.reverse_y, 0).await? != 0,
            },
            color_mode: NDColorMode::from_i32(handle.read_int32(ad.base.color_mode, 0).await?),
            data_type: u8::try_from(handle.read_int32(ad.base.data_type, 0).await?)
                .ok()
                .and_then(NDDataType::from_ordinal)
                // C `switch (dataType)` has no default: an out-of-range value
                // leaves the raw buffer untouched. There is no Rust equivalent
                // of "do nothing with an unknown type", so fall back to the
                // ADDriverBase default (UInt8).
                .unwrap_or(NDDataType::UInt8),
            sim_mode: SimMode::from_i32(handle.read_int32(self.mode, 0).await?),

            gain: handle.read_float64(ad.gain, 0).await?,
            gain_x: handle.read_float64(self.gain_x, 0).await?,
            gain_y: handle.read_float64(self.gain_y, 0).await?,
            gain_red: handle.read_float64(self.gain_red, 0).await?,
            gain_green: handle.read_float64(self.gain_green, 0).await?,
            gain_blue: handle.read_float64(self.gain_blue, 0).await?,

            offset: handle.read_float64(self.offset, 0).await?,
            noise: handle.read_float64(self.noise, 0).await?,

            peak_start_x: handle.read_int32(self.peak_start_x, 0).await?,
            peak_start_y: handle.read_int32(self.peak_start_y, 0).await?,
            peak_width_x: handle.read_int32(self.peak_width_x, 0).await?,
            peak_width_y: handle.read_int32(self.peak_width_y, 0).await?,
            peak_num_x: handle.read_int32(self.peak_num_x, 0).await?,
            peak_num_y: handle.read_int32(self.peak_num_y, 0).await?,
            peak_step_x: handle.read_int32(self.peak_step_x, 0).await?,
            peak_step_y: handle.read_int32(self.peak_step_y, 0).await?,
            peak_height_variation: handle.read_float64(self.peak_height_variation, 0).await?,

            x_sine: SineParams {
                operation: SineOperation::from_i32(
                    handle.read_int32(self.x_sine_operation, 0).await?,
                ),
                amplitude1: handle.read_float64(self.x_sine1_amplitude, 0).await?,
                frequency1: handle.read_float64(self.x_sine1_frequency, 0).await?,
                phase1: handle.read_float64(self.x_sine1_phase, 0).await?,
                amplitude2: handle.read_float64(self.x_sine2_amplitude, 0).await?,
                frequency2: handle.read_float64(self.x_sine2_frequency, 0).await?,
                phase2: handle.read_float64(self.x_sine2_phase, 0).await?,
            },
            y_sine: SineParams {
                operation: SineOperation::from_i32(
                    handle.read_int32(self.y_sine_operation, 0).await?,
                ),
                amplitude1: handle.read_float64(self.y_sine1_amplitude, 0).await?,
                frequency1: handle.read_float64(self.y_sine1_frequency, 0).await?,
                phase1: handle.read_float64(self.y_sine1_phase, 0).await?,
                amplitude2: handle.read_float64(self.y_sine2_amplitude, 0).await?,
                frequency2: handle.read_float64(self.y_sine2_frequency, 0).await?,
                phase2: handle.read_float64(self.y_sine2_phase, 0).await?,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ADDriverBase;

    fn fixture() -> (ADDriverBase, SimParams) {
        let mut ad = ADDriverBase::new("SIMTEST", 64, 64, 0).unwrap();
        let sim = SimParams::create(&mut ad.port_base).unwrap();
        (ad, sim)
    }

    #[test]
    fn all_parameter_names_are_registered() {
        let (ad, _) = fixture();
        for name in [
            "SIM_GAIN_X",
            "SIM_GAIN_Y",
            "SIM_GAIN_RED",
            "SIM_GAIN_GREEN",
            "SIM_GAIN_BLUE",
            "SIM_OFFSET",
            "SIM_NOISE",
            "RESET_IMAGE",
            "SIM_MODE",
            "SIM_PEAK_START_X",
            "SIM_PEAK_START_Y",
            "SIM_PEAK_WIDTH_X",
            "SIM_PEAK_WIDTH_Y",
            "SIM_PEAK_NUM_X",
            "SIM_PEAK_NUM_Y",
            "SIM_PEAK_STEP_X",
            "SIM_PEAK_STEP_Y",
            "SIM_PEAK_HEIGHT_VARIATION",
            "SIM_XSINE_OPERATION",
            "SIM_YSINE_OPERATION",
            "SIM_XSINE1_AMPLITUDE",
            "SIM_XSINE1_FREQUENCY",
            "SIM_XSINE1_PHASE",
            "SIM_XSINE2_AMPLITUDE",
            "SIM_XSINE2_FREQUENCY",
            "SIM_XSINE2_PHASE",
            "SIM_YSINE1_AMPLITUDE",
            "SIM_YSINE1_FREQUENCY",
            "SIM_YSINE1_PHASE",
            "SIM_YSINE2_AMPLITUDE",
            "SIM_YSINE2_FREQUENCY",
            "SIM_YSINE2_PHASE",
        ] {
            assert!(ad.port_base.find_param(name).is_some(), "missing {name}");
        }
    }

    #[test]
    fn peak_index_range_covers_exactly_the_eight_geometry_peak_params() {
        // The `SimPeakStartX <= reason <= SimPeakStepY` test in writeInt32 relies
        // on these eight being contiguous, and on PeakHeightVariation (a double)
        // being outside the range.
        let (_, sim) = fixture();
        let expected = [
            sim.peak_start_x,
            sim.peak_start_y,
            sim.peak_width_x,
            sim.peak_width_y,
            sim.peak_num_x,
            sim.peak_num_y,
            sim.peak_step_x,
            sim.peak_step_y,
        ];
        assert_eq!(sim.peak_step_y - sim.peak_start_x + 1, 8);
        for (offset, idx) in expected.iter().enumerate() {
            assert_eq!(*idx, sim.peak_start_x + offset);
        }
        assert!(sim.peak_height_variation > sim.peak_step_y);
    }

    #[test]
    fn int32_writes_that_dirty_the_image() {
        let (ad, sim) = fixture();
        let p = &ad.params;
        for reason in [
            p.base.data_type,
            p.base.color_mode,
            sim.mode,
            sim.peak_start_x,
            sim.peak_start_y,
            sim.peak_width_x,
            sim.peak_width_y,
            sim.peak_num_x,
            sim.peak_num_y,
            sim.peak_step_x,
            sim.peak_step_y,
        ] {
            assert!(sim.int32_write_dirties_image(reason, p), "reason {reason}");
        }
    }

    #[test]
    fn int32_writes_that_do_not_dirty_the_image() {
        let (ad, sim) = fixture();
        let p = &ad.params;
        for reason in [
            p.acquire,
            p.size_x,
            p.min_x,
            p.bin_x,
            p.reverse_x,
            p.image_mode,
            p.num_images,
            sim.reset_image,
            sim.x_sine_operation,
            sim.y_sine_operation,
        ] {
            assert!(!sim.int32_write_dirties_image(reason, p), "reason {reason}");
        }
    }

    #[test]
    fn float64_writes_that_dirty_the_image() {
        let (ad, sim) = fixture();
        let gain = ad.params.gain;
        for reason in [
            gain,
            sim.gain_x,
            sim.gain_y,
            sim.gain_red,
            sim.gain_green,
            sim.gain_blue,
            sim.offset,
            sim.noise,
            sim.peak_height_variation,
            sim.x_sine1_amplitude,
            sim.x_sine2_phase,
            sim.y_sine1_frequency,
            sim.y_sine2_phase,
        ] {
            assert!(sim.float64_write_dirties_image(reason, gain), "{reason}");
        }
    }

    #[test]
    fn float64_writes_to_base_params_do_not_dirty_the_image() {
        let (ad, sim) = fixture();
        let gain = ad.params.gain;
        for reason in [
            ad.params.acquire_time,
            ad.params.acquire_period,
            ad.params.temperature,
            ad.params.shutter_open_delay,
        ] {
            assert!(!sim.float64_write_dirties_image(reason, gain), "{reason}");
        }
    }

    #[test]
    fn owns_splits_base_params_from_simulation_params() {
        let (ad, sim) = fixture();
        assert!(!sim.owns(ad.params.acquire));
        assert!(!sim.owns(ad.params.gain));
        assert!(sim.owns(sim.gain_x));
        assert!(sim.owns(sim.y_sine2_phase));
        assert_eq!(sim.first_sim_param(), sim.gain_x);
    }
}
