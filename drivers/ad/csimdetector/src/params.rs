//! `ADCSimDetector`-specific asyn parameters (`ADCSimDetector.h:28-52`).

use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::PortHandle;

use crate::signals::{ChannelParams, SimConfig};
use crate::types::MAX_SIGNALS;

/// An undefined parameter reads as zero; see [`CSimParams::read_config`].
async fn read_f64(handle: &PortHandle, reason: usize, addr: i32) -> f64 {
    handle.read_float64(reason, addr).await.unwrap_or(0.0)
}

/// Parameter indices, created in the same order as the C constructor
/// (ADCSimDetector.cpp:82-91). The order is load-bearing: `writeInt32` forwards
/// a write to the base class when `function < FIRST_SIM_DETECTOR_PARAM`.
///
/// `SIM_ACQUIRE` is declared in `ADCSimDetector.h:28` but never passed to
/// `createParam`, so it does not exist as a parameter; it is not created here
/// either.
#[derive(Clone, Copy)]
pub struct CSimParams {
    /// `FIRST_SIM_DETECTOR_PARAM` — every index at or above this belongs to the
    /// simulation driver rather than to `asynNDArrayDriver`.
    pub acquire_time: usize,
    pub elapsed_time: usize,
    pub time_step: usize,
    pub num_time_points: usize,
    pub period: usize,
    pub amplitude: usize,
    pub offset: usize,
    pub frequency: usize,
    pub phase: usize,
    pub noise: usize,
}

impl CSimParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            acquire_time: base.create_param("SIM_ACQUIRE_TIME", ParamType::Float64)?,
            elapsed_time: base.create_param("SIM_ELAPSED_TIME", ParamType::Float64)?,
            time_step: base.create_param("SIM_TIME_STEP", ParamType::Float64)?,
            num_time_points: base.create_param("SIM_NUM_TIME_POINTS", ParamType::Int32)?,
            period: base.create_param("SIM_PERIOD", ParamType::Float64)?,
            amplitude: base.create_param("SIM_AMPLITUDE", ParamType::Float64)?,
            offset: base.create_param("SIM_OFFSET", ParamType::Float64)?,
            frequency: base.create_param("SIM_FREQUENCY", ParamType::Float64)?,
            phase: base.create_param("SIM_PHASE", ParamType::Float64)?,
            noise: base.create_param("SIM_NOISE", ParamType::Float64)?,
        })
    }

    /// `FIRST_SIM_DETECTOR_PARAM` (ADCSimDetector.h:41).
    pub fn first_sim_param(&self) -> usize {
        self.acquire_time
    }

    /// `function < FIRST_SIM_DETECTOR_PARAM` — the write must be forwarded to
    /// `asynNDArrayDriver::writeInt32` (ADCSimDetector.cpp:376).
    pub fn belongs_to_base(&self, reason: usize) -> bool {
        reason < self.first_sim_param()
    }

    /// Everything `computeArraysT` reads before filling the array
    /// (ADCSimDetector.cpp:139-161).
    ///
    /// Per-signal parameters live at asyn address `j` for signal `j`
    /// (`0..MAX_SIGNALS`); the array-wide ones live at address 0.
    ///
    /// Every read is infallible because C discards the `getDoubleParam` /
    /// `getIntegerParam` return codes here: on `asynParamUndefined` the C local
    /// keeps whatever it held, which for `computeArraysT`'s fresh stack arrays
    /// is indeterminate. The constructor only defines the address-0 values, so
    /// addresses 1..7 stay undefined until the `PINI YES` records of
    /// `ADCSimDetectorN.template` post theirs. Zero is substituted for an
    /// undefined parameter.
    pub async fn read_config(&self, handle: &PortHandle, nd: &NDArrayDriverParams) -> SimConfig {
        let mut channels = [ChannelParams::default(); MAX_SIGNALS];
        for (j, c) in channels.iter_mut().enumerate() {
            let addr = j as i32;
            c.amplitude = read_f64(handle, self.amplitude, addr).await;
            c.offset = read_f64(handle, self.offset, addr).await;
            c.period = read_f64(handle, self.period, addr).await;
            c.phase = read_f64(handle, self.phase, addr).await;
            c.noise = read_f64(handle, self.noise, addr).await;
        }

        SimConfig {
            num_time_points: handle
                .read_int32(self.num_time_points, 0)
                .await
                .unwrap_or(0)
                .max(0) as usize,
            time_step: read_f64(handle, self.time_step, 0).await,
            acquire_time: read_f64(handle, self.acquire_time, 0).await,
            data_type: u8::try_from(handle.read_int32(nd.data_type, 0).await.unwrap_or(0))
                .ok()
                .and_then(NDDataType::from_ordinal)
                // C `switch (dataType)` has no default: an out-of-range value
                // leaves `pArrays[0]` at whatever `pNDArrayPool->alloc` returned
                // for it. There is no Rust equivalent of "allocate an array of
                // an unknown type", so fall back to the constructor default.
                .unwrap_or(NDDataType::Float64),
            channels,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::PortFlags;

    fn fixture() -> (PortDriverBase, NDArrayDriverParams, CSimParams) {
        let mut base = PortDriverBase::new(
            "CSIMTEST",
            MAX_SIGNALS + 1,
            PortFlags {
                can_block: true,
                multi_device: true,
                ..Default::default()
            },
        );
        let nd = NDArrayDriverParams::create(&mut base).unwrap();
        let sim = CSimParams::create(&mut base).unwrap();
        (base, nd, sim)
    }

    #[test]
    fn all_parameter_names_are_registered() {
        let (base, _, _) = fixture();
        for name in [
            "SIM_ACQUIRE_TIME",
            "SIM_ELAPSED_TIME",
            "SIM_TIME_STEP",
            "SIM_NUM_TIME_POINTS",
            "SIM_PERIOD",
            "SIM_AMPLITUDE",
            "SIM_OFFSET",
            "SIM_FREQUENCY",
            "SIM_PHASE",
            "SIM_NOISE",
        ] {
            assert!(base.find_param(name).is_some(), "missing {name}");
        }
    }

    #[test]
    fn sim_acquire_is_declared_in_the_header_but_never_created() {
        let (base, _, _) = fixture();
        assert!(base.find_param("SIM_ACQUIRE").is_none());
    }

    #[test]
    fn params_are_created_contiguously_in_the_c_order() {
        let (_, _, sim) = fixture();
        let order = [
            sim.acquire_time,
            sim.elapsed_time,
            sim.time_step,
            sim.num_time_points,
            sim.period,
            sim.amplitude,
            sim.offset,
            sim.frequency,
            sim.phase,
            sim.noise,
        ];
        for (i, idx) in order.iter().enumerate() {
            assert_eq!(*idx, sim.acquire_time + i);
        }
    }

    #[test]
    fn base_class_params_sort_below_first_sim_detector_param() {
        let (_, nd, sim) = fixture();
        assert_eq!(sim.first_sim_param(), sim.acquire_time);
        for reason in [
            nd.acquire,
            nd.data_type,
            nd.array_counter,
            nd.array_callbacks,
        ] {
            assert!(sim.belongs_to_base(reason), "reason {reason}");
        }
        for reason in [sim.acquire_time, sim.num_time_points, sim.noise] {
            assert!(!sim.belongs_to_base(reason), "reason {reason}");
        }
    }
}
