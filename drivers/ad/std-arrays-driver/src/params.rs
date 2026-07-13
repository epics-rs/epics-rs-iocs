//! `NDDriverStdArrays`-specific asyn parameters (NDDriverStdArrays.h:43-76),
//! created in the exact order of the C constructor
//! (NDDriverStdArrays.cpp:59-68). The order is load-bearing: `writeInt32`
//! forwards a write to the base class when `function < FIRST_NDSA_DRIVER_PARAM`
//! (NDDriverStdArrays.cpp:368).

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// `NDSA_CallbackMode_t` (NDDriverStdArrays.h:22-26): when `doCallbacks()`
/// fires relative to array assembly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CallbackMode {
    /// `NDSA_OnUpdate` (0): publish on every waveform write in append mode.
    OnUpdate = 0,
    /// `NDSA_OnComplete` (1): publish when the array is marked complete.
    OnComplete = 1,
    /// `NDSA_OnCommand` (2): publish only on the explicit DoCallbacks command.
    OnCommand = 2,
}

/// Parameter indices for the ten driver-specific parameters.
#[derive(Clone, Copy)]
pub struct NdsaParams {
    /// `FIRST_NDSA_DRIVER_PARAM` — every index at or above this belongs to the
    /// driver rather than to `ADDriver`/`asynNDArrayDriver`.
    pub callback_mode: usize,
    pub do_callbacks: usize,
    pub append_mode: usize,
    pub num_elements: usize,
    pub next_element: usize,
    pub stride: usize,
    pub fill_value: usize,
    pub new_array: usize,
    pub array_complete: usize,
    pub array_data: usize,
}

impl NdsaParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            callback_mode: base.create_param("NDSA_CALLBACK_MODE", ParamType::Int32)?,
            do_callbacks: base.create_param("NDSA_DO_CALLBACKS", ParamType::Int32)?,
            append_mode: base.create_param("NDSA_APPEND_MODE", ParamType::Int32)?,
            num_elements: base.create_param("NDSA_NUM_ELEMENTS", ParamType::Int32)?,
            next_element: base.create_param("NDSA_NEXT_ELEMENT", ParamType::Int32)?,
            stride: base.create_param("NDSA_STRIDE", ParamType::Int32)?,
            fill_value: base.create_param("NDSA_FILL_VALUE", ParamType::Float64)?,
            new_array: base.create_param("NDSA_NEW_ARRAY", ParamType::Int32)?,
            array_complete: base.create_param("NDSA_ARRAY_COMPLETE", ParamType::Int32)?,
            // C creates NDSA_ArrayData_ as asynParamInt32 (NDDriverStdArrays.cpp:68);
            // the reason index is what the asynXXXArray writes target.
            array_data: base.create_param("NDSA_ARRAY_DATA", ParamType::Int32)?,
        })
    }

    /// `FIRST_NDSA_DRIVER_PARAM` (NDDriverStdArrays.h:44).
    pub fn first_param(&self) -> usize {
        self.callback_mode
    }

    /// `function < FIRST_NDSA_DRIVER_PARAM` — the write must be forwarded to
    /// `ADDriver::writeInt32` (NDDriverStdArrays.cpp:368).
    pub fn belongs_to_base(&self, reason: usize) -> bool {
        reason < self.first_param()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::params::ad_driver::ADDriverParams;
    use epics_rs::asyn::port::PortFlags;

    fn fixture() -> (PortDriverBase, ADDriverParams, NdsaParams) {
        let mut base = PortDriverBase::new("NDSATEST", 1, PortFlags::default());
        let ad = ADDriverParams::create(&mut base).unwrap();
        let ndsa = NdsaParams::create(&mut base).unwrap();
        (base, ad, ndsa)
    }

    #[test]
    fn all_parameter_names_are_registered() {
        let (base, _, _) = fixture();
        for name in [
            "NDSA_CALLBACK_MODE",
            "NDSA_DO_CALLBACKS",
            "NDSA_APPEND_MODE",
            "NDSA_NUM_ELEMENTS",
            "NDSA_NEXT_ELEMENT",
            "NDSA_STRIDE",
            "NDSA_FILL_VALUE",
            "NDSA_NEW_ARRAY",
            "NDSA_ARRAY_COMPLETE",
            "NDSA_ARRAY_DATA",
        ] {
            assert!(base.find_param(name).is_some(), "missing {name}");
        }
    }

    #[test]
    fn params_are_created_contiguously_in_the_c_order() {
        let (_, _, ndsa) = fixture();
        let order = [
            ndsa.callback_mode,
            ndsa.do_callbacks,
            ndsa.append_mode,
            ndsa.num_elements,
            ndsa.next_element,
            ndsa.stride,
            ndsa.fill_value,
            ndsa.new_array,
            ndsa.array_complete,
            ndsa.array_data,
        ];
        for (i, idx) in order.iter().enumerate() {
            assert_eq!(*idx, ndsa.callback_mode + i);
        }
    }

    #[test]
    fn base_class_params_sort_below_the_first_driver_param() {
        let (_, ad, ndsa) = fixture();
        assert_eq!(ndsa.first_param(), ndsa.callback_mode);
        for reason in [ad.acquire, ad.base.data_type, ad.image_mode, ad.num_images] {
            assert!(ndsa.belongs_to_base(reason), "reason {reason}");
        }
        for reason in [ndsa.callback_mode, ndsa.array_data, ndsa.stride] {
            assert!(!ndsa.belongs_to_base(reason), "reason {reason}");
        }
    }
}
