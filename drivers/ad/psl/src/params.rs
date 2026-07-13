//! The parameters PSL adds to the areaDetector base set (C `createParam`).

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

use crate::types::{PSL_CAMERA_NAME, PSL_TIFF_COMMENT};

#[derive(Clone, Copy)]
pub struct PslParams {
    /// Which camera of `GetCamList` (plus `multiconf`) is open.
    pub camera_name: usize,
    /// The tag PSLViewer writes into the TIFF file it saves.
    pub tiff_comment: usize,
}

impl PslParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            camera_name: base.create_param(PSL_CAMERA_NAME, ParamType::Int32)?,
            tiff_comment: base.create_param(PSL_TIFF_COMMENT, ParamType::Octet)?,
        })
    }
}
