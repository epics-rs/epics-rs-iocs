//! The parameters BIS adds to the areaDetector base set (C `createParam`).

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

use crate::types::{
    BIS_EPICS_SHUTTER, BIS_NUM_DARKS, BIS_SFRM_TIMEOUT, BIS_START_SCAN, BIS_STATUS,
};

#[derive(Clone, Copy)]
pub struct BrukerParams {
    /// How long the frame file is waited for, on top of the exposure time.
    pub sfrm_timeout: usize,
    /// How many dark frames `[Dark]` collects.
    pub num_darks: usize,
    /// The last message BIS broadcast on the status socket.
    pub status: usize,
    /// Internal, no record: the acquisition task asks the actor to name the
    /// next frame's file and start the scan.
    pub start_scan: usize,
    /// Internal, no record: the acquisition task asks the actor to open or
    /// close the EPICS shutter.
    pub epics_shutter: usize,
}

impl BrukerParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            sfrm_timeout: base.create_param(BIS_SFRM_TIMEOUT, ParamType::Float64)?,
            num_darks: base.create_param(BIS_NUM_DARKS, ParamType::Int32)?,
            status: base.create_param(BIS_STATUS, ParamType::Octet)?,
            start_scan: base.create_param(BIS_START_SCAN, ParamType::Int32)?,
            epics_shutter: base.create_param(BIS_EPICS_SHUTTER, ParamType::Int32)?,
        })
    }
}
