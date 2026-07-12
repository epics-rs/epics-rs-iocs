//! The parameters PhotonII adds to `ADDriver` (C `createParam` block).

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

use crate::types::*;

#[derive(Debug, Clone, Copy)]
pub struct PhotonIIParams {
    pub dr_sum_enable: usize,
    pub num_darks: usize,
    pub trigger_type: usize,
    pub trigger_edge: usize,
    pub num_subframes: usize,
    /// Internal: carries a raw p2util command line from the `p2util` iocsh
    /// command to the actor, which owns the socket.
    pub util: usize,
    /// Internal: lets the acquisition task open and close the EPICS shutter
    /// through the actor.
    pub shutter: usize,
}

impl PhotonIIParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            dr_sum_enable: base.create_param(PII_DRSUM_ENABLE, ParamType::Int32)?,
            num_darks: base.create_param(PII_NUM_DARKS, ParamType::Int32)?,
            trigger_type: base.create_param(PII_TRIGGER_TYPE, ParamType::Int32)?,
            trigger_edge: base.create_param(PII_TRIGGER_EDGE, ParamType::Int32)?,
            num_subframes: base.create_param(PII_NUM_SUBFRAMES, ParamType::Int32)?,
            util: base.create_param(PII_UTIL, ParamType::Octet)?,
            shutter: base.create_param(PII_SHUTTER, ParamType::Int32)?,
        })
    }
}
