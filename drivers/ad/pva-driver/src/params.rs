use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// pvaDriver-specific parameter indices.
///
/// Mirrors C++ `pvaDriver::PVAOverrunCounter`/`PVAPvName`/
/// `PVAPvConnectionStatus` (`pvaDriver.h`) — the only parameters this driver
/// adds on top of `ADDriver`/`asynNDArrayDriver`.
#[derive(Clone, Copy)]
pub struct PvaParams {
    pub overrun_counter: usize,
    pub pv_name: usize,
    pub pv_connection_status: usize,
}

impl PvaParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            overrun_counter: base.create_param("OVERRUN_COUNTER", ParamType::Int32)?,
            pv_name: base.create_param("PV_NAME", ParamType::Octet)?,
            pv_connection_status: base.create_param("PV_CONNECTION", ParamType::Int32)?,
        })
    }
}
