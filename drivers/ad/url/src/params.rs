use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// URL-driver-specific parameter indices.
///
/// Mirrors C++ `URLDriver::URLName` — the only parameter this driver adds
/// on top of `ADDriver`/`asynNDArrayDriver`.
#[derive(Clone, Copy)]
pub struct URLParams {
    pub url_name: usize,
}

impl URLParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            url_name: base.create_param("URL_NAME", ParamType::Octet)?,
        })
    }
}
