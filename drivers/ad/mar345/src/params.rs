//! The 8 mar345-specific asyn parameters.
//!
//! `drvInfo` strings are identical to `mar345.cpp`, so the C `mar345.template`
//! records bind unchanged.

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// Parameter indices, in the same creation order as C's constructor. The first
/// (`erase`) plays the role of C's `FIRST_MAR345_PARAM`; the last (`abort`) is
/// `LAST_MAR345_PARAM`.
#[derive(Debug, Clone, Copy)]
pub struct Mar345Params {
    pub erase: usize,
    pub erase_mode: usize,
    pub num_erase: usize,
    pub num_erased: usize,
    pub change_mode: usize,
    pub size: usize,
    pub res: usize,
    pub abort: usize,
}

impl Mar345Params {
    pub fn create(port_base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            erase: port_base.create_param("MAR_ERASE", ParamType::Int32)?,
            erase_mode: port_base.create_param("MAR_ERASE_MODE", ParamType::Int32)?,
            num_erase: port_base.create_param("MAR_NUM_ERASE", ParamType::Int32)?,
            num_erased: port_base.create_param("MAR_NUM_ERASED", ParamType::Int32)?,
            change_mode: port_base.create_param("MAR_CHANGE_MODE", ParamType::Int32)?,
            size: port_base.create_param("MAR_SIZE", ParamType::Int32)?,
            res: port_base.create_param("MAR_RESOLUTION", ParamType::Int32)?,
            abort: port_base.create_param("MAR_ABORT", ParamType::Int32)?,
        })
    }

    /// C `FIRST_MAR345_PARAM` — a reason below this belongs to the base class.
    pub fn first(&self) -> usize {
        self.erase
    }
}
