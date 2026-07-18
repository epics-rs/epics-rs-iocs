//! The asyn MCA interface, ported from `mca.h` + `drvMca.h`
//! (`mcaApp/mcaSrc`) -- the contract every hardware MCA driver implements.
//! `mca.h`'s own comment states the contract precisely: only the drvInfo
//! strings here (not the `mcaCommand` enum's raw integer values) may be used
//! by device support to resolve `pasynUser->reason`, via
//! `asynDrvUser->create()`. A driver is free to use the enum's ordinal
//! values for its own `pasynUser->reason` numbering, but is not required to.
//!
//! PUBLIC: round-2 vendor driver crates (mca-rontec, mca-amptek, ...)
//! `create_param(reason.drv_info(), ...)` against this same table so
//! [`crate::dev_mca_asyn::DevMcaAsyn`] resolves against them identically to
//! how it resolves against [`crate::fastsweep`].

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// C `mcaCommand` (`mca.h:14-37`). `lastMcaCommand`, C's array-length
/// sentinel, has no Rust variant -- [`McaReason::COUNT`] (`== 21`) serves
/// the same role, and every command has an explicit `usize` discriminant so
/// `reason as usize` is a stable index into a `[T; McaReason::COUNT]` table
/// (`DevMcaAsyn::reasons`, a driver's own `create_param` results) without
/// relying on declaration order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum McaReason {
    Data = 0,
    StartAcquire = 1,
    StopAcquire = 2,
    Erase = 3,
    ReadStatus = 4,
    ChannelAdvanceSource = 5,
    NumChannels = 6,
    AcquireMode = 7,
    Sequence = 8,
    Prescale = 9,
    PresetSweeps = 10,
    PresetLowChannel = 11,
    PresetHighChannel = 12,
    DwellTime = 13,
    PresetLiveTime = 14,
    PresetRealTime = 15,
    PresetCounts = 16,
    Acquiring = 17,
    ElapsedLiveTime = 18,
    ElapsedRealTime = 19,
    ElapsedCounts = 20,
}

impl McaReason {
    /// C `#define MAX_MCA_COMMANDS lastMcaCommand` (`mca.h:39`).
    pub const COUNT: usize = 21;

    /// Every reason, in `mcaCommand`'s declaration order -- the order
    /// [`crate::dev_mca_asyn::DevMcaAsyn::init`] resolves drvInfo strings in,
    /// matching `devMcaAsyn.c:184-204`'s call sequence.
    pub const ALL: [McaReason; Self::COUNT] = [
        Self::Data,
        Self::StartAcquire,
        Self::StopAcquire,
        Self::Erase,
        Self::ReadStatus,
        Self::ChannelAdvanceSource,
        Self::NumChannels,
        Self::AcquireMode,
        Self::Sequence,
        Self::Prescale,
        Self::PresetSweeps,
        Self::PresetLowChannel,
        Self::PresetHighChannel,
        Self::DwellTime,
        Self::PresetLiveTime,
        Self::PresetRealTime,
        Self::PresetCounts,
        Self::Acquiring,
        Self::ElapsedLiveTime,
        Self::ElapsedRealTime,
        Self::ElapsedCounts,
    ];

    /// The C `mcaXxxString` drvInfo constant for this reason (`drvMca.h:19-39`).
    pub const fn drv_info(self) -> &'static str {
        match self {
            Self::Data => MCA_DATA,
            Self::StartAcquire => MCA_START_ACQUIRE,
            Self::StopAcquire => MCA_STOP_ACQUIRE,
            Self::Erase => MCA_ERASE,
            Self::ReadStatus => MCA_READ_STATUS,
            Self::ChannelAdvanceSource => MCA_CH_ADV_SOURCE,
            Self::NumChannels => MCA_NUM_CHANNELS,
            Self::AcquireMode => MCA_ACQUIRE_MODE,
            Self::Sequence => MCA_SEQUENCE,
            Self::Prescale => MCA_PRESCALE,
            Self::PresetSweeps => MCA_PRESET_SWEEPS,
            Self::PresetLowChannel => MCA_PRESET_LOW_CHANNEL,
            Self::PresetHighChannel => MCA_PRESET_HIGH_CHANNEL,
            Self::DwellTime => MCA_DWELL_TIME,
            Self::PresetLiveTime => MCA_PRESET_LIVE,
            Self::PresetRealTime => MCA_PRESET_REAL,
            Self::PresetCounts => MCA_PRESET_COUNTS,
            Self::Acquiring => MCA_ACQUIRING,
            Self::ElapsedLiveTime => MCA_ELAPSED_LIVE,
            Self::ElapsedRealTime => MCA_ELAPSED_REAL,
            Self::ElapsedCounts => MCA_ELAPSED_COUNTS,
        }
    }

    /// The `ParamType` C's comment on each `mcaXxxString` define documents
    /// (`drvMca.h:19-39`) -- what a driver's own `create_param` call should
    /// use for this reason.
    pub const fn param_type(self) -> ParamType {
        match self {
            // C comments this "int32Array, read/write"; `createParam` in
            // both `devMCA_soft.c` and `drvFastSweep.cpp` nonetheless
            // registers it as a scalar `asynParamInt32` slot -- the actual
            // array I/O bypasses the parameter library entirely via
            // `asynInt32Array::read`/`readInt32Array`, so only the
            // resolved reason index (not this declared type) is ever used
            // for MCA_DATA. Reproduced verbatim for the same reason.
            Self::Data => ParamType::Int32,
            Self::DwellTime
            | Self::PresetLiveTime
            | Self::PresetRealTime
            | Self::PresetCounts
            | Self::ElapsedLiveTime
            | Self::ElapsedRealTime
            | Self::ElapsedCounts => ParamType::Float64,
            _ => ParamType::Int32,
        }
    }

    /// `createParam` every reason in [`Self::ALL`] order -- the same 21
    /// calls every conforming asyn MCA driver's constructor makes
    /// (`drvFastSweep.cpp:75-97`), factored out here so [`crate::fastsweep`]
    /// and round-2 vendor driver crates share one implementation instead of
    /// each hand-copying the loop. Returns a table indexed by
    /// `McaReason as usize`, the same shape
    /// [`crate::dev_mca_asyn::DevMcaAsyn`] resolves via `drvUserCreate`.
    pub fn create_params(base: &mut PortDriverBase) -> AsynResult<[usize; Self::COUNT]> {
        let mut reasons = [0usize; Self::COUNT];
        for reason in Self::ALL {
            reasons[reason as usize] = base.create_param(reason.drv_info(), reason.param_type())?;
        }
        Ok(reasons)
    }
}

/// C `enum MCAAcquireMode` (`drvMca.h:8-12`): "these enums must agree with
/// the MCA record. We define them here for drivers that should not know
/// about the MCA record." Carried on [`McaReason::AcquireMode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McaAcquireMode {
    Pha,
    Mcs,
    List,
}

impl McaAcquireMode {
    pub const fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Pha),
            1 => Some(Self::Mcs),
            2 => Some(Self::List),
            _ => None,
        }
    }

    pub const fn as_i32(self) -> i32 {
        match self {
            Self::Pha => 0,
            Self::Mcs => 1,
            Self::List => 2,
        }
    }
}

/// C `enum MCAChannelAdvanceSource` (`drvMca.h:14-17`). Carried on
/// [`McaReason::ChannelAdvanceSource`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McaChannelAdvanceSource {
    Internal,
    External,
}

impl McaChannelAdvanceSource {
    pub const fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Internal),
            1 => Some(Self::External),
            _ => None,
        }
    }

    pub const fn as_i32(self) -> i32 {
        match self {
            Self::Internal => 0,
            Self::External => 1,
        }
    }
}

pub const MCA_START_ACQUIRE: &str = "MCA_START_ACQUIRE";
pub const MCA_STOP_ACQUIRE: &str = "MCA_STOP_ACQUIRE";
pub const MCA_ERASE: &str = "MCA_ERASE";
pub const MCA_DATA: &str = "MCA_DATA";
pub const MCA_READ_STATUS: &str = "MCA_READ_STATUS";
pub const MCA_CH_ADV_SOURCE: &str = "MCA_CH_ADV_SOURCE";
pub const MCA_NUM_CHANNELS: &str = "MCA_NUM_CHANNELS";
pub const MCA_DWELL_TIME: &str = "MCA_DWELL_TIME";
pub const MCA_PRESET_LIVE: &str = "MCA_PRESET_LIVE";
pub const MCA_PRESET_REAL: &str = "MCA_PRESET_REAL";
pub const MCA_PRESET_COUNTS: &str = "MCA_PRESET_COUNTS";
pub const MCA_PRESET_LOW_CHANNEL: &str = "MCA_PRESET_LOW_CHANNEL";
pub const MCA_PRESET_HIGH_CHANNEL: &str = "MCA_PRESET_HIGH_CHANNEL";
pub const MCA_PRESET_SWEEPS: &str = "MCA_PRESET_SWEEPS";
pub const MCA_ACQUIRE_MODE: &str = "MCA_ACQUIRE_MODE";
pub const MCA_SEQUENCE: &str = "MCA_SEQUENCE";
pub const MCA_PRESCALE: &str = "MCA_PRESCALE";
pub const MCA_ACQUIRING: &str = "MCA_ACQUIRING";
pub const MCA_ELAPSED_LIVE: &str = "MCA_ELAPSED_LIVE";
pub const MCA_ELAPSED_REAL: &str = "MCA_ELAPSED_REAL";
pub const MCA_ELAPSED_COUNTS: &str = "MCA_ELAPSED_COUNTS";

/// `device(mca,INST_IO,devMcaAsyn,"asynMCA")` (`mcaSupport.dbd`) -- the DTYP
/// string a record must carry for [`crate::dev_mca_asyn::DevMcaAsyn`] to
/// bind to it.
pub const ASYN_MCA_DTYP: &str = "asynMCA";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_covers_every_reason_in_declaration_order() {
        assert_eq!(McaReason::ALL.len(), McaReason::COUNT);
        for (i, r) in McaReason::ALL.iter().enumerate() {
            assert_eq!(*r as usize, i);
        }
    }

    #[test]
    fn drv_info_strings_match_c_verbatim() {
        assert_eq!(McaReason::Data.drv_info(), "MCA_DATA");
        assert_eq!(McaReason::StartAcquire.drv_info(), "MCA_START_ACQUIRE");
        assert_eq!(McaReason::StopAcquire.drv_info(), "MCA_STOP_ACQUIRE");
        assert_eq!(McaReason::Erase.drv_info(), "MCA_ERASE");
        assert_eq!(McaReason::ReadStatus.drv_info(), "MCA_READ_STATUS");
        assert_eq!(
            McaReason::ChannelAdvanceSource.drv_info(),
            "MCA_CH_ADV_SOURCE"
        );
        assert_eq!(McaReason::NumChannels.drv_info(), "MCA_NUM_CHANNELS");
        assert_eq!(McaReason::AcquireMode.drv_info(), "MCA_ACQUIRE_MODE");
        assert_eq!(McaReason::Sequence.drv_info(), "MCA_SEQUENCE");
        assert_eq!(McaReason::Prescale.drv_info(), "MCA_PRESCALE");
        assert_eq!(McaReason::PresetSweeps.drv_info(), "MCA_PRESET_SWEEPS");
        assert_eq!(
            McaReason::PresetLowChannel.drv_info(),
            "MCA_PRESET_LOW_CHANNEL"
        );
        assert_eq!(
            McaReason::PresetHighChannel.drv_info(),
            "MCA_PRESET_HIGH_CHANNEL"
        );
        assert_eq!(McaReason::DwellTime.drv_info(), "MCA_DWELL_TIME");
        assert_eq!(McaReason::PresetLiveTime.drv_info(), "MCA_PRESET_LIVE");
        assert_eq!(McaReason::PresetRealTime.drv_info(), "MCA_PRESET_REAL");
        assert_eq!(McaReason::PresetCounts.drv_info(), "MCA_PRESET_COUNTS");
        assert_eq!(McaReason::Acquiring.drv_info(), "MCA_ACQUIRING");
        assert_eq!(McaReason::ElapsedLiveTime.drv_info(), "MCA_ELAPSED_LIVE");
        assert_eq!(McaReason::ElapsedRealTime.drv_info(), "MCA_ELAPSED_REAL");
        assert_eq!(McaReason::ElapsedCounts.drv_info(), "MCA_ELAPSED_COUNTS");
    }

    #[test]
    fn acquire_mode_round_trips() {
        for v in 0..3 {
            let mode = McaAcquireMode::from_i32(v).unwrap();
            assert_eq!(mode.as_i32(), v);
        }
        assert!(McaAcquireMode::from_i32(3).is_none());
    }

    #[test]
    fn channel_advance_source_round_trips() {
        for v in 0..2 {
            let src = McaChannelAdvanceSource::from_i32(v).unwrap();
            assert_eq!(src.as_i32(), v);
        }
        assert!(McaChannelAdvanceSource::from_i32(2).is_none());
    }
}
