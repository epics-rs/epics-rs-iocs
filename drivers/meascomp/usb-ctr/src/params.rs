use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use mca::interface::McaReason;

/// 8 counters + 1 digital I/O channel in MCS mode.
pub const MAX_MCS_COUNTERS: usize = 9;
/// Counters on the largest board (USB-CTR08); the actual number comes from
/// [`CtrModel::num_counters`].
pub const MAX_COUNTERS: usize = 8;
pub const NUM_TIMERS: usize = 4;
pub const NUM_IO_BITS: usize = 8;
pub const DIGITAL_IO_COUNTER: usize = MAX_MCS_COUNTERS - 1;

/// The boards drvUSBCTR drives, with C's MODEL values (drvUSBCTR.cpp:364-372).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CtrModel {
    Ctr08 = 0,
    Ctr04 = 1,
}

impl CtrModel {
    pub fn from_product_name(name: &str) -> Option<Self> {
        match name {
            "USB-CTR08" => Some(Self::Ctr08),
            "USB-CTR04" => Some(Self::Ctr04),
            _ => None,
        }
    }

    /// C `numCounters_`: the counters the board has. Every per-counter loop
    /// is bounded by this; libuldaq rejects counter 4..7 on a CTR04.
    pub fn num_counters(self) -> usize {
        match self {
            Self::Ctr08 => 8,
            Self::Ctr04 => 4,
        }
    }
}

/// Parameter indices for the USB-CTR driver.
#[derive(Clone, Copy)]
pub struct CtrParams {
    // Board info
    pub model_name: usize,
    pub model_number: usize,
    pub model: usize,
    pub firmware_version: usize,
    pub unique_id: usize,
    pub ul_version: usize,
    pub driver_version: usize,
    pub poll_sleep_ms: usize,
    pub poll_time_ms: usize,
    pub last_error_message: usize,

    // Pulse generator (addr 0..3)
    pub pulse_run: usize,
    pub pulse_period: usize,
    pub pulse_duty_cycle: usize,
    pub pulse_delay: usize,
    pub pulse_count: usize,
    pub pulse_idle_state: usize,

    // Counter (addr 0..7)
    pub counter_value: usize,
    pub counter_reset: usize,

    // Trigger
    pub trigger_mode: usize,

    // Digital I/O
    pub digital_direction: usize,
    pub digital_input: usize,
    pub digital_output: usize,

    // MCS
    pub mcs_current_point: usize,
    pub mcs_max_points: usize,
    pub mcs_time_wf: usize,
    pub mcs_abs_time_wf: usize,
    pub mcs_counter_enable: usize,
    pub mcs_prescale_counter: usize,
    pub mcs_point0_action: usize,

    // MCA-compatible
    pub mca_start_acquire: usize,
    pub mca_stop_acquire: usize,
    pub mca_erase: usize,
    pub mca_data: usize,
    pub mca_num_channels: usize,
    pub mca_dwell_time: usize,
    pub mca_ch_advance_source: usize,
    pub mca_preset_real: usize,
    pub mca_acquiring: usize,
    pub mca_elapsed_real: usize,
    pub mca_elapsed_live: usize,
    pub mca_elapsed_counts: usize,
    pub mca_prescale: usize,
}

impl CtrParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        let mca = McaReason::create_params(base)?;
        Ok(Self {
            model_name: base.create_param("MODEL_NAME", ParamType::Octet)?,
            model_number: base.create_param("MODEL_NUMBER", ParamType::Int32)?,
            model: base.create_param("MODEL", ParamType::Int32)?,
            firmware_version: base.create_param("FIRMWARE_VERSION", ParamType::Octet)?,
            unique_id: base.create_param("UNIQUE_ID", ParamType::Octet)?,
            ul_version: base.create_param("UL_VERSION", ParamType::Octet)?,
            driver_version: base.create_param("DRIVER_VERSION", ParamType::Octet)?,
            poll_sleep_ms: base.create_param("POLL_SLEEP_MS", ParamType::Float64)?,
            poll_time_ms: base.create_param("POLL_TIME_MS", ParamType::Float64)?,
            last_error_message: base.create_param("LAST_ERROR_MESSAGE", ParamType::Octet)?,

            pulse_run: base.create_param("PULSE_RUN", ParamType::Int32)?,
            pulse_period: base.create_param("PULSE_PERIOD", ParamType::Float64)?,
            pulse_duty_cycle: base.create_param("PULSE_DUTY_CYCLE", ParamType::Float64)?,
            pulse_delay: base.create_param("PULSE_DELAY", ParamType::Float64)?,
            pulse_count: base.create_param("PULSE_COUNT", ParamType::Int32)?,
            pulse_idle_state: base.create_param("PULSE_IDLE_STATE", ParamType::Int32)?,

            counter_value: base.create_param("COUNTER_VALUE", ParamType::Int32)?,
            counter_reset: base.create_param("COUNTER_RESET", ParamType::Int32)?,

            trigger_mode: base.create_param("TRIGGER_MODE", ParamType::Int32)?,

            digital_direction: base.create_param("DIGITAL_DIRECTION", ParamType::UInt32Digital)?,
            digital_input: base.create_param("DIGITAL_INPUT", ParamType::UInt32Digital)?,
            digital_output: base.create_param("DIGITAL_OUTPUT", ParamType::UInt32Digital)?,

            mcs_current_point: base.create_param("MCS_CURRENT_POINT", ParamType::Int32)?,
            mcs_max_points: base.create_param("MCS_MAX_POINTS", ParamType::Int32)?,
            mcs_time_wf: base.create_param("MCS_TIME_WF", ParamType::Float32Array)?,
            mcs_abs_time_wf: base.create_param("MCS_ABS_TIME_WF", ParamType::Float64Array)?,
            mcs_counter_enable: base
                .create_param("MCS_COUNTER_ENABLE", ParamType::UInt32Digital)?,
            mcs_prescale_counter: base.create_param("MCS_PRESCALE_COUNTER", ParamType::Int32)?,
            mcs_point0_action: base.create_param("MCS_POINT0_ACTION", ParamType::Int32)?,

            // All 21 drvMca.h parameters under their drvMca.h names, as C
            // creates them (drvUSBCTR.cpp:331-351): an mca record's
            // devMcaAsyn resolves every one of them at init.
            mca_start_acquire: mca[McaReason::StartAcquire as usize],
            mca_stop_acquire: mca[McaReason::StopAcquire as usize],
            mca_erase: mca[McaReason::Erase as usize],
            mca_data: mca[McaReason::Data as usize],
            mca_num_channels: mca[McaReason::NumChannels as usize],
            mca_dwell_time: mca[McaReason::DwellTime as usize],
            mca_ch_advance_source: mca[McaReason::ChannelAdvanceSource as usize],
            mca_preset_real: mca[McaReason::PresetRealTime as usize],
            mca_acquiring: mca[McaReason::Acquiring as usize],
            mca_elapsed_real: mca[McaReason::ElapsedRealTime as usize],
            mca_elapsed_live: mca[McaReason::ElapsedLiveTime as usize],
            mca_elapsed_counts: mca[McaReason::ElapsedCounts as usize],
            mca_prescale: mca[McaReason::Prescale as usize],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_board_has_its_own_counter_count() {
        assert_eq!(
            CtrModel::from_product_name("USB-CTR08"),
            Some(CtrModel::Ctr08)
        );
        assert_eq!(
            CtrModel::from_product_name("USB-CTR04"),
            Some(CtrModel::Ctr04)
        );
        assert_eq!(CtrModel::Ctr08.num_counters(), 8);
        assert_eq!(CtrModel::Ctr04.num_counters(), 4);
        assert_eq!(CtrModel::Ctr04 as i32, 1);
    }

    #[test]
    fn another_board_is_not_a_usb_ctr() {
        assert_eq!(CtrModel::from_product_name("USB-2408-2AO"), None);
    }
}
