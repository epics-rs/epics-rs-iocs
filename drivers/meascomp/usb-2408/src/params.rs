use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

/// C `MAX_ANALOG_IN` (drvMultiFunction.cpp:212): the 2408 has 16 inputs.
pub const MAX_ANALOG_IN: usize = 16;
pub const MAX_ANALOG_OUT: usize = 2;
pub const MAX_COUNTERS: usize = 2;
pub const NUM_IO_BITS: usize = 8;
pub const MAX_SIGNALS: usize = 64; // same as C++ MAX_TEMPERATURE_IN

/// Parameter indices for the USB-2408-2AO driver.
///
/// An index is the asyn reason C prints as `function=N`, so every param of
/// C's constructor is here, in its order, whether or not a 2408 uses it.
#[derive(Clone, Copy)]
pub struct MultiFunctionParams {
    // Model parameters
    pub model_name: usize,
    pub model_number: usize,
    pub firmware_version: usize,
    pub unique_id: usize,
    pub ul_version: usize,
    pub driver_version: usize,
    pub poll_sleep_ms: usize,
    pub poll_time_ms: usize,
    pub last_error_message: usize,

    // Pulse generator parameters
    pub pulse_gen_run: usize,
    pub pulse_gen_period: usize,
    pub pulse_gen_duty_cycle: usize,
    pub pulse_gen_delay: usize,
    pub pulse_gen_count: usize,
    pub pulse_gen_idle_state: usize,

    // Counter parameters
    pub counter_value: usize,
    pub counter_reset: usize,

    // Analog input parameters
    pub analog_in_value: usize,
    pub analog_in_range: usize,
    pub analog_in_type: usize,
    pub analog_in_mode: usize,
    pub analog_in_rate: usize,

    // Voltage input parameters
    pub voltage_in_value: usize,
    pub voltage_in_range: usize,

    // Temperature parameters
    pub temperature_in_value: usize,
    pub thermocouple_type: usize,
    pub thermocouple_open_detect: usize,
    pub temperature_scale: usize,
    pub temperature_filter: usize,
    pub temperature_sensor: usize,
    pub temperature_wiring: usize,

    // Waveform digitizer parameters - global
    pub wave_dig_dwell: usize,
    pub wave_dig_dwell_actual: usize,
    pub wave_dig_total_time: usize,
    pub wave_dig_first_chan: usize,
    pub wave_dig_num_chans: usize,
    pub wave_dig_num_points: usize,
    pub wave_dig_current_point: usize,
    pub wave_dig_ext_trigger: usize,
    pub wave_dig_ext_clock: usize,
    pub wave_dig_continuous: usize,
    pub wave_dig_auto_restart: usize,
    pub wave_dig_retrigger: usize,
    pub wave_dig_trigger_count: usize,
    pub wave_dig_burst_mode: usize,
    pub wave_dig_run: usize,
    pub wave_dig_time_wf: usize,
    pub wave_dig_abs_time_wf: usize,
    pub wave_dig_read_wf: usize,

    // Waveform digitizer parameters - per input
    pub wave_dig_volt_wf: usize,

    // Analog output parameters
    pub analog_out_value: usize,
    pub analog_out_range: usize,
    pub analog_out_sync_master: usize,
    pub analog_out_sync_enable: usize,
    pub analog_out_sync_write: usize,

    // Waveform generator parameters - global
    pub wave_gen_freq: usize,
    pub wave_gen_dwell: usize,
    pub wave_gen_dwell_actual: usize,
    pub wave_gen_total_time: usize,
    pub wave_gen_num_points: usize,
    pub wave_gen_current_point: usize,
    pub wave_gen_int_dwell: usize,
    pub wave_gen_user_dwell: usize,
    pub wave_gen_int_num_points: usize,
    pub wave_gen_user_num_points: usize,
    pub wave_gen_ext_trigger: usize,
    pub wave_gen_ext_clock: usize,
    pub wave_gen_continuous: usize,
    pub wave_gen_retrigger: usize,
    pub wave_gen_trigger_count: usize,
    pub wave_gen_run: usize,
    pub wave_gen_user_time_wf: usize,
    pub wave_gen_int_time_wf: usize,

    // Waveform generator parameters - per output
    pub wave_gen_wave_type: usize,
    pub wave_gen_enable: usize,
    pub wave_gen_amplitude: usize,
    pub wave_gen_offset: usize,
    pub wave_gen_pulse_width: usize,
    pub wave_gen_pulse_delay: usize,
    pub wave_gen_int_wf: usize,
    pub wave_gen_user_wf: usize,

    // Trigger parameters
    pub trigger_mode: usize,

    // Digital I/O parameters
    pub digital_direction: usize,
    pub digital_input: usize,
    pub digital_output: usize,
}

impl MultiFunctionParams {
    /// C `MultiFunction::MultiFunction` (drvMultiFunction.cpp:854-960).
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            model_name: base.create_param("MODEL_NAME", ParamType::Octet)?,
            model_number: base.create_param("MODEL_NUMBER", ParamType::Int32)?,
            firmware_version: base.create_param("FIRMWARE_VERSION", ParamType::Octet)?,
            unique_id: base.create_param("UNIQUE_ID", ParamType::Octet)?,
            ul_version: base.create_param("UL_VERSION", ParamType::Octet)?,
            driver_version: base.create_param("DRIVER_VERSION", ParamType::Octet)?,
            poll_sleep_ms: base.create_param("POLL_SLEEP_MS", ParamType::Float64)?,
            poll_time_ms: base.create_param("POLL_TIME_MS", ParamType::Float64)?,
            last_error_message: base.create_param("LAST_ERROR_MESSAGE", ParamType::Octet)?,

            pulse_gen_run: base.create_param("PULSE_RUN", ParamType::Int32)?,
            pulse_gen_period: base.create_param("PULSE_PERIOD", ParamType::Float64)?,
            pulse_gen_duty_cycle: base.create_param("PULSE_DUTY_CYCLE", ParamType::Float64)?,
            pulse_gen_delay: base.create_param("PULSE_DELAY", ParamType::Float64)?,
            pulse_gen_count: base.create_param("PULSE_COUNT", ParamType::Int32)?,
            pulse_gen_idle_state: base.create_param("PULSE_IDLE_STATE", ParamType::Int32)?,

            counter_value: base.create_param("COUNTER_VALUE", ParamType::Int32)?,
            counter_reset: base.create_param("COUNTER_RESET", ParamType::Int32)?,

            analog_in_value: base.create_param("ANALOG_IN_VALUE", ParamType::Int32)?,
            analog_in_range: base.create_param("ANALOG_IN_RANGE", ParamType::Int32)?,
            analog_in_type: base.create_param("ANALOG_IN_TYPE", ParamType::Int32)?,
            analog_in_mode: base.create_param("ANALOG_IN_MODE", ParamType::Int32)?,
            analog_in_rate: base.create_param("ANALOG_IN_RATE", ParamType::Int32)?,

            voltage_in_value: base.create_param("VOLTAGE_IN_VALUE", ParamType::Float64)?,
            voltage_in_range: base.create_param("VOLTAGE_IN_RANGE", ParamType::Int32)?,

            temperature_in_value: base.create_param("TEMPERATURE_IN_VALUE", ParamType::Float64)?,
            thermocouple_type: base.create_param("THERMOCOUPLE_TYPE", ParamType::Int32)?,
            thermocouple_open_detect: base
                .create_param("THERMOCOUPLE_OPEN_DETECT", ParamType::Int32)?,
            temperature_scale: base.create_param("TEMPERATURE_SCALE", ParamType::Int32)?,
            temperature_filter: base.create_param("TEMPERATURE_FILTER", ParamType::Int32)?,
            temperature_sensor: base.create_param("TEMPERATURE_SENSOR", ParamType::Int32)?,
            temperature_wiring: base.create_param("TEMPERATURE_WIRING", ParamType::Int32)?,

            wave_dig_dwell: base.create_param("WAVEDIG_DWELL", ParamType::Float64)?,
            wave_dig_dwell_actual: base.create_param("WAVEDIG_DWELL_ACTUAL", ParamType::Float64)?,
            wave_dig_total_time: base.create_param("WAVEDIG_TOTAL_TIME", ParamType::Float64)?,
            wave_dig_first_chan: base.create_param("WAVEDIG_FIRST_CHAN", ParamType::Int32)?,
            wave_dig_num_chans: base.create_param("WAVEDIG_NUM_CHANS", ParamType::Int32)?,
            wave_dig_num_points: base.create_param("WAVEDIG_NUM_POINTS", ParamType::Int32)?,
            wave_dig_current_point: base.create_param("WAVEDIG_CURRENT_POINT", ParamType::Int32)?,
            wave_dig_ext_trigger: base.create_param("WAVEDIG_EXT_TRIGGER", ParamType::Int32)?,
            wave_dig_ext_clock: base.create_param("WAVEDIG_EXT_CLOCK", ParamType::Int32)?,
            wave_dig_continuous: base.create_param("WAVEDIG_CONTINUOUS", ParamType::Int32)?,
            wave_dig_auto_restart: base.create_param("WAVEDIG_AUTO_RESTART", ParamType::Int32)?,
            wave_dig_retrigger: base.create_param("WAVEDIG_RETRIGGER", ParamType::Int32)?,
            wave_dig_trigger_count: base.create_param("WAVEDIG_TRIGGER_COUNT", ParamType::Int32)?,
            wave_dig_burst_mode: base.create_param("WAVEDIG_BURST_MODE", ParamType::Int32)?,
            wave_dig_run: base.create_param("WAVEDIG_RUN", ParamType::Int32)?,
            wave_dig_time_wf: base.create_param("WAVEDIG_TIME_WF", ParamType::Float32Array)?,
            wave_dig_abs_time_wf: base
                .create_param("WAVEDIG_ABS_TIME_WF", ParamType::Float64Array)?,
            wave_dig_read_wf: base.create_param("WAVEDIG_READ_WF", ParamType::Int32)?,

            // C posts it on asynFloat64Array (:1899); see MultiFunctionDriver::post.
            wave_dig_volt_wf: base.create_param("WAVEDIG_VOLT_WF", ParamType::Float32Array)?,

            analog_out_value: base.create_param("ANALOG_OUT_VALUE", ParamType::Int32)?,
            analog_out_range: base.create_param("ANALOG_OUT_RANGE", ParamType::Int32)?,
            analog_out_sync_master: base
                .create_param("ANALOG_OUT_SYNC_MASTER", ParamType::Int32)?,
            analog_out_sync_enable: base
                .create_param("ANALOG_OUT_SYNC_ENABLE", ParamType::Int32)?,
            analog_out_sync_write: base.create_param("ANALOG_OUT_SYNC_WRITE", ParamType::Int32)?,

            wave_gen_freq: base.create_param("WAVEGEN_FREQ", ParamType::Float64)?,
            wave_gen_dwell: base.create_param("WAVEGEN_DWELL", ParamType::Float64)?,
            wave_gen_dwell_actual: base.create_param("WAVEGEN_DWELL_ACTUAL", ParamType::Float64)?,
            wave_gen_total_time: base.create_param("WAVEGEN_TOTAL_TIME", ParamType::Float64)?,
            wave_gen_num_points: base.create_param("WAVEGEN_NUM_POINTS", ParamType::Int32)?,
            wave_gen_current_point: base.create_param("WAVEGEN_CURRENT_POINT", ParamType::Int32)?,
            wave_gen_int_dwell: base.create_param("WAVEGEN_INT_DWELL", ParamType::Float64)?,
            wave_gen_user_dwell: base.create_param("WAVEGEN_USER_DWELL", ParamType::Float64)?,
            wave_gen_int_num_points: base
                .create_param("WAVEGEN_INT_NUM_POINTS", ParamType::Int32)?,
            wave_gen_user_num_points: base
                .create_param("WAVEGEN_USER_NUM_POINTS", ParamType::Int32)?,
            wave_gen_ext_trigger: base.create_param("WAVEGEN_EXT_TRIGGER", ParamType::Int32)?,
            wave_gen_ext_clock: base.create_param("WAVEGEN_EXT_CLOCK", ParamType::Int32)?,
            wave_gen_continuous: base.create_param("WAVEGEN_CONTINUOUS", ParamType::Int32)?,
            wave_gen_retrigger: base.create_param("WAVEGEN_RETRIGGER", ParamType::Int32)?,
            wave_gen_trigger_count: base.create_param("WAVEGEN_TRIGGER_COUNT", ParamType::Int32)?,
            wave_gen_run: base.create_param("WAVEGEN_RUN", ParamType::Int32)?,
            wave_gen_user_time_wf: base
                .create_param("WAVEGEN_USER_TIME_WF", ParamType::Float32Array)?,
            wave_gen_int_time_wf: base
                .create_param("WAVEGEN_INT_TIME_WF", ParamType::Float32Array)?,

            wave_gen_wave_type: base.create_param("WAVEGEN_WAVE_TYPE", ParamType::Int32)?,
            wave_gen_enable: base.create_param("WAVEGEN_ENABLE", ParamType::Int32)?,
            wave_gen_amplitude: base.create_param("WAVEGEN_AMPLITUDE", ParamType::Float64)?,
            wave_gen_offset: base.create_param("WAVEGEN_OFFSET", ParamType::Float64)?,
            wave_gen_pulse_width: base.create_param("WAVEGEN_PULSE_WIDTH", ParamType::Float64)?,
            wave_gen_pulse_delay: base.create_param("WAVEGEN_PULSE_DELAY", ParamType::Float64)?,
            wave_gen_int_wf: base.create_param("WAVEGEN_INT_WF", ParamType::Float32Array)?,
            wave_gen_user_wf: base.create_param("WAVEGEN_USER_WF", ParamType::Float32Array)?,

            trigger_mode: base.create_param("TRIGGER_MODE", ParamType::Int32)?,

            digital_direction: base.create_param("DIGITAL_DIRECTION", ParamType::UInt32Digital)?,
            digital_input: base.create_param("DIGITAL_INPUT", ParamType::UInt32Digital)?,
            digital_output: base.create_param("DIGITAL_OUTPUT", ParamType::UInt32Digital)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::{InterfaceMask, PortFlags};

    /// drvMultiFunction.cpp:854-960, in order.
    const C_PARAMS: [&str; 85] = [
        "MODEL_NAME",
        "MODEL_NUMBER",
        "FIRMWARE_VERSION",
        "UNIQUE_ID",
        "UL_VERSION",
        "DRIVER_VERSION",
        "POLL_SLEEP_MS",
        "POLL_TIME_MS",
        "LAST_ERROR_MESSAGE",
        "PULSE_RUN",
        "PULSE_PERIOD",
        "PULSE_DUTY_CYCLE",
        "PULSE_DELAY",
        "PULSE_COUNT",
        "PULSE_IDLE_STATE",
        "COUNTER_VALUE",
        "COUNTER_RESET",
        "ANALOG_IN_VALUE",
        "ANALOG_IN_RANGE",
        "ANALOG_IN_TYPE",
        "ANALOG_IN_MODE",
        "ANALOG_IN_RATE",
        "VOLTAGE_IN_VALUE",
        "VOLTAGE_IN_RANGE",
        "TEMPERATURE_IN_VALUE",
        "THERMOCOUPLE_TYPE",
        "THERMOCOUPLE_OPEN_DETECT",
        "TEMPERATURE_SCALE",
        "TEMPERATURE_FILTER",
        "TEMPERATURE_SENSOR",
        "TEMPERATURE_WIRING",
        "WAVEDIG_DWELL",
        "WAVEDIG_DWELL_ACTUAL",
        "WAVEDIG_TOTAL_TIME",
        "WAVEDIG_FIRST_CHAN",
        "WAVEDIG_NUM_CHANS",
        "WAVEDIG_NUM_POINTS",
        "WAVEDIG_CURRENT_POINT",
        "WAVEDIG_EXT_TRIGGER",
        "WAVEDIG_EXT_CLOCK",
        "WAVEDIG_CONTINUOUS",
        "WAVEDIG_AUTO_RESTART",
        "WAVEDIG_RETRIGGER",
        "WAVEDIG_TRIGGER_COUNT",
        "WAVEDIG_BURST_MODE",
        "WAVEDIG_RUN",
        "WAVEDIG_TIME_WF",
        "WAVEDIG_ABS_TIME_WF",
        "WAVEDIG_READ_WF",
        "WAVEDIG_VOLT_WF",
        "ANALOG_OUT_VALUE",
        "ANALOG_OUT_RANGE",
        "ANALOG_OUT_SYNC_MASTER",
        "ANALOG_OUT_SYNC_ENABLE",
        "ANALOG_OUT_SYNC_WRITE",
        "WAVEGEN_FREQ",
        "WAVEGEN_DWELL",
        "WAVEGEN_DWELL_ACTUAL",
        "WAVEGEN_TOTAL_TIME",
        "WAVEGEN_NUM_POINTS",
        "WAVEGEN_CURRENT_POINT",
        "WAVEGEN_INT_DWELL",
        "WAVEGEN_USER_DWELL",
        "WAVEGEN_INT_NUM_POINTS",
        "WAVEGEN_USER_NUM_POINTS",
        "WAVEGEN_EXT_TRIGGER",
        "WAVEGEN_EXT_CLOCK",
        "WAVEGEN_CONTINUOUS",
        "WAVEGEN_RETRIGGER",
        "WAVEGEN_TRIGGER_COUNT",
        "WAVEGEN_RUN",
        "WAVEGEN_USER_TIME_WF",
        "WAVEGEN_INT_TIME_WF",
        "WAVEGEN_WAVE_TYPE",
        "WAVEGEN_ENABLE",
        "WAVEGEN_AMPLITUDE",
        "WAVEGEN_OFFSET",
        "WAVEGEN_PULSE_WIDTH",
        "WAVEGEN_PULSE_DELAY",
        "WAVEGEN_INT_WF",
        "WAVEGEN_USER_WF",
        "TRIGGER_MODE",
        "DIGITAL_DIRECTION",
        "DIGITAL_INPUT",
        "DIGITAL_OUTPUT",
    ];

    #[test]
    fn every_param_has_its_index_in_cs_constructor() {
        let mut base = PortDriverBase::new(
            "MFPARAMS",
            MAX_SIGNALS,
            PortFlags::default(),
            InterfaceMask::UINT32_DIGITAL
                | InterfaceMask::INT32
                | InterfaceMask::INT32_ARRAY
                | InterfaceMask::FLOAT32_ARRAY
                | InterfaceMask::FLOAT64
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::OCTET
                | InterfaceMask::ENUM
                | InterfaceMask::DRV_USER,
            InterfaceMask::UINT32_DIGITAL
                | InterfaceMask::INT32
                | InterfaceMask::INT32_ARRAY
                | InterfaceMask::FLOAT32_ARRAY
                | InterfaceMask::FLOAT64
                | InterfaceMask::FLOAT64_ARRAY
                | InterfaceMask::OCTET
                | InterfaceMask::ENUM,
        );
        MultiFunctionParams::create(&mut base).unwrap();
        let names: Vec<_> = (0..base.params.len())
            .map(|i| base.params.param_name(i).unwrap())
            .collect();
        assert_eq!(names, C_PARAMS);
    }
}
