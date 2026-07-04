use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;

pub const MAX_ANALOG_IN: usize = 8;
pub const MAX_ANALOG_OUT: usize = 2;
pub const MAX_COUNTERS: usize = 2;
pub const NUM_IO_BITS: usize = 8;
pub const MAX_SIGNALS: usize = 64; // same as C++ MAX_TEMPERATURE_IN

/// Parameter indices for the USB-2408-2AO driver.
#[derive(Clone, Copy)]
pub struct MultiFunctionParams {
    // Board info
    pub model_name: usize,
    pub model_number: usize,
    pub firmware_version: usize,
    pub unique_id: usize,
    pub ul_version: usize,
    pub driver_version: usize,
    pub poll_sleep_ms: usize,
    pub poll_time_ms: usize,
    pub last_error_message: usize,

    // Counter (addr 0..1)
    pub counter_value: usize,
    pub counter_reset: usize,

    // Analog input (addr 0..7)
    pub analog_in_value: usize,
    pub analog_in_range: usize,
    pub analog_in_type: usize,
    pub analog_in_mode: usize,
    pub analog_in_rate: usize,

    // Voltage input (addr 0..7)
    pub voltage_in_value: usize,
    pub voltage_in_range: usize,

    // Temperature (addr 0..7)
    pub temperature_in_value: usize,
    pub thermocouple_type: usize,
    pub thermocouple_open_detect: usize,
    pub temperature_scale: usize,
    pub temperature_filter: usize,

    // Analog output (addr 0..1)
    pub analog_out_value: usize,
    pub analog_out_range: usize,
    pub analog_out_sync_master: usize,
    pub analog_out_sync_enable: usize,
    pub analog_out_sync_write: usize,

    // Waveform digitizer
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
    pub wave_dig_volt_wf: usize,

    // Waveform generator
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
    pub wave_gen_wave_type: usize,
    pub wave_gen_enable: usize,
    pub wave_gen_amplitude: usize,
    pub wave_gen_offset: usize,
    pub wave_gen_pulse_width: usize,
    pub wave_gen_pulse_delay: usize,
    pub wave_gen_int_wf: usize,
    pub wave_gen_user_wf: usize,

    // Trigger
    pub trigger_mode: usize,

    // Digital I/O
    pub digital_direction: usize,
    pub digital_input: usize,
    pub digital_output: usize,
}

impl MultiFunctionParams {
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

            analog_out_value: base.create_param("ANALOG_OUT_VALUE", ParamType::Int32)?,
            analog_out_range: base.create_param("ANALOG_OUT_RANGE", ParamType::Int32)?,
            analog_out_sync_master: base
                .create_param("ANALOG_OUT_SYNC_MASTER", ParamType::Int32)?,
            analog_out_sync_enable: base
                .create_param("ANALOG_OUT_SYNC_ENABLE", ParamType::Int32)?,
            analog_out_sync_write: base.create_param("ANALOG_OUT_SYNC_WRITE", ParamType::Int32)?,

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
            wave_dig_volt_wf: base.create_param("WAVEDIG_VOLT_WF", ParamType::Float32Array)?,

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
