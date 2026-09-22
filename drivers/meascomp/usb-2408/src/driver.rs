use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use meascomp::device::DaqDevice;
use meascomp::digital_io::output_bits;

use crate::params::*;
use crate::poller::{self, PollerState};
use crate::wave_dig::{self, WaveDigScan, WaveDigState};
use crate::wave_gen::{self, WaveGenScan, WaveGenState};

/// C `MultiFunction::writeInt32`/`writeFloat32Array` resolve the channel
/// through `asynPortDriver::getAddress` (drvMultiFunction.cpp:1947, 2510),
/// which maps the no-device addr -1 to 0 (asynPortDriver.cpp:1901-1913).
fn device_addr(addr: i32) -> i32 {
    if addr == -1 { 0 } else { addr }
}

/// What asyn-rs's default `PortDriver::report` prints, which an override
/// cannot call (C `asynPortDriver::report`, asynPortDriver.cpp:3677-3710):
/// the port, its timestamp and parameter library, and at level 3 its
/// interrupt clients. This port has no octet interface, so no EOS lines.
fn write_port_report(base: &PortDriverBase, out: &mut dyn std::fmt::Write, level: i32) {
    let _ = writeln!(out, "Port: {}", base.port_name);
    if level >= 1 {
        let ts = chrono::DateTime::<chrono::Local>::from(base.current_timestamp());
        let _ = writeln!(out, "  Timestamp: {}", ts.format("%Y/%m/%d %H:%M:%S%.3f"));
        base.report_params(out, level);
    }
    if level >= 3 {
        for f in base.interrupts.clients() {
            let iface = f.iface.map_or("any", |i| i.interrupt_label());
            let addr = f.addr.map_or("any".to_string(), |a| a.to_string());
            let reason = f.reason.map_or("any".to_string(), |r| r.to_string());
            let _ = write!(
                out,
                "    {iface} callback client addr={addr}, reason={reason}"
            );
            if let Some(mask) = f.uint32_mask {
                let _ = write!(out, ", mask=0x{mask:x}");
            }
            let _ = writeln!(out);
        }
    }
}

/// USB-2408-2AO port driver.
pub struct MultiFunctionDriver {
    base: PortDriverBase,
    pub params: MultiFunctionParams,
    pub device: Arc<Mutex<DaqDevice>>,
    pub state: Arc<Mutex<PollerState>>,
    pub max_input_points: usize,
    pub max_output_points: usize,
    /// Whether AUXPORT accepts a direction change at all (`DPIOT_IO` /
    /// `DPIOT_BITIO`). The USB-2408 reports `DPIOT_NONCONFIG`.
    dio_configurable: bool,
    /// C `ADCResolution_` / `DACResolution_`, from the device.
    adc_resolution: i64,
    dac_resolution: i64,
}

impl MultiFunctionDriver {
    pub fn new(
        port_name: &str,
        device: DaqDevice,
        max_input_points: usize,
        max_output_points: usize,
    ) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            MAX_SIGNALS,
            PortFlags {
                multi_device: true,
                can_block: false,
                destructible: true,
            },
        );
        let params = MultiFunctionParams::create(&mut base)?;

        // Defaults
        base.set_float64_param(params.poll_sleep_ms, 0, 50.0)?;
        base.set_int32_param(params.wave_dig_num_points, 0, max_input_points as i32)?;
        base.set_int32_param(params.wave_gen_num_points, 0, max_output_points as i32)?;
        base.set_int32_param(params.wave_gen_user_num_points, 0, max_output_points as i32)?;
        base.set_int32_param(params.wave_gen_int_num_points, 0, max_output_points as i32)?;
        base.set_float64_param(params.wave_gen_user_dwell, 0, 0.001)?;
        base.set_float64_param(params.wave_gen_int_dwell, 0, 0.001)?;
        for ch in 0..MAX_ANALOG_OUT {
            base.set_int32_param(params.wave_gen_enable, ch as i32, 1)?;
        }
        base.set_int32_param(params.wave_dig_num_chans, 0, MAX_ANALOG_IN as i32)?;
        base.set_int32_param(params.analog_in_mode, 0, uldaq_sys::AI_DIFFERENTIAL)?;

        for ch in 0..MAX_ANALOG_IN {
            base.set_int32_param(params.analog_in_type, ch as i32, 0)?;
            base.set_int32_param(params.analog_in_range, ch as i32, uldaq_sys::BIP10VOLTS)?;
            base.set_int32_param(params.temperature_scale, ch as i32, uldaq_sys::TS_CELSIUS)?;
            base.set_int32_param(params.thermocouple_type, ch as i32, uldaq_sys::TC_K)?;
        }

        // Device info
        let product_name = device.product_name();
        let product_id = device.product_id();
        let uid = device.unique_id();
        let fw = device.firmware_version().unwrap_or_default();
        let ul_ver = DaqDevice::ul_version().unwrap_or_default();

        base.set_string_param(params.model_name, 0, product_name.clone())?;
        base.set_int32_param(params.model_number, 0, product_id as i32)?;
        base.set_string_param(params.unique_id, 0, uid.clone())?;
        base.set_string_param(params.firmware_version, 0, fw.clone())?;
        base.set_string_param(params.ul_version, 0, ul_ver)?;
        base.set_string_param(params.driver_version, 0, "0.1.0")?;

        // Only a DPIOT_IO / DPIOT_BITIO port accepts a direction change;
        // ulDConfigPort and ulDConfigBit reject anything else outright, so ask
        // the device instead of assuming. The directions themselves are left
        // to the Bd records' PINI, as C's constructor leaves them: forcing
        // the port to input here would float every output bit from here
        // until iocInit.
        let dio_configurable = matches!(
            device.digital_port_io_type(0),
            Ok(uldaq_sys::DPIOT_IO) | Ok(uldaq_sys::DPIOT_BITIO)
        );
        let adc_resolution = device
            .ai_get_info(uldaq_sys::AI_INFO_RESOLUTION, 0)
            .unwrap_or(0);
        let dac_resolution = device
            .ao_get_info(uldaq_sys::AO_INFO_RESOLUTION, 0)
            .unwrap_or(0);

        // Seed DIGITAL_INPUT so the bi records have a value to read at init.
        // The poller only pushes on a changed bit, and its one forced first
        // callback happens here -- before dbLoadRecords has run.
        if let Ok(data) = device.digital_in(uldaq_sys::AUXPORT) {
            base.set_uint32_param(params.digital_input, 0, data as u32, 0xFFFF_FFFF, 0)?;
        }

        let state = Arc::new(Mutex::new(PollerState {
            wave_dig: WaveDigState::new(max_input_points),
            wave_gen: WaveGenState::new(max_output_points),
        }));

        println!(
            "MultiFunctionDriver: port={port_name}, model={product_name}, serial={uid}, fw={fw}"
        );

        Ok(Self {
            base,
            params,
            device: Arc::new(Mutex::new(device)),
            state,
            max_input_points,
            max_output_points,
            dio_configurable,
            adc_resolution,
            dac_resolution,
        })
    }
}

impl MultiFunctionDriver {
    /// Apply array (or scalar) updates built by the wave_dig/wave_gen helpers.
    ///
    /// The poller pushes the same list through
    /// `set_params_and_notify_blocking`; on the actor thread that call would
    /// be a self-deadlock, so the store is written directly instead.
    fn apply_updates(&mut self, updates: Vec<ParamSetValue>) -> Option<String> {
        for update in updates {
            let ParamSetValue::Value {
                reason,
                addr,
                value,
            } = update
            else {
                continue;
            };
            if let Err(e) = self.base.params.set_value(reason, addr, value) {
                return Some(format!("waveform callback error: {e}"));
            }
        }
        None
    }

    /// Common tail of every write: apply the collected array updates, report
    /// the failure (if any) on LAST_ERROR_MESSAGE, run the callbacks, and
    /// return the failure as `asynError`.
    ///
    /// An early return in the middle of a write must still come through here,
    /// or the message it just set would never reach the record.
    fn finish_write(
        &mut self,
        addr: i32,
        last_error: Option<String>,
        mut wave_arrays: Vec<ParamSetValue>,
        time_wf: Option<ParamSetValue>,
    ) -> AsynResult<()> {
        if let Some(update) = time_wf {
            wave_arrays.push(update);
        }
        let last_error = self.apply_updates(wave_arrays).or(last_error);
        if let Some(msg) = &last_error {
            log::error!("{msg}");
            let _ = self.base.params.set_value(
                self.params.last_error_message,
                0,
                ParamValue::Octet(msg.clone().into_bytes()),
            );
        }
        self.base.call_param_callbacks(addr)?;
        // C returns asynError on any failed write, so the record alarms.
        match last_error {
            None => Ok(()),
            Some(message) => Err(AsynError::Status {
                status: AsynStatus::Error,
                message,
            }),
        }
    }

    /// C `computeWaveDigTimes`: the digitizer time base from the requested
    /// WAVEDIG_DWELL over WAVEDIG_NUM_POINTS, as an array callback.
    fn wave_dig_time_update(&self) -> ParamSetValue {
        let num_points = self
            .base
            .get_int32_param(self.params.wave_dig_num_points, 0)
            .unwrap_or(0)
            .max(0) as usize;
        let dwell = self
            .base
            .get_float64_param(self.params.wave_dig_dwell, 0)
            .unwrap_or(0.0);
        let mut st = self.state.lock().unwrap();
        let n = wave_dig::compute_times(&mut st.wave_dig.time_buffer, num_points, dwell);
        ParamSetValue::new(
            self.params.wave_dig_time_wf,
            0,
            ParamValue::Float32Array(st.wave_dig.time_buffer[..n].into()),
        )
    }

    /// C `computeWaveGenTimes`: both generator time bases, each from its own
    /// point count and dwell, as array callbacks.
    fn wave_gen_time_updates(&self) -> Vec<ParamSetValue> {
        let get_points = |reason| self.base.get_int32_param(reason, 0).unwrap_or(0).max(0) as usize;
        let get_dwell = |reason| self.base.get_float64_param(reason, 0).unwrap_or(0.0);
        let user = (
            get_points(self.params.wave_gen_user_num_points),
            get_dwell(self.params.wave_gen_user_dwell),
        );
        let int = (
            get_points(self.params.wave_gen_int_num_points),
            get_dwell(self.params.wave_gen_int_dwell),
        );
        let mut st = self.state.lock().unwrap();
        let n = wave_dig::compute_times(&mut st.wave_gen.user_time_buffer, user.0, user.1);
        let user_wf = ParamSetValue::new(
            self.params.wave_gen_user_time_wf,
            0,
            ParamValue::Float32Array(st.wave_gen.user_time_buffer[..n].into()),
        );
        let n = wave_dig::compute_times(&mut st.wave_gen.int_time_buffer, int.0, int.1);
        let int_wf = ParamSetValue::new(
            self.params.wave_gen_int_time_wf,
            0,
            ParamValue::Float32Array(st.wave_gen.int_time_buffer[..n].into()),
        );
        vec![user_wf, int_wf]
    }

    /// Push THERMOCOUPLE_TYPE and THERMOCOUPLE_OPEN_DETECT for `chan` to the
    /// device.
    ///
    /// ulAISetConfig rejects both with ERR_BAD_AI_CHAN_TYPE unless the channel
    /// is already configured as a thermocouple input, so this is a no-op while
    /// the channel reads volts and every caller must come back through here
    /// once ANALOG_IN_TYPE switches it to TC.
    fn apply_tc_config(&self, dev: &DaqDevice, chan: i32) -> Option<String> {
        if self
            .base
            .get_int32_param(self.params.analog_in_type, chan)
            .unwrap_or(0)
            == 0
        {
            return None;
        }
        if let Ok(tc) = self
            .base
            .get_int32_param(self.params.thermocouple_type, chan)
            && let Err(e) =
                dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TC_TYPE, chan as u32, tc as i64)
        {
            return Some(format!("ai_set_config tc_type error: {e}"));
        }
        if let Ok(detect) = self
            .base
            .get_int32_param(self.params.thermocouple_open_detect, chan)
        {
            let otd = if detect != 0 {
                uldaq_sys::OTD_ENABLED
            } else {
                uldaq_sys::OTD_DISABLED
            };
            if let Err(e) = dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_OTD_MODE, chan as u32, otd) {
                return Some(format!("ai_set_config otd error: {e}"));
            }
        }
        None
    }
}

impl PortDriver for MultiFunctionDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// C `getBounds` (drvMultiFunction.cpp:1920-1937): the raw span of the
    /// DAC and the ADC, so an ao/ai record with LINR=LINEAR converts volts to
    /// counts and back.
    fn get_bounds_int32(&self, user: &AsynUser) -> AsynResult<(i32, i32)> {
        let resolution = if user.reason == self.params.analog_out_value {
            self.dac_resolution
        } else if user.reason == self.params.analog_in_value {
            self.adc_resolution
        } else {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "no bounds for this parameter".into(),
            });
        };
        Ok((0, ((1i64 << resolution) - 1) as i32))
    }

    /// C `MultiFunction::report`: the asynPortDriver report, then the board
    /// and what this port drives on it.
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        write_port_report(&self.base, out, level);
        let board_type = self
            .base
            .get_int32_param(self.params.model_number, 0)
            .unwrap_or(0);
        let board_name = self
            .base
            .get_string_param(self.params.model_name, 0)
            .map(|name| String::from_utf8_lossy(name).into_owned())
            .unwrap_or_default();
        let _ = writeln!(
            out,
            "  Port: {}, board ID={board_type}, board type={board_name}",
            self.base.port_name
        );
        if level >= 1 {
            let _ = writeln!(out, "  analog inputs      = {MAX_ANALOG_IN}");
            let _ = writeln!(out, "  analog input bits  = {}", self.adc_resolution);
            let _ = writeln!(out, "  analog outputs     = {MAX_ANALOG_OUT}");
            let _ = writeln!(out, "  analog output bits = {}", self.dac_resolution);
            let _ = writeln!(out, "  temperature inputs = {MAX_ANALOG_IN}");
            let _ = writeln!(out, "  digital I/O ports  = 1");
            let _ = writeln!(out, "  digital I/O port     0");
            let _ = writeln!(out, "    I/O port              = {}", uldaq_sys::AUXPORT);
            let _ = writeln!(out, "    I/O bits              = {NUM_IO_BITS}");
            let configurable = u8::from(self.dio_configurable);
            let _ = writeln!(out, "    I/O bit configurable  = {configurable}");
            let _ = writeln!(out, "    I/O port configurable = {configurable}");
            let _ = writeln!(out, "    I/O port mask         = 0x{PORT_MASK:x}");
            let _ = writeln!(out, "  timers             = 0");
            let _ = write!(out, "  # counters         = {MAX_COUNTERS}");
            let _ = write!(out, "  first counter      = 0");
            let _ = write!(out, "  counterCounts = ");
            for i in 0..MAX_COUNTERS as i32 {
                let counts = self
                    .base
                    .get_int32_param(self.params.counter_value, i)
                    .unwrap_or(0);
                let _ = write!(out, " {counts}");
            }
            let _ = writeln!(out);
        }
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let mut last_error: Option<String> = None;
        let reason = user.reason;
        let addr = device_addr(user.addr);

        // The scan buffers are sized once from maxInputPoints/maxOutputPoints,
        // so a point count above that capacity must never reach the store --
        // wave_dig indexes its fixed per-channel buffers by it. The records
        // carry the same bound in DRVL/DRVH; this gate is what makes the
        // invariant hold no matter what the database says.
        let value = if reason == self.params.wave_dig_num_points {
            value.clamp(1, self.max_input_points as i32)
        } else if reason == self.params.wave_gen_num_points {
            value.clamp(1, self.max_output_points as i32)
        } else {
            value
        };

        self.base.params.set_int32(reason, addr, value)?;

        // Collected under the state lock, applied after it is dropped: both
        // helpers borrow the driver, and apply_updates needs it mutably.
        let mut wave_arrays: Vec<ParamSetValue> = Vec::new();
        let mut time_wf: Option<ParamSetValue> = None;

        // Command params act on any written value, as C's do.
        if reason == self.params.counter_reset {
            let dev = self.device.lock().unwrap();
            if let Err(e) = dev.counter_clear(addr) {
                last_error = Some(format!("counter_clear error: {e}"));
            }
        } else if reason == self.params.analog_out_value {
            // Only write immediately if sync mode is disabled
            let sync_enable = self
                .base
                .get_int32_param(self.params.analog_out_sync_enable, 0)
                .unwrap_or(0);
            if sync_enable == 0 {
                let dev = self.device.lock().unwrap();
                let range = self
                    .base
                    .get_int32_param(self.params.analog_out_range, addr)?;
                if let Err(e) =
                    dev.analog_out(addr, range, uldaq_sys::AOUT_FF_NOSCALEDATA, value as f64)
                {
                    last_error = Some(format!("analog_out error: {e}"));
                }
            }
        } else if reason == self.params.analog_out_sync_write {
            // Simultaneous write of all analog outputs
            let dev = self.device.lock().unwrap();
            let mut values = vec![0.0f64; MAX_ANALOG_OUT];
            let mut ranges = vec![uldaq_sys::BIP10VOLTS; MAX_ANALOG_OUT];
            for ch in 0..MAX_ANALOG_OUT {
                values[ch] = self
                    .base
                    .get_int32_param(self.params.analog_out_value, ch as i32)?
                    as f64;
                ranges[ch] = self
                    .base
                    .get_int32_param(self.params.analog_out_range, ch as i32)?;
            }
            if let Err(e) = dev.analog_out_array(
                0,
                (MAX_ANALOG_OUT - 1) as i32,
                &ranges,
                uldaq_sys::AOUTARRAY_FF_NOSCALEDATA,
                &mut values,
            ) {
                last_error = Some(format!("analog_out_array error: {e}"));
            }
        } else if reason == self.params.analog_in_type {
            let dev = self.device.lock().unwrap();
            let chan_type = if value != 0 {
                uldaq_sys::AI_TC
            } else {
                uldaq_sys::AI_VOLTAGE
            };
            if let Err(e) = dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TYPE, addr as u32, chan_type) {
                last_error = Some(format!("ai_set_config chan_type error: {e}"));
            } else if value != 0 {
                // The channel has just become a thermocouple input; the TC type
                // and open-detect settings the records already hold could not be
                // pushed while it was a voltage channel, so push them now.
                last_error = self.apply_tc_config(&dev, addr);
            }
        } else if reason == self.params.thermocouple_type
            || reason == self.params.thermocouple_open_detect
        {
            let dev = self.device.lock().unwrap();
            last_error = self.apply_tc_config(&dev, addr);
        } else if reason == self.params.wave_dig_run {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            // C parity (drvMultiFunction.cpp:2086-2091): start only when idle,
            // stop only when running. The busy record echoes the driver's own
            // value back, so an unguarded start would hit ERR_ALREADY_ACTIVE.
            if value != 0 && !st.wave_dig.running {
                let first_chan = self
                    .base
                    .get_int32_param(self.params.wave_dig_first_chan, 0)?
                    as usize;
                let num_chans = self
                    .base
                    .get_int32_param(self.params.wave_dig_num_chans, 0)?
                    as usize;
                let num_points = self
                    .base
                    .get_int32_param(self.params.wave_dig_num_points, 0)?
                    as usize;
                let dwell = self.base.get_float64_param(self.params.wave_dig_dwell, 0)?;
                let input_mode = self.base.get_int32_param(self.params.analog_in_mode, 0)?;
                let range = self
                    .base
                    .get_int32_param(self.params.analog_in_range, first_chan as i32)?;
                let ext_trig = self
                    .base
                    .get_int32_param(self.params.wave_dig_ext_trigger, 0)?
                    != 0;
                let ext_clk = self
                    .base
                    .get_int32_param(self.params.wave_dig_ext_clock, 0)?
                    != 0;
                let cont = self
                    .base
                    .get_int32_param(self.params.wave_dig_continuous, 0)?
                    != 0;
                let retrig = self
                    .base
                    .get_int32_param(self.params.wave_dig_retrigger, 0)?
                    != 0;
                let burst = self
                    .base
                    .get_int32_param(self.params.wave_dig_burst_mode, 0)?
                    != 0;
                st.wave_dig.auto_restart = self
                    .base
                    .get_int32_param(self.params.wave_dig_auto_restart, 0)?
                    != 0;

                if let Err(e) = wave_dig::start_wave_dig(
                    &dev,
                    &mut st.wave_dig,
                    &WaveDigScan {
                        first_chan,
                        num_chans,
                        num_points,
                        dwell,
                        input_mode,
                        range,
                        ext_trigger: ext_trig,
                        ext_clock: ext_clk,
                        continuous: cont,
                        retrigger: retrig,
                        burst_mode: burst,
                    },
                ) {
                    last_error = Some(format!("start_wave_dig error: {e}"));
                    self.base.params.set_int32(reason, addr, 0)?;
                } else {
                    self.base.params.set_float64(
                        self.params.wave_dig_dwell_actual,
                        0,
                        st.wave_dig.dwell_actual,
                    )?;
                    self.base.params.set_float64(
                        self.params.wave_dig_total_time,
                        0,
                        st.wave_dig.dwell_actual * num_points as f64,
                    )?;
                }
            } else if value == 0 {
                wave_dig::stop_wave_dig(&dev, &mut st.wave_dig);
            }
        } else if reason == self.params.wave_dig_read_wf {
            let st = self.state.lock().unwrap();
            wave_arrays = wave_dig::waveform_updates(&self.params, &st.wave_dig);
        } else if reason == self.params.wave_gen_run {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            // Same start/stop guard as the digitizer above.
            if value != 0 && !st.wave_gen.running {
                // Which channels take part, and whether they are user-defined
                // or internally generated. C startWaveGen rejects a mixture
                // because the two kinds take their dwell from different
                // parameters.
                let mut span: Option<(i32, i32)> = None;
                let mut user_mode = false;
                let mut mixed = false;
                for ch in 0..MAX_ANALOG_OUT as i32 {
                    if self.base.get_int32_param(self.params.wave_gen_enable, ch)? == 0 {
                        continue;
                    }
                    let is_user = self
                        .base
                        .get_int32_param(self.params.wave_gen_wave_type, ch)?
                        == wave_gen::WAVE_TYPE_USER;
                    match span {
                        None => {
                            span = Some((ch, ch));
                            user_mode = is_user;
                        }
                        Some((first, _)) => {
                            span = Some((first, ch));
                            mixed |= is_user != user_mode;
                        }
                    }
                }
                let usable = match span {
                    None => {
                        last_error = Some("no waveform generator channel is enabled".to_string());
                        None
                    }
                    Some(_) if mixed => {
                        last_error = Some(
                            "user-defined and internal waveforms cannot be mixed across channels"
                                .to_string(),
                        );
                        None
                    }
                    Some(range) => Some(range),
                };
                if usable.is_none() {
                    // The scan was refused, so the record must not be left
                    // reading "Run" -- it follows this parameter back to Stop.
                    self.base.params.set_int32(reason, addr, 0)?;
                }
                if let Some((first_chan, last_chan)) = usable {
                    // The point count and dwell come from whichever pair the
                    // selected wave type uses; NUM_POINTS/DWELL/FREQ are the
                    // readbacks of that choice.
                    let (points_param, dwell_param) = if user_mode {
                        (
                            self.params.wave_gen_user_num_points,
                            self.params.wave_gen_user_dwell,
                        )
                    } else {
                        (
                            self.params.wave_gen_int_num_points,
                            self.params.wave_gen_int_dwell,
                        )
                    };
                    let num_points = (self.base.get_int32_param(points_param, 0)? as usize)
                        .clamp(1, self.max_output_points);
                    let dwell = self
                        .base
                        .get_float64_param(dwell_param, 0)?
                        .max(f64::MIN_POSITIVE);
                    let freq = 1.0 / (dwell * num_points as f64);
                    self.base.params.set_int32(
                        self.params.wave_gen_num_points,
                        0,
                        num_points as i32,
                    )?;
                    self.base
                        .params
                        .set_float64(self.params.wave_gen_dwell, 0, dwell)?;
                    self.base
                        .params
                        .set_float64(self.params.wave_gen_freq, 0, freq)?;

                    let ext_trig = self
                        .base
                        .get_int32_param(self.params.wave_gen_ext_trigger, 0)?
                        != 0;
                    let ext_clk = self
                        .base
                        .get_int32_param(self.params.wave_gen_ext_clock, 0)?
                        != 0;
                    let cont = self
                        .base
                        .get_int32_param(self.params.wave_gen_continuous, 0)?
                        != 0;
                    let retrig = self
                        .base
                        .get_int32_param(self.params.wave_gen_retrigger, 0)?
                        != 0;

                    // Save current AO values for restore on stop
                    let mut saved = vec![0.0f64; MAX_ANALOG_OUT];
                    for ch in 0..MAX_ANALOG_OUT as i32 {
                        saved[ch as usize] = self
                            .base
                            .get_int32_param(self.params.analog_out_value, ch)?
                            as f64;
                    }

                    // Build per-channel waveforms, then interleave for ulAOutScan
                    let mut per_chan = Vec::with_capacity(MAX_ANALOG_OUT);
                    for ch in first_chan..=last_chan {
                        let wave_type = self
                            .base
                            .get_int32_param(self.params.wave_gen_wave_type, ch)?;
                        let amp = self
                            .base
                            .get_float64_param(self.params.wave_gen_amplitude, ch)?;
                        let offset = self
                            .base
                            .get_float64_param(self.params.wave_gen_offset, ch)?;
                        let pw = self
                            .base
                            .get_float64_param(self.params.wave_gen_pulse_width, ch)?;
                        if wave_type == wave_gen::WAVE_TYPE_USER {
                            // A user waveform shorter than the scan is repeated;
                            // an absent one leaves the channel at zero volts.
                            let user = &st.wave_gen.user_waveforms[ch as usize];
                            per_chan.push(if user.is_empty() {
                                vec![0.0; num_points]
                            } else {
                                (0..num_points).map(|i| user[i % user.len()]).collect()
                            });
                        } else {
                            per_chan.push(wave_gen::generate_waveform(
                                wave_type, num_points, amp, offset, pw,
                            ));
                        }
                    }
                    // Interleave: [ch0_pt0, ch1_pt0, ch0_pt1, ch1_pt1, ...]
                    let mut waveform = Vec::with_capacity(per_chan.len() * num_points);
                    for pt in 0..num_points {
                        waveform.extend(per_chan.iter().map(|chan| chan[pt]));
                    }
                    // Convert voltage to raw 16-bit DAC units (NOSCALEDATA mode)
                    wave_gen::volts_to_dac(&mut waveform);

                    if let Err(e) = wave_gen::start_wave_gen(
                        &dev,
                        &mut st.wave_gen,
                        &WaveGenScan {
                            first_chan,
                            last_chan,
                            num_points,
                            freq,
                            range: uldaq_sys::BIP10VOLTS,
                            ext_trigger: ext_trig,
                            ext_clock: ext_clk,
                            continuous: cont,
                            retrigger: retrig,
                        },
                        &waveform,
                        &saved,
                    ) {
                        last_error = Some(format!("start_wave_gen error: {e}"));
                        self.base.params.set_int32(reason, addr, 0)?;
                    } else {
                        self.base.params.set_float64(
                            self.params.wave_gen_dwell_actual,
                            0,
                            st.wave_gen.dwell_actual,
                        )?;
                        self.base.params.set_float64(
                            self.params.wave_gen_total_time,
                            0,
                            st.wave_gen.dwell_actual * num_points as f64,
                        )?;
                    }
                }
            } else if value == 0 {
                wave_gen::stop_wave_gen(&dev, &mut st.wave_gen);
            }
        }

        // C recomputes the time bases on a point-count write
        // (drvMultiFunction.cpp:2104, :2195-2198), not only when a scan starts.
        if reason == self.params.wave_dig_num_points {
            time_wf = Some(self.wave_dig_time_update());
        }
        if reason == self.params.wave_gen_user_num_points
            || reason == self.params.wave_gen_int_num_points
        {
            wave_arrays.extend(self.wave_gen_time_updates());
        }

        self.finish_write(addr, last_error, wave_arrays, time_wf)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        self.base.params.set_float64(reason, addr, value)?;

        // Frequency change → update dwell
        if reason == self.params.wave_gen_freq && value > 0.0 {
            let num_points =
                self.base
                    .get_int32_param(self.params.wave_gen_num_points, 0)? as f64;
            let dwell = 1.0 / (value * num_points);
            self.base
                .params
                .set_float64(self.params.wave_gen_dwell, 0, dwell)?;
        }

        // C recomputes the time bases on a dwell write (drvMultiFunction.cpp:
        // 2308-2316).
        let mut wave_arrays = Vec::new();
        let mut time_wf = None;
        if reason == self.params.wave_dig_dwell {
            time_wf = Some(self.wave_dig_time_update());
        }
        if reason == self.params.wave_gen_user_dwell || reason == self.params.wave_gen_int_dwell {
            wave_arrays.extend(self.wave_gen_time_updates());
        }

        self.finish_write(addr, None, wave_arrays, time_wf)
    }

    /// C `readFloat32Array`: the generator arrays run to WAVEGEN_NUM_POINTS,
    /// the digitizer time base to WAVEDIG_NUM_POINTS.
    fn read_float32_array(&mut self, user: &AsynUser, buf: &mut [f32]) -> AsynResult<usize> {
        let reason = user.reason;
        let points_param = if reason == self.params.wave_dig_time_wf {
            self.params.wave_dig_num_points
        } else {
            self.params.wave_gen_num_points
        };
        let num_points = self
            .base
            .get_int32_param(points_param, 0)
            .unwrap_or(0)
            .max(0) as usize;
        let st = self.state.lock().unwrap();
        let user_wf: Vec<f32>;
        let src: &[f32] = if reason == self.params.wave_gen_user_time_wf {
            &st.wave_gen.user_time_buffer
        } else if reason == self.params.wave_gen_int_time_wf {
            &st.wave_gen.int_time_buffer
        } else if reason == self.params.wave_dig_time_wf {
            &st.wave_dig.time_buffer
        } else if reason == self.params.wave_gen_user_wf {
            let ch = device_addr(user.addr) as usize;
            let Some(wf) = st.wave_gen.user_waveforms.get(ch) else {
                return Ok(0);
            };
            user_wf = wf.iter().map(|v| *v as f32).collect();
            &user_wf
        } else {
            return Ok(0);
        };
        let n = buf.len().min(num_points).min(src.len());
        buf[..n].copy_from_slice(&src[..n]);
        Ok(n)
    }

    /// C `readFloat64Array`: a digitized channel or the absolute time base,
    /// up to WAVEDIG_NUM_POINTS.
    fn read_float64_array(&mut self, user: &AsynUser, buf: &mut [f64]) -> AsynResult<usize> {
        let num_points = self
            .base
            .get_int32_param(self.params.wave_dig_num_points, 0)
            .unwrap_or(0)
            .max(0) as usize;
        let st = self.state.lock().unwrap();
        let src: &[f64] = if user.reason == self.params.wave_dig_volt_wf {
            let Some(chan) = st
                .wave_dig
                .channel_buffers
                .get(device_addr(user.addr) as usize)
            else {
                return Ok(0);
            };
            chan
        } else if user.reason == self.params.wave_dig_abs_time_wf {
            &st.wave_dig.abs_time_buffer
        } else {
            return Ok(0);
        };
        let n = buf.len().min(num_points).min(src.len());
        buf[..n].copy_from_slice(&src[..n]);
        Ok(n)
    }

    /// Load a user-defined generator waveform (volts) for one channel.
    fn write_float32_array(&mut self, user: &AsynUser, data: &[f32]) -> AsynResult<()> {
        if user.reason != self.params.wave_gen_user_wf {
            return Ok(());
        }
        let ch = device_addr(user.addr) as usize;
        if ch >= MAX_ANALOG_OUT {
            return Ok(());
        }
        let mut st = self.state.lock().unwrap();
        let n = data.len().min(st.wave_gen.max_points);
        st.wave_gen.user_waveforms[ch] = data[..n].iter().map(|v| *v as f64).collect();
        Ok(())
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        let mut last_error: Option<String> = None;
        let reason = user.reason;
        let addr = user.addr;

        if reason == self.params.digital_output {
            let direction = self
                .base
                .get_uint32_param(self.params.digital_direction, 0)?;
            let dev = self.device.lock().unwrap();
            if mask & direction == PORT_MASK {
                // Every bit is an output and every bit is written: one word
                // write, as C does.
                if let Err(e) = dev.digital_out(uldaq_sys::AUXPORT, (value & mask) as u64) {
                    last_error = Some(format!("digital_out error: {e}"));
                }
            } else {
                for (bit, level) in output_bits(value, mask, direction, NUM_IO_BITS) {
                    if let Err(e) = dev.digital_bit_out(uldaq_sys::AUXPORT, bit, level) {
                        last_error = Some(format!("digital_bit_out error: {e}"));
                    }
                }
            }
        } else if reason == self.params.digital_direction {
            if !self.dio_configurable {
                // The USB-2408 AUXPORT is open-collector with no direction
                // control. C "Cannot program direction. Set open collector
                // output to 0" (drvMultiFunction.cpp:2369-2376): each masked
                // bit is released, and the stored mask is what the
                // DIGITAL_OUTPUT gate then honours.
                let dev = self.device.lock().unwrap();
                for bit in (0..NUM_IO_BITS).filter(|bit| mask & (1 << bit) != 0) {
                    if let Err(e) = dev.digital_bit_out(uldaq_sys::AUXPORT, bit as i32, false) {
                        last_error = Some(format!("digital_bit_out error: {e}"));
                    }
                }
            } else {
                let dev = self.device.lock().unwrap();
                for bit in 0..NUM_IO_BITS {
                    if mask & (1 << bit) != 0 {
                        let dir = if (value >> bit) & 1 != 0 {
                            uldaq_sys::DD_OUTPUT
                        } else {
                            uldaq_sys::DD_INPUT
                        };
                        if let Err(e) = dev.digital_config_bit(uldaq_sys::AUXPORT, bit as i32, dir)
                        {
                            last_error = Some(format!("digital_config_bit error: {e}"));
                        }
                    }
                }
            }
        }

        self.base.params.set_uint32(reason, addr, value, mask, 0)?;

        self.finish_write(addr, last_error, Vec::new(), None)
    }
}

/// Runtime wrapper.
pub struct MultiFunctionRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: MultiFunctionParams,
    pub device: Arc<Mutex<DaqDevice>>,
    _poller_handle: std::thread::JoinHandle<()>,
}

impl MultiFunctionRuntime {
    pub fn port_handle(&self) -> &epics_rs::asyn::port_handle::PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// Create a USB-2408-2AO driver, start the port runtime actor and polling thread.
pub fn create_usb_2408(
    port_name: &str,
    unique_id: &str,
    max_input_points: usize,
    max_output_points: usize,
) -> Result<MultiFunctionRuntime, String> {
    let device = DaqDevice::connect(unique_id)
        .map_err(|e| format!("failed to connect to USB-2408 device: {e}"))?;

    let driver = MultiFunctionDriver::new(port_name, device, max_input_points, max_output_points)
        .map_err(|e| format!("failed to create MultiFunctionDriver: {e}"))?;

    let params = driver.params;
    let device = driver.device.clone();
    let state = driver.state.clone();

    let (runtime_handle, _actor_jh) = create_port_runtime(driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the USB-2408 port runtime: {e}"))?;

    let poller_handle = poller::start_poller(
        runtime_handle.port_handle().clone(),
        params,
        device.clone(),
        state,
    );

    Ok(MultiFunctionRuntime {
        runtime_handle,
        params,
        device,
        _poller_handle: poller_handle,
    })
}
