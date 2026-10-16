use std::sync::{Arc, Mutex};

use epics_rs::asyn::asyn_record::PortClaim;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use meascomp::device::DaqDevice;
use uldaq_sys::UlError;

use crate::params::*;
use crate::poller::{self, PollerState};
use crate::trace::{self, DRIVER_NAME, SelfTrace};
use crate::wave_dig::{self, WaveDigScan, WaveDigState};
use crate::wave_gen::{self, WaveGenScan, WaveGenState};

/// C `MultiFunction::writeInt32`/`writeFloat32Array` resolve the channel
/// through `asynPortDriver::getAddress` (drvMultiFunction.cpp:1947, 2510),
/// which maps the no-device addr -1 to 0 (asynPortDriver.cpp:1901-1913).
fn device_addr(addr: i32) -> i32 {
    if addr == -1 { 0 } else { addr }
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

        // C's ctor defaults (drvMultiFunction.cpp:457-473) and no others:
        // a param C leaves unset must not override its record's db value.
        base.set_float64_param(params.poll_sleep_ms, 0, 50.0)?;
        base.set_int32_param(params.wave_gen_user_num_points, 0, 1)?;
        base.set_int32_param(params.wave_gen_int_num_points, 0, 1)?;
        base.set_int32_param(params.wave_dig_num_points, 0, 1)?;
        base.set_int32_param(params.wave_dig_run, 0, 0)?;
        base.set_int32_param(params.wave_gen_run, 0, 0)?;
        for ch in 0..MAX_ANALOG_OUT as i32 {
            base.set_int32_param(params.analog_out_range, ch, uldaq_sys::BIP10VOLTS)?;
        }
        for ch in 0..MAX_ANALOG_IN {
            base.set_int32_param(params.thermocouple_type, ch as i32, uldaq_sys::TC_J)?;
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
        base.set_string_param(params.driver_version, 0, "4.4")?;

        // Only a DPIOT_IO / DPIOT_BITIO port accepts a direction change;
        // ulDConfigPort and ulDConfigBit reject anything else outright, so ask
        // the device instead of assuming.
        let dio_configurable = matches!(
            device.digital_port_io_type(0),
            Ok(uldaq_sys::DPIOT_IO) | Ok(uldaq_sys::DPIOT_BITIO)
        );
        if dio_configurable {
            let _ = device.digital_config_port(uldaq_sys::AUXPORT, uldaq_sys::DD_INPUT);
        }

        // Seed DIGITAL_INPUT so the bi records have a value to read at init.
        // The poller only pushes on a changed bit, and its one forced first
        // callback happens here -- before dbLoadRecords has run.
        if let Ok(data) = device.digital_in(uldaq_sys::AUXPORT) {
            base.set_uint32_param(params.digital_input, 0, data as u32, 0xFFFF_FFFF, 0)?;
        }

        let state = Arc::new(Mutex::new(PollerState {
            wave_dig: WaveDigState::new(max_input_points),
            wave_gen: WaveGenState::new(max_output_points),
            trace: SelfTrace::default(),
            ai_input_mode: uldaq_sys::AI_SINGLE_ENDED,
        }));

        Ok(Self {
            base,
            params,
            device: Arc::new(Mutex::new(device)),
            state,
            max_input_points,
            max_output_points,
            dio_configurable,
        })
    }
}

impl MultiFunctionDriver {
    /// Apply array (or scalar) updates built by the wave_dig/wave_gen helpers.
    ///
    /// The poller pushes the same list through
    /// `set_params_and_notify_blocking`; on the actor thread that call would
    /// be a self-deadlock, so the store is written directly instead.
    fn apply_updates(&mut self, updates: Vec<ParamSetValue>) -> AsynResult<()> {
        for update in updates {
            if let ParamSetValue::Value {
                reason,
                addr,
                value,
            } = update
            {
                self.base.set_param_value(reason, addr, value)?;
            }
        }
        Ok(())
    }

    /// Common tail of `write_int32`: apply the collected array updates, run
    /// the callbacks and print C's summary line for `status`.
    ///
    /// An early return in the middle of a write must still come through here,
    /// or its array updates would never reach the records.
    fn finish_write(
        &mut self,
        user: &AsynUser,
        value: i32,
        status: UlError,
        wave_arrays: Vec<ParamSetValue>,
    ) -> AsynResult<()> {
        self.apply_updates(wave_arrays)?;
        let addr = device_addr(user.addr);
        self.base.call_param_callbacks(addr)?;
        let port = &self.base.port_name;
        if status == 0 {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "{DRIVER_NAME}:writeInt32, port {port}, wrote {value} to address {addr}"
                ),
            );
        } else {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "{DRIVER_NAME}:writeInt32, port {port}, function={}, ERROR writing {value} to address {addr}, status={status}",
                    user.reason
                ),
            );
        }
        trace::asyn_status(status)
    }

    /// C `defineWaveform(channel)`: set NUM_POINTS/DWELL/FREQ from the
    /// channel's wave type and, for an internal type, rebuild and post its
    /// WAVEGEN_INT_WF. `Ok(false)` is C's -1.
    fn define_waveform(
        &mut self,
        wave_gen_state: &mut WaveGenState,
        channel: i32,
        trace: &SelfTrace,
        updates: &mut Vec<ParamSetValue>,
    ) -> AsynResult<bool> {
        let p = self.params;
        let wave_type = self.int(p.wave_gen_wave_type, channel);
        let user = wave_type == wave_gen::WAVE_TYPE_USER;
        let (points_param, dwell_param) = if user {
            (p.wave_gen_user_num_points, p.wave_gen_user_dwell)
        } else {
            (p.wave_gen_int_num_points, p.wave_gen_int_dwell)
        };
        let num_points = self.int(points_param, 0);
        // C compares as size_t, so a negative count is too large as well.
        if num_points < 0 || num_points as usize > self.max_output_points {
            trace.print(
                TraceMask::ERROR,
                format_args!(
                    "{DRIVER_NAME}:defineWaveform: ERROR numPoints={num_points} must be less than maxOutputPoints={}",
                    self.max_output_points
                ),
            );
            return Ok(false);
        }
        let dwell = self.float(dwell_param, 0);
        let wave = wave_gen::InternalWave {
            wave_type,
            num_points: num_points as usize,
            dwell,
            offset: self.float(p.wave_gen_offset, channel),
            amplitude: self.float(p.wave_gen_amplitude, channel),
            pulse_width: self.float(p.wave_gen_pulse_width, channel),
            pulse_delay: self.float(p.wave_gen_pulse_delay, channel),
        };
        self.base
            .set_int32_param(p.wave_gen_num_points, 0, num_points)?;
        self.base.set_float64_param(p.wave_gen_dwell, 0, dwell)?;
        self.base
            .set_float64_param(p.wave_gen_freq, 0, 1. / dwell / f64::from(num_points))?;
        if user {
            return Ok(true);
        }
        let buffer = &mut wave_gen_state.int_waveforms[channel as usize];
        wave_gen::define_internal(buffer, &wave);
        updates.push(ParamSetValue::new(
            p.wave_gen_int_wf,
            channel,
            ParamValue::Float32Array(buffer[..wave.num_points].into()),
        ));
        Ok(true)
    }

    /// C `startWaveGen` from the current WaveGen parameters; the result is
    /// the uldaq status it reported, or -1 for a refused configuration.
    fn start_wave_gen(
        &mut self,
        dev: &DaqDevice,
        wave_gen_state: &mut WaveGenState,
        trace: &SelfTrace,
        updates: &mut Vec<ParamSetValue>,
    ) -> AsynResult<UlError> {
        let p = self.params;
        // C returns -1 and leaves the 1 writeInt32 set in WAVEGEN_RUN.
        let refuse = |msg: Option<&str>| -> AsynResult<UlError> {
            if let Some(msg) = msg {
                trace.print(
                    TraceMask::ERROR,
                    format_args!("{DRIVER_NAME}:startWaveGen: {msg}"),
                );
            }
            Ok(-1)
        };
        let ext_trigger = self.int(p.wave_gen_ext_trigger, 0) != 0;
        let ext_clock = self.int(p.wave_gen_ext_clock, 0) != 0;
        let continuous = self.int(p.wave_gen_continuous, 0) != 0;
        let retrigger = self.int(p.wave_gen_retrigger, 0) != 0;

        let mut span: Option<(i32, i32)> = None;
        let mut first_type = -1;
        let mut wave_type = -1;
        let mut saved = [0.0f64; MAX_ANALOG_OUT];
        for ch in 0..MAX_ANALOG_OUT as i32 {
            saved[ch as usize] = f64::from(self.int(p.analog_out_value, ch));
            if self.int(p.wave_gen_enable, ch) == 0 {
                continue;
            }
            wave_type = self.int(p.wave_gen_wave_type, ch);
            let first = span.map_or(ch, |(first, _)| first);
            if span.is_none() {
                first_type = wave_type;
            }
            // Internal types take their dwell from the frequency, so the two
            // kinds cannot share a scan.
            if (first_type == wave_gen::WAVE_TYPE_USER) != (wave_type == wave_gen::WAVE_TYPE_USER) {
                return refuse(Some(
                    "ERROR if any enabled waveform type is user-defined then all must be.",
                ));
            }
            span = Some((first, ch));
            if !self.define_waveform(wave_gen_state, ch, trace, updates)? {
                return refuse(None);
            }
        }
        let Some((first_chan, last_chan)) = span else {
            return refuse(Some("ERROR no enabled channels"));
        };

        // defineWaveform has set these.
        let num_points = self.int(p.wave_gen_num_points, 0).max(0) as usize;
        let dwell = self.float(p.wave_gen_dwell, 0);
        let num_chans = (last_chan - first_chan + 1) as usize;
        let mut waveform = vec![0.0f64; num_chans * num_points];
        for i in 0..num_chans {
            let k = first_chan as usize + i;
            let input = if self.int(p.wave_gen_wave_type, k as i32) == wave_gen::WAVE_TYPE_USER {
                &wave_gen_state.user_waveforms[k]
            } else {
                &wave_gen_state.int_waveforms[k]
            };
            // C reads the user scaling at the scan index, not the channel.
            let (amplitude, offset) = if wave_type == wave_gen::WAVE_TYPE_USER {
                (
                    self.float(p.wave_gen_amplitude, i as i32),
                    self.float(p.wave_gen_offset, i as i32),
                )
            } else {
                (1.0, 0.0)
            };
            for j in 0..num_points {
                waveform[j * num_chans + i] = wave_gen::volts_to_dac(input[j], amplitude, offset);
            }
        }

        if let Err(code) = wave_gen::start_wave_gen(
            dev,
            wave_gen_state,
            &WaveGenScan {
                first_chan,
                last_chan,
                num_points,
                freq: 1.0 / (dwell * num_points as f64),
                range: uldaq_sys::BIP10VOLTS,
                ext_trigger,
                ext_clock,
                continuous,
                retrigger,
            },
            &waveform,
            &saved,
            trace,
        ) {
            return Ok(code);
        }
        self.base
            .set_float64_param(p.wave_gen_dwell_actual, 0, wave_gen_state.dwell_actual)?;
        self.base.set_float64_param(
            p.wave_gen_total_time,
            0,
            wave_gen_state.dwell_actual * num_points as f64,
        )?;
        Ok(0)
    }

    /// The writes C answers with `defineWaveform` and, while the generator
    /// runs, `stopWaveGen` + `startWaveGen` (`drvMultiFunction.cpp:2171-2183,
    /// 2292-2305`).
    fn wave_gen_settings(&self) -> [usize; 13] {
        let p = &self.params;
        [
            p.wave_gen_wave_type,
            p.wave_gen_user_num_points,
            p.wave_gen_int_num_points,
            p.wave_gen_enable,
            p.wave_gen_ext_trigger,
            p.wave_gen_ext_clock,
            p.wave_gen_continuous,
            p.wave_gen_user_dwell,
            p.wave_gen_int_dwell,
            p.wave_gen_pulse_width,
            p.wave_gen_pulse_delay,
            p.wave_gen_amplitude,
            p.wave_gen_offset,
        ]
    }

    /// C `computeWaveGenTimes`: both generator time axes, posted when a
    /// point count or dwell is written.
    fn compute_wave_gen_times(&self) -> [ParamSetValue; 2] {
        let p = &self.params;
        let axis = |points, dwell, wf| {
            let times = wave_dig::time_axis(self.int(points, 0), self.float(dwell, 0));
            ParamSetValue::new(wf, 0, ParamValue::Float32Array(times.into()))
        };
        [
            axis(
                p.wave_gen_user_num_points,
                p.wave_gen_user_dwell,
                p.wave_gen_user_time_wf,
            ),
            axis(
                p.wave_gen_int_num_points,
                p.wave_gen_int_dwell,
                p.wave_gen_int_time_wf,
            ),
        ]
    }

    fn wave_gen_restore(&self) -> Vec<(i32, i32)> {
        wave_gen::restore_outputs(&self.params, |r, a| self.int(r, a))
    }

    /// C `getIntegerParam`, whose status every caller ignores: an unset
    /// value reads as the 0 an untouched local would hold here.
    fn int(&self, reason: usize, addr: i32) -> i32 {
        self.base.get_int32_param(reason, addr).unwrap_or(0)
    }

    /// C `getDoubleParam`; see [`Self::int`].
    fn float(&self, reason: usize, addr: i32) -> f64 {
        self.base.get_float64_param(reason, addr).unwrap_or(0.0)
    }

    /// C `getUIntDigitalParam`; see [`Self::int`].
    fn uint(&self, reason: usize, addr: i32) -> u32 {
        self.base.get_uint32_param(reason, addr).unwrap_or(0)
    }

    /// A WaveGen setting write: `defineWaveform(addr)`, then
    /// `stopWaveGen(); startWaveGen();` of a running generator, statuses or-ed.
    fn wave_gen_setting_changed(
        &mut self,
        addr: i32,
        trace: &SelfTrace,
        updates: &mut Vec<ParamSetValue>,
    ) -> AsynResult<UlError> {
        let device = self.device.clone();
        let dev = device.lock().unwrap();
        let state = self.state.clone();
        let mut st = state.lock().unwrap();
        if (0..MAX_ANALOG_OUT as i32).contains(&addr) {
            self.define_waveform(&mut st.wave_gen, addr, trace, updates)?;
        }
        if !st.wave_gen.running {
            return Ok(0);
        }
        let restore = self.wave_gen_restore();
        let stopped = wave_gen::stop_wave_gen(&dev, &mut st.wave_gen, &restore, trace);
        Ok(stopped | self.start_wave_gen(&dev, &mut st.wave_gen, trace, updates)?)
    }

    /// Whether `chan` is configured as a thermocouple input. ulAISetConfig
    /// rejects the TC settings with ERR_BAD_AI_CHAN_TYPE otherwise, so C
    /// writes them only then.
    fn is_thermocouple(&self, chan: i32) -> bool {
        self.int(self.params.analog_in_type, chan) != 0
    }

    /// C `setOpenThermocoupleDetect`.
    fn set_open_tc_detect(&self, dev: &DaqDevice, chan: i32, value: i32) -> UlError {
        let otd = if value != 0 {
            uldaq_sys::OTD_ENABLED
        } else {
            uldaq_sys::OTD_DISABLED
        };
        self.trace().report(
            &dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_OTD_MODE, chan as u32, otd),
            "setOpenThermocoupleDetect",
            "Setting thermocouple open detect mode",
        )
    }

    fn trace(&self) -> SelfTrace {
        self.state.lock().unwrap().trace.clone()
    }
}

impl PortDriver for MultiFunctionDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        const FUNCTION: &str = "writeInt32";
        let mut status: UlError = 0;
        let trace = self.trace();
        let reason = user.reason;
        let addr = device_addr(user.addr);

        // The scan buffers are sized once from maxInputPoints/maxOutputPoints,
        // so a point count above that capacity must never reach the store --
        // wave_dig indexes its fixed per-channel buffers by it.
        let value = if reason == self.params.wave_dig_num_points {
            if value > self.max_input_points as i32 {
                trace.print(
                    TraceMask::ERROR,
                    format_args!(
                        "{DRIVER_NAME}::{FUNCTION} error WaveDigNumPoints={value} must be less than MaxInputPoints={}",
                        self.max_input_points
                    ),
                );
            }
            value.min(self.max_input_points as i32)
        } else if reason == self.params.wave_gen_num_points {
            value.clamp(1, self.max_output_points as i32)
        } else {
            value
        };

        self.base.set_int32_param(reason, addr, value)?;

        // Collected under the state lock, applied after it is dropped: both
        // helpers borrow the driver, and apply_updates needs it mutably.
        let mut wave_arrays: Vec<ParamSetValue> = Vec::new();

        if reason == self.params.wave_dig_num_points {
            let dwell = self.float(self.params.wave_dig_dwell, 0);
            wave_arrays.push(wave_dig::compute_wave_dig_times(&self.params, value, dwell));
        } else if reason == self.params.counter_reset {
            let dev = self.device.lock().unwrap();
            status = trace.report(
                &dev.counter_load(addr, uldaq_sys::CRT_LOAD, 0),
                FUNCTION,
                "Resetting counter",
            );
        } else if reason == self.params.analog_out_value {
            // Only write immediately if sync mode is disabled
            let sync_enable = self.int(self.params.analog_out_sync_enable, 0);
            if sync_enable == 0 {
                if self.state.lock().unwrap().wave_gen.running {
                    trace.report_error(
                        -1,
                        FUNCTION,
                        "cannot write analog outputs while waveform generator is running.",
                    );
                    // C returns before its callbacks and its summary line.
                    return trace::asyn_status(-1);
                }
                let dev = self.device.lock().unwrap();
                let range = self.int(self.params.analog_out_range, addr);
                status = trace.report(
                    &dev.analog_out(addr, range, uldaq_sys::AOUT_FF_NOSCALEDATA, value as f64),
                    FUNCTION,
                    "calling AOut",
                );
            }
        } else if reason == self.params.analog_out_sync_write {
            // Simultaneous write of all analog outputs
            if value != 0 {
                let dev = self.device.lock().unwrap();
                let mut values = vec![0.0f64; MAX_ANALOG_OUT];
                let mut ranges = vec![uldaq_sys::BIP10VOLTS; MAX_ANALOG_OUT];
                for ch in 0..MAX_ANALOG_OUT {
                    values[ch] = self.int(self.params.analog_out_value, ch as i32) as f64;
                    ranges[ch] = self.int(self.params.analog_out_range, ch as i32);
                }
                // C `writeAnalogOutSync` does not print its status.
                status = trace::status_of(&dev.analog_out_array(
                    0,
                    (MAX_ANALOG_OUT - 1) as i32,
                    &ranges,
                    uldaq_sys::AOUTARRAY_FF_NOSCALEDATA,
                    &mut values,
                ));
            }
        } else if reason == self.params.analog_in_mode {
            // C's DIFFERENTIAL is the record's menu value 0 (:225, :1989).
            self.state.lock().unwrap().ai_input_mode = if value == 0 {
                uldaq_sys::AI_DIFFERENTIAL
            } else {
                uldaq_sys::AI_SINGLE_ENDED
            };
        } else if reason == self.params.analog_in_type {
            let dev = self.device.lock().unwrap();
            let chan_type = if value != 0 {
                uldaq_sys::AI_TC
            } else {
                uldaq_sys::AI_VOLTAGE
            };
            status = trace.report(
                &dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TYPE, addr as u32, chan_type),
                FUNCTION,
                "Setting analog input type",
            );
            if value != 0 {
                // The TC type and open-detect settings the records already
                // hold could not be pushed while the channel read volts.
                let tc = self.int(self.params.thermocouple_type, addr);
                status = trace.report(
                    &dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TC_TYPE, addr as u32, tc as i64),
                    FUNCTION,
                    "Set thermocouple type",
                );
                let detect = self.int(self.params.thermocouple_open_detect, addr);
                self.set_open_tc_detect(&dev, addr, detect);
            }
        } else if reason == self.params.thermocouple_type && self.is_thermocouple(addr) {
            let dev = self.device.lock().unwrap();
            status = trace.report(
                &dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TC_TYPE, addr as u32, value as i64),
                FUNCTION,
                "Setting thermocouple type",
            );
        } else if reason == self.params.thermocouple_open_detect && self.is_thermocouple(addr) {
            let dev = self.device.lock().unwrap();
            self.set_open_tc_detect(&dev, addr, value);
        } else if reason == self.params.wave_dig_run {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            // C parity (drvMultiFunction.cpp:2086-2091): start only when idle,
            // stop only when running. The busy record echoes the driver's own
            // value back, so an unguarded start would hit ERR_ALREADY_ACTIVE.
            if value != 0 && !st.wave_dig.running {
                let scan = WaveDigScan::read(
                    &self.params,
                    st.ai_input_mode,
                    |r, a| self.int(r, a),
                    |r, a| self.float(r, a),
                );
                (status, wave_arrays) =
                    wave_dig::start_wave_dig(&dev, &mut st.wave_dig, &self.params, &scan, &trace);
            } else if value == 0 && st.wave_dig.running {
                let restart = WaveDigScan::read_restart(
                    &self.params,
                    st.ai_input_mode,
                    |r, a| self.int(r, a),
                    |r, a| self.float(r, a),
                );
                wave_arrays = wave_dig::stop_wave_dig(
                    &dev,
                    &mut st.wave_dig,
                    &self.params,
                    restart.as_ref(),
                    &trace,
                );
            }
        } else if reason == self.params.wave_dig_read_wf {
            if value != 0 {
                let st = self.state.lock().unwrap();
                wave_arrays = wave_dig::waveform_updates(&self.params, &st.wave_dig, &trace);
            }
        } else if reason == self.params.wave_gen_run {
            let device = self.device.clone();
            let dev = device.lock().unwrap();
            let state = self.state.clone();
            let mut st = state.lock().unwrap();
            // Same start/stop guard as the digitizer above.
            if value != 0 && !st.wave_gen.running {
                status = self.start_wave_gen(&dev, &mut st.wave_gen, &trace, &mut wave_arrays)?;
            } else if value == 0 && st.wave_gen.running {
                let restore = self.wave_gen_restore();
                status = wave_gen::stop_wave_gen(&dev, &mut st.wave_gen, &restore, &trace);
            }
        } else if self.wave_gen_settings().contains(&reason) {
            status = self.wave_gen_setting_changed(addr, &trace, &mut wave_arrays)?;
        }
        if reason == self.params.wave_gen_user_num_points
            || reason == self.params.wave_gen_int_num_points
        {
            wave_arrays.extend(self.compute_wave_gen_times());
        }

        self.finish_write(user, value, status, wave_arrays)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        self.base.set_float64_param(reason, addr, value)?;

        // Frequency change → update dwell
        if reason == self.params.wave_gen_freq && value > 0.0 {
            let num_points = self.int(self.params.wave_gen_num_points, 0) as f64;
            let dwell = 1.0 / (value * num_points);
            self.base
                .set_float64_param(self.params.wave_gen_dwell, 0, dwell)?;
        }

        let mut status: UlError = 0;
        if self.wave_gen_settings().contains(&reason) {
            let mut updates = Vec::new();
            status =
                self.wave_gen_setting_changed(device_addr(addr), &self.trace(), &mut updates)?;
            self.apply_updates(updates)?;
        } else if reason == self.params.wave_dig_dwell {
            let num_points = self.int(self.params.wave_dig_num_points, 0);
            let times = wave_dig::compute_wave_dig_times(&self.params, num_points, value);
            self.apply_updates(vec![times])?;
        }
        if reason == self.params.wave_gen_user_dwell || reason == self.params.wave_gen_int_dwell {
            let times = self.compute_wave_gen_times();
            self.apply_updates(times.into())?;
        }

        self.base.call_param_callbacks(addr)?;
        let port = &self.base.port_name;
        let addr = device_addr(addr);
        if status == 0 {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "{DRIVER_NAME}:writeFloat64, port {port}, wrote {value:.6} to address {addr}"
                ),
            );
        } else {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "{DRIVER_NAME}:writeFloat64, port {port}, ERROR writing {value:.6} to address {addr}, status={status}"
                ),
            );
        }
        trace::asyn_status(status)
    }

    /// Generator time bases: WAVEGEN_USER_TIME_WF and WAVEGEN_INT_TIME_WF,
    /// each `i * dwell` over its own point count. C computeWaveGenTimes.
    /// C `readFloat32Array`: at most WAVEGEN_NUM_POINTS (WAVEDIG_NUM_POINTS
    /// for the digitizer time axis) of the selected buffer.
    fn read_float32_array(&mut self, user: &AsynUser, buf: &mut [f32]) -> AsynResult<usize> {
        const FUNCTION: &str = "readFloat32Array";
        let p = self.params;
        let addr = device_addr(user.addr);
        let error = |msg: std::fmt::Arguments<'_>| {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!("{DRIVER_NAME}:{FUNCTION}: ERROR: {msg}"),
            );
            trace::asyn_status(-1).map(|()| 0)
        };
        if addr >= MAX_ANALOG_OUT as i32 {
            return error(format_args!("addr={addr} max={}", MAX_ANALOG_OUT - 1));
        }
        let reason = user.reason;
        let mut num_points = self.int(p.wave_gen_num_points, 0);
        let time_axis =
            |points, dwell| wave_dig::time_axis(self.int(points, 0), self.float(dwell, 0));
        let st = self.state.lock().unwrap();
        let data: std::borrow::Cow<'_, [f32]> = if reason == p.wave_gen_user_wf {
            st.wave_gen.user_waveforms[addr as usize].as_slice().into()
        } else if reason == p.wave_gen_int_wf {
            st.wave_gen.int_waveforms[addr as usize].as_slice().into()
        } else if reason == p.wave_gen_user_time_wf {
            time_axis(p.wave_gen_user_num_points, p.wave_gen_user_dwell).into()
        } else if reason == p.wave_gen_int_time_wf {
            time_axis(p.wave_gen_int_num_points, p.wave_gen_int_dwell).into()
        } else if reason == p.wave_dig_time_wf {
            num_points = self.int(p.wave_dig_num_points, 0);
            time_axis(p.wave_dig_num_points, p.wave_dig_dwell).into()
        } else {
            return error(format_args!("unknown function={reason}"));
        };
        // C copies past a time axis' last computed point from its zeroed
        // buffer; here the copy stops at the data.
        let n = buf.len().min(num_points.max(0) as usize).min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }

    /// Load a user-defined generator waveform (volts) for one channel.
    fn write_float32_array(&mut self, user: &AsynUser, data: &[f32]) -> AsynResult<()> {
        const FUNCTION: &str = "writeFloat32Array";
        if user.reason != self.params.wave_gen_user_wf {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "{DRIVER_NAME}:{FUNCTION}: ERROR: unknown function={}",
                    user.reason
                ),
            );
            return trace::asyn_status(-1);
        }
        let addr = device_addr(user.addr);
        let mut st = self.state.lock().unwrap();
        let max_points = st.wave_gen.max_points;
        if addr as usize >= MAX_ANALOG_OUT || data.len() > max_points {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "{DRIVER_NAME}:{FUNCTION}: ERROR: addr={addr} max={}, nElements={} max={max_points}",
                    MAX_ANALOG_OUT - 1,
                    data.len()
                ),
            );
            return trace::asyn_status(-1);
        }
        st.wave_gen.user_waveforms[addr as usize][..data.len()].copy_from_slice(data);
        Ok(())
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        const FUNCTION: &str = "writeUInt32Digital";
        let mut status: UlError = 0;
        let trace = self.trace();
        let reason = user.reason;
        let addr = user.addr;
        // C's local `direction`: read only on the output path.
        let mut direction = 0;

        if reason == self.params.digital_output {
            direction = self.uint(self.params.digital_direction, 0);
            let dev = self.device.lock().unwrap();
            let io_mask = (1u32 << NUM_IO_BITS) - 1;
            if mask & direction == io_mask {
                // Every bit is an output and being written: one word write.
                status = trace.report(
                    &dev.digital_out(uldaq_sys::AUXPORT, (value & mask) as u64),
                    FUNCTION,
                    "Calling DOut",
                );
            } else {
                // Otherwise only the masked bits configured as outputs.
                for bit in 0..NUM_IO_BITS {
                    if mask & direction & (1 << bit) != 0 {
                        let bit_val = (value >> bit) & 1;
                        status = trace.report(
                            &dev.digital_bit_out(uldaq_sys::AUXPORT, bit as i32, bit_val != 0),
                            FUNCTION,
                            "Calling DBitOut",
                        );
                    }
                }
            }
        } else if reason == self.params.digital_direction && self.dio_configurable {
            let dev = self.device.lock().unwrap();
            for bit in 0..NUM_IO_BITS {
                if mask & (1 << bit) != 0 {
                    let dir = if (value >> bit) & 1 != 0 {
                        uldaq_sys::DD_OUTPUT
                    } else {
                        uldaq_sys::DD_INPUT
                    };
                    status = trace.report(
                        &dev.digital_config_bit(uldaq_sys::AUXPORT, bit as i32, dir),
                        FUNCTION,
                        "Calling ConfigBit",
                    );
                }
            }
        }

        self.base.set_uint32_param(reason, addr, value, mask, 0)?;

        self.base.call_param_callbacks(addr)?;
        if status == 0 {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "{DRIVER_NAME}:{FUNCTION}, port {}, function={reason}, wrote value=0x{value:x}, mask=0x{mask:x}, direction=0x{direction:x}",
                    self.base.port_name
                ),
            );
        }
        trace::asyn_status(status)
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
/// `claim` is the port name taken before the device is opened, as C's
/// `asynPortDriver` base registers it first.
pub fn create_usb_2408(
    claim: &PortClaim,
    unique_id: &str,
    max_input_points: usize,
    max_output_points: usize,
) -> Result<MultiFunctionRuntime, String> {
    let device = DaqDevice::connect(unique_id)
        .map_err(|e| format!("failed to connect to USB-2408 device: {e}"))?;

    let driver =
        MultiFunctionDriver::new(claim.name(), device, max_input_points, max_output_points)
            .map_err(|e| format!("failed to create MultiFunctionDriver: {e}"))?;

    let params = driver.params;
    let device = driver.device.clone();
    let state = driver.state.clone();

    let (runtime_handle, _actor_jh) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the USB-2408 port runtime: {e}"))?;

    state
        .lock()
        .unwrap()
        .trace
        .bind(runtime_handle.port_handle().clone());
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
