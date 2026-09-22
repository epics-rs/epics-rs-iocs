use std::fmt;
use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use meascomp::device::DaqDevice;
use meascomp::digital_io::output_bits;

use crate::params::*;
use crate::poller::{self, PollerState};
use crate::trace::DRIVER;
use crate::wave_dig::{self, WaveDigScan, WaveDigState};
use crate::wave_gen::{self, WaveGenScan, WaveGenState};

/// C `MultiFunction::writeInt32`/`writeFloat32Array` resolve the channel
/// through `asynPortDriver::getAddress` (drvMultiFunction.cpp:1947, 2510),
/// which maps the no-device addr -1 to 0 (asynPortDriver.cpp:1901-1913).
/// Whether C `mapTriggerType` has a uldaq trigger for this Measurement
/// Computing trigger number (drvMultiFunction.cpp:1384-1403, :1405-1435):
/// all of 0..=19 but the two hysteresis gates.
fn is_supported_trigger_mode(mode: i32) -> bool {
    (0..=19).contains(&mode) && mode != 2 && mode != 3
}

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
        // ASYN_CANBLOCK, as C declares it (drvMultiFunction.cpp:821): every
        // write does USB I/O behind the device mutex the poller holds for a
        // whole sweep, so records must complete asynchronously instead of
        // blocking the thread that processes them.
        let mut base = PortDriverBase::new(
            port_name,
            MAX_SIGNALS,
            PortFlags {
                multi_device: true,
                can_block: true,
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
        // One channel, as the NumChans menu's first state.
        base.set_int32_param(params.wave_dig_num_chans, 0, 1)?;
        base.set_int32_param(params.analog_in_mode, 0, uldaq_sys::AI_DIFFERENTIAL)?;

        for ch in 0..MAX_ANALOG_IN {
            base.set_int32_param(params.analog_in_type, ch as i32, 0)?;
            base.set_int32_param(params.analog_in_range, ch as i32, uldaq_sys::BIP10VOLTS)?;
            base.set_int32_param(params.temperature_scale, ch as i32, uldaq_sys::TS_CELSIUS)?;
            // Type J, as C's constructor and libuldaq default to.
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
    /// return the failure as `asynError` -- or, on success, print
    /// `function`'s TRACEIO_DRIVER line with what it `wrote`, as C does.
    ///
    /// An early return in the middle of a write must still come through here,
    /// or the message it just set would never reach the record.
    fn finish_write(
        &mut self,
        user: &AsynUser,
        addr: i32,
        last_error: Option<String>,
        wave_arrays: Vec<ParamSetValue>,
        function: &str,
        wrote: fmt::Arguments<'_>,
    ) -> AsynResult<()> {
        // C delivers each channel's arrays with that channel's callbacks, so
        // every address an update names gets its callbacks now, not at the
        // poller's next sweep.
        let mut touched: Vec<i32> = wave_arrays
            .iter()
            .filter_map(|u| match u {
                ParamSetValue::Value { addr: a, .. } if *a != addr => Some(*a),
                _ => None,
            })
            .collect();
        touched.sort_unstable();
        touched.dedup();
        let last_error = self.apply_updates(wave_arrays).or(last_error);
        for a in touched {
            self.base.call_param_callbacks(a)?;
        }
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
            None => {
                user.print(
                    TraceMask::IO_DRIVER,
                    file!(),
                    line!(),
                    format_args!("{DRIVER}:{function}, port {}, {wrote}", self.base.port_name),
                );
                Ok(())
            }
            Some(message) => Err(AsynError::Status {
                status: AsynStatus::Error,
                message,
            }),
        }
    }

    /// C `reportError(0, functionName, message)` (drvMultiFunction.cpp:1298-
    /// 1305): the TRACEIO_DRIVER line C prints after each libuldaq call that
    /// succeeded.
    fn info(&self, function: &str, message: &str) {
        self.base.trace_print(
            TraceMask::IO_DRIVER,
            &format!("{DRIVER}::{function} Info: {message}"),
        );
    }

    /// C `readWaveDig`'s FLOW line for each channel it hands out
    /// (drvMultiFunction.cpp:1895-1898).
    fn trace_wave_dig_callbacks(&self, dig: &WaveDigState) {
        for ch in dig.first_chan..(dig.first_chan + dig.num_chans).min(MAX_ANALOG_IN) {
            let first = dig.channel_buffers[ch].first().copied().unwrap_or(0.0);
            self.base.trace_print(
                TraceMask::FLOW,
                &format!(
                    "{DRIVER}:readWaveDig:, doing callbacks on input {ch}, first value={first:.6}"
                ),
            );
        }
    }

    /// C `defineWaveform`: publish the point count, dwell and frequency the
    /// channel's wave type uses and, for an internal waveform, compute it into
    /// the channel's buffer and hand it out on WAVEGEN_INT_WF. A point count
    /// beyond the buffers is refused, as C refuses it.
    fn define_waveform(
        &mut self,
        generator: &mut WaveGenState,
        channel: i32,
        updates: &mut Vec<ParamSetValue>,
    ) -> Result<(), String> {
        let get_i32 = |base: &PortDriverBase, reason, addr| {
            base.get_int32_param(reason, addr)
                .map_err(|e| e.to_string())
        };
        let get_f64 = |base: &PortDriverBase, reason, addr| {
            base.get_float64_param(reason, addr)
                .map_err(|e| e.to_string())
        };
        let wave_type = get_i32(&self.base, self.params.wave_gen_wave_type, channel)?;
        let user = wave_type == wave_gen::WAVE_TYPE_USER;
        let (points_param, dwell_param) = if user {
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
        let num_points = get_i32(&self.base, points_param, 0)?.max(0) as usize;
        if num_points > self.max_output_points {
            return Err(format!(
                "numPoints={num_points} must be less than maxOutputPoints={}",
                self.max_output_points
            ));
        }
        let dwell = get_f64(&self.base, dwell_param, 0)?;
        self.base
            .params
            .set_int32(self.params.wave_gen_num_points, 0, num_points as i32)
            .map_err(|e| e.to_string())?;
        self.base
            .params
            .set_float64(self.params.wave_gen_dwell, 0, dwell)
            .map_err(|e| e.to_string())?;
        self.base
            .params
            .set_float64(
                self.params.wave_gen_freq,
                0,
                1.0 / dwell / num_points as f64,
            )
            .map_err(|e| e.to_string())?;
        if user {
            return Ok(());
        }
        let shape = wave_gen::WaveShape {
            wave_type,
            amplitude: get_f64(&self.base, self.params.wave_gen_amplitude, channel)?,
            offset: get_f64(&self.base, self.params.wave_gen_offset, channel)?,
            pulse_width: get_f64(&self.base, self.params.wave_gen_pulse_width, channel)?,
            pulse_delay: get_f64(&self.base, self.params.wave_gen_pulse_delay, channel)?,
            dwell,
        };
        let data = wave_gen::generate_waveform(&shape, num_points);
        if let Some(buffer) = generator.int_buffers.get_mut(channel as usize) {
            buffer[..num_points].copy_from_slice(&data);
        }
        updates.push(ParamSetValue::new(
            self.params.wave_gen_int_wf,
            channel,
            ParamValue::Float32Array(data.into()),
        ));
        Ok(())
    }

    /// C `startWaveGen`: define every enabled channel, then play the span of
    /// them from their buffers -- a user channel's stored volts scaled by its
    /// Amplitude and Offset, an internal one's computed waveform.
    fn start_generator(
        &mut self,
        dev: &DaqDevice,
        generator: &mut WaveGenState,
        updates: &mut Vec<ParamSetValue>,
    ) -> Result<(), String> {
        let get_i32 = |base: &PortDriverBase, reason, addr| {
            base.get_int32_param(reason, addr)
                .map_err(|e| e.to_string())
        };
        let flag = |base: &PortDriverBase, reason| get_i32(base, reason, 0).map(|v| v != 0);
        let ext_trigger = flag(&self.base, self.params.wave_gen_ext_trigger)?;
        let ext_clock = flag(&self.base, self.params.wave_gen_ext_clock)?;
        let continuous = flag(&self.base, self.params.wave_gen_continuous)?;
        let retrigger = flag(&self.base, self.params.wave_gen_retrigger)?;

        let mut span: Option<(i32, i32)> = None;
        let mut first_is_user = false;
        let mut saved = [None; MAX_ANALOG_OUT];
        for ch in 0..MAX_ANALOG_OUT as i32 {
            if get_i32(&self.base, self.params.wave_gen_enable, ch)? == 0 {
                continue;
            }
            let is_user = get_i32(&self.base, self.params.wave_gen_wave_type, ch)?
                == wave_gen::WAVE_TYPE_USER;
            match span {
                None => {
                    span = Some((ch, ch));
                    first_is_user = is_user;
                }
                Some((first, _)) => span = Some((first, ch)),
            }
            // Saved to be put back when the scan ends, C waveGenSavedOutput.
            saved[ch as usize] = Some(f64::from(get_i32(
                &self.base,
                self.params.analog_out_value,
                ch,
            )?));
            // User-defined and internal waveforms take their dwell from
            // different parameters, so they cannot share a scan.
            if is_user != first_is_user {
                return Err(
                    "if any enabled waveform type is user-defined then all must be".to_string(),
                );
            }
            self.define_waveform(generator, ch, updates)?;
        }
        let Some((first_chan, last_chan)) = span else {
            return Err("no enabled channels".to_string());
        };

        // defineWaveform above left the point count and dwell in use.
        let num_points = (get_i32(&self.base, self.params.wave_gen_num_points, 0)?.max(0) as usize)
            .min(self.max_output_points);
        let dwell = self
            .base
            .get_float64_param(self.params.wave_gen_dwell, 0)
            .map_err(|e| e.to_string())?;

        // Interleave [ch0_pt0, ch1_pt0, ch0_pt1, ...] in volts, then convert
        // to DAC counts (NOSCALEDATA mode).
        let mut per_chan: Vec<Vec<f64>> = Vec::with_capacity(MAX_ANALOG_OUT);
        for ch in first_chan..=last_chan {
            let wave_type = get_i32(&self.base, self.params.wave_gen_wave_type, ch)?;
            let samples = if wave_type == wave_gen::WAVE_TYPE_USER {
                let amplitude = self
                    .base
                    .get_float64_param(self.params.wave_gen_amplitude, ch)
                    .map_err(|e| e.to_string())?;
                let offset = self
                    .base
                    .get_float64_param(self.params.wave_gen_offset, ch)
                    .map_err(|e| e.to_string())?;
                generator.user_buffers[ch as usize][..num_points]
                    .iter()
                    .map(|v| f64::from(*v) * amplitude + offset)
                    .collect()
            } else {
                generator.int_buffers[ch as usize][..num_points]
                    .iter()
                    .map(|v| f64::from(*v))
                    .collect()
            };
            per_chan.push(samples);
        }
        let mut waveform = Vec::with_capacity(per_chan.len() * num_points);
        for pt in 0..num_points {
            waveform.extend(per_chan.iter().map(|chan| chan[pt]));
        }
        wave_gen::volts_to_dac(&mut waveform);

        let options = wave_gen::start_wave_gen(
            dev,
            generator,
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
            saved,
        )
        .map_err(|e| format!("start_wave_gen error: {e}"))?;
        self.info("startWaveGen", "Calling AOutScan");
        // C startWaveGen: Run is 1 once the scan is running.
        self.base
            .params
            .set_int32(self.params.wave_gen_run, 0, 1)
            .map_err(|e| e.to_string())?;
        self.base.trace_print(
            TraceMask::FLOW,
            &format!(
                "{DRIVER}:startWaveGen: called cbAOutScan, firstChan={first_chan}, \
                 lastChan={last_chan}, numPoints*numWaveGenChans_={}, dwell={:.6}, \
                 options=0x{options:x}",
                (last_chan - first_chan + 1) as usize * num_points,
                generator.dwell_actual
            ),
        );
        // The axis of the waveform now playing follows the dwell it plays
        // at, which the device may have rounded (upstream-c-defects #230).
        let (time_buffer, time_wf) = if first_is_user {
            (
                &mut generator.user_time_buffer,
                self.params.wave_gen_user_time_wf,
            )
        } else {
            (
                &mut generator.int_time_buffer,
                self.params.wave_gen_int_time_wf,
            )
        };
        updates.push(wave_dig::time_update(
            time_buffer,
            time_wf,
            num_points,
            generator.dwell_actual,
        ));
        self.base
            .params
            .set_float64(self.params.wave_gen_dwell_actual, 0, generator.dwell_actual)
            .map_err(|e| e.to_string())?;
        self.base
            .params
            .set_float64(
                self.params.wave_gen_total_time,
                0,
                generator.dwell_actual * num_points as f64,
            )
            .map_err(|e| e.to_string())?;
        Ok(())
    }

    /// C `stopWaveGen`: Run back to 0, the scan stopped, and the outputs it
    /// drove put back.
    fn stop_generator(&mut self, dev: &DaqDevice, generator: &mut WaveGenState) {
        let _ = self.base.params.set_int32(self.params.wave_gen_run, 0, 0);
        for _ in wave_gen::stop_wave_gen(dev, generator) {
            self.info("stopWaveGen", "calling AOut");
        }
    }

    /// A waveform parameter changed: redefine `channel`'s waveform and, if the
    /// generator is running, restart it on the new one (C stopWaveGen +
    /// startWaveGen).
    fn redefine_waveform(
        &mut self,
        channel: i32,
        updates: &mut Vec<ParamSetValue>,
    ) -> Option<String> {
        let (device, state) = (self.device.clone(), self.state.clone());
        let dev = device.lock().unwrap();
        let mut st = state.lock().unwrap();
        let mut failure = self
            .define_waveform(&mut st.wave_gen, channel, updates)
            .err();
        if st.wave_gen.running {
            self.stop_generator(&dev, &mut st.wave_gen);
            if let Err(e) = self.start_generator(&dev, &mut st.wave_gen, updates) {
                failure = Some(e);
                let _ = self.base.params.set_int32(self.params.wave_gen_run, 0, 0);
            }
        }
        failure
    }

    /// C `startWaveDig`: read every digitizer setting and start the scan,
    /// publishing the actual dwell and total time -- or, when the device
    /// refuses, the dwell C reports for that (see [`wave_dig::start_wave_dig`]).
    fn start_digitizer(
        &mut self,
        dev: &DaqDevice,
        dig: &mut WaveDigState,
        updates: &mut Vec<ParamSetValue>,
    ) -> Result<(), String> {
        let get_i32 = |base: &PortDriverBase, reason| {
            base.get_int32_param(reason, 0).map_err(|e| e.to_string())
        };
        let flag = |base: &PortDriverBase, reason| get_i32(base, reason).map(|v| v != 0);
        let first_chan = get_i32(&self.base, self.params.wave_dig_first_chan)?.max(0) as usize;
        let num_chans = get_i32(&self.base, self.params.wave_dig_num_chans)?.max(0) as usize;
        let num_points = get_i32(&self.base, self.params.wave_dig_num_points)?.max(0) as usize;
        let dwell = self
            .base
            .get_float64_param(self.params.wave_dig_dwell, 0)
            .map_err(|e| e.to_string())?;
        let input_mode = get_i32(&self.base, self.params.analog_in_mode)?;
        let mut ranges = [uldaq_sys::BIP10VOLTS; MAX_ANALOG_IN];
        for (ch, range) in ranges.iter_mut().enumerate() {
            *range = self
                .base
                .get_int32_param(self.params.analog_in_range, ch as i32)
                .map_err(|e| e.to_string())?;
        }
        let scan = WaveDigScan {
            first_chan,
            num_chans,
            num_points,
            dwell,
            input_mode,
            ranges,
            ext_trigger: flag(&self.base, self.params.wave_dig_ext_trigger)?,
            ext_clock: flag(&self.base, self.params.wave_dig_ext_clock)?,
            continuous: flag(&self.base, self.params.wave_dig_continuous)?,
            retrigger: flag(&self.base, self.params.wave_dig_retrigger)?,
            burst_mode: flag(&self.base, self.params.wave_dig_burst_mode)?,
        };
        self.base
            .params
            .set_int32(self.params.wave_dig_current_point, 0, 0)
            .map_err(|e| e.to_string())?;
        let started = wave_dig::start_wave_dig(dev, dig, &scan);
        // A dwell comes back from every start that got past the queue load.
        let dwell_actual = match &started {
            Ok(s) => Some(s.dwell_actual),
            Err(e) => e.dwell_actual,
        };
        if dwell_actual.is_some() {
            self.info("startWaveDig", "Calling ALoadQueue");
        }
        if let Some(d) = dwell_actual {
            self.base
                .params
                .set_float64(self.params.wave_dig_dwell_actual, 0, d)
                .map_err(|e| e.to_string())?;
        }
        match started {
            Ok(s) => {
                self.info("startWaveDig", "Calling AInScan");
                self.base
                    .params
                    .set_int32(self.params.wave_dig_run, 0, 1)
                    .map_err(|e| e.to_string())?;
                self.base.trace_print(
                    TraceMask::FLOW,
                    &format!(
                        "{DRIVER}:startWaveDig: called cbAInScan, firstChan={first_chan}, \
                         lastChan={}, numPoints={num_points}, dwell={:.6}, options=0x{:x}",
                        first_chan + num_chans - 1,
                        s.dwell_actual,
                        s.options
                    ),
                );
                self.base
                    .params
                    .set_float64(
                        self.params.wave_dig_total_time,
                        0,
                        s.dwell_actual * num_points as f64,
                    )
                    .map_err(|e| e.to_string())?;
                // The time axis of the samples this scan takes follows the
                // dwell it runs at, not the one asked for (upstream-c-defects
                // #230).
                updates.push(wave_dig::time_update(
                    &mut dig.time_buffer,
                    self.params.wave_dig_time_wf,
                    num_points,
                    s.dwell_actual,
                ));
                Ok(())
            }
            Err(e) => Err(format!("start_wave_dig error: {}", e.message)),
        }
    }

    /// C `stopWaveDig`, the single end of every digitizer scan -- a Stop, a
    /// finished one-shot, a scan the device dropped: Run goes back to 0, the
    /// points acquired go to the waveform records, the scan is stopped, and
    /// with AutoRestart a new scan starts from the current settings.
    fn stop_digitizer(
        &mut self,
        dev: &DaqDevice,
        dig: &mut WaveDigState,
        updates: &mut Vec<ParamSetValue>,
    ) -> Option<String> {
        let _ = self.base.params.set_int32(self.params.wave_dig_run, 0, 0);
        let _ = self.base.params.set_int32(
            self.params.wave_dig_current_point,
            0,
            dig.current_point as i32,
        );
        self.trace_wave_dig_callbacks(dig);
        updates.extend(wave_dig::waveform_updates(&self.params, dig));
        if wave_dig::stop_wave_dig(dev, dig) {
            self.info("stopWaveDig", "Stopping AIn scan");
        }
        let auto_restart = self
            .base
            .get_int32_param(self.params.wave_dig_auto_restart, 0)
            .unwrap_or(0)
            != 0;
        if auto_restart {
            return self.start_digitizer(dev, dig, updates).err();
        }
        None
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
        wave_dig::time_update(
            &mut st.wave_dig.time_buffer,
            self.params.wave_dig_time_wf,
            num_points,
            dwell,
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
        vec![
            wave_dig::time_update(
                &mut st.wave_gen.user_time_buffer,
                self.params.wave_gen_user_time_wf,
                user.0,
                user.1,
            ),
            wave_dig::time_update(
                &mut st.wave_gen.int_time_buffer,
                self.params.wave_gen_int_time_wf,
                int.0,
                int.1,
            ),
        ]
    }

    /// C `isThermocouple`: `chan` is configured as a thermocouple input. The
    /// thermocouple-type and open-detect writes need it: ulAISetConfig
    /// rejects both with ERR_BAD_AI_CHAN_TYPE on a voltage channel.
    fn is_thermocouple(&self, chan: i32) -> bool {
        self.base
            .get_int32_param(self.params.analog_in_type, chan)
            .unwrap_or(0)
            != 0
    }

    /// C's thermocouple-type write (drvMultiFunction.cpp:1972-1978,
    /// :2004-2016): `chan`'s stored type to the device, reported under
    /// `message` as C words each of the two call sites.
    fn set_tc_type(&self, dev: &DaqDevice, chan: i32, message: &str) -> Option<String> {
        let tc = self
            .base
            .get_int32_param(self.params.thermocouple_type, chan)
            .ok()?;
        match dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TC_TYPE, chan as u32, tc as i64) {
            Ok(()) => {
                self.info("writeInt32", message);
                None
            }
            Err(e) => Some(format!("ai_set_config tc_type error: {e}")),
        }
    }

    /// C `setOpenThermocoupleDetect` (drvMultiFunction.cpp:2214-2238):
    /// `chan`'s stored open-thermocouple detection to the device.
    fn set_open_detect(&self, dev: &DaqDevice, chan: i32) -> Option<String> {
        let detect = self
            .base
            .get_int32_param(self.params.thermocouple_open_detect, chan)
            .ok()?;
        let otd = if detect != 0 {
            uldaq_sys::OTD_ENABLED
        } else {
            uldaq_sys::OTD_DISABLED
        };
        match dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_OTD_MODE, chan as u32, otd) {
            Ok(()) => {
                self.info(
                    "setOpenThermocoupleDetect",
                    "Setting thermocouple open detect mode",
                );
                None
            }
            Err(e) => Some(format!("ai_set_config otd error: {e}")),
        }
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
        // What C's trace line reports: the value as written, before any clamp.
        let written = value;

        // The scan buffers are sized once from maxInputPoints/maxOutputPoints,
        // so a point count above that capacity must never reach the store --
        // wave_dig indexes its fixed per-channel buffers by it. The records
        // carry the same bound in DRVL/DRVH; this gate is what makes the
        // invariant hold no matter what the database says.
        let value = if reason == self.params.wave_dig_num_points {
            value.clamp(1, self.max_input_points as i32)
        } else if reason == self.params.wave_gen_num_points {
            value.clamp(1, self.max_output_points as i32)
        } else if reason == self.params.analog_in_mode {
            // The USB-2408 has only differential and single-ended inputs; C
            // takes anything but differential as single-ended
            // (drvMultiFunction.cpp:1989), so no other mode reaches ulAIn.
            if value == uldaq_sys::AI_DIFFERENTIAL {
                uldaq_sys::AI_DIFFERENTIAL
            } else {
                uldaq_sys::AI_SINGLE_ENDED
            }
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
            match dev.counter_clear(addr) {
                Ok(()) => self.info("writeInt32", "Resetting counter"),
                Err(e) => last_error = Some(format!("counter_clear error: {e}")),
            }
        } else if reason == self.params.analog_out_value {
            // Only write immediately if sync mode is disabled
            let sync_enable = self
                .base
                .get_int32_param(self.params.analog_out_sync_enable, 0)
                .unwrap_or(0);
            if sync_enable == 0 && self.state.lock().unwrap().wave_gen.running {
                // C refuses before touching the DAC (drvMultiFunction.cpp:
                // 2131-2135): the generator owns both outputs while it runs.
                last_error =
                    Some("cannot write analog outputs while waveform generator is running".into());
            } else if sync_enable == 0 {
                let dev = self.device.lock().unwrap();
                let range = self
                    .base
                    .get_int32_param(self.params.analog_out_range, addr)?;
                match dev.analog_out(addr, range, uldaq_sys::AOUT_FF_NOSCALEDATA, value as f64) {
                    Ok(()) => self.info("writeInt32", "calling AOut"),
                    Err(e) => last_error = Some(format!("analog_out error: {e}")),
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
            match dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TYPE, addr as u32, chan_type) {
                Ok(()) => self.info("writeInt32", "Setting analog input type"),
                Err(e) => last_error = Some(format!("ai_set_config chan_type error: {e}")),
            }
            if value != 0 {
                // The channel has just become a thermocouple input; the TC type
                // and open-detect settings the records already hold could not be
                // pushed while it was a voltage channel, so push them now, both,
                // as C does whatever the type write returned.
                let tc_type = self.set_tc_type(&dev, addr, "Set thermocouple type");
                let open_detect = self.set_open_detect(&dev, addr);
                last_error = last_error.or(tc_type).or(open_detect);
            }
        } else if reason == self.params.analog_in_rate {
            // C: the per-channel ADC data rate (samples/s), which sets the
            // conversion time and the 50/60 Hz rejection of every read on the
            // channel, ulAIn, ulTIn and the scan queue alike.
            let dev = self.device.lock().unwrap();
            match dev.ai_set_config_dbl(uldaq_sys::AI_CFG_CHAN_DATA_RATE, addr as u32, value as f64)
            {
                Ok(()) => self.info("writeInt32", "Setting data rate"),
                Err(e) => last_error = Some(format!("ai_set_config_dbl data_rate error: {e}")),
            }
        } else if reason == self.params.analog_in_mode {
            // Kept in the parameter and applied at each ulAIn / scan, as C's
            // Linux build keeps it in aiInputMode_ (drvMultiFunction.cpp:
            // 1984-1991), which reports it set.
            self.info("writeInt32", "Setting analog input mode");
        } else if reason == self.params.thermocouple_type && self.is_thermocouple(addr) {
            let dev = self.device.lock().unwrap();
            last_error = self.set_tc_type(&dev, addr, "Setting thermocouple type");
        } else if reason == self.params.thermocouple_open_detect && self.is_thermocouple(addr) {
            let dev = self.device.lock().unwrap();
            last_error = self.set_open_detect(&dev, addr);
        } else if reason == self.params.trigger_mode {
            // Cached for the scans, as C's Linux build caches it
            // (drvMultiFunction.cpp:2073-2083); a mode mapTriggerType has no
            // uldaq trigger for is refused as C refuses it.
            if is_supported_trigger_mode(value) {
                self.info("writeInt32", "Setting trigger mode");
            } else {
                last_error = Some(format!("unsupported trigger mode {value}"));
            }
        } else if reason == self.params.wave_dig_trigger_count {
            self.info("writeInt32", "Setting waveDig trigger count");
        } else if reason == self.params.wave_gen_trigger_count {
            self.info("writeInt32", "Setting waveGen trigger count");
        } else if reason == self.params.wave_dig_run {
            let (device, state) = (self.device.clone(), self.state.clone());
            let dev = device.lock().unwrap();
            let mut st = state.lock().unwrap();
            // C parity (drvMultiFunction.cpp:2086-2091): start only when idle,
            // stop only when running.
            if value != 0 && !st.wave_dig.running {
                if let Err(e) = self.start_digitizer(&dev, &mut st.wave_dig, &mut wave_arrays) {
                    last_error = Some(e);
                    self.base.params.set_int32(reason, addr, 0)?;
                }
            } else if value == 0 && st.wave_dig.running {
                last_error = self.stop_digitizer(&dev, &mut st.wave_dig, &mut wave_arrays);
            }
        } else if reason == self.params.wave_dig_scan_end {
            // The poller saw scan `value` go idle: end it here, the one owner
            // of that transition. A report about an earlier scan is ignored.
            let (device, state) = (self.device.clone(), self.state.clone());
            let dev = device.lock().unwrap();
            let mut st = state.lock().unwrap();
            if st.wave_dig.running && st.wave_dig.generation as i32 == value {
                last_error = self.stop_digitizer(&dev, &mut st.wave_dig, &mut wave_arrays);
            }
        } else if reason == self.params.wave_dig_read_wf {
            let st = self.state.lock().unwrap();
            self.trace_wave_dig_callbacks(&st.wave_dig);
            wave_arrays = wave_dig::waveform_updates(&self.params, &st.wave_dig);
        } else if reason == self.params.wave_gen_run {
            let (device, state) = (self.device.clone(), self.state.clone());
            let dev = device.lock().unwrap();
            let mut st = state.lock().unwrap();
            // Same start/stop guard as the digitizer above.
            if value != 0 && !st.wave_gen.running {
                if let Err(e) = self.start_generator(&dev, &mut st.wave_gen, &mut wave_arrays) {
                    last_error = Some(e);
                    // The scan was refused, so the record must not be left
                    // reading "Run" -- it follows this parameter back to Stop.
                    self.base.params.set_int32(reason, addr, 0)?;
                }
            } else if value == 0 && st.wave_gen.running {
                self.stop_generator(&dev, &mut st.wave_gen);
            }
        } else if reason == self.params.wave_gen_scan_end {
            // The poller saw generator scan `value` go idle: end it here, the
            // one owner of that transition. A report about an earlier scan is
            // ignored.
            let (device, state) = (self.device.clone(), self.state.clone());
            let dev = device.lock().unwrap();
            let mut st = state.lock().unwrap();
            if st.wave_gen.running && st.wave_gen.generation as i32 == value {
                self.stop_generator(&dev, &mut st.wave_gen);
            }
        }

        // C writeInt32 (drvMultiFunction.cpp:2171-2183): these redefine the
        // channel's waveform, and a running generator restarts with it.
        if [
            self.params.wave_gen_wave_type,
            self.params.wave_gen_user_num_points,
            self.params.wave_gen_int_num_points,
            self.params.wave_gen_enable,
            self.params.wave_gen_ext_trigger,
            self.params.wave_gen_ext_clock,
            self.params.wave_gen_continuous,
        ]
        .contains(&reason)
            && let Some(e) = self.redefine_waveform(addr, &mut wave_arrays)
        {
            last_error = Some(e);
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
        wave_arrays.extend(time_wf);

        self.finish_write(
            user,
            addr,
            last_error,
            wave_arrays,
            "writeInt32",
            format_args!("wrote {written} to address {addr}"),
        )
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

        // C writeFloat64 (drvMultiFunction.cpp:2294-2305): these redefine the
        // channel's waveform, and a running generator restarts with it.
        let mut last_error = None;
        if [
            self.params.wave_gen_user_dwell,
            self.params.wave_gen_int_dwell,
            self.params.wave_gen_pulse_width,
            self.params.wave_gen_pulse_delay,
            self.params.wave_gen_amplitude,
            self.params.wave_gen_offset,
        ]
        .contains(&reason)
        {
            last_error = self.redefine_waveform(device_addr(addr), &mut wave_arrays);
        }
        wave_arrays.extend(time_wf);

        self.finish_write(
            user,
            addr,
            last_error,
            wave_arrays,
            "writeFloat64",
            format_args!("wrote {value:.6} to address {}", device_addr(addr)),
        )
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
        let src: &[f32] = if reason == self.params.wave_gen_user_time_wf {
            &st.wave_gen.user_time_buffer
        } else if reason == self.params.wave_gen_int_time_wf {
            &st.wave_gen.int_time_buffer
        } else if reason == self.params.wave_dig_time_wf {
            &st.wave_dig.time_buffer
        } else if reason == self.params.wave_gen_user_wf || reason == self.params.wave_gen_int_wf {
            let buffers = if reason == self.params.wave_gen_user_wf {
                &st.wave_gen.user_buffers
            } else {
                &st.wave_gen.int_buffers
            };
            let Some(wf) = buffers.get(device_addr(user.addr) as usize) else {
                return Ok(0);
            };
            wf
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

    /// Load a user-defined generator waveform (volts) for one channel, C
    /// `writeFloat32Array` (drvMultiFunction.cpp:2504-2529): the points
    /// written replace the head of the buffer, and more than it holds is
    /// refused rather than cut short.
    fn write_float32_array(&mut self, user: &AsynUser, data: &[f32]) -> AsynResult<()> {
        if user.reason != self.params.wave_gen_user_wf {
            return Ok(());
        }
        let ch = device_addr(user.addr) as usize;
        if ch >= MAX_ANALOG_OUT {
            return Ok(());
        }
        let mut st = self.state.lock().unwrap();
        let buffer = &mut st.wave_gen.user_buffers[ch];
        if data.len() > buffer.len() {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!(
                    "{} points written, the waveform holds {}",
                    data.len(),
                    buffer.len()
                ),
            });
        }
        buffer[..data.len()].copy_from_slice(data);
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
        let direction = self
            .base
            .get_uint32_param(self.params.digital_direction, 0)?;

        if reason == self.params.digital_output {
            let dev = self.device.lock().unwrap();
            if mask & direction == PORT_MASK {
                // Every bit is an output and every bit is written: one word
                // write, as C does.
                match dev.digital_out(uldaq_sys::AUXPORT, (value & mask) as u64) {
                    Ok(()) => self.info("writeUInt32Digital", "Calling DOut"),
                    Err(e) => last_error = Some(format!("digital_out error: {e}")),
                }
            } else {
                for (bit, level) in output_bits(value, mask, direction, NUM_IO_BITS) {
                    match dev.digital_bit_out(uldaq_sys::AUXPORT, bit, level) {
                        Ok(()) => self.info("writeUInt32Digital", "Calling DBitOut"),
                        Err(e) => last_error = Some(format!("digital_bit_out error: {e}")),
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
                    match dev.digital_bit_out(uldaq_sys::AUXPORT, bit as i32, false) {
                        Ok(()) => self.info("writeUInt32Digital", "Calling BitOut"),
                        Err(e) => last_error = Some(format!("digital_bit_out error: {e}")),
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
                        match dev.digital_config_bit(uldaq_sys::AUXPORT, bit as i32, dir) {
                            Ok(()) => self.info("writeUInt32Digital", "Calling ConfigBit"),
                            Err(e) => {
                                last_error = Some(format!("digital_config_bit error: {e}"));
                            }
                        }
                    }
                }
            }
        }

        self.base.params.set_uint32(reason, addr, value, mask, 0)?;

        // C reads the direction only for an output write; a direction write
        // prints the 0 it was initialised to (drvMultiFunction.cpp:2337, :2384).
        let printed_direction = if reason == self.params.digital_output {
            direction
        } else {
            0
        };
        self.finish_write(
            user,
            addr,
            last_error,
            Vec::new(),
            "writeUInt32Digital",
            format_args!(
                "function={reason}, wrote value=0x{value:x}, mask=0x{mask:x}, \
                 direction=0x{printed_direction:x}"
            ),
        )
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_hysteresis_gates_lack_a_uldaq_trigger() {
        for mode in [0, 1, 4, 12, 19] {
            assert!(is_supported_trigger_mode(mode), "{mode}");
        }
        for mode in [-1, 2, 3, 20] {
            assert!(!is_supported_trigger_mode(mode), "{mode}");
        }
    }
}
