use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use meascomp::device::DaqDevice;

use crate::params::*;
use crate::poller::{self, PollerState};
use crate::wave_dig::{self, WaveDigScan, WaveDigState};
use crate::wave_gen::{self, WaveGenScan, WaveGenState};

/// USB-2408-2AO port driver.
pub struct MultiFunctionDriver {
    base: PortDriverBase,
    pub params: MultiFunctionParams,
    pub device: Arc<Mutex<DaqDevice>>,
    pub state: Arc<Mutex<PollerState>>,
    pub max_input_points: usize,
    pub max_output_points: usize,
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
        base.set_string_param(params.driver_version, 0, "0.1.0".into())?;

        // Configure AUXPORT as input by default
        let _ = device.digital_config_port(uldaq_sys::AUXPORT, uldaq_sys::DD_INPUT);

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
        })
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
        let reason = user.reason;
        let addr = user.addr;

        self.base.params.set_int32(reason, addr, value)?;

        if reason == self.params.counter_reset && value != 0 {
            let dev = self.device.lock().unwrap();
            if let Err(e) = dev.counter_clear(addr) {
                log::error!("counter_clear error: {e}");
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
                    log::error!("analog_out error: {e}");
                }
            }
        } else if reason == self.params.analog_out_sync_write {
            // Simultaneous write of all analog outputs
            if value != 0 {
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
                    log::error!("analog_out_array error: {e}");
                }
            }
        } else if reason == self.params.analog_in_type {
            let dev = self.device.lock().unwrap();
            let chan_type = if value != 0 {
                uldaq_sys::AI_TC
            } else {
                uldaq_sys::AI_VOLTAGE
            };
            if let Err(e) = dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TYPE, addr as u32, chan_type) {
                log::error!("ai_set_config chan_type error: {e}");
            }
        } else if reason == self.params.thermocouple_type {
            let dev = self.device.lock().unwrap();
            if let Err(e) =
                dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_TC_TYPE, addr as u32, value as i64)
            {
                log::error!("ai_set_config tc_type error: {e}");
            }
        } else if reason == self.params.thermocouple_open_detect {
            let dev = self.device.lock().unwrap();
            let otd = if value != 0 {
                uldaq_sys::OTD_ENABLED
            } else {
                uldaq_sys::OTD_DISABLED
            };
            if let Err(e) = dev.ai_set_config(uldaq_sys::AI_CFG_CHAN_OTD_MODE, addr as u32, otd) {
                log::error!("ai_set_config otd error: {e}");
            }
        } else if reason == self.params.wave_dig_run {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            if value != 0 {
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
                    log::error!("start_wave_dig error: {e}");
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
            } else {
                wave_dig::stop_wave_dig(&dev, &mut st.wave_dig);
            }
        } else if reason == self.params.wave_gen_run {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            if value != 0 {
                let num_points = self
                    .base
                    .get_int32_param(self.params.wave_gen_num_points, 0)?
                    as usize;
                let freq = self.base.get_float64_param(self.params.wave_gen_freq, 0)?;
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
                for ch in 0..MAX_ANALOG_OUT as i32 {
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
                    per_chan.push(wave_gen::generate_waveform(
                        wave_type, num_points, amp, offset, pw,
                    ));
                }
                // Interleave: [ch0_pt0, ch1_pt0, ch0_pt1, ch1_pt1, ...]
                let mut waveform = Vec::with_capacity(MAX_ANALOG_OUT * num_points);
                for pt in 0..num_points {
                    waveform.extend(per_chan.iter().map(|chan| chan[pt]));
                }
                // Convert voltage to raw 16-bit DAC units (NOSCALEDATA mode)
                wave_gen::volts_to_dac(&mut waveform);

                if let Err(e) = wave_gen::start_wave_gen(
                    &dev,
                    &mut st.wave_gen,
                    &WaveGenScan {
                        first_chan: 0,
                        last_chan: (MAX_ANALOG_OUT - 1) as i32,
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
                    log::error!("start_wave_gen error: {e}");
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
            } else {
                wave_gen::stop_wave_gen(&dev, &mut st.wave_gen);
            }
        }

        self.base.call_param_callbacks(addr)?;
        Ok(())
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

        self.base.call_param_callbacks(addr)?;
        Ok(())
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;

        if reason == self.params.digital_output {
            let dev = self.device.lock().unwrap();
            for bit in 0..NUM_IO_BITS {
                if mask & (1 << bit) != 0 {
                    let bit_val = (value >> bit) & 1;
                    if let Err(e) =
                        dev.digital_bit_out(uldaq_sys::AUXPORT, bit as i32, bit_val != 0)
                    {
                        log::error!("digital_bit_out error: {e}");
                    }
                }
            }
        } else if reason == self.params.digital_direction {
            let dev = self.device.lock().unwrap();
            for bit in 0..NUM_IO_BITS {
                if mask & (1 << bit) != 0 {
                    let dir = if (value >> bit) & 1 != 0 {
                        uldaq_sys::DD_OUTPUT
                    } else {
                        uldaq_sys::DD_INPUT
                    };
                    if let Err(e) = dev.digital_config_bit(uldaq_sys::AUXPORT, bit as i32, dir) {
                        log::error!("digital_config_bit error: {e}");
                    }
                }
            }
        }

        self.base.params.set_uint32(reason, addr, value, mask, 0)?;
        self.base.call_param_callbacks(addr)?;
        Ok(())
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

    let (runtime_handle, _actor_jh) = create_port_runtime(driver, RuntimeConfig::default());

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
