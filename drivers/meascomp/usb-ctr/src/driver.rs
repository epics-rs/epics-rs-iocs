use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use meascomp::device::DaqDevice;
use meascomp::digital_io::output_bits;

use crate::mcs::{self, McsScan, McsState};
use crate::params::*;
use crate::poller::{self, PollerState};
use crate::pulse_gen;
use crate::scaler::ScalerState;

/// C `USBCTR::writeInt32`/`writeFloat64` resolve the counter number through
/// `asynPortDriver::getAddress` (drvUSBCTR.cpp:1096, 1288), which maps the
/// no-device addr -1 to 0 (asynPortDriver.cpp:1901-1913).
fn device_addr(addr: i32) -> i32 {
    if addr == -1 { 0 } else { addr }
}

/// USB-CTR08 port driver.
pub struct CtrDriver {
    base: PortDriverBase,
    pub params: CtrParams,
    pub device: Arc<Mutex<DaqDevice>>,
    pub state: Arc<Mutex<PollerState>>,
    pub max_time_points: usize,
    /// Whether AUXPORT accepts a direction change at all (`DPIOT_IO` /
    /// `DPIOT_BITIO`). The USB-CTR08 reports `DPIOT_BITIO`.
    dio_configurable: bool,
    /// Whether each timer is actually generating, C `pulseGenRunning_`.
    /// Only [`CtrDriver::start_pulse_generator`] sets it, once the device has
    /// accepted the start, and only [`CtrDriver::stop_pulse_generator`]
    /// clears it -- the PULSE_RUN setpoint is what was asked for, not this.
    pulse_running: [bool; NUM_TIMERS],
}

impl CtrDriver {
    pub fn new(port_name: &str, device: DaqDevice, max_time_points: usize) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            MAX_MCS_COUNTERS,
            PortFlags {
                multi_device: true,
                can_block: false,
                destructible: true,
            },
        );
        let params = CtrParams::create(&mut base)?;

        // Set defaults
        base.set_float64_param(params.poll_sleep_ms, 0, 50.0)?;
        base.set_int32_param(params.mcs_max_points, 0, max_time_points as i32)?;
        base.set_int32_param(params.mca_num_channels, 0, max_time_points as i32)?;

        // Store device info
        let product_name = device.product_name();
        let product_id = device.product_id();
        let uid = device.unique_id();
        let fw = device.firmware_version().unwrap_or_default();
        let ul_ver = DaqDevice::ul_version().unwrap_or_default();

        let model =
            CtrModel::from_product_name(&product_name).ok_or_else(|| AsynError::Status {
                status: AsynStatus::Error,
                message: format!("{product_name} is not a USB-CTR08 or USB-CTR04"),
            })?;
        base.set_int32_param(params.model, 0, model as i32)?;
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

        // Seed DIGITAL_INPUT so the bi records have a value to read at init.
        // The poller only pushes on a changed bit, and its one forced first
        // callback happens here -- before dbLoadRecords has run.
        if let Ok(data) = device.digital_in(uldaq_sys::AUXPORT) {
            base.set_uint32_param(params.digital_input, 0, data as u32, 0xFFFF_FFFF, 0)?;
        }

        // Put the pulse generators in a known state, as C's constructor does:
        // a timer left running by a previous IOC would otherwise keep
        // pulsing while its Run record reads Stop.
        for timer in 0..NUM_TIMERS as i32 {
            if let Err(e) = pulse_gen::stop(&device, timer) {
                log::error!("pulse_gen stop({timer}) error: {e}");
            }
        }

        let state = Arc::new(Mutex::new(PollerState {
            num_counters: model.num_counters(),
            scaler: ScalerState::new(),
            mcs: McsState::new(max_time_points),
        }));

        println!("CtrDriver: port={port_name}, model={product_name}, serial={uid}, fw={fw}");

        Ok(Self {
            base,
            params,
            device: Arc::new(Mutex::new(device)),
            state,
            max_time_points,
            dio_configurable,
            pulse_running: [false; NUM_TIMERS],
        })
    }
}

impl CtrDriver {
    /// C `startPulseGenerator`: start `timer` from its parameters, write the
    /// timing the device actually runs back to them, and only then mark it
    /// running.
    fn start_pulse_generator(&mut self, dev: &DaqDevice, timer: i32) -> Result<(), String> {
        if timer < 0 || timer as usize >= NUM_TIMERS {
            return Err(format!("pulse generator {timer} does not exist"));
        }
        let get_f64 = |reason| {
            self.base
                .get_float64_param(reason, timer)
                .map_err(|e| e.to_string())
        };
        let period = get_f64(self.params.pulse_period)?;
        let duty = get_f64(self.params.pulse_duty_cycle)?;
        let delay = get_f64(self.params.pulse_delay)?;
        let get_i32 = |reason| {
            self.base
                .get_int32_param(reason, timer)
                .map_err(|e| e.to_string())
        };
        let count = get_i32(self.params.pulse_count)? as u64;
        let idle = get_i32(self.params.pulse_idle_state)?;
        let (actual_period, actual_duty, actual_delay) =
            pulse_gen::start(dev, timer, period, duty, delay, count, idle)
                .map_err(|e| format!("pulse_gen start error: {e}"))?;
        self.pulse_running[timer as usize] = true;
        let _ = self
            .base
            .set_float64_param(self.params.pulse_period, timer, actual_period);
        let _ = self
            .base
            .set_float64_param(self.params.pulse_duty_cycle, timer, actual_duty);
        let _ = self
            .base
            .set_float64_param(self.params.pulse_delay, timer, actual_delay);
        Ok(())
    }

    /// C `stopPulseGenerator`: the timer counts as stopped whether or not the
    /// stop command itself succeeds.
    fn stop_pulse_generator(&mut self, dev: &DaqDevice, timer: i32) -> Result<(), String> {
        if let Some(running) = self.pulse_running.get_mut(timer as usize) {
            *running = false;
        }
        pulse_gen::stop(dev, timer).map_err(|e| format!("pulse_gen stop error: {e}"))
    }

    /// C's restart on a timing change: stop and start again, but only a timer
    /// that is really running.
    fn restart_pulse_generator(&mut self, dev: &DaqDevice, timer: i32) -> Option<String> {
        if !self
            .pulse_running
            .get(timer as usize)
            .copied()
            .unwrap_or(false)
        {
            return None;
        }
        let stopped = self.stop_pulse_generator(dev, timer);
        let started = self.start_pulse_generator(dev, timer);
        stopped.and(started).err()
    }

    /// Common tail of every write: run the callbacks, then report a failure
    /// the way C does -- `asynError` back to the record, so it alarms --
    /// besides publishing it on LAST_ERROR_MESSAGE.
    fn finish_write(&mut self, addr: i32, last_error: Option<String>) -> AsynResult<()> {
        if let Some(msg) = &last_error {
            log::error!("{msg}");
            let _ = self.base.params.set_value(
                self.params.last_error_message,
                0,
                ParamValue::Octet(msg.clone().into_bytes()),
            );
        }
        self.base.call_param_callbacks(addr)?;
        match last_error {
            None => Ok(()),
            Some(message) => Err(AsynError::Status {
                status: AsynStatus::Error,
                message,
            }),
        }
    }
}

impl PortDriver for CtrDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let mut last_error: Option<String> = None;
        let reason = user.reason;
        let addr = device_addr(user.addr);

        self.base.params.set_int32(reason, addr, value)?;

        if reason == self.params.pulse_run {
            let device = self.device.clone();
            let dev = device.lock().unwrap();
            // C starts even when it believes the timer runs: with Count != 0
            // there is no way to know that it has finished.
            let result = if value != 0 {
                self.start_pulse_generator(&dev, addr)
            } else if self
                .pulse_running
                .get(addr as usize)
                .copied()
                .unwrap_or(false)
            {
                self.stop_pulse_generator(&dev, addr)
            } else {
                Ok(())
            };
            last_error = result.err();
        } else if reason == self.params.pulse_count || reason == self.params.pulse_idle_state {
            let device = self.device.clone();
            let dev = device.lock().unwrap();
            last_error = self.restart_pulse_generator(&dev, addr);
        } else if reason == self.params.counter_reset {
            // Any write resets, as C's ulCLoad(CRT_LOAD, 0) does.
            let dev = self.device.lock().unwrap();
            if let Err(e) = dev.counter_clear(addr) {
                last_error = Some(format!("counter_clear error: {e}"));
            }
        } else if reason == self.params.digital_output {
            let dev = self.device.lock().unwrap();
            if let Err(e) = dev.digital_out(uldaq_sys::AUXPORT, value as u64) {
                last_error = Some(format!("digital_out error: {e}"));
            }
        } else if reason == self.params.mca_start_acquire {
            let already_running = self.state.lock().unwrap().mcs.running;
            if value != 0 && !already_running {
                let dev = self.device.lock().unwrap();
                let mut st = self.state.lock().unwrap();
                let num_channels =
                    self.base.get_int32_param(self.params.mca_num_channels, 0)? as usize;
                let dwell = self.base.get_float64_param(self.params.mca_dwell_time, 0)?;
                let ch_adv = self
                    .base
                    .get_int32_param(self.params.mca_ch_advance_source, 0)?;
                let prescale = self.base.get_int32_param(self.params.mca_prescale, 0)?;
                let trigger = self.base.get_int32_param(self.params.trigger_mode, 0)? != 0;
                let enable = self
                    .base
                    .get_uint32_param(self.params.mcs_counter_enable, 0)?;
                st.mcs.preset_real_time = self
                    .base
                    .get_float64_param(self.params.mca_preset_real, 0)?;
                let point0_no_clear = self
                    .base
                    .get_int32_param(self.params.mcs_point0_action, 0)?
                    != 0;
                let num_counters = st.num_counters;
                if let Err(e) = mcs::start_mcs(
                    &dev,
                    &mut st.mcs,
                    &McsScan {
                        num_points: num_channels,
                        dwell_time: dwell,
                        counter_enable: enable,
                        ch_advance_source: ch_adv,
                        prescale,
                        ext_trigger: trigger,
                        point0_no_clear,
                    },
                    num_counters,
                ) {
                    last_error = Some(format!("start_mcs error: {e}"));
                }
                self.base
                    .params
                    .set_float64(self.params.mca_dwell_time, 0, st.mcs.dwell_time)?;
                self.base
                    .params
                    .set_int32(self.params.mca_acquiring, 0, 1)?;
            }
        } else if reason == self.params.mca_stop_acquire {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            mcs::stop_mcs(&dev, &mut st.mcs);
            st.mcs.acquiring = false;
            self.base
                .params
                .set_int32(self.params.mca_acquiring, 0, 0)?;
        } else if reason == self.params.mca_erase {
            let mut st = self.state.lock().unwrap();
            mcs::erase_mcs(&mut st.mcs);
        }

        self.finish_write(addr, last_error)
    }

    /// MCS spectrum readout. C `USBCTR::readInt32Array` / the `mcaReadData`
    /// the mca record and SIS38XX_waveform.template both go through: the data
    /// lives in McsState::mcs_buffers and only leaves the driver here.
    fn read_int32_array(&mut self, user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        if user.reason != self.params.mca_data {
            return Ok(0);
        }
        let counter = user.addr as usize;
        let num_channels = self
            .base
            .get_int32_param(self.params.mca_num_channels, 0)
            .unwrap_or(0)
            .max(0) as usize;
        let st = self.state.lock().unwrap();
        let Some(src) = st.mcs.mcs_buffers.get(counter) else {
            return Ok(0);
        };
        let n = buf.len().min(src.len()).min(num_channels);
        buf[..n].copy_from_slice(&src[..n]);
        Ok(n)
    }

    /// MCS time base (seconds from the start of the scan).
    fn read_float32_array(&mut self, user: &AsynUser, buf: &mut [f32]) -> AsynResult<usize> {
        if user.reason != self.params.mcs_time_wf {
            return Ok(0);
        }
        // Only the points the scan is configured for, as C
        // computeMCSTimes' doCallbacksFloat32Array(.., numTimePoints, ..)
        // does -- a full-length time base against a short spectrum is
        // unplottable.
        let num_channels = self
            .base
            .get_int32_param(self.params.mca_num_channels, 0)
            .unwrap_or(0)
            .max(0) as usize;
        let st = self.state.lock().unwrap();
        let n = buf.len().min(st.mcs.time_buffer.len()).min(num_channels);
        buf[..n].copy_from_slice(&st.mcs.time_buffer[..n]);
        Ok(n)
    }

    /// MCS absolute time base (seconds past the EPICS epoch, per acquired point).
    fn read_float64_array(&mut self, user: &AsynUser, buf: &mut [f64]) -> AsynResult<usize> {
        if user.reason != self.params.mcs_abs_time_wf {
            return Ok(0);
        }
        let st = self.state.lock().unwrap();
        // Points actually acquired, as C readMCS reports them.
        let n = buf
            .len()
            .min(st.mcs.abs_time_buffer.len())
            .min(st.mcs.current_point);
        buf[..n].copy_from_slice(&st.mcs.abs_time_buffer[..n]);
        Ok(n)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let mut last_error: Option<String> = None;
        let reason = user.reason;
        let addr = device_addr(user.addr);
        self.base.params.set_float64(reason, addr, value)?;

        // A timing change restarts a running pulse generator with it.
        if reason == self.params.pulse_period
            || reason == self.params.pulse_duty_cycle
            || reason == self.params.pulse_delay
        {
            let device = self.device.clone();
            let dev = device.lock().unwrap();
            last_error = self.restart_pulse_generator(&dev, addr);
        } else if reason == self.params.mca_dwell_time {
            // C computeMCSTimes: the time base follows the dwell as soon as
            // it is written, not only once a scan starts.
            let num_points = self
                .base
                .get_int32_param(self.params.mca_num_channels, 0)?
                .max(0) as usize;
            let times = {
                let mut st = self.state.lock().unwrap();
                let n = mcs::compute_times(&mut st.mcs, num_points, value);
                st.mcs.time_buffer[..n].to_vec()
            };
            self.base.params.set_value(
                self.params.mcs_time_wf,
                0,
                ParamValue::Float32Array(times.into()),
            )?;
        }

        self.finish_write(addr, last_error)
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
            // C USBCTR writes bit by bit, only the output bits in the mask.
            let direction = self
                .base
                .get_uint32_param(self.params.digital_direction, 0)?;
            let dev = self.device.lock().unwrap();
            for (bit, level) in output_bits(value, mask, direction, NUM_IO_BITS) {
                if let Err(e) = dev.digital_bit_out(uldaq_sys::AUXPORT, bit, level) {
                    last_error = Some(format!("digital_bit_out error: {e}"));
                }
            }
        } else if reason == self.params.digital_direction {
            if !self.dio_configurable {
                last_error =
                    Some("digital direction is fixed on this model; ignoring write".to_string());
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

        self.finish_write(addr, last_error)
    }
}

/// Runtime wrapper exposing the port handle and device.
pub struct CtrRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: CtrParams,
    pub device: Arc<Mutex<DaqDevice>>,
    pub state: Arc<Mutex<PollerState>>,
    _poller_handle: std::thread::JoinHandle<()>,
}

impl CtrRuntime {
    pub fn port_handle(&self) -> &epics_rs::asyn::port_handle::PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// Create a USB-CTR driver, start the port runtime actor and polling thread.
pub fn create_usb_ctr(
    port_name: &str,
    unique_id: &str,
    max_time_points: usize,
) -> Result<CtrRuntime, String> {
    let device = DaqDevice::connect(unique_id)
        .map_err(|e| format!("failed to connect to CTR device: {e}"))?;

    let driver = CtrDriver::new(port_name, device, max_time_points)
        .map_err(|e| format!("failed to create CtrDriver: {e}"))?;

    let params = driver.params;
    let device = driver.device.clone();
    let state = driver.state.clone();

    let (runtime_handle, _actor_jh) = create_port_runtime(driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the USB-CTR port runtime: {e}"))?;

    let poller_handle = poller::start_poller(
        runtime_handle.port_handle().clone(),
        params,
        device.clone(),
        state.clone(),
    );

    Ok(CtrRuntime {
        runtime_handle,
        params,
        device,
        state,
        _poller_handle: poller_handle,
    })
}
