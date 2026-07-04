use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use meascomp::device::DaqDevice;

use crate::mcs::{self, McsScan, McsState};
use crate::params::*;
use crate::poller::{self, PollerState};
use crate::pulse_gen;
use crate::scaler::{self, ScalerState};

/// USB-CTR08 port driver.
pub struct CtrDriver {
    base: PortDriverBase,
    pub params: CtrParams,
    pub device: Arc<Mutex<DaqDevice>>,
    pub state: Arc<Mutex<PollerState>>,
    pub max_time_points: usize,
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

        base.set_string_param(params.model_name, 0, product_name.clone())?;
        base.set_int32_param(params.model_number, 0, product_id as i32)?;
        base.set_string_param(params.unique_id, 0, uid.clone())?;
        base.set_string_param(params.firmware_version, 0, fw.clone())?;
        base.set_string_param(params.ul_version, 0, ul_ver)?;
        base.set_string_param(params.driver_version, 0, "0.1.0".into())?;

        // Configure AUXPORT as bit-configurable input by default
        let _ = device.digital_config_port(uldaq_sys::AUXPORT, uldaq_sys::DD_INPUT);

        let state = Arc::new(Mutex::new(PollerState {
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
        })
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
        let reason = user.reason;
        let addr = user.addr;

        self.base.params.set_int32(reason, addr, value)?;

        if reason == self.params.pulse_run
            || reason == self.params.pulse_count
            || reason == self.params.pulse_idle_state
        {
            let dev = self.device.lock().unwrap();
            let running = self.base.get_int32_param(self.params.pulse_run, addr)? != 0;
            if running {
                // Stop first if restarting due to parameter change
                if reason != self.params.pulse_run {
                    let _ = pulse_gen::stop(&dev, addr);
                }
                let period = self
                    .base
                    .get_float64_param(self.params.pulse_period, addr)?;
                let duty = self
                    .base
                    .get_float64_param(self.params.pulse_duty_cycle, addr)?;
                let delay = self.base.get_float64_param(self.params.pulse_delay, addr)?;
                let count = self.base.get_int32_param(self.params.pulse_count, addr)? as u64;
                let idle = self
                    .base
                    .get_int32_param(self.params.pulse_idle_state, addr)?;
                match pulse_gen::start(&dev, addr, period, duty, delay, count, idle) {
                    Ok((actual_period, actual_duty, actual_delay)) => {
                        // Write back actual values from hardware
                        let _ = self.base.set_float64_param(
                            self.params.pulse_period,
                            addr,
                            actual_period,
                        );
                        let _ = self.base.set_float64_param(
                            self.params.pulse_duty_cycle,
                            addr,
                            actual_duty,
                        );
                        let _ = self.base.set_float64_param(
                            self.params.pulse_delay,
                            addr,
                            actual_delay,
                        );
                    }
                    Err(e) => log::error!("pulse_gen start error: {e}"),
                }
            } else if reason == self.params.pulse_run
                && let Err(e) = pulse_gen::stop(&dev, addr)
            {
                log::error!("pulse_gen stop error: {e}");
            }
        } else if reason == self.params.counter_reset {
            if value != 0 {
                let dev = self.device.lock().unwrap();
                if let Err(e) = dev.counter_clear(addr) {
                    log::error!("counter_clear error: {e}");
                }
            }
        } else if reason == self.params.digital_output {
            let dev = self.device.lock().unwrap();
            if let Err(e) = dev.digital_out(uldaq_sys::AUXPORT, value as u64) {
                log::error!("digital_out error: {e}");
            }
        } else if reason == self.params.scaler_arm {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            if value != 0 {
                if let Err(e) = scaler::start_scaler(&dev, &mut st.scaler) {
                    log::error!("start_scaler error: {e}");
                }
            } else {
                scaler::stop_scaler(&dev, &mut st.scaler);
            }
        } else if reason == self.params.scaler_reset {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            scaler::reset_scaler(&dev, &mut st.scaler);
            // Clear all presets (matches C++ resetScaler behavior)
            for i in 0..MAX_COUNTERS {
                st.scaler.presets[i] = 0;
                let _ = self
                    .base
                    .params
                    .set_int32(self.params.scaler_presets, i as i32, 0);
            }
        } else if reason == self.params.scaler_presets {
            let mut st = self.state.lock().unwrap();
            if (addr as usize) < MAX_COUNTERS {
                st.scaler.presets[addr as usize] = value as u64;
            }
        } else if reason == self.params.mca_start_acquire {
            if value != 0 {
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
                    .get_int32_param(self.params.mcs_counter_enable, 0)?
                    as u32;
                st.mcs.preset_real_time = self
                    .base
                    .get_float64_param(self.params.mca_preset_real, 0)?;
                let point0_no_clear = self
                    .base
                    .get_int32_param(self.params.mcs_point0_action, 0)?
                    != 0;
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
                ) {
                    log::error!("start_mcs error: {e}");
                }
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

        self.base.call_param_callbacks(addr)?;
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        self.base.params.set_float64(reason, addr, value)?;

        // Restart pulse generator if period or duty cycle changes while running
        if reason == self.params.pulse_period
            || reason == self.params.pulse_duty_cycle
            || reason == self.params.pulse_delay
        {
            let running = self
                .base
                .get_int32_param(self.params.pulse_run, addr)
                .unwrap_or(0)
                != 0;
            if running {
                let dev = self.device.lock().unwrap();
                let _ = pulse_gen::stop(&dev, addr);
                let period = self
                    .base
                    .get_float64_param(self.params.pulse_period, addr)?;
                let duty = self
                    .base
                    .get_float64_param(self.params.pulse_duty_cycle, addr)?;
                let delay = self.base.get_float64_param(self.params.pulse_delay, addr)?;
                let count = self.base.get_int32_param(self.params.pulse_count, addr)? as u64;
                let idle = self
                    .base
                    .get_int32_param(self.params.pulse_idle_state, addr)?;
                match pulse_gen::start(&dev, addr, period, duty, delay, count, idle) {
                    Ok((actual_period, actual_duty, actual_delay)) => {
                        let _ = self.base.set_float64_param(
                            self.params.pulse_period,
                            addr,
                            actual_period,
                        );
                        let _ = self.base.set_float64_param(
                            self.params.pulse_duty_cycle,
                            addr,
                            actual_duty,
                        );
                        let _ = self.base.set_float64_param(
                            self.params.pulse_delay,
                            addr,
                            actual_delay,
                        );
                    }
                    Err(e) => log::error!("pulse_gen restart error: {e}"),
                }
            }
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

/// Runtime wrapper exposing the port handle and device.
pub struct CtrRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: CtrParams,
    pub device: Arc<Mutex<DaqDevice>>,
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

    let (runtime_handle, _actor_jh) = create_port_runtime(driver, RuntimeConfig::default());

    let poller_handle = poller::start_poller(
        runtime_handle.port_handle().clone(),
        params,
        device.clone(),
        state,
    );

    Ok(CtrRuntime {
        runtime_handle,
        params,
        device,
        _poller_handle: poller_handle,
    })
}
