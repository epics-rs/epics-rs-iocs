use std::sync::{Arc, Mutex};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::asyn_record::PortClaim;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use meascomp::device::DaqDevice;
use uldaq_sys::UlError;

use crate::mcs::{self, McsScan, McsState};
use crate::params::*;
use crate::poller::{self, PollerState};
use crate::pulse_gen::{self, PulseRequest};
use crate::scaler::ScalerState;
use crate::trace::{self, SelfTrace};

/// C `USBCTR::writeInt32`/`writeFloat64` resolve the counter number through
/// `asynPortDriver::getAddress` (drvUSBCTR.cpp:1096, 1288), which maps the
/// no-device addr -1 to 0 (asynPortDriver.cpp:1901-1913).
fn device_addr(addr: i32) -> i32 {
    if addr == -1 { 0 } else { addr }
}

/// C's `status` from one uldaq call.
fn status_of<T>(result: &meascomp::error::Result<T>) -> UlError {
    result.as_ref().err().map_or(0, |e| e.code)
}

/// USB-CTR08 port driver.
pub struct CtrDriver {
    base: PortDriverBase,
    pub params: CtrParams,
    pub device: Arc<Mutex<DaqDevice>>,
    pub state: Arc<Mutex<PollerState>>,
    pub max_time_points: usize,
    /// C `pulseGenRunning_`.
    pulse_running: [bool; NUM_TIMERS],
}

/// drvUSBCTR.cpp:256-257.
const USBCTR_INTERRUPTS: InterfaceMask = InterfaceMask::INT32
    .or(InterfaceMask::UINT32_DIGITAL)
    .or(InterfaceMask::INT32_ARRAY)
    .or(InterfaceMask::FLOAT32_ARRAY)
    .or(InterfaceMask::FLOAT64)
    .or(InterfaceMask::FLOAT64_ARRAY)
    .or(InterfaceMask::OCTET);
const USBCTR_INTERFACES: InterfaceMask = USBCTR_INTERRUPTS.or(InterfaceMask::DRV_USER);

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
            USBCTR_INTERFACES,
            USBCTR_INTERRUPTS,
        );
        let params = CtrParams::create(&mut base)?;

        // Set defaults
        base.set_float64_param(params.poll_sleep_ms, 0, 50.0)?;
        base.set_int32_param(params.pulse_run, 0, 0)?;
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
        base.set_string_param(params.driver_version, 0, "4.3")?;

        // Seed DIGITAL_INPUT so the bi records have a value to read at init.
        // The poller only pushes on a changed bit, and its one forced first
        // callback happens here -- before dbLoadRecords has run.
        if let Ok(data) = device.digital_in(uldaq_sys::AUXPORT) {
            base.set_uint32_param(params.digital_input, 0, data as u32, 0xFFFF_FFFF, 0)?;
        }

        // C sets numCounters_ only for these two boards.
        let num_counters = match product_name.as_str() {
            "USB-CTR08" => {
                base.set_int32_param(params.model, 0, 0)?;
                8
            }
            "USB-CTR04" => {
                base.set_int32_param(params.model, 0, 1)?;
                4
            }
            _ => {
                println!("Unknown model");
                MAX_COUNTERS
            }
        };
        let state = Arc::new(Mutex::new(PollerState {
            num_counters,
            scaler: ScalerState::new(),
            mcs: McsState::new(max_time_points),
            trace: SelfTrace::default(),
        }));

        let mut driver = Self {
            base,
            params,
            device: Arc::new(Mutex::new(device)),
            state,
            max_time_points,
            pulse_running: [false; NUM_TIMERS],
        };
        // C's constructor ends its MCS setup with eraseMCS (drvUSBCTR.cpp:425).
        driver.erase_mcs()?;
        Ok(driver)
    }
}

impl CtrDriver {
    /// C `USBCTR::eraseMCS`: the buffers, the point and every address's
    /// elapsed times and counts.
    fn erase_mcs(&mut self) -> AsynResult<()> {
        let num_points = self.int(self.params.mca_num_channels, 0);
        mcs::erase_mcs(
            &mut self.state.lock().unwrap().mcs,
            num_points.max(0) as usize,
        );
        self.base
            .set_int32_param(self.params.mcs_current_point, 0, 0)?;
        for i in 0..MAX_MCS_COUNTERS as i32 {
            self.base
                .set_float64_param(self.params.mca_elapsed_live, i, 0.0)?;
            self.base
                .set_float64_param(self.params.mca_elapsed_real, i, 0.0)?;
            self.base
                .set_float64_param(self.params.mca_elapsed_counts, i, 0.0)?;
            self.base.call_param_callbacks(i)?;
        }
        Ok(())
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

    /// C `pulseGenRunning_[addr]`.
    fn pulse_running(&self, addr: i32) -> bool {
        usize::try_from(addr)
            .ok()
            .and_then(|i| self.pulse_running.get(i))
            .copied()
            .unwrap_or(false)
    }

    fn set_pulse_running(&mut self, addr: i32, running: bool) {
        if let Some(r) = usize::try_from(addr)
            .ok()
            .and_then(|i| self.pulse_running.get_mut(i))
        {
            *r = running;
        }
    }

    /// C `USBCTR::startPulseGenerator`: start from the timer's params and
    /// write back what the hardware gave.
    fn start_pulse_generator(&mut self, trace: &SelfTrace, addr: i32) -> UlError {
        let request = match self.pulse_request(addr) {
            Ok(r) => r,
            Err(_) => return -1,
        };
        let started = pulse_gen::start(trace, &self.device.lock().unwrap(), addr, &request);
        match started {
            Ok((period, duty_cycle, delay)) => {
                self.set_pulse_running(addr, true);
                let _ = self
                    .base
                    .set_float64_param(self.params.pulse_period, addr, period);
                let _ = self
                    .base
                    .set_float64_param(self.params.pulse_duty_cycle, addr, duty_cycle);
                let _ = self
                    .base
                    .set_float64_param(self.params.pulse_delay, addr, delay);
                0
            }
            Err(e) => e.code,
        }
    }

    /// C `USBCTR::stopPulseGenerator`.
    fn stop_pulse_generator(&mut self, trace: &SelfTrace, addr: i32) -> UlError {
        self.set_pulse_running(addr, false);
        status_of(&pulse_gen::stop(trace, &self.device.lock().unwrap(), addr))
    }

    fn pulse_request(&self, addr: i32) -> AsynResult<PulseRequest> {
        Ok(PulseRequest {
            period: self.float(self.params.pulse_period, addr),
            duty_cycle: self.float(self.params.pulse_duty_cycle, addr),
            delay: self.float(self.params.pulse_delay, addr),
            count: self.int(self.params.pulse_count, addr) as u64,
            idle_state: self.int(self.params.pulse_idle_state, addr),
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

    /// C `USBCTR::report` (drvUSBCTR.cpp:1545-1570): the timers, scaler and
    /// MCS state, then `asynPortDriver::report`.
    fn report(&mut self, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(out, "  Port: {}", self.base.port_name);
        if level >= 1 {
            let _ = writeln!(out, "  Pulse generators:");
            for (i, running) in self.pulse_running.iter().enumerate() {
                let _ = writeln!(out, "    {i}: Running:{}", i32::from(*running));
            }
            let current_point = self.int(self.params.mcs_current_point, 0);
            let st = self.state.lock().unwrap();
            let _ = writeln!(out, "  numCounters: {}", st.num_counters);
            let _ = writeln!(out, "  Scaler:");
            let _ = writeln!(out, "    Running: {}", i32::from(st.scaler.running));
            // C keeps both arrays as epicsInt32.
            for i in 0..st.num_counters {
                let _ = writeln!(
                    out,
                    "    {i}: preset={}, count={}",
                    st.scaler.presets[i] as i32, st.scaler.counts[i] as i32
                );
            }
            let _ = writeln!(out, "  MCS:");
            let _ = writeln!(out, "    Running: {}", i32::from(st.mcs.running));
            let _ = writeln!(out, "    maxTimePoints: {}", self.max_time_points);
            let _ = writeln!(out, "    MCSErased: {}", i32::from(st.mcs.erased));
            let _ = writeln!(out, "    currentPoint: {current_point}");
        }
        self.asyn_port_driver_report(out, level);
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let mut status: UlError = 0;
        let trace = self.state.lock().unwrap().trace.clone();
        let reason = user.reason;
        let addr = device_addr(user.addr);

        self.base.set_int32_param(reason, addr, value)?;

        if reason == self.params.pulse_run {
            // C starts on any non-zero write, running or not: with a pulse
            // count it cannot tell when the last run ended.
            if value != 0 {
                status = self.start_pulse_generator(&trace, addr);
            } else if self.pulse_running(addr) {
                status = self.stop_pulse_generator(&trace, addr);
            }
        } else if reason == self.params.pulse_count || reason == self.params.pulse_idle_state {
            if self.pulse_running(addr) {
                status = self.stop_pulse_generator(&trace, addr);
                status |= self.start_pulse_generator(&trace, addr);
            }
        } else if reason == self.params.counter_reset {
            let dev = self.device.lock().unwrap();
            if let Err(e) = dev.counter_load(addr, uldaq_sys::CRT_LOAD, 0) {
                status = e.code;
            }
        } else if reason == self.params.trigger_mode {
            // C maps the Windows UL trigger values onto uldaq's, defaulting
            // to TRIG_LOW, and still sets the trigger after the error line.
            let trigger_type = match value {
                0 => uldaq_sys::TRIG_POS_EDGE,
                1 => uldaq_sys::TRIG_NEG_EDGE,
                6 => uldaq_sys::TRIG_HIGH,
                7 => uldaq_sys::TRIG_LOW,
                _ => {
                    trace.print(
                        TraceMask::ERROR,
                        format_args!(
                            "USBCTR::writeInt32 error unsupported trigger value={value}\n"
                        ),
                    );
                    uldaq_sys::TRIG_LOW
                }
            };
            let dev = self.device.lock().unwrap();
            if let Err(e) = dev.daq_in_set_trigger(trigger_type, Default::default(), 0.0, 0.0, 0) {
                status = e.code;
                trace.print(
                    TraceMask::ERROR,
                    format_args!(
                        "USBCTR::writeInt32 error calling cbDaqSetTrigger status={}, error={}\n",
                        e.code,
                        e.message()
                    ),
                );
            }
        } else if reason == self.params.mca_start_acquire {
            let (scaler_running, mcs_running) = {
                let st = self.state.lock().unwrap();
                (st.scaler.running, st.mcs.running)
            };
            let num_channels = self.int(self.params.mca_num_channels, 0);
            let current_point = self.int(self.params.mcs_current_point, 0);
            if scaler_running {
                status = -1;
            } else if mcs_running {
                // Already acquiring.
            } else if current_point >= num_channels {
                // Already complete: toggle MCA_ACQUIRING so the SNL program
                // clears Acquiring.
                self.base.set_int32_param(self.params.mca_acquiring, 0, 1)?;
                self.base.call_param_callbacks(0)?;
                self.base.set_int32_param(self.params.mca_acquiring, 0, 0)?;
            } else {
                self.base.set_int32_param(self.params.mca_acquiring, 0, 1)?;
                let num_channels = num_channels.max(0) as usize;
                let dev = self.device.lock().unwrap();
                let mut st = self.state.lock().unwrap();
                st.mcs.erased = false;
                let dwell = self.float(self.params.mca_dwell_time, 0);
                let ch_adv = self.int(self.params.mca_ch_advance_source, 0);
                let prescale = self.int(self.params.mca_prescale, 0);
                let enable = self.uint(self.params.mcs_counter_enable, 0);
                st.mcs.preset_real_time = self.float(self.params.mca_preset_real, 0);
                let point0_action = self.int(self.params.mcs_point0_action, 0);
                // C `writeInt32` ignores `startMCS`'s status; it prints its own.
                let num_counters = st.num_counters;
                let actual_dwell = mcs::start_mcs(
                    &dev,
                    &mut st.mcs,
                    &McsScan {
                        num_counters,
                        num_points: num_channels,
                        dwell_time: dwell,
                        counter_enable: enable,
                        ch_advance_source: ch_adv,
                        prescale,
                        prescale_counter: self.int(self.params.mcs_prescale_counter, 0),
                        point0_action,
                    },
                    &trace,
                );
                self.base
                    .set_float64_param(self.params.mca_dwell_time, 0, actual_dwell)?;
                // C writes the dwell the clock runs back without recomputing
                // its axis (drvUSBCTR.cpp:711, :873-880). C defect, not copied
                // (upstream-c-defects #230).
                let times = mcs::compute_times(&mut st.mcs, num_channels, actual_dwell);
                let times = ParamValue::Float32Array(times.into());
                self.base
                    .do_callbacks_array(self.params.mcs_time_wf, 0, times)?;
                self.base
                    .set_int32_param(self.params.mcs_current_point, 0, 0)?;
            }
        } else if reason == self.params.mca_stop_acquire {
            let dev = self.device.lock().unwrap();
            let mut st = self.state.lock().unwrap();
            let num_counters = st.num_counters;
            // C stops only a running scan, and leaves it to a running scaler.
            let pass = if st.mcs.running && !st.scaler.running {
                mcs::stop_mcs(&dev, &mut st.mcs, &trace)
            } else {
                None
            };
            for (reason, addr, value) in pass
                .iter()
                .flat_map(|p| p.updates(&self.params, num_counters))
            {
                self.base.set_param_value(reason, addr, value)?;
            }
        } else if reason == self.params.mca_erase {
            let port = &self.base.port_name;
            user.print(
                TraceMask::FLOW,
                file!(),
                line!(),
                format_args!("USBCTR:writeInt32: [{port} addr={addr}]: erased\n"),
            );
            self.erase_mcs()?;
        } else if reason == self.params.mca_num_channels && value > self.max_time_points as i32 {
            let max = self.max_time_points as i32;
            self.base
                .set_int32_param(self.params.mca_num_channels, 0, max)?;
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!("USBCTR:writeInt32:  # channels={value} too large, max={max}\n"),
            );
        }

        self.base.call_param_callbacks(addr)?;
        let port = &self.base.port_name;
        if status == 0 {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("USBCTR:writeInt32, port {port}, function={reason}, wrote {value} to address {addr}\n"),
            );
        } else {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!("USBCTR:writeInt32, port {port}, function={reason}, ERROR writing {value} to address {addr}, status={status}\n"),
            );
        }
        trace::asyn_status(status)
    }

    /// MCS spectrum readout. C `USBCTR::readInt32Array` / the `mcaReadData`
    /// the mca record and SIS38XX_waveform.template both go through: the data
    /// lives in McsState::mcs_buffers and only leaves the driver here.
    fn read_int32_array(&mut self, user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        user.print(
            TraceMask::FLOW,
            file!(),
            line!(),
            format_args!(
                "USBCTR:readInt32Array: entry, command={}, signal={}, numRead={}, &data={:p}\n",
                user.reason,
                user.addr,
                buf.len(),
                buf.as_ptr()
            ),
        );
        if user.reason != self.params.mca_data {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "USBCTR:readInt32Array: got illegal command {}\n",
                    user.reason
                ),
            );
            trace::asyn_status(-1)?;
        }
        let counter = user.addr as usize;
        let num_channels = self.int(self.params.mca_num_channels, 0).max(0) as usize;
        let st = self.state.lock().unwrap();
        let Some(src) = st.mcs.mcs_buffers.get(counter) else {
            return Ok(0);
        };
        // C copies nChans points but reports only the acquired ones, and
        // never 0 so NORD stays non-zero.
        let num_copy = buf.len().min(src.len()).min(num_channels);
        buf[..num_copy].copy_from_slice(&src[..num_copy]);
        let current_point = st.mcs.current_point;
        let num_actual = buf.len().min(current_point).max(1);
        user.print(
            TraceMask::FLOW,
            file!(),
            line!(),
            format_args!(
                "USBCTR:readInt32Array: [signal={counter}]: read {num_actual} chans (numRead={}, numCopy={num_copy}, currentPoint={current_point}, nChans={num_channels})\n",
                buf.len()
            ),
        );
        Ok(num_actual)
    }

    /// MCS time base (seconds from the start of the scan).
    fn read_float32_array(&mut self, user: &AsynUser, buf: &mut [f32]) -> AsynResult<usize> {
        if user.reason != self.params.mcs_time_wf {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "USBCTR:readFloat32Array: ERROR: unknown function={}\n",
                    user.reason
                ),
            );
            trace::asyn_status(-1)?;
        }
        // Only the points the scan is configured for, as C
        // computeMCSTimes' doCallbacksFloat32Array(.., numTimePoints, ..)
        // does -- a full-length time base against a short spectrum is
        // unplottable.
        let num_channels = self.int(self.params.mca_num_channels, 0).max(0) as usize;
        let st = self.state.lock().unwrap();
        let n = buf.len().min(st.mcs.time_buffer.len()).min(num_channels);
        buf[..n].copy_from_slice(&st.mcs.time_buffer[..n]);
        Ok(n)
    }

    /// MCS absolute time base (seconds since the epoch, per acquired point).
    fn read_float64_array(&mut self, user: &AsynUser, buf: &mut [f64]) -> AsynResult<usize> {
        if user.reason != self.params.mcs_abs_time_wf {
            // C's functionName is "readFloat6Array".
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "USBCTR:readFloat6Array: ERROR: unknown function={}\n",
                    user.reason
                ),
            );
            trace::asyn_status(-1)?;
        }
        let num_channels = self.int(self.params.mca_num_channels, 0).max(0) as usize;
        let st = self.state.lock().unwrap();
        let n = buf
            .len()
            .min(st.mcs.abs_time_buffer.len())
            .min(num_channels);
        buf[..n].copy_from_slice(&st.mcs.abs_time_buffer[..n]);
        Ok(n)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let mut status: UlError = 0;
        let trace = self.state.lock().unwrap().trace.clone();
        let reason = user.reason;
        let addr = device_addr(user.addr);
        self.base.set_float64_param(reason, addr, value)?;
        if reason == self.params.mca_preset_real {
            // C readMCS reads the preset on every pass.
            self.state.lock().unwrap().mcs.preset_real_time = value;
        } else if reason == self.params.mca_dwell_time {
            let num_points = self.int(self.params.mca_num_channels, 0);
            let mut st = self.state.lock().unwrap();
            let times = mcs::compute_times(&mut st.mcs, num_points.max(0) as usize, value);
            // computeMCSTimes' doCallbacksFloat32Array (drvUSBCTR.cpp:883).
            let times = ParamValue::Float32Array(times.into());
            drop(st);
            self.base
                .do_callbacks_array(self.params.mcs_time_wf, 0, times)?;
        }

        if (reason == self.params.pulse_period
            || reason == self.params.pulse_duty_cycle
            || reason == self.params.pulse_delay)
            && self.pulse_running(addr)
        {
            status = self.stop_pulse_generator(&trace, addr);
            status |= self.start_pulse_generator(&trace, addr);
        }

        self.base.call_param_callbacks(addr)?;
        let port = &self.base.port_name;
        if status == 0 {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "USBCTR:writeFloat64, port {port}, wrote {value:.6} to address {addr}\n",
                    value = CFloat(value)
                ),
            );
        } else {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!("USBCTR:writeFloat64, port {port}, ERROR writing {value:.6} to address {addr}, status={status}\n", value = CFloat(value)),
            );
        }
        trace::asyn_status(status)
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        let mut status: UlError = 0;
        let reason = user.reason;
        let addr = user.addr;

        if reason == self.params.digital_output {
            // C writes only the masked bits configured as outputs.
            let direction = self.uint(self.params.digital_direction, 0);
            let dev = self.device.lock().unwrap();
            for bit in 0..NUM_IO_BITS {
                if mask & direction & (1 << bit) != 0 {
                    let bit_val = (value >> bit) & 1;
                    status = status_of(&dev.digital_bit_out(
                        uldaq_sys::AUXPORT,
                        bit as i32,
                        bit_val != 0,
                    ));
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
                        status = e.code;
                    }
                }
            }
        }

        self.base.set_uint32_param(reason, addr, value, mask, 0)?;
        // C's last loop pass leaves `outValue` at bit NUM_IO_BITS-1 of `value`.
        let out_value = if reason == self.params.digital_output {
            (value >> (NUM_IO_BITS - 1)) & 1
        } else {
            0
        };
        let direction = self.uint(self.params.digital_direction, 0);

        self.base.call_param_callbacks(addr)?;
        let port = &self.base.port_name;
        if status == 0 {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("USBCTR:writeUInt32Digital, port {port}, wrote outValue=0x{out_value:x}, value=0x{value:x}, mask=0x{mask:x}, direction=0x{direction:x}\n"),
            );
        } else {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!("USBCTR:writeUInt32Digital, port {port}, ERROR writing outValue=0x{out_value:x}, value=0x{value:x}, mask=0x{mask:x}, direction=0x{direction:x}, status={status}\n"),
            );
        }
        trace::asyn_status(status)
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
/// `claim` is the port name taken before the device is opened, as C's
/// `asynPortDriver` base registers it first.
pub fn create_usb_ctr(
    claim: &PortClaim,
    unique_id: &str,
    max_time_points: usize,
) -> Result<CtrRuntime, String> {
    let device = DaqDevice::connect(unique_id)
        .map_err(|e| format!("failed to connect to CTR device: {e}"))?;

    let driver = CtrDriver::new(claim.name(), device, max_time_points)
        .map_err(|e| format!("failed to create CtrDriver: {e}"))?;

    let params = driver.params;
    let device = driver.device.clone();
    let state = driver.state.clone();

    let (runtime_handle, _actor_jh) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the USB-CTR port runtime: {e}"))?;

    state
        .lock()
        .unwrap()
        .trace
        .bind(runtime_handle.port_handle().clone());
    // C puts the pulse generators in a known state before its poller starts.
    {
        let st = state.lock().unwrap();
        let dev = device.lock().unwrap();
        for timer in 0..NUM_TIMERS as i32 {
            let _ = pulse_gen::stop(&st.trace, &dev, timer);
        }
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_port_carries_cs_interface_and_interrupt_masks() {
        use InterfaceMask as I;
        let drv = CtrDriver::new("CTRMASKS", DaqDevice::unconnected(), 16).unwrap();
        let interrupts = I::INT32
            | I::UINT32_DIGITAL
            | I::INT32_ARRAY
            | I::FLOAT32_ARRAY
            | I::FLOAT64
            | I::FLOAT64_ARRAY
            | I::OCTET;
        assert_eq!(drv.base.interrupt_mask(), interrupts);
        assert_eq!(
            drv.base.interface_mask(),
            interrupts | I::DRV_USER | I::COMMON
        );
    }

    /// A dwell-time write runs computeMCSTimes, whose
    /// doCallbacksFloat32Array posts the time base at once, typed and
    /// unstored, before writeFloat64's callParamCallbacks posts the dwell
    /// (drvUSBCTR.cpp:873-884, :1302-1304).
    #[test]
    fn a_dwell_write_posts_the_time_base_as_an_array_callback() {
        use epics_rs::asyn::interfaces::InterfaceType;
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;

        let mut drv = CtrDriver::new("CTRDWELL", DaqDevice::unconnected(), 16).unwrap();
        let (dwell, times) = (drv.params.mca_dwell_time, drv.params.mcs_time_wf);
        drv.base
            .set_int32_param(drv.params.mca_num_channels, 0, 4)
            .unwrap();
        drv.base.own_interrupts(InterruptGate::opened());
        drv.base.call_param_callbacks(0).unwrap();
        let mut rx = drv.base.interrupts.subscribe_async(256, 4096);

        drv.write_float64(&mut AsynUser::new(dwell), 0.5).unwrap();
        let posted: Vec<_> = std::iter::from_fn(|| rx.try_recv().ok())
            .filter(|v| v.reason == dwell || v.reason == times)
            .map(|v| match v.value {
                ParamValue::Float32Array(a) => format!("times {:?} {:?}", &a[..], v.iface),
                ParamValue::Float64(x) => format!("dwell {x}"),
                other => format!("{other:?}"),
            })
            .collect();
        assert_eq!(
            posted,
            [
                format!(
                    "times {:?} {:?}",
                    [0.0f32, 0.5, 1.0, 1.5],
                    Some(InterfaceType::Float32Array)
                ),
                "dwell 0.5".to_string(),
            ]
        );
        assert!(
            drv.base
                .params
                .get_float32_array(times, 0)
                .unwrap()
                .is_empty(),
            "the time base was stored"
        );
    }

    /// The start writes back the dwell ulDaqInScan runs and posts the axis
    /// of that dwell (upstream-c-defects #230); on the null handle the scan
    /// fails and the dwell stays the one asked for.
    #[test]
    fn an_mcs_start_posts_the_time_base_of_the_dwell_it_runs() {
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;

        let mut drv = CtrDriver::new("CTRSTART", DaqDevice::unconnected(), 16).unwrap();
        let p = drv.params;
        drv.base.set_int32_param(p.mca_num_channels, 0, 4).unwrap();
        drv.base
            .set_float64_param(p.mca_dwell_time, 0, 0.25)
            .unwrap();
        drv.base.own_interrupts(InterruptGate::opened());
        drv.base.call_param_callbacks(0).unwrap();
        let mut rx = drv.base.interrupts.subscribe_async(256, 4096);

        let _ = drv.write_int32(&mut AsynUser::new(p.mca_start_acquire), 1);
        let axes: Vec<_> = std::iter::from_fn(|| rx.try_recv().ok())
            .filter(|v| v.reason == p.mcs_time_wf)
            .map(|v| match v.value {
                ParamValue::Float32Array(a) => a.to_vec(),
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(axes, [vec![0.0f32, 0.25, 0.5, 0.75]]);
    }

    #[test]
    fn a_direction_write_calls_config_bit_unprobed() {
        let mut drv = CtrDriver::new("CTRDIR", DaqDevice::unconnected(), 16).unwrap();
        let mut user = AsynUser::new(drv.params.digital_direction);
        // ulDConfigBit fails on the null handle.
        assert!(drv.write_uint32_digital(&mut user, 1, 0x01).is_err());
    }
}
