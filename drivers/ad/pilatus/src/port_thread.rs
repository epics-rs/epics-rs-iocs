//! The camserver exchanges C runs on the asyn port thread, inside
//! `writeInt32` / `writeFloat64` / `writeOctet`. They run inline here too, on
//! the port's own thread, so a write queued behind one lands only after it.

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::trace::TraceMask;

use crate::camserver::CamParams;
use crate::driver::PilatusDriver;
use crate::protocol::{
    cmd_delay, cmd_expperiod, cmd_exptime, cmd_gapfill, cmd_nexpframe, cmd_nimages,
    cmd_reset_module_power, cmd_set_threshold, gain_index, parse_energy_setting, parse_tau_cutoff,
    parse_threshold_ev, parse_version, thread_channel_values,
};
use crate::types::{CAMSERVER_DEFAULT_TIMEOUT, CAMSERVER_RESET_POWER_TIMEOUT, TriggerMode};

impl PilatusDriver {
    /// C `getIntegerParam`; `None` while the parameter is undefined.
    fn get_i32(&self, reason: usize) -> Option<i32> {
        self.ad.port_base.get_int32_param(reason, 0).ok()
    }

    /// C `getDoubleParam`; `None` while the parameter is undefined.
    fn get_f64(&self, reason: usize) -> Option<f64> {
        self.ad.port_base.get_float64_param(reason, 0).ok()
    }

    fn set_i32(&mut self, reason: usize, value: i32) {
        self.ad.port_base.set_i32(reason, value);
    }

    fn set_f64(&mut self, reason: usize, value: f64) {
        let _ = self.ad.port_base.set_float64_param(reason, 0, value);
    }

    fn set_str(&mut self, reason: usize, value: impl Into<String>) {
        self.ad.port_base.set_str(reason, value.into());
    }

    /// C `writeReadCamserver`.
    pub(crate) fn cam_write_read(&mut self, cmd: &str, timeout: f64) -> AsynStatus {
        self.cam.write_read(&mut self.ad.port_base, cmd, timeout)
    }

    /// C `setAcquireParams()`.
    pub(crate) fn set_acquire_params(&mut self) {
        let (ad, p) = (self.ad.params, self.p);
        let trigger_mode = self
            .get_i32(ad.trigger_mode)
            .unwrap_or(TriggerMode::Internal as i32);
        // When we change modes download all exposure parameters, since some modes
        // replace values with new parameters.
        if trigger_mode == TriggerMode::Alignment as i32 {
            self.set_i32(ad.num_images, 1);
        }

        let mut ival = self.get_i32(ad.num_images).unwrap_or(0);
        if ival < 1 {
            ival = 1;
            self.set_i32(ad.num_images, ival);
        }
        self.cam_write_read(&cmd_nimages(ival), CAMSERVER_DEFAULT_TIMEOUT);

        let mut ival = self.get_i32(ad.num_exposures).unwrap_or(0);
        if ival < 1 {
            ival = 1;
            self.set_i32(ad.num_exposures, ival);
        }
        self.cam_write_read(&cmd_nexpframe(ival), CAMSERVER_DEFAULT_TIMEOUT);

        let mut dval = self.get_f64(ad.acquire_time).unwrap_or(-1.0);
        if dval < 0.0 {
            dval = 1.0;
            self.set_f64(ad.acquire_time, dval);
        }
        self.cam_write_read(&cmd_exptime(dval), CAMSERVER_DEFAULT_TIMEOUT);

        let mut dval = self.get_f64(ad.acquire_period).unwrap_or(-1.0);
        if dval < 0.0 {
            dval = 2.0;
            self.set_f64(ad.acquire_period, dval);
        }
        self.cam_write_read(&cmd_expperiod(dval), CAMSERVER_DEFAULT_TIMEOUT);

        let mut dval = self.get_f64(p.delay_time).unwrap_or(-1.0);
        if dval < 0.0 {
            dval = 0.0;
            self.set_f64(p.delay_time, dval);
        }
        self.cam_write_read(&cmd_delay(dval), CAMSERVER_DEFAULT_TIMEOUT);

        let mut ival = self.get_i32(p.gap_fill).unwrap_or(-2);
        if !(-2..=0).contains(&ival) {
            ival = -2;
            self.set_i32(p.gap_fill, ival);
        }
        // -2 means GapFill is not supported (single-element detector).
        if ival != -2 {
            self.cam_write_read(&cmd_gapfill(ival), CAMSERVER_DEFAULT_TIMEOUT);
        }

        // Read back the pixel count-rate cut-off.
        let status = self.cam_write_read("Tau", 5.0);
        if status == AsynStatus::Success
            && let Some(cutoff) = parse_tau_cutoff(self.cam.from())
        {
            self.set_i32(p.pixel_cutoff, cutoff);
        }
    }

    /// C `setThreshold()`.
    pub(crate) fn set_threshold(&mut self) {
        let (ad, p) = (self.ad.params, self.p);
        let dgain = self.get_f64(ad.gain).unwrap_or(0.0);
        let igain = gain_index(dgain);
        let (threshold, mut energy) = {
            let s = self.shared.lock();
            (s.demanded_threshold, s.demanded_energy)
        };
        if energy == 0.0 {
            energy = threshold * 2.0;
        }
        let cmd = cmd_set_threshold(energy, igain, threshold);

        // Set the status to waiting so we can be notified when it has finished.
        self.set_i32(ad.status, ADStatus::Waiting as i32);
        self.set_str(ad.status_message, "Setting threshold");
        let _ = self.ad.port_base.call_param_callbacks(0);

        // This command can take 96 seconds on a 6M.
        let status = self.cam_write_read(&cmd, 110.0);
        let new_status = if status != AsynStatus::Success {
            ADStatus::Error
        } else {
            ADStatus::Idle
        };
        self.set_i32(ad.status, new_status as i32);
        self.set_i32(p.threshold_apply, 0);

        // Read back the actual threshold setting, in case we are out of bounds.
        let status = self.cam_write_read("SetThreshold", 5.0);
        if status == AsynStatus::Success
            && let Some(ev) = parse_threshold_ev(self.cam.from())
        {
            self.set_f64(p.threshold, ev as f64 / 1000.0);
        }

        // Read back the actual energy setting.
        let status = self.cam_write_read("SetEnergy", 5.0);
        if status == AsynStatus::Success {
            let ev = parse_energy_setting(self.cam.from());
            self.set_f64(p.energy, ev as f64 / 1000.0);
        }

        // SetThreshold resets nimages and gapfill, so re-send the acquisition
        // parameters.
        self.set_acquire_params();
        let _ = self.ad.port_base.call_param_callbacks(0);
    }

    /// C `resetModulePower()`.
    pub(crate) fn reset_module_power(&mut self) {
        let (major, minor, patch) = self.version;
        // This command only exists on camserver 7.9.0 and higher.
        if major < 7 || (major == 7 && minor < 9) {
            self.ad.port_base.trace(
                TraceMask::ERROR,
                &format!(
                    "pilatusDetector::resetModulePower ResetModulePower not supported on \
                     version {major}.{minor}.{patch} of camserver\n"
                ),
            );
            return;
        }
        let reason = self.ad.params.status_message;
        self.set_str(reason, "Resetting module power");
        let _ = self.ad.port_base.call_param_callbacks(0);
        let reset_time = self.get_i32(self.p.reset_power_time).unwrap_or(0);
        self.cam_write_read(
            &cmd_reset_module_power(reset_time),
            CAMSERVER_RESET_POWER_TIMEOUT + reset_time as f64,
        );
        // The threshold must be set after resetting module power.
        self.set_threshold();
    }

    /// C `pilatusStatus()`.
    pub(crate) fn pilatus_status(&mut self) {
        let (ad, p) = (self.ad.params, self.p);
        if self.first_status_call {
            let status = self.cam_write_read("version", 1.0);
            if status == AsynStatus::Success {
                if let Some(v) = parse_version(self.cam.from()) {
                    self.set_str(p.tvx_version, v.version_string.clone());
                    self.set_str(ad.base.sdk_version, v.version_string);
                    if let Some(x) = v.major {
                        self.version.0 = x;
                    }
                    if let Some(x) = v.minor {
                        self.version.1 = x;
                    }
                    if let Some(x) = v.patch {
                        self.version.2 = x;
                    }
                }
                self.set_i32(ad.status, ADStatus::Idle as i32);
            } else {
                self.set_i32(ad.status, ADStatus::Error as i32);
            }
            self.first_status_call = false;
        }

        let status = self.cam_write_read("thread", 1.0);
        if status == AsynStatus::Success {
            // Fixed (upstream-c-defects.md #12): resolve each channel independently
            // so a partial reply for one channel never leaks another channel's
            // value (C shared one `temp` / `humid` pair across all channels).
            let reply = self.cam.from().to_string();

            if let Some((temp, humid)) = thread_channel_values(&reply, 0) {
                self.set_f64(p.th_temp_0, temp);
                self.set_f64(p.th_humid_0, humid);
                self.set_f64(ad.temperature, temp);
            }
            if let Some((temp, humid)) = thread_channel_values(&reply, 1) {
                self.set_f64(p.th_temp_1, temp);
                self.set_f64(p.th_humid_1, humid);
            }
            if let Some((temp, humid)) = thread_channel_values(&reply, 2) {
                self.set_f64(p.th_temp_2, temp);
                self.set_f64(p.th_humid_2, humid);
            }
            // Channel 3 handling removed as fixed defect #10 — the `thread` reply
            // defines only channels 0-2 and there is no channel-3 parameter.
        } else {
            self.set_i32(ad.status, ADStatus::Error as i32);
        }
        let _ = self.ad.port_base.call_param_callbacks(0);
    }
}
