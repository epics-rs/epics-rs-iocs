//! The Merlin areaDetector driver (C `merlinDetector`).
//!
//! Threading invariant, and the reason the C locking comments do not carry
//! over: **the command channel is owned by the port actor**. Every `SET`/
//! `GET`/`CMD` runs inside `write_int32`/`write_float64`/the init handler, so
//! command transactions are serialised by construction and can never
//! interleave. C removed the driver lock around `mpxWriteRead` ("I do not
//! believe you can nest locks"), leaving two threads free to interleave a
//! write with another's read on the same socket. The data channel is a second
//! socket, owned exclusively by the acquisition task.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};

use crate::connection::{MpxConnection, default_timeout};
use crate::params::MerlinParams;
use crate::task::{AcquisitionContext, start_acquisition_task};
use crate::types::*;

/// State the acquisition task and the port actor both touch.
///
/// C kept `imagesRemaining` as a plain `int` written by `writeInt32` and
/// decremented by the data thread, with no synchronisation.
pub struct SharedState {
    /// Set until the status task has pushed the autosaved settings; the data
    /// task must not run before then (C `startingUp`).
    pub starting_up: AtomicBool,
    /// Frames still expected in this acquisition; -1 means "continuous".
    pub images_remaining: AtomicI32,
    /// Frames the device emits per acquire in the current Quad mode.
    pub frames_per_acquire: AtomicI32,
}

impl SharedState {
    fn new() -> Self {
        Self {
            starting_up: AtomicBool::new(true),
            images_remaining: AtomicI32::new(0),
            frames_per_acquire: AtomicI32::new(1),
        }
    }
}

pub struct MerlinDetector {
    pub ad: ADDriverBase,
    pub params: MerlinParams,
    /// Internal trigger the status task uses to run the startup sequence on
    /// the actor thread (see [`MerlinDetector::initialise`]). Not bound to any
    /// record.
    init_param: usize,
    det_type: DetectorType,
    cmd: MpxConnection,
    shared: Arc<SharedState>,
}

impl MerlinDetector {
    #[allow(clippy::too_many_arguments)]
    fn new(
        port_name: &str,
        cmd: MpxConnection,
        max_size_x: i32,
        max_size_y: i32,
        det_type: DetectorType,
        max_memory: usize,
        shared: Arc<SharedState>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, max_size_x, max_size_y, max_memory)?;
        let params = MerlinParams::create(&mut ad.port_base)?;
        let init_param = ad
            .port_base
            .create_param("MERLIN_INIT", epics_rs::asyn::param::ParamType::Int32)?;

        let base = &mut ad.port_base;
        base.set_string_param(
            ad.params.base.manufacturer,
            0,
            det_type.manufacturer().into(),
        )?;
        base.set_string_param(ad.params.base.model, 0, det_type.model().into())?;
        base.set_string_param(
            ad.params.base.driver_version,
            0,
            env!("CARGO_PKG_VERSION").into(),
        )?;
        base.set_string_param(params.select_gui, 0, det_type.gui().into())?;

        base.set_int32_param(ad.params.max_size_x, 0, max_size_x)?;
        base.set_int32_param(ad.params.max_size_y, 0, max_size_y)?;
        base.set_int32_param(ad.params.size_x, 0, max_size_x)?;
        base.set_int32_param(ad.params.size_y, 0, max_size_y)?;
        base.set_int32_param(ad.params.base.array_size_x, 0, max_size_x)?;
        base.set_int32_param(ad.params.base.array_size_y, 0, max_size_y)?;
        base.set_int32_param(ad.params.base.array_size, 0, 0)?;
        base.set_int32_param(
            ad.params.base.data_type,
            0,
            epics_rs::ad_core::ndarray::NDDataType::UInt32 as u8 as i32,
        )?;
        base.set_int32_param(ad.params.image_mode, 0, MerlinImageMode::Continuous as i32)?;
        base.set_int32_param(ad.params.trigger_mode, 0, TriggerMode::Internal as i32)?;
        base.set_int32_param(params.profile_control, 0, MPXPROFILES_IMAGE)?;
        base.set_int32_param(params.counter_depth, 0, 12)?;
        base.set_int32_param(ad.params.status, 0, ADStatus::Initializing as i32)?;

        Ok(Self {
            ad,
            params,
            init_param,
            det_type,
            cmd,
            shared,
        })
    }

    fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    fn get_f64(&self, reason: usize) -> f64 {
        self.ad
            .port_base
            .get_float64_param(reason, 0)
            .unwrap_or(0.0)
    }

    /// Startup sequence, run once on the actor thread after the IOC has
    /// finished loading autosaved settings (C `merlinStatus`).
    fn initialise(&mut self) -> AsynResult<()> {
        self.shared.starting_up.store(false, Ordering::Release);

        self.set_acquire_params()?;
        self.set_roi()?;
        self.update_threshold_scan_params()?;
        self.get_threshold()?;

        // C read SOFTWAREVERSION into a local and threw it away.
        match self.cmd.get(MPXVAR_GETSOFTWAREVERSION, default_timeout()) {
            Ok(version) => {
                let base = &mut self.ad.port_base;
                base.set_string_param(self.ad.params.base.sdk_version, 0, version)?;
            }
            Err(e) => log::error!("merlin: could not read the Labview software version: {e}"),
        }

        let base = &mut self.ad.port_base;
        base.set_int32_param(self.ad.params.status, 0, ADStatus::Idle as i32)?;
        base.set_string_param(
            self.ad.params.status_message,
            0,
            "Waiting for acquire command".into(),
        )?;
        Ok(())
    }

    /// Push the acquisition settings the device keeps as separate variables
    /// (C `setAcquireParams`).
    fn set_acquire_params(&mut self) -> AsynResult<()> {
        if self.shared.starting_up.load(Ordering::Acquire) {
            return Ok(());
        }
        let timeout = default_timeout();

        if matches!(
            self.det_type,
            DetectorType::MerlinXbpm | DetectorType::UomXbpm
        ) {
            let exposures = self.get_i32(self.ad.params.num_exposures);
            self.set_or_log(MPXVAR_IMAGESTOSUM, &exposures.to_string());
            let bkg = self.get_i32(self.params.enable_background_corr);
            self.set_or_log(MPXVAR_ENABLEBACKROUNDCORR, &bkg.to_string());
            let sum = self.get_i32(self.params.enable_image_sum);
            self.set_or_log(MPXVAR_ENABLEIMAGEAVERAGE, &sum.to_string());
        }

        // Clamp the values the device will reject, writing the clamp back so
        // the readback records agree with what was sent.
        let mut num_images = self.get_i32(self.ad.params.num_images);
        if num_images < 1 {
            num_images = 1;
            self.ad
                .port_base
                .set_int32_param(self.ad.params.num_images, 0, num_images)?;
        }
        let mut num_exposures = self.get_i32(self.ad.params.num_exposures);
        if num_exposures < 1 {
            num_exposures = 1;
            self.ad
                .port_base
                .set_int32_param(self.ad.params.num_exposures, 0, num_exposures)?;
        }
        let mut counter_depth = self.get_i32(self.params.counter_depth);
        if !matches!(counter_depth, 6 | 12 | 24) {
            counter_depth = 12;
            self.ad
                .port_base
                .set_int32_param(self.params.counter_depth, 0, counter_depth)?;
        }
        let mut acquire_time = self.get_f64(self.ad.params.acquire_time);
        if acquire_time < 0.0 {
            acquire_time = 1.0;
            self.ad
                .port_base
                .set_float64_param(self.ad.params.acquire_time, 0, acquire_time)?;
        }
        let mut acquire_period = self.get_f64(self.ad.params.acquire_period);
        if acquire_period < 0.0 {
            acquire_period = 1.0;
            self.ad.port_base.set_float64_param(
                self.ad.params.acquire_period,
                0,
                acquire_period,
            )?;
        }
        self.ad.port_base.call_param_callbacks(0)?;

        self.set_or_log(MPXVAR_NUMFRAMESPERTRIGGER, &num_exposures.to_string());
        self.set_or_log(MPXVAR_COUNTERDEPTH, &counter_depth.to_string());
        // The device takes milliseconds.
        self.set_or_log(
            MPXVAR_ACQUISITIONTIME,
            &format!("{:.6}", acquire_time * 1000.0),
        );
        self.set_or_log(
            MPXVAR_ACQUISITIONPERIOD,
            &format!("{:.6}", acquire_period * 1000.0),
        );

        let trigger = TriggerMode::from_i32(self.get_i32(self.ad.params.trigger_mode))
            .unwrap_or(TriggerMode::Internal);
        let (start, stop) = match trigger {
            TriggerMode::Internal => (TM_TRIG_INTERNAL, Some(TM_TRIG_INTERNAL)),
            TriggerMode::ExternalEnable => (TM_TRIG_RISING, Some(TM_TRIG_FALLING)),
            TriggerMode::ExternalTriggerLow => (TM_TRIG_FALLING, Some(TM_TRIG_INTERNAL)),
            TriggerMode::ExternalTriggerHigh => (TM_TRIG_RISING, Some(TM_TRIG_INTERNAL)),
            TriggerMode::ExternalTriggerRising => (TM_TRIG_RISING, Some(TM_TRIG_RISING)),
            // C sets no stop trigger for software triggering.
            TriggerMode::SoftwareTrigger => (TM_TRIG_SOFTWARE, None),
        };
        self.set_or_log(MPXVAR_TRIGGERSTART, start);
        if let Some(stop) = stop {
            self.set_or_log(MPXVAR_TRIGGERSTOP, stop);
        }

        // The server may lengthen the period to fit the readout time; take its
        // value back. C tested a stale `status` here instead of this GET's.
        if let Some(period) = self.cmd.get_f64(MPXVAR_ACQUISITIONPERIOD, timeout) {
            self.ad.port_base.set_float64_param(
                self.ad.params.acquire_period,
                0,
                period / 1000.0,
            )?;
        }
        Ok(())
    }

    /// Only the Manchester XBPM supports a hardware ROI (C `setROI`).
    fn set_roi(&mut self) -> AsynResult<()> {
        if self.det_type != DetectorType::UomXbpm {
            return Ok(());
        }
        let max_x = self.get_i32(self.ad.params.max_size_x);
        let max_y = self.get_i32(self.ad.params.max_size_y);
        let max = [max_x, max_y];

        let mut offset = [
            self.get_i32(self.ad.params.min_x),
            self.get_i32(self.ad.params.min_y),
        ];
        let mut size = [
            self.get_i32(self.ad.params.size_x),
            self.get_i32(self.ad.params.size_y),
        ];
        let mut binning = [
            self.get_i32(self.ad.params.bin_x),
            self.get_i32(self.ad.params.bin_y),
        ];

        for d in 0..2 {
            offset[d] = offset[d].clamp(0, (max[d] - 1).max(0));
            size[d] = size[d].clamp(1, (max[d] - offset[d]).max(1));
            binning[d] = binning[d].clamp(1, size[d]);
        }

        let base = &mut self.ad.port_base;
        base.set_int32_param(self.ad.params.min_x, 0, offset[0])?;
        base.set_int32_param(self.ad.params.min_y, 0, offset[1])?;
        base.set_int32_param(self.ad.params.size_x, 0, size[0])?;
        base.set_int32_param(self.ad.params.size_y, 0, size[1])?;
        base.set_int32_param(self.ad.params.bin_x, 0, binning[0])?;
        base.set_int32_param(self.ad.params.bin_y, 0, binning[1])?;

        let value = format!("{} {} {} {}", offset[0], offset[1], size[0], size[1]);
        self.set_or_log(MPXVAR_ROI, &value);
        Ok(())
    }

    /// Read every threshold and the operating energy back from the device
    /// (C `getThreshold`).
    fn get_threshold(&mut self) -> AsynResult<()> {
        if self.shared.starting_up.load(Ordering::Acquire) {
            return Ok(());
        }
        let timeout = default_timeout();
        for (i, name) in MPXVAR_THRESHOLD.iter().enumerate() {
            if let Some(v) = self.cmd.get_f64(name, timeout) {
                self.ad
                    .port_base
                    .set_float64_param(self.params.thresholds[i], 0, v)?;
            }
        }
        if let Some(v) = self.cmd.get_f64(MPXVAR_OPERATINGENERGY, timeout) {
            self.ad
                .port_base
                .set_float64_param(self.params.operating_energy, 0, v)?;
        }
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    /// Push the threshold-scan window, then read back what the device accepted
    /// (C `updateThresholdScanParms`).
    fn update_threshold_scan_params(&mut self) -> AsynResult<()> {
        if self.shared.starting_up.load(Ordering::Acquire) {
            return Ok(());
        }
        let timeout = default_timeout();
        let start = self.get_f64(self.params.start_threshold_scan);
        let stop = self.get_f64(self.params.stop_threshold_scan);
        let step = self.get_f64(self.params.step_threshold_scan);
        let scan = self.get_i32(self.params.threshold_scan);

        self.set_or_log(MPXVAR_THSTART, &format!("{start:.6}"));
        self.set_or_log(MPXVAR_THSTOP, &format!("{stop:.6}"));
        self.set_or_log(MPXVAR_THSTEP, &format!("{step:.6}"));
        self.set_or_log(MPXVAR_THSSCAN, &scan.to_string());

        for (name, reason) in [
            (MPXVAR_THSTART, self.params.start_threshold_scan),
            (MPXVAR_THSTEP, self.params.step_threshold_scan),
            (MPXVAR_THSTOP, self.params.stop_threshold_scan),
        ] {
            if let Some(v) = self.cmd.get_f64(name, timeout) {
                self.ad.port_base.set_float64_param(reason, 0, v)?;
            }
        }
        if let Some(v) = self.cmd.get_i32(MPXVAR_THSSCAN, timeout) {
            self.ad
                .port_base
                .set_int32_param(self.params.threshold_scan, 0, v)?;
        }
        Ok(())
    }

    /// The device allows only one of counter-1 and continuous read/write at a
    /// time, so both are read back after either changes (C `setModeCommands`).
    fn set_mode_commands(&mut self, reason: usize) -> AsynResult<()> {
        let timeout = default_timeout();
        if reason == self.params.enable_counter1 {
            let mut v = self.get_i32(self.params.enable_counter1);
            if !(0..=1).contains(&v) {
                v = 0;
                self.ad
                    .port_base
                    .set_int32_param(self.params.enable_counter1, 0, v)?;
            }
            self.set_or_log(MPXVAR_ENABLECOUNTER1, &v.to_string());
        }
        if reason == self.params.continuous_rw {
            let mut v = self.get_i32(self.params.continuous_rw);
            if !(0..=1).contains(&v) {
                v = 0;
                self.ad
                    .port_base
                    .set_int32_param(self.params.continuous_rw, 0, v)?;
            }
            self.set_or_log(MPXVAR_CONTINUOUSRW, &v.to_string());
        }

        if let Some(v) = self.cmd.get_i32(MPXVAR_ENABLECOUNTER1, timeout) {
            self.ad
                .port_base
                .set_int32_param(self.params.enable_counter1, 0, v)?;
        }
        if let Some(v) = self.cmd.get_i32(MPXVAR_CONTINUOUSRW, timeout) {
            self.ad
                .port_base
                .set_int32_param(self.params.continuous_rw, 0, v)?;
        }
        Ok(())
    }

    /// Apply one of the six canned Quad modes (C `SetQuadMode`).
    fn set_quad_mode(&mut self, mode: i32) -> AsynResult<()> {
        let Some(s) = quad_mode_settings(mode) else {
            log::error!("merlin: unknown Quad mode {mode}");
            return Ok(());
        };
        self.shared
            .frames_per_acquire
            .store(s.frames_per_acquire, Ordering::Release);
        self.ad
            .port_base
            .set_int32_param(self.params.counter_depth, 0, s.counter_depth)?;

        self.set_or_log(MPXVAR_COUNTERDEPTH, &s.counter_depth.to_string());
        self.set_or_log(MPXVAR_ENABLECOUNTER1, &s.enable_counter1.to_string());
        self.set_or_log(MPXVAR_CONTINUOUSRW, &s.continuous_rw.to_string());
        self.set_or_log(MPXVAR_COLOURMODE, &s.colour_mode.to_string());
        self.set_or_log(MPXVAR_CHARGESUMMING, &s.charge_summing.to_string());
        Ok(())
    }

    /// Start (or stop) an acquisition (C `writeInt32`, ADAcquire branch).
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        let timeout = default_timeout();
        let status = self.get_i32(self.ad.params.status);
        let idle = status == ADStatus::Idle as i32 || status == ADStatus::Error as i32;

        if value != 0 && idle {
            let base = &mut self.ad.port_base;
            base.set_int32_param(self.ad.params.status, 0, ADStatus::Acquire as i32)?;
            base.set_string_param(self.ad.params.status_message, 0, "Acquiring...".into())?;
            base.set_int32_param(self.ad.params.num_images_counter, 0, 0)?;

            let mut images_to_acquire = self.get_i32(self.ad.params.num_images);
            let image_mode = MerlinImageMode::from_i32(self.get_i32(self.ad.params.image_mode))
                .unwrap_or(MerlinImageMode::Continuous);
            let profile_mask = self.get_i32(self.params.profile_control);
            let per_acquire = self.shared.frames_per_acquire.load(Ordering::Acquire);

            let remaining = match image_mode {
                MerlinImageMode::Single => per_acquire,
                MerlinImageMode::Multiple => images_to_acquire * per_acquire,
                MerlinImageMode::Continuous => {
                    images_to_acquire = 0;
                    -1
                }
                MerlinImageMode::ThresholdScan => {
                    let start = self.get_f64(self.params.start_threshold_scan);
                    let stop = self.get_f64(self.params.stop_threshold_scan);
                    let step = self.get_f64(self.params.step_threshold_scan);
                    let steps = if step != 0.0 {
                        ((stop - start) / step) as i32
                    } else {
                        0
                    };
                    let base = &mut self.ad.port_base;
                    base.set_string_param(
                        self.ad.params.status_message,
                        0,
                        "Performing Threshold Scan...".into(),
                    )?;
                    // The device forces one frame per step; match the PV to it.
                    base.set_int32_param(self.ad.params.num_images, 0, 1)?;
                    steps
                }
                // The server returns one summed image for the whole background
                // acquisition.
                MerlinImageMode::BackgroundCalibrate => 1,
            };
            self.shared
                .images_remaining
                .store(remaining, Ordering::Release);

            match image_mode {
                MerlinImageMode::ThresholdScan => {
                    if let Err(e) = self.cmd.command(MPXCMD_THSCAN, timeout) {
                        log::error!("merlin: THSCAN failed: {e}");
                    }
                }
                MerlinImageMode::BackgroundCalibrate => {
                    self.set_or_log(MPXVAR_BACKGROUNDCOUNT, &images_to_acquire.to_string());
                    if let Err(e) = self.cmd.command(MPXCMD_BACKGROUNDACQUIRE, timeout) {
                        log::error!("merlin: BCKGRND failed: {e}");
                    }
                }
                _ => {
                    self.set_or_log(MPXVAR_NUMFRAMESTOACQUIRE, &images_to_acquire.to_string());
                    let cmd = if profile_mask & MPXPROFILES_IMAGE != 0 {
                        MPXCMD_STARTACQUISITION
                    } else {
                        MPXCMD_PROFILES
                    };
                    if let Err(e) = self.cmd.command(cmd, timeout) {
                        log::error!("merlin: {cmd} failed: {e}");
                    }
                }
            }
        } else if value == 0 && status == ADStatus::Acquire as i32 {
            self.ad
                .port_base
                .set_int32_param(self.ad.params.status, 0, ADStatus::Idle as i32)?;
            if let Err(e) = self.cmd.command(MPXCMD_STOPACQUISITION, timeout) {
                log::error!("merlin: STOPACQUISITION failed: {e}");
            }
        }
        Ok(())
    }

    fn set_or_log(&self, name: &str, value: &str) {
        if let Err(e) = self.cmd.set(name, value, default_timeout()) {
            log::error!("merlin: SET {name} {value} failed: {e}");
        }
    }
}

impl PortDriver for MerlinDetector {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .params
            .set_int32(reason, user.addr, value)?;

        if reason == self.init_param {
            self.initialise()?;
        } else if reason == self.params.reset {
            if let Err(e) = self.cmd.command(MPXCMD_RESET, default_timeout()) {
                log::error!("merlin: RESET failed: {e}");
            }
            // The Labview server cannot be reconnected to after a reset, so C
            // exits the IOC and relies on a supervisor to restart it. Keeping
            // the IOC up would leave every channel dead with no way back.
            log::error!("merlin: RESET issued — exiting so the IOC can be restarted");
            self.ad.port_base.call_param_callbacks(0)?;
            std::process::exit(0);
        } else if reason == self.params.quad_merlin_mode {
            self.set_quad_mode(value)?;
        } else if reason == self.params.software_trigger {
            if let Err(e) = self.cmd.command(MPXCMD_SOFTWARETRIGGER, default_timeout()) {
                log::error!("merlin: SWTRIGGER failed: {e}");
            }
        } else if reason == self.ad.params.acquire {
            self.set_acquire(value)?;
        } else if reason == self.ad.params.trigger_mode
            || reason == self.ad.params.num_images
            || reason == self.ad.params.num_exposures
            || reason == self.params.counter_depth
            || reason == self.params.enable_background_corr
            || reason == self.params.enable_image_sum
        {
            self.set_acquire_params()?;
        } else if reason == self.ad.params.size_x
            || reason == self.ad.params.size_y
            || reason == self.ad.params.min_x
            || reason == self.ad.params.min_y
        {
            self.set_roi()?;
        } else if reason == self.params.enable_counter1 || reason == self.params.continuous_rw {
            self.set_mode_commands(reason)?;
        } else if reason == self.params.threshold_apply {
            self.get_threshold()?;
        } else if reason == self.params.profile_control {
            self.set_or_log(MPXCMD_PROFILES, &value.to_string());
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .params
            .set_float64(reason, user.addr, value)?;

        if let Some(i) = self.params.thresholds.iter().position(|p| *p == reason) {
            self.set_or_log(MPXVAR_THRESHOLD[i], &format!("{value:.6}"));
            self.get_threshold()?;
        } else if reason == self.params.operating_energy {
            self.set_or_log(MPXVAR_OPERATINGENERGY, &format!("{value:.6}"));
            self.get_threshold()?;
        } else if reason == self.ad.params.acquire_time || reason == self.ad.params.acquire_period {
            self.set_acquire_params()?;
        } else if reason == self.params.start_threshold_scan
            || reason == self.params.stop_threshold_scan
            || reason == self.params.step_threshold_scan
        {
            self.update_threshold_scan_params()?;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }
}

impl ADDriver for MerlinDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

/// Everything the IOC layer needs to keep the detector alive and wire plugins
/// to it.
pub struct MerlinRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub params: MerlinParams,
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    #[allow(dead_code)]
    data_task: std::thread::JoinHandle<()>,
    #[allow(dead_code)]
    status_task: std::thread::JoinHandle<()>,
}

impl MerlinRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.array_output.lock().add(sender);
    }
}

/// Create the detector port and start its two tasks.
///
/// `cmd_handle` and `data_handle` are the two `drvAsynIPPort`s the startup
/// script created — the command socket and the data socket.
#[allow(clippy::too_many_arguments)]
pub fn create_merlin_detector(
    port_name: &str,
    cmd_handle: PortHandle,
    data_handle: PortHandle,
    max_size_x: i32,
    max_size_y: i32,
    det_type: DetectorType,
    max_memory: usize,
) -> AsynResult<MerlinRuntime> {
    let shared = Arc::new(SharedState::new());
    let cmd = MpxConnection::new(cmd_handle, MPX_MAXLINE);
    let data = MpxConnection::new(data_handle, det_type.max_frame_bytes());

    // C sends this from the constructor, before the parameters exist, so a
    // restarted IOC never inherits a running acquisition.
    if let Err(e) = cmd.command(MPXCMD_STOPACQUISITION, default_timeout()) {
        log::error!("merlin: initial STOPACQUISITION failed: {e}");
    }

    let det = MerlinDetector::new(
        port_name,
        cmd,
        max_size_x,
        max_size_y,
        det_type,
        max_memory,
        shared.clone(),
    )?;
    let ad_params = det.ad.params;
    let params = det.params;
    let init_param = det.init_param;
    let pool = det.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let queued_counter = Arc::new(QueuedArrayCounter::new());

    let (data_task, status_task) = start_acquisition_task(AcquisitionContext {
        data,
        handle: runtime_handle.port_handle().clone(),
        output: ArrayPublisher::new(array_output.clone()),
        queued: queued_counter.clone(),
        ad_params,
        params,
        init_param,
        det_type,
        shared,
    });

    Ok(MerlinRuntime {
        runtime_handle,
        ad_params,
        params,
        pool,
        array_output,
        queued_counter,
        data_task,
        status_task,
    })
}
