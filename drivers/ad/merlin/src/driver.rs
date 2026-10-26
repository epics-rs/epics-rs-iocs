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

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::ndarray::{NDArray, NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};

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
    /// C `pFlatField`: allocated by the constructor and never used, but it
    /// holds its pool buffer for the driver's life (merlinDetector.cpp:1390).
    _flat_field: Option<NDArray>,
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
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            max_memory,
            ASYN_CANBLOCK,
            InterfaceMask::UNMIGRATED_SUBCLASS,
            InterfaceMask::UNMIGRATED_SUBCLASS_INTERRUPTS,
        )?;
        // C sends this once ADDriver exists but before its own parameters
        // do, so a restarted IOC never inherits a running acquisition.
        let _ = cmd.command(&mut ad, MPXCMD_STOPACQUISITION, default_timeout());
        // A refusal leaves C's pointer NULL; the pool has printed its line.
        let _flat_field = ad
            .pool
            .pool()
            .alloc(
                vec![
                    NDDimension::new(max_size_x as usize),
                    NDDimension::new(max_size_y as usize),
                ],
                NDDataType::UInt32,
            )
            .ok();
        let params = MerlinParams::create(&mut ad.port_base)?;
        let init_param = ad
            .port_base
            .create_param("MERLIN_INIT", epics_rs::asyn::param::ParamType::Int32)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, det_type.manufacturer())?;
        base.set_string_param(ad.params.base.model, 0, det_type.model())?;
        base.set_string_param(params.select_gui, 0, det_type.gui())?;

        base.set_int32_param(ad.params.max_size_x, 0, max_size_x)?;
        base.set_int32_param(ad.params.max_size_y, 0, max_size_y)?;
        base.set_int32_param(ad.params.size_x, 0, max_size_x)?;
        base.set_int32_param(ad.params.size_y, 0, max_size_y)?;
        base.set_int32_param(ad.params.base.array_size_x, 0, max_size_x)?;
        base.set_int32_param(ad.params.base.array_size_y, 0, max_size_y)?;
        base.set_int32_param(ad.params.base.array_size, 0, 0)?;
        base.set_int32_param(ad.params.base.data_type, 0, NDDataType::UInt32 as u8 as i32)?;
        base.set_int32_param(ad.params.image_mode, 0, MerlinImageMode::Continuous as i32)?;
        base.set_int32_param(ad.params.trigger_mode, 0, TriggerMode::Internal as i32)?;
        base.set_int32_param(params.profile_control, 0, MPXPROFILES_IMAGE)?;
        base.set_int32_param(ad.params.status, 0, ADStatus::Initializing as i32)?;

        Ok(Self {
            ad,
            params,
            init_param,
            det_type,
            cmd,
            shared,
            _flat_field,
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
        if let Ok(version) =
            self.cmd
                .get(&mut self.ad, MPXVAR_GETSOFTWAREVERSION, default_timeout())
        {
            let base = &mut self.ad.port_base;
            base.set_string_param(self.ad.params.base.sdk_version, 0, version)?;
        }

        let base = &mut self.ad.port_base;
        base.set_int32_param(self.ad.params.status, 0, ADStatus::Idle as i32)?;
        base.set_string_param(
            self.ad.params.status_message,
            0,
            "Waiting for acquire command",
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
            self.try_set(MPXVAR_IMAGESTOSUM, &exposures.to_string());
            let bkg = self.get_i32(self.params.enable_background_corr);
            self.try_set(MPXVAR_ENABLEBACKROUNDCORR, &bkg.to_string());
            let sum = self.get_i32(self.params.enable_image_sum);
            self.try_set(MPXVAR_ENABLEIMAGEAVERAGE, &sum.to_string());
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

        self.try_set(MPXVAR_NUMFRAMESPERTRIGGER, &num_exposures.to_string());
        self.try_set(MPXVAR_COUNTERDEPTH, &counter_depth.to_string());
        // The device takes milliseconds.
        self.try_set(
            MPXVAR_ACQUISITIONTIME,
            &format!("{:.6}", CFloat(acquire_time * 1000.0)),
        );
        self.try_set(
            MPXVAR_ACQUISITIONPERIOD,
            &format!("{:.6}", CFloat(acquire_period * 1000.0)),
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
        self.try_set(MPXVAR_TRIGGERSTART, start);
        if let Some(stop) = stop {
            self.try_set(MPXVAR_TRIGGERSTOP, stop);
        }

        // The server may lengthen the period to fit the readout time; take its
        // value back. C tested a stale `status` here instead of this GET's.
        if let Some(period) = self
            .cmd
            .get_f64(&mut self.ad, MPXVAR_ACQUISITIONPERIOD, timeout)
        {
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
        self.try_set(MPXVAR_ROI, &value);
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
            if let Some(v) = self.cmd.get_f64(&mut self.ad, name, timeout) {
                self.ad
                    .port_base
                    .set_float64_param(self.params.thresholds[i], 0, v)?;
            }
        }
        if let Some(v) = self
            .cmd
            .get_f64(&mut self.ad, MPXVAR_OPERATINGENERGY, timeout)
        {
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

        self.try_set(
            MPXVAR_THSTART,
            &format!("{start:.6}", start = CFloat(start)),
        );
        self.try_set(MPXVAR_THSTOP, &format!("{stop:.6}", stop = CFloat(stop)));
        self.try_set(MPXVAR_THSTEP, &format!("{step:.6}", step = CFloat(step)));
        self.try_set(MPXVAR_THSSCAN, &scan.to_string());

        for (name, reason) in [
            (MPXVAR_THSTART, self.params.start_threshold_scan),
            (MPXVAR_THSTEP, self.params.step_threshold_scan),
            (MPXVAR_THSTOP, self.params.stop_threshold_scan),
        ] {
            if let Some(v) = self.cmd.get_f64(&mut self.ad, name, timeout) {
                self.ad.port_base.set_float64_param(reason, 0, v)?;
            }
        }
        if let Some(v) = self.cmd.get_i32(&mut self.ad, MPXVAR_THSSCAN, timeout) {
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
            self.try_set(MPXVAR_ENABLECOUNTER1, &v.to_string());
        }
        if reason == self.params.continuous_rw {
            let mut v = self.get_i32(self.params.continuous_rw);
            if !(0..=1).contains(&v) {
                v = 0;
                self.ad
                    .port_base
                    .set_int32_param(self.params.continuous_rw, 0, v)?;
            }
            self.try_set(MPXVAR_CONTINUOUSRW, &v.to_string());
        }

        if let Some(v) = self
            .cmd
            .get_i32(&mut self.ad, MPXVAR_ENABLECOUNTER1, timeout)
        {
            self.ad
                .port_base
                .set_int32_param(self.params.enable_counter1, 0, v)?;
        }
        if let Some(v) = self.cmd.get_i32(&mut self.ad, MPXVAR_CONTINUOUSRW, timeout) {
            self.ad
                .port_base
                .set_int32_param(self.params.continuous_rw, 0, v)?;
        }
        Ok(())
    }

    /// Apply one of the six canned Quad modes (C `SetQuadMode`).
    fn set_quad_mode(&mut self, mode: i32) -> AsynResult<()> {
        let Some(s) = quad_mode_settings(mode) else {
            return Ok(());
        };
        self.shared
            .frames_per_acquire
            .store(s.frames_per_acquire, Ordering::Release);
        self.ad
            .port_base
            .set_int32_param(self.params.counter_depth, 0, s.counter_depth)?;

        self.try_set(MPXVAR_COUNTERDEPTH, &s.counter_depth.to_string());
        self.try_set(MPXVAR_ENABLECOUNTER1, &s.enable_counter1.to_string());
        self.try_set(MPXVAR_CONTINUOUSRW, &s.continuous_rw.to_string());
        self.try_set(MPXVAR_COLOURMODE, &s.colour_mode.to_string());
        self.try_set(MPXVAR_CHARGESUMMING, &s.charge_summing.to_string());
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
            base.set_string_param(self.ad.params.status_message, 0, "Acquiring...")?;
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
                        "Performing Threshold Scan...",
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

            // C keeps the status of these two commands only.
            match image_mode {
                MerlinImageMode::ThresholdScan => {
                    self.cmd.command(&mut self.ad, MPXCMD_THSCAN, timeout)?;
                }
                MerlinImageMode::BackgroundCalibrate => {
                    self.try_set(MPXVAR_BACKGROUNDCOUNT, &images_to_acquire.to_string());
                    self.cmd
                        .command(&mut self.ad, MPXCMD_BACKGROUNDACQUIRE, timeout)?;
                }
                _ => {
                    self.try_set(MPXVAR_NUMFRAMESTOACQUIRE, &images_to_acquire.to_string());
                    let cmd = if profile_mask & MPXPROFILES_IMAGE != 0 {
                        MPXCMD_STARTACQUISITION
                    } else {
                        MPXCMD_PROFILES
                    };
                    let _ = self.cmd.command(&mut self.ad, cmd, timeout);
                }
            }
        } else if value == 0 && status == ADStatus::Acquire as i32 {
            self.ad
                .port_base
                .set_int32_param(self.ad.params.status, 0, ADStatus::Idle as i32)?;
            let _ = self
                .cmd
                .command(&mut self.ad, MPXCMD_STOPACQUISITION, timeout);
        }
        Ok(())
    }

    fn try_set(&mut self, name: &str, value: &str) {
        let _ = self.cmd.set(&mut self.ad, name, value, default_timeout());
    }
}

impl PortDriver for MerlinDetector {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    /// C inherits `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.ad.write_octet(user, data)
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .set_int32_param(reason, user.addr, value)?;

        let status = if reason == self.init_param {
            self.initialise()
        } else if reason == self.params.reset {
            let _ = self
                .cmd
                .command(&mut self.ad, MPXCMD_RESET, default_timeout());
            // The Labview server cannot be reconnected to after a reset, so C
            // exits the IOC and relies on a supervisor to restart it. Keeping
            // the IOC up would leave every channel dead with no way back.
            self.ad.port_base.call_param_callbacks(0)?;
            std::process::exit(0);
        } else if reason == self.params.quad_merlin_mode {
            self.set_quad_mode(value)
        } else if reason == self.params.software_trigger {
            let _ = self
                .cmd
                .command(&mut self.ad, MPXCMD_SOFTWARETRIGGER, default_timeout());
            Ok(())
        } else if reason == self.ad.params.acquire {
            self.set_acquire(value)
        } else if reason == self.ad.params.trigger_mode
            || reason == self.ad.params.num_images
            || reason == self.ad.params.num_exposures
            || reason == self.params.counter_depth
            || reason == self.params.enable_background_corr
            || reason == self.params.enable_image_sum
        {
            self.set_acquire_params()
        } else if reason == self.ad.params.size_x
            || reason == self.ad.params.size_y
            || reason == self.ad.params.min_x
            || reason == self.ad.params.min_y
        {
            self.set_roi()
        } else if reason == self.params.enable_counter1 || reason == self.params.continuous_rw {
            self.set_mode_commands(reason)
        } else if reason == self.params.threshold_apply {
            self.get_threshold()
        } else if reason == self.params.profile_control {
            self.try_set(MPXCMD_PROFILES, &value.to_string());
            Ok(())
        } else if reason < self.params.first() {
            ADDriver::ad_write_int32(self, user, value)
        } else {
            Ok(())
        };

        self.ad.port_base.call_param_callbacks(0)?;
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "merlinDetector:writeInt32: error, status={}, reason={reason} value={value}\n",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("merlinDetector:writeInt32: reason={reason}, value={value}\n"),
            ),
        }
        status
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        // C ignores the status and restores 0 on a failed write
        // (merlinDetector.cpp:1206,1290-1294).
        let old = self
            .ad
            .port_base
            .get_float64_param(reason, user.addr)
            .unwrap_or(0.0);
        let stored = self
            .ad
            .port_base
            .set_float64_param(reason, user.addr, value);

        let name = if let Some(i) = self.params.thresholds.iter().position(|p| *p == reason) {
            Some(MPXVAR_THRESHOLD[i])
        } else if reason == self.params.operating_energy {
            Some(MPXVAR_OPERATINGENERGY)
        } else {
            None
        };
        let status = if let Some(name) = name {
            let set = self
                .cmd
                .set(
                    &mut self.ad,
                    name,
                    &format!("{value:.6}", value = CFloat(value)),
                    default_timeout(),
                )
                .map_err(AsynError::from);
            let _ = self.get_threshold();
            set
        } else if reason == self.ad.params.acquire_time || reason == self.ad.params.acquire_period {
            // C drops both helpers' statuses (merlinDetector.cpp:1274-1282).
            let _ = self.set_acquire_params();
            stored
        } else if reason == self.params.start_threshold_scan
            || reason == self.params.stop_threshold_scan
            || reason == self.params.step_threshold_scan
        {
            let _ = self.update_threshold_scan_params();
            stored
        } else if reason < self.params.first() {
            // ADDriver::writeFloat64 is asynPortDriver's (merlinDetector.cpp:1287).
            self.ad.port_base.write_float64_param(user, value)
        } else {
            stored
        };

        match &status {
            Err(e) => {
                let _ = self.ad.port_base.set_float64_param(reason, user.addr, old);
                user.print(
                    TraceMask::ERROR,
                    file!(),
                    line!(),
                    format_args!(
                        "merlinDetector:writeFloat64 error, status={} function={reason}, value={value:.6}\n",
                        e.status() as i32, value = CFloat(value)
                    ),
                );
            }
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "merlinDetector:writeFloat64: function={reason}, value={value:.6}\n",
                    value = CFloat(value)
                ),
            ),
        }
        let _ = self.ad.port_base.call_param_callbacks(0);
        status
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
    pool: DriverPool,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    #[allow(dead_code)]
    data_task: std::thread::JoinHandle<()>,
    #[allow(dead_code)]
    status_task: std::thread::JoinHandle<()>,
}

impl MerlinRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
        self.array_output.lock().add(sender);
    }
}

/// Create the detector port and start its two tasks.
///
/// `cmd_handle` and `data_handle` are the users of the two `drvAsynIPPort`s
/// the startup script created — the command socket and the data socket. C
/// ignores both connect statuses (merlinDetector.cpp:1398-1401), so either
/// may be a user with no port.
#[allow(clippy::too_many_arguments)]
pub fn create_merlin_detector(
    claim: &PortClaim,
    cmd_handle: SyncIOHandle,
    data_handle: SyncIOHandle,
    max_size_x: i32,
    max_size_y: i32,
    det_type: DetectorType,
    max_memory: usize,
) -> AsynResult<MerlinRuntime> {
    let port_name = claim.name();
    let shared = Arc::new(SharedState::new());
    let cmd = MpxConnection::new(cmd_handle, MPX_MAXLINE);
    let data = MpxConnection::new(data_handle, det_type.max_frame_bytes());

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
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let (data_task, status_task) = start_acquisition_task(AcquisitionContext {
        data,
        handle: runtime_handle.port_handle().clone(),
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
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
        data_task,
        status_task,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ShutterMode;
    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::interfaces::InterfaceType;
    use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
    use epics_rs::asyn::param::ParamValue;
    use epics_rs::asyn::port::PortFlags;
    use epics_rs::asyn::registry::claim_port;
    use std::time::Duration;

    use epics_rs::asyn::sync_io::SyncIOHandle;

    /// A detector whose command socket never connects: the writes under test
    /// must not need the device.
    fn detector(name: &str) -> MerlinDetector {
        detector_sized(name, 1 << 20)
    }

    fn detector_sized(name: &str, max_memory: usize) -> MerlinDetector {
        let ip = format!("{name}_CMD");
        let driver = DrvAsynIPPort::new(&ip, "127.0.0.1:1").unwrap();
        let (runtime, _) =
            create_port_runtime(&claim_port(&ip).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        MerlinDetector::new(
            name,
            MpxConnection::new(
                SyncIOHandle::from_handle(handle, 0, Duration::ZERO),
                MPX_MAXLINE,
            ),
            512,
            512,
            DetectorType::MerlinQuad,
            max_memory,
            Arc::new(SharedState::new()),
        )
        .unwrap()
    }

    /// C `setIntegerParam(merlinEnableCounter1, atoi(fromLabviewValue))`
    /// (merlinDetector.cpp:575): glibc's atoi stops at the first non-digit.
    #[test]
    fn a_read_back_mode_is_atoi() {
        use std::io::{Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            loop {
                let mut header = [0u8; MPX_FRAME_HEADER_LEN];
                if sock.read_exact(&mut header).is_err() {
                    return;
                }
                let len: usize = std::str::from_utf8(&header[4..14])
                    .unwrap()
                    .parse()
                    .unwrap();
                let mut body = vec![0u8; len - 1];
                sock.read_exact(&mut body).unwrap();
                let body = String::from_utf8(body).unwrap();
                let mut f = body.split(',');
                let (kind, name) = (f.next().unwrap(), f.next().unwrap());
                let reply = match (kind, name) {
                    ("GET", MPXVAR_ENABLECOUNTER1) => format!("GET,{name},1x,0"),
                    ("GET", _) => format!("GET,{name},0,0"),
                    _ => format!("{kind},{name},0"),
                };
                let msg = format!("MPX,{:010},{reply}", reply.len() + 1);
                sock.write_all(msg.as_bytes()).unwrap();
            }
        });
        let ip = "MERLIN_ATOI_CMD";
        let driver = DrvAsynIPPort::new(ip, &addr.to_string()).unwrap();
        let (runtime, _) =
            create_port_runtime(&claim_port(ip).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        let mut det = MerlinDetector::new(
            "MERLIN_ATOI",
            MpxConnection::new(
                SyncIOHandle::from_handle(handle, 0, Duration::ZERO),
                MPX_MAXLINE,
            ),
            512,
            512,
            DetectorType::MerlinQuad,
            1 << 20,
            Arc::new(SharedState::new()),
        )
        .unwrap();
        let counter1 = det.params.enable_counter1;
        det.ad.port_base.set_int32_param(counter1, 0, 0).unwrap();
        det.set_mode_commands(counter1).unwrap();
        assert_eq!(det.ad.port_base.get_int32_param(counter1, 0).unwrap(), 1);
    }

    /// C ignores both `pasynOctetSyncIO->connect` statuses
    /// (merlinDetector.cpp:1398-1401): with neither socket port configured
    /// the detector port still exists.
    #[test]
    fn a_detector_whose_socket_ports_are_missing_still_boots() {
        let (cmd, status) =
            SyncIOHandle::connect_port("MERLIN_NONE_CMD", 0, Duration::ZERO, InterfaceType::Octet);
        assert!(status.is_err());
        let (data, status) =
            SyncIOHandle::connect_port("MERLIN_NONE_DATA", 0, Duration::ZERO, InterfaceType::Octet);
        assert!(status.is_err());
        let rt = create_merlin_detector(
            &claim_port("MERLIN_NONE").unwrap(),
            cmd,
            data,
            512,
            512,
            DetectorType::MerlinQuad,
            1 << 20,
        )
        .expect("the detector port");
        let sync = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(5));
        let p = rt.ad_params;
        sync.write_int32(p.num_images, 3).unwrap();
        assert_eq!(sync.read_int32(p.num_images).unwrap(), 3);
    }

    /// merlinDetector.cpp never sets NDDriverVersion, so it keeps the ADCore
    /// version asynNDArrayDriver gave it (asynNDArrayDriver.cpp:966).
    #[test]
    fn the_driver_version_is_left_to_adcore() {
        let det = detector("MERLIN_VERSION");
        let base = &det.ad.port_base;
        let p = det.ad.params.base;
        assert_eq!(
            base.get_string_param(p.driver_version, 0).unwrap(),
            base.get_string_param(p.ad_core_version, 0).unwrap()
        );
    }

    /// The constructor allocates the flat-field buffer from the driver's pool
    /// and keeps it (merlinDetector.cpp:1390), so PoolPollStats counts it.
    #[test]
    fn the_flat_field_buffer_is_counted_on_the_pool() {
        let mut det = detector("MERLIN_FLAT_FIELD");
        let p = det.ad.params.base;
        let mut user = AsynUser::new(p.pool_poll_stats);
        det.write_int32(&mut user, 1).unwrap();
        let base = &det.ad.port_base;
        assert_eq!(base.get_int32_param(p.pool_alloc_buffers, 0).unwrap(), 1);
        assert_eq!(base.get_int32_param(p.pool_free_buffers, 0).unwrap(), 0);
        assert_eq!(base.get_float64_param(p.pool_used_memory, 0).unwrap(), 1.0);
    }

    /// C's constructor leaves COUNTERDEPTH unset (merlinDetector.cpp:1434);
    /// only setAcquireParams defaults it to 12, so the output record boots UDF.
    #[test]
    fn counter_depth_starts_undefined() {
        let det = detector("MERLIN_COUNTER_DEPTH");
        let reason = det.params.counter_depth;
        assert!(matches!(
            det.ad.port_base.get_int32_param(reason, 0),
            Err(AsynError::ParamUndefined(r)) if r == reason
        ));
    }

    #[test]
    fn a_base_class_write_reaches_addriver() {
        let mut det = detector("MERLIN_BASE_WRITE");
        let p = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(p.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        let mut user = AsynUser::new(p.shutter_control);
        det.write_int32(&mut user, 1).unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(p.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }

    /// ADDriver::writeInt32 and asynNDArrayDriver::writeInt32 each close with
    /// an ASYN_TRACEIO_DRIVER line (ADDriver.cpp:111-113,
    /// asynNDArrayDriver.cpp:703-705).
    #[test]
    fn a_base_class_write_runs_both_base_class_write_int32s() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "MERLIN_BASE_TRACE";
        let det = detector(name);
        let p = det.ad.params.base;
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default()).unwrap();
        let h = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        h.trace().set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        h.write_int32_blocking(p.pool_poll_stats, 0, 1).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for driver in ["asynNDArrayDriver", "ADDriver"] {
            assert!(
                text.contains(&format!(
                    "{driver}:writeInt32: function={}",
                    p.pool_poll_stats
                )),
                "{driver}: {text:?}"
            );
        }
        std::mem::forget(runtime);
    }

    #[test]
    fn a_refused_threshold_write_fails_and_keeps_the_old_value() {
        let mut det = detector("MERLIN_THRESHOLD_WRITE");
        let threshold = det.params.thresholds[0];
        det.ad
            .port_base
            .set_float64_param(threshold, 0, 7.5)
            .unwrap();
        let mut user = AsynUser::new(threshold);
        assert!(
            det.write_float64(&mut user, 9.0).is_err(),
            "a SET the device never answered reported success"
        );
        assert_eq!(
            det.ad.port_base.get_float64_param(threshold, 0).unwrap(),
            7.5
        );
    }

    #[test]
    fn a_refused_write_to_an_unset_threshold_restores_zero() {
        let mut det = detector("MERLIN_UNSET_THRESHOLD");
        let threshold = det.params.thresholds[1];
        let mut user = AsynUser::new(threshold);
        assert!(det.write_float64(&mut user, 9.0).is_err());
        assert_eq!(
            det.ad.port_base.get_float64_param(threshold, 0).unwrap(),
            0.0
        );
    }

    /// merlinDetector never writes `pasynUser->errorMessage`, so a refused
    /// SET reaches the record as its status alone.
    #[test]
    fn a_refused_threshold_write_hands_the_record_no_message() {
        let mut det = detector("MERLIN_WRITE_MESSAGE");
        let mut user = AsynUser::new(det.params.thresholds[0]);
        let e = det.write_float64(&mut user, 9.0).unwrap_err();
        assert_eq!(e.message(), "");
    }

    /// mpxWrite hands every request to toLabViewStr (mpxConnection.cpp:571),
    /// the constructor's STOPACQUISITION included, whether or not it went out.
    #[test]
    fn the_constructors_stop_is_the_string_to_server() {
        let (cmd, _) =
            SyncIOHandle::connect_port("MERLIN_ECHO_CMD", 0, Duration::ZERO, InterfaceType::Octet);
        let (data, _) =
            SyncIOHandle::connect_port("MERLIN_ECHO_DATA", 0, Duration::ZERO, InterfaceType::Octet);
        let rt = create_merlin_detector(
            &claim_port("MERLIN_ECHO").unwrap(),
            cmd,
            data,
            512,
            512,
            DetectorType::MerlinQuad,
            1 << 20,
        )
        .unwrap();
        let sync = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(5));
        let (text, status) = sync
            .read_octet(rt.ad_params.string_to_server, 256)
            .into_parts();
        status.unwrap();
        // NORD 35 in C: the 34 characters and the NUL.
        assert_eq!(text, b"MPX,0000000020,CMD,STOPACQUISITION\0");
    }

    /// mpxReadCmd hands every reply it reads to fromLabViewStr
    /// (mpxConnection.cpp:763), and the write's callbacks carry both strings.
    #[test]
    fn a_command_reply_is_the_string_from_server() {
        use std::io::{Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        std::thread::spawn(move || {
            let (mut s, _) = listener.accept().unwrap();
            let mut header = [0u8; 15];
            s.read_exact(&mut header).unwrap();
            let len: usize = std::str::from_utf8(&header[4..14])
                .unwrap()
                .parse()
                .unwrap();
            let mut body = vec![0u8; len - 1];
            s.read_exact(&mut body).unwrap();
            let reply = "CMD,SWTRIGGER,0";
            write!(s, "MPX,{:010},{reply}", reply.len() + 1).unwrap();
            std::thread::sleep(Duration::from_secs(2));
        });
        let ip = "MERLIN_REPLY_CMD";
        let driver = DrvAsynIPPort::new(ip, &addr).unwrap();
        let (runtime, _) =
            create_port_runtime(&claim_port(ip).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        let mut det = MerlinDetector::new(
            "MERLIN_REPLY",
            MpxConnection::new(
                SyncIOHandle::from_handle(handle, 0, Duration::from_secs(1)),
                MPX_MAXLINE,
            ),
            512,
            512,
            DetectorType::MerlinQuad,
            1 << 20,
            Arc::new(SharedState::new()),
        )
        .unwrap();
        let mut user = AsynUser::new(det.params.software_trigger);
        det.write_int32(&mut user, 1).unwrap();
        let p = det.ad.params;
        let base = &det.ad.port_base;
        assert_eq!(
            base.get_string_param(p.string_to_server, 0).unwrap(),
            b"MPX,0000000014,CMD,SWTRIGGER"
        );
        assert_eq!(
            base.get_string_param(p.string_from_server, 0).unwrap(),
            b"MPX,0000000016,CMD,SWTRIGGER,0"
        );
    }

    /// C's writeFloat64 error line for THRESHOLD0 reads `function=99`: the
    /// ADDriver params, then DELAY_TIME, then the thresholds.
    #[test]
    fn a_threshold_error_line_prints_cs_function_number() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "MERLIN_FUNCTION_NO";
        let det = detector(name);
        let threshold = det.params.thresholds[0];
        let (runtime, _) =
            create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default()).unwrap();
        let h = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        h.trace().set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace().set_trace_mask(Some(name), TraceMask::ERROR);
        assert!(h.write_float64_blocking(threshold, 0, 10.0).is_err());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.contains(
                "merlinDetector:writeFloat64 error, status=4 function=99, value=10.000000"
            ),
            "{text:?}"
        );
        std::mem::forget(runtime);
    }

    /// The last frame of a Single acquisition under a software trigger;
    /// returns ADAcquire, SoftwareTrigger and NDArrayCounter after it.
    fn last_software_triggered_frame(name: &str, max_memory: usize) -> (i32, i32, i32) {
        let mut det = detector_sized(name, max_memory);
        let (ad_params, params) = (det.ad.params, det.params);
        let base = &mut det.ad.port_base;
        base.set_int32_param(ad_params.acquire, 0, 1).unwrap();
        base.set_int32_param(
            ad_params.trigger_mode,
            0,
            TriggerMode::SoftwareTrigger as i32,
        )
        .unwrap();
        base.set_int32_param(params.software_trigger, 0, 1).unwrap();
        det.shared.images_remaining.store(1, Ordering::Release);
        let handle = run_quad_frame(name, det, NDArrayOutput::new());
        let acquire = handle.read_int32_blocking(ad_params.acquire, 0).unwrap();
        let trigger = handle
            .read_int32_blocking(params.software_trigger, 0)
            .unwrap();
        let counter = handle
            .read_int32_blocking(ad_params.base.array_counter, 0)
            .unwrap();
        (acquire, trigger, counter)
    }

    /// One 256x256 quad data frame through handle_frame, on `det`'s port.
    fn run_quad_frame(name: &str, det: MerlinDetector, output: NDArrayOutput) -> PortHandle {
        let ctx = quad_frame_context(name, det, output);
        let handle = ctx.handle.clone();
        run_quad_frame_in(ctx);
        handle
    }

    /// The data task's context on `det`'s port, before any frame.
    fn quad_frame_context(
        name: &str,
        det: MerlinDetector,
        output: NDArrayOutput,
    ) -> AcquisitionContext {
        let (ad_params, params, init_param) = (det.ad.params, det.params, det.init_param);
        let pool = det.ad.pool.clone();
        let shared = det.shared.clone();
        let claim = claim_port(name).unwrap();
        let (runtime, _) = create_port_runtime(&claim, det, RuntimeConfig::default()).unwrap();
        let handle = runtime.port_handle().clone();
        // Published open, so its callbacks post.
        claim
            .publish(handle.clone(), &epics_rs::asyn::registry::open_owner())
            .unwrap();
        std::mem::forget(runtime);
        AcquisitionContext {
            data: MpxConnection::new(
                SyncIOHandle::from_handle(handle.clone(), 0, Duration::ZERO),
                MPX_MAXLINE,
            ),
            handle: handle.clone(),
            output: ArrayPublisher::new(Arc::new(parking_lot::Mutex::new(output))),
            pool,
            ad_params,
            params,
            init_param,
            det_type: DetectorType::MerlinQuad,
            shared,
        }
    }

    fn run_quad_frame_in(ctx: AcquisitionContext) {
        let mut body = crate::protocol::tests::quad_header(768, 1);
        body.resize(768 + 256 * 256 * 2, 0);
        epics_rs::ad_core::runtime::run_thread(move || async move {
            crate::task::handle_frame(&ctx, &body, &mut String::new()).await;
        })
        .join()
        .unwrap();
    }

    /// A 16-bit frame costs its pool buffer and no other buffer of its size:
    /// C swaps and flips the socket bytes into `pData` in one pass
    /// (merlinDetector.cpp:464-490).
    #[test]
    fn a_frame_decodes_straight_into_its_pool_buffer() {
        use epics_rs::ad_core::ndarray::NDDataView;
        use epics_rs::ad_core::plugin::channel::ndarray_channel;

        let (w, h) = (200usize, 120usize);
        let size = w * h * 2;
        let dims = || vec![NDDimension::new(w), NDDimension::new(h)];
        let counted = crate::alloc_count::count(size);
        let pooled = DriverPool::standalone(0).alloc(dims(), NDDataType::UInt16);
        let pool_only = counted.stop();
        drop(pooled);

        let det = detector_sized("MERLIN_FRAME_ALLOCS", 0);
        let mut output = NDArrayOutput::new();
        let (tx, mut rx) = ndarray_channel("MERLIN_FRAME_ALLOCS_SINK", 4);
        output.add(tx);
        let ctx = quad_frame_context("MERLIN_FRAME_ALLOCS", det, output);
        let header = String::from_utf8(crate::protocol::tests::quad_header(768, 1))
            .unwrap()
            .replacen("256,256,", &format!("{w},{h},"), 1);
        let mut body = header.into_bytes();
        // Big-endian pixels, each the value of its row-major index.
        body.extend((0..w * h).flat_map(|i| (i as u16).to_be_bytes()));
        let counted = crate::alloc_count::count(size);
        epics_rs::ad_core::runtime::run_thread(move || async move {
            crate::task::handle_frame(&ctx, &body, &mut String::new()).await;
        })
        .join()
        .unwrap();
        let allocs = counted.stop();

        assert_eq!(allocs, pool_only, "frame-sized allocations");
        assert_eq!(rx.pending(), 1, "the frame is published");
        let array = rx.blocking_recv().unwrap();
        let NDDataView::U16(pixels) = array.data().view() else {
            panic!("a 16-bit frame is NDUInt16");
        };
        // Source row h-1-y lands on row y.
        let expected: Vec<u16> = (0..h)
            .flat_map(|y| (0..w).map(move |x| ((h - 1 - y) * w + x) as u16))
            .collect();
        assert_eq!(pixels, &expected[..]);
    }

    /// merlinTask only stores what a frame the pool refuses sets (the counters,
    /// :145-152, and the status message, :441-442) and `continue`s past its
    /// callParamCallbacks (:305): the next driver callback posts them.
    #[test]
    fn a_refused_frame_posts_nothing_until_the_next_callback() {
        let det = detector_sized("MERLIN_REFUSED_POSTS", 1000);
        let (message, counter) = (
            det.ad.params.status_message,
            det.ad.params.base.array_counter,
        );
        let ctx = quad_frame_context("MERLIN_REFUSED_POSTS", det, NDArrayOutput::new());
        let handle = ctx.handle.clone();
        let mut posts = handle.interrupts().subscribe_async(256, 64);
        run_quad_frame_in(ctx);
        let mut watched = || {
            handle.read_int32_blocking(counter, 0).unwrap();
            let mut seen = Vec::new();
            // Only what the frame sets: the open flush's zero and empty
            // message can still be arriving.
            while let Ok(v) = posts.try_recv() {
                match v.value {
                    ParamValue::Int32(n) if v.reason == counter && n != 0 => {
                        seen.push(format!("counter {n}"))
                    }
                    ParamValue::Octet(text) if v.reason == message && !text.is_empty() => {
                        seen.push(format!("message {}", String::from_utf8_lossy(&text)))
                    }
                    _ => {}
                }
            }
            seen
        };
        assert_eq!(watched(), Vec::<String>::new());
        handle.call_param_callbacks_blocking(0).unwrap();
        assert_eq!(
            watched(),
            [
                "counter 1",
                "message Error: run out of buffers in detector driver"
            ]
        );
    }

    /// The last frame of an acquisition stores Acquire 0 and Status Idle
    /// (merlinDetector.cpp:296-300) and the pass's own callParamCallbacks
    /// (:305) posts them, with no other callback after it.
    #[test]
    fn the_last_frame_posts_the_acquisition_complete() {
        // 1 MiB of the limit is the flat-field buffer.
        let mut det = detector_sized("MERLIN_DONE_POSTS", 2 << 20);
        let (acquire, status) = (det.ad.params.acquire, det.ad.params.status);
        let base = &mut det.ad.port_base;
        base.set_int32_param(acquire, 0, 1).unwrap();
        base.set_int32_param(status, 0, ADStatus::Acquire as i32)
            .unwrap();
        det.shared.images_remaining.store(1, Ordering::Release);
        let ctx = quad_frame_context("MERLIN_DONE_POSTS", det, NDArrayOutput::new());
        let handle = ctx.handle.clone();
        let mut posts = handle.interrupts().subscribe_async(256, 64);
        run_quad_frame_in(ctx);
        handle.read_int32_blocking(acquire, 0).unwrap();
        let mut seen = Vec::new();
        while let Ok(v) = posts.try_recv() {
            match v.value {
                ParamValue::Int32(0) if v.reason == acquire => seen.push("acquire 0"),
                ParamValue::Int32(n) if v.reason == status && n == ADStatus::Idle as i32 => {
                    seen.push("status idle")
                }
                _ => {}
            }
        }
        assert_eq!(seen, ["acquire 0", "status idle"]);
    }

    /// merlinTask holds the driver lock from the end of its read to the
    /// next (merlinDetector.cpp:137-305): another thread's callParamCallbacks
    /// waits for the pass's end and cannot post the counters a half pass
    /// stored. The pass is stopped in the middle by its "Unknown header" line
    /// (:250-253), which waits on the trace file.
    #[test]
    fn another_callback_cannot_post_a_half_pass() {
        use epics_rs::asyn::trace::TraceFile;
        let name = "MERLIN_HALF_PASS";
        let det = detector_sized(name, 2 << 20);
        let counter = det.ad.params.base.array_counter;
        let ctx = quad_frame_context(name, det, NDArrayOutput::new());
        let handle = ctx.handle.clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = Arc::new(std::sync::Mutex::new(std::fs::File::create(&path).unwrap()));
        handle
            .trace()
            .set_trace_file(Some(name), TraceFile::File(file.clone()));
        let mut posts = handle.interrupts().subscribe_async(256, 64);
        let mut counted = move || {
            let mut n = 0;
            while let Ok(v) = posts.try_recv() {
                if v.reason == counter && matches!(v.value, ParamValue::Int32(1)) {
                    n += 1;
                }
            }
            n
        };

        let trace_held = file.lock().unwrap();
        let pass = std::thread::spawn(move || {
            let body = b"XXX,000001".to_vec();
            epics_rs::ad_core::runtime::run_thread(move || async move {
                crate::task::handle_frame(&ctx, &body, &mut String::new()).await;
            })
            .join()
            .unwrap();
        });
        std::thread::sleep(Duration::from_millis(200));
        let other = {
            let handle = handle.clone();
            std::thread::spawn(move || handle.call_param_callbacks_blocking(0))
        };
        std::thread::sleep(Duration::from_millis(200));
        assert_eq!(counted(), 0, "another callback posted the half pass");
        drop(trace_held);
        pass.join().unwrap();
        other.join().unwrap().unwrap();
        handle.read_int32_blocking(counter, 0).unwrap();
        assert_eq!(counted(), 1);
        let _ = std::fs::remove_file(&path);
    }

    /// copyProfileToNDArray32 posts the profiles at once through
    /// doCallbacksInt32Array, Y then X (merlinDetector.cpp:415-416), on the
    /// asynInt32Array interface; the counters the pass stored go out only at
    /// its end (:305).
    #[test]
    fn a_profile_frame_posts_its_profiles_before_the_pass() {
        // 1 MiB of the limit is the flat-field buffer.
        let det = detector_sized("MERLIN_PROFILES", 2 << 20);
        let (counter, x, y) = (
            det.ad.params.base.array_counter,
            det.params.profile_x,
            det.params.profile_y,
        );
        let ctx = quad_frame_context("MERLIN_PROFILES", det, NDArrayOutput::new());
        let handle = ctx.handle.clone();
        let mut posts = handle.interrupts().subscribe_async(256, 1 << 16);
        let mut body = crate::protocol::tests::quad_header(768, 1);
        body[..3].copy_from_slice(b"PR1");
        body.resize(768 + (256 + 256) * 8, 0);
        epics_rs::ad_core::runtime::run_thread(move || async move {
            crate::task::handle_frame(&ctx, &body, &mut String::new()).await;
        })
        .join()
        .unwrap();
        handle.read_int32_blocking(counter, 0).unwrap();
        let mut seen = Vec::new();
        while let Ok(v) = posts.try_recv() {
            match v.value {
                ParamValue::Int32(1) if v.reason == counter => seen.push("counter 1".to_string()),
                ParamValue::Int32Array(a) if v.reason == x || v.reason == y => seen.push(format!(
                    "{} {} {:?}",
                    if v.reason == y { "Y" } else { "X" },
                    a.len(),
                    v.iface
                )),
                _ => {}
            }
        }
        assert_eq!(
            seen,
            [
                "Y 256 Some(Int32Array)",
                "X 256 Some(Int32Array)",
                "counter 1"
            ]
        );
    }

    /// The array a `prefix` frame publishes, on a port whose time source is
    /// fixed at `at`.
    fn stamped_frame(
        name: &str,
        prefix: &[u8; 3],
        len: usize,
        at: std::time::SystemTime,
    ) -> Arc<NDArray> {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        let (sender, mut receiver) = ndarray_channel(&format!("{name}_SINK"), 4);
        let mut output = NDArrayOutput::new();
        output.add(sender);
        // 1 MiB of the limit is the flat-field buffer.
        let mut det = detector_sized(name, 2 << 20);
        det.ad.port_base.register_timestamp_source(move || at);
        let ctx = quad_frame_context(name, det, output);
        let mut body = crate::protocol::tests::quad_header(768, 1);
        body[..3].copy_from_slice(prefix);
        body.resize(768 + len, 0);
        epics_rs::ad_core::runtime::run_thread(move || async move {
            crate::task::handle_frame(&ctx, &body, &mut String::new()).await;
        })
        .join()
        .unwrap();
        receiver.blocking_recv().expect("the frame's array")
    }

    /// Data and profile frames take epicsTS from the port's time source, which
    /// C merlinTask never does, and timeStamp from the clock at the pass's
    /// start, as C does (merlinDetector.cpp:101, :255-258).
    #[test]
    fn a_frame_carries_the_port_time_stamp() {
        use epics_rs::ad_core::timestamp::EpicsTimestamp;
        let at = std::time::UNIX_EPOCH + Duration::new(1_700_000_000, 250_000_000);
        let stamp = EpicsTimestamp::from(at);
        for (name, prefix, len) in [
            ("MERLIN_FRAME_TS", b"MQ1", 256 * 256 * 2),
            ("MERLIN_PROFILE_TS", b"PR1", (256 + 256) * 8),
        ] {
            let before = EpicsTimestamp::now().as_f64();
            let array = stamped_frame(name, prefix, len, at);
            let after = EpicsTimestamp::now().as_f64();
            assert_eq!(array.timestamp, stamp, "{name}");
            assert!(
                (before..=after).contains(&array.time_stamp),
                "{name}: {}",
                array.time_stamp
            );
        }
    }

    /// merlinDetector.cpp sets the array size and type params only in the
    /// constructor (:1494-1497) and never the time-stamp ones: a frame
    /// leaves them all as they were.
    #[test]
    fn a_frame_leaves_the_array_params_alone() {
        // 1 MiB of the limit is the flat-field buffer.
        let det = detector_sized("MERLIN_FRAME_PARAMS", 2 << 20);
        let p = det.ad.params.base;
        let handle = run_quad_frame("MERLIN_FRAME_PARAMS", det, NDArrayOutput::new());
        let int = |reason| handle.read_int32_blocking(reason, 0).unwrap();
        assert_eq!(
            [
                int(p.array_size_x),
                int(p.array_size_y),
                int(p.array_size),
                int(p.n_dimensions),
                int(p.data_type),
                int(p.epics_ts_sec),
                int(p.epics_ts_nsec),
            ],
            [512, 512, 0, 0, NDDataType::UInt32 as u8 as i32, 0, 0]
        );
        assert_eq!(
            handle.read_float64_blocking(p.timestamp_rbv, 0).unwrap(),
            0.0
        );
    }

    /// merlinDetector.cpp:209/:242: a frame the pool refuses `continue`s past
    /// the software-trigger reset and the completion (:290-305).
    #[test]
    fn a_refused_last_frame_leaves_the_acquisition_running() {
        // 1 MiB of the limit is the flat-field buffer.
        let (acquire, trigger, _) = last_software_triggered_frame("MERLIN_LAST_OK", 2 << 20);
        assert_eq!((acquire, trigger), (0, 0));
        let (acquire, trigger, _) = last_software_triggered_frame("MERLIN_LAST_REFUSED", 1000);
        assert_eq!((acquire, trigger), (1, 1));
    }

    /// merlinDetector.cpp:150-152 counts every data frame before the
    /// NDArray is made.
    #[test]
    fn a_refused_frame_still_advances_the_array_counter() {
        let (_, _, counter) = last_software_triggered_frame("MERLIN_COUNTER_REFUSED", 1000);
        assert_eq!(counter, 1);
    }

    /// ADDriver::writeFloat64 is asynPortDriver's (merlinDetector.cpp:1287), whose
    /// IO_DRIVER line precedes the driver's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "MERLIN_BASE_F64";
        let mut det = detector(port);
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let delay = det.ad.params.shutter_open_delay;
        let mut user = AsynUser::new(delay).with_addr(0);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        let _ = det.write_float64(&mut user, 2.0);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base = format!(
            "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
        );
        let own = format!("merlinDetector:writeFloat64: function={delay}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
    }

    /// C drops setAcquireParams' status (merlinDetector.cpp:1274-1276): a
    /// dead server leaves the write the store's success and the new value.
    #[test]
    fn an_acquire_time_write_keeps_the_stores_status() {
        let mut det = detector("MERLIN_ACQ_TIME");
        let t = det.ad.params.acquire_time;
        det.write_float64(&mut AsynUser::new(t).with_addr(0), 1.5)
            .unwrap();
        assert_eq!(det.ad.port_base.get_float64_param(t, 0).unwrap(), 1.5);
    }

    /// merlinDetector.cpp:1489: MaxSize is merlinDetectorConfig's maxSizeX/Y.
    #[test]
    fn max_size_is_the_configured_size() {
        let det = detector("MERLIN_MAX_SIZE");
        let (p, base) = (det.ad.params, &det.ad.port_base);
        assert_eq!(base.get_int32_param(p.max_size_x, 0).unwrap(), 512);
        assert_eq!(base.get_int32_param(p.max_size_y, 0).unwrap(), 512);
    }

    /// An MPX server that answers each command with a stray frame and then
    /// the frame that echoes it.
    struct MpxServer {
        base: PortDriverBase,
        pending: std::collections::VecDeque<u8>,
        sent: Arc<std::sync::Mutex<Vec<String>>>,
    }

    impl PortDriver for MpxServer {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let request = String::from_utf8_lossy(data);
            self.sent.lock().unwrap().push(request.to_string());
            let mut fields = request.split(',').skip(2);
            let (kind, name) = (fields.next().unwrap(), fields.next().unwrap());
            let echo = match kind {
                "GET" => format!("GET,{name},2.2,0"),
                _ => format!("{kind},{name},0"),
            };
            for body in ["GET,STRAY,1,0".to_string(), echo] {
                let frame = format!("MPX,{:010},{body}", body.len() + 1);
                self.pending.extend(frame.bytes());
            }
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            let n = buf.len().min(self.pending.len());
            for (dst, src) in buf.iter_mut().zip(self.pending.drain(..n)) {
                *dst = src;
            }
            OctetReadResult::ok(n, EomReason::CNT)
        }
    }

    /// C `mpxWrite`/`mpxReadCmd` publish every command and every frame read
    /// back, the constructor's STOPACQUISITION included.
    #[test]
    fn a_command_exchange_lands_on_string_to_and_from_server() {
        let ip = "MERLIN_WIRE_CMD";
        let sent = Arc::new(std::sync::Mutex::new(Vec::new()));
        let server = MpxServer {
            base: PortDriverBase::new(
                ip,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            pending: Default::default(),
            sent: sent.clone(),
        };
        let (runtime, _) =
            create_port_runtime(&claim_port(ip).unwrap(), server, RuntimeConfig::default())
                .unwrap();
        let cmd = MpxConnection::new(
            SyncIOHandle::from_handle(runtime.port_handle().clone(), 0, Duration::ZERO),
            MPX_MAXLINE,
        );
        let mut det = MerlinDetector::new(
            "MERLIN_WIRE",
            cmd,
            512,
            512,
            DetectorType::MerlinQuad,
            1 << 20,
            Arc::new(SharedState::new()),
        )
        .unwrap();
        let wire = |det: &MerlinDetector| {
            let base = &det.ad.port_base;
            let to = base.get_string_param(det.ad.params.string_to_server, 0);
            let from = base.get_string_param(det.ad.params.string_from_server, 0);
            (to.unwrap().to_vec(), from.unwrap().to_vec())
        };
        let frame = |body: &str| format!("MPX,{:010},{body}", body.len() + 1).into_bytes();
        let stop = crate::protocol::encode(MpxKind::Cmd, MPXCMD_STOPACQUISITION, None).unwrap();
        let stop_ack = frame(&format!("CMD,{MPXCMD_STOPACQUISITION},0"));
        assert_eq!(wire(&det), (stop.into_bytes(), stop_ack));

        let version = det
            .cmd
            .get(
                &mut det.ad,
                MPXVAR_GETSOFTWAREVERSION,
                Duration::from_secs(1),
            )
            .unwrap();
        assert_eq!(version, "2.2");
        let get = crate::protocol::encode(MpxKind::Get, MPXVAR_GETSOFTWAREVERSION, None).unwrap();
        let reply = frame(&format!("GET,{MPXVAR_GETSOFTWAREVERSION},2.2,0"));
        assert_eq!(wire(&det), (get.into_bytes(), reply));

        // mpxSet's value is `%f` (merlinDetector.cpp writeFloat64).
        let threshold = det.params.thresholds[0];
        for (v, text) in [(f64::NAN, "nan"), (-f64::NAN, "-nan")] {
            det.write_float64(&mut AsynUser::new(threshold), v).unwrap();
            let want = format!(",SET,{},{text}", MPXVAR_THRESHOLD[0]);
            assert!(
                sent.lock().unwrap().iter().any(|r| r.ends_with(&want)),
                "{want} not in {:?}",
                sent.lock().unwrap()
            );
        }
    }
}
