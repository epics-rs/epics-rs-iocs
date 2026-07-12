//! The Mythen asyn port driver (port of the `mythen` class, mythen.cpp).

use std::sync::Arc;
use std::sync::atomic::Ordering;

use epics_rs::ad_core::driver::{ADDriverBase, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::user::AsynUser;

use crate::detector::{Detector, Settings};
use crate::protocol::CHANNELS_PER_MODULE;

/// The parameters this driver adds to `ADDriver` (C `SD*String`).
#[derive(Debug, Clone, Copy)]
pub struct MythenParams {
    pub setting: usize,
    pub delay_time: usize,
    pub threshold: usize,
    pub energy: usize,
    pub use_flat_field: usize,
    pub use_count_rate: usize,
    pub use_bad_chan_intrpl: usize,
    pub bit_depth: usize,
    pub use_gates: usize,
    pub num_gates: usize,
    pub num_frames: usize,
    pub trigger: usize,
    pub reset: usize,
    pub tau: usize,
    pub nmodules: usize,
    pub firmware_version: usize,
    pub read_mode: usize,
}

impl MythenParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        use ParamType::{Float64, Int32, Octet};
        Ok(Self {
            setting: base.create_param("SD_SETTING", Int32)?,
            delay_time: base.create_param("SD_DELAY_TIME", Float64)?,
            threshold: base.create_param("SD_THRESHOLD", Float64)?,
            energy: base.create_param("SD_ENERGY", Float64)?,
            use_flat_field: base.create_param("SD_USE_FLATFIELD", Int32)?,
            use_count_rate: base.create_param("SD_USE_COUNTRATE", Int32)?,
            use_bad_chan_intrpl: base.create_param("SD_USE_BADCHANNEL_INTRPL", Int32)?,
            bit_depth: base.create_param("SD_BIT_DEPTH", Int32)?,
            use_gates: base.create_param("SD_USE_GATES", Int32)?,
            num_gates: base.create_param("SD_NUM_GATES", Int32)?,
            num_frames: base.create_param("SD_NUM_FRAMES", Int32)?,
            trigger: base.create_param("SD_TRIGGER", Int32)?,
            reset: base.create_param("SD_RESET", Int32)?,
            tau: base.create_param("SD_TAU", Float64)?,
            nmodules: base.create_param("SD_NMODULES", Int32)?,
            firmware_version: base.create_param("SD_FIRMWARE_VERSION", Octet)?,
            read_mode: base.create_param("SD_READ_MODE", Int32)?,
        })
    }
}

pub struct MythenDriver {
    pub ad: ADDriverBase,
    pub p: MythenParams,
    pub det: Arc<Detector>,
    /// Tells the acquisition task that an acquisition has started
    /// (C `startEventId_`).
    start_tx: rt::CommandSender<()>,
}

impl MythenDriver {
    pub fn new(
        port_name: &str,
        det: Arc<Detector>,
        max_memory: usize,
        start_tx: rt::CommandSender<()>,
    ) -> AsynResult<Self> {
        let sensor_size_x = CHANNELS_PER_MODULE as i32;
        let sensor_size_y = 1;
        let mut ad = ADDriverBase::new(port_name, sensor_size_x, sensor_size_y, max_memory)?;
        let p = MythenParams::create(&mut ad.port_base)?;

        let mut driver = Self {
            ad,
            p,
            det,
            start_tx,
        };
        driver.init_params(sensor_size_x, sensor_size_y)?;
        Ok(driver)
    }

    /// C's constructor body, mythen.cpp:1326-1377.
    fn init_params(&mut self, sensor_size_x: i32, sensor_size_y: i32) -> AsynResult<()> {
        let firmware = self.det.get_firmware()?;
        log::info!("mythen: firmware {firmware}");

        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.base.manufacturer, 0, "Dectris".into())?;
        base.set_string_param(self.ad.params.base.model, 0, "Mythen".into())?;
        base.set_string_param(self.p.firmware_version, 0, firmware)?;

        base.set_int32_param(self.ad.params.max_size_x, 0, sensor_size_x)?;
        base.set_int32_param(self.ad.params.max_size_y, 0, sensor_size_y)?;
        base.set_int32_param(self.ad.params.min_x, 0, 1)?;
        base.set_int32_param(self.ad.params.min_y, 0, 1)?;
        base.set_int32_param(self.ad.params.size_x, 0, sensor_size_x)?;
        base.set_int32_param(self.ad.params.size_y, 0, sensor_size_y)?;
        base.set_int32_param(self.ad.params.base.array_size, 0, 0)?;
        base.set_int32_param(self.ad.params.base.data_type, 0, NDDataType::UInt32 as i32)?;
        base.set_int32_param(self.ad.params.image_mode, 0, ImageMode::Single as i32)?;

        let status = self.det.get_status()?;
        self.ad
            .port_base
            .set_int32_param(self.ad.params.status, 0, status as i32)?;

        let settings = self.det.get_settings()?;
        self.apply_settings(&settings)?;

        let nmodules = self.det.read_nmodules()?;
        if nmodules < 0 {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("mythen: [-get nmodules] unexpected reply: {nmodules}"),
            });
        }
        self.ad
            .port_base
            .set_int32_param(self.p.nmodules, 0, nmodules)?;
        // One module is 1280 channels; the array the driver publishes is as
        // wide as all the modules together.
        self.ad.port_base.set_int32_param(
            self.ad.params.base.array_size_x,
            0,
            nmodules * CHANNELS_PER_MODULE as i32,
        )?;

        self.ad.port_base.call_param_callbacks(0)
    }

    /// Push a `getSettings` result into the parameter library
    /// (C `getSettings`'s `setIntegerParam`/`setDoubleParam` calls).
    fn apply_settings(&mut self, s: &Settings) -> AsynResult<()> {
        let base = &mut self.ad.port_base;
        base.set_int32_param(self.p.use_flat_field, 0, s.use_flat_field)?;
        base.set_int32_param(self.p.use_bad_chan_intrpl, 0, s.use_bad_chan_intrpl)?;
        base.set_int32_param(self.p.use_count_rate, 0, s.use_count_rate)?;
        // C stores the *bit count* here, which is what BitDepth_RBV (a longin)
        // displays; the BitDepth mbbo writes the menu index into the same
        // parameter.
        base.set_int32_param(self.p.bit_depth, 0, s.nbits)?;
        base.set_float64_param(self.ad.params.acquire_time, 0, s.acquire_time)?;
        if let Some(frames) = s.frames {
            base.set_int32_param(self.p.num_frames, 0, frames)?;
        }
        base.set_float64_param(self.p.tau, 0, s.tau)?;
        base.set_float64_param(self.p.threshold, 0, s.threshold)?;
        if let Some(energy) = s.energy {
            base.set_float64_param(self.p.energy, 0, energy)?;
        }
        if let Some(delay) = s.delay_time {
            base.set_float64_param(self.p.delay_time, 0, delay)?;
        }
        if let Some(trigger) = s.trigger {
            base.set_int32_param(self.p.trigger, 0, trigger)?;
        }
        Ok(())
    }

    /// C `getSettings` as called at the end of every write.
    fn refresh_settings(&mut self) -> AsynResult<()> {
        if self.det.state.acquiring.load(Ordering::Acquire) {
            // C refuses to talk to a running detector (mythen.cpp:689).
            return Ok(());
        }
        let settings = self.det.get_settings()?;
        self.apply_settings(&settings)
    }

    fn int_param(&self, index: usize) -> i32 {
        self.ad.port_base.get_int32_param(index, 0).unwrap_or(0)
    }

    /// C `setAcquire`, mythen.cpp:268.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        if value == 0 {
            self.det.stop()?;
            let status = self.det.get_status()?;
            self.ad
                .port_base
                .set_int32_param(self.ad.params.status, 0, status as i32)?;
            return Ok(());
        }
        if self.det.start()? && self.start_tx.try_send(()).is_err() {
            log::error!("mythen: the acquisition task is not running");
        }
        Ok(())
    }
}

impl PortDriver for MythenDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let acquire = self.ad.params.acquire;

        // C refuses every write except ADAcquire while an acquisition runs
        // (mythen.cpp:1111).
        if reason != acquire && self.det.state.acquiring.load(Ordering::Acquire) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "mythen: detector is busy".into(),
            });
        }

        self.ad.port_base.params.set_int32(reason, addr, value)?;

        if reason == acquire {
            let read_mode = self.int_param(self.p.read_mode);
            self.det.state.read_mode.store(read_mode, Ordering::Release);
            self.set_acquire(value)?;
        } else {
            let image_mode = self.int_param(self.ad.params.image_mode);
            if reason == self.p.setting {
                self.det.load_settings(value)?;
            } else if reason == self.p.use_flat_field {
                self.det.set_flat_field_correction(value)?;
            } else if reason == self.p.use_count_rate {
                self.det.set_rate_correction(value)?;
            } else if reason == self.p.use_bad_chan_intrpl {
                self.det.set_bad_chan_intrpl(value)?;
            } else if reason == self.p.bit_depth {
                self.det.set_bit_depth(value)?;
            } else if reason == self.p.num_gates {
                self.det.set_num_gates(value)?;
            } else if reason == self.p.use_gates {
                self.det.set_use_gates(value)?;
            } else if reason == self.p.num_frames {
                self.det.set_frames(value, image_mode)?;
            } else if reason == self.p.trigger {
                self.det.set_trigger(value)?;
            } else if reason == self.p.reset {
                // C flips SD_RESET on for the duration of the reset
                // (mythen.cpp:665-673).
                self.ad.port_base.set_int32_param(self.p.reset, 0, 1)?;
                self.ad.port_base.call_param_callbacks(0)?;
                self.det.reset()?;
                self.ad.port_base.set_int32_param(self.p.reset, 0, 0)?;
            } else if reason == self.ad.params.image_mode {
                // The frame count the detector needs depends on the image mode.
                let frames = self.det.state.frames.load(Ordering::Acquire);
                self.det.set_frames(frames, value)?;
            }
            self.refresh_settings()?;
        }

        self.ad.port_base.call_param_callbacks(addr)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;

        if self.det.state.acquiring.load(Ordering::Acquire) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "mythen: detector is busy".into(),
            });
        }

        self.ad.port_base.params.set_float64(reason, addr, value)?;

        if reason == self.ad.params.acquire_time {
            self.det.set_exposure_time(value)?;
        } else if reason == self.p.delay_time {
            self.det.set_delay_after_trigger(value)?;
        } else if reason == self.p.threshold {
            self.det.set_kthresh(value)?;
        } else if reason == self.p.energy {
            self.det.set_energy(value)?;
        } else if reason == self.p.tau {
            // C zeroes the parameter and errors out on a value that is neither
            // -1 nor positive (mythen.cpp:361-368).
            if !self.det.set_tau(value)? {
                self.ad.port_base.set_float64_param(self.p.tau, 0, 0.0)?;
                self.ad.port_base.call_param_callbacks(addr)?;
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!("mythen: tau must be -1 or > 0, got {value}"),
                });
            }
        }

        self.refresh_settings()?;
        self.ad.port_base.call_param_callbacks(addr)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let addr = user.addr;
        let value = String::from_utf8_lossy(data).into_owned();
        self.ad.port_base.params.set_string(reason, addr, value)?;
        self.ad.port_base.call_param_callbacks(addr)?;
        Ok(data.len())
    }
}
